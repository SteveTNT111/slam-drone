"""
Offline ground station server for 2026 NUEDC D problem.

It serves the dashboard locally and bridges UDP telemetry packets to the
browser through Server-Sent Events. It uses only the Python standard library.

Run:
  python ground_station_server.py

Open:
  http://127.0.0.1:8080
"""

from __future__ import annotations

import json
import mimetypes
import os
import queue
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parent
HOST = "0.0.0.0"
HTTP_PORT = 8080
CAR_PORT = 8893
DRONE_PORT = 8892
STATUS_PORT = 8894

CAR_MAGIC = 0xCA26
DRONE_MAGIC = 0xDA26

CAR_STRUCT = struct.Struct("<HBBHHIfffff")
DRONE_STRUCT = struct.Struct("<HBBHHIfffff")

clients: set[queue.Queue[str]] = set()
clients_lock = threading.Lock()


def broadcast(message: dict) -> None:
    payload = json.dumps(message, ensure_ascii=False)
    with clients_lock:
        stale = []
        for client in clients:
            try:
                client.put_nowait(payload)
            except queue.Full:
                stale.append(client)
        for client in stale:
            clients.discard(client)


def parse_json_packet(data: bytes) -> dict | None:
    try:
        obj = json.loads(data.decode("utf-8"))
    except Exception:
        return None
    if isinstance(obj, dict) and obj.get("kind") in {"car", "drone", "status"}:
        return obj
    return None


def parse_car_packet(data: bytes) -> dict | None:
    if len(data) != CAR_STRUCT.size:
        return parse_json_packet(data)
    magic, pkt_type, phase, seq, _reserved, stamp, x, y, yaw, speed, progress = CAR_STRUCT.unpack(data)
    if magic != CAR_MAGIC:
        return parse_json_packet(data)
    return {
        "kind": "car",
        "type": pkt_type,
        "phase": phase,
        "seq": seq,
        "time_ms": stamp,
        "x_cm": x,
        "y_cm": y,
        "yaw_deg": yaw,
        "speed_cm_s": speed,
        "progress_cm": progress,
    }


def parse_drone_packet(data: bytes) -> dict | None:
    if len(data) != DRONE_STRUCT.size:
        return parse_json_packet(data)
    magic, pkt_type, phase, seq, _reserved, stamp, x, y, height, target_error, battery = DRONE_STRUCT.unpack(data)
    if magic != DRONE_MAGIC:
        return parse_json_packet(data)
    return {
        "kind": "drone",
        "type": pkt_type,
        "phase": phase,
        "seq": seq,
        "time_ms": stamp,
        "x_cm": x,
        "y_cm": y,
        "height_cm": height,
        "target_error_cm": target_error,
        "battery_v": battery,
    }


def udp_loop(port: int, parser, name: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    print(f"[udp] listening {name} on 0.0.0.0:{port}")
    while True:
        data, addr = sock.recvfrom(2048)
        msg = parser(data)
        if msg:
            msg["source"] = f"{addr[0]}:{addr[1]}"
            msg["received_at_ms"] = int(time.time() * 1000)
            broadcast(msg)


def parse_status_packet(data: bytes) -> dict | None:
    text = data.decode("utf-8", errors="replace").strip()
    if not text:
        return None
    return {"kind": "status", "text": text}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:
        if self.path == "/events":
            self.handle_events()
            return
        path = self.path.split("?", 1)[0].lstrip("/") or "index.html"
        target = (ROOT / path).resolve()
        if not str(target).startswith(str(ROOT)) or not target.exists() or target.is_dir():
            self.send_error(404)
            return
        content = target.read_bytes()
        ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def handle_events(self) -> None:
        q: queue.Queue[str] = queue.Queue(maxsize=100)
        with clients_lock:
            clients.add(q)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()
            while True:
                try:
                    payload = q.get(timeout=10)
                    frame = f"data: {payload}\n\n".encode("utf-8")
                except queue.Empty:
                    frame = b": keepalive\n\n"
                self.wfile.write(frame)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            with clients_lock:
                clients.discard(q)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[http] {self.address_string()} {fmt % args}")


def main() -> None:
    os.chdir(ROOT)
    threading.Thread(target=udp_loop, args=(CAR_PORT, parse_car_packet, "car"), daemon=True).start()
    threading.Thread(target=udp_loop, args=(DRONE_PORT, parse_drone_packet, "drone"), daemon=True).start()
    threading.Thread(target=udp_loop, args=(STATUS_PORT, parse_status_packet, "status"), daemon=True).start()
    server = ThreadingHTTPServer((HOST, HTTP_PORT), Handler)
    print(f"[http] open http://127.0.0.1:{HTTP_PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
