#!/usr/bin/env python3

"""ROS 1 line-oriented USB serial bridge for the onboard ESP32."""

import sys
import re
import termios
import threading
import time

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "Missing pyserial. Install the ROS package dependency with: "
        "sudo apt install python3-serial"
    ) from exc


SERIAL_IO_ERRORS = (serial.SerialException, OSError, termios.error)
TELEMETRY_SEQUENCE_MODULO = 1 << 32
ESPNOW_RX_PATTERN = re.compile(
    r"^ESPNOW_RX mac=([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}) data=(.*)$"
)


class Esp32SerialBridge:
    """Reconnectable, text-line serial bridge with ROS topic interfaces."""

    def __init__(self):
        self.port = rospy.get_param(
            "~port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        )
        self.baud = int(rospy.get_param("~baud", 115200))
        self.reconnect_period = max(
            0.1, float(rospy.get_param("~reconnect_period", 2.0))
        )
        self.encoding = rospy.get_param("~encoding", "utf-8")
        self.serial_health_timeout = max(
            0.0, float(rospy.get_param("~serial_health_timeout", 8.0))
        )
        self.serial_reopen_on_silence = bool(
            rospy.get_param("~serial_reopen_on_silence", True)
        )
        self.interactive = bool(rospy.get_param("~interactive", True))
        self.telemetry_enabled = bool(
            rospy.get_param("~telemetry_enabled", True)
        )
        self.telemetry_rate = max(
            0.1, float(rospy.get_param("~telemetry_rate", 1.0))
        )
        self.telemetry_timeout = max(
            0.1, float(rospy.get_param("~telemetry_timeout", 3.0))
        )
        self.telemetry_prefix = str(
            rospy.get_param("~telemetry_prefix", "DRONE_STATUS")
        ).strip()
        self.telemetry_precision = min(
            6, max(0, int(rospy.get_param("~telemetry_precision", 3)))
        )
        self._telemetry_sequence = int(
            rospy.get_param("~telemetry_sequence_start", 1)
        ) % TELEMETRY_SEQUENCE_MODULO
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.slam_odom_topic = rospy.get_param(
            "~slam_odom_topic", "/Odometry"
        )
        self.command_topic = str(
            rospy.get_param("~command_topic", "/esp32/cmd")
        ).strip()
        self.command_prefix = str(
            rospy.get_param("~command_prefix", "CMD_drone")
        )
        configured_macs = rospy.get_param("~command_allowed_macs", [])
        if isinstance(configured_macs, str):
            configured_macs = configured_macs.split(",")
        self.command_allowed_macs = {
            str(mac).strip().upper()
            for mac in configured_macs
            if str(mac).strip()
        }

        self._serial = None
        self._serial_lock = threading.RLock()
        self._rx_buffer = bytearray()
        self._next_connect_time = 0.0
        self._unanswered_tx_since = None
        self._telemetry_lock = threading.Lock()
        self._mavros_state = None
        self._mavros_state_time = rospy.Time(0)
        self._slam_odom = None
        self._slam_odom_time = rospy.Time(0)

        self.rx_pub = rospy.Publisher("/esp32/rx", String, queue_size=100)
        self.command_pub = rospy.Publisher(
            self.command_topic, String, queue_size=20
        )
        self.tx_sub = rospy.Subscriber(
            "/esp32/tx", String, self._tx_callback, queue_size=100
        )

        if self.telemetry_enabled:
            self.state_sub = rospy.Subscriber(
                self.state_topic,
                State,
                self._state_callback,
                queue_size=10,
            )
            self.slam_odom_sub = rospy.Subscriber(
                self.slam_odom_topic,
                Odometry,
                self._slam_odom_callback,
                queue_size=10,
            )
            self.telemetry_timer = rospy.Timer(
                rospy.Duration(1.0 / self.telemetry_rate),
                self._telemetry_callback,
            )

        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "ESP32 serial bridge configured: port=%s baud=%d encoding=%s",
            self.port,
            self.baud,
            self.encoding,
        )
        rospy.loginfo(
            "ESP-NOW command filter: topic=%s prefix=%s allowed_macs=%s",
            self.command_topic,
            self.command_prefix,
            ",".join(sorted(self.command_allowed_macs))
            if self.command_allowed_macs
            else "ANY",
        )
        if self.serial_reopen_on_silence and self.serial_health_timeout > 0.0:
            rospy.loginfo(
                "ESP32 serial health monitor enabled: reopen after %.1f s "
                "without any RX following TX",
                self.serial_health_timeout,
            )
        if self.telemetry_enabled:
            rospy.loginfo(
                "ESP32 telemetry enabled: %.2f Hz, MAVROS=%s, SLAM=%s",
                self.telemetry_rate,
                self.state_topic,
                self.slam_odom_topic,
            )
        if self.interactive and sys.stdin.isatty():
            self._console_thread = threading.Thread(
                target=self._console_loop, name="esp32_terminal_input", daemon=True
            )
            self._console_thread.start()
            rospy.loginfo(
                "Interactive TX enabled: type a text line and press Enter."
            )
        elif self.interactive:
            rospy.loginfo(
                "Interactive TX requested but stdin is not a terminal; use /esp32/tx."
            )

    def _connect(self):
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                return True

            try:
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.2,
                    write_timeout=1.0,
                    rtscts=False,
                    dsrdtr=False,
                )
                self._rx_buffer.clear()
                self._unanswered_tx_since = None
                rospy.loginfo(
                    "ESP32 serial connected: %s at %d baud", self.port, self.baud
                )
                return True
            except SERIAL_IO_ERRORS as exc:
                self._serial = None
                self._next_connect_time = time.monotonic() + self.reconnect_period
                rospy.logerr(
                    "ESP32 serial not connected (%s): %s. Retrying in %.1f s.",
                    self.port,
                    exc,
                    self.reconnect_period,
                )
                return False

    def _disconnect(self, reason):
        with self._serial_lock:
            handle = self._serial
            self._serial = None
            self._rx_buffer.clear()
            self._unanswered_tx_since = None
            self._next_connect_time = time.monotonic() + self.reconnect_period
            if handle is not None:
                try:
                    if handle.is_open:
                        handle.close()
                except SERIAL_IO_ERRORS:
                    pass
        rospy.logerr(
            "ESP32 serial disconnected (%s): %s. Retrying in %.1f s.",
            self.port,
            reason,
            self.reconnect_period,
        )

    def _read_available(self):
        with self._serial_lock:
            handle = self._serial
            if handle is None or not handle.is_open:
                return
            try:
                chunk = handle.read_until(b"\n")
            except SERIAL_IO_ERRORS as exc:
                self._disconnect(exc)
                return

        if not chunk:
            return

        # The ESP32 firmware prints UART_RX for received commands and prints
        # ESP-NOW send results. Any byte proves that its UART/main loop is alive.
        with self._serial_lock:
            self._unanswered_tx_since = None
        self._rx_buffer.extend(chunk)
        while b"\n" in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            if raw_line.endswith(b"\r"):
                raw_line = raw_line[:-1]
            line = raw_line.decode(self.encoding, errors="replace")
            rospy.loginfo("[ESP32 RX] %s", line)
            self.rx_pub.publish(String(data=line))
            self._publish_filtered_command(line)

    @staticmethod
    def _parse_espnow_rx_line(line):
        match = ESPNOW_RX_PATTERN.fullmatch(line)
        if match is None:
            return None
        return match.group(1).upper(), match.group(2)

    def _publish_filtered_command(self, line):
        parsed = self._parse_espnow_rx_line(line)
        if parsed is None:
            return

        source_mac, payload = parsed
        if not payload.startswith(self.command_prefix):
            return
        if (
            self.command_allowed_macs
            and source_mac not in self.command_allowed_macs
        ):
            rospy.logwarn_throttle(
                2.0,
                "Rejected CMD_drone message from non-allowed ESP-NOW MAC %s",
                source_mac,
            )
            return

        rospy.loginfo("[ESP32 CMD] mac=%s data=%s", source_mac, payload)
        self.command_pub.publish(String(data=payload))

    def _send_line(
        self,
        text,
        source,
        warn_if_disconnected=True,
        log_success=True,
    ):
        line = text.rstrip("\r\n")
        try:
            payload = (line + "\n").encode(self.encoding)
        except (LookupError, UnicodeEncodeError) as exc:
            rospy.logerr("Cannot encode ESP32 TX from %s: %s", source, exc)
            return False

        with self._serial_lock:
            handle = self._serial
            if handle is None or not handle.is_open:
                if warn_if_disconnected:
                    rospy.logwarn(
                        "ESP32 serial is not connected; dropped TX from %s: %s",
                        source,
                        line,
                    )
                return False
            try:
                handle.write(payload)
                handle.flush()
                if self._unanswered_tx_since is None:
                    self._unanswered_tx_since = time.monotonic()
            except SERIAL_IO_ERRORS as exc:
                self._disconnect(exc)
                return False

        if log_success:
            rospy.loginfo("[ESP32 TX] %s", line)
        return True

    def _check_serial_health(self):
        if (
            not self.serial_reopen_on_silence
            or self.serial_health_timeout <= 0.0
        ):
            return

        with self._serial_lock:
            handle = self._serial
            unanswered_since = self._unanswered_tx_since
            if handle is None or not handle.is_open or unanswered_since is None:
                return
            silent_for = time.monotonic() - unanswered_since

        if silent_for >= self.serial_health_timeout:
            self._disconnect(
                "sent data but received no ESP32 bytes for %.1f s; "
                "forcing serial reopen" % silent_for
            )

    def _tx_callback(self, msg):
        self._send_line(msg.data, "/esp32/tx")

    def _state_callback(self, msg):
        with self._telemetry_lock:
            self._mavros_state = msg
            self._mavros_state_time = rospy.Time.now()

    def _slam_odom_callback(self, msg):
        with self._telemetry_lock:
            self._slam_odom = msg
            self._slam_odom_time = rospy.Time.now()

    @staticmethod
    def _safe_token(value, default):
        token = str(value).strip()
        if not token:
            return default
        return "_".join(token.replace("=", "_").split())

    def _telemetry_callback(self, _event):
        now = rospy.Time.now()
        with self._telemetry_lock:
            state = self._mavros_state
            state_time = self._mavros_state_time
            odom = self._slam_odom
            odom_time = self._slam_odom_time
            sequence = self._telemetry_sequence
            self._telemetry_sequence = (
                self._telemetry_sequence + 1
            ) % TELEMETRY_SEQUENCE_MODULO

        state_valid = (
            state is not None
            and (now - state_time).to_sec() <= self.telemetry_timeout
        )
        slam_valid = (
            odom is not None
            and (now - odom_time).to_sec() <= self.telemetry_timeout
        )

        connected = bool(state.connected) if state_valid else False
        armed = bool(state.armed) if state_valid else False
        mode = self._safe_token(state.mode, "UNKNOWN") if state_valid else "UNKNOWN"

        if slam_valid:
            position = odom.pose.pose.position
            frame = self._safe_token(odom.header.frame_id, "camera_init")
            number_format = "{:.%df}" % self.telemetry_precision
            x_text = number_format.format(position.x)
            y_text = number_format.format(position.y)
            z_text = number_format.format(position.z)
        else:
            frame = "UNKNOWN"
            x_text = "nan"
            y_text = "nan"
            z_text = "nan"

        line = (
            "{prefix} seq={sequence} mavros={mavros} connected={connected} "
            "armed={armed} "
            "mode={mode} slam={slam} frame={frame} x={x} y={y} z={z}"
        ).format(
            prefix=self._safe_token(self.telemetry_prefix, "DRONE_STATUS"),
            sequence=sequence,
            mavros=int(state_valid),
            connected=int(connected),
            armed=int(armed),
            mode=mode,
            slam=int(slam_valid),
            frame=frame,
            x=x_text,
            y=y_text,
            z=z_text,
        )

        if len(line.encode(self.encoding, errors="replace")) > 200:
            rospy.logerr_throttle(
                5.0,
                "ESP32 telemetry line exceeds firmware limit of 200 bytes",
            )
            return

        if self._send_line(
            line,
            "telemetry",
            warn_if_disconnected=False,
            log_success=False,
        ):
            rospy.loginfo_throttle(5.0, "[ESP32 TELEMETRY TX] %s", line)

    def _console_loop(self):
        while not rospy.is_shutdown():
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError) as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn("Interactive terminal input stopped: %s", exc)
                return
            if line == "":
                return
            text = line.rstrip("\r\n")
            if not text:
                continue
            self._send_line(text, "terminal")

    def run(self):
        while not rospy.is_shutdown():
            with self._serial_lock:
                connected = self._serial is not None and self._serial.is_open

            if not connected:
                wait_time = self._next_connect_time - time.monotonic()
                if wait_time > 0.0:
                    rospy.rostime.wallsleep(min(wait_time, 0.2))
                    continue
                if not self._connect():
                    rospy.rostime.wallsleep(min(self.reconnect_period, 0.2))
                continue
            self._read_available()
            self._check_serial_health()

    def close(self):
        with self._serial_lock:
            handle = self._serial
            self._serial = None
            if handle is not None:
                try:
                    if handle.is_open:
                        handle.close()
                    rospy.loginfo("ESP32 serial port closed")
                except SERIAL_IO_ERRORS as exc:
                    rospy.logwarn("Error while closing ESP32 serial port: %s", exc)


def main():
    rospy.init_node("esp32_serial")
    bridge = Esp32SerialBridge()
    bridge.run()


if __name__ == "__main__":
    main()
