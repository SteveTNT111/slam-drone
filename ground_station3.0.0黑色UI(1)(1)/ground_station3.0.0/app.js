const FIELD_W = 400;
const FIELD_H = 500;
const HOME = { x: 112.5, y: 112.5 };
const TRACK = {
  leftX: 150,
  rightX: 300,
  bottomY: 200,
  topY: 350,
  radius: 75,
  startS: 0,
  ab: 150,
  arc: Math.PI * 75,
};
TRACK.bcStart = TRACK.ab;
TRACK.cdStart = TRACK.ab + TRACK.arc;
TRACK.daStart = TRACK.cdStart + 150;
TRACK.total = TRACK.daStart + TRACK.arc;

const phaseCodes = {
  0: "IDLE",
  1: "START_SENT",
  2: "TAKEOFF",
  3: "HOVER_150",
  4: "SEARCH_CAR",
  5: "APPROACH_CAR",
  6: "FOLLOW",
  7: "DROP",
  8: "RETURN_HOME",
  9: "LAND_HOME",
  10: "LAND_ON_CAR",
  11: "WAIT_ON_CAR",
  12: "TAKEOFF_FROM_CAR",
  13: "DONE",
  250: "FAILSAFE",
  255: "UNKNOWN",
};

const phaseNames = {
  IDLE: "待命",
  START_SENT: "启动已发送",
  TAKEOFF: "起飞",
  HOVER_150: "150厘米悬停",
  SEARCH_CAR: "搜索小车",
  APPROACH_CAR: "接近小车",
  FOLLOW: "伴飞",
  DROP: "抛投",
  RETURN_HOME: "返航",
  LAND_HOME: "降落至起降点",
  LAND_ON_CAR: "动态降落",
  WAIT_ON_CAR: "平台停留",
  TAKEOFF_FROM_CAR: "平台起飞",
  DONE: "完成",
  FAILSAFE: "保护模式",
  UNKNOWN: "未知",
};

const carPhaseNames = {
  0: "待命",
  1: "启动",
  2: "前进",
  3: "经过B点",
  4: "经过C点",
  5: "经过D点",
  13: "停止",
  250: "保护停车",
  255: "未知",
};

const state = {
  car: null,
  drone: null,
  nodes: {
    ground: { seenAt: 0, detail: "未连接串口" },
    car: { seenAt: 0, detail: "等待在线包" },
    drone: { seenAt: 0, detail: "等待在线包" },
  },
  carPath: [],
  dronePath: [],
  simRunning: false,
  taskMode: 1,
  simStart: 0,
  timerStart: null,
  timerRunning: false,
  serialConnected: false,
  lastServerEventAt: 0,
  canvasScale: 1,
  mapRect: null,
};

const els = {
  canvas: document.getElementById("fieldCanvas"),
  tooltip: document.getElementById("mapTooltip"),
  timer: document.getElementById("missionTimer"),
  linkBadge: document.getElementById("linkBadge"),
  dronePhase: document.getElementById("dronePhase"),
  carPhase: document.getElementById("carPhase"),
  missionStatus: document.getElementById("missionStatus"),
  droneX: document.getElementById("droneX"),
  droneY: document.getElementById("droneY"),
  droneH: document.getElementById("droneH"),
  targetErr: document.getElementById("targetErr"),
  droneHorizontalSpeed: document.getElementById("droneHorizontalSpeed"),
  droneVerticalSpeed: document.getElementById("droneVerticalSpeed"),
  batteryV: document.getElementById("batteryV"),
  droneAge: document.getElementById("droneAge"),
  carX: document.getElementById("carX"),
  carY: document.getElementById("carY"),
  carYaw: document.getElementById("carYaw"),
  carSpeed: document.getElementById("carSpeed"),
  carProgress: document.getElementById("carProgress"),
  carAge: document.getElementById("carAge"),
  groundNodeStatus: document.getElementById("groundNodeStatus"),
  carNodeStatus: document.getElementById("carNodeStatus"),
  droneNodeStatus: document.getElementById("droneNodeStatus"),
  timeline: document.getElementById("timeline"),
  simBtn: document.getElementById("simBtn"),
  pauseSimBtn: document.getElementById("pauseSimBtn"),
  task1Btn: document.getElementById("task1Btn"),
  task2Btn: document.getElementById("task2Btn"),
  serialBtn: document.getElementById("serialBtn"),
  disconnectSerialBtn: document.getElementById("disconnectSerialBtn"),
  serialStatus: document.getElementById("serialStatus"),
  serialLog: document.getElementById("serialLog"),
  startTimerBtn: document.getElementById("startTimerBtn"),
  resetBtn: document.getElementById("resetBtn"),
};

let serialPort = null;
let serialReader = null;
let serialKeepReading = false;
let serialLogFollowLatest = true;

const ctx = els.canvas.getContext("2d");
const fieldImage = new Image();
fieldImage.src = "./assets/field_map.png";
fieldImage.onload = draw;

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function fmt(v, unit, digits = 1) {
  if (v === null || v === undefined || Number.isNaN(v)) return `-- ${unit}`;
  return `${v.toFixed(digits)} ${unit}`;
}

function phaseName(phase) {
  const code = phaseCodes[phase] || "UNKNOWN";
  return phaseNames[code] || "未知";
}

function phaseCode(phase) {
  return phaseCodes[phase] || "UNKNOWN";
}

function carPhaseName(phase) {
  return carPhaseNames[phase] || "前进";
}

function carPoseFromProgress(progress) {
  const s = ((progress % TRACK.total) + TRACK.total) % TRACK.total;
  if (s < TRACK.ab) {
    return {
      x: TRACK.leftX,
      y: TRACK.bottomY + s,
      yaw: 0,
      progress: s,
    };
  }
  if (s < TRACK.cdStart) {
    const u = (s - TRACK.bcStart) / TRACK.radius;
    const theta = Math.PI - u;
    return {
      x: 225 + TRACK.radius * Math.cos(theta),
      y: TRACK.topY + TRACK.radius * Math.sin(theta),
      yaw: (u * 180) / Math.PI,
      progress: s,
    };
  }
  if (s < TRACK.daStart) {
    const d = s - TRACK.cdStart;
    return {
      x: TRACK.rightX,
      y: TRACK.topY - d,
      yaw: 180,
      progress: s,
    };
  }
  const u = (s - TRACK.daStart) / TRACK.radius;
  const theta = -u;
  return {
    x: 225 + TRACK.radius * Math.cos(theta),
    y: TRACK.bottomY + TRACK.radius * Math.sin(theta),
    yaw: (180 + (u * 180) / Math.PI) % 360,
    progress: s,
  };
}

function mapToCanvas(x, y) {
  const rect = state.mapRect;
  return {
    x: rect.x + (x / FIELD_W) * rect.w,
    y: rect.y + ((FIELD_H - y) / FIELD_H) * rect.h,
  };
}

function canvasToMap(x, y) {
  const rect = state.mapRect;
  return {
    x: ((x - rect.x) / rect.w) * FIELD_W,
    y: FIELD_H - ((y - rect.y) / rect.h) * FIELD_H,
  };
}

function resizeCanvas() {
  const box = els.canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(640, Math.floor(box.width * dpr));
  const height = Math.max(620, Math.floor(box.height * dpr));
  if (els.canvas.width !== width || els.canvas.height !== height) {
    els.canvas.width = width;
    els.canvas.height = height;
  }
  state.canvasScale = dpr;
}

function draw() {
  resizeCanvas();
  const w = els.canvas.width;
  const h = els.canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#fdfdf9";
  ctx.fillRect(0, 0, w, h);

  const pad = (3 * state.canvasScale) / 2;
  const usableW = w - pad * 2;
  const usableH = h - pad * 2;
  const scale = Math.min(usableW / FIELD_W, usableH / FIELD_H);
  const mapW = FIELD_W * scale;
  const mapH = FIELD_H * scale;
  state.mapRect = {
    x: (w - mapW) / 2,
    y: (h - mapH) / 2,
    w: mapW,
    h: mapH,
  };

  drawField();
  drawHistory(state.carPath, "#e11d48", 2.5);
  drawHistory(state.dronePath, "#2563eb", 2);
  if (state.car) drawCar(state.car);
  if (state.drone) drawDrone(state.drone);
}

function drawField() {
  const r = state.mapRect;
  ctx.save();
  ctx.fillStyle = "#ffffff";
  ctx.strokeStyle = "#111";
  ctx.lineWidth = 3 * state.canvasScale;
  ctx.strokeRect(r.x, r.y, r.w, r.h);

  if (fieldImage.complete) {
    ctx.drawImage(fieldImage, r.x, r.y, r.w, r.h);
  } else {
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#111";
    ctx.lineWidth = 4 * state.canvasScale;
    ctx.strokeRect(r.x, r.y, r.w, r.h);
    drawTrack();
    drawPoint("H", HOME.x, HOME.y, "#111");
    drawPoint("A", 150, 200, "#111");
    drawPoint("B", 150, 350, "#111");
    drawPoint("C", 300, 350, "#111");
    drawPoint("D", 300, 200, "#111");
  }
  ctx.restore();
}

function drawTrack() {
  ctx.save();
  ctx.strokeStyle = "#111";
  ctx.lineWidth = 3 * state.canvasScale;
  ctx.lineCap = "round";
  ctx.beginPath();
  const pA = mapToCanvas(150, 200);
  const pB = mapToCanvas(150, 350);
  const pC = mapToCanvas(300, 350);
  const pD = mapToCanvas(300, 200);
  const topC = mapToCanvas(225, 350);
  const bottomC = mapToCanvas(225, 200);
  const rx = (75 / FIELD_W) * state.mapRect.w;
  const ry = (75 / FIELD_H) * state.mapRect.h;
  ctx.moveTo(pA.x, pA.y);
  ctx.lineTo(pB.x, pB.y);
  drawEllipseArc(topC.x, topC.y, rx, ry, Math.PI, 0, true);
  ctx.lineTo(pD.x, pD.y);
  drawEllipseArc(bottomC.x, bottomC.y, rx, ry, 0, Math.PI, true);
  ctx.stroke();

  ctx.strokeStyle = "#111";
  ctx.lineWidth = 2 * state.canvasScale;
  ctx.beginPath();
  const startL = mapToCanvas(130, 200);
  const startR = mapToCanvas(170, 200);
  ctx.moveTo(startL.x, startL.y);
  ctx.lineTo(startR.x, startR.y);
  ctx.stroke();
  ctx.restore();
}

function drawEllipseArc(cx, cy, rx, ry, start, end, ccw) {
  ctx.save();
  ctx.translate(cx, cy);
  ctx.scale(rx, ry);
  ctx.arc(0, 0, 1, start, end, ccw);
  ctx.restore();
}

function drawPoint(label, x, y, color) {
  const p = mapToCanvas(x, y);
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 5 * state.canvasScale, 0, Math.PI * 2);
  ctx.fill();
  ctx.font = `${18 * state.canvasScale}px Microsoft YaHei, Arial`;
  ctx.fillText(label, p.x + 9 * state.canvasScale, p.y + 5 * state.canvasScale);
  ctx.restore();
}

function drawHistory(points, color, width) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.75;
  ctx.lineWidth = width * state.canvasScale;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  points.forEach((pt, idx) => {
    const p = mapToCanvas(pt.x, pt.y);
    if (idx === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawCar(car) {
  const p = mapToCanvas(car.x, car.y);
  const size = 22 * state.canvasScale;
  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(((car.yaw || 0) * Math.PI) / 180);
  ctx.fillStyle = "#fff";
  ctx.strokeStyle = "#e11d48";
  ctx.lineWidth = 3 * state.canvasScale;
  ctx.beginPath();
  ctx.roundRect(-size * 0.55, -size * 0.75, size * 1.1, size * 1.5, 5 * state.canvasScale);
  ctx.fill();
  ctx.stroke();
  ctx.strokeStyle = "#111";
  ctx.lineWidth = 2 * state.canvasScale;
  ctx.beginPath();
  ctx.arc(0, 0, size * 0.32, 0, Math.PI * 2);
  ctx.moveTo(-size * 0.32, 0);
  ctx.lineTo(size * 0.32, 0);
  ctx.moveTo(0, -size * 0.32);
  ctx.lineTo(0, size * 0.32);
  ctx.stroke();
  ctx.restore();
  drawLabel(p.x, p.y, "小车", "#e11d48");
}

function drawDrone(drone) {
  const p = mapToCanvas(drone.x, drone.y);
  ctx.save();
  ctx.strokeStyle = "#2563eb";
  ctx.fillStyle = "#2563eb";
  ctx.lineWidth = 3 * state.canvasScale;
  const arm = 18 * state.canvasScale;
  ctx.beginPath();
  ctx.moveTo(p.x - arm, p.y);
  ctx.lineTo(p.x + arm, p.y);
  ctx.moveTo(p.x, p.y - arm);
  ctx.lineTo(p.x, p.y + arm);
  ctx.stroke();
  for (const [dx, dy] of [[-arm, 0], [arm, 0], [0, -arm], [0, arm]]) {
    ctx.beginPath();
    ctx.arc(p.x + dx, p.y + dy, 6 * state.canvasScale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.arc(p.x, p.y, 5 * state.canvasScale, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
  drawLabel(p.x, p.y, "无人机", "#2563eb", 18);
}

function drawLabel(x, y, text, color, offset = -18) {
  ctx.save();
  ctx.font = `${13 * state.canvasScale}px Microsoft YaHei, Arial`;
  const metrics = ctx.measureText(text);
  const px = x - metrics.width / 2 - 5 * state.canvasScale;
  const py = y + offset * state.canvasScale - 18 * state.canvasScale;
  ctx.fillStyle = "rgba(255,255,255,0.92)";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5 * state.canvasScale;
  ctx.beginPath();
  ctx.roundRect(px, py, metrics.width + 10 * state.canvasScale, 22 * state.canvasScale, 5 * state.canvasScale);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.fillText(text, px + 5 * state.canvasScale, py + 15 * state.canvasScale);
  ctx.restore();
}

function updatePanels() {
  const now = Date.now();
  if (state.drone) {
    els.dronePhase.textContent = phaseName(state.drone.phase);
    els.droneX.textContent = fmt(state.drone.x, "cm");
    els.droneY.textContent = fmt(state.drone.y, "cm");
    els.droneH.textContent = fmt(state.drone.height, "cm");
    els.targetErr.textContent = fmt(state.drone.targetError, "cm");
    els.droneHorizontalSpeed.textContent = fmt(state.drone.horizontalSpeed, "cm/s");
    els.droneVerticalSpeed.textContent = fmt(state.drone.verticalSpeed, "cm/s");
    els.batteryV.textContent = fmt(state.drone.battery, "V", 2);
    els.droneAge.textContent = `${((now - state.drone.receivedAt) / 1000).toFixed(1)} s`;
  }
  if (state.car) {
    els.carPhase.textContent = carPhaseName(state.car.phase);
    els.carX.textContent = fmt(state.car.x, "cm");
    els.carY.textContent = fmt(state.car.y, "cm");
    els.carYaw.textContent = fmt(state.car.yaw, "°", 0);
    els.carSpeed.textContent = fmt(state.car.speed, "cm/s");
    els.carProgress.textContent = fmt(state.car.progress, "cm");
    els.carAge.textContent = `${((now - state.car.receivedAt) / 1000).toFixed(1)} s`;
  }

  const code = state.drone ? phaseCode(state.drone.phase) : "IDLE";
  els.missionStatus.textContent = phaseNames[code] || "待命";
  updateTimeline(code);
  updateLinkBadge(now);
  updateNodeStatus(now);
  updateTimer();
}

function updateTimeline(phaseCodeValue) {
  const order = ["TAKEOFF", "HOVER_150", "FOLLOW", "DROP", "LAND_ON_CAR", "RETURN_HOME", "DONE"];
  const phaseAliases = {
    TAKEOFF: "TAKEOFF",
    HOVER_150: "HOVER_150",
    FOLLOW: "FOLLOW",
    DROP: "DROP",
    LAND_ON_CAR: "LAND_ON_CAR",
    RETURN_HOME: "RETURN_HOME",
    LAND_HOME: "RETURN_HOME",
    DONE: "DONE",
  };
  const current = phaseAliases[phaseCodeValue] || "";
  const idx = order.indexOf(current);
  [...els.timeline.children].forEach((li) => {
    const step = li.dataset.step;
    const stepIdx = order.indexOf(step);
    li.classList.toggle("active", step === current);
    li.classList.toggle("done", idx >= 0 && stepIdx >= 0 && stepIdx < idx);
  });
}

function updateLinkBadge(now) {
  const last = Math.max(
    state.car ? state.car.receivedAt : 0,
    state.drone ? state.drone.receivedAt : 0,
    state.lastServerEventAt
  );
  if (last && now - last < 1200) {
    els.linkBadge.className = "link-badge online";
    els.linkBadge.textContent = "链路在线";
  } else if (last && now - last < 3000) {
    els.linkBadge.className = "link-badge warn";
    els.linkBadge.textContent = "链路延迟";
  } else {
    els.linkBadge.className = "link-badge offline";
    els.linkBadge.textContent = state.simRunning ? "模拟运行" : "离线演示";
  }
}

function markNode(node, detail) {
  if (!state.nodes[node]) return;
  state.nodes[node].seenAt = Date.now();
  state.nodes[node].detail = detail;
}

function updateSingleNode(el, node, onlineText) {
  const data = state.nodes[node];
  if (node === "ground" && state.serialConnected) {
    el.className = "node-status online";
    el.querySelector("small").textContent = data.detail || onlineText;
    return;
  }
  const age = data.seenAt ? Date.now() - data.seenAt : Infinity;
  let stateClass = "offline";
  let text = data.detail;
  if (age < 3000) {
    stateClass = "online";
    text = onlineText;
  } else if (age < 8000) {
    stateClass = "warn";
    text = `延迟 ${Math.round(age / 1000)} s`;
  } else if (data.seenAt) {
    text = "超过 8 s 未收到";
  }
  el.className = `node-status ${stateClass}`;
  el.querySelector("small").textContent = text;
}

function updateNodeStatus(now = Date.now()) {
  updateSingleNode(els.groundNodeStatus, "ground", "串口在线");
  updateSingleNode(els.carNodeStatus, "car", "已收到小车包");
  updateSingleNode(els.droneNodeStatus, "drone", "已收到无人机包");
}

function updateTimer() {
  if (!state.timerRunning || state.timerStart === null) return;
  const elapsed = Date.now() - state.timerStart;
  const total = elapsed / 1000;
  const min = Math.floor(total / 60);
  const sec = total - min * 60;
  els.timer.textContent = `${String(min).padStart(2, "0")}:${sec.toFixed(1).padStart(4, "0")}`;
}

function pushPath(path, point) {
  if (!point) return;
  const last = path[path.length - 1];
  if (!last || Math.hypot(last.x - point.x, last.y - point.y) > 15) {
    path.push({ x: point.x, y: point.y });
    if (path.length > 700) path.shift();
  }
}

function applyTelemetry(msg) {
  const now = Date.now();
  if (msg.kind === "car") {
    markNode("car", "收到小车遥测");
    const x = distanceFromMessage(msg, ["x_cm", "x"], ["x_mm"], state.car?.x ?? 150);
    const y = distanceFromMessage(msg, ["y_cm", "y"], ["y_mm"], state.car?.y ?? 200);
    state.car = {
      x: clamp(x, 0, FIELD_W),
      y: clamp(y, 0, FIELD_H),
      yaw: msg.yaw_deg || 0,
      speed: distanceFromMessage(msg, ["speed_cm_s", "speed"], ["speed_mm_s"], 0),
      progress: distanceFromMessage(msg, ["progress_cm", "progress"], ["progress_mm"], 0),
      phase: msg.phase ?? 0,
      receivedAt: now,
    };
    pushPath(state.carPath, state.car);
  }
  if (msg.kind === "drone") {
    markNode("drone", "收到无人机遥测");
    const x = distanceFromMessage(msg, ["x_cm", "x"], ["x_mm"], state.drone?.x ?? HOME.x);
    const y = distanceFromMessage(msg, ["y_cm", "y"], ["y_mm"], state.drone?.y ?? HOME.y);
    state.drone = {
      x: clamp(x, 0, FIELD_W),
      y: clamp(y, 0, FIELD_H),
      height: distanceFromMessage(msg, ["height_cm", "height", "z"], ["height_mm"], 0),
      targetError: distanceFromMessage(msg, ["target_error_cm", "target_error"], ["target_error_mm"], 0),
      horizontalSpeed: msg.horizontal_speed_cm_s ?? (msg.horizontal_speed_mm_s !== undefined ? msg.horizontal_speed_mm_s / 10 : estimateDroneHorizontalSpeed(msg)),
      verticalSpeed: msg.vertical_speed_cm_s ?? (msg.vertical_speed_mm_s !== undefined ? msg.vertical_speed_mm_s / 10 : estimateDroneVerticalSpeed(msg)),
      battery: msg.battery_v || 0,
      phase: msg.phase ?? 0,
      simTime: msg.sim_time,
      receivedAt: now,
    };
    pushPath(state.dronePath, state.drone);
  }
  if (msg.kind === "status") {
    updateNodeFromStatus(msg);
    els.missionStatus.textContent = msg.text || "状态更新";
  }
}

function updateNodeFromStatus(msg) {
  const text = String(msg.text || "").toLowerCase();
  if (text.includes("ground_online") || msg.source === 3 || msg.src === 3) {
    markNode("ground", "地面站端已启动");
  }
  if (text.includes("car_online") || msg.source === 1 || msg.src === 1) {
    markNode("car", text.includes("car_online") ? "小车端已上线" : "收到小车状态");
  }
  if (text.includes("drone_online") || msg.source === 2 || msg.src === 2) {
    markNode("drone", text.includes("drone_online") ? "无人机端已上线" : "收到无人机状态");
  }
  if (text.includes("car_link_timeout")) {
    state.nodes.car.detail = "无人机端提示小车超时";
  }
}

function addSerialLog(text, kind = "rx") {
  const time = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  const isNearBottom =
    els.serialLog.scrollHeight - els.serialLog.scrollTop - els.serialLog.clientHeight < 16;
  const previousScrollTop = els.serialLog.scrollTop;
  const line = document.createElement("div");
  line.textContent = `[${time}] ${kind}: ${text}`;
  els.serialLog.appendChild(line);

  const maxRows = serialLogFollowLatest || isNearBottom ? 300 : 1000;
  while (els.serialLog.children.length > maxRows) {
    els.serialLog.removeChild(els.serialLog.firstChild);
  }

  if (serialLogFollowLatest || isNearBottom) {
    serialLogFollowLatest = true;
    els.serialLog.scrollTop = els.serialLog.scrollHeight;
  } else {
    els.serialLog.scrollTop = previousScrollTop;
    requestAnimationFrame(() => {
      els.serialLog.scrollTop = previousScrollTop;
    });
  }
}

function parseSerialLine(line) {
  const raw = line.trim();
  if (!raw) return null;

  try {
    const obj = JSON.parse(raw);
    if (obj && typeof obj === "object") {
      if (obj.kind === "car" || obj.kind === "drone" || obj.kind === "status") return obj;
      if (obj.src === "car" || obj.from === "car" || obj.role === "car") return normalizeKvMessage("CAR", obj);
      if (obj.src === "drone" || obj.from === "drone" || obj.role === "drone") return normalizeKvMessage("DRONE", obj);
    }
  } catch {
    // Continue with text formats.
  }

  const tokens = raw.split(/[,\s;，；]+/).filter(Boolean);
  if (tokens.length > 0) {
    const head = tokens[0].toUpperCase();
    const values = {};
    for (const token of tokens.slice(1)) {
      const match = token.match(/^([^=:：]+)[:=：](.+)$/);
      if (match) values[match[1].trim().toLowerCase()] = match[2].trim();
    }
    if (["CAR", "小车", "C"].includes(tokens[0]) || head === "CAR") return normalizeKvMessage("CAR", values);
    if (["DRONE", "无人机", "UAV", "D"].includes(tokens[0]) || head === "DRONE" || head === "UAV") return normalizeKvMessage("DRONE", values);
    if (head === "STATUS" || tokens[0] === "状态") {
      return { kind: "status", text: tokens.slice(1).join(" ") || raw };
    }
  }

  return parseChineseStatus(raw);
}

function num(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function pick(obj, names, fallback = undefined) {
  for (const name of names) {
    if (obj[name] !== undefined) return obj[name];
  }
  return fallback;
}

function distanceFromMessage(obj, cmNames, mmNames, fallback = 0) {
  const cmValue = pick(obj, cmNames);
  if (cmValue !== undefined) return num(cmValue, fallback);
  const mmValue = pick(obj, mmNames);
  if (mmValue !== undefined) return num(mmValue, fallback * 10) / 10;
  return fallback;
}

function normalizeKvMessage(role, values) {
  if (role === "CAR") {
    return {
      kind: "car",
      phase: phaseFromText(pick(values, ["phase", "state", "status", "阶段", "状态"], 2), true),
      x_cm: distanceFromMessage(values, ["x", "x_cm", "cx"], ["x_mm"], state.car?.x ?? 150),
      y_cm: distanceFromMessage(values, ["y", "y_cm", "cy"], ["y_mm"], state.car?.y ?? 200),
      yaw_deg: num(pick(values, ["yaw", "yaw_deg", "angle"], state.car?.yaw ?? 0)),
      speed_cm_s: distanceFromMessage(values, ["speed", "v", "speed_cm_s"], ["speed_mm_s"], state.car?.speed ?? 0),
      progress_cm: distanceFromMessage(values, ["progress", "s", "progress_cm"], ["progress_mm"], state.car?.progress ?? 0),
    };
  }
  return {
    kind: "drone",
    phase: phaseFromText(pick(values, ["phase", "state", "status", "阶段", "状态"], 0), false),
    x_cm: distanceFromMessage(values, ["x", "x_cm"], ["x_mm"], state.drone?.x ?? HOME.x),
    y_cm: distanceFromMessage(values, ["y", "y_cm"], ["y_mm"], state.drone?.y ?? HOME.y),
    height_cm: distanceFromMessage(values, ["h", "height", "height_cm", "z"], ["height_mm"], state.drone?.height ?? 0),
    target_error_cm: distanceFromMessage(values, ["err", "error", "target_error_cm"], ["target_error_mm"], state.drone?.targetError ?? 0),
    horizontal_speed_cm_s: distanceFromMessage(values, ["hs", "horizontal_speed", "horizontal_speed_cm_s"], ["horizontal_speed_mm_s"], state.drone?.horizontalSpeed ?? 0),
    vertical_speed_cm_s: distanceFromMessage(values, ["vs", "vertical_speed", "vertical_speed_cm_s"], ["vertical_speed_mm_s"], state.drone?.verticalSpeed ?? 0),
    battery_v: num(pick(values, ["bat", "batt", "battery", "battery_v"], state.drone?.battery ?? 0)),
  };
}

function phaseFromText(value, isCar) {
  if (typeof value === "number") return value;
  const text = String(value ?? "").toLowerCase();
  if (/^\d+$/.test(text)) return Number(text);
  if (isCar) {
    if (text.includes("启动") || text.includes("start")) return 1;
    if (text.includes("b")) return 3;
    if (text.includes("c")) return 4;
    if (text.includes("d")) return 5;
    if (text.includes("完成") || text.includes("停止") || text.includes("done") || text.includes("stop")) return 13;
    if (text.includes("保护") || text.includes("fail")) return 250;
    return 2;
  }
  if (text.includes("起飞") || text.includes("takeoff")) return 2;
  if (text.includes("悬停") || text.includes("hover")) return 3;
  if (text.includes("搜索") || text.includes("search")) return 4;
  if (text.includes("接近") || text.includes("approach")) return 5;
  if (text.includes("伴飞") || text.includes("follow")) return 6;
  if (text.includes("抛投") || text.includes("drop")) return 7;
  if (text.includes("返航") || text.includes("return")) return 8;
  if (text.includes("起降点") || text.includes("land_home")) return 9;
  if (text.includes("动态降落") || text.includes("平台降落") || text.includes("land_car")) return 10;
  if (text.includes("停留") || text.includes("wait")) return 11;
  if (text.includes("平台起飞")) return 12;
  if (text.includes("完成") || text.includes("done")) return 13;
  if (text.includes("保护") || text.includes("fail")) return 250;
  return 0;
}

function parseChineseStatus(raw) {
  if (raw.includes("小车")) {
    return {
      kind: "car",
      phase: phaseFromText(raw, true),
      x_cm: state.car?.x ?? 150,
      y_cm: state.car?.y ?? 200,
      yaw_deg: state.car?.yaw ?? 0,
      speed_cm_s: state.car?.speed ?? 0,
      progress_cm: state.car?.progress ?? 0,
    };
  }
  if (raw.includes("无人机") || raw.includes("飞机") || raw.includes("抛投") || raw.includes("返航") || raw.includes("降落") || raw.includes("伴飞")) {
    return {
      kind: "drone",
      phase: phaseFromText(raw, false),
      x_cm: state.drone?.x ?? HOME.x,
      y_cm: state.drone?.y ?? HOME.y,
      height_cm: state.drone?.height ?? 0,
      target_error_cm: state.drone?.targetError ?? 0,
      horizontal_speed_cm_s: state.drone?.horizontalSpeed ?? 0,
      vertical_speed_cm_s: state.drone?.verticalSpeed ?? 0,
      battery_v: state.drone?.battery ?? 0,
    };
  }
  return { kind: "status", text: raw };
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    addSerialLog("当前浏览器不支持 Web Serial，请使用 Chrome/Edge，并通过 http://127.0.0.1:8080 打开页面。", "提示");
    els.serialStatus.textContent = "浏览器不支持";
    return;
  }
  try {
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });
    serialKeepReading = true;
    state.serialConnected = true;
    els.serialStatus.textContent = "已连接";
    els.linkBadge.className = "link-badge online";
    els.linkBadge.textContent = "串口在线";
    markNode("ground", "串口已连接");
    updateNodeStatus();
    addSerialLog("串口已连接，波特率 115200", "系统");
    readSerialLoop();
  } catch (error) {
    addSerialLog(`连接失败：${error.message}`, "错误");
    els.serialStatus.textContent = "连接失败";
  }
}

async function readSerialLoop() {
  const decoder = new TextDecoderStream();
  const closed = serialPort.readable.pipeTo(decoder.writable);
  serialReader = decoder.readable.getReader();
  let buffer = "";
  try {
    while (serialKeepReading) {
      const { value, done } = await serialReader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split(/\r?\n/);
      buffer = lines.pop() || "";
      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed) continue;
        addSerialLog(trimmed);
        const msg = parseSerialLine(trimmed);
        if (msg) {
          applyTelemetry(msg);
          state.lastServerEventAt = Date.now();
          draw();
          updatePanels();
        }
      }
    }
  } catch (error) {
    addSerialLog(`读取失败：${error.message}`, "错误");
  } finally {
    serialReader?.releaseLock();
    await closed.catch(() => {});
  }
}

async function disconnectSerial() {
  serialKeepReading = false;
  try {
    await serialReader?.cancel();
    await serialPort?.close();
  } catch {
    // Ignore close errors.
  }
  serialReader = null;
  serialPort = null;
  state.serialConnected = false;
  els.serialStatus.textContent = "未连接";
  state.nodes.ground = { seenAt: 0, detail: "未连接串口" };
  updateNodeStatus();
  addSerialLog("串口已断开", "系统");
}

function estimateDroneHorizontalSpeed(msg) {
  if (!state.drone || !state.drone.receivedAt) return 0;
  const dt = (Date.now() - state.drone.receivedAt) / 1000;
  if (dt <= 0.02) return state.drone.horizontalSpeed || 0;
  const x = distanceFromMessage(msg, ["x_cm", "x"], ["x_mm"], state.drone.x);
  const y = distanceFromMessage(msg, ["y_cm", "y"], ["y_mm"], state.drone.y);
  const dx = x - state.drone.x;
  const dy = y - state.drone.y;
  return Math.hypot(dx, dy) / dt;
}

function estimateDroneVerticalSpeed(msg) {
  if (!state.drone || !state.drone.receivedAt) return 0;
  const dt = (Date.now() - state.drone.receivedAt) / 1000;
  if (dt <= 0.02) return state.drone.verticalSpeed || 0;
  const height = distanceFromMessage(msg, ["height_cm", "height", "z"], ["height_mm"], state.drone.height);
  return (height - state.drone.height) / dt;
}

function simulationTick() {
  if (!state.simRunning) return;
  const elapsed = (Date.now() - state.simStart) / 1000;
  const carSpeed = 11;
  const progress = Math.min(elapsed * carSpeed, TRACK.total);
  const carPose = carPoseFromProgress(progress);
  const carPhase = simulatedCarPhase(progress);
  applyTelemetry({
    kind: "car",
    x_cm: carPose.x,
    y_cm: carPose.y,
    yaw_deg: carPose.yaw,
    speed_cm_s: progress >= TRACK.total ? 0 : carSpeed,
    progress_cm: progress,
    phase: carPhase,
  });

  const drone = simulatedDrone(elapsed, carPose);
  const lastDrone = state.drone;
  if (lastDrone && lastDrone.simTime !== undefined) {
    const dt = elapsed - lastDrone.simTime;
    if (dt > 0.02) {
      drone.horizontal_speed_cm_s = Math.hypot(drone.x_cm - lastDrone.x, drone.y_cm - lastDrone.y) / dt;
      drone.vertical_speed_cm_s = (drone.height_cm - lastDrone.height) / dt;
    }
  }
  drone.horizontal_speed_cm_s ??= 0;
  drone.vertical_speed_cm_s ??= 0;
  drone.sim_time = elapsed;
  applyTelemetry(drone);
}

function simulatedCarPhase(progress) {
  if (progress >= TRACK.total) return 13;
  if (progress >= TRACK.daStart && progress < TRACK.daStart + 10) return 5;
  if (progress >= TRACK.cdStart && progress < TRACK.cdStart + 10) return 4;
  if (progress >= TRACK.bcStart && progress < TRACK.bcStart + 10) return 3;
  if (progress > 0) return 2;
  return 0;
}

function simulatedDrone(t, carPose) {
  if (t < 4) {
    return { kind: "drone", x_cm: HOME.x, y_cm: HOME.y, height_cm: t * 37.5, phase: 2, target_error_cm: 0, battery_v: 12.3 };
  }
  if (t < 7) {
    return { kind: "drone", x_cm: HOME.x, y_cm: HOME.y, height_cm: 150, phase: 3, target_error_cm: 0, battery_v: 12.1 };
  }

  const targetX = carPose.x;
  const targetY = carPose.y;
  const blend = clamp((t - 7) / 4.5, 0, 1);
  let x = HOME.x + (targetX - HOME.x) * blend;
  let y = HOME.y + (targetY - HOME.y) * blend;
  let phase = blend < 1 ? 5 : 6;
  let height = 150;
  let err = Math.max(0, 30 * (1 - blend));

  if (blend >= 1) {
    x = targetX + Math.sin(t * 2) * 2;
    y = targetY + Math.cos(t * 1.7) * 2;
    err = Math.abs(Math.sin(t * 2.3)) * 4;
  }

  if (state.taskMode === 1 && t > 39 && t < 42) {
    phase = 7;
    err = Math.min(err, 2.5);
  }
  if (state.taskMode === 1 && t >= 45) {
    phase = t > 55 ? 9 : 8;
    const b = clamp((t - 45) / 10, 0, 1);
    x = targetX + (HOME.x - targetX) * b;
    y = targetY + (HOME.y - targetY) * b;
    height = t > 55 ? Math.max(0, 150 - (t - 55) * 40) : 150;
  }

  if (state.taskMode === 2 && t > 22 && t < 36) {
    phase = 10;
    height = Math.max(0, 150 - (t - 22) * 10.8);
    err = Math.max(1.5, 8 - (t - 22) * 0.45);
  } else if (state.taskMode === 2 && t >= 36 && t < 41) {
    phase = 11;
    height = 0;
    err = 0;
  } else if (state.taskMode === 2 && t >= 41 && t < 46) {
    phase = 12;
    height = (t - 41) * 30;
    err = 0;
  } else if (state.taskMode === 2 && t >= 50) {
    phase = t > 70 ? 9 : 8;
    const b = clamp((t - 50) / 20, 0, 1);
    x = targetX + (HOME.x - targetX) * b;
    y = targetY + (HOME.y - targetY) * b;
    height = t > 70 ? Math.max(0, 150 - (t - 70) * 40) : 150;
  }

  if (t > 76 || carPose.progress >= TRACK.total) phase = 13;
  return { kind: "drone", x_cm: x, y_cm: y, height_cm: height, phase, target_error_cm: err, battery_v: 12.0 };
}

function startSimulation() {
  state.simRunning = true;
  state.simStart = Date.now();
  startTimer();
}

function pauseSimulation() {
  state.simRunning = false;
}

function resetAll() {
  state.car = null;
  state.drone = null;
  state.carPath = [];
  state.dronePath = [];
  state.simRunning = false;
  state.timerStart = null;
  state.timerRunning = false;
  els.timer.textContent = "00:00.0";
  els.startTimerBtn.textContent = "开始计时";
  draw();
  updatePanels();
}

function startTimer() {
  state.timerStart = Date.now();
  state.timerRunning = true;
  els.startTimerBtn.textContent = "重新计时";
}

function connectServerEvents() {
  if (location.protocol !== "http:" && location.protocol !== "https:") return;
  try {
    const events = new EventSource("/events");
    events.onopen = () => {
      state.lastServerEventAt = Date.now();
      els.linkBadge.className = "link-badge online";
      els.linkBadge.textContent = "等待遥测";
    };
    events.onmessage = (event) => {
      state.lastServerEventAt = Date.now();
      const msg = JSON.parse(event.data);
      applyTelemetry(msg);
      draw();
      updatePanels();
    };
    events.onerror = () => {
      els.linkBadge.className = "link-badge warn";
      els.linkBadge.textContent = "桥接断开";
    };
  } catch {
    // Static file mode keeps the offline simulator available.
  }
}

els.simBtn.addEventListener("click", startSimulation);
els.pauseSimBtn.addEventListener("click", pauseSimulation);
els.task1Btn.addEventListener("click", () => {
  state.taskMode = 1;
  startSimulation();
});
els.task2Btn.addEventListener("click", () => {
  state.taskMode = 2;
  startSimulation();
});
els.resetBtn.addEventListener("click", resetAll);
els.startTimerBtn.addEventListener("click", startTimer);
els.serialBtn.addEventListener("click", connectSerial);
els.disconnectSerialBtn.addEventListener("click", disconnectSerial);
els.serialLog.addEventListener("wheel", () => {
  serialLogFollowLatest = false;
});
els.serialLog.addEventListener("scroll", () => {
  const isNearBottom =
    els.serialLog.scrollHeight - els.serialLog.scrollTop - els.serialLog.clientHeight < 16;
  if (isNearBottom) serialLogFollowLatest = true;
});

els.canvas.addEventListener("mousemove", (event) => {
  if (!state.mapRect) return;
  const rect = els.canvas.getBoundingClientRect();
  const x = (event.clientX - rect.left) * (els.canvas.width / rect.width);
  const y = (event.clientY - rect.top) * (els.canvas.height / rect.height);
  const m = canvasToMap(x, y);
  if (m.x < 0 || m.x > FIELD_W || m.y < 0 || m.y > FIELD_H) {
    els.tooltip.hidden = true;
    return;
  }
  els.tooltip.hidden = false;
  els.tooltip.style.left = `${event.clientX - rect.left + 16}px`;
  els.tooltip.style.top = `${event.clientY - rect.top + 16}px`;
  els.tooltip.innerHTML = `x=${m.x.toFixed(1)} cm<br>y=${m.y.toFixed(1)} cm`;
});

els.canvas.addEventListener("mouseleave", () => {
  els.tooltip.hidden = true;
});

window.addEventListener("resize", draw);

if (!CanvasRenderingContext2D.prototype.roundRect) {
  CanvasRenderingContext2D.prototype.roundRect = function roundRect(x, y, w, h, r) {
    const radius = Math.min(r, w / 2, h / 2);
    this.moveTo(x + radius, y);
    this.arcTo(x + w, y, x + w, y + h, radius);
    this.arcTo(x + w, y + h, x, y + h, radius);
    this.arcTo(x, y + h, x, y, radius);
    this.arcTo(x, y, x + w, y, radius);
    return this;
  };
}

connectServerEvents();
setInterval(() => {
  simulationTick();
  draw();
  updatePanels();
}, 100);
draw();
updatePanels();
