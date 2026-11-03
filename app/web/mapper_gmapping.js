// mapper_gmapping のブラウザ画面
//
// juiz の HTTP ブローカーの /api/container_process/call で ContainerProcess を呼ぶ。
// - コンテナ gmapping: get_state(5 Hz)、get_map と get_map_info(地図が更新されたとき)、save_map、reset
// - juiz 0.2 の Transformations: 推定した姿勢は mapper の ContainerPose(/api/container/pose、基準は map_frame)から読む。
//   読めなければ get_state の pose を使う
// - mobile_sim(subsystem としてつながっていれば): get_state(真の姿勢とスキャン)、set_velocity(操作)
//   mapper の HTTP 経由で ipc://mobile_sim/... を呼ぶので、mobile_sim 側の CORS の設定は要らない
"use strict";

const POLL_MS = 200;
const MAP_POLL_MS = 1000;
const TRAIL_MAX = 3000;
const UNKNOWN = 205;

// ---- juiz の呼び出し ----

class JuizClient {
  constructor() {
    this.ids = {}; // "gmapping_get_state" などの型名 → 識別子
    this.containers = {};
  }

  // ContainerProcess の一覧(subsystem の分も)から識別子を集める。同じ型が複数あれば最初のもの
  async discover() {
    const res = await fetch("/api/container_process/list?recursive=true");
    if (!res.ok) throw new Error(`container_process/list: ${res.status}`);
    for (const e of await res.json()) {
      const [type, container] = e.type_name.split(":");
      if (this.ids[type]) continue;
      const ns = e.namespace ? `${e.namespace}/` : "";
      this.ids[type] = `${e.broker_type_name}://${e.broker_name}/container_process/${ns}${e.name}::${e.type_name}`;
      this.containers[type] = container;
    }
    for (const k of ["get_state", "get_map", "get_map_info", "save_map", "reset"]) {
      if (!this.ids[`gmapping_${k}`]) throw new Error(`gmapping_${k} が見つかりません`);
    }
  }

  has(type) { return Boolean(this.ids[type]); }

  // Container `name` の、Anchor `reference` から見た位置。Transformations が無い・Transform が無いときは null
  async containerPose(name, reference) {
    if (this.noPose || !reference) return null;
    const q = `identifier=${encodeURIComponent(name)}&reference=${encodeURIComponent(reference)}`;
    const res = await fetch(`/api/container/pose?${q}`);
    if (res.status === 404) { this.noPose = true; return null; }
    if (!res.ok) return null;
    const p = (await res.json()).pose;
    const q4 = p.orientation;
    const theta = Math.atan2(2 * (q4.w * q4.z + q4.x * q4.y), 1 - 2 * (q4.y * q4.y + q4.z * q4.z));
    return { x: p.position.x, y: p.position.y, theta };
  }

  async call(type, args = {}, asBlob = false) {
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(args),
    });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return asBlob ? res.blob() : res.json();
  }
}

// ---- 座標変換 ----
// 表示する範囲 (世界座標の正方形) をキャンバスに合わせる。地図の画像は info.origin が左下の画素の角

class View {
  constructor(canvas) {
    this.canvas = canvas;
    this.cx = 0; this.cy = 0; this.size = 20; // 表示する正方形の中心と一辺 [m]
  }

  setRange(xmin, ymin, xmax, ymax) {
    this.cx = (xmin + xmax) / 2;
    this.cy = (ymin + ymax) / 2;
    this.size = Math.max(xmax - xmin, ymax - ymin, 1);
  }

  get pxPerM() { return this.canvas.width / this.size; }

  toCanvas(x, y) {
    const k = this.pxPerM;
    return [(x - this.cx) * k + this.canvas.width / 2, this.canvas.height / 2 - (y - this.cy) * k];
  }
}

// ---- 地図 ----

class MapLayer {
  constructor() {
    this.image = null;
    this.info = null;
    this.known = null; // 不明でない画素の範囲(世界座標)[xmin, ymin, xmax, ymax]
    this.updates = -1;
    this.blob = null;
  }

  async load(client) {
    const blob = await client.call("gmapping_get_map", {}, true);
    const info = await client.call("gmapping_get_map_info");
    const image = await loadImage(blob);
    this.known = knownArea(image, info);
    this.image = image;
    this.info = info;
    this.updates = info.updates;
    this.blob = blob;
  }

  draw(ctx, view) {
    if (!this.image) return;
    const { resolution: r, origin: [ox, oy], width: w, height: h } = this.info;
    const [x0, y0] = view.toCanvas(ox, oy + h * r); // 画像の左上
    const k = view.pxPerM * r;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.image, x0, y0, w * k, h * k);
  }
}

function knownArea(image, info) {
  const c = document.createElement("canvas");
  c.width = image.width;
  c.height = image.height;
  const g = c.getContext("2d");
  g.drawImage(image, 0, 0);
  const px = g.getImageData(0, 0, c.width, c.height).data;
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (let j = 0; j < c.height; j++) {
    for (let i = 0; i < c.width; i++) {
      if (px[(j * c.width + i) * 4] !== UNKNOWN) {
        x0 = Math.min(x0, i); x1 = Math.max(x1, i);
        y0 = Math.min(y0, j); y1 = Math.max(y1, j);
      }
    }
  }
  if (x0 === Infinity) return null;
  const { resolution: r, origin: [ox, oy] } = info;
  return [ox + x0 * r, oy + (c.height - 1 - y1) * r, ox + (x1 + 1) * r, oy + (c.height - y0) * r];
}

async function loadImage(blob) {
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  } finally {
    URL.revokeObjectURL(url);
  }
}

// ---- 描画 ----

function draw(view, map, st, sim, trails, opts) {
  const ctx = view.canvas.getContext("2d");
  ctx.fillStyle = "#cdcdcd";
  ctx.fillRect(0, 0, view.canvas.width, view.canvas.height);
  map.draw(ctx, view);
  if (!st) return;

  const robot = sim ? sim.robots[0] : null;
  const radius = robot ? robot.radius : 0.2;
  // 取付位置は mapper が lidar_mount から受け取ったもの(SLAM が使っている値)
  if (robot && opts.showScan) drawScan(ctx, view, st.pose, robot.scan, st.laser_pose || robot.scan.mount);
  if (opts.showOdom) {
    drawPath(ctx, view, trails.odom, "rgba(120, 120, 120, 0.7)", [4, 4]);
    drawRobot(ctx, view, st.odom, radius, { stroke: "#888", dash: [4, 3] });
  }
  if (robot && opts.showTruth) {
    drawPath(ctx, view, trails.truth, "rgba(26, 138, 58, 0.6)");
    drawRobot(ctx, view, robot.pose, radius, { stroke: "#1a8a3a" });
  }
  drawPath(ctx, view, st.trajectory.map(([x, y]) => [x, y]), "rgba(224, 96, 0, 0.8)");
  if (opts.showParticles) {
    ctx.fillStyle = "rgba(45, 106, 214, 0.8)";
    for (const [x, y] of st.particles) {
      const [px, py] = view.toCanvas(x, y);
      ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
    }
  }
  drawRobot(ctx, view, st.pose, radius, { fill: "rgba(224, 96, 0, 0.85)", stroke: "#803000" });
}

function drawPath(ctx, view, pts, color, dash = []) {
  if (pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.setLineDash(dash);
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [cx, cy] = view.toCanvas(x, y);
    if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawRobot(ctx, view, p, radius, { fill = null, stroke, dash = [] }) {
  const [cx, cy] = view.toCanvas(p.x, p.y);
  const rad = Math.max(radius * view.pxPerM, 4);
  ctx.lineWidth = 2;
  ctx.setLineDash(dash);
  ctx.strokeStyle = stroke;
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.stroke();
  ctx.setLineDash([]);
  // 向き(キャンバスは y が下向きなので角度の符号を反転)
  ctx.strokeStyle = fill ? "#fff" : stroke;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + rad * Math.cos(-p.theta), cy + rad * Math.sin(-p.theta));
  ctx.stroke();
}

// mobile_sim のスキャンを、推定した姿勢から描く(地図の壁と重なれば推定が合っている)
function drawScan(ctx, view, pose, s, mount) {
  const c = Math.cos(pose.theta), sn = Math.sin(pose.theta);
  const sx = pose.x + c * mount.x - sn * mount.y;
  const sy = pose.y + sn * mount.x + c * mount.y;
  const st = pose.theta + mount.theta;
  ctx.fillStyle = "#d4006a";
  s.ranges.forEach((range, i) => {
    if (range > s.range_max) return;
    const a = st + s.angle_min + i * s.angle_increment;
    const [ex, ey] = view.toCanvas(sx + range * Math.cos(a), sy + range * Math.sin(a));
    ctx.fillRect(ex - 1, ey - 1, 2, 2);
  });
}

// ---- 速度の入力(キーボードとパッド) ----

class VelocityInput {
  constructor(pad) {
    this.keys = new Set();
    this.padValue = null; // [前後 -1..1, 旋回 -1..1]
    this.pad = pad;
    window.addEventListener("keydown", (e) => {
      if (e.target instanceof HTMLInputElement && e.target.type === "text") return;
      if (this.keyAxis(e.key)) { this.keys.add(e.key); e.preventDefault(); }
    });
    window.addEventListener("keyup", (e) => this.keys.delete(e.key));
    window.addEventListener("blur", () => this.keys.clear());
    const move = (e) => {
      const rect = pad.getBoundingClientRect();
      const u = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      const v = ((e.clientY - rect.top) / rect.height) * 2 - 1;
      this.padValue = [clamp(-v, -1, 1), clamp(-u, -1, 1)];
    };
    pad.addEventListener("pointerdown", (e) => { pad.setPointerCapture(e.pointerId); move(e); });
    pad.addEventListener("pointermove", (e) => { if (this.padValue) move(e); });
    const release = () => { this.padValue = null; };
    pad.addEventListener("pointerup", release);
    pad.addEventListener("pointercancel", release);
    this.drawPad();
  }

  keyAxis(key) {
    return {
      w: [1, 0], ArrowUp: [1, 0], s: [-1, 0], ArrowDown: [-1, 0],
      a: [0, 1], ArrowLeft: [0, 1], d: [0, -1], ArrowRight: [0, -1],
    }[key];
  }

  value() {
    if (this.padValue) return this.padValue;
    if (this.keys.size === 0) return null;
    let f = 0, t = 0;
    for (const k of this.keys) {
      const ax = this.keyAxis(k);
      if (ax) { f += ax[0]; t += ax[1]; }
    }
    return [clamp(f, -1, 1), clamp(t, -1, 1)];
  }

  drawPad() {
    const ctx = this.pad.getContext("2d");
    const { width: w, height: h } = this.pad;
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = "#bbb";
    ctx.beginPath();
    ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h);
    ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
    ctx.stroke();
    const v = this.value();
    if (v) {
      ctx.fillStyle = "#2d6ad6";
      ctx.beginPath();
      ctx.arc(w / 2 - (v[1] * w) / 2, h / 2 - (v[0] * h) / 2, 10, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function pushTrail(trail, x, y) {
  const last = trail[trail.length - 1];
  if (!last || Math.hypot(x - last[0], y - last[1]) > 0.02) {
    trail.push([x, y]);
    if (trail.length > TRAIL_MAX) trail.shift();
  }
}

// ---- 表示 ----

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => v.toFixed(d);
const fmtPose = (p) => `x ${fmt(p.x)}, y ${fmt(p.y)}, θ ${fmt(p.theta)}`;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const poseErr = (a, b) => `${fmt(Math.hypot(a.x - b.x, a.y - b.y), 3)} m, ${fmt(Math.abs(wrap(a.theta - b.theta)), 3)} rad`;

function showState(st, sim, map, name, fromContainer) {
  $("s_pose").textContent = st.initialized ? fmtPose(st.pose) : (st.waiting_laser_pose ? "(LiDAR の取付位置待ち)" : "(スキャン待ち)");
  $("s_frame").textContent = !st.map_frame ? "(書かない)"
    : st.pose_error ? `${st.map_frame} → ${name}: ${st.pose_error}`
    : `${st.map_frame} → ${name}${fromContainer ? "(ContainerPose から)" : ""}`;
  $("s_frame").classList.toggle("bad", Boolean(st.pose_error));
  $("s_mount").textContent = st.laser_pose ? fmtPose(st.laser_pose) : "-";
  $("s_updates").textContent = `${st.updates} 回(粒子 ${st.particles.length})`;
  $("s_scans").textContent = `${st.scans}(捨てた ${st.dropped})`;
  $("s_map").textContent = map.info ? `${map.info.width}×${map.info.height}、${map.info.resolution} m、${map.updates} 回目の更新まで` : "まだ無い";
  $("s_error").textContent = st.last_error || "なし";
  $("s_error").classList.toggle("bad", Boolean(st.last_error));
  const robot = sim ? sim.robots[0] : null;
  $("s_truth").textContent = robot ? fmtPose(robot.pose) : "(mobile_sim が無い)";
  $("s_err").textContent = robot && st.initialized ? poseErr(st.pose, robot.pose) : "-";
  $("s_odom_err").textContent = robot && st.initialized ? poseErr(st.odom, robot.pose) : "-";
}

function setStatus(id, text, cls) {
  const c = $(id);
  c.textContent = text;
  c.className = `status ${cls}`;
}

function showError(err) {
  console.error(err);
  setStatus("connection", `エラー: ${err.message}`, "error");
}

function say(text) { $("message").textContent = text; }

async function main() {
  const client = new JuizClient();
  await client.discover();
  setStatus("connection", `接続済み(コンテナ ${client.containers.gmapping_get_state})`, "ok");
  const hasSim = client.has("mobile_sim_get_state");
  if (hasSim) setStatus("sim_connection", "mobile_sim につながっています", "ok");
  else setStatus("sim_connection", "mobile_sim は見つかりません(真の姿勢は出しません)", "warn");
  const canDrive = client.has("mobile_sim_set_velocity");
  $("drive").hidden = !canDrive;

  const view = new View($("map"));
  const map = new MapLayer();
  const input = canDrive ? new VelocityInput($("pad")) : null;
  const trails = { truth: [], odom: [] };
  let st = null, sim = null, sentZero = true, loadingMap = false, lastMapPoll = 0;

  $("save").addEventListener("click", async () => {
    try {
      const yaml = await client.call("gmapping_save_map", { path: $("save_path").value });
      say(`保存しました: ${yaml}`);
    } catch (err) { say(`保存できませんでした: ${err.message}`); }
  });
  $("download").addEventListener("click", () => {
    if (!map.blob) { say("地図がまだありません"); return; }
    const a = document.createElement("a");
    a.href = URL.createObjectURL(map.blob);
    a.download = "map.png";
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });
  $("reset").addEventListener("click", async () => {
    try {
      await client.call("gmapping_reset");
      map.image = null; map.info = null; map.known = null; map.updates = -1; map.blob = null;
      trails.truth = []; trails.odom = [];
      say("地図を捨てました。次のスキャンから作り直します");
    } catch (err) { say(`できませんでした: ${err.message}`); }
  });

  let busy = false;
  setInterval(async () => {
    if (busy) return; // 前の周がまだ終わっていない
    busy = true;
    try {
      st = await client.call("gmapping_get_state");
      // 推定した姿勢は ContainerPose(Anchor map_frame から見た mapper)から読む
      const name = client.containers.gmapping_get_state;
      const cp = st.initialized ? await client.containerPose(name, st.map_frame) : null;
      if (cp) st.pose = cp;
      if (hasSim) sim = await client.call("mobile_sim_get_state");
      if (st.initialized) pushTrail(trails.odom, st.odom.x, st.odom.y);
      if (sim) pushTrail(trails.truth, sim.robots[0].pose.x, sim.robots[0].pose.y);

      // 地図は更新があったときだけ取り直す(mapper 側も map_update_interval ごとにしか作り直さない)
      const now = Date.now();
      if (!loadingMap && st.updates > 0 && st.updates !== map.updates && now - lastMapPoll > MAP_POLL_MS) {
        loadingMap = true;
        lastMapPoll = now;
        map.load(client).catch(showError).finally(() => { loadingMap = false; });
      }

      const margin = 1.0;
      if ($("fit").checked && map.known) {
        const [x0, y0, x1, y1] = map.known;
        view.setRange(x0 - margin, y0 - margin, x1 + margin, y1 + margin);
      } else if (map.info) {
        const { resolution: r, origin: [ox, oy], width: w, height: h } = map.info;
        view.setRange(ox, oy, ox + w * r, oy + h * r);
      } else if (st.initialized) {
        view.setRange(st.pose.x - 5, st.pose.y - 5, st.pose.x + 5, st.pose.y + 5);
      }
      draw(view, map, st.initialized ? st : null, sim, trails, {
        showParticles: $("show_particles").checked,
        showScan: $("show_scan").checked,
        showTruth: $("show_truth").checked,
        showOdom: $("show_odom").checked,
      });
      showState(st, sim, map, client.containers.gmapping_get_state, Boolean(cp));

      if (input) {
        // 入力があるあいだは指令を送り続ける(mobile_sim は指令が途切れると止まる)
        input.drawPad();
        const v = input.value();
        if (v) {
          await client.call("mobile_sim_set_velocity", {
            velocity: { linear: { x: v[0] * Number($("max_v").value), y: 0.0 }, angular: v[1] * Number($("max_w").value) },
            index: 0,
          });
          sentZero = false;
        } else if (!sentZero) {
          await client.call("mobile_sim_set_velocity", { velocity: { linear: { x: 0, y: 0 }, angular: 0 }, index: 0 });
          sentZero = true;
        }
      }
      setStatus("connection", `接続済み(コンテナ ${client.containers.gmapping_get_state})`, "ok");
    } catch (err) {
      showError(err);
    } finally {
      busy = false;
    }
  }, POLL_MS);
}

main().catch(showError);
