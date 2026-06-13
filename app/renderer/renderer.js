'use strict';
// Renderer — live scan canvas + control wiring for Step 1 (serial acquisition).
// The world<->screen transform and visual treatment mirror the design prototype's
// draw() loop (LiDAR Bridge.dc.html): RANGE=9, sensor at origin, view centred on
// world Y=2.75, baseScale = min(W/9.2, H/7.4).

const $ = (id) => document.getElementById(id);

const RANGE = 9;
const POINT_FADE = 900; // ms — a ray fades out over this window
const SWEEP_PERIOD = 0.85; // s per revolution (cosmetic)
const NBINS = 720; // angle bins for the point cloud (0.5° each)

// ---- device catalogue (mirrors the prototype's SENSORS) -------------------
const SENSORS = [
  { id: 'a3', name: 'RPLIDAR A3', range: '25m', hz: '10',
    cfg: { connType: 'serial', comPort: 'SIM', baudrate: '1000000', scanMode: 'express', precision: '2', distMin: '0.0', distMax: '25.0', coordSys: 'cartesian', quality: false } },
  { id: 's2e', name: 'RPLIDAR S2E', range: '30m', hz: '10',
    cfg: { connType: 'network', comPort: 'COM5', baudrate: '1000000', scanMode: 'standard', precision: '2', distMin: '0.0', distMax: '30.0', coordSys: 'cartesian', quality: true } },
  { id: 'a2', name: 'RPLIDAR A2', range: '12m', hz: '10',
    cfg: { connType: 'serial', comPort: 'COM3', baudrate: '115200', scanMode: 'standard', precision: '1', distMin: '0.0', distMax: '12.0', coordSys: 'cartesian', quality: false } },
];

const cfgs = {};
SENSORS.forEach((s) => (cfgs[s.id] = Object.assign({}, s.cfg)));

const ui = {
  streaming: true,
  connected: false,
  connectedId: null,
  selected: 'a3',
  tab: 'track',
};

// ---- view / canvas --------------------------------------------------------
const wrap = $('canvasWrap');
const canvas = $('scan');
const ctx = canvas.getContext('2d');
const WC = { x: 0, y: 2.75 };
const view = { z: 1, panX: 0, panY: 0 };
let W = 0, H = 0, baseScale = 60, dpr = 1;

function resize() {
  dpr = Math.min(window.devicePixelRatio || 1, 2);
  W = wrap.clientWidth;
  H = wrap.clientHeight;
  canvas.width = W * dpr;
  canvas.height = H * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  baseScale = Math.min(W / 9.2, H / 7.4);
}
const scale = () => baseScale * view.z;
const wts = (wx, wy) => {
  const s = scale();
  return [W / 2 + (wx - WC.x) * s + view.panX, H / 2 - (wy - WC.y) * s + view.panY];
};
const stw = (sx, sy) => {
  const s = scale();
  return [(sx - W / 2 - view.panX) / s + WC.x, WC.y - (sy - H / 2 - view.panY) / s];
};

// ---- point cloud store ----------------------------------------------------
// One slot per angle bin holding the latest WORLD point (metres) + arrival time.
// The pipeline (main process) already applied placement + mm->m, so the renderer
// just plots; persistent slots keep the prototype's sweep-fade look across scans.
const bins = new Array(NBINS).fill(null);
const bg = { captured: false, subtract: false, tol: 0.18, contour: null, show: true, hide: true };

function ingestScan(frame) {
  if (!ui.streaming) return;
  const now = performance.now();
  const pts = frame.pts; // [worldX, worldY, fg] per bin; fg < 0 = empty
  const nb = frame.nbins;
  for (let i = 0; i < nb; i++) {
    const fg = pts[i * 3 + 2];
    if (fg < 0) continue;
    bins[i] = { x: pts[i * 3], y: pts[i * 3 + 1], fg, t: now };
  }
  liveStats.points = frame.count;
  liveStats.latency = frame.periodMs;
  bg.contour = frame.bg || null;
  syncBgUi(frame.bgCaptured, frame.capturing);
}

// ---- stats ----------------------------------------------------------------
const liveStats = { points: 0, latency: 0 };
let frames = 0, fpsAccum = 0, fps = 0;

// ---- draw -----------------------------------------------------------------
let sweep = 0;
let lastFrame = performance.now();

function draw() {
  const now = performance.now();
  const dt = Math.min(0.05, (now - lastFrame) / 1000);
  lastFrame = now;
  if (ui.streaming) sweep = (sweep + (2 * Math.PI * dt) / SWEEP_PERIOD) % (2 * Math.PI);

  const s = scale();
  ctx.clearRect(0, 0, W, H);

  // cartesian grid
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(255,255,255,0.04)';
  for (let gx = -4; gx <= 4; gx++) {
    const [x1, y1] = wts(gx, -0.5), [, y2] = wts(gx, 6);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1, y2); ctx.stroke();
  }
  for (let gy = 0; gy <= 6; gy++) {
    const [x1, y1] = wts(-4, gy), [x2] = wts(4, gy);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y1); ctx.stroke();
  }

  // range rings
  const [ox, oy] = wts(0, 0);
  ctx.strokeStyle = 'rgba(0,229,255,0.07)';
  for (let r = 1.5; r <= RANGE; r += 1.5) {
    ctx.beginPath(); ctx.arc(ox, oy, r * s, 0, 2 * Math.PI); ctx.stroke();
  }

  // captured background baseline (orange dashed ghost outline)
  if (bg.captured && bg.show && bg.contour && bg.contour.length) {
    ctx.save();
    ctx.beginPath();
    for (let i = 0; i < bg.contour.length; i += 2) {
      const [sx, sy] = wts(bg.contour[i], bg.contour[i + 1]);
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
    }
    ctx.strokeStyle = 'rgba(255,176,0,0.32)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }

  // point cloud. with subtraction on: foreground = green, background = dim/hidden.
  const sub = bg.subtract && bg.captured;
  for (let i = 0; i < NBINS; i++) {
    const p = bins[i];
    if (!p) continue;
    const age = now - p.t;
    if (age > POINT_FADE) continue;
    const fresh = Math.max(0, 1 - age / POINT_FADE);
    const [sx, sy] = wts(p.x, p.y);
    const isBg = sub && p.fg === 0;
    if (isBg && bg.hide) continue;
    if (isBg) {
      ctx.fillStyle = 'rgba(120,130,140,' + (0.1 + 0.12 * fresh) + ')';
      ctx.fillRect(sx - 0.8, sy - 0.8, 1.6, 1.6);
      continue;
    }
    if (sub && p.fg === 1) {
      ctx.fillStyle = 'rgba(57,255,122,' + (0.3 + 0.7 * fresh) + ')';
      ctx.fillRect(sx - 1.3, sy - 1.3, 2.8, 2.8);
      continue;
    }
    const a = 0.14 + 0.82 * fresh;
    if (fresh > 0.82) {
      ctx.fillStyle = 'rgba(210,248,255,' + a + ')';
      ctx.fillRect(sx - 1.2, sy - 1.2, 2.6, 2.6);
    } else {
      ctx.fillStyle = 'rgba(0,210,238,' + a + ')';
      ctx.fillRect(sx - 0.9, sy - 0.9, 1.8, 1.8);
    }
  }

  // sweep line
  const [ex, ey] = wts(Math.cos(sweep) * RANGE, Math.sin(sweep) * RANGE);
  const grad = ctx.createLinearGradient(ox, oy, ex, ey);
  grad.addColorStop(0, 'rgba(0,229,255,0.5)');
  grad.addColorStop(1, 'rgba(0,229,255,0)');
  ctx.strokeStyle = grad;
  ctx.lineWidth = 1.6;
  ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ex, ey); ctx.stroke();

  // sensor marker
  ctx.save();
  ctx.translate(ox, oy);
  ctx.strokeStyle = 'rgba(0,229,255,0.18)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(0, 0, RANGE * s, 0, 2 * Math.PI); ctx.stroke();
  ctx.fillStyle = '#00e5ff';
  ctx.shadowColor = '#00e5ff';
  ctx.shadowBlur = 10;
  ctx.beginPath(); ctx.arc(0, 0, 6, 0, 2 * Math.PI); ctx.fill();
  ctx.shadowBlur = 0;
  ctx.fillStyle = '#06181c';
  ctx.beginPath(); ctx.arc(0, 0, 2.5, 0, 2 * Math.PI); ctx.fill();
  ctx.restore();

  // stats
  frames++;
  fpsAccum += dt;
  if (fpsAccum >= 0.5) {
    fps = Math.round(frames / fpsAccum);
    frames = 0; fpsAccum = 0;
    $('fps').textContent = fps;
  }
  $('pointCount').textContent = ui.streaming ? liveStats.points : '—';
  $('latency').textContent = ui.connected && liveStats.latency ? liveStats.latency.toFixed(0) : '—';

  requestAnimationFrame(draw);
}

// ---- device list rendering ------------------------------------------------
function renderDevices() {
  const list = $('deviceList');
  list.innerHTML = '';
  let online = 0;
  for (const s of SENSORS) {
    const isOnline = ui.connected && ui.connectedId === s.id;
    if (isOnline) online++;
    const sel = ui.selected === s.id;
    const card = document.createElement('div');
    card.className = 'dev-card' + (sel ? ' sel' : '');
    const dotColor = isOnline ? '#39ff7a' : '#ff4d5e';
    const c = cfgs[s.id];
    const addr = c.connType === 'serial' ? c.comPort : c.ipAddr || 'NET';
    const conn = c.connType === 'serial' ? 'SERIAL' : 'NET';
    card.innerHTML =
      (sel ? '<span class="selbar"></span>' : '') +
      `<div style="display:flex;align-items:center;gap:7px">
         <span class="dot" style="background:${dotColor};box-shadow:0 0 6px ${dotColor}"></span>
         <span class="dev-name" style="color:${sel ? '#e8ecf1' : '#cdd4dc'}">${s.name}</span>
       </div>
       <div class="mono" style="display:flex;justify-content:space-between;font-size:9.5px;color:#5b636d"><span>${conn} ${addr}</span></div>
       <div class="mono" style="display:flex;gap:10px;font-size:9.5px;color:#717a84"><span>RANGE ${s.range}</span><span>${s.hz} Hz</span></div>`;
    card.onclick = () => selectDevice(s.id);
    list.appendChild(card);
  }
  $('onlineCount').textContent = online + ' ONLINE';
}

function selectDevice(id) {
  saveConnFields(ui.selected);
  ui.selected = id;
  const s = SENSORS.find((x) => x.id === id);
  $('selName').textContent = s.name;
  loadConnFields(id);
  renderDevices();
}

// ---- form <-> cfg sync ----------------------------------------------------
function loadConnFields(id) {
  const c = cfgs[id];
  setConn(c.connType);
  $('comPort').value = c.comPort;
  $('baudrate').value = c.baudrate;
  $('ipAddr').value = c.ipAddr || '192.168.11.2';
  $('ipPort').value = c.ipPort || '8089';
  setScan(c.scanMode);
  setPrecision(c.precision);
  $('distMin').value = c.distMin;
  $('distMax').value = c.distMax;
  setCoord(c.coordSys);
  setQuality(c.quality);
  $('posX').value = '0.00'; $('posY').value = '0.00'; $('rot').value = '0.0';
  pushPlacement();
}

function pushPlacement() {
  const placement = {
    x: parseFloat($('posX').value) || 0,
    y: parseFloat($('posY').value) || 0,
    rot: parseFloat($('rot').value) || 0,
  };
  window.lidar.setConfig({ placement });
}

function saveConnFields(id) {
  const c = cfgs[id];
  if (!c) return;
  c.comPort = $('comPort').value;
  c.baudrate = $('baudrate').value;
  c.ipAddr = $('ipAddr').value;
  c.ipPort = $('ipPort').value;
  c.distMin = $('distMin').value;
  c.distMax = $('distMax').value;
}

function segActivate(groupAttr, attr, val) {
  document.querySelectorAll(`[${groupAttr}]`).forEach((b) => {
    b.classList.toggle('active', b.getAttribute(groupAttr) === val);
  });
}

let connType = 'serial', scanMode = 'express', precision = '2', coordSys = 'cartesian', quality = false;

function setConn(v) {
  connType = v;
  segActivate('data-conn', 'data-conn', v);
  $('serialFields').style.display = v === 'serial' ? 'flex' : 'none';
  $('networkFields').style.display = v === 'network' ? 'flex' : 'none';
  cfgs[ui.selected].connType = v;
}
function setScan(v) { scanMode = v; segActivate('data-scan', 'data-scan', v); cfgs[ui.selected].scanMode = v; }
function setPrecision(v) {
  precision = String(v);
  segActivate('data-p', 'data-p', precision);
  $('samplesLabel').textContent = 360 * parseInt(precision, 10) + ' samples/scan';
  cfgs[ui.selected].precision = precision;
}
function setCoord(v) { coordSys = v; segActivate('data-coord', 'data-coord', v); cfgs[ui.selected].coordSys = v; }
function setQuality(v) {
  quality = !!v;
  $('qualityBtn').classList.toggle('active', quality);
  $('qualityBtn').textContent = 'FILTER ' + (quality ? 'ON' : 'OFF');
  cfgs[ui.selected].quality = quality;
  if (ui.connected) window.lidar.setConfig({ quality });
}

// ---- background mask ------------------------------------------------------
function syncBgUi(captured, capturing) {
  bg.captured = !!captured;
  const status = $('bgStatus');
  if (capturing) { status.textContent = 'CAPTURING…'; status.style.color = '#ffb000'; }
  else if (captured) { status.textContent = 'CAPTURED'; status.style.color = '#39ff7a'; }
  else { status.textContent = 'NO BASELINE'; status.style.color = '#5b636d'; }
  $('bgControls').style.display = captured ? 'flex' : 'none';
}
function setBgSubtract(on) {
  bg.subtract = on;
  $('bgKnob').style.left = on ? '24px' : '2px';
  $('bgKnob').style.background = on ? '#39ff7a' : '#717a84';
  $('bgSubToggle').style.borderColor = on ? 'rgba(57,255,122,0.5)' : 'rgba(255,255,255,0.12)';
  $('bgSubToggle').style.background = on ? 'rgba(57,255,122,0.12)' : '#0e1216';
  window.lidar.setConfig({ bgSubtract: on });
}

// ---- connect / disconnect -------------------------------------------------
async function doConnect() {
  if (ui.connected) {
    await window.lidar.disconnect();
    ui.connected = false;
    ui.connectedId = null;
    setConnStatus('disconnected', '#717a84');
    $('connectBtn').textContent = 'CONNECT';
    $('connectBtn').classList.add('green');
    $('protoDot').style.background = '#454c54';
    renderDevices();
    return;
  }
  saveConnFields(ui.selected);
  const cfg = Object.assign({}, cfgs[ui.selected], {
    comPort: $('comPort').value,
    baudrate: $('baudrate').value,
    scanMode,
    distMin: $('distMin').value,
    distMax: $('distMax').value,
    quality,
    connType,
    placement: {
      x: parseFloat($('posX').value) || 0,
      y: parseFloat($('posY').value) || 0,
      rot: parseFloat($('rot').value) || 0,
    },
  });
  setConnStatus('connecting…', '#ffb000');
  const res = await window.lidar.connect(cfg);
  if (res.ok) {
    ui.connected = true;
    ui.connectedId = ui.selected;
    const fw = res.info && res.info.firmware ? ' · fw ' + res.info.firmware : '';
    setConnStatus((res.simulated ? 'SIMULATOR' : 'connected') + fw, '#39ff7a');
    $('connectBtn').textContent = 'DISCONNECT';
    $('connectBtn').classList.remove('green');
    $('protoDot').style.background = '#39ff7a';
    $('protoDot').style.boxShadow = '0 0 7px #39ff7a';
  } else {
    setConnStatus('ERROR: ' + res.error, '#ff4d5e');
  }
  renderDevices();
}

function setConnStatus(msg, color) {
  const el = $('connStatus');
  el.textContent = msg;
  el.style.color = color || '#717a84';
}

// ---- pan / zoom -----------------------------------------------------------
function zoomAt(cx, cy, factor) {
  const before = stw(cx, cy);
  view.z = Math.max(0.3, Math.min(6, view.z * factor));
  const after = stw(cx, cy);
  const s = scale();
  view.panX += (after[0] - before[0]) * s;
  view.panY -= (after[1] - before[1]) * s;
}
function fitView() { view.z = 1; view.panX = 0; view.panY = 0; }

// ---- wiring ---------------------------------------------------------------
function wireControls() {
  document.querySelectorAll('[data-conn]').forEach((b) => (b.onclick = () => setConn(b.getAttribute('data-conn'))));
  document.querySelectorAll('[data-scan]').forEach((b) => (b.onclick = () => setScan(b.getAttribute('data-scan'))));
  document.querySelectorAll('[data-p]').forEach((b) => (b.onclick = () => setPrecision(b.getAttribute('data-p'))));
  document.querySelectorAll('[data-coord]').forEach((b) => (b.onclick = () => setCoord(b.getAttribute('data-coord'))));
  $('qualityBtn').onclick = () => setQuality(!quality);
  $('connectBtn').onclick = doConnect;

  const pushDist = () => { if (ui.connected) window.lidar.setConfig({ distMin: $('distMin').value, distMax: $('distMax').value }); };
  $('distMin').onchange = pushDist;
  $('distMax').onchange = pushDist;

  $('posX').onchange = pushPlacement;
  $('posY').onchange = pushPlacement;
  $('rot').onchange = pushPlacement;
  // background mask
  $('captureBg').onclick = () => {
    if (!ui.connected) { setConnStatus('connect a sensor first', '#ffb000'); return; }
    window.lidar.captureBg();
    syncBgUi(false, true);
  };
  $('bgSubToggle').onclick = () => setBgSubtract(!bg.subtract);
  $('bgTol').oninput = () => {
    bg.tol = parseFloat($('bgTol').value);
    $('bgTolLabel').textContent = Math.round(bg.tol * 100) + ' cm';
    window.lidar.setConfig({ bgTol: bg.tol });
  };
  $('clearBg').onclick = () => {
    window.lidar.clearBg();
    bg.subtract = false;
    setBgSubtract(false);
    syncBgUi(false, false);
  };

  $('autoLevel').onclick = () => {
    $('posX').value = '0.00'; $('posY').value = '0.00'; $('rot').value = '0.0';
    pushPlacement();
    setConnStatus('placement re-levelled (0,0,0°)', '#9aa3ad');
  };

  // stream toggle (freezes canvas + acquisition display)
  $('streamBtn').onclick = () => {
    ui.streaming = !ui.streaming;
    $('streamLabel').textContent = ui.streaming ? 'STREAMING' : 'PAUSED';
    renderStreamGlyph();
  };
  renderStreamGlyph();

  // tabs
  document.querySelectorAll('.tab').forEach((t) => {
    t.onclick = () => {
      ui.tab = t.getAttribute('data-tab');
      document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('active', x === t));
      document.querySelectorAll('.tab-pane').forEach((p) => p.classList.toggle('active', p.getAttribute('data-pane') === ui.tab));
    };
  });

  // zoom / pan
  $('zoomIn').onclick = () => zoomAt(W / 2, H / 2, 1.2);
  $('zoomOut').onclick = () => zoomAt(W / 2, H / 2, 1 / 1.2);
  $('fitBtn').onclick = fitView;
  wrap.addEventListener('wheel', (e) => {
    e.preventDefault();
    const r = wrap.getBoundingClientRect();
    zoomAt(e.clientX - r.left, e.clientY - r.top, e.deltaY < 0 ? 1.1 : 1 / 1.1);
  }, { passive: false });

  let dragging = false, lastX = 0, lastY = 0;
  wrap.addEventListener('mousedown', (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
  window.addEventListener('mouseup', () => (dragging = false));
  wrap.addEventListener('mousemove', (e) => {
    const r = wrap.getBoundingClientRect();
    const [wx, wy] = stw(e.clientX - r.left, e.clientY - r.top);
    $('cursor').textContent = `x ${wx.toFixed(2)}  y ${wy.toFixed(2)}`;
    if (dragging) {
      view.panX += e.clientX - lastX;
      view.panY += e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
    }
  });

  // inert buttons (later steps) — give honest feedback
  ['projectBtn', 'recordBtn', 'protoPill'].forEach((id) => {
    $(id).onclick = () => setConnStatus($(id).textContent.trim() + ' — implemented in a later build step', '#717a84');
  });
}

function renderStreamGlyph() {
  $('streamGlyph').innerHTML = ui.streaming
    ? '<span style="display:flex;gap:2.5px"><span style="width:3px;height:11px;background:#00e5ff;display:block"></span><span style="width:3px;height:11px;background:#00e5ff;display:block"></span></span>'
    : '<span style="width:0;height:0;border-left:9px solid #00e5ff;border-top:6px solid transparent;border-bottom:6px solid transparent;display:block"></span>';
}

// ---- ports ----------------------------------------------------------------
async function refreshPorts() {
  const ports = await window.lidar.listPorts();
  const dl = $('portList');
  dl.innerHTML = '<option value="SIM">SIM — built-in simulator</option>';
  for (const p of ports) {
    const o = document.createElement('option');
    o.value = p.path;
    o.label = p.manufacturer ? `${p.path} (${p.manufacturer})` : p.path;
    dl.appendChild(o);
  }
}

// ---- boot -----------------------------------------------------------------
function boot() {
  resize();
  new ResizeObserver(resize).observe(wrap);
  loadConnFields(ui.selected);
  $('selName').textContent = SENSORS.find((s) => s.id === ui.selected).name;
  renderDevices();
  wireControls();
  refreshPorts();

  window.lidar.onScan(ingestScan);
  window.lidar.onStatus((msg) => setConnStatus(msg, '#9aa3ad'));
  window.lidar.onInfo((info) => console.log('device info', info));

  requestAnimationFrame(draw);
}

boot();
