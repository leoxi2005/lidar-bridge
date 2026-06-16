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

// ---- device list ----------------------------------------------------------
// No hardcoded LiDAR presets — the list starts with only the simulator, and real
// devices are added by AUTO-DETECT (which probes every COM port) or "+ ADD DEVICE"
// (manual entry). This way the panel only ever shows devices that actually exist.
const DEFAULT_CFG = { connType: 'serial', comPort: '', baudrate: '1000000', scanMode: 'standard', precision: '2', distMin: '0.0', distMax: '30.0', coordSys: 'cartesian', quality: false };
const SIM_DEVICE = {
  id: 'sim', name: 'Simulator (SIM)', range: '—', hz: '10', kind: 'sim',
  cfg: Object.assign({}, DEFAULT_CFG, { comPort: 'SIM' }),
};
let SENSORS = [SIM_DEVICE];

const cfgs = {};
SENSORS.forEach((s) => (cfgs[s.id] = Object.assign({}, s.cfg)));

const ui = {
  streaming: true,
  connected: false,
  connectedId: null,
  selected: 'sim',
  tab: 'track',
};

let manualSeq = 0;

// Build a stable device id from a serial port path.
function devIdForPath(path) { return 'dev_' + String(path).replace(/[^a-z0-9]/gi, '_'); }

// Replace the previously-detected devices with a fresh AUTO-DETECT result set,
// keeping SIM and any manually-added devices. Auto-selects the first found.
function applyDetected(found) {
  SENSORS = SENSORS.filter((s) => s.kind === 'sim' || s.kind === 'manual');
  for (const d of found) {
    const id = devIdForPath(d.path);
    const dev = {
      id, name: d.name, kind: 'detected',
      range: d.baudrate >= 1000000 ? '30m' : d.baudrate >= 256000 ? '25m' : '12m', hz: '10',
      firmware: d.firmware,
      cfg: Object.assign({}, DEFAULT_CFG, { comPort: d.path, baudrate: String(d.baudrate) }),
    };
    if (SENSORS.find((s) => s.id === id)) { // already present — refresh its name/cfg
      const ex = SENSORS.find((s) => s.id === id); ex.name = dev.name; ex.firmware = dev.firmware;
      Object.assign(cfgs[id], { comPort: d.path, baudrate: String(d.baudrate) });
    } else {
      SENSORS.push(dev);
      cfgs[id] = Object.assign({}, dev.cfg);
    }
  }
  if (found.length) selectDevice(devIdForPath(found[0].path));
  else renderDevices();
}

// Add a blank serial device for the user to fill in (manual port / network LiDAR).
function addManualDevice() {
  const id = 'man_' + (++manualSeq);
  const dev = { id, name: 'LiDAR ' + manualSeq, kind: 'manual', range: '30m', hz: '10', cfg: Object.assign({}, DEFAULT_CFG) };
  SENSORS.push(dev);
  cfgs[id] = Object.assign({}, dev.cfg);
  selectDevice(id);
}

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
let tracks = [];
const trails = new Map(); // id -> [[x,y], ...] motion trail
let zones = []; // [{ name, slug, pts:[[x,y]], visible, occupied }]
let draft = []; // world points of the zone being drawn (polygon, click-by-click)
let draftRect = null; // [[x0,y0],[x1,y1]] live rectangle preview while dragging
let selectedZone = -1; // index of zone being edited (Select tool)
let zoneBbox = null; // screen-space resize handles of the selected zone

function pipR(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
    if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// ---- homography (mirrors main/homography.js) ------------------------------
function hSolve(A, b, n) {
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]]; [b[col], b[piv]] = [b[piv], b[col]];
    const d = A[col][col] || 1e-12;
    for (let r = 0; r < n; r++) { if (r === col) continue; const f = A[r][col] / d; for (let c = col; c < n; c++) A[r][c] -= f * A[col][c]; b[r] -= f * b[col]; }
  }
  const x = new Array(n);
  for (let i = 0; i < n; i++) x[i] = b[i] / (A[i][i] || 1e-12);
  return x;
}
function computeH(src, dst) {
  const D = dst || [[0, 0], [1, 0], [1, 1], [0, 1]];
  const A = [], b = [];
  for (let i = 0; i < 4; i++) {
    const [x, y] = src[i], [u, v] = D[i];
    A.push([x, y, 1, 0, 0, 0, -x * u, -y * u]); b.push(u);
    A.push([0, 0, 0, x, y, 1, -x * v, -y * v]); b.push(v);
  }
  const h = hSolve(A, b, 8); h.push(1); return h;
}
function applyH(H, x, y) { const d = H[6] * x + H[7] * y + H[8] || 1e-12; return [(H[0] * x + H[1] * y + H[2]) / d, (H[3] * x + H[4] * y + H[5]) / d]; }
function invert3(H) {
  const [a, b, c, d, e, f, g, h, i] = H;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C || 1e-12, id = 1 / det;
  return [A * id, (c * h - b * i) * id, (b * f - c * e) * id, B * id, (a * i - c * g) * id, (c * d - a * f) * id, C * id, (b * g - a * h) * id, (a * e - b * d) * id];
}

// ---- warp state -----------------------------------------------------------
const CORNER_TAGS = ['TL', 'TR', 'BR', 'BL'];
const WARP_TARGETS = ['0,0', '1,0', '1,1', '0,1'];
const warp = {
  corners: [[-3, 5], [3, 5], [3, 0.5], [-3, 0.5]],
  enabled: false, H: null, Hinv: null,
  sel: [], rotStep: 15, undo: [], redo: [], marquee: null,
};
function recomputeWarp() { warp.H = computeH(warp.corners); warp.Hinv = invert3(warp.H); }
recomputeWarp();

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

  tracks = frame.tracks || [];
  updateTrails(tracks);
  renderTrackTable(tracks);

  if (frame.zoneInfo) {
    for (let i = 0; i < zones.length && i < frame.zoneInfo.length; i++) {
      const info = frame.zoneInfo[i];
      zones[i].occupied = info.on; zones[i].count = info.count; zones[i].dwell = info.dwell;
    }
    updateZoneBadges();
  }
}

// ---- zones ----------------------------------------------------------------
const slugify = (s) => s.toLowerCase().trim().replace(/\s+/g, '_').replace(/[^a-z0-9_]/g, '');

function pushZones() {
  window.lidar.setZones(zones.map((z) => ({ name: z.name, slug: z.slug, pts: z.pts })));
}

function addZone(pts) {
  const name = 'Zone ' + (zones.length + 1);
  zones.push({ name, slug: slugify(name), pts, visible: true, occupied: false });
  pushZones();
  renderZoneCards();
}

function renderZoneCards() {
  const list = $('zoneList');
  list.innerHTML = '';
  zones.forEach((z, i) => {
    const card = document.createElement('div');
    const sel = i === selectedZone;
    const border = sel ? '#00e5ff' : (z.occupied ? 'rgba(0,229,255,0.4)' : 'rgba(255,255,255,0.07)');
    card.style.cssText = `padding:10px 11px;border-radius:8px;background:#10141a;border:1px solid ${border};display:flex;flex-direction:column;gap:8px;cursor:pointer`;
    card.onclick = (e) => { if (e.target.tagName !== 'INPUT' && e.target.tagName !== 'BUTTON') { selectedZone = i; renderZoneCards(); } };
    card.innerHTML =
      `<div style="display:flex;align-items:center;gap:9px">
         <button class="zone-eye" data-i="${i}" title="visibility" style="width:22px;height:22px;border-radius:5px;border:1px solid rgba(255,255,255,0.1);background:${z.visible ? 'rgba(0,229,255,0.12)' : '#0e1216'};color:${z.visible ? '#a9e8f1' : '#5b636d'};cursor:pointer;font-size:11px;padding:0;display:flex;align-items:center;justify-content:center;flex:0 0 auto">${z.visible ? '◉' : '○'}</button>
         <input class="zone-name" data-i="${i}" value="${z.name}" style="flex:1;min-width:0;background:transparent;border:none;border-bottom:1px solid rgba(255,255,255,0.08);font-weight:600;font-size:12.5px;color:#dbe1e8;font-family:inherit;padding:2px 0">
         <span class="zone-badge" style="font-family:'IBM Plex Mono',monospace;font-size:10px;padding:3px 8px;border-radius:20px;background:${z.occupied ? 'rgba(0,229,255,0.18)' : 'rgba(255,255,255,0.05)'};color:${z.occupied ? '#bff3fb' : '#717a84'}">${z.occupied ? 'TRIGGERED' : 'IDLE'}</span>
         <button class="zone-del" data-i="${i}" title="delete" style="width:22px;height:22px;border-radius:5px;border:1px solid rgba(255,77,94,0.3);background:rgba(255,77,94,0.07);color:#ff9aa6;cursor:pointer;font-size:13px;padding:0;flex:0 0 auto">×</button>
       </div>
       <div style="display:flex;justify-content:space-between;font-family:'IBM Plex Mono',monospace;font-size:10px;color:#5b636d">
         <span>${z.pts.length} vertices</span><span style="color:#717a84">/lidar/zone/${z.slug}</span>
       </div>`;
    list.appendChild(card);
  });
  list.querySelectorAll('.zone-eye').forEach((b) => { b.onclick = () => { const i = +b.getAttribute('data-i'); zones[i].visible = !zones[i].visible; renderZoneCards(); }; });
  list.querySelectorAll('.zone-del').forEach((b) => { b.onclick = () => deleteZone(+b.getAttribute('data-i')); });
  list.querySelectorAll('.zone-name').forEach((inp) => {
    inp.onchange = () => {
      const i = +inp.getAttribute('data-i');
      zones[i].name = inp.value.trim() || zones[i].name;
      zones[i].slug = slugify(zones[i].name);
      pushZones(); renderZoneCards();
    };
  });
  $('zoneEmpty').style.display = zones.length ? 'none' : 'block';
}

function deleteZone(i) {
  zones.splice(i, 1);
  if (selectedZone === i) selectedZone = -1;
  else if (selectedZone > i) selectedZone--;
  pushZones(); renderZoneCards();
}

function updateZoneBadges() {
  const cards = $('zoneList').children;
  for (let i = 0; i < zones.length && i < cards.length; i++) {
    const badge = cards[i].querySelector('.zone-badge');
    if (!badge) continue;
    const o = zones[i].occupied;
    const c = zones[i].count || 0;
    badge.textContent = o ? ('TRIGGERED · ' + c) : 'IDLE';
    badge.style.background = o ? 'rgba(0,229,255,0.18)' : 'rgba(255,255,255,0.05)';
    badge.style.color = o ? '#bff3fb' : '#717a84';
    if (i !== selectedZone) cards[i].style.borderColor = o ? 'rgba(0,229,255,0.4)' : 'rgba(255,255,255,0.07)';
  }
}

function setTool(tool) {
  ui.tool = tool;
  document.querySelectorAll('[data-tool]').forEach((b) => b.classList.toggle('active', b.getAttribute('data-tool') === tool));
  draftRect = null;
  if (tool === 'zone') { draft = []; $('draftBanner').style.display = 'flex'; $('draftCount').textContent = '0 pts'; }
  else { draft = []; $('draftBanner').style.display = 'none'; }
}

function finishZone() {
  if (draft.length >= 3) addZone(draft.slice());
  setTool('select');
}
function cancelDraft() { setTool('select'); }

// ---- warp overlay + interaction -------------------------------------------
function warpInteractive() { return ui.tool === 'warp' || ui.tab === 'warp'; }

function drawWarpOverlay(s) {
  const showWarp = warpInteractive() || warp.enabled;
  if (!showWarp || !warp.Hinv) return;
  const c = warp.corners, Hi = warp.Hinv, on = warp.enabled, interactive = warpInteractive();
  const mapU = (u, v) => { const d = Hi[6] * u + Hi[7] * v + Hi[8]; return [(Hi[0] * u + Hi[1] * v + Hi[2]) / d, (Hi[3] * u + Hi[4] * v + Hi[5]) / d]; };
  // fill
  ctx.beginPath(); c.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); }); ctx.closePath();
  ctx.fillStyle = on ? 'rgba(0,229,255,0.07)' : 'rgba(139,160,170,0.045)'; ctx.fill();
  // perspective grid
  const N = 8; ctx.lineWidth = 1; ctx.strokeStyle = on ? 'rgba(0,229,255,0.26)' : 'rgba(139,160,170,0.2)';
  for (let i = 1; i < N; i++) {
    const u = i / N;
    const a = mapU(u, 0), b = mapU(u, 1); const pa = wts(a[0], a[1]), pb = wts(b[0], b[1]);
    ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
    const cc = mapU(0, u), dd = mapU(1, u); const pc = wts(cc[0], cc[1]), pd = wts(dd[0], dd[1]);
    ctx.beginPath(); ctx.moveTo(pc[0], pc[1]); ctx.lineTo(pd[0], pd[1]); ctx.stroke();
  }
  // outline
  ctx.beginPath(); c.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); }); ctx.closePath();
  ctx.lineWidth = 2; ctx.strokeStyle = on ? '#00e5ff' : '#8fa3ad'; ctx.stroke();
  // corner handles
  c.forEach((p, i) => {
    const [sx, sy] = wts(p[0], p[1]);
    const isSel = warp.sel.indexOf(i) >= 0;
    const r = interactive ? (isSel ? 8 : 6) : 5;
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI);
    ctx.fillStyle = isSel ? '#ffffff' : (on ? '#00e5ff' : '#8fa3ad');
    ctx.shadowColor = (on || isSel) ? '#00e5ff' : 'transparent'; ctx.shadowBlur = isSel ? 13 : (interactive ? 8 : 0); ctx.fill(); ctx.shadowBlur = 0;
    if (isSel) { ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2; ctx.stroke(); }
    else { ctx.fillStyle = '#06181c'; ctx.beginPath(); ctx.arc(sx, sy, 2.4, 0, 2 * Math.PI); ctx.fill(); }
    if (interactive) {
      ctx.font = "600 10px 'IBM Plex Mono', monospace"; ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
      ctx.fillStyle = isSel ? '#ffffff' : (on ? '#bff3fb' : '#b9c6cd'); ctx.fillText('(' + WARP_TARGETS[i] + ')', sx, sy - 14);
      // live world-coordinate readout under each handle (helps align to wall corners)
      ctx.font = "500 9px 'IBM Plex Mono', monospace";
      ctx.fillStyle = isSel ? '#ffffff' : 'rgba(0,229,255,0.7)';
      ctx.fillText(p[0].toFixed(2) + ', ' + p[1].toFixed(2) + ' m', sx, sy + 22);
    }
  });
  // bounding box + scale handles (when >=2 corners selected)
  if (interactive && warp.sel.length >= 2) {
    const xs = warp.sel.map((i) => c[i][0]), ys = warp.sel.map((i) => c[i][1]);
    const bx0 = Math.min.apply(null, xs), bx1 = Math.max.apply(null, xs), by0 = Math.min.apply(null, ys), by1 = Math.max.apply(null, ys);
    const bcw = [[bx0, by0], [bx1, by0], [bx1, by1], [bx0, by1]];
    const q = bcw.map((p) => wts(p[0], p[1]));
    ctx.setLineDash([4, 3]); ctx.strokeStyle = 'rgba(255,255,255,0.55)'; ctx.lineWidth = 1;
    ctx.beginPath(); q.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    const cenx = (q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4, ceny = (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4;
    const OFF = 16;
    warp._bbox = bcw.map((cw, i) => {
      let hx = q[i][0], hy = q[i][1]; const vx = hx - cenx, vy = hy - ceny, L = Math.hypot(vx, vy) || 1;
      hx += (vx / L) * OFF; hy += (vy / L) * OFF;
      const aw = [cw[0] === bx0 ? bx1 : bx0, cw[1] === by0 ? by1 : by0]; // opposite corner (world)
      return { sx: hx, sy: hy, cw, aw };
    });
    for (const h of warp._bbox) { ctx.fillStyle = '#fff'; ctx.fillRect(h.sx - 4, h.sy - 4, 8, 8); ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 1.5; ctx.strokeRect(h.sx - 4, h.sy - 4, 8, 8); }
  } else { warp._bbox = null; }

  // snap indicator (green ring on the wall point a corner is locking onto)
  if (warp._snap) {
    const [sx, sy] = wts(warp._snap[0], warp._snap[1]);
    ctx.strokeStyle = '#39ff7a'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(sx, sy, 12, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = '#39ff7a';
    ctx.font = "600 9px 'IBM Plex Mono', monospace"; ctx.textAlign = 'center';
    ctx.fillText('SNAP', sx, sy - 16);
  }

  // marquee
  if (warp.marquee) {
    const m = warp.marquee, x = Math.min(m.x0, m.x1), y = Math.min(m.y0, m.y1), w = Math.abs(m.x1 - m.x0), h = Math.abs(m.y1 - m.y0);
    ctx.fillStyle = 'rgba(0,229,255,0.08)'; ctx.fillRect(x, y, w, h);
    ctx.setLineDash([5, 4]); ctx.strokeStyle = 'rgba(0,229,255,0.9)'; ctx.lineWidth = 1; ctx.strokeRect(x, y, w, h); ctx.setLineDash([]);
  }
}

function pushWarp() { recomputeWarp(); window.lidar.setWarp({ corners: warp.corners, enabled: warp.enabled }); renderCornerInputs(); }
// live update while dragging — push corners to main each move (no DOM rebuild) so the
// output monitor / projector reposition tracked points in real time
function pushWarpLive() { recomputeWarp(); window.lidar.setWarp({ corners: warp.corners, enabled: warp.enabled }); }
function warpSnapshot() { warp.undo.push(JSON.stringify(warp.corners)); if (warp.undo.length > 80) warp.undo.shift(); warp.redo = []; updateWarpButtons(); }
function setWarpEnabled(on) {
  warp.enabled = on;
  $('warpKnob').style.left = on ? '24px' : '2px';
  $('warpKnob').style.background = on ? '#00e5ff' : '#717a84';
  $('warpToggle').style.borderColor = on ? 'rgba(0,229,255,0.5)' : 'rgba(255,255,255,0.12)';
  $('warpToggle').style.background = on ? 'rgba(0,229,255,0.12)' : '#0e1216';
  pushWarp();
}
function updateWarpSel() {
  $('warpSel').textContent = warp.sel.length ? warp.sel.length + ' selected' : 'no selection';
  $('warpSel').style.color = warp.sel.length ? '#9fe4ef' : '#5b636d';
}
function updateWarpButtons() {
  $('warpUndo').style.color = warp.undo.length ? '#cdd4dc' : '#454c54';
  $('warpRedo').style.color = warp.redo.length ? '#cdd4dc' : '#454c54';
}
function renderCornerInputs() {
  const box = $('warpCorners');
  box.innerHTML = '';
  warp.corners.forEach((p, i) => {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:8px';
    row.innerHTML =
      `<span style="width:48px;display:flex;flex-direction:column;line-height:1.15;flex:0 0 auto"><span class="mono" style="font-size:11.5px;color:#cdd4dc;font-weight:600">${CORNER_TAGS[i]}</span><span class="mono" style="font-size:8.5px;color:#00e5ff">→ ${WARP_TARGETS[i]}</span></span>
       <input class="input wc-x" data-i="${i}" value="${p[0].toFixed(2)}" style="flex:1;min-width:0">
       <input class="input wc-y" data-i="${i}" value="${p[1].toFixed(2)}" style="flex:1;min-width:0">`;
    box.appendChild(row);
  });
  box.querySelectorAll('.wc-x').forEach((inp) => inp.onchange = () => { const i = +inp.getAttribute('data-i'); warpSnapshot(); warp.corners[i][0] = parseFloat(inp.value) || 0; pushWarp(); });
  box.querySelectorAll('.wc-y').forEach((inp) => inp.onchange = () => { const i = +inp.getAttribute('data-i'); warpSnapshot(); warp.corners[i][1] = parseFloat(inp.value) || 0; pushWarp(); });
}
function warpGroupCenter(idxs) {
  const ix = idxs.length ? idxs : [0, 1, 2, 3];
  let cx = 0, cy = 0; for (const i of ix) { cx += warp.corners[i][0]; cy += warp.corners[i][1]; }
  return [cx / ix.length, cy / ix.length];
}
function warpRotate(deg) {
  const idxs = warp.sel.length ? warp.sel : [0, 1, 2, 3];
  const [cx, cy] = warpGroupCenter(idxs);
  const a = (deg * Math.PI) / 180, ca = Math.cos(a), sa = Math.sin(a);
  warpSnapshot();
  for (const i of idxs) { const dx = warp.corners[i][0] - cx, dy = warp.corners[i][1] - cy; warp.corners[i] = [cx + dx * ca - dy * sa, cy + dx * sa + dy * ca]; }
  pushWarp();
}
function warpFlip(horizontal) {
  const idxs = warp.sel.length ? warp.sel : [0, 1, 2, 3];
  const [cx, cy] = warpGroupCenter(idxs);
  warpSnapshot();
  for (const i of idxs) { if (horizontal) warp.corners[i][0] = 2 * cx - warp.corners[i][0]; else warp.corners[i][1] = 2 * cy - warp.corners[i][1]; }
  pushWarp();
}
// Nearest point-cloud point (a wall/object return) to (px,py) within maxD metres.
function findNearestCloud(px, py, maxD = 0.4) {
  let best = null, bestD = maxD;
  for (const b of bins) {
    if (!b) continue;
    const d = Math.hypot(b.x - px, b.y - py);
    if (d < bestD) { bestD = d; best = [b.x, b.y]; }
  }
  return best;
}

function warpUndo() { if (!warp.undo.length) return; warp.redo.push(JSON.stringify(warp.corners)); warp.corners = JSON.parse(warp.undo.pop()); pushWarp(); updateWarpButtons(); }
function warpRedo() { if (!warp.redo.length) return; warp.undo.push(JSON.stringify(warp.corners)); warp.corners = JSON.parse(warp.redo.pop()); pushWarp(); updateWarpButtons(); }

// ---- visual output (window / extended / NDI) ------------------------------
let outputMode = 'window';
let projectorOpen = false;
let ndiOn = false;
let syphonOn = false;
const ndiCfg = { name: 'LidarBridge-Mapping', w: '1280', h: '720', fps: '30', fit: 'fit' };

function buildNdiConfig() {
  const box = $('ndiConfig');
  box.innerHTML =
    `<label class="field"><span class="lbl">SOURCE NAME</span><input id="ndiName" class="input" value="${ndiCfg.name}"></label>
     <div class="row">
       <label class="field" style="flex:1"><span class="lbl">WIDTH</span><input id="ndiW" class="input" value="${ndiCfg.w}"></label>
       <label class="field" style="flex:1"><span class="lbl">HEIGHT</span><input id="ndiH" class="input" value="${ndiCfg.h}"></label>
       <label class="field" style="flex:1"><span class="lbl">FPS</span>
         <div class="row"><button class="seg ndi-fps" data-fps="60" style="font-size:9.5px">60</button><button class="seg ndi-fps active" data-fps="30" style="font-size:9.5px">30</button></div>
       </label>
     </div>
     <div style="display:flex;justify-content:space-between" class="mono"><span class="lbl">ASPECT</span><span id="ndiAspect" style="font-size:9.5px;color:#9fe4ef"></span></div>
     <div class="row">
       <button class="seg ndi-presets" data-preset="1280x720" style="font-size:9px">720p</button>
       <button class="seg ndi-presets" data-preset="1920x1080" style="font-size:9px">1080p</button>
       <button class="seg ndi-presets" data-preset="2560x1440" style="font-size:9px">1440p</button>
       <button class="seg ndi-presets" data-preset="3840x2160" style="font-size:9px">4K</button>
     </div>
     <div class="row">
       <button class="seg ndi-fit active" data-fit="fit" style="font-size:9.5px">FIT</button>
       <button class="seg ndi-fit" data-fit="fill" style="font-size:9.5px">FILL</button>
       <button class="seg ndi-fit" data-fit="stretch" style="font-size:9.5px">STRETCH</button>
     </div>`;
  const updAspect = () => {
    const w = parseInt($('ndiW').value, 10) || 1, h = parseInt($('ndiH').value, 10) || 1;
    const g = (a, b) => (b ? g(b, a % b) : a); const k = g(w, h) || 1;
    $('ndiAspect').textContent = `${w} × ${h}  (${w / k}:${h / k})`;
  };
  $('ndiName').onchange = () => (ndiCfg.name = $('ndiName').value);
  $('ndiW').oninput = () => { ndiCfg.w = $('ndiW').value; updAspect(); };
  $('ndiH').oninput = () => { ndiCfg.h = $('ndiH').value; updAspect(); };
  box.querySelectorAll('.ndi-presets').forEach((b) => b.onclick = () => {
    const [w, h] = b.getAttribute('data-preset').split('x');
    $('ndiW').value = w; $('ndiH').value = h; ndiCfg.w = w; ndiCfg.h = h; updAspect();
  });
  box.querySelectorAll('.ndi-fps').forEach((b) => b.onclick = () => { ndiCfg.fps = b.getAttribute('data-fps'); box.querySelectorAll('.ndi-fps').forEach((x) => x.classList.toggle('active', x === b)); });
  box.querySelectorAll('.ndi-fit').forEach((b) => b.onclick = () => { ndiCfg.fit = b.getAttribute('data-fit'); box.querySelectorAll('.ndi-fit').forEach((x) => x.classList.toggle('active', x === b)); });
  updAspect();
}

function updateOutputAction() {
  const btn = $('outputAction');
  if (outputMode === 'ndi') { btn.textContent = ndiOn ? 'STOP NDI STREAM' : 'START NDI STREAM'; }
  else if (outputMode === 'syphon') { btn.textContent = syphonOn ? 'STOP SYPHON' : 'START SYPHON'; }
  else if (outputMode === 'extended') { btn.textContent = projectorOpen ? 'CLOSE OUTPUT' : 'OPEN ON EXTENDED DISPLAY'; }
  else { btn.textContent = projectorOpen ? 'CLOSE OUTPUT' : 'OPEN OUTPUT WINDOW'; }
}

async function doOutputAction() {
  if (outputMode === 'ndi') {
    if (ndiOn) { await window.lidar.ndiStop(); ndiOn = false; $('ndiBadge').style.display = 'none'; }
    else {
      const res = await window.lidar.ndiStart(ndiCfg);
      if (res.ok) { ndiOn = true; $('ndiBadge').style.display = 'flex'; }
      else { setConnStatus(res.error, '#ffb000'); }
    }
    updateOutputAction();
    return;
  }
  if (outputMode === 'syphon') {
    if (syphonOn) { await window.lidar.syphonStop(); syphonOn = false; $('ndiBadge').style.display = 'none'; }
    else {
      const res = await window.lidar.syphonStart(ndiCfg);
      if (res.ok) { syphonOn = true; $('ndiBadge').style.display = 'flex'; setConnStatus('Syphon server "' + ndiCfg.name + '" started', '#39ff7a'); }
      else { setConnStatus(res.error, '#ffb000'); }
    }
    updateOutputAction();
    return;
  }
  if (projectorOpen) { await window.lidar.closeOutput(); }
  else { await window.lidar.openOutput(outputMode); }
}

// ---- output (OSC / TUIO) --------------------------------------------------
const out = { protocol: 'osc', host: '127.0.0.1', port: '7000', sendRate: '30', normalize: false, format: 'slots' };

function pushOutput() {
  window.lidar.setOutput(out);
  updatePill();
}
function updatePill() {
  $('protoName').textContent = out.protocol.toUpperCase();
  $('protoAddr').textContent = '→ ' + out.host + ':' + out.port;
}
function updateOscPreview() {
  const pre = $('oscPreview');
  const c = out.normalize ? 'u  v' : 'x  y';
  if (out.protocol === 'tuio') pre.textContent = '/tuio/2Dobj set <s> <c> ' + (out.normalize ? 'u v' : 'x y') + ' …\n/tuio/2Dobj alive […]  +  fseq <n>';
  else pre.textContent = '/lidar/count <n> · /lidar/pN/{on,x,y,v,id}\n/lidar/zone/<slug>  0|1 · /count · /dwell · /enter · /exit';
}
function appendOscLog(lines) {
  const mon = $('oscMonitor');
  for (const l of lines) {
    const d = document.createElement('div');
    d.textContent = l;
    if (l.includes('/zone/')) d.style.color = '#00e5ff';
    mon.appendChild(d);
  }
  while (mon.childElementCount > 80) mon.removeChild(mon.firstChild);
  mon.scrollTop = mon.scrollHeight;
  $('oscRateLabel').textContent = out.sendRate + ' Hz';
}

function updateTrails(ts) {
  const live = new Set(ts.map((t) => t.id));
  for (const id of trails.keys()) if (!live.has(id)) trails.delete(id);
  for (const t of ts) {
    let tr = trails.get(t.id);
    if (!tr) { tr = []; trails.set(t.id, tr); }
    const last = tr[tr.length - 1];
    if (!last || Math.hypot(last[0] - t.x, last[1] - t.y) > 0.03) tr.push([t.x, t.y]);
    if (tr.length > 16) tr.shift();
  }
}

function renderTrackTable(ts) {
  const list = $('trackList');
  list.innerHTML = '';
  for (const t of ts) {
    const row = document.createElement('div');
    row.style.cssText =
      "display:flex;align-items:center;padding:8px 6px;border-radius:6px;background:#10141a;border:1px solid rgba(255,255,255,0.05);font-family:'IBM Plex Mono',monospace;font-size:11px";
    const zoneColor = t.zone ? '#00e5ff' : '#5b636d';
    row.innerHTML =
      `<span style="width:44px;display:flex;align-items:center;gap:6px"><span style="width:8px;height:8px;border-radius:50%;background:${t.color};box-shadow:0 0 6px ${t.color}"></span><span style="color:#cdd4dc">${t.id}</span></span>
       <span style="flex:1;color:#9aa3ad">${t.x.toFixed(2)} <span style="color:#4d555e">/</span> ${t.y.toFixed(2)}</span>
       <span style="width:46px;text-align:right;color:#9aa3ad">${t.vel.toFixed(2)}</span>
       <span style="width:74px;text-align:right;color:${zoneColor}">${t.zone || '—'}</span>`;
    list.appendChild(row);
  }
  $('trackedCount').textContent = ts.length;
  $('trackEmpty').style.display = ts.length ? 'none' : 'block';
}

// ---- stats ----------------------------------------------------------------
const liveStats = { points: 0, latency: 0 };
let frames = 0, fpsAccum = 0, fps = 0;

// ---- draw -----------------------------------------------------------------
let sweep = 0;
let lastFrame = performance.now();
let lastDrawAt = 0; // render FPS cap (saves CPU/GPU vs 120Hz ProMotion)

function draw() {
  requestAnimationFrame(draw);
  const cap = ui.streaming ? 60 : 30; // full rate when live, half when paused
  const t0 = performance.now();
  if (t0 - lastDrawAt < 1000 / cap - 0.5) return;
  lastDrawAt = t0;
  const now = t0;
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

  // trigger zones (filled cyan, brighter when occupied)
  for (const z of zones) {
    if (!z.visible) continue;
    ctx.beginPath();
    z.pts.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.closePath();
    ctx.fillStyle = z.occupied ? 'rgba(0,229,255,0.16)' : 'rgba(0,229,255,0.05)';
    ctx.fill();
    ctx.lineWidth = z.occupied ? 1.8 : 1.2;
    ctx.strokeStyle = z.occupied ? 'rgba(0,229,255,0.9)' : 'rgba(0,229,255,0.4)';
    ctx.stroke();
    const c = z.pts.reduce((a, p) => [a[0] + p[0], a[1] + p[1]], [0, 0]).map((v) => v / z.pts.length);
    const [cx, cy] = wts(c[0], c[1]);
    ctx.fillStyle = z.occupied ? '#bff3fb' : 'rgba(0,229,255,0.55)';
    ctx.font = "600 11px 'IBM Plex Mono', monospace";
    ctx.textAlign = 'center';
    ctx.fillText(z.name.toUpperCase(), cx, cy);
  }

  // selected zone: highlight + bounding-box resize handles (Select tool)
  zoneBbox = null;
  if (selectedZone >= 0 && zones[selectedZone] && ui.tool === 'select') {
    const z = zones[selectedZone];
    ctx.beginPath();
    z.pts.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.closePath();
    ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2; ctx.stroke();
    const xs = z.pts.map((p) => p[0]), ys = z.pts.map((p) => p[1]);
    const bx0 = Math.min.apply(null, xs), bx1 = Math.max.apply(null, xs);
    const by0 = Math.min.apply(null, ys), by1 = Math.max.apply(null, ys);
    const bcw = [[bx0, by0], [bx1, by0], [bx1, by1], [bx0, by1]];
    const q = bcw.map((p) => wts(p[0], p[1]));
    ctx.setLineDash([4, 3]); ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
    ctx.beginPath(); q.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    const cenx = (q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4, ceny = (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4;
    const OFF = 14;
    zoneBbox = bcw.map((cw, i) => {
      let hx = q[i][0], hy = q[i][1]; const vx = hx - cenx, vy = hy - ceny, L = Math.hypot(vx, vy) || 1;
      hx += (vx / L) * OFF; hy += (vy / L) * OFF;
      const aw = [cw[0] === bx0 ? bx1 : bx0, cw[1] === by0 ? by1 : by0];
      return { sx: hx, sy: hy, cw, aw };
    });
    for (const h of zoneBbox) { ctx.fillStyle = '#fff'; ctx.fillRect(h.sx - 4, h.sy - 4, 8, 8); ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 1.5; ctx.strokeRect(h.sx - 4, h.sy - 4, 8, 8); }
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

  // tracked blobs (ring + crosshair + velocity arrow + trail + label)
  for (const t of tracks) {
    const col = t.color || '#00e5ff';
    const tr = trails.get(t.id);
    if (tr && tr.length > 1) {
      ctx.strokeStyle = col + '33';
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      tr.forEach((h, i) => { const [hx, hy] = wts(h[0], h[1]); i ? ctx.lineTo(hx, hy) : ctx.moveTo(hx, hy); });
      ctx.stroke();
    }
    const [bx, by] = wts(t.x, t.y);
    ctx.beginPath();
    ctx.arc(bx, by, 0.32 * s, 0, 2 * Math.PI);
    ctx.strokeStyle = col;
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.fillStyle = col + '22';
    ctx.fill();
    ctx.strokeStyle = col + '99';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(bx - 0.32 * s, by); ctx.lineTo(bx + 0.32 * s, by);
    ctx.moveTo(bx, by - 0.32 * s); ctx.lineTo(bx, by + 0.32 * s);
    ctx.stroke();
    ctx.fillStyle = col;
    ctx.beginPath(); ctx.arc(bx, by, 3, 0, 2 * Math.PI); ctx.fill();
    const v = Math.hypot(t.vx, t.vy);
    if (v > 0.05) {
      const ux = t.vx / v, uy = t.vy / v;
      const [ax, ay] = wts(t.x + ux * 0.7, t.y + uy * 0.7);
      ctx.strokeStyle = col; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ax, ay); ctx.stroke();
    }
    const label = t.id + '  ' + t.x.toFixed(2) + ',' + t.y.toFixed(2);
    ctx.font = "500 10px 'IBM Plex Mono', monospace";
    ctx.textAlign = 'left';
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(10,12,16,0.82)';
    ctx.fillRect(bx + 0.36 * s, by - 0.5 * s - 7, tw + 10, 15);
    ctx.fillStyle = col;
    ctx.fillRect(bx + 0.36 * s, by - 0.5 * s - 7, 2.5, 15);
    ctx.fillStyle = '#d6dde4';
    ctx.fillText(label, bx + 0.36 * s + 7, by - 0.5 * s + 2.5);
  }

  // rectangle zone being dragged (live preview)
  if (draftRect) {
    const [a, b] = draftRect;
    const p0 = wts(a[0], a[1]), p1 = wts(b[0], b[1]);
    const x = Math.min(p0[0], p1[0]), y = Math.min(p0[1], p1[1]);
    const w = Math.abs(p1[0] - p0[0]), h = Math.abs(p1[1] - p0[1]);
    ctx.fillStyle = 'rgba(0,229,255,0.08)'; ctx.fillRect(x, y, w, h);
    ctx.setLineDash([5, 4]); ctx.strokeStyle = 'rgba(0,229,255,0.9)'; ctx.lineWidth = 1.5;
    ctx.strokeRect(x, y, w, h); ctx.setLineDash([]);
  }

  // zone being drawn (dashed)
  if (draft.length) {
    ctx.strokeStyle = 'rgba(0,229,255,0.9)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    draft.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of draft) { const [sx, sy] = wts(p[0], p[1]); ctx.fillStyle = '#00e5ff'; ctx.beginPath(); ctx.arc(sx, sy, 3.5, 0, 2 * Math.PI); ctx.fill(); }
  }

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

  drawWarpOverlay(s);

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

  drawMonitor();
}

// Output monitor: the mapping in normalized 0–1 space (what TouchDesigner receives).
function drawMonitor() {
  const mc = $('monitor');
  if (!mc || ui.tab !== 'warp') return; // only render when visible
  const mctx = mc.__ctx || (mc.__ctx = mc.getContext('2d'));
  const w = mc.clientWidth, h = mc.clientHeight, d = Math.min(window.devicePixelRatio || 1, 2);
  if (mc.width !== w * d || mc.height !== h * d) { mc.width = w * d; mc.height = h * d; }
  mctx.setTransform(d, 0, 0, d, 0, 0);
  mctx.fillStyle = '#000'; mctx.fillRect(0, 0, w, h);
  const pad = 14, GW = w - 2 * pad, GH = h - 2 * pad;
  const m = (u, v) => [pad + u * GW, pad + v * GH];
  // 0–1 grid
  mctx.strokeStyle = 'rgba(255,255,255,0.08)'; mctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    let a = m(i / 4, 0), b = m(i / 4, 1); mctx.beginPath(); mctx.moveTo(a[0], a[1]); mctx.lineTo(b[0], b[1]); mctx.stroke();
    a = m(0, i / 4); b = m(1, i / 4); mctx.beginPath(); mctx.moveTo(a[0], a[1]); mctx.lineTo(b[0], b[1]); mctx.stroke();
  }
  mctx.strokeStyle = 'rgba(0,229,255,0.5)'; mctx.lineWidth = 1.5; mctx.strokeRect(pad, pad, GW, GH);
  // registration corner targets
  [[0, 0], [1, 0], [1, 1], [0, 1]].forEach((c) => {
    const [x, y] = m(c[0], c[1]);
    mctx.strokeStyle = '#00e5ff'; mctx.lineWidth = 1.2;
    mctx.beginPath(); mctx.arc(x, y, 5, 0, 2 * Math.PI); mctx.stroke();
    mctx.beginPath(); mctx.moveTo(x - 7, y); mctx.lineTo(x + 7, y); mctx.moveTo(x, y - 7); mctx.lineTo(x, y + 7); mctx.stroke();
  });
  const [cx, cy] = m(0.5, 0.5);
  mctx.strokeStyle = 'rgba(255,255,255,0.25)';
  mctx.beginPath(); mctx.moveTo(cx - 6, cy); mctx.lineTo(cx + 6, cy); mctx.moveTo(cx, cy - 6); mctx.lineTo(cx, cy + 6); mctx.stroke();
  // zones through H
  for (const z of zones) {
    if (!z.visible) continue;
    mctx.beginPath();
    z.pts.forEach((p, i) => { const [u, v] = applyH(warp.H, p[0], p[1]); const [x, y] = m(u, v); i ? mctx.lineTo(x, y) : mctx.moveTo(x, y); });
    mctx.closePath();
    mctx.fillStyle = z.occupied ? 'rgba(0,229,255,0.18)' : 'rgba(0,229,255,0.06)'; mctx.fill();
    mctx.strokeStyle = 'rgba(0,229,255,0.6)'; mctx.lineWidth = 1; mctx.stroke();
  }
  // tracked points (normalized), clipped to the mapped area
  for (const t of tracks) {
    if (t.u == null || t.u < 0 || t.u > 1 || t.v < 0 || t.v > 1) continue;
    const [x, y] = m(t.u, t.v);
    mctx.fillStyle = t.color || '#00e5ff';
    mctx.beginPath(); mctx.arc(x, y, 4, 0, 2 * Math.PI); mctx.fill();
  }
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
    const dotColor = isOnline ? '#39ff7a' : s.kind === 'sim' ? '#717a84' : '#ff4d5e';
    const c = cfgs[s.id];
    const addr = s.kind === 'sim' ? 'built-in' : c.connType === 'serial' ? (c.comPort || '—') : (c.ipAddr || 'NET');
    const conn = s.kind === 'sim' ? 'SIM' : c.connType === 'serial' ? 'SERIAL' : 'NET';
    const meta = s.kind === 'detected'
      ? `<span>${conn} ${addr}</span><span>baud ${c.baudrate}</span>`
      : `<span>${conn} ${addr}</span>`;
    const sub = s.kind === 'sim'
      ? `<span>built-in simulator</span>`
      : `<span>RANGE ${s.range}</span><span>${s.hz} Hz</span>` + (s.firmware ? `<span>fw ${s.firmware}</span>` : '');
    const delBtn = s.kind === 'sim' ? '' :
      `<span class="dev-del" data-del="${s.id}" title="Xoá" style="margin-left:auto;color:#717a84;cursor:pointer;font-size:13px;padding:0 2px">×</span>`;
    card.innerHTML =
      (sel ? '<span class="selbar"></span>' : '') +
      `<div style="display:flex;align-items:center;gap:7px">
         <span class="dot" style="background:${dotColor};box-shadow:0 0 6px ${dotColor}"></span>
         <span class="dev-name" style="color:${sel ? '#e8ecf1' : '#cdd4dc'}">${s.name}</span>
         ${delBtn}
       </div>
       <div class="mono" style="display:flex;justify-content:space-between;font-size:9.5px;color:#5b636d">${meta}</div>
       <div class="mono" style="display:flex;gap:10px;font-size:9.5px;color:#717a84">${sub}</div>`;
    card.onclick = (e) => {
      if (e.target && e.target.getAttribute && e.target.getAttribute('data-del')) { deleteDevice(s.id); return; }
      selectDevice(s.id);
    };
    list.appendChild(card);
  }
  $('onlineCount').textContent = online + ' ONLINE';
}

function selectDevice(id) {
  saveConnFields(ui.selected);
  ui.selected = id;
  const s = SENSORS.find((x) => x.id === id) || SIM_DEVICE;
  $('selName').textContent = s.name;
  loadConnFields(s.id);
  renderDevices();
}

function deleteDevice(id) {
  if (id === 'sim') return;
  if (ui.connected && ui.connectedId === id) return; // don't remove the live one
  SENSORS = SENSORS.filter((s) => s.id !== id);
  delete cfgs[id];
  if (ui.selected === id) { ui.selected = 'sim'; $('selName').textContent = SIM_DEVICE.name; loadConnFields('sim'); }
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
  setNetProto(c.netProto || 'udp');
  setScan(c.scanMode);
  setPrecision(c.precision);
  $('distMin').value = c.distMin;
  $('distMax').value = c.distMax;
  setCoord(c.coordSys);
  setQuality(c.quality);
  $('posX').value = '0.00'; $('posY').value = '0.00'; $('rot').value = '0.0';
  pushPlacement();
}

const smoothing = { on: true, amount: 0.5 };
function pushSmooth() {
  window.lidar.setConfig({ smooth: smoothing.on, smoothAmount: smoothing.amount });
  $('smoothKnob').style.left = smoothing.on ? '24px' : '2px';
  $('smoothKnob').style.background = smoothing.on ? '#39ff7a' : '#717a84';
  $('smoothToggle').style.borderColor = smoothing.on ? 'rgba(57,255,122,0.5)' : 'rgba(255,255,255,0.12)';
  $('smoothToggle').style.background = smoothing.on ? 'rgba(57,255,122,0.12)' : '#0e1216';
  $('smoothLabel').textContent = smoothing.amount.toFixed(2);
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
  c.netProto = netProto;
  c.distMin = $('distMin').value;
  c.distMax = $('distMax').value;
}

function segActivate(groupAttr, attr, val) {
  document.querySelectorAll(`[${groupAttr}]`).forEach((b) => {
    b.classList.toggle('active', b.getAttribute(groupAttr) === val);
  });
}

let connType = 'serial', scanMode = 'express', precision = '2', coordSys = 'cartesian', quality = false, netProto = 'udp';

function setConn(v) {
  connType = v;
  segActivate('data-conn', 'data-conn', v);
  $('serialFields').style.display = v === 'serial' ? 'flex' : 'none';
  $('networkFields').style.display = v === 'network' ? 'flex' : 'none';
  cfgs[ui.selected].connType = v;
}
function setNetProto(v) { netProto = v; segActivate('data-net', 'data-net', v); cfgs[ui.selected].netProto = v; }
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
    ipAddr: $('ipAddr').value,
    ipPort: $('ipPort').value,
    netProto,
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
  document.querySelectorAll('[data-net]').forEach((b) => (b.onclick = () => setNetProto(b.getAttribute('data-net'))));
  document.querySelectorAll('[data-scan]').forEach((b) => (b.onclick = () => setScan(b.getAttribute('data-scan'))));
  document.querySelectorAll('[data-p]').forEach((b) => (b.onclick = () => setPrecision(b.getAttribute('data-p'))));
  document.querySelectorAll('[data-coord]').forEach((b) => (b.onclick = () => setCoord(b.getAttribute('data-coord'))));
  $('qualityBtn').onclick = () => setQuality(!quality);
  $('connectBtn').onclick = doConnect;
  $('autoDetectBtn').onclick = autoDetect;
  $('addDeviceBtn').onclick = addManualDevice;

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

  $('smoothToggle').onclick = () => { smoothing.on = !smoothing.on; pushSmooth(); };
  $('smoothAmt').oninput = () => { smoothing.amount = parseFloat($('smoothAmt').value); pushSmooth(); };

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
    t.onclick = () => switchTab(t.getAttribute('data-tab'));
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
  let warpDrag = null, warpScale = null; // warpDrag = { origin:[wx,wy], starts:{i:[x,y]} }
  let zoneStart = null, zoneDragging = false, zoneMoved = false;
  let zoneMove = null, zoneResize = null; // editing an existing zone (Select tool)
  wrap.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;
    const r = wrap.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    if (ui.tool === 'zone') {
      // record the press; on release: a click adds a polygon point, a drag makes a rectangle
      zoneStart = stw(mx, my);
      zoneDragging = true;
      zoneMoved = false;
      draftRect = null;
      return;
    }
    if (warpInteractive()) {
      // scale handle?
      if (warp._bbox) {
        for (const h of warp._bbox) {
          if (Math.hypot(h.sx - mx, h.sy - my) <= 8) {
            warpScale = { anchor: h.aw.slice(), ref: h.cw.slice(), snap: warp.corners.map((cc) => cc.slice()) };
            warpSnapshot();
            return;
          }
        }
      }
      let hit = -1;
      warp.corners.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); if (Math.hypot(sx - mx, sy - my) <= 10) hit = i; });
      if (hit >= 0) {
        if (warp.sel.indexOf(hit) < 0) warp.sel = e.shiftKey ? warp.sel.concat(hit) : [hit];
        const o = stw(mx, my);
        warpDrag = { origin: o, starts: {} };
        for (const i of warp.sel) warpDrag.starts[i] = warp.corners[i].slice();
        warpSnapshot(); updateWarpSel();
      } else {
        if (!e.shiftKey) warp.sel = [];
        warp.marquee = { x0: mx, y0: my, x1: mx, y1: my }; updateWarpSel();
      }
      return;
    }
    // SELECT tool: edit existing zones (resize handle / move / select / deselect)
    if (ui.tool === 'select') {
      const [wx, wy] = stw(mx, my);
      if (selectedZone >= 0 && zoneBbox) {
        for (const h of zoneBbox) {
          if (Math.hypot(h.sx - mx, h.sy - my) <= 8) {
            zoneResize = { anchor: h.aw.slice(), ref: h.cw.slice(), snap: zones[selectedZone].pts.map((p) => p.slice()) };
            return;
          }
        }
      }
      let hit = -1;
      for (let i = zones.length - 1; i >= 0; i--) {
        if (zones[i].visible !== false && pipR(wx, wy, zones[i].pts)) { hit = i; break; }
      }
      if (hit >= 0) {
        selectedZone = hit; renderZoneCards();
        zoneMove = { start: [wx, wy], orig: zones[hit].pts.map((p) => p.slice()) };
        return;
      }
      if (selectedZone >= 0) { selectedZone = -1; renderZoneCards(); }
    }
    dragging = true; lastX = e.clientX; lastY = e.clientY;
  });
  window.addEventListener('mouseup', (e) => {
    if (zoneMove || zoneResize) { zoneMove = null; zoneResize = null; pushZones(); return; }
    if (ui.tool === 'zone' && zoneDragging) {
      zoneDragging = false;
      if (zoneMoved && draftRect) {
        // drag -> rectangle zone (4 corners)
        const [a, b] = draftRect;
        const x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]);
        const y0 = Math.min(a[1], b[1]), y1 = Math.max(a[1], b[1]);
        addZone([[x0, y1], [x1, y1], [x1, y0], [x0, y0]]);
        draft = []; draftRect = null;
        setTool('select');
      } else if (zoneStart) {
        // click -> add a polygon vertex (FINISH to complete)
        draft.push(zoneStart);
        $('draftCount').textContent = draft.length + ' pts';
      }
      zoneStart = null;
      return;
    }
    if (warpScale) { warpScale = null; pushWarp(); }
    if (warpDrag) { warpDrag = null; warp._snap = null; pushWarp(); }
    if (warp.marquee) {
      const m = warp.marquee;
      const x0 = Math.min(m.x0, m.x1), x1 = Math.max(m.x0, m.x1), y0 = Math.min(m.y0, m.y1), y1 = Math.max(m.y0, m.y1);
      const sel = e.shiftKey ? warp.sel.slice() : [];
      warp.corners.forEach((p, i) => { const [sx, sy] = wts(p[0], p[1]); if (sx >= x0 && sx <= x1 && sy >= y0 && sy <= y1 && sel.indexOf(i) < 0) sel.push(i); });
      warp.sel = sel; warp.marquee = null; updateWarpSel();
    }
    dragging = false;
  });
  wrap.addEventListener('mousemove', (e) => {
    const r = wrap.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    const [wx, wy] = stw(mx, my);
    $('cursor').textContent = `x ${wx.toFixed(2)}  y ${wy.toFixed(2)}`;
    if (ui.tool === 'zone' && zoneDragging) {
      if (Math.hypot(wx - zoneStart[0], wy - zoneStart[1]) > 0.12) {
        zoneMoved = true;
        draftRect = [zoneStart, [wx, wy]];
      }
      return;
    }
    if (zoneMove && selectedZone >= 0) {
      const dx = wx - zoneMove.start[0], dy = wy - zoneMove.start[1];
      zones[selectedZone].pts = zoneMove.orig.map((p) => [p[0] + dx, p[1] + dy]);
      return;
    }
    if (zoneResize && selectedZone >= 0) {
      const rx = (wx - zoneResize.anchor[0]) / ((zoneResize.ref[0] - zoneResize.anchor[0]) || 1e-6);
      const ry = (wy - zoneResize.anchor[1]) / ((zoneResize.ref[1] - zoneResize.anchor[1]) || 1e-6);
      zones[selectedZone].pts = zoneResize.snap.map((p) => [
        zoneResize.anchor[0] + (p[0] - zoneResize.anchor[0]) * rx,
        zoneResize.anchor[1] + (p[1] - zoneResize.anchor[1]) * ry,
      ]);
      return;
    }
    if (warpScale) {
      const rx = (wx - warpScale.anchor[0]) / ((warpScale.ref[0] - warpScale.anchor[0]) || 1e-6);
      const ry = (wy - warpScale.anchor[1]) / ((warpScale.ref[1] - warpScale.anchor[1]) || 1e-6);
      for (const i of warp.sel) {
        warp.corners[i][0] = warpScale.anchor[0] + (warpScale.snap[i][0] - warpScale.anchor[0]) * rx;
        warp.corners[i][1] = warpScale.anchor[1] + (warpScale.snap[i][1] - warpScale.anchor[1]) * ry;
      }
      pushWarpLive();
      return;
    }
    if (warpDrag) {
      let dx = wx - warpDrag.origin[0], dy = wy - warpDrag.origin[1];
      if (e.shiftKey) { if (Math.abs(dx) >= Math.abs(dy)) dy = 0; else dx = 0; } // axis-lock
      for (const i of warp.sel) {
        const s = warpDrag.starts[i];
        warp.corners[i][0] = s[0] + dx;
        warp.corners[i][1] = s[1] + dy;
      }
      // magnetic snap to a wall point when dragging a single corner (not while axis-locking)
      warp._snap = null;
      if (warp.sel.length === 1 && !e.shiftKey) {
        const i = warp.sel[0];
        const t = findNearestCloud(warp.corners[i][0], warp.corners[i][1], 0.4);
        if (t) { warp.corners[i] = [t[0], t[1]]; warp._snap = t; }
      }
      pushWarpLive();
      return;
    }
    if (warp.marquee) { warp.marquee.x1 = mx; warp.marquee.y1 = my; return; }
    if (dragging) {
      view.panX += e.clientX - lastX;
      view.panY += e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
    }
  });

  // tools + zone drawing
  document.querySelectorAll('[data-tool]').forEach((b) => {
    if (b.disabled) return;
    b.onclick = () => setTool(b.getAttribute('data-tool'));
  });
  $('finishZone').onclick = finishZone;
  $('cancelZone').onclick = cancelDraft;

  // output (OSC / TUIO)
  document.querySelectorAll('[data-proto]').forEach((b) => {
    b.onclick = () => {
      out.protocol = b.getAttribute('data-proto');
      document.querySelectorAll('[data-proto]').forEach((x) => x.classList.toggle('active', x === b));
      updateOscPreview();
      pushOutput();
    };
  });
  document.querySelectorAll('[data-rate]').forEach((b) => {
    b.onclick = () => {
      out.sendRate = b.getAttribute('data-rate');
      document.querySelectorAll('[data-rate]').forEach((x) => x.classList.toggle('active', x === b));
      pushOutput();
    };
  });
  $('oscHost').onchange = () => { out.host = $('oscHost').value; pushOutput(); };
  $('oscPort').onchange = () => { out.port = $('oscPort').value; pushOutput(); };
  updateOscPreview();
  updatePill();

  // warp
  $('warpToggle').onclick = () => setWarpEnabled(!warp.enabled);
  $('warpSelectAll').onclick = () => { warp.sel = [0, 1, 2, 3]; updateWarpSel(); };
  $('warpClear').onclick = () => { warp.sel = []; updateWarpSel(); };
  $('warpUndo').onclick = warpUndo;
  $('warpRedo').onclick = warpRedo;
  $('warpRotStep').onchange = () => { warp.rotStep = parseFloat($('warpRotStep').value) || 15; };
  $('warpRotCCW').onclick = () => warpRotate(-warp.rotStep);
  $('warpRotCW').onclick = () => warpRotate(warp.rotStep);
  $('warpFlipH').onclick = () => warpFlip(true);
  $('warpFlipV').onclick = () => warpFlip(false);
  $('warpDragBtn').onclick = () => setTool('warp');
  $('warpReset').onclick = () => { warpSnapshot(); warp.corners = [[-3, 5], [3, 5], [3, 0.5], [-3, 0.5]]; warp.sel = []; updateWarpSel(); pushWarp(); };
  renderCornerInputs();
  updateWarpSel();
  updateWarpButtons();

  // output mode + action
  buildNdiConfig();
  // Syphon is macOS-only — hide the button elsewhere
  if (window.lidar.platform !== 'darwin') {
    const sb = document.querySelector('[data-omode="syphon"]');
    if (sb) sb.style.display = 'none';
  }
  document.querySelectorAll('[data-omode]').forEach((b) => {
    b.onclick = () => {
      outputMode = b.getAttribute('data-omode');
      document.querySelectorAll('[data-omode]').forEach((x) => x.classList.toggle('active', x === b));
      $('ndiConfig').style.display = (outputMode === 'ndi' || outputMode === 'syphon') ? 'flex' : 'none';
      updateOutputAction();
    };
  });
  $('outputAction').onclick = doOutputAction;
  window.lidar.onProjectorState((stP) => { projectorOpen = stP.open; updateOutputAction(); });

  // keyboard: warp undo/redo
  window.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && (e.key === 'z' || e.key === 'Z')) {
      e.preventDefault();
      if (e.shiftKey) warpRedo(); else warpUndo();
    }
    if ((e.key === 'Delete' || e.key === 'Backspace') && selectedZone >= 0 && document.activeElement && document.activeElement.tagName !== 'INPUT') {
      e.preventDefault();
      deleteZone(selectedZone);
    }
  });
  $('drawNewZone').onclick = () => { switchTab('zones'); setTool('zone'); };

  $('protoPill').onclick = () => switchTab('output');
  $('projectBtn').onclick = doOutputAction;

  // record / playback
  $('recordBtn').onclick = toggleRecord;
  $('recBtn2').onclick = toggleRecord;
  $('playBtn').onclick = () => { if (rec.takes.length) playTake(rec.takes[rec.takes.length - 1].id); };
  startTransportLoop();
}

function switchTab(name) {
  ui.tab = name;
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('active', x.getAttribute('data-tab') === name));
  document.querySelectorAll('.tab-pane').forEach((p) => p.classList.toggle('active', p.getAttribute('data-pane') === name));
}

function renderStreamGlyph() {
  $('streamGlyph').innerHTML = ui.streaming
    ? '<span style="display:flex;gap:2.5px"><span style="width:3px;height:11px;background:#00e5ff;display:block"></span><span style="width:3px;height:11px;background:#00e5ff;display:block"></span></span>'
    : '<span style="width:0;height:0;border-left:9px solid #00e5ff;border-top:6px solid transparent;border-bottom:6px solid transparent;display:block"></span>';
}

// ---- record / playback ----------------------------------------------------
const rec = { recording: false, start: 0, takes: [], playing: false, playStart: 0, playDur: 0 };
const fmtTime = (ms) => { const s = Math.floor(ms / 1000); return String(Math.floor(s / 60)).padStart(2, '0') + ':' + String(s % 60).padStart(2, '0'); };

async function toggleRecord() {
  if (rec.recording) {
    rec.recording = false; setRecUi(false);
    const res = await window.lidar.recordStop();
    if (res.take) { rec.takes.push(res.take); renderTakes(); }
  } else {
    if (!ui.connected) { setConnStatus('connect a source first', '#ffb000'); return; }
    await window.lidar.recordStart();
    rec.recording = true; rec.playing = false; rec.start = performance.now(); setRecUi(true);
  }
}
function setRecUi(on) {
  $('recBtn2Label').textContent = on ? 'STOP' : 'RECORD';
  $('recBtn').lastElementChild.textContent = on ? 'STOP' : 'RECORD';
  $('recDot').style.animation = on ? 'blink 1s infinite' : 'none';
}
function renderTakes() {
  const box = $('takes');
  box.innerHTML = '';
  rec.takes.forEach((t) => {
    const chip = document.createElement('button');
    chip.className = 'btn';
    chip.style.cssText = 'height:26px;padding:0 9px';
    chip.innerHTML = `<span style="color:#cdd4dc">${t.name}</span> <span style="color:#5b636d">${fmtTime(t.durMs)}</span>`;
    chip.onclick = () => playTake(t.id);
    box.appendChild(chip);
  });
}
async function playTake(id) {
  const res = await window.lidar.playTake(id);
  if (res.ok) {
    rec.playing = true; rec.playStart = performance.now(); rec.playDur = res.durMs || 1000;
    ui.connected = true;
    const t = rec.takes.find((x) => x.id === id);
    setConnStatus('▶ playing ' + (t ? t.name : 'take'), '#9fe4ef');
  } else setConnStatus(res.error || 'play failed', '#ff4d5e');
}
function startTransportLoop() {
  setInterval(() => {
    const tc = $('timecode'), ph = $('playhead');
    if (rec.recording) {
      tc.textContent = fmtTime(performance.now() - rec.start);
      ph.style.background = 'rgba(255,77,94,0.4)'; ph.style.width = '100%';
    } else if (rec.playing) {
      const ms = (performance.now() - rec.playStart) % (rec.playDur || 1);
      tc.textContent = fmtTime(ms);
      ph.style.background = 'linear-gradient(90deg,rgba(0,229,255,0.25),rgba(0,229,255,0.5))';
      ph.style.width = (100 * ms / (rec.playDur || 1)) + '%';
    } else { ph.style.width = '0'; }
  }, 120);
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

// Press-and-go: probe every serial port at each baudrate, identify any RPLIDAR,
// then auto-fill COM port + baudrate with the first one found.
async function autoDetect() {
  const btn = $('autoDetectBtn');
  const box = $('detectResult');
  if (ui.connected) {
    box.style.display = 'block';
    box.textContent = '⚠ Đang kết nối — bấm DISCONNECT trước khi dò.';
    return;
  }
  btn.disabled = true;
  const label = btn.textContent;
  btn.textContent = '⏳ ĐANG DÒ…';
  box.style.display = 'block';
  box.textContent = 'Đang quét các cổng COM và thử baudrate…';
  await refreshPorts();
  try {
    const res = await window.lidar.autodetect();
    if (!res.ok) { box.textContent = '⚠ ' + (res.error || 'lỗi dò'); return; }
    if (!res.devices || !res.devices.length) {
      box.innerHTML = '✕ Không thấy RPLIDAR.\nKiểm tra: cáp/đổi cổng USB · đã cài driver CP210x/CH340 · không có app khác (RoboStudio) đang giữ cổng.';
      return;
    }
    // Put every detected LiDAR into the DEVICES list so the user can pick one.
    applyDetected(res.devices);
    const n = res.devices.length;
    let html = `✓ Tìm thấy <span style="color:#39ff7a">${n}</span> LiDAR — xem ở danh sách <b>DEVICES</b> phía trên, bấm chọn rồi <b>CONNECT</b>.\n`;
    html += res.devices.map((x) => `   • ${x.name} · ${x.path} @ ${x.baudrate}`).join('\n');
    box.innerHTML = html;
  } catch (e) {
    box.textContent = '⚠ lỗi: ' + e.message;
  } finally {
    btn.disabled = false;
    btn.textContent = label;
  }
}

// ---- boot -----------------------------------------------------------------
function boot() {
  resize();
  new ResizeObserver(resize).observe(wrap);
  loadConnFields(ui.selected);
  $('selName').textContent = (SENSORS.find((s) => s.id === ui.selected) || SIM_DEVICE).name;
  renderDevices();
  wireControls();
  pushSmooth();
  refreshPorts();

  window.lidar.onScan(ingestScan);
  window.lidar.onStatus((msg) => setConnStatus(msg, '#9aa3ad'));
  window.lidar.onInfo((info) => console.log('device info', info));
  window.lidar.onOscLog(appendOscLog);

  requestAnimationFrame(draw);
}

// ---- preset collect / apply (called from main via the File menu) ----------
window.__collectPreset = function () {
  return {
    selected: ui.selected,
    cfgs: cfgs,
    connType: connType, netProto: netProto,
    comPort: $('comPort').value, baudrate: $('baudrate').value,
    ipAddr: $('ipAddr').value, ipPort: $('ipPort').value,
    scanMode: scanMode, precision: precision, coordSys: coordSys, quality: quality,
    distMin: $('distMin').value, distMax: $('distMax').value,
    placement: { x: $('posX').value, y: $('posY').value, rot: $('rot').value },
    bg: { subtract: bg.subtract, tol: bg.tol, captured: bg.captured },
    smoothing: { on: smoothing.on, amount: smoothing.amount },
    zones: zones.map(function (z) { return { name: z.name, slug: z.slug, pts: z.pts, visible: z.visible }; }),
    warp: { corners: warp.corners, enabled: warp.enabled, rotStep: warp.rotStep },
    out: { protocol: out.protocol, host: out.host, port: out.port, sendRate: out.sendRate, format: out.format, normalize: out.normalize },
    ndiCfg: ndiCfg,
    outputMode: outputMode,
  };
};

window.__applyPreset = function (o) {
  if (!o) return;
  if (o.cfgs) Object.keys(o.cfgs).forEach(function (k) { cfgs[k] = Object.assign({}, cfgs[k], o.cfgs[k]); });
  if (o.selected) ui.selected = o.selected;
  if (o.connType) setConn(o.connType);
  if (o.netProto) setNetProto(o.netProto);
  if (o.comPort != null) $('comPort').value = o.comPort;
  if (o.baudrate != null) $('baudrate').value = o.baudrate;
  if (o.ipAddr != null) $('ipAddr').value = o.ipAddr;
  if (o.ipPort != null) $('ipPort').value = o.ipPort;
  if (o.scanMode) setScan(o.scanMode);
  if (o.precision) setPrecision(o.precision);
  if (o.coordSys) setCoord(o.coordSys);
  if (o.distMin != null) $('distMin').value = o.distMin;
  if (o.distMax != null) $('distMax').value = o.distMax;
  if (o.quality != null) setQuality(o.quality);
  if (o.placement) { $('posX').value = o.placement.x; $('posY').value = o.placement.y; $('rot').value = o.placement.rot; pushPlacement(); }
  if (o.bg) {
    bg.tol = parseFloat(o.bg.tol) || 0.18;
    $('bgTol').value = bg.tol; $('bgTolLabel').textContent = Math.round(bg.tol * 100) + ' cm';
    window.lidar.setConfig({ bgTol: bg.tol });
    syncBgUi(!!o.bg.captured, false);
    setBgSubtract(!!o.bg.subtract);
  }
  if (o.smoothing) { smoothing.on = !!o.smoothing.on; smoothing.amount = parseFloat(o.smoothing.amount); if (isNaN(smoothing.amount)) smoothing.amount = 0.5; $('smoothAmt').value = smoothing.amount; pushSmooth(); }
  if (o.zones) { zones = o.zones.map(function (z) { return { name: z.name, slug: z.slug, pts: z.pts, visible: z.visible !== false, occupied: false }; }); pushZones(); renderZoneCards(); }
  if (o.warp) {
    warp.corners = o.warp.corners; warp.rotStep = o.warp.rotStep || 15;
    if ($('warpRotStep')) $('warpRotStep').value = warp.rotStep;
    renderCornerInputs(); recomputeWarp(); setWarpEnabled(!!o.warp.enabled);
  }
  if (o.out) {
    Object.assign(out, o.out);
    $('oscHost').value = out.host; $('oscPort').value = out.port;
    document.querySelectorAll('[data-proto]').forEach(function (b) { b.classList.toggle('active', b.getAttribute('data-proto') === out.protocol); });
    document.querySelectorAll('[data-rate]').forEach(function (b) { b.classList.toggle('active', b.getAttribute('data-rate') === String(out.sendRate)); });
    updateOscPreview(); updatePill(); pushOutput();
  }
  if (o.ndiCfg) { Object.assign(ndiCfg, o.ndiCfg); buildNdiConfig(); }
  if (o.outputMode) {
    outputMode = o.outputMode;
    document.querySelectorAll('[data-omode]').forEach(function (b) { b.classList.toggle('active', b.getAttribute('data-omode') === outputMode); });
    $('ndiConfig').style.display = (outputMode === 'ndi' || outputMode === 'syphon') ? 'flex' : 'none';
    updateOutputAction();
  }
  window.lidar.setConfig({ distMin: $('distMin').value, distMax: $('distMax').value, quality: quality });
  $('selName').textContent = (SENSORS.find(function (s) { return s.id === ui.selected; }) || { name: '' }).name;
  renderDevices();
};

boot();
