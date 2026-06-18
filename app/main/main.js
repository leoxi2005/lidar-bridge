'use strict';
const { app, BrowserWindow, ipcMain, screen, Menu, dialog } = require('electron');
const path = require('path');
const fs = require('fs');
const { RPLidar } = require('./rplidar');
const { Simulator } = require('./simulator');
const { Pipeline } = require('./pipeline');
const { OscSender, oscMessage, oscBundle } = require('./osc');
const { applyH } = require('./homography');

// In dev (`npm start`) the app menu / About / Dock default to "Electron".
// Set the real product name so it shows "LiDAR Bridge" everywhere.
app.setName('LEOXI-LIDARTRACKING');
// Use our icon for the Dock in dev (packaged builds get it from electron-builder).
if (process.platform === 'darwin' && app.dock) {
  try { app.dock.setIcon(path.join(__dirname, '..', 'build', 'icon.png')); } catch (_) {}
}

let projWin = null; // projector output window
let ndiSender = null; // native NDI sender (koffi -> NDI runtime)
let ndiWin = null; // offscreen window rendering the mapping for NDI
let ndiNative = false; // true when streaming exact-resolution RGBA via IPC
let syServer = null; // Syphon server (macOS only, optional node-syphon)
let syWin = null; // offscreen window rendering the mapping for Syphon

// record / playback of raw scan sessions
let recording = false;
let recStart = 0;
let recFrames = [];
let takes = []; // [{ id, name, durMs, frames:[{t, nodes}] }]
let takeSeq = 0;

let win = null;
let source = null; // active RPLidar | Simulator (single-sensor mode)
let lastScanAt = 0;

// ---- surfaces (v3) --------------------------------------------------------
// Each projection surface (a wall, the floor, …) owns its own sensor group +
// Pipeline (own warp/zones/tracking) + OSC namespace + NDI resolution. One
// surface with the simple connect behaves like the previous single-output app.
let surfaceSeq = 0;
function makeSurface(name, oscPrefix) {
  surfaceSeq++;
  return {
    id: 's' + surfaceSeq,
    name: name || ('Mặt ' + surfaceSeq),
    oscPrefix: oscPrefix || '',     // '' -> /lidar/...  ;  'wall1' -> /wall1/lidar/...
    sensorIds: [],                  // connected sensors feeding this surface
    pipeline: new Pipeline(),
    ndi: { w: 1280, h: 720, fps: 30 },
    lastOut: { tracks: [], zones: [] },
  };
}
let surfaces = [makeSurface('Mặt 1', '')];
let activeSurfaceId = surfaces[0].id;
function activeSurface() { return surfaces.find((s) => s.id === activeSurfaceId) || surfaces[0]; }
// `pipeline` always points at the active surface, so the existing warp/zones/
// config/background IPC handlers edit whichever surface the UI has selected.
let pipeline = surfaces[0].pipeline;

// ---- multi-sensor fusion (F7) ---------------------------------------------
let fusionMode = false;
const fusionSources = new Map(); // id -> { sensor, pose, enabled }
const fusionScans = new Map();   // id -> latest nodes[]
let fusionTimer = null;

// auto-reconnect watchdog (real hardware)
let lastConnectCfg = null;
let autoReconnect = false;
let reconnecting = false;
let watchdog = null;
const STALL_MS = 2500; // no scans for this long -> assume lost, reconnect

// network output
const sender = new OscSender();
let outCfg = { protocol: 'osc', host: '127.0.0.1', port: 7000, sendRate: 30, normalize: false, format: 'slots' };
const MAX_SLOTS = 64; // safety cap on instancing slots
let peakSlots = 0; // high-water mark of concurrent tracks this session (auto-sizes the slot set)
let sendTimer = null;
let tuioFseq = 0;
let lastLogAt = 0;
let latestOut = { tracks: [], zones: [] };
const zonePrevOn = new Map(); // slug -> last sent occupancy (for enter/exit pulses)

// Push per-zone OSC messages (state, count, dwell, enter/exit) into a bundle array.
function sendZones(msgs, lines) {
  for (const z of latestOut.zones) {
    const base = '/lidar/zone/' + z.slug;
    msgs.push({ a: base, args: [{ type: 'i', value: z.on ? 1 : 0 }] });
    msgs.push({ a: base + '/count', args: [{ type: 'i', value: z.count || 0 }] });
    msgs.push({ a: base + '/dwell', args: [{ type: 'f', value: z.dwell || 0 }] });
    const prev = zonePrevOn.get(z.slug) || false;
    if (z.on && !prev) msgs.push({ a: base + '/enter', args: [{ type: 'i', value: 1 }] });
    if (!z.on && prev) msgs.push({ a: base + '/exit', args: [{ type: 'i', value: 1 }] });
    zonePrevOn.set(z.slug, z.on);
    if (lines) lines.push(base + '  ' + (z.on ? 1 : 0) + '  n=' + (z.count || 0) + '  d=' + (z.dwell || 0).toFixed(1) + 's');
  }
}

function createWindow() {
  win = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1100,
    minHeight: 680,
    backgroundColor: '#0a0b0e',
    title: 'LEOXI-LIDARTRACKING',
    show: !process.env.LIDAR_AUTOSHOT,
    webPreferences: {
      preload: path.join(__dirname, '..', 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });
  win.loadFile(path.join(__dirname, '..', 'renderer', 'index.html'));
  win.on('closed', () => {
    win = null;
    stopSyphon();
    stopNdi();
    if (projWin && !projWin.isDestroyed()) projWin.close();
  });
  if (process.env.LIDAR_AUTOSHOT) runAutoShot();
}

function send(channel, payload) {
  if (win && !win.isDestroyed()) win.webContents.send(channel, payload);
}

// Full pipeline pass for one scan, then stream the frame to the renderer.
function onScan(nodes) {
  const now = Date.now();
  if (recording) recFrames.push({ t: now - recStart, nodes });
  const periodMs = lastScanAt ? now - lastScanAt : 100;
  lastScanAt = now;
  const dtSec = Math.min(0.5, periodMs / 1000);
  const frame = pipeline.process(nodes, dtSec);
  frame.periodMs = periodMs;
  send('lidar:scan', frame);

  latestOut = {
    tracks: frame.tracks,
    zones: frame.zoneInfo || [],
  };

  broadcastProjectorFrame({
    warpEnabled: pipeline.warpEnabled,
    tracks: frame.tracks.map((t) => ({ u: t.u, v: t.v, out: t.out, color: t.color, id: t.id })),
    zones: pipeline.zones.map((z, i) => ({
      name: z.name,
      occupied: !!frame.zoneOcc[i],
      uv: z.pts.map((p) => applyH(pipeline.warpH, p[0], p[1])),
    })),
  });
}

// Fusion tick: merge the latest scan from every connected sensor (each through its
// own pose + baseline) into one world frame, then stream it like onScan does.
let fusionLast = 0;
function fusionTick() {
  const now = Date.now();
  const periodMs = fusionLast ? now - fusionLast : 100;
  fusionLast = now;
  const dtSec = Math.min(0.5, periodMs / 1000);
  // sensors explicitly claimed by some surface; the default surface (index 0) with
  // no explicit list scoops up whatever's left, so 1-surface fusion = "all sensors".
  const claimed = new Set();
  for (const surf of surfaces) for (const id of surf.sensorIds) claimed.add(id);
  for (const surf of surfaces) {
    let ids = surf.sensorIds;
    if (!ids.length && surf === surfaces[0]) ids = [...fusionSources.keys()].filter((id) => !claimed.has(id));
    const sensors = [];
    for (const id of ids) {
      const s = fusionSources.get(id);
      if (!s || s.enabled === false) continue;
      sensors.push({ id, nodes: fusionScans.get(id) || [], pose: s.pose || {} });
    }
    const frame = surf.pipeline.processFusion(sensors, dtSec);
    surf.lastOut = { tracks: frame.tracks, zones: frame.zoneInfo || [] };
    emitSurfaceOsc(surf);                  // every surface emits OSC under its prefix
    if (surf.id === activeSurfaceId) {     // only the selected surface drives the UI
      frame.periodMs = periodMs;
      send('lidar:scan', frame);
      latestOut = surf.lastOut;
      broadcastProjectorFrame({
        warpEnabled: surf.pipeline.warpEnabled,
        tracks: frame.tracks.map((t) => ({ u: t.u, v: t.v, out: t.out, color: t.color, id: t.id })),
        zones: surf.pipeline.zones.map((z, i) => ({
          name: z.name, occupied: !!frame.zoneOcc[i],
          uv: z.pts.map((p) => applyH(surf.pipeline.warpH, p[0], p[1])),
        })),
      });
    }
  }
}

// Emit one surface's tracks/zones over OSC under its namespace. prefix '' -> the
// root /lidar/... (back-compat); 'wall1' -> /wall1/lidar/... so TouchDesigner can
// route each surface independently. Uses the slots (instancing) layout.
function emitSurfaceOsc(surf) {
  const base = surf.oscPrefix ? '/' + surf.oscPrefix : '';
  const ts = surf.lastOut.tracks || [];
  const zs = surf.lastOut.zones || [];
  const coord = (t) => (outCfg.normalize && t.u != null ? [t.u, t.v] : [t.x, t.y]);
  const msgs = [{ a: base + '/lidar/count', args: [{ type: 'i', value: ts.length }] }];
  for (let i = 0; i < ts.length; i++) {
    const t = ts[i]; const [a, b] = coord(t);
    msgs.push({ a: `${base}/lidar/p${i}/on`, args: [{ type: 'i', value: 1 }] });
    msgs.push({ a: `${base}/lidar/p${i}/x`, args: [{ type: 'f', value: a }] });
    msgs.push({ a: `${base}/lidar/p${i}/y`, args: [{ type: 'f', value: b }] });
    msgs.push({ a: `${base}/lidar/p${i}/v`, args: [{ type: 'f', value: t.vel || 0 }] });
    msgs.push({ a: `${base}/lidar/p${i}/id`, args: [{ type: 'i', value: parseInt(t.id, 10) }] });
  }
  for (const z of zs) {
    const zb = `${base}/lidar/zone/${z.slug}`;
    msgs.push({ a: zb, args: [{ type: 'i', value: z.on ? 1 : 0 }] });
    msgs.push({ a: zb + '/count', args: [{ type: 'i', value: z.count || 0 }] });
    msgs.push({ a: zb + '/dwell', args: [{ type: 'f', value: z.dwell || 0 }] });
  }
  sender.sendBundle(msgs);
}

async function teardownFusion() {
  if (fusionTimer) { clearInterval(fusionTimer); fusionTimer = null; }
  for (const [, s] of fusionSources) {
    try { s.sensor.removeAllListeners(); await s.sensor.disconnect(); } catch (_) {}
  }
  fusionSources.clear();
  fusionScans.clear();
  fusionMode = false;
}

// Send a frame to every output surface (visible projector window + offscreen Syphon render).
function broadcastProjectorFrame(payload) {
  if (projWin && !projWin.isDestroyed()) projWin.webContents.send('projector:frame', payload);
  if (syWin && !syWin.isDestroyed()) syWin.webContents.send('projector:frame', payload);
  if (ndiWin && !ndiWin.isDestroyed()) ndiWin.webContents.send('projector:frame', payload);
}

// ---- projector output window ---------------------------------------------
function openProjector(mode) {
  if (projWin && !projWin.isDestroyed()) { projWin.focus(); return; }
  const o = {
    width: 960, height: 600, frame: false, backgroundColor: '#000', title: 'LEOXI-LIDARTRACKING — Output',
    webPreferences: { preload: path.join(__dirname, '..', 'projector-preload.js'), contextIsolation: true, nodeIntegration: false },
  };
  if (mode === 'extended') {
    const displays = screen.getAllDisplays();
    const primary = screen.getPrimaryDisplay();
    const ext = displays.find((d) => d.id !== primary.id);
    if (ext) { o.x = ext.bounds.x; o.y = ext.bounds.y; o.width = ext.bounds.width; o.height = ext.bounds.height; o.fullscreen = true; }
  }
  projWin = new BrowserWindow(o);
  projWin.loadFile(path.join(__dirname, '..', 'renderer', 'projector.html'));
  projWin.on('closed', () => { projWin = null; send('lidar:projector-state', { open: false }); });
  send('lidar:projector-state', { open: true });
}

ipcMain.on('projector:exit', () => { if (projWin && !projWin.isDestroyed()) projWin.close(); });
ipcMain.on('projector:fullscreen', () => { if (projWin && !projWin.isDestroyed()) projWin.setFullScreen(!projWin.isFullScreen()); });

// ---- Syphon output (macOS) -----------------------------------------------
// Renders the clean mapping in an offscreen window and publishes each painted
// frame to a Syphon server, so TouchDesigner can receive it via "Syphon Spout In".
function startSyphon(cfg) {
  if (process.platform !== 'darwin') return { ok: false, error: 'Syphon is macOS-only. On Windows use NDI Tools / OBS to capture the OUTPUT window.' };
  let Syphon;
  try { Syphon = require('node-syphon'); } catch (e) { return { ok: false, error: 'node-syphon not installed.' }; }
  stopSyphon();
  const W = parseInt(cfg.w, 10) || 1280;
  const H = parseInt(cfg.h, 10) || 720;
  // Retina: the offscreen framebuffer = window size × scaleFactor. Divide the window
  // size by the display scale so the published pixels equal the requested W×H exactly.
  const sf = (screen.getPrimaryDisplay().scaleFactor) || 1;
  syWin = new BrowserWindow({
    show: false, width: Math.max(1, Math.round(W / sf)), height: Math.max(1, Math.round(H / sf)),
    webPreferences: {
      offscreen: true,
      preload: path.join(__dirname, '..', 'projector-preload.js'),
      contextIsolation: true, nodeIntegration: false,
    },
  });
  syWin.loadFile(path.join(__dirname, '..', 'renderer', 'projector.html'), { search: 'clean=1' });
  syWin.webContents.setFrameRate(parseInt(cfg.fps, 10) || 30);
  try {
    syServer = new Syphon.SyphonOpenGLServer(cfg.name || 'LidarBridge-Mapping');
  } catch (e) {
    stopSyphon();
    return { ok: false, error: 'Syphon server failed: ' + e.message };
  }
  syWin.webContents.on('paint', (_e, _dirty, image) => {
    if (!syServer) return;
    const size = image.getSize();
    const bmp = image.getBitmap(); // Electron is BGRA; Syphon expects RGBA -> swizzle R/B
    const n = bmp.length;
    // fast swizzle: one 32-bit op per pixel (swap byte0<->byte2, keep G/A)
    const rgba = new Uint8ClampedArray(n);
    const src32 = new Uint32Array(bmp.buffer, bmp.byteOffset, n >> 2);
    const out32 = new Uint32Array(rgba.buffer);
    for (let i = 0; i < src32.length; i++) {
      const p = src32[i];
      out32[i] = (p & 0xff00ff00) | ((p & 0x000000ff) << 16) | ((p & 0x00ff0000) >> 16);
    }
    try {
      syServer.publishImageData(
        rgba,
        { x: 0, y: 0, width: size.width, height: size.height },
        { width: size.width, height: size.height },
        true, // flip to GL bottom-left origin
        'GL_TEXTURE_2D'
      );
    } catch (err) { /* ignore per-frame errors */ }
  });
  return { ok: true };
}
function stopSyphon() {
  if (syServer) { try { syServer.dispose && syServer.dispose(); } catch (_) {} syServer = null; }
  if (syWin && !syWin.isDestroyed()) syWin.close();
  syWin = null;
}
ipcMain.handle('lidar:syphon-start', async (_e, cfg) => startSyphon(cfg || {}));
ipcMain.handle('lidar:syphon-stop', async () => { stopSyphon(); return { ok: true }; });

// ---- network output -------------------------------------------------------
function startSender() {
  stopSender();
  peakSlots = 0;
  sender.configure({ host: outCfg.host, port: outCfg.port });
  const hz = Math.max(1, Math.min(120, outCfg.sendRate));
  sendTimer = setInterval(emitOutput, 1000 / hz);
}
function stopSender() {
  if (sendTimer) clearInterval(sendTimer);
  sendTimer = null;
}

function emitOutput() {
  if (!source && !fusionMode) return; // fusion has no single `source` — still emit
  const lines = [];
  // Coordinate is normalized u,v in [0,1] when warp "apply to output" is on (step 7),
  // otherwise raw world metres.
  const coord = (t) => (outCfg.normalize && t.u != null ? [t.u, t.v] : [t.x, t.y]);

  // INSTANCING mode (OSC only): fixed slots p0..pN with a stable channel set, ideal
  // for driving Geometry instancing in TouchDesigner. Empty slots send on=0.
  if (outCfg.protocol === 'osc' && outCfg.format === 'slots') {
    const ts = latestOut.tracks;
    // Send ONLY the active tracks (no padding). The channel set is exactly the current
    // count. NOTE: TouchDesigner's OSC In CHOP never forgets a channel it has seen, so
    // after a busy moment higher slots linger with stale values — Trim/cull by
    // /lidar/count in TD to keep exactly the active set.
    const msgs = [{ a: '/lidar/count', args: [{ type: 'i', value: ts.length }] }];
    for (let i = 0; i < ts.length; i++) {
      const t = ts[i];
      const [a, b] = coord(t);
      msgs.push({ a: `/lidar/p${i}/on`, args: [{ type: 'i', value: 1 }] });
      msgs.push({ a: `/lidar/p${i}/x`, args: [{ type: 'f', value: a }] });
      msgs.push({ a: `/lidar/p${i}/y`, args: [{ type: 'f', value: b }] });
      msgs.push({ a: `/lidar/p${i}/v`, args: [{ type: 'f', value: t.vel || 0 }] });
      msgs.push({ a: `/lidar/p${i}/id`, args: [{ type: 'i', value: parseInt(t.id, 10) }] });
    }
    sendZones(msgs, lines);
    sender.sendBundle(msgs); // one UDP packet
    lines.push(`/lidar/count  ${ts.length}`);
    ts.slice(0, 4).forEach((t, i) => { const [a, b] = coord(t); lines.push(`/lidar/p${i}  on 1  ${a.toFixed(2)} ${b.toFixed(2)}  v${(t.vel || 0).toFixed(2)}  id${t.id}`); });
    for (const z of latestOut.zones) lines.push(`/lidar/zone/${z.slug}  ${z.on ? 1 : 0}`);
    const nowS = Date.now();
    if (lines.length && nowS - lastLogAt > 140) { lastLogAt = nowS; send('lidar:osc-log', lines); }
    return;
  }

  if (outCfg.protocol === 'tuio') {
    const msgs = [];
    msgs.push(oscMessage('/tuio/2Dobj', [{ type: 's', value: 'source' }, { type: 's', value: 'LidarBridge' }]));
    const alive = [{ type: 's', value: 'alive' }];
    for (const t of latestOut.tracks) alive.push({ type: 'i', value: parseInt(t.id, 10) });
    msgs.push(oscMessage('/tuio/2Dobj', alive));
    for (const t of latestOut.tracks) {
      const [a, b] = coord(t);
      const sid = parseInt(t.id, 10);
      msgs.push(oscMessage('/tuio/2Dobj', [
        { type: 's', value: 'set' }, { type: 'i', value: sid }, { type: 'i', value: sid },
        { type: 'f', value: a }, { type: 'f', value: b }, { type: 'f', value: 0 },
        { type: 'f', value: t.vx || 0 }, { type: 'f', value: t.vy || 0 }, { type: 'f', value: 0 },
        { type: 'f', value: t.vel || 0 }, { type: 'f', value: 0 },
      ]));
      lines.push(`/tuio/2Dobj set ${sid} ${a.toFixed(3)} ${b.toFixed(3)}`);
    }
    msgs.push(oscMessage('/tuio/2Dobj', [{ type: 's', value: 'fseq' }, { type: 'i', value: ++tuioFseq }]));
    sender.sendRaw(oscBundle(msgs));
  } else {
    for (const t of latestOut.tracks) {
      const [a, b] = coord(t);
      const addr = `/lidar/trk/${t.id}`;
      sender.sendMessage(addr, [{ type: 'f', value: a }, { type: 'f', value: b }, { type: 'f', value: t.vel || 0 }]);
      lines.push(`${addr}  ${a.toFixed(outCfg.normalize ? 3 : 2)}  ${b.toFixed(outCfg.normalize ? 3 : 2)}  ${(t.vel || 0).toFixed(2)}`);
    }
    for (const z of latestOut.zones) {
      const addr = `/lidar/zone/${z.slug}`;
      sender.sendMessage(addr, [{ type: 'i', value: z.on ? 1 : 0 }]);
      lines.push(`${addr}  ${z.on ? 1 : 0}`);
    }
  }

  const now = Date.now();
  if (lines.length && now - lastLogAt > 140) {
    lastLogAt = now;
    send('lidar:osc-log', lines);
  }
}

function wireSource(src) {
  src.on('scan', onScan);
  src.on('status', (msg) => send('lidar:status', String(msg)));
  src.on('info', (info) => send('lidar:info', info));
  src.on('health', (h) => send('lidar:health', h));
  src.on('error', (err) => send('lidar:status', 'ERROR: ' + err.message));
}

// ---- auto-reconnect watchdog ---------------------------------------------
function startWatchdog() {
  stopWatchdog();
  watchdog = setInterval(() => {
    if (!autoReconnect || reconnecting || !source) return;
    if (lastScanAt && Date.now() - lastScanAt > STALL_MS) doReconnect();
  }, 1000);
}
function stopWatchdog() {
  if (watchdog) clearInterval(watchdog);
  watchdog = null;
}
async function doReconnect() {
  if (!autoReconnect || reconnecting || !lastConnectCfg) return;
  reconnecting = true;
  send('lidar:status', 'signal lost — reconnecting…');
  send('lidar:conn', { state: 'reconnecting' });
  while (autoReconnect && win && !win.isDestroyed()) {
    try {
      if (source) { source.removeAllListeners(); try { await source.disconnect(); } catch (_) {} source = null; }
      source = new RPLidar();
      wireSource(source);
      if (lastConnectCfg.network) await source.connect({ host: lastConnectCfg.host, port: lastConnectCfg.port, udp: lastConnectCfg.udp });
      else await source.connect({ path: lastConnectCfg.comPort, baudRate: parseInt(lastConnectCfg.baudrate, 10) || 115200 });
      lastScanAt = Date.now();
      startSender();
      send('lidar:status', 'reconnected');
      send('lidar:conn', { state: 'connected' });
      break;
    } catch (e) {
      send('lidar:status', 'reconnect failed, retrying… (' + e.message + ')');
      await new Promise((r) => setTimeout(r, 1500));
    }
  }
  reconnecting = false;
}

async function teardownSource() {
  stopSender();
  await teardownFusion();
  if (source) {
    try {
      await source.disconnect();
    } catch (_) {
      /* ignore */
    }
    source.removeAllListeners();
    source = null;
  }
}

// ---- IPC -----------------------------------------------------------------
ipcMain.handle('lidar:list-ports', async () => {
  try {
    const { SerialPort } = require('serialport');
    const ports = await SerialPort.list();
    return ports.map((p) => ({
      path: p.path,
      manufacturer: p.manufacturer || '',
      serialNumber: p.serialNumber || '',
      vendorId: p.vendorId || '',
      productId: p.productId || '',
    }));
  } catch (e) {
    return [];
  }
});

// Auto-detect: scan every serial port, try each candidate baudrate, and ask the
// device to identify itself (GET_INFO). Returns the RPLIDARs it found with the
// port + baudrate + model, so the UI can auto-fill the connection fields.
ipcMain.handle('lidar:autodetect', async () => {
  if (source) return { ok: false, error: 'Đang kết nối — hãy DISCONNECT trước khi dò.' };
  let SerialPort, probe;
  try {
    ({ SerialPort } = require('serialport'));
    ({ probe } = require('./rplidar'));
  } catch (e) {
    return { ok: false, error: 'serial unavailable: ' + e.message };
  }
  let ports = [];
  try { ports = await SerialPort.list(); } catch (_) {}
  // Skip obvious non-sensor ports (Bluetooth/debug) to keep the scan quick.
  const candidates = ports
    .map((p) => p.path)
    .filter((path) => path && !/Bluetooth|debug-console/i.test(path));
  send('lidar:status', `auto-detect: quét ${candidates.length} cổng…`);
  const bauds = [1000000, 256000, 115200]; // S2/S3 first, then A3/S1, then A1/A2
  const found = [];
  for (const path of candidates) {
    for (const baud of bauds) {
      send('lidar:status', `auto-detect: thử ${path} @ ${baud}…`);
      let info = null;
      try { info = await probe(path, baud); } catch (_) {}
      if (info) { found.push(info); break; } // got it — next port
    }
  }
  send('lidar:status', found.length ? `auto-detect: tìm thấy ${found.length} thiết bị` : 'auto-detect: không thấy RPLIDAR nào');
  return { ok: true, devices: found };
});

ipcMain.handle('lidar:connect', async (_evt, config) => {
  autoReconnect = false; reconnecting = false; stopWatchdog(); // clean slate
  await teardownSource();
  pipeline.setConfig({
    quality: config.quality,
    distMin: config.distMin,
    distMax: config.distMax,
    placement: config.placement,
  });
  lastScanAt = 0;

  const isSim =
    String(config.comPort).trim().toUpperCase() === 'SIM' ||
    config.connType === 'simulator';

  try {
    if (isSim) {
      source = new Simulator();
      wireSource(source);
      const info = await source.connect();
      startSender();
      return { ok: true, simulated: true, info };
    }
    source = new RPLidar();
    wireSource(source);
    let info;
    if (config.connType === 'network') {
      info = await source.connect({ host: config.ipAddr, port: config.ipPort, udp: config.netProto === 'udp' });
      lastConnectCfg = { network: true, host: config.ipAddr, port: config.ipPort, udp: config.netProto === 'udp' };
    } else {
      info = await source.connect({ path: config.comPort, baudRate: parseInt(config.baudrate, 10) || 115200 });
      lastConnectCfg = { comPort: config.comPort, baudrate: config.baudrate };
    }
    startSender();
    autoReconnect = true;
    startWatchdog();
    return { ok: true, simulated: false, info };
  } catch (e) {
    await teardownSource();
    return { ok: false, error: e.message };
  }
});

ipcMain.handle('lidar:disconnect', async () => {
  autoReconnect = false; // user-initiated -> don't auto-reconnect
  reconnecting = false;
  stopWatchdog();
  await teardownSource();
  return { ok: true };
});

ipcMain.handle('lidar:config', async (_evt, patch) => {
  pipeline.setConfig(patch);
  return { ok: true };
});

// ---- multi-sensor fusion IPC (F7) ----------------------------------------
// devices: [{ id, connType, comPort, baudrate, ipAddr, ipPort, netProto, pose }]
// common:  { distMin, distMax, quality }
ipcMain.handle('lidar:connect-fusion', async (_evt, payload) => {
  autoReconnect = false; reconnecting = false; stopWatchdog();
  await teardownSource();
  const { devices = [], common = {} } = payload || {};
  if (!devices.length) return { ok: false, error: 'no devices' };
  // apply shared acquisition config to every surface's pipeline
  for (const surf of surfaces) surf.pipeline.setConfig({ quality: common.quality, distMin: common.distMin, distMax: common.distMax });
  fusionMode = true;
  const connected = [];
  for (const d of devices) {
    const isSim = String(d.comPort).trim().toUpperCase() === 'SIM' || d.connType === 'simulator';
    const sensor = isSim ? new Simulator() : new RPLidar();
    const id = d.id;
    sensor.on('scan', (nodes) => { fusionScans.set(id, nodes); lastScanAt = Date.now(); });
    sensor.on('status', (msg) => send('lidar:status', d.name + ': ' + String(msg)));
    sensor.on('error', (err) => send('lidar:status', d.name + ' ERROR: ' + err.message));
    try {
      if (isSim) await sensor.connect();
      else if (d.connType === 'network') await sensor.connect({ host: d.ipAddr, port: d.ipPort, udp: d.netProto === 'udp' });
      else await sensor.connect({ path: d.comPort, baudRate: parseInt(d.baudrate, 10) || 1000000 });
      fusionSources.set(id, { sensor, pose: d.pose || {}, enabled: d.enabled !== false });
      connected.push(d.name || id);
    } catch (e) {
      send('lidar:status', (d.name || id) + ' failed: ' + e.message);
    }
  }
  if (!fusionSources.size) { fusionMode = false; return { ok: false, error: 'không kết nối được sensor nào' }; }
  fusionLast = 0;
  const hz = outCfg.sendRate && outCfg.sendRate > 0 ? Math.min(60, outCfg.sendRate) : 30;
  fusionTimer = setInterval(fusionTick, Math.round(1000 / hz)); // tick emits per-surface OSC
  send('lidar:status', `FUSION: ${fusionSources.size} sensor — ${connected.join(', ')}`);
  return { ok: true, connected: fusionSources.size };
});

// ---- surfaces IPC (v3) ----------------------------------------------------
function surfaceInfo(s) {
  return { id: s.id, name: s.name, oscPrefix: s.oscPrefix, sensorIds: s.sensorIds.slice(), ndi: s.ndi, active: s.id === activeSurfaceId };
}
ipcMain.handle('lidar:surfaces-list', async () => ({ ok: true, surfaces: surfaces.map(surfaceInfo), activeId: activeSurfaceId }));
ipcMain.handle('lidar:surface-add', async (_e, { name, oscPrefix } = {}) => {
  const s = makeSurface(name, oscPrefix);
  // new surface inherits the shared acquisition config
  s.pipeline.setConfig({ quality: pipeline.cfg.quality, distMin: pipeline.cfg.distMin, distMax: pipeline.cfg.distMax });
  surfaces.push(s);
  return { ok: true, surfaces: surfaces.map(surfaceInfo), id: s.id };
});
ipcMain.handle('lidar:surface-remove', async (_e, { id }) => {
  if (surfaces.length <= 1) return { ok: false, error: 'cần ít nhất 1 mặt' };
  surfaces = surfaces.filter((s) => s.id !== id);
  if (activeSurfaceId === id) { activeSurfaceId = surfaces[0].id; pipeline = surfaces[0].pipeline; }
  return { ok: true, surfaces: surfaces.map(surfaceInfo), activeId: activeSurfaceId };
});
ipcMain.handle('lidar:surface-update', async (_e, { id, name, oscPrefix, sensorIds, ndi }) => {
  const s = surfaces.find((x) => x.id === id);
  if (!s) return { ok: false, error: 'no surface' };
  if (name !== undefined) s.name = name;
  if (oscPrefix !== undefined) s.oscPrefix = oscPrefix.replace(/[^a-z0-9_]/gi, '');
  if (Array.isArray(sensorIds)) s.sensorIds = sensorIds.slice();
  if (ndi) s.ndi = Object.assign(s.ndi, ndi);
  return { ok: true, surfaces: surfaces.map(surfaceInfo) };
});
// Switch which surface the warp/zones/transform/display act on. Returns that
// surface's warp corners + zones so the renderer can load them into the UI.
ipcMain.handle('lidar:surface-select', async (_e, { id }) => {
  const s = surfaces.find((x) => x.id === id);
  if (!s) return { ok: false, error: 'no surface' };
  activeSurfaceId = id;
  pipeline = s.pipeline; // existing warp/zones/config IPC now target this surface
  return {
    ok: true, activeId: id,
    warp: { corners: s.pipeline.warpCorners, enabled: s.pipeline.warpEnabled },
    zones: s.pipeline.zones.map((z) => ({ name: z.name, slug: z.slug, pts: z.pts })),
    mask: s.pipeline.mask.map((p) => [p[0], p[1]]),
  };
});
// Export/import ALL surfaces (for presets) — name, OSC prefix, assigned sensors,
// NDI res, warp, zones, acquisition cfg.
ipcMain.handle('lidar:surfaces-export', async () => ({
  ok: true,
  surfaces: surfaces.map((s) => ({
    name: s.name, oscPrefix: s.oscPrefix, sensorIds: s.sensorIds.slice(), ndi: s.ndi,
    warp: { corners: s.pipeline.warpCorners, enabled: s.pipeline.warpEnabled },
    zones: s.pipeline.zones.map((z) => ({ name: z.name, slug: z.slug, pts: z.pts })),
    mask: s.pipeline.mask.map((p) => [p[0], p[1]]),
    cfg: { distMin: s.pipeline.cfg.distMin, distMax: s.pipeline.cfg.distMax, quality: s.pipeline.cfg.quality },
  })),
}));
ipcMain.handle('lidar:surfaces-import', async (_e, arr) => {
  if (!Array.isArray(arr) || !arr.length) return { ok: false, error: 'empty' };
  surfaces = arr.map((d) => {
    const s = makeSurface(d.name, d.oscPrefix);
    s.sensorIds = Array.isArray(d.sensorIds) ? d.sensorIds.slice() : [];
    if (d.ndi) s.ndi = Object.assign(s.ndi, d.ndi);
    if (d.cfg) s.pipeline.setConfig(d.cfg);
    if (d.warp) s.pipeline.setWarp({ corners: d.warp.corners, enabled: d.warp.enabled });
    if (Array.isArray(d.zones)) s.pipeline.setZones(d.zones);
    if (Array.isArray(d.mask)) s.pipeline.setMask(d.mask);
    return s;
  });
  activeSurfaceId = surfaces[0].id; pipeline = surfaces[0].pipeline;
  return { ok: true, surfaces: surfaces.map(surfaceInfo), activeId: activeSurfaceId };
});

// live-update one sensor's pose (position/rotation/scale) during fusion
ipcMain.handle('lidar:sensor-pose', async (_evt, { id, pose }) => {
  const s = fusionSources.get(id);
  if (s) s.pose = pose || {};
  return { ok: true };
});

// enable/disable one sensor's contribution to the fusion (motor stays on; just
// excluded from the merge — instant toggle, no reconnect).
ipcMain.handle('lidar:sensor-enable', async (_evt, { id, on }) => {
  const s = fusionSources.get(id);
  if (s) s.enabled = !!on;
  return { ok: true };
});

// background capture/clear for fusion: target one sensor id, or all when id omitted
ipcMain.handle('lidar:sensor-bg', async (_evt, { id, action }) => {
  const ids = id ? [id] : [...fusionSources.keys()];
  for (const sid of ids) {
    if (action === 'clear') pipeline.clearBackgroundSensor(sid);
    else pipeline.captureBackgroundSensor(sid);
  }
  return { ok: true };
});

ipcMain.handle('lidar:bg-capture', async () => {
  pipeline.captureBackground();
  return { ok: true };
});

ipcMain.handle('lidar:bg-clear', async () => {
  pipeline.clearBackground();
  return { ok: true };
});

ipcMain.handle('lidar:zones', async (_evt, zones) => {
  pipeline.setZones(zones);
  return { ok: true };
});

// track mask (include polygon) for the active surface
ipcMain.handle('lidar:mask', async (_evt, pts) => {
  pipeline.setMask(pts);
  return { ok: true };
});

// Replays a recorded take through the same pipeline, emitting `scan` like a real
// source (loops). Reuses the EventEmitter source contract.
const { EventEmitter } = require('events');
class PlaybackSource extends EventEmitter {
  constructor(frames) { super(); this.frames = frames; this.connected = false; this._timer = null; this.info = { firmware: 'PLAYBACK', model: 0 }; }
  async connect() {
    this.connected = true;
    this.emit('info', this.info);
    this.emit('status', 'playing take');
    let i = 0;
    const dur = this.frames.length ? this.frames[this.frames.length - 1].t + 100 : 100;
    const t0 = Date.now();
    this._timer = setInterval(() => {
      const elapsed = (Date.now() - t0) % dur;
      // emit all frames whose timestamp passed since last tick (cheap: advance index)
      while (i < this.frames.length && this.frames[i].t <= elapsed) { this.emit('scan', this.frames[i].nodes); i++; }
      if (i >= this.frames.length) i = 0; // loop
    }, 33);
    return this.info;
  }
  async disconnect() { this.connected = false; if (this._timer) clearInterval(this._timer); this._timer = null; }
}

ipcMain.handle('lidar:record-start', async () => {
  recording = true; recStart = Date.now(); recFrames = [];
  return { ok: true };
});
ipcMain.handle('lidar:record-stop', async () => {
  recording = false;
  if (!recFrames.length) return { ok: true, take: null };
  const durMs = recFrames[recFrames.length - 1].t;
  const id = ++takeSeq;
  const take = { id, name: 'Take ' + String(id).padStart(2, '0'), durMs, frames: recFrames };
  takes.push(take);
  recFrames = [];
  return { ok: true, take: { id: take.id, name: take.name, durMs } };
});
ipcMain.handle('lidar:play-take', async (_evt, id) => {
  const take = takes.find((t) => t.id === id);
  if (!take) return { ok: false, error: 'take not found' };
  await teardownSource();
  source = new PlaybackSource(take.frames);
  wireSource(source);
  await source.connect();
  startSender();
  return { ok: true, durMs: take.durMs };
});

ipcMain.handle('lidar:open-output', async (_evt, mode) => {
  openProjector(mode);
  return { ok: true };
});
ipcMain.handle('lidar:close-output', async () => {
  if (projWin && !projWin.isDestroyed()) projWin.close();
  return { ok: true };
});

// Native NDI sender: off-screen render of the clean mapping -> NDI runtime (koffi).
function startNdi(cfg) {
  let NdiSender;
  try { ({ NdiSender } = require('./ndi')); } catch (e) { return { ok: false, error: 'koffi unavailable: ' + e.message }; }
  stopNdi();
  const W = parseInt(cfg.w, 10) || 1280;
  const H = parseInt(cfg.h, 10) || 720;
  ndiSender = new NdiSender();
  try {
    ndiSender.start(cfg.name || 'LidarBridge-Mapping', W, H, cfg.fps);
  } catch (e) {
    ndiSender = null;
    return { ok: false, error: e.message };
  }
  const fps = parseInt(cfg.fps, 10) || 30;

  // NATIVE mode: render the mapping into a fixed W×H canvas inside a hidden page and
  // stream raw RGBA pixels here via IPC — exact resolution, not capped to the screen
  // (heavier: ~W*H*4 bytes/frame over IPC). Used when the user wants true projector px.
  if (cfg.native) {
    ndiNative = true;
    ndiWin = new BrowserWindow({
      show: false, width: 200, height: 200,
      webPreferences: { preload: path.join(__dirname, '..', 'projector-preload.js'), contextIsolation: true, nodeIntegration: false },
    });
    ndiWin.loadFile(path.join(__dirname, '..', 'renderer', 'projector.html'), { search: `clean=1&nw=${W}&nh=${H}&fps=${fps}` });
    console.log(`NDI native: streaming exact ${W}x${H} @ ${fps}fps via IPC`);
    return { ok: true, renderW: W, renderH: H, requestedW: W, requestedH: H, capped: false, native: true };
  }

  ndiNative = false;
  // An offscreen BrowserWindow's framebuffer is capped to the primary display's
  // pixel size (and a framed window loses ~48px to its title bar — which is why a
  // 2836x2660 request came out 1920x1032). So: render frameless, and shrink the
  // requested W×H to fit the screen *preserving aspect* (allow exact if it fits).
  // The NDI frame then always has the correct aspect (e.g. near-square 2836:2660);
  // TouchDesigner can upscale to the real projector resolution losslessly.
  const disp = screen.getPrimaryDisplay();
  const sf = disp.scaleFactor || 1;
  const screenPxW = Math.max(640, Math.round(disp.size.width * sf));
  const screenPxH = Math.max(480, Math.round(disp.size.height * sf));
  const fit = Math.min(1, screenPxW / W, screenPxH / H);
  const renderW = Math.max(2, Math.round(W * fit));
  const renderH = Math.max(2, Math.round(H * fit));
  const winW = Math.max(1, Math.round(renderW / sf));
  const winH = Math.max(1, Math.round(renderH / sf));
  if (fit < 1) console.log(`NDI: requested ${W}x${H} exceeds screen ${screenPxW}x${screenPxH}; rendering ${renderW}x${renderH} (same aspect) — upscale in TD`);
  ndiWin = new BrowserWindow({
    show: false, frame: false, useContentSize: true, width: winW, height: winH,
    webPreferences: { offscreen: true, preload: path.join(__dirname, '..', 'projector-preload.js'), contextIsolation: true, nodeIntegration: false },
  });
  ndiWin.loadFile(path.join(__dirname, '..', 'renderer', 'projector.html'), { search: 'clean=1' });
  ndiWin.webContents.setFrameRate(fps);
  ndiWin.webContents.on('paint', (_e, _dirty, image) => {
    if (!ndiSender) return;
    const size = image.getSize();
    const bmp = image.getBitmap(); // BGRA top-down -> NDI BGRA directly
    try { ndiSender.send(bmp, size.width, size.height); } catch (err) { /* ignore */ }
  });
  return { ok: true, renderW, renderH, requestedW: W, requestedH: H, capped: fit < 1, native: false };
}
function stopNdi() {
  if (ndiWin && !ndiWin.isDestroyed()) ndiWin.close();
  ndiWin = null;
  ndiNative = false;
  if (ndiSender) { ndiSender.stop(); ndiSender = null; }
}
// Native-mode RGBA frames streamed from the hidden render page.
ipcMain.on('ndi:frame', (_evt, msg) => {
  if (!ndiSender || !ndiNative || !msg || !msg.buf) return;
  try { ndiSender.send(Buffer.from(msg.buf), msg.w, msg.h, true); } catch (_) {}
});
ipcMain.handle('lidar:ndi-start', async (_evt, cfg) => startNdi(cfg || {}));
ipcMain.handle('lidar:ndi-stop', async () => { stopNdi(); return { ok: true }; });

ipcMain.handle('lidar:warp', async (_evt, patch) => {
  pipeline.setWarp(patch);
  if (patch.enabled !== undefined) outCfg.normalize = !!patch.enabled; // "apply to output"
  return { ok: true };
});

ipcMain.handle('lidar:output', async (_evt, patch) => {
  const rateChanged = patch.sendRate !== undefined && patch.sendRate !== outCfg.sendRate;
  if (patch.protocol !== undefined) outCfg.protocol = patch.protocol;
  if (patch.host !== undefined) outCfg.host = patch.host;
  if (patch.port !== undefined) outCfg.port = parseInt(patch.port, 10) || outCfg.port;
  if (patch.sendRate !== undefined) outCfg.sendRate = parseInt(patch.sendRate, 10) || outCfg.sendRate;
  if (patch.normalize !== undefined) outCfg.normalize = !!patch.normalize;
  if (patch.format !== undefined) outCfg.format = patch.format;
  sender.configure({ host: outCfg.host, port: outCfg.port });
  if (rateChanged && sendTimer) startSender();
  return { ok: true };
});

// ---- verification screenshot (no screen-capture permission needed) -------
async function runAutoShot() {
  const out = process.env.LIDAR_AUTOSHOT;
  try {
    await new Promise((r) => win.webContents.once('did-finish-load', r));
    await win.webContents.executeJavaScript("document.getElementById('connectBtn').click();");
    await new Promise((r) => setTimeout(r, 2600));
    if (process.env.LIDAR_SHOT_JS) {
      await win.webContents.executeJavaScript(process.env.LIDAR_SHOT_JS);
      await new Promise((r) => setTimeout(r, parseInt(process.env.LIDAR_SHOT_WAIT || '1200', 10)));
    }
    let target = win;
    if (process.env.LIDAR_SHOT_PROJ && projWin && !projWin.isDestroyed()) target = projWin;
    try {
      const lat = await win.webContents.executeJavaScript("document.getElementById('latency').textContent");
      const fps = await win.webContents.executeJavaScript("document.getElementById('fps').textContent");
      console.log('LATENCY_MS ' + lat + ' FPS ' + fps);
    } catch (_) { /* ignore */ }
    const img = await target.webContents.capturePage();
    fs.writeFileSync(out, img.toPNG());
    console.log('SHOT_SAVED ' + out);
  } catch (e) {
    console.error('AUTOSHOT_ERROR', e);
  }
  app.quit();
}

// ---- preset save / load --------------------------------------------------
async function savePreset() {
  if (!win) return;
  const res = await dialog.showSaveDialog(win, {
    title: 'Save preset', defaultPath: 'lidar-bridge.json',
    filters: [{ name: 'LiDAR Bridge preset', extensions: ['json'] }],
  });
  if (res.canceled || !res.filePath) return;
  let state = {};
  try { state = await win.webContents.executeJavaScript('window.__collectPreset && window.__collectPreset()'); } catch (_) {}
  state = state || {};
  state.baseline = pipeline.bgCaptured ? Array.from(pipeline.bg) : null;
  fs.writeFileSync(res.filePath, JSON.stringify(state, null, 2));
  send('lidar:status', 'preset saved: ' + path.basename(res.filePath));
}

async function openPreset() {
  if (!win) return;
  const res = await dialog.showOpenDialog(win, {
    title: 'Open preset', properties: ['openFile'],
    filters: [{ name: 'LiDAR Bridge preset', extensions: ['json'] }],
  });
  if (res.canceled || !res.filePaths || !res.filePaths.length) return;
  let obj;
  try { obj = JSON.parse(fs.readFileSync(res.filePaths[0], 'utf8')); } catch (e) { send('lidar:status', 'preset load failed: ' + e.message); return; }
  if (obj.baseline) pipeline.setBaseline(obj.baseline);
  try { await win.webContents.executeJavaScript('window.__applyPreset(' + JSON.stringify(obj) + ')'); } catch (e) { send('lidar:status', 'apply failed: ' + e.message); }
  send('lidar:status', 'preset loaded: ' + path.basename(res.filePaths[0]));
}

function buildMenu() {
  const isMac = process.platform === 'darwin';
  const template = [
    ...(isMac ? [{ role: 'appMenu' }] : []),
    {
      label: 'File',
      submenu: [
        { label: 'Save Preset…', accelerator: 'CmdOrCtrl+S', click: savePreset },
        { label: 'Open Preset…', accelerator: 'CmdOrCtrl+O', click: openPreset },
        { type: 'separator' },
        isMac ? { role: 'close' } : { role: 'quit' },
      ],
    },
    { role: 'editMenu' },
    { role: 'viewMenu' },
    { role: 'windowMenu' },
  ];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

app.whenReady().then(() => { buildMenu(); createWindow(); });
app.on('window-all-closed', async () => {
  await teardownSource();
  if (process.platform !== 'darwin') app.quit();
});
app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
