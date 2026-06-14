'use strict';
const { app, BrowserWindow, ipcMain, screen } = require('electron');
const path = require('path');
const fs = require('fs');
const { RPLidar } = require('./rplidar');
const { Simulator } = require('./simulator');
const { Pipeline } = require('./pipeline');
const { OscSender, oscMessage, oscBundle } = require('./osc');
const { applyH } = require('./homography');

let projWin = null; // projector output window
let ndi = null; // NDI sender (optional, requires grandiose + NDI SDK)
let syServer = null; // Syphon server (macOS only, optional node-syphon)
let syWin = null; // offscreen window rendering the mapping for Syphon

// record / playback of raw scan sessions
let recording = false;
let recStart = 0;
let recFrames = [];
let takes = []; // [{ id, name, durMs, frames:[{t, nodes}] }]
let takeSeq = 0;

let win = null;
let source = null; // active RPLidar | Simulator
const pipeline = new Pipeline();
let lastScanAt = 0;

// network output
const sender = new OscSender();
let outCfg = { protocol: 'osc', host: '127.0.0.1', port: 7000, sendRate: 30, normalize: false, format: 'slots' };
const MAX_SLOTS = 64; // safety cap on instancing slots
let peakSlots = 0; // high-water mark of concurrent tracks this session (auto-sizes the slot set)
let sendTimer = null;
let tuioFseq = 0;
let lastLogAt = 0;
let latestOut = { tracks: [], zones: [] };

function createWindow() {
  win = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1100,
    minHeight: 680,
    backgroundColor: '#0a0b0e',
    title: 'LiDAR Bridge',
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
    zones: pipeline.zones.map((z, i) => ({ slug: z.slug, on: !!frame.zoneOcc[i] })),
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

// Send a frame to every output surface (visible projector window + offscreen Syphon render).
function broadcastProjectorFrame(payload) {
  if (projWin && !projWin.isDestroyed()) projWin.webContents.send('projector:frame', payload);
  if (syWin && !syWin.isDestroyed()) syWin.webContents.send('projector:frame', payload);
}

// ---- projector output window ---------------------------------------------
function openProjector(mode) {
  if (projWin && !projWin.isDestroyed()) { projWin.focus(); return; }
  const o = {
    width: 960, height: 600, frame: false, backgroundColor: '#000', title: 'LiDAR Bridge — Output',
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
  if (!source) return;
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
    sender.sendMessage('/lidar/count', [{ type: 'i', value: ts.length }]);
    for (let i = 0; i < ts.length; i++) {
      const t = ts[i];
      const [a, b] = coord(t);
      sender.sendMessage(`/lidar/p${i}/on`, [{ type: 'i', value: 1 }]);
      sender.sendMessage(`/lidar/p${i}/x`, [{ type: 'f', value: a }]);
      sender.sendMessage(`/lidar/p${i}/y`, [{ type: 'f', value: b }]);
      sender.sendMessage(`/lidar/p${i}/v`, [{ type: 'f', value: t.vel || 0 }]);
      sender.sendMessage(`/lidar/p${i}/id`, [{ type: 'i', value: parseInt(t.id, 10) }]);
    }
    for (const z of latestOut.zones) sender.sendMessage(`/lidar/zone/${z.slug}`, [{ type: 'i', value: z.on ? 1 : 0 }]);
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

async function teardownSource() {
  stopSender();
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
    }));
  } catch (e) {
    return [];
  }
});

ipcMain.handle('lidar:connect', async (_evt, config) => {
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
    if (config.connType === 'network') {
      throw new Error('Network (TCP/UDP) acquisition is a later build step — use SERIAL or SIM.');
    }
    source = new RPLidar();
    wireSource(source);
    const info = await source.connect({
      path: config.comPort,
      baudRate: parseInt(config.baudrate, 10) || 115200,
    });
    startSender();
    return { ok: true, simulated: false, info };
  } catch (e) {
    await teardownSource();
    return { ok: false, error: e.message };
  }
});

ipcMain.handle('lidar:disconnect', async () => {
  await teardownSource();
  return { ok: true };
});

ipcMain.handle('lidar:config', async (_evt, patch) => {
  pipeline.setConfig(patch);
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

// NDI broadcast requires the NDI SDK + the native `grandiose` binding, which are not
// bundled here. Load lazily and report cleanly if unavailable (no crash).
ipcMain.handle('lidar:ndi-start', async (_evt, cfg) => {
  try {
    const grandiose = require('grandiose'); // optional dependency
    ndi = await grandiose.send({ name: cfg.name || 'LidarBridge-Mapping' });
    return { ok: true, active: true };
  } catch (e) {
    return { ok: false, error: 'NDI broadcast needs the NDI SDK + `grandiose` native module (not installed in this build). WINDOW / EXTENDED output work without it.' };
  }
});
ipcMain.handle('lidar:ndi-stop', async () => {
  ndi = null;
  return { ok: true };
});

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
      console.log('LATENCY_MS ' + lat);
    } catch (_) { /* ignore */ }
    const img = await target.webContents.capturePage();
    fs.writeFileSync(out, img.toPNG());
    console.log('SHOT_SAVED ' + out);
  } catch (e) {
    console.error('AUTOSHOT_ERROR', e);
  }
  app.quit();
}

app.whenReady().then(createWindow);
app.on('window-all-closed', async () => {
  await teardownSource();
  if (process.platform !== 'darwin') app.quit();
});
app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
