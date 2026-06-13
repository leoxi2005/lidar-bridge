'use strict';
const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const { RPLidar } = require('./rplidar');
const { Simulator } = require('./simulator');
const { Pipeline } = require('./pipeline');
const { OscSender, oscMessage, oscBundle } = require('./osc');

let win = null;
let source = null; // active RPLidar | Simulator
const pipeline = new Pipeline();
let lastScanAt = 0;

// network output
const sender = new OscSender();
let outCfg = { protocol: 'osc', host: '127.0.0.1', port: 7000, sendRate: 30, normalize: false };
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
  win.on('closed', () => (win = null));
  if (process.env.LIDAR_AUTOSHOT) runAutoShot();
}

function send(channel, payload) {
  if (win && !win.isDestroyed()) win.webContents.send(channel, payload);
}

// Full pipeline pass for one scan, then stream the frame to the renderer.
function onScan(nodes) {
  const now = Date.now();
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
}

// ---- network output -------------------------------------------------------
function startSender() {
  stopSender();
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
    const img = await win.webContents.capturePage();
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
