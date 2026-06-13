'use strict';
const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const { RPLidar } = require('./rplidar');
const { Simulator } = require('./simulator');

let win = null;
let source = null; // active RPLidar | Simulator

// Live acquisition config mirrored from the UI. Used for the FILTER stage.
let cfg = {
  quality: false, // quality filter on/off
  distMin: 0.0, // meters
  distMax: 25.0, // meters
};

let lastScanAt = 0;

function createWindow() {
  win = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1100,
    minHeight: 680,
    backgroundColor: '#0a0b0e',
    title: 'LiDAR Bridge',
    webPreferences: {
      preload: path.join(__dirname, '..', 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });
  win.loadFile(path.join(__dirname, '..', 'renderer', 'index.html'));
  win.on('closed', () => (win = null));
}

function send(channel, payload) {
  if (win && !win.isDestroyed()) win.webContents.send(channel, payload);
}

// FILTER + TRANSFORM(mm->m) stage. Emits an interleaved Float32Array of
// [angleDeg, distMeters, ...] so the renderer can apply live placement.
function onScan(nodes) {
  const out = new Float32Array(nodes.length * 2);
  let n = 0;
  for (const node of nodes) {
    const distM = node.distMm / 1000;
    if (distM <= 0) continue;
    if (cfg.quality && node.quality < 150) continue;
    if (distM < cfg.distMin || distM > cfg.distMax) continue;
    out[n * 2] = node.angle;
    out[n * 2 + 1] = distM;
    n++;
  }
  const buf = out.subarray(0, n * 2);
  const now = Date.now();
  const periodMs = lastScanAt ? now - lastScanAt : 0;
  lastScanAt = now;
  send('lidar:scan', { points: buf, count: n, periodMs });
}

function wireSource(src) {
  src.on('scan', onScan);
  src.on('status', (msg) => send('lidar:status', String(msg)));
  src.on('info', (info) => send('lidar:info', info));
  src.on('health', (h) => send('lidar:health', h));
  src.on('error', (err) => send('lidar:status', 'ERROR: ' + err.message));
}

async function teardownSource() {
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
  cfg = {
    quality: !!config.quality,
    distMin: parseFloat(config.distMin) || 0,
    distMax: parseFloat(config.distMax) || 25,
  };
  lastScanAt = 0;

  const isSim =
    String(config.comPort).trim().toUpperCase() === 'SIM' ||
    config.connType === 'simulator';

  try {
    if (isSim) {
      source = new Simulator();
      wireSource(source);
      const info = await source.connect();
      return { ok: true, simulated: true, info };
    }
    if (config.connType === 'network') {
      throw new Error('Network (TCP/UDP) acquisition is a later build step — use SERIAL or SIM for step 1.');
    }
    source = new RPLidar();
    wireSource(source);
    const info = await source.connect({
      path: config.comPort,
      baudRate: parseInt(config.baudrate, 10) || 115200,
    });
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

ipcMain.handle('lidar:config', async (_evt, config) => {
  if (config.quality !== undefined) cfg.quality = !!config.quality;
  if (config.distMin !== undefined) cfg.distMin = parseFloat(config.distMin) || 0;
  if (config.distMax !== undefined) cfg.distMax = parseFloat(config.distMax) || 25;
  return { ok: true };
});

app.whenReady().then(createWindow);
app.on('window-all-closed', async () => {
  await teardownSource();
  if (process.platform !== 'darwin') app.quit();
});
app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
