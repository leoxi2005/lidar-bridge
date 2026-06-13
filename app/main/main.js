'use strict';
const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const { RPLidar } = require('./rplidar');
const { Simulator } = require('./simulator');
const { Pipeline } = require('./pipeline');

let win = null;
let source = null; // active RPLidar | Simulator
const pipeline = new Pipeline();
let lastScanAt = 0;

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
