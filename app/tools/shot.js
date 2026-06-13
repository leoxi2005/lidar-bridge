'use strict';
// Headless-ish verification: load the renderer, connect to the SIM source,
// let a few scans render, then save a PNG via capturePage. No screen perms needed.
//   electron tools/shot.js [outfile]
const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const { Simulator } = require('../main/simulator');

const OUT = process.argv[2] || '/tmp/lidar_shot.png';
let win, sim;

function onScan(nodes) {
  const out = new Float32Array(nodes.length * 2);
  let n = 0;
  for (const node of nodes) {
    const d = node.distMm / 1000;
    if (d <= 0) continue;
    out[n * 2] = node.angle;
    out[n * 2 + 1] = d;
    n++;
  }
  if (win && !win.isDestroyed()) {
    win.webContents.send('lidar:scan', { points: out.subarray(0, n * 2), count: n, periodMs: 100 });
  }
}

ipcMain.handle('lidar:list-ports', async () => []);
ipcMain.handle('lidar:connect', async () => {
  sim = new Simulator();
  sim.on('scan', onScan);
  await sim.connect();
  return { ok: true, simulated: true, info: sim.info };
});
ipcMain.handle('lidar:disconnect', async () => ({ ok: true }));
ipcMain.handle('lidar:config', async () => ({ ok: true }));

app.whenReady().then(async () => {
  win = new BrowserWindow({
    width: 1440, height: 900, show: true, backgroundColor: '#0a0b0e',
    webPreferences: { preload: path.join(__dirname, '..', 'preload.js'), contextIsolation: true, nodeIntegration: false },
  });
  await win.loadFile(path.join(__dirname, '..', 'renderer', 'index.html'));
  // Click CONNECT (defaults to SIM port), then let it run.
  await win.webContents.executeJavaScript("document.getElementById('connectBtn').click();");
  await new Promise((r) => setTimeout(r, 2500));
  const img = await win.webContents.capturePage();
  fs.writeFileSync(OUT, img.toPNG());
  console.log('SHOT_SAVED ' + OUT);
  app.quit();
});
