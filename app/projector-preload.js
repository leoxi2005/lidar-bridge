'use strict';
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('projector', {
  onFrame: (cb) => { const h = (_e, f) => cb(f); ipcRenderer.on('projector:frame', h); },
  exit: () => ipcRenderer.send('projector:exit'),
  toggleFullscreen: () => ipcRenderer.send('projector:fullscreen'),
  // native NDI: ship a raw RGBA frame buffer (ArrayBuffer) to main for NDI send
  ndiFrame: (buf, w, h) => ipcRenderer.send('ndi:frame', { w, h, buf }),
});
