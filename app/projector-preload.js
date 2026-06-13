'use strict';
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('projector', {
  onFrame: (cb) => { const h = (_e, f) => cb(f); ipcRenderer.on('projector:frame', h); },
  exit: () => ipcRenderer.send('projector:exit'),
  toggleFullscreen: () => ipcRenderer.send('projector:fullscreen'),
});
