'use strict';
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('lidar', {
  listPorts: () => ipcRenderer.invoke('lidar:list-ports'),
  connect: (config) => ipcRenderer.invoke('lidar:connect', config),
  disconnect: () => ipcRenderer.invoke('lidar:disconnect'),
  setConfig: (config) => ipcRenderer.invoke('lidar:config', config),
  captureBg: () => ipcRenderer.invoke('lidar:bg-capture'),
  clearBg: () => ipcRenderer.invoke('lidar:bg-clear'),

  onScan: (cb) => {
    const h = (_e, data) => cb(data);
    ipcRenderer.on('lidar:scan', h);
    return () => ipcRenderer.removeListener('lidar:scan', h);
  },
  onStatus: (cb) => {
    const h = (_e, msg) => cb(msg);
    ipcRenderer.on('lidar:status', h);
    return () => ipcRenderer.removeListener('lidar:status', h);
  },
  onInfo: (cb) => {
    const h = (_e, info) => cb(info);
    ipcRenderer.on('lidar:info', h);
    return () => ipcRenderer.removeListener('lidar:info', h);
  },
  onHealth: (cb) => {
    const h = (_e, x) => cb(x);
    ipcRenderer.on('lidar:health', h);
    return () => ipcRenderer.removeListener('lidar:health', h);
  },
});
