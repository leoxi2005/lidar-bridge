'use strict';
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('lidar', {
  listPorts: () => ipcRenderer.invoke('lidar:list-ports'),
  autodetect: () => ipcRenderer.invoke('lidar:autodetect'),
  connect: (config) => ipcRenderer.invoke('lidar:connect', config),
  disconnect: () => ipcRenderer.invoke('lidar:disconnect'),
  setConfig: (config) => ipcRenderer.invoke('lidar:config', config),
  captureBg: () => ipcRenderer.invoke('lidar:bg-capture'),
  clearBg: () => ipcRenderer.invoke('lidar:bg-clear'),
  setZones: (zones) => ipcRenderer.invoke('lidar:zones', zones),
  setOutput: (cfg) => ipcRenderer.invoke('lidar:output', cfg),
  setWarp: (cfg) => ipcRenderer.invoke('lidar:warp', cfg),
  openOutput: (mode) => ipcRenderer.invoke('lidar:open-output', mode),
  closeOutput: () => ipcRenderer.invoke('lidar:close-output'),
  ndiStart: (cfg) => ipcRenderer.invoke('lidar:ndi-start', cfg),
  ndiStop: () => ipcRenderer.invoke('lidar:ndi-stop'),
  syphonStart: (cfg) => ipcRenderer.invoke('lidar:syphon-start', cfg),
  syphonStop: () => ipcRenderer.invoke('lidar:syphon-stop'),
  platform: process.platform,
  recordStart: () => ipcRenderer.invoke('lidar:record-start'),
  recordStop: () => ipcRenderer.invoke('lidar:record-stop'),
  playTake: (id) => ipcRenderer.invoke('lidar:play-take', id),
  onProjectorState: (cb) => {
    const h = (_e, s) => cb(s);
    ipcRenderer.on('lidar:projector-state', h);
    return () => ipcRenderer.removeListener('lidar:projector-state', h);
  },
  onOscLog: (cb) => {
    const h = (_e, lines) => cb(lines);
    ipcRenderer.on('lidar:osc-log', h);
    return () => ipcRenderer.removeListener('lidar:osc-log', h);
  },

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
