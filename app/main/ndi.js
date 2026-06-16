'use strict';
// Native NDI® sender via koffi FFI to the NDI runtime library (no compiled addon).
// Requires the free "NDI Runtime" / "NDI Tools" to be installed on the machine:
//   Windows: Processing.NDI.Lib.x64.dll (the installer sets NDI_RUNTIME_DIR_V6/PATH)
//   macOS:   libndi.dylib in /usr/local/lib (installed by NDI Tools for Mac)
// Electron's getBitmap() is BGRA top-down, which maps directly to NDI's BGRA FourCC —
// no swizzle or flip needed.

const koffi = require('koffi');
const path = require('path');

// Struct type registrations are global (independent of which lib we load).
koffi.struct('NDIlib_send_create_t', {
  p_ndi_name: 'const char *',
  p_groups: 'const char *',
  clock_video: 'bool',
  clock_audio: 'bool',
});
koffi.struct('NDIlib_video_frame_v2_t', {
  xres: 'int',
  yres: 'int',
  FourCC: 'int',
  frame_rate_N: 'int',
  frame_rate_D: 'int',
  picture_aspect_ratio: 'float',
  frame_format_type: 'int',
  timecode: 'int64_t',
  p_data: 'uint8_t *',
  line_stride_in_bytes: 'int',
  p_metadata: 'const char *',
  timestamp: 'int64_t',
});

const fourcc = (s) => s.charCodeAt(0) | (s.charCodeAt(1) << 8) | (s.charCodeAt(2) << 16) | (s.charCodeAt(3) << 24);
const FOURCC_BGRA = fourcc('BGRA');
const FOURCC_RGBA = fourcc('RGBA'); // canvas getImageData is RGBA
const FRAME_FORMAT_PROGRESSIVE = 1;

function libCandidates() {
  const p = process.platform;
  // directories next to the executable / resources (drop the runtime here = no install),
  // plus the project's ndi-runtime/<plat> folder so `npm start` (dev) finds it too.
  const exeDir = path.dirname(process.execPath);
  const plat = p === 'win32' ? 'win' : p === 'darwin' ? 'mac' : 'linux';
  const appDirs = [exeDir, process.resourcesPath || exeDir, process.cwd(),
    path.join(__dirname, '..', 'ndi-runtime', plat)];
  if (p === 'win32') {
    const list = [];
    for (const v of ['NDI_RUNTIME_DIR_V6', 'NDI_RUNTIME_DIR_V5', 'NDI_RUNTIME_DIR_V4']) {
      if (process.env[v]) list.push(process.env[v].replace(/\\$/, '') + '\\Processing.NDI.Lib.x64.dll');
    }
    for (const d of appDirs) list.push(path.join(d, 'Processing.NDI.Lib.x64.dll'));
    list.push('Processing.NDI.Lib.x64.dll'); // on PATH after install
    return list;
  }
  if (p === 'darwin') {
    const list = [];
    for (const d of appDirs) list.push(path.join(d, 'libndi.dylib'));
    list.push('/usr/local/lib/libndi.dylib', '/usr/local/lib/libndi.4.dylib', '/opt/homebrew/lib/libndi.dylib', 'libndi.dylib');
    return list;
  }
  const list = [];
  for (const d of appDirs) list.push(path.join(d, 'libndi.so'));
  list.push('libndi.so', 'libndi.so.5', 'libndi.so.4', '/usr/lib/libndi.so');
  return list;
}

class NdiSender {
  constructor() { this.lib = null; this.inst = null; }

  start(name, w, h, fps) {
    let lastErr = 'not found';
    for (const c of libCandidates()) {
      try { this.lib = koffi.load(c); break; } catch (e) { lastErr = e.message; this.lib = null; }
    }
    if (!this.lib) throw new Error('NDI runtime not found (install NDI Tools / NDI Runtime). ' + lastErr);

    const init = this.lib.func('bool NDIlib_initialize()');
    this._create = this.lib.func('void *NDIlib_send_create(NDIlib_send_create_t *)');
    this._sendVideo = this.lib.func('void NDIlib_send_send_video_v2(void *, NDIlib_video_frame_v2_t *)');
    this._destroy = this.lib.func('void NDIlib_send_destroy(void *)');

    if (!init()) throw new Error('NDIlib_initialize() failed');
    // clock_video MUST be false: when true, NDIlib_send_send_video_v2 *sleeps* inside
    // the call to pace frames to the rate — and we call it synchronously on the main
    // thread, which stalls the scan/OSC loop (latency spikes to ~2x). We already pace
    // via webContents.setFrameRate(fps) on the offscreen window, so let send() return now.
    this.inst = this._create({ p_ndi_name: name || 'LidarBridge', p_groups: null, clock_video: false, clock_audio: false });
    if (!this.inst) throw new Error('NDIlib_send_create() failed');
    this.fps = parseInt(fps, 10) || 30;
  }

  send(bgra, w, h, rgba) {
    if (!this.inst) return;
    this._sendVideo(this.inst, {
      xres: w,
      yres: h,
      FourCC: rgba ? FOURCC_RGBA : FOURCC_BGRA,
      frame_rate_N: this.fps,
      frame_rate_D: 1,
      picture_aspect_ratio: 0, // 0 => square pixels (xres/yres)
      frame_format_type: FRAME_FORMAT_PROGRESSIVE,
      timecode: 0,
      p_data: bgra,
      line_stride_in_bytes: w * 4,
      p_metadata: null,
      timestamp: 0,
    });
  }

  stop() {
    try { if (this.inst && this._destroy) this._destroy(this.inst); } catch (_) {}
    this.inst = null;
  }
}

module.exports = { NdiSender };
