'use strict';
// Hokuyo UST-10LX / UST-20LX driver — speaks SCIP 2.0 over TCP (Ethernet).
// The RPLIDAR driver talks Slamtec's binary protocol; Hokuyo is a completely
// different, line-based ASCII protocol, so this is a separate driver. It exposes
// the SAME EventEmitter surface as RPLidar (`connect`/`disconnect` + 'scan',
// 'status', 'info', 'error' events) and emits the identical scan node shape
//   { angle (deg), distMm, quality }
// so the whole pipeline / OSC / fusion / background-subtract downstream is reused
// unchanged.
//
// SCIP 2.0 essentials (Hokuyo "Communication Protocol Specification"):
//   request : "<CMD>[params]\n"                       (LF-terminated ASCII line)
//   reply   : "<echo>\n<status><sum>\n[data lines...]\n\n"   (blank line ends it)
//   status  : 2 chars ("00" ok / "99" data / "02" already-on ...) + 1 checksum char
//   PP      : parameters (AMIN/AMAX/ARES/AFRT/DMIN/DMAX/MODL...)
//   BM      : switch the laser/measurement on
//   MD      : stream distance data continuously (3-char encoding, distance only)
//   QT      : stop measurement
//
// Distance decoding: after the timestamp line, each output line is up to 64 data
// characters followed by ONE checksum char; strip that trailing char per line,
// concatenate, then read 3 chars per point. Each char carries 6 bits (value-0x30).

const { EventEmitter } = require('events');
const net = require('net');

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

// UST-10LX / UST-20LX defaults, used only if PP can't be read.
const UST_DEFAULTS = { AMIN: 0, AMAX: 1080, ARES: 1440, AFRT: 540, DMIN: 20, DMAX: 60000 };
const DEFAULT_PORT = 10940;

const pad = (n, w) => String(n).padStart(w, '0');

// Decode one 3-character (18-bit) SCIP-encoded value at offset i of an ASCII string.
function dec3(s, i) {
  return ((s.charCodeAt(i) - 0x30) << 12) | ((s.charCodeAt(i + 1) - 0x30) << 6) | (s.charCodeAt(i + 2) - 0x30);
}

class Hokuyo extends EventEmitter {
  constructor() {
    super();
    this._sock = null;
    this.buffer = ''; // SCIP is line-based ASCII; accumulate as latin1 string
    this.connected = false;
    this.info = null;
    this.params = null;
    this._mode = 'cmd'; // 'cmd' = awaiting a command reply | 'scan' = streaming MD
    this._pending = null; // { resolve, reject } for the in-flight command
    this._startStep = UST_DEFAULTS.AMIN;
    this._rxBytes = 0;
    this._nodeCount = 0;
    this._diagTimer = null;
  }

  // cfg: { host, port } — UST is Ethernet only. port defaults to 10940.
  async connect(cfg = {}) {
    if (this._sock) await this.disconnect();
    const host = cfg.host;
    const port = parseInt(cfg.port, 10) || DEFAULT_PORT;
    if (!host) throw new Error('Hokuyo cần IP address (mặc định 192.168.0.10)');
    await this._openTcp(host, port);
    return this._init();
  }

  _openTcp(host, port) {
    return new Promise((resolve, reject) => {
      this._log(`TCP ${host}:${port}`);
      let settled = false;
      const timer = setTimeout(() => {
        if (settled) return;
        settled = true;
        try { this._sock && this._sock.destroy(); } catch (_) {}
        reject(new Error(`connect timeout ${host}:${port}`));
      }, 4000);
      this._sock = net.createConnection({ host, port }, () => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        this._sock.setNoDelay(true);
        resolve();
      });
      this._sock.on('data', (chunk) => this._onData(chunk));
      this._sock.on('error', (err) => {
        if (!settled) { settled = true; clearTimeout(timer); return reject(err); }
        if (this._pending) { const p = this._pending; this._pending = null; p.reject(err); }
        this.emit('error', err);
      });
      this._sock.on('close', () => { this.connected = false; this.emit('status', 'connection closed'); });
    });
  }

  async _init() {
    // Stop any measurement still running from a previous session, then flush.
    this._mode = 'cmd';
    this._writeLine('QT');
    await delay(80);
    this.buffer = '';

    // Ensure SCIP 2.0 (harmless on a device already in 2.0; ignore its reply).
    try { await this._command('SCIP2.0', 800); } catch (_) {}

    // Read device parameters so the angle mapping matches the exact model.
    try {
      const block = await this._command('PP', 1500);
      this.params = this._parseParams(block);
      this.info = {
        model: this.params.MODL || 'Hokuyo',
        firmware: this.params.PROT || '',
        serial: this.params.SERI || '',
      };
      this.emit('info', this.info);
      this._log(`model ${this.info.model} · steps ${this.params.AMIN}..${this.params.AMAX}`);
    } catch (e) {
      this.params = { ...UST_DEFAULTS };
      this._log('PP failed, using UST defaults (' + e.message + ')');
    }

    // Turn the laser on. 00 = ok, 02 = already on; anything else we still try.
    try { await this._command('BM', 1500); } catch (e) { this._log('BM: ' + e.message); }

    // Start continuous distance streaming across the full measurable arc.
    const p = this.params || UST_DEFAULTS;
    const amin = Number.isFinite(p.AMIN) ? p.AMIN : UST_DEFAULTS.AMIN;
    const amax = Number.isFinite(p.AMAX) ? p.AMAX : UST_DEFAULTS.AMAX;
    this._startStep = amin;
    // MD <start4><end4><cluster2><interval1><scans2>  (scans 00 = continuous)
    const md = 'MD' + pad(amin, 4) + pad(amax, 4) + '00' + '0' + '00';
    this._mode = 'scan';
    this._writeLine(md);
    this.connected = true;
    this.emit('status', 'scanning');

    // Diagnostics parity with the RPLIDAR driver: bytes/pts per second.
    this._rxBytes = 0; this._nodeCount = 0;
    if (this._diagTimer) clearInterval(this._diagTimer);
    this._diagTimer = setInterval(() => {
      this.emit('status', `scan rx ${this._rxBytes}B/s · ${this._nodeCount} pts/s`);
      this._rxBytes = 0; this._nodeCount = 0;
    }, 1000);
    return this.info;
  }

  async disconnect() {
    if (this._diagTimer) { clearInterval(this._diagTimer); this._diagTimer = null; }
    try { this._writeLine('QT'); await delay(20); } catch (_) {}
    try {
      if (this._sock) { this._sock.removeAllListeners(); this._sock.destroy(); }
    } catch (_) {}
    this._sock = null;
    this.connected = false;
    this.buffer = '';
    this.mode = 'cmd';
    this._pending = null;
  }

  // ---- internals ----------------------------------------------------------
  _log(msg) { this.emit('status', msg); }

  _writeLine(s) {
    if (this._sock && this._sock.writable) this._sock.write(s + '\n');
  }

  // Send a command and resolve with its full reply block (blank-line terminated).
  _command(cmd, timeoutMs = 1500) {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        if (this._pending && this._pending._t === timer) this._pending = null;
        reject(new Error('timeout: ' + cmd));
      }, timeoutMs);
      this._pending = {
        _t: timer,
        resolve: (block) => { clearTimeout(timer); resolve(block); },
        reject: (e) => { clearTimeout(timer); reject(e); },
      };
      this._writeLine(cmd);
    });
  }

  _onData(chunk) {
    this._rxBytes += chunk.length;
    this.buffer += chunk.toString('latin1');
    // Each SCIP reply is terminated by a blank line (two consecutive LFs). Data
    // lines are never empty, so "\n\n" reliably frames a complete reply.
    let idx;
    while ((idx = this.buffer.indexOf('\n\n')) >= 0) {
      const block = this.buffer.slice(0, idx);
      this.buffer = this.buffer.slice(idx + 2);
      if (this._mode === 'scan') {
        this._handleScanBlock(block);
      } else if (this._pending) {
        const p = this._pending; this._pending = null; p.resolve(block);
      }
      // else: stray reply in cmd mode with no waiter — drop it.
    }
  }

  // Parse a PP/VV reply: lines are "KEY:value;<sum>". Numeric values -> Number.
  _parseParams(block) {
    const out = {};
    for (const ln of block.split('\n')) {
      const m = ln.match(/^([A-Z]+):(.*?);/);
      if (!m) continue;
      const key = m[1];
      const val = m[2];
      out[key] = /^-?\d+$/.test(val) ? parseInt(val, 10) : val;
    }
    return out;
  }

  _handleScanBlock(block) {
    const lines = block.split('\n');
    // lines[0] = command echo, lines[1] = "<status2><sum>"
    const status = lines[1] ? lines[1].slice(0, 2) : '';
    if (status === '00') return;       // MD acknowledgement (no data yet)
    if (status !== '99') {             // 99 = data carrying; anything else is odd
      this._log('scan status ' + (status || '?'));
      return;
    }
    // lines[2] = timestamp; lines[3..] = data lines (each ends in a checksum char)
    let enc = '';
    for (let i = 3; i < lines.length; i++) {
      const ln = lines[i];
      if (ln.length < 2) continue;     // need at least 1 data char + checksum
      enc += ln.slice(0, ln.length - 1);
    }

    const p = this.params || UST_DEFAULTS;
    const AFRT = Number.isFinite(p.AFRT) ? p.AFRT : UST_DEFAULTS.AFRT;
    const ARES = Number.isFinite(p.ARES) ? p.ARES : UST_DEFAULTS.ARES;
    const DMIN = Number.isFinite(p.DMIN) ? p.DMIN : UST_DEFAULTS.DMIN;
    const start = this._startStep;
    const per = 360 / ARES; // degrees per step

    const n = Math.floor(enc.length / 3);
    const nodes = new Array(n);
    let w = 0;
    for (let i = 0; i < n; i++) {
      const d = dec3(enc, i * 3); // distance in mm
      if (d < DMIN) continue;     // 0..DMIN are Hokuyo error/no-echo codes
      let angle = (start + i - AFRT) * per; // 0 deg = sensor front, +CCW
      if (angle < 0) angle += 360;
      else if (angle >= 360) angle -= 360;
      nodes[w++] = { angle, distMm: d, quality: 255 }; // MD carries no intensity
    }
    nodes.length = w;
    this._nodeCount += w;
    if (w) this.emit('scan', nodes);
  }
}

// Best-effort network probe: open TCP, ask PP, resolve device info (or null).
// Mirrors rplidar.probe() so a future network auto-detect can reuse it.
function probe(host, port = DEFAULT_PORT, timeoutMs = 1200) {
  return new Promise((resolve) => {
    const dev = new Hokuyo();
    let done = false;
    const finish = (r) => { if (done) return; done = true; dev.disconnect().catch(() => {}); resolve(r); };
    const timer = setTimeout(() => finish(null), timeoutMs);
    dev.connect({ host, port })
      .then((info) => { clearTimeout(timer); finish(info ? { ...info, host, port, name: info.model } : null); })
      .catch(() => { clearTimeout(timer); finish(null); });
  });
}

module.exports = { Hokuyo, probe };
