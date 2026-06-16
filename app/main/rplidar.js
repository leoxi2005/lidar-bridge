'use strict';
// Minimal RPLIDAR serial driver — implements the Slamtec wire protocol directly.
// The reference TouchDesigner plugin links Slamtec's sl_lidar SDK; Node has no such
// binding, so we speak the protocol over `serialport`. Step 1 uses the legacy SCAN
// command (0x20), which every RPLIDAR (A1/A2/A3/S-series) supports.
//
// Protocol reference (Slamtec "Interface Protocol"):
//   request    : 0xA5 <cmd> [payloadLen ...payload checksum]
//   descriptor : 0xA5 0x5A <len[30b]+mode[2b], 4 bytes LE> <dataType>   (7 bytes)
//   scan node  : 5 bytes  -> start flag, quality, angle_q6, distance_q2

const { EventEmitter } = require('events');
// Lazy-loaded so the app (and SIM mode) still boots if the native serial
// binding is unavailable on this machine.
let SerialPort = null;
function loadSerialPort() {
  if (!SerialPort) ({ SerialPort } = require('serialport'));
  return SerialPort;
}

const CMD = {
  STOP: 0x25,
  RESET: 0x40,
  SCAN: 0x20,
  FORCE_SCAN: 0x21,
  GET_INFO: 0x50,
  GET_HEALTH: 0x52,
  // Motor control. A-series accessory board uses SET_MOTOR_PWM (0xF0, pwm 0..1023,
  // default 660). S-series (S1/S2/S3) have an integrated motor driven by the HQ
  // motor-speed command (0xA8, rpm). The A-series DTR trick does NOT spin S-series.
  SET_MOTOR_PWM: 0xf0,
  HQ_MOTOR_SPEED: 0xa8,
};
const DEFAULT_MOTOR_PWM = 660; // Slamtec default PWM for the accessory board

const SYNC0 = 0xa5;
const SYNC1 = 0x5a;

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

class RPLidar extends EventEmitter {
  constructor() {
    super();
    this.port = null;
    this._sock = null;
    this._write = null;
    this._kind = 'serial';
    this.buffer = Buffer.alloc(0);
    this.mode = 'idle'; // 'idle' | 'response' | 'scan'
    this._pending = null; // { n, resolve, reject }
    this.connected = false;
    this.info = null;
    this.health = null;
    this._scan = []; // nodes accumulated for the current revolution
    // diagnostics — bytes/nodes seen since scan start (surfaced to the status line)
    this._rxBytes = 0;
    this._nodeCount = 0;
    this._diagTimer = null;
  }

  // ---- public API ---------------------------------------------------------
  // cfg: serial -> { path, baudRate }; network -> { host, port, udp }
  async connect(cfg) {
    if (this.port || this._sock) await this.disconnect();
    if (cfg.host) await this._openNetwork(cfg);
    else await this._openSerial(cfg);
    return this._init();
  }

  async _openSerial({ path, baudRate }) {
    this._kind = 'serial';
    this._log(`opening ${path} @ ${baudRate}`);
    const SP = loadSerialPort();
    this.port = new SP({ path, baudRate, autoOpen: false });
    this.port.on('data', (chunk) => this._onData(chunk));
    this.port.on('error', (err) => this.emit('error', err));
    this.port.on('close', () => { this.connected = false; this.emit('status', 'port closed'); });
    this._write = (buf) => this.port.write(buf);
    await new Promise((res, rej) => this.port.open((e) => (e ? rej(e) : res())));
    // Spin the motor: on A1 the motor runs when DTR is asserted low (false).
    await this._setSignals({ dtr: false });
  }

  async _openNetwork({ host, port, udp }) {
    port = parseInt(port, 10) || 8089;
    if (udp) {
      this._kind = 'udp';
      const dgram = require('dgram');
      this._log(`UDP ${host}:${port}`);
      this._sock = dgram.createSocket('udp4');
      this._sock.on('message', (msg) => this._onData(msg));
      this._sock.on('error', (err) => this.emit('error', err));
      this._write = (buf) => this._sock.send(buf, port, host);
      await new Promise((res) => this._sock.connect ? this._sock.connect(port, host, res) : res());
    } else {
      this._kind = 'tcp';
      const net = require('net');
      this._log(`TCP ${host}:${port}`);
      await new Promise((res, rej) => {
        this._sock = net.createConnection({ host, port }, res);
        this._sock.on('data', (chunk) => this._onData(chunk));
        this._sock.on('error', (err) => { this.emit('error', err); rej(err); });
        this._sock.on('close', () => { this.connected = false; this.emit('status', 'connection closed'); });
      });
      this._write = (buf) => this._sock.write(buf);
    }
  }

  async _init() {
    // Reset parser state, stop any scan already running, then probe the device.
    this._send(CMD.STOP);
    await delay(30);
    this.buffer = Buffer.alloc(0);
    this.mode = 'idle';

    try {
      this.info = await this._getInfo();
      this.emit('info', this.info);
    } catch (e) {
      this._log('getInfo failed (continuing): ' + e.message);
    }
    try {
      this.health = await this._getHealth();
      this.emit('health', this.health);
      if (this.health.status === 2) {
        this._log('device reports protection-stop; resetting');
        this._send(CMD.RESET);
        await delay(800);
        this.buffer = Buffer.alloc(0);
      }
    } catch (e) {
      this._log('getHealth failed (continuing): ' + e.message);
    }

    // Spin up the motor. We send BOTH motor commands unconditionally: the one the
    // device doesn't recognise is simply ignored (we don't await a response). This
    // covers A-series (PWM 0xF0) and S-series (HQ speed 0xA8) without model probing.
    this._startMotor();
    await delay(350); // give the motor time to reach speed before scanning

    await this._startScan();
    this.connected = true;
    this.emit('status', 'scanning');

    // Diagnostics: report bytes received + nodes/sec so we can tell whether data is
    // flowing at all (motor/scan issue) vs flowing-but-filtered (range/parse issue).
    this._rxBytes = 0; this._nodeCount = 0;
    if (this._diagTimer) clearInterval(this._diagTimer);
    this._diagTimer = setInterval(() => {
      this.emit('status', `scan rx ${this._rxBytes}B/s · ${this._nodeCount} pts/s`);
      this._rxBytes = 0; this._nodeCount = 0;
    }, 1000);
    return this.info;
  }

  _startMotor() {
    // A-series accessory board: PWM at default duty.
    this._send(CMD.SET_MOTOR_PWM, [DEFAULT_MOTOR_PWM & 0xff, (DEFAULT_MOTOR_PWM >> 8) & 0xff]);
    // S-series integrated motor: HQ speed control. 0xFFFF asks for the model default.
    this._send(CMD.HQ_MOTOR_SPEED, [0xff, 0xff]);
  }

  _stopMotor() {
    this._send(CMD.SET_MOTOR_PWM, [0x00, 0x00]);
    this._send(CMD.HQ_MOTOR_SPEED, [0x00, 0x00]);
  }

  async disconnect() {
    if (this._diagTimer) { clearInterval(this._diagTimer); this._diagTimer = null; }
    try {
      this._send(CMD.STOP);
      await delay(20);
      this._stopMotor();
      if (this._kind === 'serial') await this._setSignals({ dtr: true }); // stop motor (A1)
    } catch (_) {
      /* ignore */
    }
    try {
      if (this.port) await new Promise((res) => this.port.close(() => res()));
      else if (this._sock) { this._sock.removeAllListeners(); this._sock.destroy ? this._sock.destroy() : this._sock.close(); }
    } catch (_) { /* ignore */ }
    this.port = null;
    this._sock = null;
    this._write = null;
    this.connected = false;
    this.mode = 'idle';
    this.buffer = Buffer.alloc(0);
    this._scan = [];
  }

  // ---- internals ----------------------------------------------------------
  _log(msg) {
    this.emit('status', msg);
  }

  _setSignals(sig) {
    return new Promise((res) => {
      if (!this.port || !this.port.set) return res();
      this.port.set(sig, () => res());
    });
  }

  _send(cmd, payload) {
    let pkt;
    if (payload && payload.length) {
      pkt = Buffer.from([SYNC0, cmd, payload.length, ...payload]);
      let cs = 0;
      for (const b of pkt) cs ^= b;
      pkt = Buffer.concat([pkt, Buffer.from([cs])]);
    } else {
      pkt = Buffer.from([SYNC0, cmd]);
    }
    if (this._write) this._write(pkt);
  }

  _onData(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    if (this.mode === 'scan') {
      this._rxBytes += chunk.length;
      this._parseScan();
    } else if (this._pending && this.buffer.length >= this._pending.n) {
      const { n, resolve } = this._pending;
      const out = this.buffer.subarray(0, n);
      this.buffer = this.buffer.subarray(n);
      this._pending = null;
      resolve(Buffer.from(out));
    }
  }

  _readBytes(n, timeoutMs = 1500) {
    return new Promise((resolve, reject) => {
      if (this.buffer.length >= n) {
        const out = this.buffer.subarray(0, n);
        this.buffer = this.buffer.subarray(n);
        return resolve(Buffer.from(out));
      }
      const timer = setTimeout(() => {
        if (this._pending && this._pending.resolve === resolve) this._pending = null;
        reject(new Error(`timeout waiting for ${n} bytes`));
      }, timeoutMs);
      this._pending = {
        n,
        resolve: (buf) => {
          clearTimeout(timer);
          resolve(buf);
        },
        reject,
      };
    });
  }

  async _readDescriptor() {
    const d = await this._readBytes(7);
    if (d[0] !== SYNC0 || d[1] !== SYNC1) {
      throw new Error('bad response descriptor');
    }
    const lenMode = d.readUInt32LE(2);
    return { len: lenMode & 0x3fffffff, sendMode: lenMode >>> 30, dataType: d[6] };
  }

  async _getInfo() {
    this._send(CMD.GET_INFO);
    const desc = await this._readDescriptor();
    const p = await this._readBytes(desc.len); // 20 bytes
    const fwMinor = p[2];
    return {
      model: p[0],
      firmware: `${p[1]}.${fwMinor < 10 ? '0' + fwMinor : fwMinor}`,
      hardware: p[3],
      serial: Buffer.from(p.subarray(4, 20)).toString('hex').toUpperCase(),
    };
  }

  async _getHealth() {
    this._send(CMD.GET_HEALTH);
    const desc = await this._readDescriptor();
    const p = await this._readBytes(desc.len); // 3 bytes
    return { status: p[0], errorCode: (p[2] << 8) | p[1] };
  }

  async _startScan() {
    this._send(CMD.SCAN);
    const desc = await this._readDescriptor();
    // 0x81 = legacy standard scan (5-byte nodes), what this parser expects. Some
    // devices/modes answer with a different data type (e.g. express/dense). Warn but
    // keep going so the diagnostics can show whether bytes are flowing at all.
    if (desc.dataType !== 0x81) {
      this._log('warning: scan data type 0x' + desc.dataType.toString(16) + ' (expected 0x81 standard)');
    }
    this._scanDataType = desc.dataType;
    this._scan = [];
    this.mode = 'scan';
    this._parseScan(); // drain anything already buffered
  }

  _parseScan() {
    const buf = this.buffer;
    let i = 0;
    while (buf.length - i >= 5) {
      const b0 = buf[i];
      const b1 = buf[i + 1];
      const s = b0 & 0x1; // start flag
      const sInv = (b0 >> 1) & 0x1; // inverted start flag
      const check = b1 & 0x1; // must be 1
      if (s === sInv || check !== 1) {
        // Lost byte alignment — resync by skipping one byte.
        i += 1;
        continue;
      }
      const quality = b0 >> 2;
      const angleQ6 = ((buf[i + 2] << 7) | (b1 >> 1)) & 0x7fff;
      const distQ2 = (buf[i + 4] << 8) | buf[i + 3];
      const angle = angleQ6 / 64.0; // degrees
      const distMm = distQ2 / 4.0;

      if (s === 1 && this._scan.length) {
        this.emit('scan', this._scan);
        this._scan = [];
      }
      this._scan.push({ angle, distMm, quality });
      this._nodeCount += 1;
      i += 5;
    }
    this.buffer = buf.subarray(i);
  }
}

// Best-effort map of the GET_INFO model byte -> friendly name. Slamtec model
// numbers aren't fully published and vary by firmware, so unknown values fall
// back to a generic label with the raw byte. The baudrate that answered is a
// stronger hint (A1/A2=115200, A3/S1=256000, S2/S3/T1=1000000).
function modelName(model, baud) {
  const known = {
    0x18: 'RPLIDAR A1', 0x19: 'RPLIDAR A1',
    0x28: 'RPLIDAR A2',
    0x49: 'RPLIDAR A3', 0x41: 'RPLIDAR A3',
    0x61: 'RPLIDAR S1',
    0x71: 'RPLIDAR S2', 0x72: 'RPLIDAR S2',
    0x73: 'RPLIDAR S2E',
    0x81: 'RPLIDAR S3',
  };
  if (known[model]) return known[model];
  if (baud >= 1000000) return 'RPLIDAR S2/S3 (model 0x' + model.toString(16) + ')';
  if (baud >= 256000) return 'RPLIDAR A3/S1 (model 0x' + model.toString(16) + ')';
  return 'RPLIDAR A1/A2 (model 0x' + model.toString(16) + ')';
}

// Open a port at one baudrate, ask GET_INFO, and resolve the device info if a
// valid RPLIDAR response comes back within `timeoutMs`. Always closes the port.
// Used by auto-detect to figure out which port/baud a sensor is on. Resolves
// null on any failure (so the caller can just try the next candidate).
function probe(path, baudRate, timeoutMs = 800) {
  return new Promise((resolve) => {
    let port = null, buffer = Buffer.alloc(0), done = false, timer = null;
    const finish = (result) => {
      if (done) return; done = true;
      if (timer) clearTimeout(timer);
      try { if (port && port.isOpen) port.close(() => {}); } catch (_) {}
      resolve(result);
    };
    let SP;
    try { ({ SerialPort: SP } = require('serialport')); } catch (_) { return resolve(null); }
    try { port = new SP({ path, baudRate, autoOpen: false }); } catch (_) { return resolve(null); }
    port.on('error', () => finish(null));
    port.on('data', (chunk) => {
      buffer = Buffer.concat([buffer, chunk]);
      // Locate a GET_INFO descriptor: 0xA5 0x5A <len LE 4B> <dataType>.
      const k = buffer.indexOf(Buffer.from([SYNC0, SYNC1]));
      if (k < 0 || buffer.length < k + 7) return;
      const len = buffer.readUInt32LE(k + 2) & 0x3fffffff; // expect 20
      if (buffer.length < k + 7 + len) return;
      const p = buffer.subarray(k + 7, k + 7 + len);
      const fwMinor = p[2];
      finish({
        model: p[0],
        name: modelName(p[0], baudRate),
        firmware: `${p[1]}.${fwMinor < 10 ? '0' + fwMinor : fwMinor}`,
        hardware: p[3],
        baudrate: baudRate,
        path,
      });
    });
    port.open((err) => {
      if (err) return finish(null);
      try {
        port.write(Buffer.from([SYNC0, CMD.STOP])); // stop any running scan first
        setTimeout(() => { buffer = Buffer.alloc(0); port.write(Buffer.from([SYNC0, CMD.GET_INFO])); }, 40);
      } catch (_) { finish(null); }
    });
    timer = setTimeout(() => finish(null), timeoutMs);
  });
}

module.exports = { RPLidar, probe, modelName };
