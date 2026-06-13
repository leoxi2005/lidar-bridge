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
};

const SYNC0 = 0xa5;
const SYNC1 = 0x5a;

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

class RPLidar extends EventEmitter {
  constructor() {
    super();
    this.port = null;
    this.buffer = Buffer.alloc(0);
    this.mode = 'idle'; // 'idle' | 'response' | 'scan'
    this._pending = null; // { n, resolve, reject }
    this.connected = false;
    this.info = null;
    this.health = null;
    this._scan = []; // nodes accumulated for the current revolution
  }

  // ---- public API ---------------------------------------------------------
  async connect({ path, baudRate }) {
    if (this.port) await this.disconnect();
    this._log(`opening ${path} @ ${baudRate}`);

    const SP = loadSerialPort();
    this.port = new SP({ path, baudRate, autoOpen: false });
    this.port.on('data', (chunk) => this._onData(chunk));
    this.port.on('error', (err) => this.emit('error', err));
    this.port.on('close', () => {
      this.connected = false;
      this.emit('status', 'port closed');
    });

    await new Promise((res, rej) => this.port.open((e) => (e ? rej(e) : res())));

    // Spin the motor: on A1 the motor runs when DTR is asserted low (false).
    // A2/A3 ignore DTR (PWM-driven) but tolerate it, so this is safe everywhere.
    await this._setSignals({ dtr: false });

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

    await this._startScan();
    this.connected = true;
    this.emit('status', 'scanning');
    return this.info;
  }

  async disconnect() {
    if (!this.port) return;
    try {
      this._send(CMD.STOP);
      await delay(20);
      await this._setSignals({ dtr: true }); // stop motor (A1)
    } catch (_) {
      /* ignore */
    }
    await new Promise((res) => this.port.close(() => res()));
    this.port = null;
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
    this.port.write(pkt);
  }

  _onData(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    if (this.mode === 'scan') {
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
    if (desc.dataType !== 0x81) {
      throw new Error('unexpected scan data type 0x' + desc.dataType.toString(16));
    }
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
      i += 5;
    }
    this.buffer = buf.subarray(i);
  }
}

module.exports = { RPLidar };
