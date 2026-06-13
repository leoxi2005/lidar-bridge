'use strict';
// Hardware-free LiDAR source. Raycasts a synthetic room (the same scene the design
// prototype fakes) and emits the exact same `scan` frames as the real RPLidar driver,
// so the rest of the pipeline can't tell them apart. Select it with COM port "SIM".

const { EventEmitter } = require('events');

const WALLS = [
  [-4, -0.5, 4, -0.5],
  [4, -0.5, 4, 6],
  [4, 6, -4, 6],
  [-4, 6, -4, -0.5],
];

class Simulator extends EventEmitter {
  constructor() {
    super();
    this.connected = false;
    this.info = { model: 0, firmware: 'SIM', hardware: 0, serial: 'SIMULATOR' };
    this._timer = null;
    this._rays = 540;
    this._hz = 10;
    this.blobs = [
      { x: -2, y: 1.5, tx: -1, ty: 2.5, r: 0.28, speed: 0.62 },
      { x: 1.5, y: 3, tx: 2, ty: 2, r: 0.28, speed: 0.78 },
      { x: 2.7, y: 4.6, tx: 1, ty: 4, r: 0.28, speed: 0.5 },
      { x: -1, y: 4, tx: -2, ty: 3, r: 0.28, speed: 0.7 },
    ];
  }

  async connect() {
    this.connected = true;
    this.emit('info', this.info);
    this.emit('status', 'simulator scanning');
    const period = 1000 / this._hz;
    let last = Date.now();
    this._timer = setInterval(() => {
      const now = Date.now();
      const dt = Math.min(0.2, (now - last) / 1000);
      last = now;
      this._stepBlobs(dt);
      this.emit('scan', this._castScan());
    }, period);
    return this.info;
  }

  async disconnect() {
    this.connected = false;
    if (this._timer) clearInterval(this._timer);
    this._timer = null;
    this.emit('status', 'simulator stopped');
  }

  _stepBlobs(dt) {
    for (const b of this.blobs) {
      const dx = b.tx - b.x;
      const dy = b.ty - b.y;
      const d = Math.hypot(dx, dy);
      if (d < 0.25) {
        b.tx = -3.4 + Math.random() * 6.8;
        b.ty = 0.1 + Math.random() * 5.4;
      } else {
        b.x += (dx / d) * b.speed * dt;
        b.y += (dy / d) * b.speed * dt;
      }
    }
  }

  _castScan() {
    const nodes = [];
    const step = (2 * Math.PI) / this._rays;
    for (let i = 0; i < this._rays; i++) {
      const a = i * step;
      const dx = Math.cos(a);
      const dy = Math.sin(a);
      let best = Infinity;
      for (const sg of WALLS) {
        const t = raySeg(0, 0, dx, dy, sg);
        if (t > 0 && t < best) best = t;
      }
      for (const b of this.blobs) {
        const t = rayCircle(0, 0, dx, dy, b.x, b.y, b.r);
        if (t > 0 && t < best) best = t;
      }
      if (!isFinite(best) || best > 25) continue;
      const jitter = (Math.random() - 0.5) * 0.01;
      const angleDeg = ((a * 180) / Math.PI) % 360;
      nodes.push({
        angle: angleDeg,
        distMm: (best + jitter) * 1000,
        quality: 200,
      });
    }
    return nodes;
  }
}

function raySeg(ox, oy, dx, dy, sg) {
  const ex = sg[2] - sg[0];
  const ey = sg[3] - sg[1];
  const den = dx * -ey - -ex * dy;
  if (Math.abs(den) < 1e-9) return -1;
  const t = (-(sg[0] - ox) * ey + ex * (sg[1] - oy)) / den;
  const u = (dx * (sg[1] - oy) - dy * (sg[0] - ox)) / den;
  if (t > 0 && u >= 0 && u <= 1) return t;
  return -1;
}

function rayCircle(ox, oy, dx, dy, cx, cy, r) {
  const fx = ox - cx;
  const fy = oy - cy;
  const b = 2 * (fx * dx + fy * dy);
  const c = fx * fx + fy * fy - r * r;
  const disc = b * b - 4 * c;
  if (disc < 0) return -1;
  const sq = Math.sqrt(disc);
  let t = (-b - sq) / 2;
  if (t > 0) return t;
  t = (-b + sq) / 2;
  return t > 0 ? t : -1;
}

module.exports = { Simulator };
