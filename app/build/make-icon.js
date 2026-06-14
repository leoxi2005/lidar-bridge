'use strict';
// Generate the LiDAR Bridge app icon (1024px PNG) — dark bg + cyan radar rings +
// a tracked dot, matching the in-app design. No external image libs: we draw into
// an RGBA buffer and encode a PNG with zlib. Run: node build/make-icon.js
const fs = require('fs');
const zlib = require('zlib');
const path = require('path');

const N = 1024;
const buf = Buffer.alloc(N * N * 4);

const BG = [10, 11, 14];      // #0a0b0e
const CYAN = [56, 224, 235];  // accent
const GREEN = [120, 230, 140];
const ORANGE = [245, 170, 60];

function px(x, y, rgb, a) {
  if (x < 0 || y < 0 || x >= N || y >= N) return;
  const i = (y * N + x) * 4;
  const ia = 1 - a;
  buf[i]   = Math.round(buf[i]   * ia + rgb[0] * a);
  buf[i+1] = Math.round(buf[i+1] * ia + rgb[1] * a);
  buf[i+2] = Math.round(buf[i+2] * ia + rgb[2] * a);
  buf[i+3] = 255;
}

const cx = N / 2, cy = N / 2;
const corner = N * 0.22; // squircle-ish rounded corner radius

// rounded-rect background mask + subtle radial glow toward center
for (let y = 0; y < N; y++) {
  for (let x = 0; x < N; x++) {
    // rounded corner test
    const dx = Math.max(corner - x, x - (N - corner), 0);
    const dy = Math.max(corner - y, y - (N - corner), 0);
    const cd = Math.hypot(dx, dy);
    let inside = 1;
    if (cd > corner) inside = 0;
    else if (cd > corner - 1.5) inside = (corner - cd) / 1.5;
    if (inside <= 0) { const i=(y*N+x)*4; buf[i+3]=0; continue; }
    const r = Math.hypot(x - cx, y - cy) / (N / 2);
    const glow = Math.max(0, 0.10 * (1 - r));
    const rgb = [BG[0] + 30*glow, BG[1] + 50*glow, BG[2] + 60*glow];
    const i = (y*N+x)*4;
    buf[i]=rgb[0]; buf[i+1]=rgb[1]; buf[i+2]=rgb[2]; buf[i+3]=Math.round(255*inside);
  }
}

// anti-aliased ring (stroke) by coverage of distance to radius
function ring(radius, width, rgb, alpha) {
  const r0 = radius - width/2, r1 = radius + width/2;
  const x0 = Math.max(0, Math.floor(cx - r1 - 2)), x1 = Math.min(N, Math.ceil(cx + r1 + 2));
  const y0 = Math.max(0, Math.floor(cy - r1 - 2)), y1 = Math.min(N, Math.ceil(cy + r1 + 2));
  for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) {
    const d = Math.hypot(x - cx, y - cy);
    let cov = 0;
    if (d >= r0 && d <= r1) cov = 1;
    else if (d > r0 - 1 && d < r0) cov = d - (r0 - 1);
    else if (d > r1 && d < r1 + 1) cov = (r1 + 1) - d;
    if (cov > 0) px(x, y, rgb, alpha * cov);
  }
}

function disc(px0, py0, radius, rgb) {
  const x0 = Math.max(0, Math.floor(px0 - radius - 2)), x1 = Math.min(N, Math.ceil(px0 + radius + 2));
  const y0 = Math.max(0, Math.floor(py0 - radius - 2)), y1 = Math.min(N, Math.ceil(py0 + radius + 2));
  for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) {
    const d = Math.hypot(x - px0, y - py0);
    let cov = 0;
    if (d <= radius - 1) cov = 1; else if (d < radius + 1) cov = (radius + 1 - d) / 2;
    if (cov > 0) {
      // soft outer glow
      px(x, y, rgb, Math.min(1, cov));
    }
  }
}

// concentric radar rings, fading outward
ring(N*0.40, 6, CYAN, 0.30);
ring(N*0.30, 6, CYAN, 0.45);
ring(N*0.20, 6, CYAN, 0.65);
ring(N*0.10, 6, CYAN, 0.85);

// sweep line (radius from center to upper-right)
(() => {
  const ang = -Math.PI * 0.18, len = N*0.42;
  for (let t = 0; t < len; t++) {
    const x = cx + Math.cos(ang) * t, y = cy + Math.sin(ang) * t;
    const a = 0.5 * (1 - t/len);
    for (let o=-2;o<=2;o++) px(Math.round(x), Math.round(y)+o, CYAN, a*(1-Math.abs(o)/3));
  }
})();

// center sensor + a couple of tracked dots on the rings
disc(cx, cy, 16, CYAN);
disc(cx + Math.cos(-0.18*Math.PI)*N*0.30, cy + Math.sin(-0.18*Math.PI)*N*0.30, 26, CYAN);
disc(cx + Math.cos(0.7*Math.PI)*N*0.20, cy + Math.sin(0.7*Math.PI)*N*0.20, 22, GREEN);
disc(cx + Math.cos(0.25*Math.PI)*N*0.40, cy + Math.sin(0.25*Math.PI)*N*0.40, 20, ORANGE);

// --- encode PNG ---
function png(buffer, w, h) {
  function chunk(type, data) {
    const len = Buffer.alloc(4); len.writeUInt32BE(data.length, 0);
    const t = Buffer.from(type, 'ascii');
    const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(Buffer.concat([t, data])) >>> 0, 0);
    return Buffer.concat([len, t, data, crc]);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4);
  ihdr[8]=8; ihdr[9]=6; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0; // 8-bit RGBA
  const raw = Buffer.alloc((w*4+1)*h);
  for (let y=0;y<h;y++){ raw[y*(w*4+1)]=0; buffer.copy(raw, y*(w*4+1)+1, y*w*4, y*w*4+w*4); }
  const idat = zlib.deflateSync(raw, { level: 9 });
  const sig = Buffer.from([137,80,78,71,13,10,26,10]);
  return Buffer.concat([sig, chunk('IHDR', ihdr), chunk('IDAT', idat), chunk('IEND', Buffer.alloc(0))]);
}
const CRC = (() => { const t=[]; for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;t[n]=c;} return t; })();
function crc32(b){ let c=0xffffffff; for(let i=0;i<b.length;i++) c=CRC[(c^b[i])&0xff]^(c>>>8); return c^0xffffffff; }

const out = path.join(__dirname, 'icon.png');
fs.writeFileSync(out, png(buf, N, N));
console.log('wrote', out);
