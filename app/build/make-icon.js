'use strict';
// Generate the LEOXI-LIDARTRACKING app icon (1024px PNG): a polished radar —
// gradient squircle bg, a glowing sweep sector, crisp rings, and tracked dots
// with soft halos. No image libs: draw into an RGBA buffer, encode PNG via zlib.
// Run: node build/make-icon.js   (then build/make-icns.sh turns it into icon.icns)
const fs = require('fs');
const zlib = require('zlib');
const path = require('path');

const N = 1024;
const buf = Buffer.alloc(N * N * 4); // RGBA, premultiplied-ish straight alpha
const cx = N / 2, cy = N / 2;
const R = N / 2;

const CYAN = [56, 224, 235];
const GREEN = [120, 230, 140];
const ORANGE = [245, 170, 60];

function clamp(v) { return v < 0 ? 0 : v > 255 ? 255 : v; }
function setPx(x, y, r, g, b, a) {
  if (x < 0 || y < 0 || x >= N || y >= N) return;
  const i = (y * N + x) * 4;
  const sa = a, ia = 1 - a;
  buf[i]   = clamp(buf[i]   * ia + r * sa);
  buf[i+1] = clamp(buf[i+1] * ia + g * sa);
  buf[i+2] = clamp(buf[i+2] * ia + b * sa);
  buf[i+3] = Math.max(buf[i+3], Math.round(255 * a));
}
function addPx(x, y, r, g, b, a) { // additive (for glows)
  if (x < 0 || y < 0 || x >= N || y >= N) return;
  const i = (y * N + x) * 4;
  buf[i]   = clamp(buf[i]   + r * a);
  buf[i+1] = clamp(buf[i+1] + g * a);
  buf[i+2] = clamp(buf[i+2] + b * a);
  buf[i+3] = Math.max(buf[i+3], Math.round(255 * Math.min(1, a)));
}

const corner = N * 0.235; // squircle corner radius

// --- background: rounded square, vertical gradient + cyan radial glow at center ---
for (let y = 0; y < N; y++) {
  for (let x = 0; x < N; x++) {
    const dx = Math.max(corner - x, x - (N - corner), 0);
    const dy = Math.max(corner - y, y - (N - corner), 0);
    const cd = Math.hypot(dx, dy);
    let inside = 1;
    if (cd > corner) inside = 0;
    else if (cd > corner - 1.5) inside = (corner - cd) / 1.5;
    const i = (y*N+x)*4;
    if (inside <= 0) { buf[i+3] = 0; continue; }
    const t = y / N;                    // top->bottom
    let r = 16 - 11*t, g = 20 - 13*t, b = 28 - 18*t;  // #10141c -> #050709
    const rr = Math.hypot(x - cx, y - cy) / R;
    const glow = Math.max(0, 1 - rr) ** 2 * 0.18;
    r += CYAN[0]*glow*0.5; g += CYAN[1]*glow*0.5; b += CYAN[2]*glow*0.5;
    buf[i]=clamp(r); buf[i+1]=clamp(g); buf[i+2]=clamp(b); buf[i+3]=Math.round(255*inside);
  }
}

// --- radar sweep sector: wedge centered around an angle, fading with radius, ---
// --- brightest at its leading edge. Classic radar look. ---
const sweepLead = -Math.PI * 0.16;       // leading-edge angle
const sweepSpan = Math.PI * 0.42;        // wedge width (trailing tail)
const sweepR = R * 0.80;
for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
  const i = (y*N+x)*4; if (buf[i+3] === 0) continue;
  const ddx = x - cx, ddy = y - cy;
  const rad = Math.hypot(ddx, ddy);
  if (rad > sweepR) continue;
  let da = sweepLead - Math.atan2(ddy, ddx); // 0 at leading edge, grows into the tail
  while (da < -Math.PI) da += 2*Math.PI; while (da > Math.PI) da -= 2*Math.PI;
  if (da < 0 || da > sweepSpan) continue;
  const angT = 1 - da / sweepSpan;           // 1 at leading edge -> 0 at tail
  const radT = 1 - rad / sweepR;             // fade outward
  const a = 0.34 * angT * angT * (0.35 + 0.65*radT);
  addPx(x, y, CYAN[0], CYAN[1], CYAN[2], a);
}

// anti-aliased ring stroke
function ring(radius, width, rgb, alpha) {
  const r0 = radius - width/2, r1 = radius + width/2;
  const x0=Math.max(0,Math.floor(cx-r1-2)), x1=Math.min(N,Math.ceil(cx+r1+2));
  const y0=Math.max(0,Math.floor(cy-r1-2)), y1=Math.min(N,Math.ceil(cy+r1+2));
  for (let y=y0;y<y1;y++) for (let x=x0;x<x1;x++) {
    const d = Math.hypot(x-cx,y-cy); let cov=0;
    if (d>=r0 && d<=r1) cov=1;
    else if (d>r0-1 && d<r0) cov=d-(r0-1);
    else if (d>r1 && d<r1+1) cov=(r1+1)-d;
    if (cov>0) setPx(x,y,rgb[0],rgb[1],rgb[2],alpha*cov);
  }
}

// dot with soft additive halo
function dot(px0, py0, radius, rgb) {
  const halo = radius * 3.2;
  const x0=Math.max(0,Math.floor(px0-halo)), x1=Math.min(N,Math.ceil(px0+halo));
  const y0=Math.max(0,Math.floor(py0-halo)), y1=Math.min(N,Math.ceil(py0+halo));
  for (let y=y0;y<y1;y++) for (let x=x0;x<x1;x++) {
    const d = Math.hypot(x-px0,y-py0);
    if (d < halo) { // glow
      const g = (1 - d/halo) ** 2.4 * 0.55;
      addPx(x,y,rgb[0],rgb[1],rgb[2],g);
    }
    let cov=0; // crisp core
    if (d<=radius-1) cov=1; else if (d<radius+1) cov=(radius+1-d)/2;
    if (cov>0) setPx(x,y,clamp(rgb[0]+60),clamp(rgb[1]+25),clamp(rgb[2]+25),cov);
  }
}

// rings (thin, premium), plus a faint inner border just inside the squircle
ring(R*0.84, 2.5, CYAN, 0.10);
ring(R*0.62, 3, CYAN, 0.22);
ring(R*0.42, 3, CYAN, 0.34);
ring(R*0.22, 3, CYAN, 0.50);

// tracked dots on a few rings + center sensor
dot(cx, cy, 13, CYAN);
dot(cx + Math.cos(sweepLead)*R*0.62, cy + Math.sin(sweepLead)*R*0.62, 30, CYAN);
dot(cx + Math.cos(Math.PI*0.74)*R*0.42, cy + Math.sin(Math.PI*0.74)*R*0.42, 22, GREEN);
dot(cx + Math.cos(Math.PI*0.34)*R*0.62, cy + Math.sin(Math.PI*0.34)*R*0.62, 20, ORANGE);

// --- encode PNG ---
function pngEncode(buffer, w, h) {
  function chunk(type, data) {
    const len=Buffer.alloc(4); len.writeUInt32BE(data.length,0);
    const t=Buffer.from(type,'ascii');
    const crc=Buffer.alloc(4); crc.writeUInt32BE(crc32(Buffer.concat([t,data]))>>>0,0);
    return Buffer.concat([len,t,data,crc]);
  }
  const ihdr=Buffer.alloc(13);
  ihdr.writeUInt32BE(w,0); ihdr.writeUInt32BE(h,4);
  ihdr[8]=8; ihdr[9]=6; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0;
  const raw=Buffer.alloc((w*4+1)*h);
  for(let y=0;y<h;y++){ raw[y*(w*4+1)]=0; buffer.copy(raw,y*(w*4+1)+1,y*w*4,y*w*4+w*4); }
  const idat=zlib.deflateSync(raw,{level:9});
  const sig=Buffer.from([137,80,78,71,13,10,26,10]);
  return Buffer.concat([sig,chunk('IHDR',ihdr),chunk('IDAT',idat),chunk('IEND',Buffer.alloc(0))]);
}
const CRC=(()=>{const t=[];for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;t[n]=c;}return t;})();
function crc32(b){let c=0xffffffff;for(let i=0;i<b.length;i++)c=CRC[(c^b[i])&0xff]^(c>>>8);return c^0xffffffff;}

const out=path.join(__dirname,'icon.png');
fs.writeFileSync(out, pngEncode(buf,N,N));
console.log('wrote', out);
