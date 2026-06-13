'use strict';
// Projector output window: renders the mapping in normalized 0–1 space (registration
// targets, grid, crosshair, zones + tracked points through the homography). Receives
// normalized frames from the main process; chrome toggles + keyboard F/H/G/L/T/Esc.

const $ = (id) => document.getElementById(id);
const canvas = $('proj');
const ctx = canvas.getContext('2d');
let frame = { tracks: [], zones: [], warpEnabled: false };
const opts = { grid: true, labels: true, test: false, chrome: true, bright: 1 };

function resize() {
  const d = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = innerWidth * d;
  canvas.height = innerHeight * d;
  ctx.setTransform(d, 0, 0, d, 0, 0);
  $('dims').textContent = innerWidth + ' × ' + innerHeight;
}
window.addEventListener('resize', resize);

function draw() {
  const W = innerWidth, H = innerHeight;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#000'; ctx.fillRect(0, 0, W, H);
  ctx.globalAlpha = opts.bright;

  // fit the 0–1 square into the window with margin, preserving aspect
  const margin = 0.06 * Math.min(W, H);
  const side = Math.min(W, H) - 2 * margin;
  const ox = (W - side) / 2, oy = (H - side) / 2;
  const m = (u, v) => [ox + u * side, oy + v * side];

  if (opts.test) {
    // test pattern: concentric + diagonal
    for (let i = 0; i <= 10; i++) {
      ctx.strokeStyle = i % 5 === 0 ? 'rgba(0,229,255,0.5)' : 'rgba(255,255,255,0.12)';
      const a = m(i / 10, 0), b = m(i / 10, 1); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      const c = m(0, i / 10), d = m(1, i / 10); ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke();
    }
  } else if (opts.grid) {
    ctx.strokeStyle = 'rgba(255,255,255,0.10)'; ctx.lineWidth = 1;
    for (let i = 0; i <= 8; i++) {
      const a = m(i / 8, 0), b = m(i / 8, 1); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      const c = m(0, i / 8), d = m(1, i / 8); ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke();
    }
  }

  // border + registration targets
  ctx.strokeStyle = 'rgba(0,229,255,0.6)'; ctx.lineWidth = 1.5; ctx.strokeRect(ox, oy, side, side);
  [[0, 0], [1, 0], [1, 1], [0, 1]].forEach((c) => {
    const [x, y] = m(c[0], c[1]);
    ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x - 13, y); ctx.lineTo(x + 13, y); ctx.moveTo(x, y - 13); ctx.lineTo(x, y + 13); ctx.stroke();
    if (opts.labels) { ctx.fillStyle = '#7fd6e6'; ctx.font = "10px 'IBM Plex Mono'"; ctx.fillText('(' + c[0] + ',' + c[1] + ')', x + 12, y - 12); }
  });
  const [cx, cy] = m(0.5, 0.5);
  ctx.strokeStyle = 'rgba(255,255,255,0.3)';
  ctx.beginPath(); ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10); ctx.stroke();

  // zones
  for (const z of frame.zones) {
    if (!z.uv || z.uv.length < 3) continue;
    ctx.beginPath(); z.uv.forEach((p, i) => { const [x, y] = m(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.closePath();
    ctx.fillStyle = z.occupied ? 'rgba(0,229,255,0.18)' : 'rgba(0,229,255,0.06)'; ctx.fill();
    ctx.strokeStyle = z.occupied ? 'rgba(0,229,255,0.9)' : 'rgba(0,229,255,0.4)'; ctx.lineWidth = 1.4; ctx.stroke();
    if (opts.labels) {
      const c = z.uv.reduce((a, p) => [a[0] + p[0], a[1] + p[1]], [0, 0]).map((v) => v / z.uv.length);
      const [lx, ly] = m(c[0], c[1]); ctx.fillStyle = '#bff3fb'; ctx.textAlign = 'center'; ctx.font = "600 11px 'IBM Plex Mono'";
      ctx.fillText((z.name || '').toUpperCase(), lx, ly); ctx.textAlign = 'left';
    }
  }
  // tracks
  for (const t of frame.tracks) {
    if (t.out) continue;
    const [x, y] = m(t.u, t.v);
    ctx.fillStyle = t.color || '#00e5ff';
    ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fill();
    if (opts.labels) { ctx.fillStyle = '#d6dde4'; ctx.font = "10px 'IBM Plex Mono'"; ctx.fillText(t.id, x + 9, y + 3); }
  }
  ctx.globalAlpha = 1;
  requestAnimationFrame(draw);
}

function setBtn(id, on) { $(id).classList.toggle('on', on); }
function applyChrome() { $('topChrome').classList.toggle('hidden', !opts.chrome); $('botChrome').classList.toggle('hidden', !opts.chrome); }

$('bGrid').onclick = () => { opts.grid = !opts.grid; setBtn('bGrid', opts.grid); };
$('bLabels').onclick = () => { opts.labels = !opts.labels; setBtn('bLabels', opts.labels); };
$('bTest').onclick = () => { opts.test = !opts.test; setBtn('bTest', opts.test); };
$('bHide').onclick = () => { opts.chrome = !opts.chrome; applyChrome(); };
$('bFull').onclick = () => window.projector.toggleFullscreen();
$('bExit').onclick = () => window.projector.exit();
$('bright').oninput = () => { opts.bright = parseFloat($('bright').value); };

window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (k === 'g') $('bGrid').click();
  else if (k === 'l') $('bLabels').click();
  else if (k === 't') $('bTest').click();
  else if (k === 'h') $('bHide').click();
  else if (k === 'f') window.projector.toggleFullscreen();
  else if (k === 'escape') window.projector.exit();
});

window.projector.onFrame((f) => {
  frame = f;
  const on = f.warpEnabled;
  $('warpPillTxt').textContent = on ? 'WARPED 0–1' : 'RAW METRES';
  $('warpPill').firstElementChild.style.background = on ? '#00e5ff' : '#5b636d';
});

resize();
requestAnimationFrame(draw);
