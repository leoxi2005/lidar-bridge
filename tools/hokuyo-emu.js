'use strict';
// Hokuyo UST-10LX EMULATOR — a fake sensor for testing WITHOUT hardware.
// Listens on TCP :10940 and speaks just enough SCIP 2.0 for app/main/hokuyo.js:
//   QT / SCIP2.0 / PP / BM  -> canned replies
//   MD (continuous)         -> streams a synthetic scene at ~10 Hz
// It emits an animated scene (a back wall + a person walking left↔right) so the
// app's point cloud visibly moves and OSC fires — exercising parser, angle
// mapping, the fusion branch and the fusion watchdog end-to-end.
//
// Run:   node tools/hokuyo-emu.js            (listens 0.0.0.0:10940)
//        node tools/hokuyo-emu.js 10941      (custom port for a 2nd sensor)
// In the app connect to host 127.0.0.1, port 10940, brand = hokuyo.

const net = require('net');

const PORT = parseInt(process.argv[2], 10) || 10940;

// UST-10LX parameters (must match what hokuyo.js expects from PP).
const PP = { MODL: 'UST-10LX', DMIN: 20, DMAX: 60000, ARES: 1440, AMIN: 0, AMAX: 1080, AFRT: 540, SCAN: 600 };
const N = PP.AMAX - PP.AMIN + 1; // points per scan

// SCIP checksum: sum of the line's bytes, low 6 bits, +0x30. hokuyo.js ignores
// it (just strips the trailing char), but we emit a real one for realism.
function sum(s) {
  let t = 0;
  for (let i = 0; i < s.length; i++) t += s.charCodeAt(i);
  return String.fromCharCode((t & 0x3f) + 0x30);
}
// Encode one distance (mm) as 3 SCIP chars, 6 bits each.
function enc3(d) {
  return String.fromCharCode(((d >> 12) & 0x3f) + 0x30, ((d >> 6) & 0x3f) + 0x30, (d & 0x3f) + 0x30);
}
// A reply block is terminated by a blank line -> content + "\n\n".
function reply(lines) { return lines.join('\n') + '\n\n'; }

// Build the encoded body for one scan, chunked into 64-data-char lines each
// followed by a checksum char — exactly how a real UST frames MD data.
// Simulated venue: a RECTANGULAR room, sensor mounted mid-wall looking in.
// For each ray we cast from the sensor and return the distance to the nearest
// wall (or the walking person) — so the point cloud traces the ROOM's actual
// shape, not a circle. (A constant distance in every direction is what makes a
// circle; a real room does not do that.) Sensor at origin, front (0°) = +Y.
const ROOM = { halfW: 4.0, depth: 6.0 }; // 8 m wide × 6 m deep
const PERSON_R = 0.35;
const per = 360 / PP.ARES;               // degrees per step (matches hokuyo.js)

function rayDistanceM(dx, dy, px, py) {
  let best = Infinity;
  // three visible walls: back (y=depth), left (x=-halfW), right (x=+halfW)
  if (dy > 1e-6) { const t = ROOM.depth / dy; if (t > 0 && Math.abs(t * dx) <= ROOM.halfW) best = Math.min(best, t); }
  if (dx < -1e-6) { const t = -ROOM.halfW / dx; const y = t * dy; if (t > 0 && y >= 0 && y <= ROOM.depth) best = Math.min(best, t); }
  if (dx > 1e-6) { const t = ROOM.halfW / dx; const y = t * dy; if (t > 0 && y >= 0 && y <= ROOM.depth) best = Math.min(best, t); }
  // person = a circle the ray can hit before the wall (occludes it)
  const b = -2 * (dx * px + dy * py);
  const c0 = px * px + py * py - PERSON_R * PERSON_R;
  const disc = b * b - 4 * c0;
  if (disc >= 0) { const t = (-b - Math.sqrt(disc)) / 2; if (t > 0 && t < best) best = t; }
  return best;
}

function scanBody(frame) {
  let enc = '';
  const px = 2.2 * Math.sin(frame / 15), py = 3.0; // person walks left↔right mid-room
  for (let i = 0; i < N; i++) {
    const a = (PP.AMIN + i - PP.AFRT) * per * Math.PI / 180; // ray angle, 0 = front(+Y)
    const dx = -Math.sin(a), dy = Math.cos(a);
    const t = rayDistanceM(dx, dy, px, py);
    // Rays past the room edge (pointing behind the wall) get no return -> 0 (< DMIN).
    let d = Number.isFinite(t) ? Math.round(t * 1000 + 8 * Math.sin(i)) : 0;
    if (d < 0) d = 0;
    enc += enc3(d);
  }
  const lines = [];
  for (let i = 0; i < enc.length; i += 64) {
    const chunk = enc.slice(i, i + 64);
    lines.push(chunk + sum(chunk));
  }
  return lines;
}

const server = net.createServer((sock) => {
  const who = `${sock.remoteAddress}:${sock.remotePort}`;
  console.log(`[emu:${PORT}] client connected ${who}`);
  sock.setNoDelay(true);
  let buf = '';
  let streamer = null;
  let frame = 0;

  const stopStream = () => { if (streamer) { clearInterval(streamer); streamer = null; } };

  sock.on('data', (chunk) => {
    buf += chunk.toString('latin1');
    let nl;
    while ((nl = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, nl);
      buf = buf.slice(nl + 1);
      const cmd = line.slice(0, 2);
      if (line.startsWith('MD')) {
        // Ack (status 00), then stream data blocks (status 99) continuously.
        console.log(`[emu:${PORT}] MD -> streaming`);
        sock.write(reply([line, '00' + sum('00')]));
        stopStream();
        streamer = setInterval(() => {
          const ts = '0000'; // timestamp (ignored by hokuyo.js, occupies lines[2])
          const body = [line, '99' + sum('99'), ts + sum(ts), ...scanBody(frame++)];
          sock.write(reply(body));
        }, 100); // ~10 Hz
      } else if (cmd === 'QT' || cmd === 'RS') {
        stopStream();
        sock.write(reply([line, '00' + sum('00')]));
      } else if (line.startsWith('SCIP2.0')) {
        sock.write(reply([line, '00' + sum('00')]));
      } else if (cmd === 'BM') {
        sock.write(reply([line, '00' + sum('00')])); // 00 = laser on
      } else if (cmd === 'PP' || cmd === 'VV' || cmd === 'II') {
        const body = [line, '00' + sum('00')];
        for (const k of Object.keys(PP)) { const kv = `${k}:${PP[k]};`; body.push(kv + sum(kv)); }
        sock.write(reply(body));
      } else if (line.length) {
        sock.write(reply([line, '0E' + sum('0E')])); // unknown command
      }
    }
  });

  sock.on('close', () => { stopStream(); console.log(`[emu:${PORT}] client closed ${who}`); });
  sock.on('error', (e) => { stopStream(); console.log(`[emu:${PORT}] socket error ${e.message}`); });
});

server.listen(PORT, () => console.log(`[emu] Hokuyo UST-10LX emulator listening on 0.0.0.0:${PORT} — connect app to 127.0.0.1:${PORT}, brand=hokuyo`));
