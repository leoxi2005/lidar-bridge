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
function scanBody(frame) {
  let enc = '';
  const centre = PP.AFRT; // step index straight ahead
  // A "person": a distance dip that sweeps ±400 steps around centre.
  const personStep = centre + Math.round(380 * Math.sin(frame / 12));
  for (let i = 0; i < N; i++) {
    const step = PP.AMIN + i;
    let d = 4000; // back wall at 4 m
    const off = Math.abs(step - personStep);
    if (off < 45) d = 1500 + off * 8; // a rounded body ~1.5 m away
    d += Math.round(15 * Math.sin(i)); // a little surface noise
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
