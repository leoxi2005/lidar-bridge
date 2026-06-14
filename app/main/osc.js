'use strict';
// Minimal OSC 1.0 over UDP. No external dependency — OSC message framing is small:
// an address pattern (OSC-string), a type-tag string (",fff"), then big-endian args.
// Also builds TUIO 1.1 /tuio/2Dobj bundles (used from step 7).

const dgram = require('dgram');

function oscString(s) {
  const b = Buffer.from(String(s), 'ascii');
  const total = (Math.floor(b.length / 4) + 1) * 4; // always >=1 null, padded to 4
  const out = Buffer.alloc(total);
  b.copy(out);
  return out;
}

// args: [{ type:'f'|'i'|'s', value }]
function oscMessage(address, args) {
  const parts = [oscString(address), oscString(',' + args.map((a) => a.type).join(''))];
  for (const a of args) {
    if (a.type === 'f') { const b = Buffer.alloc(4); b.writeFloatBE(a.value); parts.push(b); }
    else if (a.type === 'i') { const b = Buffer.alloc(4); b.writeInt32BE(a.value | 0); parts.push(b); }
    else if (a.type === 's') { parts.push(oscString(a.value)); }
  }
  return Buffer.concat(parts);
}

// bundle: { timetag?: <immediate>, elements: [Buffer(message), ...] }
function oscBundle(elements) {
  const parts = [oscString('#bundle')];
  const tt = Buffer.alloc(8); tt.writeUInt32BE(0, 0); tt.writeUInt32BE(1, 4); // immediate
  parts.push(tt);
  for (const el of elements) {
    const size = Buffer.alloc(4); size.writeInt32BE(el.length);
    parts.push(size, el);
  }
  return Buffer.concat(parts);
}

class OscSender {
  constructor() {
    this.sock = dgram.createSocket('udp4');
    this.host = '127.0.0.1';
    this.port = 7000;
  }
  configure({ host, port }) {
    if (host) this.host = host;
    if (port) this.port = parseInt(port, 10) || this.port;
  }
  sendRaw(buf) {
    this.sock.send(buf, this.port, this.host, () => {});
  }
  sendMessage(address, args) {
    this.sendRaw(oscMessage(address, args));
  }
  // msgs: [{ a: address, args: [...] }] -> one UDP packet (OSC bundle)
  sendBundle(msgs) {
    if (!msgs.length) return;
    this.sendRaw(oscBundle(msgs.map((m) => oscMessage(m.a, m.args))));
  }
  close() {
    try { this.sock.close(); } catch (_) { /* ignore */ }
  }
}

module.exports = { OscSender, oscMessage, oscBundle, oscString };
