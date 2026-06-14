'use strict';
// Real-time tracking pipeline (runs in the main process). Grows across build steps:
//   step 2: FILTER + TRANSFORM (placement, mm->m) -> world-space point cloud
//   step 3: background SUBTRACT
//   step 4: CLUSTER (DBSCAN) + TRACK
//   step 5: ZONES occupancy
//   step 7: MAP (homography)
//
// Each scan is binned into NBINS angle slots. Binning index is guarded with `% NBINS`
// so angle == 360° can never write past the array end (reference-plugin known issue a).
// Distances arrive in millimetres and are converted to metres here, once, so the whole
// world space downstream is in metres (reference-plugin known issue b).

const NBINS = 720; // 0.5° resolution

const DEG = Math.PI / 180;
const { computeH, applyH } = require('./homography');

// tracking parameters (README: ε≈0.30 m, minPts≈3, max-jump ≈0.6 m)
const CLUSTER_EPS = 0.3;
const CLUSTER_MINPTS = 3;
const MAX_BLOB_RADIUS = 0.9; // reject wall/furniture clusters larger than a person
const MAX_BLOB_POINTS = 90;
const MAX_JUMP = 0.6; // nearest-neighbor gate
const MAX_MISSED = 8; // drop a track after this many unmatched frames
const CONFIRM_HITS = 4; // frames a blob must persist before it becomes a real track
const TRACK_PALETTE = ['#00e5ff', '#39ff7a', '#ffb000', '#ff5d8f'];

// DBSCAN over an array of [x,y] points -> array of clusters (arrays of indices).
function dbscan(pts, eps, minPts) {
  const n = pts.length;
  const eps2 = eps * eps;
  const visited = new Uint8Array(n);
  const cl = new Int32Array(n).fill(-1);
  const region = (i) => {
    const out = [];
    for (let j = 0; j < n; j++) {
      const dx = pts[i][0] - pts[j][0];
      const dy = pts[i][1] - pts[j][1];
      if (dx * dx + dy * dy <= eps2) out.push(j);
    }
    return out;
  };
  let cid = 0;
  for (let i = 0; i < n; i++) {
    if (visited[i]) continue;
    visited[i] = 1;
    const nb = region(i);
    if (nb.length < minPts) continue;
    cl[i] = cid;
    const queue = nb.slice();
    for (let q = 0; q < queue.length; q++) {
      const j = queue[q];
      if (!visited[j]) {
        visited[j] = 1;
        const nb2 = region(j);
        if (nb2.length >= minPts) for (const k of nb2) queue.push(k);
      }
      if (cl[j] < 0) cl[j] = cid;
    }
    cid++;
  }
  const groups = Array.from({ length: cid }, () => []);
  for (let i = 0; i < n; i++) if (cl[i] >= 0) groups[cl[i]].push(i);
  return groups;
}

// ray-casting point-in-polygon (poly = [[x,y],...])
function pointInPoly(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1];
    const xj = poly[j][0], yj = poly[j][1];
    if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

function angleToBin(angleDeg) {
  let idx = Math.round((angleDeg * NBINS) / 360) % NBINS; // guard (a): never == NBINS
  if (idx < 0) idx += NBINS;
  return idx;
}

class Pipeline {
  constructor() {
    this.cfg = {
      quality: false,
      distMin: 0.0,
      distMax: 25.0,
      placement: { x: 0, y: 0, rot: 0 },
      bgSubtract: false,
      bgTol: 0.18, // metres
      smooth: true, // one-euro position smoothing
      smoothMin: 1.0, // mincutoff (lower = smoother but more lag)
      smoothBeta: 0.4, // speed coefficient (higher = less lag when moving fast)
      smoothDcutoff: 1.0,
    };
    // per-bin scratch reused each scan
    this._dist = new Float32Array(NBINS); // metres, 0 = empty
    // background baseline (metres per bin); 0 = no baseline captured for that bin
    this.bg = new Float32Array(NBINS);
    this.bgCaptured = false;
    this._capFrames = 0; // >0 while capturing (counts down)

    // tracking state
    this.tracks = [];
    this._nextId = 0; // ids assigned lazily on confirmation, starting at 01

    // trigger zones: [{ name, slug, pts:[[x,y],...] }]
    this.zones = [];
    this._zoneRT = new Map(); // slug -> { since } for dwell timing

    // warp (corner-pin homography): source corners (metres) -> unit square
    this.warpCorners = [[-3, 5], [3, 5], [3, 0.5], [-3, 0.5]];
    this.warpEnabled = false;
    this.warpH = computeH(this.warpCorners);
  }

  setWarp(patch) {
    if (patch.corners) this.warpCorners = patch.corners;
    if (patch.enabled !== undefined) this.warpEnabled = !!patch.enabled;
    this.warpH = computeH(this.warpCorners);
  }

  setZones(zones) {
    this.zones = (zones || []).map((z) => ({
      name: z.name,
      slug: z.slug,
      pts: z.pts,
    }));
  }

  setConfig(patch) {
    if (patch.quality !== undefined) this.cfg.quality = !!patch.quality;
    if (patch.distMin !== undefined) this.cfg.distMin = parseFloat(patch.distMin) || 0;
    if (patch.distMax !== undefined) this.cfg.distMax = parseFloat(patch.distMax) || 25;
    if (patch.bgSubtract !== undefined) this.cfg.bgSubtract = !!patch.bgSubtract;
    if (patch.bgTol !== undefined) this.cfg.bgTol = parseFloat(patch.bgTol) || 0.18;
    if (patch.smooth !== undefined) this.cfg.smooth = !!patch.smooth;
    if (patch.smoothAmount !== undefined) {
      // 0 = light (responsive), 1 = heavy (smooth): map to mincutoff 3.0 -> 0.3 Hz
      const a = Math.max(0, Math.min(1, parseFloat(patch.smoothAmount)));
      this.cfg.smoothMin = 3.0 - a * 2.7;
    }
    if (patch.placement) {
      this.cfg.placement = {
        x: parseFloat(patch.placement.x) || 0,
        y: parseFloat(patch.placement.y) || 0,
        rot: parseFloat(patch.placement.rot) || 0,
      };
    }
  }

  // Capture the empty-room baseline. We accumulate the MAX distance per bin over a
  // handful of scans, so a person moving through during capture (a *closer* reading)
  // doesn't get baked into the wall baseline.
  captureBackground(frames = 12) {
    this.bg.fill(0);
    this._capFrames = frames;
    this.bgCaptured = false;
  }

  clearBackground() {
    this.bg.fill(0);
    this.bgCaptured = false;
    this._capFrames = 0;
    this.cfg.bgSubtract = false;
  }

  // restore a saved baseline (from a preset)
  setBaseline(arr) {
    if (!arr || !arr.length) return;
    this.bg = Float32Array.from(arr);
    this.bgCaptured = true;
    this._capFrames = 0;
  }

  // nodes: [{ angle (deg), distMm, quality }]; dtSec = seconds since previous scan
  // returns a frame: { pts, count, nbins, tracks, ... }
  process(nodes, dtSec = 0.1) {
    const dist = this._dist;
    dist.fill(0);

    const { x: px, y: py, rot } = this.cfg.placement;
    const cosR = Math.cos(rot * DEG);
    const sinR = Math.sin(rot * DEG);

    // pts layout: 3 floats per bin -> [worldX, worldY, fg]; fg = -1 means empty
    const pts = new Float32Array(NBINS * 3);
    for (let i = 0; i < NBINS; i++) pts[i * 3 + 2] = -1;

    const capturing = this._capFrames > 0;
    const subtract = this.cfg.bgSubtract && this.bgCaptured;
    const tol = this.cfg.bgTol;
    const fgPts = []; // foreground world points fed to clustering

    let count = 0;
    for (const node of nodes) {
      const distM = node.distMm / 1000; // mm -> m (known issue b)
      if (distM <= 0) continue;
      if (this.cfg.quality && node.quality < 150) continue;
      if (distM < this.cfg.distMin || distM > this.cfg.distMax) continue;

      const idx = angleToBin(node.angle);
      const a = node.angle * DEG;
      // sensor-local cartesian, then placement (rotation + translation) -> world metres
      const lx = Math.cos(a) * distM;
      const ly = Math.sin(a) * distM;
      const wx = px + lx * cosR - ly * sinR;
      const wy = py + lx * sinR + ly * cosR;

      // SUBTRACT: a point is foreground only if meaningfully closer than the baseline.
      let fg = 1;
      if (subtract) {
        const base = this.bg[idx];
        fg = base > 0.001 ? (distM < base - tol ? 1 : 0) : 1;
      }

      // accumulate baseline (max distance per bin) while capturing
      if (capturing && distM > this.bg[idx]) this.bg[idx] = distM;

      if (pts[idx * 3 + 2] < 0) count++; // first hit for this bin
      dist[idx] = distM;
      pts[idx * 3] = wx;
      pts[idx * 3 + 1] = wy;
      pts[idx * 3 + 2] = fg;
      if (fg === 1) fgPts.push([wx, wy]);
    }

    if (capturing) {
      this._capFrames--;
      if (this._capFrames === 0) this.bgCaptured = true;
    }

    // CLUSTER + TRACK
    const blobs = this._clusterBlobs(fgPts);
    this._updateTracks(blobs, dtSec);

    // ZONES: per-zone people count, occupancy, dwell time (confirmed tracks only).
    const confirmed = this.confirmedTracks();
    const nowMs = Date.now();
    for (const t of confirmed) t.zone = '';
    const zoneInfo = this.zones.map((z) => {
      let cnt = 0;
      for (const t of confirmed) {
        if (pointInPoly(t.x, t.y, z.pts)) { cnt++; if (!t.zone) t.zone = z.name; }
      }
      const on = cnt > 0;
      let rt = this._zoneRT.get(z.slug);
      if (!rt) { rt = { since: 0 }; this._zoneRT.set(z.slug, rt); }
      if (on && !rt.since) rt.since = nowMs;
      if (!on) rt.since = 0;
      return { slug: z.slug, name: z.name, on, count: cnt, dwell: rt.since ? (nowMs - rt.since) / 1000 : 0 };
    });
    const zoneOcc = zoneInfo.map((z) => z.on);

    const frame = {
      pts,
      count,
      nbins: NBINS,
      bgCaptured: this.bgCaptured,
      capturing,
      zoneOcc,
      zoneInfo,
      tracks: confirmed.map((t) => {
        const [u, v] = applyH(this.warpH, t.x, t.y);
        return {
          id: t.id,
          x: t.x,
          y: t.y,
          u,
          v,
          out: u < 0 || u > 1 || v < 0 || v > 1, // outside the mapped area
          vx: t.vx,
          vy: t.vy,
          vel: Math.hypot(t.vx, t.vy),
          color: t.color,
          zone: t.zone || '',
        };
      }),
    };

    // baseline contour (world metres) for the orange dashed ghost outline
    if (this.bgCaptured) {
      const c = [];
      for (let i = 0; i < NBINS; i++) {
        const base = this.bg[i];
        if (base <= 0.001 || base > this.cfg.distMax) continue;
        const a = (i / NBINS) * 2 * Math.PI;
        const lx = Math.cos(a) * base;
        const ly = Math.sin(a) * base;
        c.push(px + lx * cosR - ly * sinR, py + lx * sinR + ly * cosR);
      }
      frame.bg = Float32Array.from(c);
    }

    return frame;
  }

  // DBSCAN clusters -> person-sized blobs (centroid + radius), walls rejected by size.
  _clusterBlobs(fgPts) {
    if (fgPts.length < CLUSTER_MINPTS) return [];
    const groups = dbscan(fgPts, CLUSTER_EPS, CLUSTER_MINPTS);
    const blobs = [];
    for (const g of groups) {
      if (g.length > MAX_BLOB_POINTS) continue;
      let cx = 0, cy = 0;
      for (const i of g) { cx += fgPts[i][0]; cy += fgPts[i][1]; }
      cx /= g.length; cy /= g.length;
      let r = 0;
      for (const i of g) {
        const d = Math.hypot(fgPts[i][0] - cx, fgPts[i][1] - cy);
        if (d > r) r = d;
      }
      if (r > MAX_BLOB_RADIUS) continue; // too big to be a person -> wall/furniture
      blobs.push({ x: cx, y: cy, r });
    }
    return blobs;
  }

  // Nearest-neighbor assignment with a max-jump gate; age out missing tracks.
  // A track must persist CONFIRM_HITS frames before it gets a (small, stable) public
  // id — this kills the flicker/noise that otherwise spawns thousands of throwaway IDs.
  _updateTracks(blobs, dt) {
    const used = new Set();
    for (const t of this.tracks) {
      let best = -1;
      let bestD = MAX_JUMP;
      blobs.forEach((b, i) => {
        if (used.has(i)) return;
        const d = Math.hypot(b.x - t.x, b.y - t.y);
        if (d < bestD) { bestD = d; best = i; }
      });
      if (best >= 0) {
        const b = blobs[best];
        used.add(best);
        // one-euro smoothing on position (reduces jitter, stays responsive when moving)
        const sx = this.cfg.smooth ? this._oneEuro(t, 'x', b.x, dt) : b.x;
        const sy = this.cfg.smooth ? this._oneEuro(t, 'y', b.y, dt) : b.y;
        if (dt > 0) {
          t.vx = 0.6 * t.vx + 0.4 * ((sx - t.x) / dt);
          t.vy = 0.6 * t.vy + 0.4 * ((sy - t.y) / dt);
        }
        t.x = sx; t.y = sy; t.r = b.r; t.missed = 0; t.hits++;
        if (!t.id && t.hits >= CONFIRM_HITS) {
          t.id = String(++this._nextId).padStart(2, '0');
          t.color = TRACK_PALETTE[(this._nextId - 1) % TRACK_PALETTE.length];
        }
      } else {
        t.missed++;
      }
    }
    blobs.forEach((b, i) => {
      if (used.has(i)) return;
      this.tracks.push({ id: null, color: null, x: b.x, y: b.y, r: b.r, vx: 0, vy: 0, missed: 0, hits: 1 });
    });
    this.tracks = this.tracks.filter((t) => t.missed < MAX_MISSED);
  }

  // One-euro filter (per track, per axis). Adaptive low-pass: smooths when still,
  // tracks quickly when moving.
  _oneEuro(t, ax, x, dt) {
    if (!(dt > 0)) return x;
    const st = t._oe || (t._oe = {});
    const s = st[ax] || (st[ax] = { xPrev: x, dxPrev: 0, init: false });
    const alpha = (cutoff) => { const tau = 1 / (2 * Math.PI * cutoff); return 1 / (1 + tau / dt); };
    if (!s.init) { s.init = true; s.xPrev = x; s.dxPrev = 0; return x; }
    const dx = (x - s.xPrev) / dt;
    const ad = alpha(this.cfg.smoothDcutoff);
    const edx = ad * dx + (1 - ad) * s.dxPrev;
    const cutoff = this.cfg.smoothMin + this.cfg.smoothBeta * Math.abs(edx);
    const a = alpha(cutoff);
    const fx = a * x + (1 - a) * s.xPrev;
    s.xPrev = fx; s.dxPrev = edx;
    return fx;
  }

  // confirmed tracks only (have a public id)
  confirmedTracks() { return this.tracks.filter((t) => t.id); }
}

module.exports = { Pipeline, NBINS, angleToBin };
