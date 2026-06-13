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
    };
    // per-bin scratch reused each scan
    this._dist = new Float32Array(NBINS); // metres, 0 = empty
  }

  setConfig(patch) {
    if (patch.quality !== undefined) this.cfg.quality = !!patch.quality;
    if (patch.distMin !== undefined) this.cfg.distMin = parseFloat(patch.distMin) || 0;
    if (patch.distMax !== undefined) this.cfg.distMax = parseFloat(patch.distMax) || 25;
    if (patch.placement) {
      this.cfg.placement = {
        x: parseFloat(patch.placement.x) || 0,
        y: parseFloat(patch.placement.y) || 0,
        rot: parseFloat(patch.placement.rot) || 0,
      };
    }
  }

  // nodes: [{ angle (deg), distMm, quality }]
  // returns a frame: { pts: Float32Array[x,y,fg per bin], count, nbins }
  process(nodes) {
    const dist = this._dist;
    dist.fill(0);

    const { x: px, y: py, rot } = this.cfg.placement;
    const cosR = Math.cos(rot * DEG);
    const sinR = Math.sin(rot * DEG);

    // pts layout: 3 floats per bin -> [worldX, worldY, fg]; fg = -1 means empty
    const pts = new Float32Array(NBINS * 3);
    for (let i = 0; i < NBINS; i++) pts[i * 3 + 2] = -1;

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

      if (pts[idx * 3 + 2] < 0) count++; // first hit for this bin
      dist[idx] = distM;
      pts[idx * 3] = wx;
      pts[idx * 3 + 1] = wy;
      pts[idx * 3 + 2] = 1; // foreground=1 (background subtraction added in step 3)
    }

    return { pts, count, nbins: NBINS };
  }
}

module.exports = { Pipeline, NBINS, angleToBin };
