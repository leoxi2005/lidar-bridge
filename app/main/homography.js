'use strict';
// Perspective transform (corner-pin homography). Maps 4 source corners in world
// metres -> the unit square (0,0)(1,0)(1,1)(0,1). Solve the 8-equation linear
// system by Gaussian elimination, exactly like the prototype's computeH/solve8.
// Shared by the main process (track -> u,v for output) and copied into the renderer
// for the warp overlay / output monitor.

// Solve A x = b for n unknowns (in-place Gaussian elimination with partial pivot).
function solve(A, b, n) {
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]];
    [b[col], b[piv]] = [b[piv], b[col]];
    const d = A[col][col] || 1e-12;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = A[r][col] / d;
      for (let c = col; c < n; c++) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }
  const x = new Array(n);
  for (let i = 0; i < n; i++) x[i] = b[i] / (A[i][i] || 1e-12);
  return x;
}

// src: [[x,y]*4] world metres; dst: [[u,v]*4] (defaults to the unit square).
// Returns the 9-element row-major homography [h0..h8] with h8 = 1.
function computeH(src, dst) {
  const D = dst || [[0, 0], [1, 0], [1, 1], [0, 1]];
  const A = [];
  const b = [];
  for (let i = 0; i < 4; i++) {
    const [x, y] = src[i];
    const [u, v] = D[i];
    A.push([x, y, 1, 0, 0, 0, -x * u, -y * u]);
    b.push(u);
    A.push([0, 0, 0, x, y, 1, -x * v, -y * v]);
    b.push(v);
  }
  const h = solve(A, b, 8);
  h.push(1);
  return h;
}

// Apply homography: world (x,y) -> normalized (u,v).
function applyH(H, x, y) {
  const d = H[6] * x + H[7] * y + H[8] || 1e-12;
  return [(H[0] * x + H[1] * y + H[2]) / d, (H[3] * x + H[4] * y + H[5]) / d];
}

// Invert a 3x3 (row-major) — used by the renderer to map the unit square back to world.
function invert3(H) {
  const [a, b, c, d, e, f, g, h, i] = H;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C || 1e-12;
  const id = 1 / det;
  return [
    A * id, (c * h - b * i) * id, (b * f - c * e) * id,
    B * id, (a * i - c * g) * id, (c * d - a * f) * id,
    C * id, (b * g - a * h) * id, (a * e - b * d) * id,
  ];
}

module.exports = { computeH, applyH, invert3 };
