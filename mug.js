// Mug (two-cylinder object) frame and the "handle towards the user's head" cone.
// Mug frame: origin = centre of cylinder B (the handle the user grasps); z = A->B (handle direction);
// y = cylinder axis, orthogonal to z and as vertical as possible; x = y x z.
// Rotations are flat row-major 3x3 arrays; root_link frame unless stated.
export const DEG = Math.PI / 180;
export const CONE_DEG = 45;
// Seated user head centre in U (pelvis frame), as drawn on the page.
export const HEAD_U = [0, 0, 0.63];

export function rotationFromZ(z) {
  const n = Math.hypot(...z), Z = z.map(v => v / n);
  let y = [-Z[2] * Z[0], -Z[2] * Z[1], 1 - Z[2] * Z[2]];  // up minus its component along z
  const m = Math.hypot(...y); y = y.map(v => v / m);
  const x = [y[1] * Z[2] - y[2] * Z[1], y[2] * Z[0] - y[0] * Z[2], y[0] * Z[1] - y[1] * Z[0]];
  return [x[0], y[0], Z[0], x[1], y[1], Z[1], x[2], y[2], Z[2]];
}
export const rows = R => [R.slice(0, 3), R.slice(3, 6), R.slice(6, 9)];
// root_link <-> U (user pelvis, x gaze, y left, z up; root_link at (d, y0, h) in U with yaw 180 deg)
export const headRoot = pl => [pl.d - HEAD_U[0], pl.y0 - HEAD_U[1], HEAD_U[2] - pl.h];
export const rootToU = (p, pl) => [pl.d - p[0], pl.y0 - p[1], p[2] + pl.h];
export const rotRootToU = R => R.map((v, i) => i < 6 ? -v : v);  // diag(-1,-1,1) R
export const toHead = (p, pl) => { const h = headRoot(pl), v = h.map((x, k) => x - p[k]), n = Math.hypot(...v); return v.map(x => x / n); };

// Indices of the direction samples set in mask (words per point = W, at point i) within the cone around u.
export function feasibleDirs(mask, W, i, dirs, u, coneDeg = CONE_DEG) {
  const c = Math.cos(coneDeg * DEG) - 1e-9, out = [];
  for (let w = 0; w < W; w++) {
    let bits = mask[i * W + w];
    while (bits) {
      const b = 31 - Math.clz32(bits); bits &= ~(1 << b);
      const k = 32 * w + b, d = dirs[k];
      if (d[0] * u[0] + d[1] * u[1] + d[2] * u[2] >= c) out.push(k);
    }
  }
  return out;
}
export const angleDeg = (a, b) => Math.acos(Math.max(-1, Math.min(1, a.reduce((s, v, k) => s + v * b[k], 0)))) / DEG;
