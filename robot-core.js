const DEG=Math.PI/180;
const mm = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
const mv = (A, v) => A.map(r => r.reduce((s, x, k) => s + x * v[k], 0));
const T3 = A => A[0].map((_, j) => A.map(r => r[j]));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function rodrigues(a, q) {
  const c = Math.cos(q), s = Math.sin(q), t = 1 - c, [x, y, z] = a;
  return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y], [t * x * y + s * z, t * y * y + c, t * y * z - s * x], [t * x * z - s * y, t * y * z + s * x, t * z * z + c]];
}
export function fk(ch, q) {
  let R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], p = [0, 0, 0]; const zs = [], ps = [];
  for (let i = 0; i < ch.T.length; i++) {
    const T = ch.T[i];
    p = p.map((v, k) => v + mv(R, [T[0][3], T[1][3], T[2][3]])[k]);
    R = mm(R, T.slice(0, 3).map(r => r.slice(0, 3)));
    zs.push(mv(R, ch.axis[i])); ps.push(p.slice());
    R = mm(R, rodrigues(ch.axis[i], q[i]));
  }
  const tl = ch.tool; p = p.map((v, k) => v + mv(R, [tl[0][3], tl[1][3], tl[2][3]])[k]);
  R = mm(R, tl.slice(0, 3).map(r => r.slice(0, 3)));
  const J = [[], [], [], [], [], []];
  for (let i = 0; i < ch.T.length; i++) { const c = cross(zs[i], p.map((v, k) => v - ps[i][k])); for (let k = 0; k < 3; k++) { J[k][i] = c[k]; J[k + 3][i] = zs[i][k]; } }
  return { p, R, J };
}
function solve(A, b) { // Gaussian elimination with partial pivoting
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let m = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[m][c])) m = r;
    [M[c], M[m]] = [M[m], M[c]];
    for (let r = c + 1; r < n; r++) { const f = M[r][c] / M[c][c]; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = M[r][n]; for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k]; x[r] = s / M[r][r]; }
  return x;
}
const rotErr = (Rd, R) => {
  const E=mm(Rd,T3(R)), c=Math.max(-1,Math.min(1,(E[0][0]+E[1][1]+E[2][2]-1)/2)), angle=Math.acos(c);
  const v=[E[2][1]-E[1][2],E[0][2]-E[2][0],E[1][0]-E[0][1]];
  if(angle<1e-7)return v.map(x=>x/2);
  if(Math.PI-angle<1e-5) {
    let i=0;for(let k=1;k<3;k++)if(E[k][k]>E[i][i])i=k;
    const a=[0,0,0];a[i]=Math.sqrt(Math.max(0,(E[i][i]+1)/2));
    for(let k=0;k<3;k++)if(k!==i)a[k]=(E[k][i]+E[i][k])/(4*a[i]);
    return a.map(x=>x*angle);
  }
  return v.map(x=>x*angle/(2*Math.sin(angle)));
};
export const rotAngle = (Rd, R) => { const E = mm(Rd, T3(R)); return Math.acos(Math.min(1, Math.max(-1, (E[0][0] + E[1][1] + E[2][2] - 1) / 2))); };

// damped least squares + joint-limit clamping, several seeds (same scheme as the offline grid)
// options.dir: desired index pointing direction (root frame, unit); soft axis-alignment task, rotation about the finger free.
const fingerAxis = (ch, R) => mv(R, ch.finger_dir);
const dirAngle = (ch, R, d) => Math.acos(Math.min(1, Math.max(-1, fingerAxis(ch, R).reduce((s, v, k) => s + v * d[k], 0))));
export function robotIK(ch, target, Rd, options = {}) {
  const lo = ch.qmin, hi = ch.qmax, wr = 0.3, dd = options.dir, wd = 0.3;
  const seeds = [...(options.seeds || []), ...(options.current ? [options.current] : []), lo.map(() => 0), lo.map((l, i) => (l + hi[i]) / 2)];
  for (let k = 0; k < (options.randomSeeds ?? 25); k++) seeds.push(lo.map((l, i) => l + (options.rng || Math.random)() * (hi[i] - l)));
  let best = null;
  for (const s of seeds) {
    let q = s.map((v, i) => Math.min(hi[i], Math.max(lo[i], v)));
    for (let it = 0; it < (options.iterations ?? 200); it++) {
      const { p, R, J } = fk(ch, q);
      let e = target.map((v, k) => v - p[k]), Jt = J.slice(0, 3);
      if (Math.hypot(...e) < 0.001 && (!Rd || rotAngle(Rd, R) < .5 * DEG) && (!dd || dirAngle(ch, R, dd) < 3 * DEG)) break;
      if (Rd) { e = e.concat(rotErr(Rd, R).map(v => wr * v)); Jt = Jt.concat(J.slice(3).map(r => r.map(v => wr * v))); }
      else if (dd) {  // a x a_des = rotation that aligns the finger; project the angular Jacobian orthogonally to a
        const a = fingerAxis(ch, R), P = [0, 1, 2].map(i => [0, 1, 2].map(j => (i === j ? 1 : 0) - a[i] * a[j]));
        const Jw = J.slice(3), Jp = P.map(r => Jw[0].map((_, c) => r.reduce((s2, v, k) => s2 + v * Jw[k][c], 0)));
        e = e.concat(cross(a, dd).map(v => wd * v)); Jt = Jt.concat(Jp.map(r => r.map(v => wd * v)));
      }
      const A = Jt.map(r => Jt.map(r2 => r.reduce((s2, v, k) => s2 + v * r2[k], 0)));
      A.forEach((r, i) => r[i] += 0.02 * 0.02);
      const y = solve(A, e);
      q = q.map((v, i) => { const d = Math.max(-0.2, Math.min(0.2, Jt.reduce((s2, r, k) => s2 + r[i] * y[k], 0))); return Math.min(hi[i], Math.max(lo[i], v + d)); });
    }
    const { p, R } = fk(ch, q);
    const err = Math.hypot(...target.map((v, k) => v - p[k])), ang = Rd ? rotAngle(Rd, R) : 0, dang = dd ? dirAngle(ch, R, dd) : 0;
    const score = err + (Rd && ang > 1 * DEG ? 1 : 0) + (dd ? 0.02 * dang : 0);  // position first, then pointing
    if (!best || score < best.score) best = { q, err, ang, dang, score };
    if (err < 0.002 && (!Rd || ang < 1 * DEG) && (!dd || dang < 10 * DEG)) break;
  }
  return best;
}

