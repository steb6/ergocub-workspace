// Integer lattice coverage: hash the raw int16 indices, then build summed-volume
// tables. Each candidate box costs eight table reads per coverage count.
export function createPlacementSearch(left, right, step = 0.02) {
  const key = (x, y, z) => (x + 32768) * 4294967296 + (y + 32768) * 65536 + z + 32768;
  const sets = [left, right].map(indices => {
    const set = new Set();
    for (let i = 0; i < indices.length; i += 3) set.add(key(indices[i], indices[i + 1], indices[i + 2]));
    return set;
  });
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const indices of [left, right]) for (let i = 0; i < indices.length; i++) {
    const k = i % 3; lo[k] = Math.min(lo[k], indices[i]); hi[k] = Math.max(hi[k], indices[i]);
  }
  if (!sets[0].size && !sets[1].size) { lo.fill(0); hi.fill(0); }
  const [nx, ny, nz] = hi.map((v, k) => v - lo[k] + 2), yz = ny * nz;
  const union = new Uint32Array(nx * yz), both = new Uint32Array(nx * yz);
  for (let x = 1; x < nx; x++) for (let y = 1; y < ny; y++) for (let z = 1; z < nz; z++) {
    const id = x * yz + y * nz + z, k = key(x + lo[0] - 1, y + lo[1] - 1, z + lo[2] - 1);
    const l = sets[0].has(k), r = sets[1].has(k);
    union[id] = l || r ? 1 : 0; both[id] = l && r ? 1 : 0;
  }
  // Three cumulative passes avoid unsigned intermediate arithmetic pitfalls.
  for (const table of [union, both]) {
    for (let x = 1; x < nx; x++) for (let y = 1; y < ny; y++) for (let z = 1; z < nz; z++) {
      const i = x * yz + y * nz + z; table[i] += table[i - 1];
    }
    for (let x = 1; x < nx; x++) for (let y = 1; y < ny; y++) for (let z = 1; z < nz; z++) {
      const i = x * yz + y * nz + z; table[i] += table[i - nz];
    }
    for (let x = 1; x < nx; x++) for (let y = 1; y < ny; y++) for (let z = 1; z < nz; z++) {
      const i = x * yz + y * nz + z; table[i] += table[i - yz];
    }
  }
  function count(table, a, b) {
    const l = a.map((v, k) => Math.max(v, lo[k]) - lo[k]);
    const h = b.map((v, k) => Math.min(v, hi[k]) - lo[k] + 1);
    if (l.some((v, k) => v >= h[k])) return 0;
    const at = (x, y, z) => table[x * yz + y * nz + z];
    return at(...h) - at(l[0], h[1], h[2]) - at(h[0], l[1], h[2]) - at(h[0], h[1], l[2])
      + at(l[0], l[1], h[2]) + at(l[0], h[1], l[2]) + at(h[0], l[1], l[2]) - at(...l);
  }
  function coverage(box, placement) {
    const center = [placement.d - box.center[0], placement.y0 - box.center[1], box.center[2] - placement.h];
    const a = center.map((v, k) => Math.ceil((v - box.size[k] / 2 - 1e-7) / step));
    const b = center.map((v, k) => Math.floor((v + box.size[k] / 2 + 1e-7) / step));
    const samples = a.reduce((n, v, k) => n * Math.max(0, b[k] - v + 1), 1);
    const counts = { union: count(union, a, b), both: count(both, a, b) };
    return { samples, counts, fractions: { union: samples ? counts.union / samples : 0, both: samples ? counts.both / samples : 0 } };
  }
  function findBest(box) {
    let best = null, candidates = 0;
    const minD = box.center[0] + box.size[0] / 2 + 0.15;
    for (let di = 25; di <= 60; di++) {
      const d = di * 0.02; if (d + 1e-9 < minD) continue;
      for (let yi = -25; yi <= 25; yi++) for (let hi = 12; hi <= 16; hi++) { // seated user: pelvis 0.46-0.54 m above floor
        const placement = { d: +d.toFixed(2), y0: +(yi * 0.02).toFixed(2), h: +(hi * 0.02).toFixed(2) };
        const c = coverage(box, placement); candidates++;
        if (!best || c.fractions.union > best.coverage.fractions.union ||
            (c.fractions.union === best.coverage.fractions.union && (c.fractions.both > best.coverage.fractions.both ||
            (c.fractions.both === best.coverage.fractions.both && placement.d > best.placement.d)))) best = { placement, coverage: c };
      }
    }
    return best ? { ...best, candidates } : null;
  }
  return { coverage, findBest };
}
