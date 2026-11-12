import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import URDFLoader from 'urdf-loader';

const $ = id => document.getElementById(id);
const DEG = Math.PI / 180;

// ---------- scene: everything lives in `world` = root_link frame (z up) ----------
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1d2026);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.position.set(1.5, 0.9, 1.0);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0.05, 0.3, 0); orbit.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x444455, 2.0));
const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(2, 3, 2); scene.add(sun);
const world = new THREE.Group(); world.rotation.x = -Math.PI / 2; scene.add(world);
const grid = new THREE.GridHelper(2, 20, 0x555555, 0x333333); grid.rotation.x = Math.PI / 2; world.add(grid);
world.add(new THREE.AxesHelper(0.15));

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize); resize();

// ---------- robot ----------
let robot = null;
new URDFLoader().load('ergocub.urdf', r => {
  robot = r; world.add(r);
  const mat = new THREE.MeshStandardMaterial({ color: 0xc8ccd2, metalness: 0.1, roughness: 0.6 });
  r.traverse(o => { if (o.isMesh) o.material = mat; });
  $('loading').remove();
  window.robot = r;
});
$('c_robot').onchange = e => { if (robot) robot.visible = e.target.checked; };

// ---------- arm chains (same as offline: ergocub-cartesian-control chain + control-board limits) ----------
const chains = await (await fetch('data/chains.json')).json();
const meta = await (await fetch('data/meta.json')).json();
const qcur = { l: new Array(7).fill(0), r: new Array(7).fill(0) };
const R_DOWN = { l: [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], r: [[0, 0, -1], [1, 0, 0], [0, -1, 0]] };

const mm = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
const mv = (A, v) => A.map(r => r.reduce((s, x, k) => s + x * v[k], 0));
const T3 = A => A[0].map((_, j) => A.map(r => r[j]));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function rodrigues(a, q) {
  const c = Math.cos(q), s = Math.sin(q), t = 1 - c, [x, y, z] = a;
  return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y], [t * x * y + s * z, t * y * y + c, t * y * z - s * x], [t * x * z - s * y, t * y * z + s * x, t * z * z + c]];
}
function fk(ch, q) {
  let R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], p = [0, 0, 0]; const zs = [], ps = [];
  for (let i = 0; i < 7; i++) {
    const T = ch.T[i];
    p = p.map((v, k) => v + mv(R, [T[0][3], T[1][3], T[2][3]])[k]);
    R = mm(R, T.slice(0, 3).map(r => r.slice(0, 3)));
    zs.push(mv(R, ch.axis[i])); ps.push(p.slice());
    R = mm(R, rodrigues(ch.axis[i], q[i]));
  }
  const tl = ch.tool; p = p.map((v, k) => v + mv(R, [tl[0][3], tl[1][3], tl[2][3]])[k]);
  R = mm(R, tl.slice(0, 3).map(r => r.slice(0, 3)));
  const J = [[], [], [], [], [], []];
  for (let i = 0; i < 7; i++) { const c = cross(zs[i], p.map((v, k) => v - ps[i][k])); for (let k = 0; k < 3; k++) { J[k][i] = c[k]; J[k + 3][i] = zs[i][k]; } }
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
const rotErr = (Rd, R) => { const E = mm(Rd, T3(R)); return [0.5 * (E[2][1] - E[1][2]), 0.5 * (E[0][2] - E[2][0]), 0.5 * (E[1][0] - E[0][1])]; };
const rotAngle = (Rd, R) => { const E = mm(Rd, T3(R)); return Math.acos(Math.min(1, Math.max(-1, (E[0][0] + E[1][1] + E[2][2] - 1) / 2))); };

// damped least squares + joint-limit clamping, several seeds (same scheme as the offline grid)
function ik(side, target, Rd) {
  const ch = chains[side], lo = ch.qmin, hi = ch.qmax, wr = 0.3;
  const seeds = [qcur[side].slice(), lo.map(() => 0), lo.map((l, i) => (l + hi[i]) / 2)];
  for (let k = 0; k < 25; k++) seeds.push(lo.map((l, i) => l + Math.random() * (hi[i] - l)));
  let best = null;
  for (const s of seeds) {
    let q = s.map((v, i) => Math.min(hi[i], Math.max(lo[i], v)));
    for (let it = 0; it < 200; it++) {
      const { p, R, J } = fk(ch, q);
      let e = target.map((v, k) => v - p[k]), Jt = J.slice(0, 3);
      if (Rd) { e = e.concat(rotErr(Rd, R).map(v => wr * v)); Jt = Jt.concat(J.slice(3).map(r => r.map(v => wr * v))); }
      const A = Jt.map(r => Jt.map(r2 => r.reduce((s2, v, k) => s2 + v * r2[k], 0)));
      A.forEach((r, i) => r[i] += 0.02 * 0.02);
      const y = solve(A, e);
      q = q.map((v, i) => { const d = Math.max(-0.2, Math.min(0.2, Jt.reduce((s2, r, k) => s2 + r[i] * y[k], 0))); return Math.min(hi[i], Math.max(lo[i], v + d)); });
    }
    const { p, R } = fk(ch, q);
    const err = Math.hypot(...target.map((v, k) => v - p[k])), ang = Rd ? rotAngle(Rd, R) : 0;
    const score = err + (Rd && ang > 10 * DEG ? 1 : 0);
    if (!best || score < best.score) best = { q, err, ang, score };
    if (err < 0.002 && (!Rd || ang < 5 * DEG)) break;
  }
  return best;
}

const targetMarker = new THREE.Mesh(new THREE.SphereGeometry(0.015), new THREE.MeshBasicMaterial({ color: 0xff3366 }));
targetMarker.visible = false; world.add(targetMarker);
function setArm(side, q) { qcur[side] = q; if (robot) chains[side].joints.forEach((n, i) => robot.setJointValue(n, q[i])); }
function goTo(target) {
  const side = $('arm').value, Rd = $('ikori').value === 'down' ? R_DOWN[side] : null;
  targetMarker.position.set(...target); targetMarker.visible = true;
  const b = ik(side, target, Rd);
  setArm(side, b.q);
  const ok = b.err < 0.01 && (!Rd || b.ang < 10 * DEG);
  const msg = `${ok ? 'reached' : 'NOT reachable'}: position error ${(b.err * 100).toFixed(1)} cm` +
    (Rd ? `, orientation error ${(b.ang / DEG).toFixed(1)}°` : '') +
    `\njoints [deg]: ${b.q.map(v => (v / DEG).toFixed(0)).join(' ')}`;
  $('ikmsg').innerHTML = `<span class="${ok ? 'ok' : 'bad'}">${msg}</span>`;
  window.lastIK = b;
}
$('go').onclick = () => goTo(['tx', 'ty', 'tz'].map(i => parseFloat($(i).value)));
$('home').onclick = () => { setArm('l', new Array(7).fill(0)); setArm('r', new Array(7).fill(0)); targetMarker.visible = false; $('ikmsg').textContent = ''; };

// ---------- point clouds ----------
const COLORS = { left: new THREE.Color(0x4a90ff), right: new THREE.Color(0xff9f40), both: new THREE.Color(0x3ddc84) };
const HL = new THREE.Color(0xffee00);
const clouds = {}; // clouds[variant][name] = THREE.Points
const pmat = name => new THREE.PointsMaterial({ size: 0.008, vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false });
for (const v of ['free', 'down']) {
  clouds[v] = {};
  for (const n of ['left', 'right', 'both']) {
    const buf = await (await fetch(`data/${v}_${n}.bin`)).arrayBuffer();
    const idx = new Int16Array(buf), pos = new Float32Array(idx.length);
    for (let i = 0; i < idx.length; i++) pos[i] = idx[i] * meta.step;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const col = new Float32Array(pos.length); for (let i = 0; i < pos.length; i += 3) COLORS[n].toArray(col, i);
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const pts = new THREE.Points(g, pmat(n)); pts.name = n; pts.visible = false;
    clouds[v][n] = pts; world.add(pts);
  }
}
function refreshClouds() {
  const v = $('variant').value;
  for (const vv in clouds) for (const n in clouds[vv]) clouds[vv][n].visible = vv === v && $('c_' + n).checked;
  for (const n of ['left', 'right', 'both']) $('n_' + n).textContent = `(${meta[v + '_' + n]} pts)`;
  const s = $('psize').value / 1000;
  for (const vv in clouds) for (const n in clouds[vv]) clouds[vv][n].material.size = s;
  updateRegion();
}
['variant', 'c_left', 'c_right', 'c_both', 'psize'].forEach(i => $(i).addEventListener('input', refreshClouds));

// Shift+click on a visible point -> IK target
const ray = new THREE.Raycaster(); ray.params.Points.threshold = 0.01;
renderer.domElement.addEventListener('pointerdown', ev => {
  if (!ev.shiftKey) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1), camera);
  const vis = Object.values(clouds[$('variant').value]).filter(p => p.visible);
  const hit = ray.intersectObjects(vis)[0];
  if (!hit) return;
  const a = hit.object.geometry.attributes.position, t = [a.getX(hit.index), a.getY(hit.index), a.getZ(hit.index)];
  ['tx', 'ty', 'tz'].forEach((id, k) => $(id).value = t[k].toFixed(3));
  if (hit.object.name !== 'both') $('arm').value = hit.object.name[0];
  goTo(t);
});

// ---------- region ----------
const reg = { type: 'box', center: [0.25, 0.0, 0.35], dims: { box: [0.3, 0.4, 0.3], sphere: [0.15], ellipsoid: [0.15, 0.25, 0.12] }, rpy: [0, 0, 0] };
const regMat = new THREE.MeshBasicMaterial({ color: 0x66d9ff, transparent: true, opacity: 0.07, depthWrite: false, side: THREE.DoubleSide });
const regMesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), regMat);
const regEdges = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: 0x66d9ff }));
regMesh.add(regEdges); world.add(regMesh); regMesh.visible = false;
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.addEventListener('dragging-changed', e => orbit.enabled = !e.value);
gizmo.addEventListener('objectChange', () => {
  reg.center = regMesh.position.toArray().map(v => +v.toFixed(3));
  const e = new THREE.Euler().setFromQuaternion(regMesh.quaternion, 'ZYX');
  reg.rpy = [e.x, e.y, e.z].map(v => +(v / DEG).toFixed(1));
  syncSliders(); updateRegion();
});
scene.add(gizmo);

function sliderRow(label, get, set, min, max, step) {
  const row = document.createElement('div'); row.className = 'row';
  row.innerHTML = `<label class="k">${label}</label><input type="range" min="${min}" max="${max}" step="${step}"><input type="number" min="${min}" max="${max}" step="${step}">`;
  const [rg, nb] = row.querySelectorAll('input');
  const upd = v => { set(parseFloat(v)); rg.value = nb.value = get(); updateRegion(); };
  rg.oninput = () => upd(rg.value); nb.onchange = () => upd(nb.value);
  row.sync = () => { rg.value = nb.value = get(); };
  return row;
}
let rows = [];
function buildSliders() {
  const box = $('sliders'); box.innerHTML = ''; rows = [];
  ['x', 'y', 'z'].forEach((a, k) => rows.push(sliderRow('center ' + a, () => reg.center[k], v => reg.center[k] = v, -0.6, 1.0, 0.005)));
  const names = { box: ['size x', 'size y', 'size z'], sphere: ['radius'], ellipsoid: ['semi-ax a', 'semi-ax b', 'semi-ax c'] }[reg.type];
  names.forEach((nm, k) => rows.push(sliderRow(nm, () => reg.dims[reg.type][k], v => reg.dims[reg.type][k] = Math.max(0.005, v), 0.005, 1.2, 0.005)));
  if (reg.type !== 'sphere') ['roll', 'pitch', 'yaw'].forEach((a, k) => rows.push(sliderRow(a + ' °', () => reg.rpy[k], v => reg.rpy[k] = v, -180, 180, 1)));
  rows.forEach(r => { box.appendChild(r); r.sync(); });
}
const syncSliders = () => rows.forEach(r => r.sync());
$('rtype').onchange = e => {
  reg.type = e.target.value;
  const g = reg.type === 'box' ? new THREE.BoxGeometry(1, 1, 1) : new THREE.SphereGeometry(reg.type === 'sphere' ? 1 : 1, 32, 16);
  regMesh.geometry = g; regEdges.geometry = reg.type === 'box' ? new THREE.EdgesGeometry(g) : new THREE.WireframeGeometry(new THREE.SphereGeometry(1, 16, 8));
  if (reg.type === 'sphere') reg.rpy = [0, 0, 0];
  buildSliders(); updateRegion();
};
const applyGizmo = () => {
  const m = $('gizmo').value;
  if (m === 'off' || !$('c_region').checked) gizmo.detach(); else { gizmo.attach(regMesh); gizmo.setMode(m); }
};
$('gizmo').onchange = applyGizmo;
$('c_region').onchange = () => { regMesh.visible = $('c_region').checked; applyGizmo(); updateRegion(); };
$('c_hl').onchange = () => updateRegion();

const insideIdx = {};
function updateRegion() {
  regMesh.position.set(...reg.center);
  regMesh.quaternion.setFromEuler(new THREE.Euler(reg.rpy[0] * DEG, reg.rpy[1] * DEG, reg.rpy[2] * DEG, 'ZYX')); // URDF rpy convention
  const d = reg.dims[reg.type];
  regMesh.scale.set(...(reg.type === 'sphere' ? [d[0], d[0], d[0]] : d));
  const v = $('variant').value, on = $('c_region').checked, hl = on && $('c_hl').checked;
  const inv = new THREE.Matrix4().compose(regMesh.position, regMesh.quaternion, new THREE.Vector3(1, 1, 1)).invert();
  const p = new THREE.Vector3(); const counts = {};
  for (const n of ['left', 'right', 'both']) {
    const g = clouds[v][n].geometry, a = g.attributes.position, c = g.attributes.color; insideIdx[n] = [];
    for (let i = 0; i < a.count; i++) {
      let inside = false;
      if (on) {
        p.fromBufferAttribute(a, i).applyMatrix4(inv);
        if (reg.type === 'box') inside = Math.abs(p.x) <= d[0] / 2 && Math.abs(p.y) <= d[1] / 2 && Math.abs(p.z) <= d[2] / 2;
        else if (reg.type === 'sphere') inside = p.length() <= d[0];
        else inside = (p.x / d[0]) ** 2 + (p.y / d[1]) ** 2 + (p.z / d[2]) ** 2 <= 1;
      }
      if (inside) insideIdx[n].push(i);
      (hl ? (inside ? HL : COLORS[n].clone().multiplyScalar(0.3)) : COLORS[n]).toArray(c.array, i * 3);
    }
    c.needsUpdate = true; counts[n] = insideIdx[n].length;
  }
  const vol = reg.type === 'box' ? d[0] * d[1] * d[2] : reg.type === 'sphere' ? 4 / 3 * Math.PI * d[0] ** 3 : 4 / 3 * Math.PI * d[0] * d[1] * d[2];
  const nGrid = vol / meta.step ** 3;
  $('regmsg').innerHTML = on ? `region volume ${(vol * 1000).toFixed(1)} L ≈ ${nGrid.toFixed(0)} grid cells\n` +
    ['left', 'right', 'both'].map(n => `${n}: ${counts[n]} reachable (${(100 * counts[n] / nGrid).toFixed(0)}% of region)`).join('\n') : '';
  window.regionCounts = counts;
}
$('dl').onclick = () => {
  const v = $('variant').value, q = regMesh.quaternion;
  const out = {
    type: reg.type, frame: 'root_link', units: 'm', center: reg.center,
    dims: reg.type === 'box' ? { size_xyz: reg.dims.box } : reg.type === 'sphere' ? { radius: reg.dims.sphere[0] } : { semi_axes_abc: reg.dims.ellipsoid },
    orientation: { rpy_deg: reg.rpy, rpy_convention: 'URDF (R = Rz(yaw) Ry(pitch) Rx(roll)), fixed axes of root_link', quaternion_xyzw: [q.x, q.y, q.z, q.w] },
    reachability_variant: v === 'free' ? 'position only, orientation free' : 'palm down, fingers forward (±10 deg)',
    reachable_counts: window.regionCounts,
    notes: 'ergoCub, torso locked at 0, fixed root; end-effector = l/r_hand_palm origin; grid 2 cm; reachable if IK position error < 1 cm',
  };
  if ($('c_dlpts').checked) {
    out.reachable_points = {};
    for (const n of ['left', 'right', 'both']) {
      const a = clouds[v][n].geometry.attributes.position;
      out.reachable_points[n] = insideIdx[n].map(i => [a.getX(i), a.getY(i), a.getZ(i)].map(x => +x.toFixed(3)));
    }
  }
  const url = URL.createObjectURL(new Blob([JSON.stringify(out, null, 1)], { type: 'application/json' }));
  Object.assign(document.createElement('a'), { href: url, download: `ergocub_region_${reg.type}.json` }).click();
};

buildSliders(); refreshClouds();
window.app = { goTo, fk, chains, reg, updateRegion, clouds };
renderer.setAnimationLoop(() => renderer.render(scene, camera));

// ---------- self-test / screenshot presets: index.html?test , ?region ----------
const qs = new URLSearchParams(location.search);
if (qs.has('test') || qs.has('region') || qs.has('down') || qs.has('ik')) {
  while (!robot) await new Promise(r => setTimeout(r, 100));
  if (qs.has('down')) { $('variant').value = 'down'; refreshClouds(); }
  if (qs.has('region')) { $('c_region').checked = true; $('c_region').onchange(); $('c_both').checked = true; refreshClouds(); }
  const out = [];
  for (const [side, t] of [['l', [0.25, 0.25, 0.35]], ['r', [0.2, -0.3, 0.2]], ['l', [0.9, 0.0, 0.3]]]) {
    $('arm').value = side; goTo(t);
    const w = new THREE.Vector3(); robot.links[side + '_hand_palm'].getWorldPosition(w); world.worldToLocal(w);
    const p = fk(chains[side], window.lastIK.q).p;
    out.push(`${side} target ${t} ik_err ${window.lastIK.err.toFixed(4)} urdf-vs-fk ${Math.hypot(w.x - p[0], w.y - p[1], w.z - p[2]).toExponential(2)}`);
  }
  $('dl').onclick.toString();
  out.push('region counts ' + JSON.stringify(window.regionCounts));
  if (qs.has('ik')) { // ?ik=side,x,y,z,ori  e.g. ik=l,0.3,0.2,0.3,down
    const [sd, x, y, z, o] = qs.get('ik').split(','); $('arm').value = sd; $('ikori').value = o || 'free';
    goTo([+x, +y, +z]); camera.position.set(0.9, 0.9, 0.5); orbit.target.set(+x, +z, -y); orbit.update();
  }
  const pre = document.createElement('pre'); pre.id = 'test'; pre.textContent = out.join('\n'); document.body.appendChild(pre);
}
