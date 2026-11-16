import { createPlacementSearch } from './placement.js?v=4';
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
const chains = await (await fetch('data/chains.json?v=4')).json();
const meta = await (await fetch('data/meta.json?v=4')).json();
const qcur = {}; for (const k in chains) qcur[k] = new Array(chains[k].joints.length).fill(0);
const tk = () => $('torso').value === 'free' ? 't' : '';  // chain/cloud prefix: '' torso locked, 't' torso free
const vkey = () => tk() + $('variant').value;
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
const rotErr = (Rd, R) => { const E = mm(Rd, T3(R)); return [0.5 * (E[2][1] - E[1][2]), 0.5 * (E[0][2] - E[2][0]), 0.5 * (E[1][0] - E[0][1])]; };
const rotAngle = (Rd, R) => { const E = mm(Rd, T3(R)); return Math.acos(Math.min(1, Math.max(-1, (E[0][0] + E[1][1] + E[2][2] - 1) / 2))); };

// damped least squares + joint-limit clamping, several seeds (same scheme as the offline grid)
function ik(side, target, Rd) {
  const key = tk() + side, ch = chains[key], lo = ch.qmin, hi = ch.qmax, wr = 0.3;
  const seeds = [qcur[key].slice(), lo.map(() => 0), lo.map((l, i) => (l + hi[i]) / 2)];
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
function setArm(key, q) {  // key = side or 't'+side; locked torso -> torso joints back to 0
  qcur[key] = q; if (!robot) return;
  if (key.length === 1) ['torso_roll', 'torso_pitch', 'torso_yaw'].forEach(n => robot.setJointValue(n, 0));
  chains[key].joints.forEach((n, i) => robot.setJointValue(n, q[i]));
}
function goTo(target) {
  const side = $('arm').value, Rd = $('ikori').value === 'down' ? R_DOWN[side] : null;
  targetMarker.position.set(...target); targetMarker.visible = true;
  const b = ik(side, target, Rd);
  setArm(tk() + side, b.q);
  const ok = b.err < 0.01 && (!Rd || b.ang < 10 * DEG);
  const msg = `${ok ? 'reached' : 'NOT reachable'}: position error ${(b.err * 100).toFixed(1)} cm` +
    (Rd ? `, orientation error ${(b.ang / DEG).toFixed(1)}°` : '') +
    `\njoints [deg]: ${b.q.map(v => (v / DEG).toFixed(0)).join(' ')}`;
  $('ikmsg').innerHTML = `<span class="${ok ? 'ok' : 'bad'}">${msg}</span>`;
  window.lastIK = b;
}
$('go').onclick = () => goTo(['tx', 'ty', 'tz'].map(i => parseFloat($(i).value)));
$('home').onclick = () => { for (const k of ['tl', 'tr']) setArm(k, new Array(10).fill(0)); setArm('l', new Array(7).fill(0)); setArm('r', new Array(7).fill(0)); targetMarker.visible = false; $('ikmsg').textContent = ''; };
$('torso').addEventListener('change', () => $('home').onclick());

// ---------- point clouds ----------
const COLORS = { left: new THREE.Color(0x4a90ff), right: new THREE.Color(0xff9f40), both: new THREE.Color(0x3ddc84) };
const HL = new THREE.Color(0xffee00);
const clouds = {}; // clouds[variant][name] = THREE.Points
const pmat = name => new THREE.PointsMaterial({ size: 0.008, vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false });
for (const v of ['free', 'down', 'tfree', 'tdown']) {
  clouds[v] = {};
  for (const n of ['left', 'right', 'both']) {
    const buf = await (await fetch(`data/${v}_${n}.bin?v=4`)).arrayBuffer();
    const idx = new Int16Array(buf), pos = new Float32Array(idx.length);
    for (let i = 0; i < idx.length; i++) pos[i] = idx[i] * meta.step;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const col = new Float32Array(pos.length); for (let i = 0; i < pos.length; i += 3) COLORS[n].toArray(col, i);
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const pts = new THREE.Points(g, pmat(n)); pts.name = n; pts.visible = false;
    pts.userData.indices = idx;
    clouds[v][n] = pts; world.add(pts);
  }
}
function refreshClouds() {
  const v = vkey();
  for (const vv in clouds) for (const n in clouds[vv]) clouds[vv][n].visible = vv === v && $('c_' + n).checked;
  for (const n of ['left', 'right', 'both']) $('n_' + n).textContent = `(${meta[v + '_' + n]} pts)`;
  const s = $('psize').value / 1000;
  for (const vv in clouds) for (const n in clouds[vv]) clouds[vv][n].material.size = s;
  updateRegion();
}
['torso', 'variant', 'c_left', 'c_right', 'c_both', 'psize'].forEach(i => $(i).addEventListener('input', refreshClouds));

// Shift+click on a visible point -> IK target
const ray = new THREE.Raycaster(); ray.params.Points.threshold = 0.01;
renderer.domElement.addEventListener('pointerdown', ev => {
  if (!ev.shiftKey) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1), camera);
  const vis = Object.values(clouds[vkey()]).filter(p => p.visible);
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
  if ($('c_collab').checked || m === 'off' || !$('c_region').checked) gizmo.detach(); else { gizmo.attach(regMesh); gizmo.setMode(m); }
};
$('gizmo').onchange = applyGizmo;
$('c_region').onchange = () => { regMesh.visible = $('c_region').checked; applyGizmo(); updateRegion(); };
$('c_hl').onchange = () => updateRegion();

// ---------- collaborator: geometry in U (x gaze, y left, z up) ----------
const placement = { d: 0.71, y0: -0.20, h: 0.27 };
const boxU = { center: [0.45, -0.20, 0.45], size: [0.30, 0.60, 0.40] };
const placementRows = [], boxRows = [];
const user = new THREE.Group(); user.rotation.z = Math.PI; user.visible = false; world.add(user);
const humanMat = new THREE.MeshStandardMaterial({ color: 0xb5aea4, transparent: true, opacity: 0.6, roughness: 0.9 });
const chairMat = new THREE.MeshStandardMaterial({ color: 0x777b82, transparent: true, opacity: 0.5 });
function bodyPart(size, position, material = humanMat) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
  mesh.position.set(...position); user.add(mesh); return mesh;
}
function limb(a, b, radius, material = humanMat) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 12), material);
  user.add(mesh); placeLimb(mesh, a, b); return mesh;
}
function placeLimb(mesh, a, b) {
  const start = new THREE.Vector3(...a), end = new THREE.Vector3(...b), delta = end.clone().sub(start);
  mesh.position.copy(start.add(end).multiplyScalar(0.5)); mesh.scale.y = delta.length();
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
}
bodyPart([0.20, 0.30, 0.16], [0, 0, 0]);
bodyPart([0.20, 0.36, 0.38], [-0.02, 0, 0.26]);
limb([0, 0, 0.45], [0, 0, 0.52], 0.055);
const head = new THREE.Mesh(new THREE.SphereGeometry(0.11, 20, 16), humanMat);
head.position.set(0, 0, 0.63); head.scale.z = 1.2; user.add(head);
bodyPart([0.05, 0.045, 0.045], [0.115, 0, 0.63]); // nose points along gaze
bodyPart([0.48, 0.46, 0.05], [0.10, 0, -0.11], chairMat);
bodyPart([0.05, 0.46, 0.43], [-0.15, 0, 0.12], chairMat);
const shanks = [], feet = [], chairLegs = [];
for (const y of [-0.12, 0.12]) {
  limb([0, y, 0], [0.36, y, -0.03], 0.075);
  shanks.push(limb([0.36, y, -0.03], [0.36, y, -0.46], 0.05));
  feet.push(bodyPart([0.23, 0.10, 0.06], [0.43, y, -0.48]));
  if (y > 0) { // left arm rests on the lap
    limb([0, y * 1.8, 0.42], [0.10, y * 1.8, 0.16], 0.043);
    limb([0.10, y * 1.8, 0.16], [0.32, y, 0.08], 0.035);
  }
  for (const x of [-0.10, 0.30]) chairLegs.push({ mesh: limb([x, y * 1.6, -0.13], [x, y * 1.6, -0.5], 0.018, chairMat), x, y: y * 1.6 });
}
const rightUpper = limb([0, -0.216, 0.42], [0.25, -0.22, 0.30], 0.043);
const rightForearm = limb([0.25, -0.22, 0.30], [0.45, -0.20, 0.29], 0.035);
const rightHand = bodyPart([0.11, 0.075, 0.035], [0.45, -0.20, 0.29]);
const tableMat = new THREE.MeshStandardMaterial({ color: 0x90969d, transparent: true, opacity: 0.35, depthWrite: false });
const tableTop = bodyPart([1, 1, 1], [0, 0, 0], tableMat);
const tableLegs = Array.from({ length: 4 }, () => bodyPart([1, 1, 1], [0, 0, 0], tableMat));
user.add(new THREE.AxesHelper(0.22));
const labelCanvas = document.createElement('canvas'); labelCanvas.width = 256; labelCanvas.height = 48;
const labelContext = labelCanvas.getContext('2d'); labelContext.font = '28px sans-serif'; labelContext.fillStyle = '#ffffff';
labelContext.fillText('user pelvis', 5, 34);
const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(labelCanvas), depthTest: false }));
label.position.set(-0.10, -0.30, 0.02); label.scale.set(0.34, 0.064, 1); user.add(label);
function updatePlacement() {
  const { d, y0, h } = placement, floor = h - 0.779;
  user.position.set(d, y0, -h);
  shanks.forEach((mesh, k) => placeLimb(mesh, [0.36, [-0.12, 0.12][k], -0.03], [0.36, [-0.12, 0.12][k], floor + 0.06]));
  feet.forEach(mesh => mesh.position.z = floor + 0.03);
  chairLegs.forEach(({ mesh, x, y }) => placeLimb(mesh, [x, y, -0.13], [x, y, floor]));
  $('pelvis_height').textContent = `User pelvis above floor: ${(0.779 - h).toFixed(3)} m`;
  const surface = boxU.center[2] - boxU.size[2] / 2;
  const xMin = 0.15, xMax = d - 0.15, width = 0.9;
  tableTop.position.set((xMin + xMax) / 2, boxU.center[1], surface - 0.015);
  tableTop.scale.set(Math.max(0.01, xMax - xMin), width, 0.03);
  const legHeight = Math.max(0, surface - 0.03 - floor);
  tableLegs.forEach((mesh, i) => {
    mesh.position.set(i < 2 ? xMin + 0.025 : xMax - 0.025,
      boxU.center[1] + (i % 2 ? 1 : -1) * (width / 2 - 0.025), floor + legHeight / 2);
    mesh.scale.set(0.035, 0.035, legHeight);
    mesh.visible = $('c_table').checked && legHeight > 0;
  });
  tableTop.visible = $('c_table').checked;
  const hand = [boxU.center[0], boxU.center[1], surface + 0.045];
  const elbow = [hand[0] * 0.52, -0.216 + (hand[1] + 0.216) * 0.5, (0.42 + hand[2]) / 2 + 0.025];
  placeLimb(rightUpper, [0, -0.216, 0.42], elbow);
  placeLimb(rightForearm, elbow, hand); rightHand.position.set(...hand);
  if ($('c_collab').checked) {
    reg.center = [d - boxU.center[0], y0 - boxU.center[1], boxU.center[2] - h].map(v => +v.toFixed(6));
    reg.dims.box = boxU.size.slice(); reg.rpy = [0, 0, 0]; syncSliders();
  }
  $('bestmsg').textContent = ''; window.bestPlacement = null;
}
for (const [key, min, max] of [['d', 0.5, 1.2], ['y0', -0.5, 0.5], ['h', 0, 0.6]]) {
  const row = sliderRow(key, () => placement[key], v => { placement[key] = Number.isFinite(v) ? Math.min(max, Math.max(min, v)) : placement[key]; updatePlacement(); }, min, max, 0.01);
  $('placement').appendChild(row); placementRows.push(row); row.sync();
}
for (const [field, label, min, max] of [['center', 'centre', -0.8, 1.2], ['size', 'size', 0.02, 1.2]]) {
  ['x', 'y', 'z'].forEach((axis, k) => {
    const row = sliderRow(`${label} ${axis}${field === 'center' ? '_U' : ''}`, () => boxU[field][k], v => {
      if (Number.isFinite(v)) boxU[field][k] = Math.min(max, Math.max(min, v));
      updatePlacement();
    }, min, max, 0.01);
    $('collab_box').appendChild(row); boxRows.push(row); row.sync();
  });
}
$('standing_box').onclick = () => {
  boxU.center = [0.35, -0.20, 0.55]; boxU.size = [0.30, 0.60, 0.40];
  boxRows.forEach(row => row.sync()); updatePlacement(); updateRegion();
};
$('c_table').onchange = () => { updatePlacement(); updateRegion(); };
function enableCollaboratorControls(on) {
  document.querySelectorAll('#collab_box input, #standing_box, #c_table, #best_placement').forEach(el => el.disabled = !on);
}
enableCollaboratorControls(false);
$('c_collab').onchange = () => {
  const on = $('c_collab').checked; enableCollaboratorControls(on); user.visible = on; grid.position.z = on ? -0.779 : 0;
  if (on) {
    $('rtype').value = 'box'; $('rtype').onchange({ target: $('rtype') });
    $('c_region').checked = $('c_hl').checked = true;
    camera.position.set(1.7, 1.0, 1.8); orbit.target.set(0.35, -0.12, 0.05); orbit.update();
  }
  document.querySelectorAll('#sliders input, #rtype, #gizmo, #c_region, #c_hl').forEach(el => el.disabled = on);
  updatePlacement(); $('c_region').onchange();
};
updatePlacement();

const insideIdx = {};
function updateRegion() {
  regMesh.position.set(...reg.center);
  regMesh.quaternion.setFromEuler(new THREE.Euler(reg.rpy[0] * DEG, reg.rpy[1] * DEG, reg.rpy[2] * DEG, 'ZYX')); // URDF rpy convention
  const d = reg.dims[reg.type];
  regMesh.scale.set(...(reg.type === 'sphere' ? [d[0], d[0], d[0]] : d));
  const v = vkey(), on = $('c_region').checked, hl = on && $('c_hl').checked;
  const inv = new THREE.Matrix4().compose(regMesh.position, regMesh.quaternion, new THREE.Vector3(1, 1, 1)).invert();
  const p = new THREE.Vector3(); const counts = {};
  for (const n of ['left', 'right', 'both']) {
    const g = clouds[v][n].geometry, a = g.attributes.position, c = g.attributes.color; insideIdx[n] = [];
    for (let i = 0; i < a.count; i++) {
      let inside = false;
      if (on) {
        p.fromBufferAttribute(a, i).applyMatrix4(inv);
        if (reg.type === 'box') inside = Math.abs(p.x) <= d[0] / 2 + 1e-7 && Math.abs(p.y) <= d[1] / 2 + 1e-7 && Math.abs(p.z) <= d[2] / 2 + 1e-7;
        else if (reg.type === 'sphere') inside = p.length() <= d[0];
        else inside = (p.x / d[0]) ** 2 + (p.y / d[1]) ** 2 + (p.z / d[2]) ** 2 <= 1;
      }
      if (inside) insideIdx[n].push(i);
      (hl ? (inside ? HL : COLORS[n].clone().multiplyScalar(0.3)) : COLORS[n]).toArray(c.array, i * 3);
    }
    c.needsUpdate = true; counts[n] = insideIdx[n].length;
  }
  const vol = reg.type === 'box' ? d[0] * d[1] * d[2] : reg.type === 'sphere' ? 4 / 3 * Math.PI * d[0] ** 3 : 4 / 3 * Math.PI * d[0] * d[1] * d[2];
  // Axis-aligned boxes: count lattice samples, including boundary points (not volume / cell volume).
  const exactGrid = reg.type === 'box' && reg.rpy.every(v => v === 0);
  const nGrid = exactGrid ? d.reduce((n, size, k) => n * Math.max(0,
    Math.floor((reg.center[k] + size / 2 + 1e-7) / meta.step) - Math.ceil((reg.center[k] - size / 2 - 1e-7) / meta.step) + 1), 1) : vol / meta.step ** 3;
  counts.union = counts.left + counts.right - counts.both;
  window.regionCoverage = { grid_samples: nGrid, denominator: exactGrid ? 'lattice points inside, boundaries included' : 'approximate volume / grid cell volume',
    counts, fractions: Object.fromEntries(Object.entries(counts).map(([n, c]) => [n, nGrid ? c / nGrid : 0])) };
  $('regmsg').innerHTML = on ? `region volume ${(vol * 1000).toFixed(1)} L; ${exactGrid ? '' : '≈ '}${nGrid.toFixed(0)} grid samples\n` +
    ['left', 'right', 'both', 'union'].map(n => `${n}: ${counts[n]} reachable (${(100 * window.regionCoverage.fractions[n]).toFixed(0)}% of region)`).join('\n') : '';
  window.regionCounts = counts;
}
const placementSearches = {};
function bestPlacement() {
  const started = performance.now(), v = vkey();
  if (!placementSearches[v]) placementSearches[v] = createPlacementSearch(
    clouds[v].left.userData.indices, clouds[v].right.userData.indices, meta.step);
  const result = placementSearches[v].findBest(boxU);
  if (!result) {
    window.bestPlacement = null;
    $('bestmsg').textContent = 'No feasible placement: box far face + 0.15 m exceeds d = 1.20 m.';
    return null;
  }
  Object.assign(placement, result.placement);
  placementRows.forEach(row => row.sync()); updatePlacement(); updateRegion();
  result.elapsed_ms = performance.now() - started; result.variant = v;
  window.bestPlacement = result;
  $('bestmsg').textContent = `Best d=${placement.d.toFixed(2)}, y0=${placement.y0.toFixed(2)}, h=${placement.h.toFixed(2)} m\n` +
    `union ${(100 * result.coverage.fractions.union).toFixed(2)}%; both ${(100 * result.coverage.fractions.both).toFixed(2)}%\n` +
    `${result.candidates} placements; ${result.elapsed_ms.toFixed(0)} ms`;
  return result;
}
$('best_placement').onclick = bestPlacement;
['torso', 'variant'].forEach(id => $(id).addEventListener('input', () => {
  $('bestmsg').textContent = ''; window.bestPlacement = null;
}));

$('dl').onclick = () => {
  const v = vkey(), q = regMesh.quaternion;
  const out = {
    type: reg.type, frame: 'root_link', units: 'm', center: reg.center,
    dims: reg.type === 'box' ? { size_xyz: reg.dims.box } : reg.type === 'sphere' ? { radius: reg.dims.sphere[0] } : { semi_axes_abc: reg.dims.ellipsoid },
    orientation: { rpy_deg: reg.rpy, rpy_convention: 'URDF (R = Rz(yaw) Ry(pitch) Rx(roll)), fixed axes of root_link', quaternion_xyzw: [q.x, q.y, q.z, q.w] },
    reachability_variant: (v.endsWith('free') ? 'position only, orientation free' : 'palm down, fingers forward (±10 deg)') +
      (v[0] === 't' ? '; torso joints used (roll ±23, pitch -18..45, yaw ±43 deg)' : '; torso locked at 0'),
    reachable_counts: window.regionCounts,
    notes: 'ergoCub, fixed root (root_link); end-effector = l/r_hand_palm origin; grid 2 cm; reachable if IK position error < 1 cm',
  };
  if ($('c_collab').checked) out.collaborator = {
    frame_U: { origin: 'seated user pelvis', axes: { x: 'user gaze', y: 'user left', z: 'up' }, units: 'm' },
    box_U: { center: boxU.center.slice(), size_xyz: boxU.size.slice(), rpy_deg: [0, 0, 0] },
    table: { visible: $('c_table').checked, surface_z_U: boxU.center[2] - boxU.size[2] / 2, height_above_floor: boxU.center[2] - boxU.size[2] / 2 - placement.h + 0.779 },
    box_root_link: { center: reg.center, size_xyz: reg.dims.box, rpy_deg: reg.rpy },
    placement: { root_link_in_U: { position: [placement.d, placement.y0, placement.h], yaw_deg: 180 },
      user_pelvis_in_root_link: { position: [placement.d, placement.y0, -placement.h], yaw_deg: 180 },
      pelvis_height_above_floor: 0.779 - placement.h, floor_z_root_link: -0.779 },
    coverage: window.regionCoverage,
    notes: 'No collision checking; target = palm frame at wrist, fingers approximately 10 cm further',
  };
  if ($('c_dlpts').checked) {
    out.reachable_points = {};
    for (const n of ['left', 'right', 'both']) {
      const a = clouds[v][n].geometry.attributes.position;
      out.reachable_points[n] = insideIdx[n].map(i => [a.getX(i), a.getY(i), a.getZ(i)].map(x => +x.toFixed(3)));
    }
  }
  const url = URL.createObjectURL(new Blob([JSON.stringify(out, null, 1)], { type: 'application/json' }));
  window.lastDownload = out;
  Object.assign(document.createElement('a'), { href: url, download: `ergocub_region_${reg.type}.json` }).click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
};

buildSliders(); refreshClouds();
window.app = { goTo, fk, chains, reg, updateRegion, clouds, placement, user, boxU, updatePlacement, bestPlacement };
renderer.setAnimationLoop(() => renderer.render(scene, camera));

// ---------- self-test / screenshot presets: index.html?test , ?region ----------
const qs = new URLSearchParams(location.search);
if (qs.has('collab')) { $('c_collab').checked = true; $('c_collab').onchange(); }
if (qs.has('collab') && qs.has('best')) bestPlacement();
if (qs.has('test') || qs.has('region') || qs.has('down') || qs.has('locked') || qs.has('ik')) {
  while (!robot) await new Promise(r => setTimeout(r, 100));
  if (qs.has('down')) { $('variant').value = 'down'; refreshClouds(); }
  if (qs.has('locked')) { $('torso').value = 'locked'; refreshClouds(); }
  if (qs.has('region')) { $('c_region').checked = true; $('c_region').onchange(); $('c_both').checked = true; refreshClouds(); }
  const out = [];
  for (const [side, t] of [['l', [0.25, 0.25, 0.35]], ['r', [0.2, -0.3, 0.2]], ['l', [0.9, 0.0, 0.3]]]) {
    $('arm').value = side; goTo(t);
    const w = new THREE.Vector3(); robot.links[side + '_hand_palm'].getWorldPosition(w); world.worldToLocal(w);
    const p = fk(chains[tk() + side], window.lastIK.q).p;
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
