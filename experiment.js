import * as THREE from 'three';
import { HUMAN, RAD, PRESETS, PAPER, ParticleFilter, bounds, humanFK, humanIK, observe, seededRandom } from './experiment-core.js?v=5';
const $ = id => document.getElementById(id);
const clone = x => JSON.parse(JSON.stringify(x));
const ACTIVE_CANDIDATES = 8;

export function createExperiment(api) {
  const panel = document.createElement('section'); panel.id = 'experiment';
  panel.innerHTML = `<h2>Handover experiment (paper pipeline)</h2>
  <small>Simulated right upper limb (7 DOFs); estimates the impairment coefficients α of shoulder flexion Sy and elbow flexion Ey. Needs the collaborator setup.</small>
  <fieldset id="exp-settings" style="border:0;padding:0;margin:6px 0">
    <div class="row"><label class="k">truth</label><select id="exp-preset"><option value="healthy">healthy</option><option value="shoulder">shoulder flexion impaired</option><option value="shoulderHalf">shoulder half impaired</option><option value="elbow">elbow flexion impaired</option><option value="elbowHalf">elbow half impaired</option><option value="custom">custom</option></select></div>
    <div class="row"><label class="k">α* Sy</label><input id="exp-s" type="range" min="0" max="1" step="0.05" value="0"><output id="exp-s-value">0.00</output></div>
    <div class="row"><label class="k">α* Ey</label><input id="exp-e" type="range" min="0" max="1" step="0.05" value="0"><output id="exp-e-value">0.00</output></div>
    <div class="row"><label class="k">poses</label><select id="exp-policy"><option value="random">random in box (paper)</option><option value="active">active: max info gain (extension)</option></select></div>
    <div class="row"><label class="k">track σ</label><input id="exp-noise" type="number" min="0" max="0.5" step="0.01" value="0.03"><small>rad, skeleton noise</small></div>
    <div class="row"><label class="k">max N</label><input id="exp-max" type="number" min="1" max="100" step="1" value="25"><label>seed</label><input id="exp-seed" type="number" min="0" max="999999" step="1" value="2026"></div>
  </fieldset>
  <div class="row"><button id="exp-start">Start</button><button id="exp-step">Step</button><button id="exp-reset">Reset</button></div>
  <div class="row"><label class="k">speed</label><input id="exp-speed" type="range" min="1" max="10" value="3"><output id="exp-speed-value">3×</output></div>
  <div id="exp-status" style="white-space:pre-line;margin:5px 0;padding:4px;background:#1d2026">Ready.</div>
  <canvas id="exp-coefficients" width="600" height="330" style="width:100%;height:165px"></canvas>
  <canvas id="exp-error" width="600" height="180" style="width:100%;height:90px"></canvas>
  <div id="exp-result"></div><button id="exp-download" disabled>Download run JSON</button>
  <pre id="exp-summary" style="display:none"></pre>
  <details style="margin-top:6px"><summary>Model &amp; assumptions</summary><small>
  As the paper: bounds of Eq. (2) with q<sub>m</sub>=q<sub>n</sub>, ζ=5°; human IK of Eq. (3) (γ=0.05, best effort if unreachable); particle filter with 50 particles (uniform init), σ<sub>q</sub>²=0.2 rad², resampling when N<sub>eff</sub>/N&lt;0.4 with jitter σ²=0.05 (clipped to [0,1]); stop when both variances &lt;0.003 for 3 handovers or after 25. Handover poses random in the box (restricted to points ergoCub reaches).<br>
  Assumed (not in the paper): segment lengths 0.30/0.26/0.09 m, healthy limits, q<sub>n</sub> (upper arm horizontal, elbow 30°), position-only handover, numerical IK (penalty + bounded Gauss-Newton, 4 starts), tracking noise σ. The simulated user uses the same IK with the true α, as in the paper's simulation.<br>
  Extension: "active" scores ${ACTIVE_CANDIDATES} random reachable poses by a Gaussian approximation of the expected information gain, ½ ln det(I + Cov<sub>pred</sub>/σ<sub>q</sub>²), and takes the best.</small></details>`;
  $('bestmsg').parentElement.insertBefore(panel, $('bestmsg').nextElementSibling.nextElementSibling);
  const markers = new THREE.Group(); api.user.add(markers);
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0x71d8b3 });
  const joints = Array.from({ length: 4 }, () => { const m = new THREE.Mesh(new THREE.SphereGeometry(.021, 12, 8), jointMaterial); api.user.add(m); m.visible = false; return m; });
  const object = new THREE.Mesh(new THREE.BoxGeometry(.045, .045, .065), new THREE.MeshStandardMaterial({ color: 0xffcf66, metalness: .15, roughness: .4 }));
  object.visible = false; api.world.add(object);
  let token = 0, running = false, busy = false, timer = null, pool = null, pf = null, records = [], settings = null, rng = null, truth = null, fast = false;
  const REST = [0, 10 * RAD, 0, 20 * RAD, 0, 0, 0];
  let humanQ = REST.slice();
  const locked = [];
  function lockSettings(on) {
    $('exp-settings').disabled = on;
    if (on && !locked.length) document.querySelectorAll('#placement input,#collab_box input,#standing_box,#best_placement,#torso,#c_collab').forEach(el => { locked.push([el, el.disabled]); el.disabled = true; });
    if (!on) { locked.forEach(([el, disabled]) => el.disabled = disabled); locked.length = 0; }
  }
  function controls() {
    const enabled = api.enabled(); panel.style.opacity = enabled ? '1' : '.5';
    $('exp-start').disabled = !enabled || !!pf?.done;
    $('exp-step').disabled = !enabled || busy || running || !!pf?.done;
    $('exp-start').textContent = running ? 'Pause' : 'Start';
    $('exp-download').disabled = !records.length;
  }
  function drawHuman(q) {
    const f = humanFK(q); api.drawHuman(f.points); humanQ = q.slice();
    f.points.forEach((p, i) => { joints[i].position.set(...p); joints[i].visible = api.enabled(); });
  }
  function clearMarkers() { for (const child of [...markers.children]) { markers.remove(child); child.geometry.dispose(); child.material.dispose(); } }
  function reset() {
    token++; running = false; busy = false; clearTimeout(timer); timer = null; pool = null; pf = null; records = []; settings = null; truth = null;
    object.visible = false; clearMarkers(); lockSettings(false); joints.forEach(j => j.visible = false); humanQ = REST.slice(); if (api.enabled()) api.resetHuman();
    $('exp-status').textContent = 'Ready. Start runs handovers until the stopping rule; Step runs one.';
    $('exp-result').textContent = ''; $('exp-summary').textContent = ''; plot(); controls();
  }
  function settingsFromUI() {
    const num = (id, lo, hi, fallback) => { const v = Number($(id).value); return Number.isFinite(v) ? Math.max(lo, Math.min(hi, v)) : fallback; };
    return { preset: $('exp-preset').value, groundTruth: [num('exp-s', 0, 1, 0), num('exp-e', 0, 1, 0)], policy: $('exp-policy').value,
      activeCandidates: ACTIVE_CANDIDATES, trackingStd: num('exp-noise', 0, .5, .03), filter: { ...PAPER, maxHandovers: Math.round(num('exp-max', 1, 100, 25)) },
      seed: Math.round(num('exp-seed', 0, 999999, 2026)), box: clone(api.box()), placement: clone(api.placement()), torso: api.torso(), model: clone(HUMAN),
      task: 'position only; handover point = ergoCub palm target = human hand tip' };
  }
  function init() {
    settings = settingsFromUI(); truth = settings.groundTruth.slice(); rng = seededRandom(settings.seed);
    pf = new ParticleFilter(settings.filter, rng);
    pool = api.candidatePool();
    if (!pool.length) throw Error('No ergoCub-reachable point inside the box: change placement/box/torso, then Reset.');
    lockSettings(true); drawHuman(REST);
  }
  // Draw a random reachable handover point and plan ergoCub's arm; unplannable points are dropped.
  function drawPose() {
    while (pool.length) {
      const i = Math.floor(rng() * pool.length), c = pool[i], plan = c.plan === undefined ? (c.plan = api.plan(c)) : c.plan;
      if (plan) return c;
      pool.splice(i, 1);
    }
    throw Error('ergoCub IK failed on every point of the box.');
  }
  function choose() {
    if (settings.policy !== 'active') { const c = drawPose(); return { c, predictions: pf.predict(c.pose), score: null }; }
    let best = null;
    for (let k = 0; k < settings.activeCandidates; k++) {
      const c = drawPose(), predictions = pf.predict(c.pose), score = pf.information(predictions);
      if (!best || score > best.score) best = { c, predictions, score };
    }
    return best;
  }
  function animate(plan, q, myToken) {
    if (fast) { api.apply(plan, plan.q); object.position.set(...api.palmPosition(plan)); object.visible = true; drawHuman(q); return Promise.resolve(true); }
    const duration = 1600 / Number($('exp-speed').value), robotFrom = api.current(plan), humanFrom = humanQ.slice(), start = performance.now();
    const ease = t => { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); };
    return new Promise(resolve => {
      function frame() {
        const now = performance.now();
        if (myToken !== token) { resolve(false); return; }
        const t = (now - start) / duration, a = ease(t * 2), b = ease(t * 2 - 1);  // robot first, then the human reaches
        api.apply(plan, robotFrom.map((v, i) => v + a * (plan.q[i] - v)));
        object.position.set(...api.palmPosition(plan)); object.visible = true;
        drawHuman(humanFrom.map((v, i) => v + b * (q[i] - v)));
        if (t < 1) setTimeout(frame, 16); else resolve(true);  // timers, not rAF: also advances in headless tests
      }
      frame();
    });
  }
  async function step() {
    if (busy || pf?.done || !api.enabled()) return;
    const myToken = token; busy = true; controls();
    try {
      if (!pf) init();
      const t0 = performance.now(), { c, predictions, score } = choose();
      const reached = humanIK(c.pose, truth);                         // simulated user: only the simulator sees α*
      const observation = observe(reached.q, settings.trackingStd, rng);
      const posterior = pf.update(predictions, observation), computationMs = performance.now() - t0;
      const index = records.length + 1, err = posterior.mean.reduce((s, v, k) => s + Math.abs(v - truth[k]), 0) / 2;
      records.push({ index, pose_U: c.pose, robot: { side: c.plan.side, q: c.plan.q, positionError: c.plan.err, palm_root: c.root }, infoScore: score,
        simulatedQ: reached.q, humanPositionError: reached.error, observation: { joints: ['Sy', 'Ey'], rad: observation },
        predictions,
        posterior: clone(posterior), particles: clone(pf.particles), weights: pf.weights.slice(), meanAbsoluteError: err, computationMs });
      const m = new THREE.Mesh(new THREE.SphereGeometry(.011, 10, 8), new THREE.MeshBasicMaterial({ color: new THREE.Color().setHSL(.66 * (1 - (index - 1) / Math.max(1, settings.filter.maxHandovers - 1)), .9, .55) }));
      m.position.set(...c.pose); markers.add(m);
      if (!await animate(c.plan, reached.q, myToken)) return;
      $('exp-status').textContent = `Handover ${index}/${settings.filter.maxHandovers} · ergoCub ${c.plan.side === 'l' ? 'left' : 'right'} arm · ${settings.policy}\n` +
        `α̂ = [${posterior.mean.map(v => v.toFixed(2))}] · MAE ${err.toFixed(3)} · user miss ${(reached.error * 100).toFixed(1)} cm\n` +
        `compute ${computationMs.toFixed(0)} ms · resamplings ${pf.resamplings} · ${pf.done ? `stopped: ${pf.reason}` : `variance streak ${pf.stable}/3`}`;
      plot(); renderResult();
      if (pf.done) running = false;
    } catch (error) { running = false; $('exp-status').textContent = error.message; console.error(error); }
    finally {
      if (myToken === token) {
        busy = false; controls();
        if (running) timer = setTimeout(step, fast ? 0 : 400 / Number($('exp-speed').value));
      }
    }
  }
  function plot() {
    const history = pf?.history || [], t = truth || [Number($('exp-s').value), Number($('exp-e').value)], n = settings?.filter.maxHandovers || 25;
    const c = $('exp-coefficients'), g = c.getContext('2d'); g.fillStyle = '#1d2026'; g.fillRect(0, 0, c.width, c.height);
    for (let k = 0; k < 2; k++) {
      const top = 24 + k * 160, left = 55, width = 530, height = 112, x = i => left + width * i / n, y = v => top + height * (1 - v), col = k ? '#ffb65d' : '#79baff';
      g.font = '20px system-ui'; g.fillStyle = col; g.fillText(`${k ? 'Elbow Ey' : 'Shoulder Sy'}: α̂ mean ± σ, dashed = true α*`, left, top - 5);
      g.strokeStyle = '#48505d'; g.lineWidth = 1;
      for (const v of [0, .5, 1]) { g.beginPath(); g.moveTo(left, y(v)); g.lineTo(left + width, y(v)); g.stroke(); g.fillStyle = '#aab1bb'; g.font = '16px system-ui'; g.fillText(v.toFixed(1), 10, y(v) + 5); }
      g.strokeStyle = '#e4dfcf'; g.setLineDash([7, 6]); g.beginPath(); g.moveTo(left, y(t[k])); g.lineTo(left + width, y(t[k])); g.stroke(); g.setLineDash([]);
      if (history.length) {
        g.fillStyle = col + '40'; g.beginPath();
        history.forEach((s, i) => { const yy = y(Math.min(1, s.mean[k] + s.std[k])); i ? g.lineTo(x(i), yy) : g.moveTo(x(i), yy); });
        for (let i = history.length - 1; i >= 0; i--) g.lineTo(x(i), y(Math.max(0, history[i].mean[k] - history[i].std[k])));
        g.closePath(); g.fill();
        g.strokeStyle = col; g.lineWidth = 3; g.beginPath(); history.forEach((s, i) => i ? g.lineTo(x(i), y(s.mean[k])) : g.moveTo(x(i), y(s.mean[k]))); g.stroke();
        g.fillStyle = '#ff6b6b'; history.forEach((s, i) => { if (s.resampled) g.fillRect(x(i) - 2, top + height - 8, 4, 8); });
      }
      g.fillStyle = '#aaa'; g.font = '15px system-ui'; g.fillText('0', left, top + height + 18); g.fillText(`handover (red tick = resampling)`, left + 160, top + height + 18); g.fillText(String(n), left + width - 18, top + height + 18);
    }
    const e = $('exp-error'), ctx = e.getContext('2d'), yy = v => 140 - 110 * Math.min(1, v / 0.5);
    ctx.fillStyle = '#1d2026'; ctx.fillRect(0, 0, e.width, e.height);
    ctx.fillStyle = '#ddd'; ctx.font = '19px system-ui'; ctx.fillText('Mean absolute IC error |α̂ − α*| (dashed = 0.1)', 55, 22);
    ctx.strokeStyle = '#666'; ctx.setLineDash([5, 5]); ctx.beginPath(); ctx.moveTo(55, yy(.1)); ctx.lineTo(585, yy(.1)); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = '#8bdb98'; ctx.lineWidth = 3; ctx.beginPath();
    history.forEach((s, i) => { const err = s.mean.reduce((sum, v, k) => sum + Math.abs(v - t[k]), 0) / 2, xx = 55 + 530 * i / n; i ? ctx.lineTo(xx, yy(err)) : ctx.moveTo(xx, yy(err)); }); ctx.stroke();
    ctx.fillStyle = '#aaa'; ctx.font = '16px system-ui'; ctx.fillText('0.5', 10, 36); ctx.fillText('0', 20, 145); ctx.fillText('0', 55, 168); ctx.fillText(String(n), 567, 168);
  }
  function renderResult() {
    if (!pf || !truth) return;
    const state = pf.summary(), estimate = bounds(state.mean), actual = bounds(truth), deg = v => (v / RAD).toFixed(0);
    $('exp-result').innerHTML = `<small>${pf.done ? `Final estimate after ${pf.count} handovers (${pf.reason})` : 'Current estimate'}; fROM in degrees</small>
      <table style="width:100%;font-size:11px"><thead><tr><th>DOF</th><th>α̂ ± σ</th><th>α*</th><th>fROM est.</th><th>fROM true</th></tr></thead><tbody>${[1, 3].map((j, k) =>
      `<tr><td>${HUMAN.names[j]}</td><td>${state.mean[k].toFixed(2)} ± ${state.std[k].toFixed(2)}</td><td>${truth[k].toFixed(2)}</td><td>[${deg(estimate.lower[j])}, ${deg(estimate.upper[j])}]</td><td>[${deg(actual.lower[j])}, ${deg(actual.upper[j])}]</td></tr>`).join('')}</tbody></table>`;
    if (pf.done) $('exp-summary').textContent = JSON.stringify({ preset: settings.preset, policy: settings.policy, truth, estimate: state.mean, std: state.std, n: pf.count, resamplings: pf.resamplings, reason: pf.reason, maxComputeMs: Math.max(...records.map(r => r.computationMs)) });
  }
  function exportRun() {
    const final = pf ? pf.summary() : null;
    return { schemaVersion: 2, paper: 'arXiv:2610.19896v2', frame: 'U: user pelvis, x gaze, y left, z up', units: { position: 'm', joints: 'rad' },
      settings: clone(settings), reachablePointsInBox: pool?.length ?? 0, beliefHistory: clone(pf?.history || []), handovers: clone(records),
      finalEstimate: final && { ...final, functionalROM: bounds(final.mean), trueFunctionalROM: bounds(truth), stopped: pf.done, reason: pf.reason || 'in progress', handovers: pf.count, resamplings: pf.resamplings } };
  }
  function configure(p) {  // URL / scripted configuration
    if (p.preset && PRESETS[p.preset]) { $('exp-preset').value = p.preset; $('exp-preset').onchange(); }
    if (p.policy) $('exp-policy').value = p.policy;
    if (p.n) $('exp-max').value = p.n;
    if (p.seed) $('exp-seed').value = p.seed;
    if (p.noise) $('exp-noise').value = p.noise;
    if (p.speed) { $('exp-speed').value = p.speed; $('exp-speed').oninput(); }
    fast = 'fast' in p;
  }
  const syncTruth = () => ['s', 'e'].forEach(k => $(`exp-${k}-value`).textContent = Number($(`exp-${k}`).value).toFixed(2));
  $('exp-preset').onchange = () => { const p = PRESETS[$('exp-preset').value]; if (p) { $('exp-s').value = p[0]; $('exp-e').value = p[1]; syncTruth(); plot(); } };
  ['s', 'e'].forEach(k => $(`exp-${k}`).oninput = () => { syncTruth(); $('exp-preset').value = 'custom'; plot(); });
  $('exp-speed').oninput = () => $('exp-speed-value').textContent = $('exp-speed').value + '×';
  $('exp-start').onclick = () => { running = !running; clearTimeout(timer); controls(); if (running && !busy) step(); };
  $('exp-step').onclick = step; $('exp-reset').onclick = reset;
  $('exp-download').onclick = () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(exportRun(), null, 1)], { type: 'application/json' }));
    Object.assign(document.createElement('a'), { href: url, download: `ergocub-handover-${settings.preset}-${settings.policy}-${settings.seed}.json` }).click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  $('c_collab').addEventListener('change', () => { reset(); markers.visible = api.enabled(); joints.forEach(j => j.visible = api.enabled()); });
  reset();
  return { step, reset, configure, exportRun, start: () => { if (!running && !pf?.done) { running = true; controls(); if (!busy) step(); } },
    get state() { return { running, busy, done: !!pf?.done, count: records.length }; } };
}
