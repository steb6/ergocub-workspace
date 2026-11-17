// Browser approximation of Semeraro et al., arXiv:2610.19896v2.
// No DOM/three dependency (also used by the headless convergence test page).
export const RAD = Math.PI / 180;
export const HUMAN = {
  names: ['Sx', 'Sy', 'Sz', 'Ey', 'Ez', 'Wx', 'Wy'],
  shoulder: [0, -0.216, 0.42], lengths: [0.30, 0.26, 0.07], palm: 0.10,
  // lengths[2] = grip: wrist -> fist centre along the hand axis (where the grasped cylinder B lies). The task frame
  // is the wrist frame translated there (set 0 for the bare anatomical wrist).
  // Numerical anthropometry and reference posture are assumptions, not values
  // reported in the paper. Rotations are intrinsic X,-Y,Z,-Y,Z,X,-Y.
  lower: [-45, -40, -90, 0, -90, -80, -30].map(v => v * RAD),
  upper: [150, 180, 90, 150, 90, 70, 40].map(v => v * RAD),
  nominal: [0, 90, 0, 30, 0, 0, 0].map(v => v * RAD),
  zeta: 0.08727, gamma: 0.05, taskWeight: 2500, rotationScale: 0.15,
};
export const PRESETS = { healthy: [0, 0], shoulder: [1, 0], elbow: [0, 1], shoulderHalf: [0.5, 0], elbowHalf: [0, 0.5] };
export function seededRandom(seed = 2026) {
  return () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t ^= t + Math.imul(t ^ t >>> 7, 61 | t); return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export function bounds(alpha, model = HUMAN) {
  const a = [0, alpha[0], 0, alpha[1], 0, 0, 0], qm = model.nominal;
  return {
    lower: model.lower.map((v, i) => v + a[i] * (Math.min(model.nominal[i] - model.zeta, qm[i]) - v)),
    upper: model.upper.map((v, i) => v - a[i] * (v - Math.max(model.nominal[i] + model.zeta, qm[i]))),
  };
}
function mul(A, B) {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) C[r*3+c] = A[r*3]*B[c]+A[r*3+1]*B[3+c]+A[r*3+2]*B[6+c];
  return C;
}
const mv = (R, v) => [R[0]*v[0]+R[1]*v[1]+R[2]*v[2], R[3]*v[0]+R[4]*v[1]+R[5]*v[2], R[6]*v[0]+R[7]*v[1]+R[8]*v[2]];
const axes = [[1,0,0],[0,-1,0],[0,0,1],[0,-1,0],[0,0,1],[1,0,0],[0,-1,0]];
function rotation(i, q) {
  const [x,y,z] = axes[i], c = Math.cos(q), s = Math.sin(q), t = 1-c;
  return [t*x*x+c,t*x*y-s*z,t*x*z+s*y,t*x*y+s*z,t*y*y+c,t*y*z-s*x,t*x*z-s*y,t*y*z+s*x,t*z*z+c];
}
export function humanFK(q, model = HUMAN, jacobian = false) {
  let R = [1,0,0,0,1,0,0,0,1], p = model.shoulder.slice();
  const origins = [], directions = [], points = [p.slice()];
  for (let i = 0; i < 7; i++) {
    if (jacobian) { origins.push(p.slice()); directions.push(mv(R, axes[i])); }
    R = mul(R, rotation(i, q[i]));
    const length = i === 2 ? model.lengths[0] : i === 4 ? model.lengths[1] : i === 6 ? model.lengths[2] : 0;
    if (length) { const d = mv(R, [0,0,-length]); p = p.map((v,k) => v+d[k]); points.push(p.slice()); }
  }
  // Calibrated wrist axes: x=-anatomical y, y=anatomical x, z=anatomical z.
  // z points back along the hand/forearm; y is the through-fist cylinder axis.
  R = mul(R, [0,1,0,-1,0,0,0,0,1]);
  if (points.length === 3) points.push(p.slice());
  const dir = mv(R, [0,0,-1]), knuckle = p.map((v,k) => v + model.palm*dir[k]);
  const J = jacobian ? directions.map((a,i) => { const d = p.map((v,k) => v-origins[i][k]); return [a[1]*d[2]-a[2]*d[1],a[2]*d[0]-a[0]*d[2],a[0]*d[1]-a[1]*d[0]]; }) : null;
  return { p, points, R, J, angularJ: directions, dir, knuckle };
}

function solve(A,b) {
  const n=b.length, M=A.map((r,i)=>[...r,b[i]]);
  for(let c=0;c<n;c++) {
    let k=c; for(let r=c+1;r<n;r++) if(Math.abs(M[r][c])>Math.abs(M[k][c])) k=r;
    [M[c],M[k]]=[M[k],M[c]];
    for(let r=c+1;r<n;r++) {const f=M[r][c]/M[c][c];for(let j=c;j<=n;j++)M[r][j]-=f*M[c][j];}
  }
  const x=Array(n).fill(0);for(let r=n-1;r>=0;r--){let s=M[r][n];for(let j=r+1;j<n;j++)s-=M[r][j]*x[j];x[r]=s/M[r][r];}return x;
}
// Eq. (3) approximated by bounded, damped Gauss–Newton with a stiff task
// penalty. It returns a best-effort posture even for unreachable positions.
// Full grip-frame pose (target {p, R} or position array). Matrix residual avoids the false zero at a 180-degree error.
export function humanIK(target, alpha, model = HUMAN) {
  const tp=target.p || target, tr=target.R || null, wr=(model.rotationScale ?? .15)/Math.sqrt(2);
  const lim=bounds(alpha,model), a=[0,alpha[0],0,alpha[1],0,0,0];
  const w=a.map(v=>v*v+model.gamma*(1-v)*(1-v));
  const residual=f=>tp.map((v,k)=>v-f.p[k]).concat(tr?tr.map((v,k)=>wr*(v-f.R[k])):[]);
  const cost=q=>{const e=residual(humanFK(q,model));return model.taskWeight*e.reduce((s,v)=>s+v*v,0)+q.reduce((s,v,k)=>s+w[k]*(v-model.nominal[k])**2,0);};
  let best=null;
  for(const start of [model.nominal, [0,55*RAD,0,70*RAD,0,0,0], [0,90*RAD,0,0,0,0,0], [60*RAD,30*RAD,-60*RAD,100*RAD,0,0,0]]) {
    let q=start.map((v,k)=>clamp(v,lim.lower[k],lim.upper[k])), value=cost(q);
    for(let it=0;it<55;it++) {
      const f=humanFK(q,model,true), e=residual(f), J=f.J.map((j,i)=>{
        if(!tr)return j;
        const [x,y,z]=f.angularJ[i], R=f.R;
        return j.concat([0,1,2].flatMap(r=>[0,1,2].map(c=>wr*(r===0?y*R[6+c]-z*R[3+c]:r===1?z*R[c]-x*R[6+c]:x*R[3+c]-y*R[c]))));
      });
      const A=J.map((u,i)=>J.map((v,j)=>model.taskWeight*u.reduce((s,x,k)=>s+x*v[k],0)+(i===j?w[i]+0.2:0)));
      const b=J.map((u,i)=>model.taskWeight*u.reduce((s,x,k)=>s+x*e[k],0)-w[i]*(q[i]-model.nominal[i]));
      const dq=solve(A,b).map(v=>clamp(v,-0.3,0.3));
      let next=q, nv=value;
      for(const step of [1,0.5,0.25,0.1]) {
        const trial=q.map((v,k)=>clamp(v+step*dq[k],lim.lower[k],lim.upper[k])), cv=cost(trial);
        if(cv<nv){next=trial;nv=cv;break;}
      }
      const gain=value-nv; q=next;value=nv;
      if(gain<1e-8)break;
    }
    if(!best||value<best.cost) best={q,cost:value};
  }
  const f=humanFK(best.q,model);
  const angle=tr?Math.acos(clamp((tr.reduce((s,v,k)=>s+v*f.R[k],0)-1)/2,-1,1)):0;
  return {...best,...f,error:Math.hypot(...f.p.map((v,k)=>v-tp[k])),orientationError:angle};
}
// Paper defaults (Sec. II-C): 50 particles, sigma_q^2 = 0.2 rad^2, resample when
// Neff/Np < 0.4 with Gaussian jitter of variance 0.05, stop when both marginal
// variances < 0.003 for 3 consecutive updates or after 25 handovers.
export const PAPER = { particles: 50, likelihoodVariance: 0.2, essRatio: 0.4, jitterVariance: 0.05,
  varianceThreshold: 0.003, consecutive: 3, maxHandovers: 25, epsilon: 1e-12 };
export function gaussian(rng) { return Math.sqrt(-2 * Math.log(Math.max(1e-12, rng()))) * Math.cos(2 * Math.PI * rng()); }
export function summarize(particles, weights) {
  const mean=[0,0],variance=[0,0];
  particles.forEach((a,i)=>a.forEach((v,k)=>mean[k]+=weights[i]*v));
  particles.forEach((a,i)=>a.forEach((v,k)=>variance[k]+=weights[i]*(v-mean[k])**2));
  return {mean,variance,std:variance.map(Math.sqrt),ess:1/weights.reduce((s,v)=>s+v*v,0)};
}
export class ParticleFilter {
  constructor(options = {}, rng = Math.random) {
    Object.assign(this, PAPER, options); this.rng = rng;
    // Initial particles uniform in [0,1]^2 (paper: equal weights, mean ~0.5).
    this.particles = Array.from({ length: this.particles }, () => [rng(), rng()]);
    this.weights = this.particles.map(() => 1 / this.particles.length);
    this.count = 0; this.stable = 0; this.resamplings = 0; this.done = false; this.reason = '';
    this.history = [this.summary()];
  }
  summary() { return summarize(this.particles, this.weights); }
  // Eq. (6): human-model IK of every particle at the handover pose; keep (Sy, Ey).
  predict(pose) { return this.particles.map(a => { const q = humanIK(pose, a).q; return [q[1], q[3]]; }); }
  // Eqs. (5), (7), (8), (9).
  update(predictions, observation) {
    const v = this.likelihoodVariance;
    const logs = this.weights.map((w, i) => Math.log(w + this.epsilon) -
      predictions[i].reduce((s, p, k) => s + (p - observation[k]) ** 2, 0) / (2 * v));
    const peak = Math.max(...logs), sum = logs.reduce((s, l) => s + Math.exp(l - peak), 0);
    this.weights = logs.map(l => Math.exp(l - peak) / sum);
    const state = this.summary(); this.count++;
    this.stable = state.variance.every(x => x < this.varianceThreshold) ? this.stable + 1 : 0;
    this.done = this.stable >= this.consecutive || this.count >= this.maxHandovers;
    this.reason = this.stable >= this.consecutive ? `variance < ${this.varianceThreshold} for ${this.consecutive} updates` : this.done ? `${this.maxHandovers} handovers` : '';
    state.resampled = false;
    if (!this.done && state.ess / this.particles.length < this.essRatio) { this.resample(); state.resampled = true; }
    state.resamplings = this.resamplings; this.history.push(state); return state;
  }
  resample() { // multinomial with replacement + jitter, clipped to the IC range [0,1]
    const cdf = []; this.weights.reduce((s, w, i) => (cdf[i] = s + w), 0);
    const s = Math.sqrt(this.jitterVariance), clip = x => Math.max(0, Math.min(1, x));
    this.particles = this.particles.map(() => {
      const u = this.rng() * cdf[cdf.length - 1]; let i = cdf.findIndex(c => c >= u); if (i < 0) i = cdf.length - 1;
      return this.particles[i].map(x => clip(x + s * gaussian(this.rng)));
    });
    this.weights = this.particles.map(() => 1 / this.particles.length); this.resamplings++;
  }
  // Extension (not in the paper): Gaussian approximation of the expected
  // information gain I(alpha; q) = 0.5 ln det(I + Cov_pred / sigma_q^2).
  information(predictions) {
    const m=[0,0];predictions.forEach((q,i)=>q.forEach((x,k)=>m[k]+=this.weights[i]*x));
    let a=0,b=0,c=0;predictions.forEach((q,i)=>{const x=q[0]-m[0],y=q[1]-m[1],w=this.weights[i];a+=w*x*x;b+=w*x*y;c+=w*y*y;});
    const v=this.likelihoodVariance;return .5*Math.log(Math.max(1,(1+a/v)*(1+c/v)-(b/v)**2));
  }
}
// Simulated skeleton tracking: true-IC IK + Gaussian joint noise (noise std is an assumption).
export function observe(q, std, rng) { return [q[1] + std * gaussian(rng), q[3] + std * gaussian(rng)]; }
// Uniform random handover position inside the box (paper: random poses in the action space).
export function randomInBox(box, rng) { return box.center.map((c, k) => c + (rng() - 0.5) * box.size[k]); }
