import { Vec3, type Quat } from '../math';
import * as V from '../math/vec3';
import * as Q from '../math/quat';
import * as M3 from '../math/mat3';
import type { RigidBody } from '../dynamics/body';

/** Per-point solver state. Accumulated impulses persist across steps for warm starting. */
export interface SolverPoint {
  /** Feature id from the manifold; used to match impulses between steps. */
  readonly id: number;
  /** Anchors in each body's frame, relative to its center of mass. */
  readonly localA: Vec3;
  readonly localB: Vec3;
  /** World offsets from each center of mass, refreshed by prepareContact. */
  readonly rA: Vec3;
  readonly rB: Vec3;
  /** Separation of a speculative contact (0 for a touching or penetrating one). */
  gap: number;
  /** Target normal relative velocity (restitution), set by prepareContact. */
  restitutionTarget: number;
  normalMass: number;
  tangentMass1: number;
  tangentMass2: number;
  impulseN: number;
  impulseT1: number;
  impulseT2: number;
}

/** Contact between two bodies. `normal` points from A toward B. */
export interface ContactConstraint {
  readonly bodyA: RigidBody;
  readonly bodyB: RigidBody;
  readonly normal: Vec3;
  readonly tangent1: Vec3;
  readonly tangent2: Vec3;
  readonly friction: number;
  readonly restitution: number;
  readonly points: SolverPoint[];
  /** Broadphase proxy ids of the two colliders (A < B). */
  readonly pair: readonly [number, number];
  /** 0 for a convex pair; triangle index + 1 for a contact with a mesh triangle. */
  readonly sub: number;
  /** Normal effective-mass matrix K_ij (coupled across points through rotation). Filled by prepareContact. */
  normalK: Float64Array;
}

/** Orthonormal tangent basis perpendicular to unit vector n. */
export function tangentBasis(n: Vec3): [Vec3, Vec3] {
  const t1 = V.perpendicular(new Vec3(), n);
  const t2 = V.cross(new Vec3(), n, t1);
  return [t1, t2];
}

/** Applies a linear impulse P at world offset r from the center of mass. */
export function applyImpulse(body: RigidBody, P: Vec3, r: Vec3): void {
  applyImpulseXYZ(body, P.x, P.y, P.z, r.x, r.y, r.z);
}

/** Same as `applyImpulse` with scalar arguments (no allocation). */
export function applyImpulseXYZ(
  body: RigidBody,
  px: number,
  py: number,
  pz: number,
  rx: number,
  ry: number,
  rz: number,
): void {
  if (!body.isSimulated) return;
  const im = body.invMass;
  const lv = body.linearVelocity;
  lv.x += px * im;
  lv.y += py * im;
  lv.z += pz * im;
  const tx = ry * pz - rz * py;
  const ty = rz * px - rx * pz;
  const tz = rx * py - ry * px;
  const e = body.worldInvInertia.e;
  const av = body.angularVelocity;
  av.x += (e[0] as number) * tx + (e[1] as number) * ty + (e[2] as number) * tz;
  av.y += (e[3] as number) * tx + (e[4] as number) * ty + (e[5] as number) * tz;
  av.z += (e[6] as number) * tx + (e[7] as number) * ty + (e[8] as number) * tz;
}

/** Effective inverse mass of the pair along unit direction d at offsets rA and rB. */
export function effectiveMass(a: RigidBody, b: RigidBody, rA: Vec3, rB: Vec3, d: Vec3): number {
  let k = a.invMass + b.invMass;
  if (a.isSimulated) {
    const c = V.cross(new Vec3(), rA, d);
    k += V.dot(c, M3.mulVec(new Vec3(), a.worldInvInertia, c));
  }
  if (b.isSimulated) {
    const c = V.cross(new Vec3(), rB, d);
    k += V.dot(c, M3.mulVec(new Vec3(), b.worldInvInertia, c));
  }
  return k;
}

/**
 * Refreshes per-step quantities from the current poses: offsets from the centers of mass, effective
 * masses along normal and tangents, and the restitution target. `preRelativeNormalSpeed` is the
 * normal speed measured at the contact point; restitution is applied only above `threshold`.
 */
export function prepareContact(c: ContactConstraint, threshold: number, dt = 0): void {
  const a = c.bodyA;
  const b = c.bodyB;
  for (const p of c.points) {
    // Impulses act at the midpoint of the two surface anchors, so both bodies receive their
    // offsets from the same world point. Equal and opposite impulses then exert no net torque
    // about the origin even for tangential friction.
    const ra = rotated(a.orientation, p.localA);
    const rb = rotated(b.orientation, p.localB);
    const midx = 0.5 * (a.position.x + ra.x + b.position.x + rb.x);
    const midy = 0.5 * (a.position.y + ra.y + b.position.y + rb.y);
    const midz = 0.5 * (a.position.z + ra.z + b.position.z + rb.z);
    V.set(p.rA, midx - a.position.x, midy - a.position.y, midz - a.position.z);
    V.set(p.rB, midx - b.position.x, midy - b.position.y, midz - b.position.z);
    p.normalMass = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.normal));
    p.tangentMass1 = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.tangent1));
    p.tangentMass2 = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.tangent2));
    // Measured from the velocities at the start of the step: gravity applied this step would
    // otherwise inflate every rebound by g·dt.
    const vn0 = startNormalSpeedAt(c, p);
    // Restitution: bounce back with speed e·|vn0| only for approaches faster than the threshold.
    p.restitutionTarget = vn0 < -threshold ? -c.restitution * vn0 : 0;
    // A speculative contact (shapes still apart) lets the bodies approach by the gap, no further.
    if (p.gap > 0 && dt > 0) p.restitutionTarget = -p.gap / dt;
  }
}

const ROT_SCRATCH_A = new Vec3();
const ROT_SCRATCH_B = new Vec3();
let rotToggle = false;

/** q * v into one of two reused vectors (so a caller may hold two results at once). */
function rotated(q: Quat, v: Vec3): Vec3 {
  rotToggle = !rotToggle;
  return Q.rotate(rotToggle ? ROT_SCRATCH_A : ROT_SCRATCH_B, q, v);
}

/** Normal effective-mass matrix over all points of a constraint: K_ij = invMA + invMB + (rA_i×n)·IA(rA_j×n) + (rB_i×n)·IB(rB_j×n). */
export function buildNormalK(c: ContactConstraint): void {
  buildNormalKRaw(c);
  // Four coplanar points have a rank-3 matrix (translation and two rotations), so the complementarity
  // problem has a family of solutions and the solver would pick an arbitrary, asymmetric one. A tiny
  // Tikhonov term selects the symmetric, minimum-norm solution without changing the physics.
  const n = c.points.length;
  let maxDiag = 0;
  for (let i = 0; i < n; i++) maxDiag = Math.max(maxDiag, c.normalK[i * n + i] as number);
  const eps = 1e-9 * maxDiag;
  for (let i = 0; i < n; i++) c.normalK[i * n + i] = (c.normalK[i * n + i] as number) + eps;
}

function buildNormalKRaw(c: ContactConstraint): void {
  const n = c.points.length;
  if (c.normalK.length !== n * n) c.normalK = new Float64Array(n * n);
  const nr = c.normal;
  const a = c.bodyA;
  const b = c.bodyB;
  const aSim = a.isSimulated;
  const bSim = b.isSimulated;
  const base = (aSim ? a.invMass : 0) + (bSim ? b.invMass : 0);
  const K = c.normalK;
  for (let i = 0; i < n; i++) {
    const pi = c.points[i] as SolverPoint;
    for (let j = 0; j < n; j++) {
      const pj = c.points[j] as SolverPoint;
      let k = base;
      if (aSim) k += crossDot(a.worldInvInertia.e, pi.rA, pj.rA, nr);
      if (bSim) k += crossDot(b.worldInvInertia.e, pi.rB, pj.rB, nr);
      K[i * n + j] = k;
    }
  }
}

/** (r1 × n) · I (r2 × n) for a 3x3 row-major I. */
function crossDot(e: Float64Array, r1: Vec3, r2: Vec3, n: Vec3): number {
  const ax = r1.y * n.z - r1.z * n.y;
  const ay = r1.z * n.x - r1.x * n.z;
  const az = r1.x * n.y - r1.y * n.x;
  const bx = r2.y * n.z - r2.z * n.y;
  const by = r2.z * n.x - r2.x * n.z;
  const bz = r2.x * n.y - r2.y * n.x;
  const ix = (e[0] as number) * bx + (e[1] as number) * by + (e[2] as number) * bz;
  const iy = (e[3] as number) * bx + (e[4] as number) * by + (e[5] as number) * bz;
  const iz = (e[6] as number) * bx + (e[7] as number) * by + (e[8] as number) * bz;
  return ax * ix + ay * iy + az * iz;
}

function invOrZero(k: number): number {
  return k > 0 ? 1 / k : 0;
}

/** Warm start: re-applies the accumulated impulses from the previous step. */
export function warmStart(c: ContactConstraint): void {
  for (const p of c.points) {
    const P = impulseVector(c, p.impulseN, p.impulseT1, p.impulseT2);
    applyImpulse(c.bodyA, V.negate(new Vec3(), P), p.rA);
    applyImpulse(c.bodyB, P, p.rB);
  }
}

function impulseVector(c: ContactConstraint, n: number, t1: number, t2: number): Vec3 {
  return new Vec3(
    c.normal.x * n + c.tangent1.x * t1 + c.tangent2.x * t2,
    c.normal.y * n + c.tangent1.y * t1 + c.tangent2.y * t2,
    c.normal.z * n + c.tangent1.z * t1 + c.tangent2.z * t2,
  );
}

/**
 * Largest change in relative speed (m/s) caused by any impulse update since the last reset. The
 * world resets it before each velocity iteration and stops iterating once it is negligible.
 */
export const solverStats = { maxChange: 0 };

/**
 * One sequential-impulse pass. All normal impulses are solved first (each clamped to ≥ 0), then
 * friction is solved against the updated normal loads, so no point's friction is sized from an
 * unsettled normal load. Solving per point in one interleaved loop lets friction at one corner
 * saturate and pitch the body.
 */
export function solveVelocity(c: ContactConstraint): void {
  const n = c.normal;
  if (c.points.length > 1 && solveNormalBlock(c)) {
    // Block solution applied: impulses already updated.
  } else {
    for (const p of c.points) {
      const vn = relativeSpeed(c, p, n.x, n.y, n.z);
      const next = Math.max(0, p.impulseN + (p.restitutionTarget - vn) * p.normalMass);
      const dn = next - p.impulseN;
      p.impulseN = next;
      if (p.normalMass > 0) trackChange(Math.abs(dn) / p.normalMass);
      applyPair(c, p, n.x * dn, n.y * dn, n.z * dn);
    }
  }
  for (const p of c.points) {
    const maxF = c.friction * p.impulseN;
    solveTangent(c, p, c.tangent1, maxF, 1);
    solveTangent(c, p, c.tangent2, maxF, 2);
  }
}

/** Scratch for the block solve; manifolds have at most MAX_BLOCK points, larger ones fall back. */
const MAX_BLOCK = 4;
const BLK_Q = new Float64Array(MAX_BLOCK);
const BLK_LAM = new Float64Array(MAX_BLOCK);
const BLK_S = new Int32Array(MAX_BLOCK);
const BLK_M = new Float64Array(MAX_BLOCK * (MAX_BLOCK + 1));

/**
 * Solves the normal impulses of a multi-point manifold as a linear complementarity problem:
 * find λ ≥ 0 with w = Kλ + q ≥ 0 and λ·w = 0, where q = (vn − target) − Kλ_old. Active sets are
 * enumerated (at most 16 for four points); the first consistent one is taken. Returns false if no
 * subset is consistent, in which case the caller falls back to the sequential update.
 */
function solveNormalBlock(c: ContactConstraint): boolean {
  const pts = c.points;
  const n = pts.length;
  if (n > MAX_BLOCK) return false;
  const K = c.normalK;
  const nrm = c.normal;
  for (let i = 0; i < n; i++) {
    const p = pts[i] as SolverPoint;
    let kl = 0;
    for (let j = 0; j < n; j++) kl += (K[i * n + j] as number) * (pts[j] as SolverPoint).impulseN;
    BLK_Q[i] = relativeSpeed(c, p, nrm.x, nrm.y, nrm.z) - p.restitutionTarget - kl;
  }
  const EPS = 1e-12;
  for (let mask = (1 << n) - 1; mask >= 0; mask--) {
    let m = 0;
    for (let i = 0; i < n; i++) if (mask & (1 << i)) BLK_S[m++] = i;
    BLK_LAM.fill(0, 0, n);
    if (m > 0) {
      for (let r = 0; r < m; r++) {
        const si = BLK_S[r] as number;
        for (let k = 0; k < m; k++) {
          BLK_M[r * (m + 1) + k] = K[si * n + (BLK_S[k] as number)] as number;
        }
        BLK_M[r * (m + 1) + m] = -(BLK_Q[si] as number);
      }
      if (!eliminate(m)) continue;
      let ok = true;
      for (let k = 0; k < m; k++) {
        const v = BLK_M[k * (m + 1) + m] as number;
        if (v < -EPS) ok = false;
        BLK_LAM[BLK_S[k] as number] = Math.max(0, v);
      }
      if (!ok) continue;
    }
    // Check w = Kλ + q ≥ 0 on the inactive set.
    let feasible = true;
    for (let i = 0; i < n && feasible; i++) {
      if (mask & (1 << i)) continue;
      let w = BLK_Q[i] as number;
      for (let j = 0; j < n; j++) w += (K[i * n + j] as number) * (BLK_LAM[j] as number);
      if (w < -1e-9) feasible = false;
    }
    if (!feasible) continue;
    // Apply the incremental impulses.
    for (let i = 0; i < n; i++) {
      const p = pts[i] as SolverPoint;
      const lam = BLK_LAM[i] as number;
      const d = lam - p.impulseN;
      p.impulseN = lam;
      if (p.normalMass > 0) trackChange(Math.abs(d) / p.normalMass);
      applyPair(c, p, nrm.x * d, nrm.y * d, nrm.z * d);
    }
    return true;
  }
  return false;
}

/**
 * Gaussian elimination with partial pivoting on the augmented m×(m+1) system in BLK_M; the
 * solution is left in the last column. Returns false if singular.
 */
function eliminate(m: number): boolean {
  const w = m + 1;
  for (let col = 0; col < m; col++) {
    let piv = col;
    for (let r = col + 1; r < m; r++) {
      if (Math.abs(BLK_M[r * w + col] as number) > Math.abs(BLK_M[piv * w + col] as number))
        piv = r;
    }
    if (Math.abs(BLK_M[piv * w + col] as number) < 1e-14) return false;
    if (piv !== col) {
      for (let k = 0; k < w; k++) {
        const t = BLK_M[col * w + k] as number;
        BLK_M[col * w + k] = BLK_M[piv * w + k] as number;
        BLK_M[piv * w + k] = t;
      }
    }
    const pc = BLK_M[col * w + col] as number;
    for (let r = col + 1; r < m; r++) {
      const f = (BLK_M[r * w + col] as number) / pc;
      for (let k = col; k <= m; k++) {
        BLK_M[r * w + k] = (BLK_M[r * w + k] as number) - f * (BLK_M[col * w + k] as number);
      }
    }
  }
  for (let r = m - 1; r >= 0; r--) {
    let s = BLK_M[r * w + m] as number;
    for (let k = r + 1; k < m; k++)
      s -= (BLK_M[r * w + k] as number) * (BLK_M[k * w + m] as number);
    BLK_M[r * w + m] = s / (BLK_M[r * w + r] as number);
  }
  return true;
}

/** Solves one friction direction at a point, clamped to the Coulomb bound. `which` picks the stored accumulator. */
function solveTangent(
  c: ContactConstraint,
  p: SolverPoint,
  t: Vec3,
  maxF: number,
  which: 1 | 2,
): void {
  const mass = which === 1 ? p.tangentMass1 : p.tangentMass2;
  const acc = which === 1 ? p.impulseT1 : p.impulseT2;
  const vt = relativeSpeed(c, p, t.x, t.y, t.z);
  const next = Math.max(-maxF, Math.min(maxF, acc - vt * mass));
  const d = next - acc;
  if (mass > 0) trackChange(Math.abs(d) / mass);
  if (which === 1) p.impulseT1 = next;
  else p.impulseT2 = next;
  applyPair(c, p, t.x * d, t.y * d, t.z * d);
}

function trackChange(dv: number): void {
  if (dv > solverStats.maxChange) solverStats.maxChange = dv;
}

/** Applies +P to body B and −P to body A at the point's offsets. */
function applyPair(c: ContactConstraint, p: SolverPoint, px: number, py: number, pz: number): void {
  applyImpulseXYZ(c.bodyA, -px, -py, -pz, p.rA.x, p.rA.y, p.rA.z);
  applyImpulseXYZ(c.bodyB, px, py, pz, p.rB.x, p.rB.y, p.rB.z);
}

/** Normal speed of B relative to A at a contact point, from the velocities at the start of the step. */
function startNormalSpeedAt(c: ContactConstraint, p: SolverPoint): number {
  const a = c.bodyA;
  const b = c.bodyB;
  return speedAlong(
    a.stepStartLinear,
    a.stepStartAngular,
    b.stepStartLinear,
    b.stepStartAngular,
    p,
    c.normal.x,
    c.normal.y,
    c.normal.z,
  );
}

/** Speed of B relative to A at a contact point along (dx, dy, dz), from the current velocities. */
function relativeSpeed(c: ContactConstraint, p: SolverPoint, dx: number, dy: number, dz: number) {
  const a = c.bodyA;
  const b = c.bodyB;
  return speedAlong(
    a.linearVelocity,
    a.angularVelocity,
    b.linearVelocity,
    b.angularVelocity,
    p,
    dx,
    dy,
    dz,
  );
}

function speedAlong(
  la: Vec3,
  wa: Vec3,
  lb: Vec3,
  wb: Vec3,
  p: SolverPoint,
  dx: number,
  dy: number,
  dz: number,
): number {
  const r = p.rA;
  const s = p.rB;
  const vax = la.x + (wa.y * r.z - wa.z * r.y);
  const vay = la.y + (wa.z * r.x - wa.x * r.z);
  const vaz = la.z + (wa.x * r.y - wa.y * r.x);
  const vbx = lb.x + (wb.y * s.z - wb.z * s.y);
  const vby = lb.y + (wb.z * s.x - wb.x * s.z);
  const vbz = lb.z + (wb.x * s.y - wb.y * s.x);
  return (vbx - vax) * dx + (vby - vay) * dy + (vbz - vaz) * dz;
}

/** Normal speed of B relative to A at a contact point. */
export function normalSpeedAt(c: ContactConstraint, p: SolverPoint): number {
  return relativeSpeed(c, p, c.normal.x, c.normal.y, c.normal.z);
}

/**
 * Nonlinear Gauss–Seidel position correction. Pushes the bodies apart along the normal until the
 * separation reaches −slop, using the current poses and leaving velocities unchanged.
 * Returns the most negative remaining position error.
 */
export function solvePosition(
  c: ContactConstraint,
  baumgarte: number,
  slop: number,
  maxStep = Infinity,
): number {
  let worst = 0;
  const a = c.bodyA;
  const b = c.bodyB;
  const n = c.normal;
  for (const p of c.points) {
    const ra = rotated(a.orientation, p.localA);
    const rb = rotated(b.orientation, p.localB);
    const sep =
      (b.position.x + rb.x - a.position.x - ra.x) * n.x +
      (b.position.y + rb.y - a.position.y - ra.y) * n.y +
      (b.position.z + rb.z - a.position.z - ra.z) * n.z;
    const error = Math.max(-maxStep / baumgarte, Math.min(0, sep + slop));
    worst = Math.min(worst, error);
    if (error === 0) continue;
    // Linear-only correction, so the effective mass is the translational one.
    const ima = a.isSimulated ? a.invMass : 0;
    const imb = b.isSimulated ? b.invMass : 0;
    const kn = ima + imb;
    if (!(kn > 0)) continue;
    const s = (-baumgarte * error) / kn;
    applyPositionImpulse(a, -n.x * s, -n.y * s, -n.z * s);
    applyPositionImpulse(b, n.x * s, n.y * s, n.z * s);
  }
  return worst;
}

/**
 * Position correction is linear only. Pushing bodies apart at contact points also rotates them;
 * with sequential per-point corrections on multi-point manifolds that rotation grows and tall stacks
 * topple (measured: a 20-box tower collapsed within seconds). The velocity solve still carries the
 * rotational effects, so only the positional fix-up is translational.
 */
function applyPositionImpulse(body: RigidBody, px: number, py: number, pz: number): void {
  if (!body.isSimulated) return;
  body.position.x += px * body.invMass;
  body.position.y += py * body.invMass;
  body.position.z += pz * body.invMass;
}
