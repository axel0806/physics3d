import { Vec3 } from '../math';
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
  /** Normal effective-mass matrix K_ij (coupled across points through rotation). Filled by prepareContact. */
  normalK: number[][];
}

/** Orthonormal tangent basis perpendicular to unit vector n. */
export function tangentBasis(n: Vec3): [Vec3, Vec3] {
  const t1 = V.perpendicular(new Vec3(), n);
  const t2 = V.cross(new Vec3(), n, t1);
  return [t1, t2];
}

/** Applies a linear impulse P at world offset r from the center of mass. */
export function applyImpulse(body: RigidBody, P: Vec3, r: Vec3): void {
  if (!body.isSimulated) return;
  body.linearVelocity.x += P.x * body.invMass;
  body.linearVelocity.y += P.y * body.invMass;
  body.linearVelocity.z += P.z * body.invMass;
  const torque = V.cross(new Vec3(), r, P);
  const dw = M3.mulVec(new Vec3(), body.worldInvInertia, torque);
  V.add(body.angularVelocity, body.angularVelocity, dw);
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
export function prepareContact(c: ContactConstraint, threshold: number): void {
  const a = c.bodyA;
  const b = c.bodyB;
  c.points.forEach((p) => {
    // Impulses act at the midpoint of the two surface anchors, so both bodies receive their
    // offsets from the same world point. Equal and opposite impulses then exert no net torque
    // about the origin even for tangential friction.
    const pa = V.add(new Vec3(), a.position, Q.rotate(new Vec3(), a.orientation, p.localA));
    const pb = V.add(new Vec3(), b.position, Q.rotate(new Vec3(), b.orientation, p.localB));
    const mid = V.lerp(new Vec3(), pa, pb, 0.5);
    V.sub(p.rA, mid, a.position);
    V.sub(p.rB, mid, b.position);
    p.normalMass = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.normal));
    p.tangentMass1 = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.tangent1));
    p.tangentMass2 = invOrZero(effectiveMass(a, b, p.rA, p.rB, c.tangent2));
    // Measured from the velocities at the start of the step: gravity applied this step would
    // otherwise inflate every rebound by g·dt.
    const vn0 = startNormalSpeedAt(c, p);
    // Restitution: bounce back with speed e·|vn0| only for approaches faster than the threshold.
    p.restitutionTarget = vn0 < -threshold ? -c.restitution * vn0 : 0;
  });
}

/** Normal effective-mass matrix over all points of a constraint: K_ij = invMA + invMB + (rA_i×n)·IA(rA_j×n) + (rB_i×n)·IB(rB_j×n). */
export function buildNormalK(c: ContactConstraint): void {
  buildNormalKRaw(c);
  // Four coplanar points have a rank-3 matrix (translation and two rotations), so the complementarity
  // problem has a family of solutions and the solver would pick an arbitrary, asymmetric one. A tiny
  // Tikhonov term selects the symmetric, minimum-norm solution without changing the physics.
  let maxDiag = 0;
  c.normalK.forEach((row, i) => (maxDiag = Math.max(maxDiag, row[i] ?? 0)));
  const eps = 1e-9 * maxDiag;
  c.normalK.forEach((row, i) => (row[i] = (row[i] ?? 0) + eps));
}

function buildNormalKRaw(c: ContactConstraint): void {
  const n = c.normal;
  const a = c.bodyA;
  const b = c.bodyB;
  const cross = (r: Vec3) => V.cross(new Vec3(), r, n);
  const ca = c.points.map((p) => cross(p.rA));
  const cb = c.points.map((p) => cross(p.rB));
  const ia = ca.map((v) =>
    a.isSimulated ? M3.mulVec(new Vec3(), a.worldInvInertia, v) : new Vec3(),
  );
  const ib = cb.map((v) =>
    b.isSimulated ? M3.mulVec(new Vec3(), b.worldInvInertia, v) : new Vec3(),
  );
  c.normalK = c.points.map((_, i) =>
    c.points.map((_, j) => {
      let k = (a.isSimulated ? a.invMass : 0) + (b.isSimulated ? b.invMass : 0);
      if (a.isSimulated) k += V.dot(ca[i] as Vec3, ia[j] as Vec3);
      if (b.isSimulated) k += V.dot(cb[i] as Vec3, ib[j] as Vec3);
      return k;
    }),
  );
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
 * One sequential-impulse pass. All normal impulses are solved first (each clamped to ≥ 0), then
 * friction is solved against the updated normal loads, so no point's friction is sized from an
 * unsettled normal load. Solving per point in one interleaved loop lets friction at one corner
 * saturate and pitch the body.
 */
export function solveVelocity(c: ContactConstraint): void {
  const a = c.bodyA;
  const b = c.bodyB;
  if (c.points.length > 1 && solveNormalBlock(c)) {
    // Block solution applied: impulses already updated.
  } else {
    for (const p of c.points) {
      const vn = normalSpeedAt(c, p);
      const next = Math.max(0, p.impulseN + (p.restitutionTarget - vn) * p.normalMass);
      const dn = next - p.impulseN;
      p.impulseN = next;
      const Pn = V.scale(new Vec3(), c.normal, dn);
      applyImpulse(a, V.negate(new Vec3(), Pn), p.rA);
      applyImpulse(b, Pn, p.rB);
    }
  }
  for (const p of c.points) {
    const maxF = c.friction * p.impulseN;
    solveTangent(c, p, c.tangent1, maxF, 1);
    solveTangent(c, p, c.tangent2, maxF, 2);
  }
}

/**
 * Solves the normal impulses of a multi-point manifold as a linear complementarity problem:
 * find λ ≥ 0 with w = Kλ + q ≥ 0 and λ·w = 0, where q = (vn − target) − Kλ_old. Active sets are
 * enumerated (at most 16 for four points); the first consistent one is taken. Returns false if no
 * subset is consistent, in which case the caller falls back to the sequential update.
 */
function solveNormalBlock(c: ContactConstraint): boolean {
  const pts = c.points;
  const n = pts.length;
  const K = c.normalK;
  const lamOld = pts.map((p) => p.impulseN);
  const q = pts.map((p, i) => {
    let kl = 0;
    for (let j = 0; j < n; j++) kl += (K[i]?.[j] ?? 0) * (lamOld[j] ?? 0);
    return normalSpeedAt(c, p) - p.restitutionTarget - kl;
  });
  const EPS = 1e-12;
  for (let mask = (1 << n) - 1; mask >= 0; mask--) {
    const S: number[] = [];
    for (let i = 0; i < n; i++) if (mask & (1 << i)) S.push(i);
    const lam = new Array<number>(n).fill(0);
    if (S.length > 0) {
      const A = S.map((i) => S.map((j) => K[i]?.[j] ?? 0));
      const rhs = S.map((i) => -(q[i] ?? 0));
      const x = solveDense(A, rhs);
      if (!x) continue;
      let ok = true;
      S.forEach((i, k) => {
        const v = x[k] ?? 0;
        if (v < -EPS) ok = false;
        lam[i] = Math.max(0, v);
      });
      if (!ok) continue;
    }
    // Check w = Kλ + q ≥ 0 on the inactive set.
    let feasible = true;
    for (let i = 0; i < n && feasible; i++) {
      if (S.includes(i)) continue;
      let w = q[i] ?? 0;
      for (let j = 0; j < n; j++) w += (K[i]?.[j] ?? 0) * lam[j]!;
      if (w < -1e-9) feasible = false;
    }
    if (!feasible) continue;
    // Apply the incremental impulses.
    for (let i = 0; i < n; i++) {
      const p = pts[i] as SolverPoint;
      const d = (lam[i] ?? 0) - (p.impulseN ?? 0);
      p.impulseN = lam[i] ?? 0;
      const P = V.scale(new Vec3(), c.normal, d);
      applyImpulse(c.bodyA, V.negate(new Vec3(), P), p.rA);
      applyImpulse(c.bodyB, P, p.rB);
    }
    return true;
  }
  return false;
}

/** Dense Gaussian elimination with partial pivoting for the tiny block systems; null if singular. */
function solveDense(Ain: number[][], bin: number[]): number[] | null {
  const n = bin.length;
  const M = Ain.map((row, i) => [...row, bin[i] ?? 0]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++)
      if (Math.abs(M[r]?.[c] ?? 0) > Math.abs(M[piv]?.[c] ?? 0)) piv = r;
    const pr = M[piv] ?? [];
    if (Math.abs(pr[c] ?? 0) < 1e-14) return null;
    M[piv] = M[c] ?? [];
    M[c] = pr;
    for (let r = c + 1; r < n; r++) {
      const row = M[r] ?? [];
      const f = (row[c] ?? 0) / (pr[c] ?? 1);
      for (let k = c; k <= n; k++) row[k] = (row[k] ?? 0) - f * (pr[k] ?? 0);
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    const row = M[r] ?? [];
    let s = row[n] ?? 0;
    for (let k = r + 1; k < n; k++) s -= (row[k] ?? 0) * (x[k] ?? 0);
    x[r] = s / (row[r] ?? 1);
  }
  return x;
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
  const vt = tangentSpeedAt(c, p, t);
  const next = Math.max(-maxF, Math.min(maxF, acc - vt * mass));
  const d = next - acc;
  if (which === 1) p.impulseT1 = next;
  else p.impulseT2 = next;
  const P = V.scale(new Vec3(), t, d);
  applyImpulse(c.bodyA, V.negate(new Vec3(), P), p.rA);
  applyImpulse(c.bodyB, P, p.rB);
}

/** Normal speed of B relative to A at a contact point, from the velocities at the start of the step. */
function startNormalSpeedAt(c: ContactConstraint, p: SolverPoint): number {
  const vA = pointVelocity(c.bodyA.stepStartLinear, c.bodyA.stepStartAngular, p.rA);
  const vB = pointVelocity(c.bodyB.stepStartLinear, c.bodyB.stepStartAngular, p.rB);
  return V.dot(V.sub(new Vec3(), vB, vA), c.normal);
}

function pointVelocity(v: Vec3, w: Vec3, r: Vec3): Vec3 {
  return new Vec3(
    v.x + (w.y * r.z - w.z * r.y),
    v.y + (w.z * r.x - w.x * r.z),
    v.z + (w.x * r.y - w.y * r.x),
  );
}

/** Normal speed of B relative to A at a contact point. */
export function normalSpeedAt(c: ContactConstraint, p: SolverPoint): number {
  const vA = c.bodyA.velocityAt(p.rA, new Vec3());
  const vB = c.bodyB.velocityAt(p.rB, new Vec3());
  return V.dot(V.sub(new Vec3(), vB, vA), c.normal);
}

function tangentSpeedAt(c: ContactConstraint, p: SolverPoint, t: Vec3): number {
  const vA = c.bodyA.velocityAt(p.rA, new Vec3());
  const vB = c.bodyB.velocityAt(p.rB, new Vec3());
  return V.dot(V.sub(new Vec3(), vB, vA), t);
}

/**
 * Nonlinear Gauss–Seidel position correction. Pushes the bodies apart along the normal until the
 * separation reaches −slop, using the current poses and leaving velocities unchanged.
 * Returns the most negative remaining position error.
 */
export function solvePosition(c: ContactConstraint, baumgarte: number, slop: number): number {
  let worst = 0;
  const a = c.bodyA;
  const b = c.bodyB;
  for (const p of c.points) {
    const pa = V.add(new Vec3(), a.position, Q.rotate(new Vec3(), a.orientation, p.localA));
    const pb = V.add(new Vec3(), b.position, Q.rotate(new Vec3(), b.orientation, p.localB));
    const sep = V.dot(V.sub(new Vec3(), pb, pa), c.normal);
    const error = Math.min(0, sep + slop);
    worst = Math.min(worst, error);
    if (error === 0) continue;
    // Linear-only correction, so the effective mass is the translational one.
    const kn = (a.isSimulated ? a.invMass : 0) + (b.isSimulated ? b.invMass : 0);
    if (!(kn > 0)) continue;
    const P = V.scale(new Vec3(), c.normal, (-baumgarte * error) / kn);
    applyPositionImpulse(a, V.negate(new Vec3(), P));
    applyPositionImpulse(b, P);
  }
  return worst;
}

/**
 * Position correction is linear only. Pushing bodies apart at contact points also rotates them;
 * with sequential per-point corrections on multi-point manifolds that rotation grows and tall stacks
 * topple (measured: a 20-box tower collapsed within seconds). The velocity solve still carries the
 * rotational effects, so only the positional fix-up is translational.
 */
function applyPositionImpulse(body: RigidBody, P: Vec3): void {
  if (!body.isSimulated) return;
  body.position.x += P.x * body.invMass;
  body.position.y += P.y * body.invMass;
  body.position.z += P.z * body.invMass;
}
