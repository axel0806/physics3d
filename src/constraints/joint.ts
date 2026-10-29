import { Mat3, Quat, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as Q from '../math/quat';
import * as M3 from '../math/mat3';
import type { RigidBody } from '../dynamics/body';

const ZERO_INERTIA = new Mat3(new Array(9).fill(0));

export function invMassOf(b: RigidBody): number {
  return b.isSimulated ? b.invMass : 0;
}

export function invInertiaOf(b: RigidBody): Mat3 {
  return b.isSimulated ? b.worldInvInertia : ZERO_INERTIA;
}

/** Anchor of a world point in a body's frame, relative to its center of mass. */
export function localPoint(body: RigidBody, world: Vec3): Vec3 {
  return Q.rotateInverse(new Vec3(), body.orientation, V.sub(new Vec3(), world, body.position));
}

/** Direction of a world vector in a body's frame. */
export function localDir(body: RigidBody, world: Vec3): Vec3 {
  return Q.rotateInverse(new Vec3(), body.orientation, world);
}

/** World offset of a body-frame vector. */
export function worldDir(body: RigidBody, local: Vec3): Vec3 {
  return Q.rotate(new Vec3(), body.orientation, local);
}

/** Translates a body (no-op unless simulated). */
export function movePosition(body: RigidBody, d: Vec3): void {
  if (!body.isSimulated) return;
  V.add(body.position, body.position, d);
}

/** Applies a small world-frame rotation vector dθ to a body's orientation. */
export function rotateBy(body: RigidBody, dtheta: Vec3): void {
  if (!body.isSimulated) return;
  const q = body.orientation;
  const r = Q.mul(new Quat(), new Quat(dtheta.x, dtheta.y, dtheta.z, 0), q);
  q.x += 0.5 * r.x;
  q.y += 0.5 * r.y;
  q.z += 0.5 * r.z;
  q.w += 0.5 * r.w;
  Q.normalize(q, q);
  body.updateWorldInertia();
}

/** Builds a perpendicular unit vector pair (t1, t2) for unit vector n. */
export function basis(n: Vec3): [Vec3, Vec3] {
  const t1 = V.perpendicular(new Vec3(), n);
  return [t1, V.cross(new Vec3(), n, t1)];
}

/**
 * A scalar constraint row. The velocity of the constrained quantity is
 * `n·(vB − vA) + aB·ωB − aA·ωA`, so a linear row has `aA = rA×n`, `aB = rB×n`, and an angular
 * row has `n = 0`, `aA = aB = axis`. One class serves contacts-style limits, motors, springs, and
 * the perpendicular locks of every joint.
 */
export class Row {
  readonly n = new Vec3();
  readonly aA = new Vec3();
  readonly aB = new Vec3();
  /** Effective mass 1/(K + γ). */
  mass = 0;
  /** Softness γ for spring rows; zero for rigid rows. */
  gamma = 0;
  impulse = 0;

  set(a: RigidBody, b: RigidBody, n: Vec3, aA: Vec3, aB: Vec3, gamma = 0): void {
    V.copy(this.n, n);
    V.copy(this.aA, aA);
    V.copy(this.aB, aB);
    this.gamma = gamma;
    let k = invMassOf(a) + invMassOf(b);
    if (V.lengthSq(n) === 0) k = 0;
    k += V.dot(aA, M3.mulVec(new Vec3(), invInertiaOf(a), aA));
    k += V.dot(aB, M3.mulVec(new Vec3(), invInertiaOf(b), aB));
    const total = k + gamma;
    this.mass = total > 0 ? 1 / total : 0;
  }

  setLinear(a: RigidBody, b: RigidBody, rA: Vec3, rB: Vec3, dir: Vec3, gamma = 0): void {
    this.set(a, b, dir, V.cross(new Vec3(), rA, dir), V.cross(new Vec3(), rB, dir), gamma);
  }

  setAngular(a: RigidBody, b: RigidBody, axis: Vec3): void {
    this.set(a, b, new Vec3(), axis, axis);
  }

  speed(a: RigidBody, b: RigidBody): number {
    let s = V.dot(this.n, b.linearVelocity) - V.dot(this.n, a.linearVelocity);
    s += V.dot(this.aB, b.angularVelocity) - V.dot(this.aA, a.angularVelocity);
    return s;
  }

  apply(a: RigidBody, b: RigidBody, lambda: number): void {
    if (a.isSimulated) {
      V.addScaled(a.linearVelocity, a.linearVelocity, this.n, -lambda * a.invMass);
      const dw = M3.mulVec(new Vec3(), a.worldInvInertia, this.aA);
      V.addScaled(a.angularVelocity, a.angularVelocity, dw, -lambda);
    }
    if (b.isSimulated) {
      V.addScaled(b.linearVelocity, b.linearVelocity, this.n, lambda * b.invMass);
      const dw = M3.mulVec(new Vec3(), b.worldInvInertia, this.aB);
      V.addScaled(b.angularVelocity, b.angularVelocity, dw, lambda);
    }
  }

  warm(a: RigidBody, b: RigidBody): void {
    if (this.impulse !== 0) this.apply(a, b, this.impulse);
  }

  /** One velocity iteration with accumulated impulse clamped to [lo, hi]. */
  solve(a: RigidBody, b: RigidBody, bias: number, lo: number, hi: number): void {
    const cdot = this.speed(a, b);
    const lambda = -this.mass * (cdot + bias + this.gamma * this.impulse);
    const old = this.impulse;
    this.impulse = Math.min(hi, Math.max(lo, old + lambda));
    this.apply(a, b, this.impulse - old);
  }

  /** Position-level correction of constraint error `c` (positive error pushes along −row). */
  position(a: RigidBody, b: RigidBody, c: number, beta: number): void {
    const lambda = -beta * c * this.mass;
    if (lambda === 0) return;
    if (a.isSimulated) {
      movePosition(a, V.scale(new Vec3(), this.n, -lambda * a.invMass));
      rotateBy(a, V.scale(new Vec3(), M3.mulVec(new Vec3(), a.worldInvInertia, this.aA), -lambda));
    }
    if (b.isSimulated) {
      movePosition(b, V.scale(new Vec3(), this.n, lambda * b.invMass));
      rotateBy(b, V.scale(new Vec3(), M3.mulVec(new Vec3(), b.worldInvInertia, this.aB), lambda));
    }
  }
}

/** Skew matrix product K contribution: −[r]× I [r]×. */
function addPointK(k: Mat3, r: Vec3, inertia: Mat3): void {
  const s = M3.skew(new Mat3(), r);
  const t = M3.mul(new Mat3(), M3.mul(new Mat3(), s, inertia), s);
  for (let i = 0; i < 9; i++) k.e[i] = (k.e[i] ?? 0) - (t.e[i] ?? 0);
}

/** Three-dimensional point-to-point lock, solved as a 3×3 block. */
export class PointLock {
  readonly rA = new Vec3();
  readonly rB = new Vec3();
  readonly impulse = new Vec3();
  private readonly kInv = new Mat3();

  private build(a: RigidBody, b: RigidBody): void {
    const k = new Mat3(new Array(9).fill(0));
    const m = invMassOf(a) + invMassOf(b);
    k.e[0] = k.e[4] = k.e[8] = m;
    addPointK(k, this.rA, invInertiaOf(a));
    addPointK(k, this.rB, invInertiaOf(b));
    if (!M3.invert(this.kInv, k)) this.kInv.e.fill(0);
  }

  prepare(a: RigidBody, b: RigidBody, localA: Vec3, localB: Vec3): void {
    Q.rotate(this.rA, a.orientation, localA);
    Q.rotate(this.rB, b.orientation, localB);
    this.build(a, b);
  }

  apply(a: RigidBody, b: RigidBody, p: Vec3): void {
    if (a.isSimulated) {
      V.addScaled(a.linearVelocity, a.linearVelocity, p, -a.invMass);
      const dw = M3.mulVec(new Vec3(), a.worldInvInertia, V.cross(new Vec3(), this.rA, p));
      V.sub(a.angularVelocity, a.angularVelocity, dw);
    }
    if (b.isSimulated) {
      V.addScaled(b.linearVelocity, b.linearVelocity, p, b.invMass);
      const dw = M3.mulVec(new Vec3(), b.worldInvInertia, V.cross(new Vec3(), this.rB, p));
      V.add(b.angularVelocity, b.angularVelocity, dw);
    }
  }

  warm(a: RigidBody, b: RigidBody): void {
    this.apply(a, b, this.impulse);
  }

  solve(a: RigidBody, b: RigidBody): void {
    const va = a.velocityAt(this.rA, new Vec3());
    const vb = b.velocityAt(this.rB, new Vec3());
    const cdot = V.sub(new Vec3(), vb, va);
    const p = V.negate(new Vec3(), M3.mulVec(new Vec3(), this.kInv, cdot));
    V.add(this.impulse, this.impulse, p);
    this.apply(a, b, p);
  }

  /**
   * Moves and rotates both bodies to close the anchor gap; returns the gap length. The error is
   * measured at the current pose but the correction acts along the Jacobian stored by `prepare`
   * (the start-of-step pose), as in SHAKE. Correcting along the end-pose gradient instead shortens
   * every arc by a third-order amount and dissipates energy at first order in dt.
   */
  position(a: RigidBody, b: RigidBody, localA: Vec3, localB: Vec3, beta: number): number {
    const pa = V.add(new Vec3(), a.position, Q.rotate(new Vec3(), a.orientation, localA));
    const pb = V.add(new Vec3(), b.position, Q.rotate(new Vec3(), b.orientation, localB));
    const c = V.sub(new Vec3(), pb, pa);
    const p = V.scale(new Vec3(), M3.mulVec(new Vec3(), this.kInv, c), -beta);
    if (a.isSimulated) {
      movePosition(a, V.scale(new Vec3(), p, -a.invMass));
      const l = V.cross(new Vec3(), this.rA, p);
      rotateBy(a, V.negate(new Vec3(), M3.mulVec(new Vec3(), a.worldInvInertia, l)));
    }
    if (b.isSimulated) {
      movePosition(b, V.scale(new Vec3(), p, b.invMass));
      const l = V.cross(new Vec3(), this.rB, p);
      rotateBy(b, M3.mulVec(new Vec3(), b.worldInvInertia, l));
    }
    return V.length(c);
  }
}

/** Three-dimensional relative-orientation lock, solved as a 3×3 block. */
export class AngularLock {
  readonly impulse = new Vec3();
  private readonly kInv = new Mat3();
  /** Orientation of B relative to A at creation: qA⁻¹·qB. */
  readonly reference: Quat;

  constructor(a: RigidBody, b: RigidBody) {
    this.reference = Q.mul(new Quat(), Q.invert(new Quat(), a.orientation), b.orientation);
  }

  private build(a: RigidBody, b: RigidBody): void {
    const k = M3.add(new Mat3(), invInertiaOf(a), invInertiaOf(b));
    if (!M3.invert(this.kInv, k)) this.kInv.e.fill(0);
  }

  prepare(a: RigidBody, b: RigidBody): void {
    this.build(a, b);
  }

  warm(a: RigidBody, b: RigidBody): void {
    this.apply(a, b, this.impulse);
  }

  apply(a: RigidBody, b: RigidBody, l: Vec3): void {
    if (a.isSimulated) {
      V.sub(a.angularVelocity, a.angularVelocity, M3.mulVec(new Vec3(), a.worldInvInertia, l));
    }
    if (b.isSimulated) {
      V.add(b.angularVelocity, b.angularVelocity, M3.mulVec(new Vec3(), b.worldInvInertia, l));
    }
  }

  solve(a: RigidBody, b: RigidBody): void {
    const cdot = V.sub(new Vec3(), b.angularVelocity, a.angularVelocity);
    const l = V.negate(new Vec3(), M3.mulVec(new Vec3(), this.kInv, cdot));
    V.add(this.impulse, this.impulse, l);
    this.apply(a, b, l);
  }

  /** World-frame rotation vector from the target orientation of B to its actual orientation. */
  error(a: RigidBody, b: RigidBody): Vec3 {
    const target = Q.mul(new Quat(), a.orientation, this.reference);
    const e = Q.mul(new Quat(), b.orientation, Q.invert(new Quat(), target));
    const s = e.w < 0 ? -2 : 2;
    return new Vec3(e.x * s, e.y * s, e.z * s);
  }

  position(a: RigidBody, b: RigidBody, beta: number): number {
    const e = this.error(a, b);
    const l = V.scale(new Vec3(), M3.mulVec(new Vec3(), this.kInv, e), -beta);
    if (a.isSimulated) {
      rotateBy(a, V.negate(new Vec3(), M3.mulVec(new Vec3(), a.worldInvInertia, l)));
    }
    if (b.isSimulated) rotateBy(b, M3.mulVec(new Vec3(), b.worldInvInertia, l));
    return V.length(e);
  }
}

/** Common interface for joints, solved inside the sequential-impulse loop with warm starting. */
export abstract class Joint {
  /** When false (default) the connected bodies do not collide with each other. */
  collideConnected = false;
  /** Position-correction fraction per iteration. */
  static beta = 0.5;
  /** Angular and linear slop below which limits are not corrected. */
  static angularSlop = 0.002;
  static linearSlop = 0.0002;

  constructor(
    readonly bodyA: RigidBody,
    readonly bodyB: RigidBody,
  ) {}

  /** Accumulated impulses (the warm-start state) of every lock and row, in field order. */
  saveImpulses(out: number[] = []): number[] {
    for (const part of Object.values(this)) {
      if (part instanceof Row) out.push(part.impulse);
      else if (part instanceof PointLock || part instanceof AngularLock) {
        out.push(part.impulse.x, part.impulse.y, part.impulse.z);
      }
    }
    return out;
  }

  /** Restores impulses written by `saveImpulses`. */
  loadImpulses(data: readonly number[]): void {
    let i = 0;
    for (const part of Object.values(this)) {
      if (part instanceof Row) part.impulse = data[i++] as number;
      else if (part instanceof PointLock || part instanceof AngularLock) {
        part.impulse.x = data[i++] as number;
        part.impulse.y = data[i++] as number;
        part.impulse.z = data[i++] as number;
      }
    }
  }

  get active(): boolean {
    return this.bodyA.isSimulated || this.bodyB.isSimulated;
  }

  abstract prepare(dt: number): void;
  abstract warmStart(): void;
  abstract solveVelocity(dt: number): void;
  /**
   * One NGS iteration over the equality locks (anchors, axes); returns the largest position error
   * before the correction. The world feeds the displacement back into the velocities (SHAKE).
   */
  abstract solvePosition(): number;
  /**
   * One NGS iteration over inequality limits. Not fed back into velocities: that would turn every
   * correction into a bounce. Runs before `solvePosition` so locks clean up what it disturbs.
   */
  solveLimits(): number {
    return 0;
  }
}
