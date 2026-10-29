import { Quat, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as Q from '../math/quat';
import type { RigidBody } from '../dynamics/body';
import { AngularLock, Joint, PointLock, Row, basis, localDir, localPoint, worldDir } from './joint';

const BETA = () => Joint.beta;
/** Largest angular limit violation (rad) corrected in one NGS iteration. */
const MAX_LIMIT_FIX = 0.2;

function checkPoint(v: Vec3, what: string): void {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new RangeError(`${what} must be finite`);
  }
}

function checkAxis(v: Vec3, what: string): void {
  checkPoint(v, what);
  if (!(V.length(v) > 0)) throw new RangeError(`${what} must have non-zero length`);
}

function checkLimits(lower: number, upper: number, what: string): void {
  if (Number.isNaN(lower) || Number.isNaN(upper) || lower > upper) {
    throw new RangeError(`${what}: lower (${lower}) must not exceed upper (${upper})`);
  }
}

/** Ball-and-socket: the two anchor points coincide. Three translational DOF locked. */
export class BallJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  private readonly point = new PointLock();

  constructor(a: RigidBody, b: RigidBody, worldAnchor: Vec3) {
    super(a, b);
    checkPoint(worldAnchor, 'BallJoint anchor');
    this.localA = localPoint(a, worldAnchor);
    this.localB = localPoint(b, worldAnchor);
  }
  prepare(): void {
    this.point.prepare(this.bodyA, this.bodyB, this.localA, this.localB);
  }
  warmStart(): void {
    this.point.warm(this.bodyA, this.bodyB);
  }
  solveVelocity(): void {
    this.point.solve(this.bodyA, this.bodyB);
  }
  solvePosition(): number {
    return this.point.position(this.bodyA, this.bodyB, this.localA, this.localB, BETA());
  }
}

/** Welds two bodies: point lock plus relative orientation lock. */
export class FixedJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  private readonly point = new PointLock();
  private readonly angular: AngularLock;

  constructor(a: RigidBody, b: RigidBody, worldAnchor: Vec3) {
    super(a, b);
    checkPoint(worldAnchor, 'FixedJoint anchor');
    this.localA = localPoint(a, worldAnchor);
    this.localB = localPoint(b, worldAnchor);
    this.angular = new AngularLock(a, b);
  }
  prepare(): void {
    this.point.prepare(this.bodyA, this.bodyB, this.localA, this.localB);
    this.angular.prepare(this.bodyA, this.bodyB);
  }
  warmStart(): void {
    this.point.warm(this.bodyA, this.bodyB);
    this.angular.warm(this.bodyA, this.bodyB);
  }
  solveVelocity(): void {
    this.angular.solve(this.bodyA, this.bodyB);
    this.point.solve(this.bodyA, this.bodyB);
  }
  solvePosition(): number {
    const e1 = this.angular.position(this.bodyA, this.bodyB, BETA());
    const e2 = this.point.position(this.bodyA, this.bodyB, this.localA, this.localB, BETA());
    return Math.max(e1, e2);
  }
}

export interface HingeOptions {
  readonly lower?: number;
  readonly upper?: number;
  readonly motorSpeed?: number;
  readonly maxMotorTorque?: number;
}

/**
 * Hinge (revolute): anchors coincide and the axes stay parallel. The angle θ about the axis is
 * measured from the relative pose at creation, positive when B turns counter-clockwise about the
 * axis as seen on A. Optional limits [lower, upper] and a velocity motor.
 */
export class HingeJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  readonly axisA: Vec3;
  readonly axisB: Vec3;
  /** Reference directions perpendicular to the axis, one in each body frame, equal at creation. */
  private readonly refA: Vec3;
  private readonly refB: Vec3;
  lower: number;
  upper: number;
  enableLimit: boolean;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorTorque: number;

  private readonly point = new PointLock();
  private readonly perp1 = new Row();
  private readonly perp2 = new Row();
  private readonly lowerRow = new Row();
  private readonly upperRow = new Row();
  private readonly motorRow = new Row();
  /** Basis perpendicular to A's axis at prepare time; the alignment error is measured against it. */
  private readonly t1 = new Vec3();
  private readonly t2 = new Vec3();
  private angle = 0;
  private lowerBias = 0;
  private upperBias = 0;
  private lowerOn = false;
  private upperOn = false;

  constructor(
    a: RigidBody,
    b: RigidBody,
    worldAnchor: Vec3,
    worldAxis: Vec3,
    options: HingeOptions = {},
  ) {
    super(a, b);
    checkPoint(worldAnchor, 'HingeJoint anchor');
    checkAxis(worldAxis, 'HingeJoint axis');
    const axis = V.normalize(new Vec3(), worldAxis);
    this.localA = localPoint(a, worldAnchor);
    this.localB = localPoint(b, worldAnchor);
    this.axisA = localDir(a, axis);
    this.axisB = localDir(b, axis);
    const ref = V.perpendicular(new Vec3(), axis);
    this.refA = localDir(a, ref);
    this.refB = localDir(b, ref);
    this.enableLimit = options.lower !== undefined || options.upper !== undefined;
    this.lower = options.lower ?? -Math.PI;
    this.upper = options.upper ?? Math.PI;
    if (this.enableLimit) checkLimits(this.lower, this.upper, 'HingeJoint limits');
    this.enableMotor = options.motorSpeed !== undefined;
    this.motorSpeed = options.motorSpeed ?? 0;
    this.maxMotorTorque = options.maxMotorTorque ?? 0;
  }

  /** Current hinge angle in (−π, π]. */
  get currentAngle(): number {
    return this.measureAngle();
  }

  private measureAngle(): number {
    const aA = worldDir(this.bodyA, this.axisA);
    const uA = worldDir(this.bodyA, this.refA);
    const uB = worldDir(this.bodyB, this.refB);
    const raw = Math.atan2(V.dot(V.cross(new Vec3(), uA, uB), aA), V.dot(uA, uB));
    if (!this.enableLimit) return raw;
    // Unwrap about the middle of the allowed range, so a pose just past a limit is corrected
    // the short way round instead of by almost a full turn the wrong way.
    const mid = 0.5 * (this.lower + this.upper);
    return mid + Math.atan2(Math.sin(raw - mid), Math.cos(raw - mid));
  }

  prepare(dt: number): void {
    const a = this.bodyA;
    const b = this.bodyB;
    this.point.prepare(a, b, this.localA, this.localB);
    this.preparePerp();
    const aA = worldDir(a, this.axisA);
    this.angle = this.measureAngle();
    this.lowerOn = this.upperOn = false;
    if (this.enableLimit) {
      const cl = this.angle - this.lower;
      const cu = this.upper - this.angle;
      this.lowerOn = cl <= 0.1;
      this.upperOn = cu <= 0.1;
      this.lowerBias = cl > 0 ? cl / dt : 0;
      this.upperBias = cu > 0 ? cu / dt : 0;
      if (this.lowerOn) this.lowerRow.setAngular(a, b, aA);
      else this.lowerRow.impulse = 0;
      if (this.upperOn) this.upperRow.setAngular(a, b, V.negate(new Vec3(), aA));
      else this.upperRow.impulse = 0;
    } else {
      this.lowerRow.impulse = this.upperRow.impulse = 0;
    }
    if (this.enableMotor) this.motorRow.setAngular(a, b, aA);
    else this.motorRow.impulse = 0;
  }

  private preparePerp(): void {
    const a = this.bodyA;
    const b = this.bodyB;
    const aA = worldDir(a, this.axisA);
    const aB = worldDir(b, this.axisB);
    const [t1, t2] = basis(aA);
    V.copy(this.t1, t1);
    V.copy(this.t2, t2);
    this.perp1.setAngular(a, b, V.cross(new Vec3(), aB, t1));
    this.perp2.setAngular(a, b, V.cross(new Vec3(), aB, t2));
  }

  warmStart(): void {
    const a = this.bodyA;
    const b = this.bodyB;
    this.point.warm(a, b);
    this.perp1.warm(a, b);
    this.perp2.warm(a, b);
    if (this.lowerOn) this.lowerRow.warm(a, b);
    if (this.upperOn) this.upperRow.warm(a, b);
    if (this.enableMotor) this.motorRow.warm(a, b);
  }

  solveVelocity(dt: number): void {
    const a = this.bodyA;
    const b = this.bodyB;
    if (this.enableMotor) {
      const max = this.maxMotorTorque * dt;
      this.motorRow.solve(a, b, -this.motorSpeed, -max, max);
    }
    if (this.lowerOn) this.lowerRow.solve(a, b, this.lowerBias, 0, Infinity);
    if (this.upperOn) this.upperRow.solve(a, b, this.upperBias, 0, Infinity);
    this.perp1.solve(a, b, 0, -Infinity, Infinity);
    this.perp2.solve(a, b, 0, -Infinity, Infinity);
    this.point.solve(a, b);
  }

  solvePosition(): number {
    const a = this.bodyA;
    const b = this.bodyB;
    let worst = 0;
    // Axis misalignment: C_i = aB·t_i, corrected along the rows prepared at the start of the step.
    const aB = worldDir(b, this.axisB);
    const c1 = V.dot(aB, this.t1);
    const c2 = V.dot(aB, this.t2);
    this.perp1.position(a, b, c1, BETA());
    this.perp2.position(a, b, c2, BETA());
    worst = Math.max(worst, Math.abs(c1), Math.abs(c2));
    worst = Math.max(worst, this.point.position(a, b, this.localA, this.localB, BETA()));
    return worst;
  }

  override solveLimits(): number {
    const a = this.bodyA;
    const b = this.bodyB;
    let worst = 0;
    if (this.enableLimit) {
      const theta = this.measureAngle();
      const axis = worldDir(a, this.axisA);
      const low = Math.max(-MAX_LIMIT_FIX, Math.min(0, theta - this.lower + Joint.angularSlop));
      const high = Math.max(-MAX_LIMIT_FIX, Math.min(0, this.upper - theta + Joint.angularSlop));
      if (low < 0) {
        this.lowerRow.setAngular(a, b, axis);
        this.lowerRow.position(a, b, low, BETA());
      }
      if (high < 0) {
        this.upperRow.setAngular(a, b, V.negate(new Vec3(), axis));
        this.upperRow.position(a, b, high, BETA());
      }
      worst = Math.max(worst, -low, -high);
    }
    return worst;
  }
}

export interface SliderOptions {
  readonly lower?: number;
  readonly upper?: number;
  readonly motorSpeed?: number;
  readonly maxMotorForce?: number;
}

/**
 * Slider (prismatic): relative rotation locked, translation allowed only along the axis. The
 * translation s is measured from the anchor separation at creation, positive along A's axis.
 */
export class SliderJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  readonly axisA: Vec3;
  lower: number;
  upper: number;
  enableLimit: boolean;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorForce: number;

  private readonly angular: AngularLock;
  private readonly perp1 = new Row();
  private readonly perp2 = new Row();
  private readonly lowerRow = new Row();
  private readonly upperRow = new Row();
  private readonly motorRow = new Row();
  private readonly t1 = new Vec3();
  private readonly t2 = new Vec3();
  private lowerBias = 0;
  private upperBias = 0;
  private lowerOn = false;
  private upperOn = false;

  constructor(
    a: RigidBody,
    b: RigidBody,
    worldAnchor: Vec3,
    worldAxis: Vec3,
    options: SliderOptions = {},
  ) {
    super(a, b);
    checkPoint(worldAnchor, 'SliderJoint anchor');
    checkAxis(worldAxis, 'SliderJoint axis');
    this.localA = localPoint(a, worldAnchor);
    this.localB = localPoint(b, worldAnchor);
    this.axisA = localDir(a, V.normalize(new Vec3(), worldAxis));
    this.angular = new AngularLock(a, b);
    this.enableLimit = options.lower !== undefined || options.upper !== undefined;
    this.lower = options.lower ?? -Infinity;
    this.upper = options.upper ?? Infinity;
    if (this.enableLimit) checkLimits(this.lower, this.upper, 'SliderJoint limits');
    this.enableMotor = options.motorSpeed !== undefined;
    this.motorSpeed = options.motorSpeed ?? 0;
    this.maxMotorForce = options.maxMotorForce ?? 0;
  }

  /** Translation along the axis (anchor separation projected on A's axis). */
  get translation(): number {
    const { d, axis } = this.frame();
    return V.dot(d, axis);
  }

  private frame() {
    const a = this.bodyA;
    const b = this.bodyB;
    const rA = worldDir(a, this.localA);
    const rB = worldDir(b, this.localB);
    const d = V.sub(
      new Vec3(),
      V.add(new Vec3(), b.position, rB),
      V.add(new Vec3(), a.position, rA),
    );
    return { rA, rB, d, axis: worldDir(a, this.axisA) };
  }

  /** Configures a row whose constrained quantity is the anchor separation along `dir` (attached to A). */
  private linearRow(row: Row, dir: Vec3, f: ReturnType<SliderJoint['frame']>): void {
    const aA = V.cross(new Vec3(), V.add(new Vec3(), f.d, f.rA), dir);
    const aB = V.cross(new Vec3(), f.rB, dir);
    row.set(this.bodyA, this.bodyB, dir, aA, aB);
  }

  prepare(dt: number): void {
    const f = this.frame();
    const [t1, t2] = basis(f.axis);
    V.copy(this.t1, t1);
    V.copy(this.t2, t2);
    this.linearRow(this.perp1, t1, f);
    this.linearRow(this.perp2, t2, f);
    this.angular.prepare(this.bodyA, this.bodyB);
    const s = V.dot(f.d, f.axis);
    this.lowerOn = this.upperOn = false;
    if (this.enableLimit) {
      const cl = s - this.lower;
      const cu = this.upper - s;
      this.lowerOn = cl <= 0.1;
      this.upperOn = cu <= 0.1;
      this.lowerBias = cl > 0 ? cl / dt : 0;
      this.upperBias = cu > 0 ? cu / dt : 0;
      if (this.lowerOn) this.linearRow(this.lowerRow, f.axis, f);
      else this.lowerRow.impulse = 0;
      if (this.upperOn) this.linearRow(this.upperRow, V.negate(new Vec3(), f.axis), f);
      else this.upperRow.impulse = 0;
    } else {
      this.lowerRow.impulse = this.upperRow.impulse = 0;
    }
    if (this.enableMotor) this.linearRow(this.motorRow, f.axis, f);
    else this.motorRow.impulse = 0;
  }

  warmStart(): void {
    const a = this.bodyA;
    const b = this.bodyB;
    this.angular.warm(a, b);
    this.perp1.warm(a, b);
    this.perp2.warm(a, b);
    if (this.lowerOn) this.lowerRow.warm(a, b);
    if (this.upperOn) this.upperRow.warm(a, b);
    if (this.enableMotor) this.motorRow.warm(a, b);
  }

  solveVelocity(dt: number): void {
    const a = this.bodyA;
    const b = this.bodyB;
    if (this.enableMotor) {
      const max = this.maxMotorForce * dt;
      this.motorRow.solve(a, b, -this.motorSpeed, -max, max);
    }
    if (this.lowerOn) this.lowerRow.solve(a, b, this.lowerBias, 0, Infinity);
    if (this.upperOn) this.upperRow.solve(a, b, this.upperBias, 0, Infinity);
    this.perp1.solve(a, b, 0, -Infinity, Infinity);
    this.perp2.solve(a, b, 0, -Infinity, Infinity);
    this.angular.solve(a, b);
  }

  solvePosition(): number {
    const a = this.bodyA;
    const b = this.bodyB;
    let worst = this.angular.position(a, b, BETA());
    const f = this.frame();
    const c1 = V.dot(f.d, this.t1);
    const c2 = V.dot(f.d, this.t2);
    this.perp1.position(a, b, c1, BETA());
    this.perp2.position(a, b, c2, BETA());
    worst = Math.max(worst, Math.abs(c1), Math.abs(c2));
    return worst;
  }

  override solveLimits(): number {
    const a = this.bodyA;
    const b = this.bodyB;
    let worst = 0;
    if (this.enableLimit) {
      const g = this.frame();
      const s = V.dot(g.d, g.axis);
      const low = Math.min(0, s - this.lower + Joint.linearSlop);
      const high = Math.min(0, this.upper - s + Joint.linearSlop);
      if (low < 0) {
        this.linearRow(this.lowerRow, g.axis, g);
        this.lowerRow.position(a, b, low, BETA());
      }
      if (high < 0) {
        this.linearRow(this.upperRow, V.negate(new Vec3(), g.axis), g);
        this.upperRow.position(a, b, high, BETA());
      }
      worst = Math.max(worst, -low, -high);
    }
    return worst;
  }
}

export interface DistanceOptions {
  /** Rest length; defaults to the anchor separation at creation. */
  readonly length?: number;
  /** Spring frequency in Hz. Zero or omitted gives a rigid rod. */
  readonly frequency?: number;
  readonly dampingRatio?: number;
}

/** Keeps two anchor points a fixed distance apart (rod), or acts as a damped spring. */
export class DistanceJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  length: number;
  frequency: number;
  dampingRatio: number;
  private readonly row = new Row();
  private bias = 0;

  constructor(
    a: RigidBody,
    b: RigidBody,
    worldAnchorA: Vec3,
    worldAnchorB: Vec3,
    options: DistanceOptions = {},
  ) {
    super(a, b);
    checkPoint(worldAnchorA, 'DistanceJoint anchor A');
    checkPoint(worldAnchorB, 'DistanceJoint anchor B');
    if (options.length !== undefined && !(Number.isFinite(options.length) && options.length >= 0)) {
      throw new RangeError('DistanceJoint length must be finite and non-negative');
    }
    this.localA = localPoint(a, worldAnchorA);
    this.localB = localPoint(b, worldAnchorB);
    this.length = options.length ?? V.distance(worldAnchorA, worldAnchorB);
    this.frequency = options.frequency ?? 0;
    this.dampingRatio = options.dampingRatio ?? 0;
  }

  private geometry() {
    const rA = worldDir(this.bodyA, this.localA);
    const rB = worldDir(this.bodyB, this.localB);
    const d = V.sub(
      new Vec3(),
      V.add(new Vec3(), this.bodyB.position, rB),
      V.add(new Vec3(), this.bodyA.position, rA),
    );
    const len = V.length(d);
    const n = len > 1e-12 ? V.scale(new Vec3(), d, 1 / len) : new Vec3(0, 1, 0);
    return { rA, rB, n, len };
  }

  prepare(dt: number): void {
    const g = this.geometry();
    this.row.setLinear(this.bodyA, this.bodyB, g.rA, g.rB, g.n);
    if (this.frequency > 0) {
      const k0 = this.row.mass > 0 ? 1 / this.row.mass : 0;
      const m = k0 > 0 ? 1 / k0 : 0;
      const omega = 2 * Math.PI * this.frequency;
      const stiffness = m * omega * omega;
      const damping = 2 * m * this.dampingRatio * omega;
      const denom = damping + dt * stiffness;
      const gamma = denom > 0 ? 1 / (dt * denom) : 0;
      this.row.setLinear(this.bodyA, this.bodyB, g.rA, g.rB, g.n, gamma);
      this.bias = denom > 0 ? (dt * stiffness * (g.len - this.length)) / (dt * denom) : 0;
    } else {
      this.bias = 0;
    }
  }

  warmStart(): void {
    this.row.warm(this.bodyA, this.bodyB);
  }

  solveVelocity(): void {
    this.row.solve(this.bodyA, this.bodyB, this.bias, -Infinity, Infinity);
  }

  solvePosition(): number {
    if (this.frequency > 0) return 0;
    const g = this.geometry();
    const c = g.len - this.length;
    this.row.position(this.bodyA, this.bodyB, c, BETA());
    return Math.abs(c);
  }
}

export interface ConeTwistOptions {
  /** Swing half-angle of the cone about the twist axis, radians. */
  readonly swingSpan: number;
  /** Twist range ±twistSpan, radians. */
  readonly twistSpan: number;
}

/**
 * Ball joint with a swing cone and twist range, for shoulders and hips. The twist axis is the
 * given world axis; swing is the angle between the two bodies' axes, twist the rotation about it.
 */
export class ConeTwistJoint extends Joint {
  readonly localA: Vec3;
  readonly localB: Vec3;
  /** Frame orientations (body frame → joint frame); the joint x axis is the twist axis. */
  private readonly frameA: Quat;
  private readonly frameB: Quat;
  swingSpan: number;
  twistSpan: number;

  private readonly point = new PointLock();
  private readonly swingRow = new Row();
  private readonly twistLow = new Row();
  private readonly twistHigh = new Row();
  private swingOn = false;
  private twistLowOn = false;
  private twistHighOn = false;
  private swingBias = 0;
  private lowBias = 0;
  private highBias = 0;

  constructor(
    a: RigidBody,
    b: RigidBody,
    worldAnchor: Vec3,
    worldTwistAxis: Vec3,
    options: ConeTwistOptions,
  ) {
    super(a, b);
    checkPoint(worldAnchor, 'ConeTwistJoint anchor');
    checkAxis(worldTwistAxis, 'ConeTwistJoint axis');
    if (!(options.swingSpan >= 0) || !(options.twistSpan >= 0)) {
      throw new RangeError('ConeTwistJoint swingSpan and twistSpan must be non-negative');
    }
    this.localA = localPoint(a, worldAnchor);
    this.localB = localPoint(b, worldAnchor);
    const x = V.normalize(new Vec3(), worldTwistAxis);
    const jointToWorld = frameFromX(x);
    this.frameA = Q.mul(new Quat(), Q.invert(new Quat(), a.orientation), jointToWorld);
    this.frameB = Q.mul(new Quat(), Q.invert(new Quat(), b.orientation), jointToWorld);
    this.swingSpan = options.swingSpan;
    this.twistSpan = options.twistSpan;
  }

  /** Swing and twist of B's frame relative to A's, plus the world axes involved. */
  private state() {
    const fa = Q.mul(new Quat(), this.bodyA.orientation, this.frameA);
    const fb = Q.mul(new Quat(), this.bodyB.orientation, this.frameB);
    const rel = Q.mul(new Quat(), Q.invert(new Quat(), fa), fb);
    if (rel.w < 0) {
      rel.x = -rel.x;
      rel.y = -rel.y;
      rel.z = -rel.z;
      rel.w = -rel.w;
    }
    const twist = 2 * Math.atan2(rel.x, rel.w);
    // swing = rel · twist⁻¹, a rotation about an axis perpendicular to x.
    const tn = Math.hypot(rel.x, rel.w);
    const tq = new Quat(tn > 0 ? rel.x / tn : 0, 0, 0, tn > 0 ? rel.w / tn : 1);
    const swing = Q.mul(new Quat(), rel, Q.invert(new Quat(), tq));
    const sinHalf = Math.hypot(swing.y, swing.z);
    const swingAngle = 2 * Math.atan2(sinHalf, swing.w);
    const swingAxisLocal =
      sinHalf > 1e-9 ? new Vec3(0, swing.y / sinHalf, swing.z / sinHalf) : new Vec3(0, 1, 0);
    return {
      twist,
      swingAngle,
      swingAxis: Q.rotate(new Vec3(), fa, swingAxisLocal),
      // Midway between the two bodies' twist axes: with a large swing, A's axis alone is not the
      // axis the twist angle changes about, and the limit chatters between swing and twist.
      twistAxis: V.normalize(
        new Vec3(),
        V.add(
          new Vec3(),
          Q.rotate(new Vec3(), fa, new Vec3(1, 0, 0)),
          Q.rotate(new Vec3(), fb, new Vec3(1, 0, 0)),
        ),
      ),
    };
  }

  prepare(dt: number): void {
    const a = this.bodyA;
    const b = this.bodyB;
    this.point.prepare(a, b, this.localA, this.localB);
    const s = this.state();
    const cs = this.swingSpan - s.swingAngle;
    this.swingOn = cs <= 0.1;
    this.swingBias = cs > 0 ? cs / dt : 0;
    if (this.swingOn) this.swingRow.setAngular(a, b, V.negate(new Vec3(), s.swingAxis));
    else this.swingRow.impulse = 0;
    const cl = s.twist + this.twistSpan;
    const ch = this.twistSpan - s.twist;
    this.twistLowOn = cl <= 0.1;
    this.twistHighOn = ch <= 0.1;
    this.lowBias = cl > 0 ? cl / dt : 0;
    this.highBias = ch > 0 ? ch / dt : 0;
    if (this.twistLowOn) this.twistLow.setAngular(a, b, s.twistAxis);
    else this.twistLow.impulse = 0;
    if (this.twistHighOn) this.twistHigh.setAngular(a, b, V.negate(new Vec3(), s.twistAxis));
    else this.twistHigh.impulse = 0;
  }

  warmStart(): void {
    const a = this.bodyA;
    const b = this.bodyB;
    this.point.warm(a, b);
    if (this.swingOn) this.swingRow.warm(a, b);
    if (this.twistLowOn) this.twistLow.warm(a, b);
    if (this.twistHighOn) this.twistHigh.warm(a, b);
  }

  solveVelocity(): void {
    const a = this.bodyA;
    const b = this.bodyB;
    if (this.swingOn) this.swingRow.solve(a, b, this.swingBias, 0, Infinity);
    if (this.twistLowOn) this.twistLow.solve(a, b, this.lowBias, 0, Infinity);
    if (this.twistHighOn) this.twistHigh.solve(a, b, this.highBias, 0, Infinity);
    this.point.solve(a, b);
  }

  solvePosition(): number {
    return this.point.position(this.bodyA, this.bodyB, this.localA, this.localB, BETA());
  }

  override solveLimits(): number {
    const a = this.bodyA;
    const b = this.bodyB;
    const s = this.state();
    const slop = Joint.angularSlop;
    let worst = 0;
    const cs = Math.min(0, this.swingSpan - s.swingAngle + slop);
    if (cs < 0) {
      this.swingRow.setAngular(a, b, V.negate(new Vec3(), s.swingAxis));
      this.swingRow.position(a, b, cs, BETA());
      worst = Math.max(worst, -cs);
    }
    const cl = Math.min(0, s.twist + this.twistSpan + slop);
    if (cl < 0) {
      this.twistLow.setAngular(a, b, s.twistAxis);
      this.twistLow.position(a, b, cl, BETA());
      worst = Math.max(worst, -cl);
    }
    const ch = Math.min(0, this.twistSpan - s.twist + slop);
    if (ch < 0) {
      this.twistHigh.setAngular(a, b, V.negate(new Vec3(), s.twistAxis));
      this.twistHigh.position(a, b, ch, BETA());
      worst = Math.max(worst, -ch);
    }
    return worst;
  }
}

/** Orientation whose local x axis maps to the unit vector `x`. */
function frameFromX(x: Vec3): Quat {
  return Q.fromUnitVectors(new Quat(), new Vec3(1, 0, 0), x);
}
