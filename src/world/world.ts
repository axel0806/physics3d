import { AABB, Quat, Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import * as T from '../math/transform';
import * as Q from '../math/quat';
import { RigidBody, type BodyOptions, type Collider, type ColliderShape } from '../dynamics/body';
import {
  integratePosition,
  integrateOrientation,
  integrateOrientationPostSolve,
  integrateVelocities,
} from '../dynamics/integrate';
import type { ConvexShape } from '../shapes/shape';
import { Sphere, Triangle, TriangleMesh } from '../shapes';
import type { Placed } from '../collision/narrowphase';
import { BroadPhase, pairKey } from '../collision/broadphase';
import { Joint } from '../constraints/joint';
import { manifoldOf, swapManifold } from '../collision/pairs';
import { convexTriangleContact } from '../collision/narrowphase/triangleContact';
import { ContactBatch } from '../solver/batch';
import { gjk } from '../collision/narrowphase';
import type { ContactManifold } from '../collision/narrowphase/contact';
import { timeOfImpact, type TimeOfImpact } from '../collision/toi';
import {
  applyImpulse,
  buildNormalK,
  effectiveMass,
  prepareContact,
  solvePosition,
  solveVelocity,
  tangentBasis,
  warmStart,
  type ContactConstraint,
  type SolverPoint,
} from '../solver/contactSolver';

export interface WorldOptions {
  readonly gravity?: Vec3;
  readonly velocityIterations?: number;
  readonly positionIterations?: number;
  /** Relative normal speed (m/s) above which restitution applies. */
  readonly restitutionThreshold?: number;
  /** Position-correction fraction per iteration and allowed penetration. */
  readonly baumgarte?: number;
  readonly linearSlop?: number;
  /** Seconds a body must stay below the sleep speeds before it sleeps. */
  readonly sleepTime?: number;
  /**
   * How bodies flagged `ccd` avoid tunnelling: 'sweep' finds the time of impact of their motion,
   * 'speculative' adds contacts for shapes that will touch within the step.
   */
  readonly ccdMode?: 'sweep' | 'speculative';
}

export interface ColliderOptions {
  readonly local?: Transform;
  readonly friction?: number;
  readonly restitution?: number;
  readonly layer?: number;
  readonly mask?: number;
}

/** Which colliders a query may report: `mask` is tested against collider layers. */
export interface QueryFilter {
  readonly mask?: number;
  readonly predicate?: (c: Collider) => boolean;
}

export interface RayHit {
  readonly collider: Collider;
  readonly point: Vec3;
  /** Surface normal at the hit, facing the ray origin. */
  readonly normal: Vec3;
  readonly distance: number;
}

export interface ShapeCastHit {
  readonly collider: Collider;
  /** Fraction of the translation travelled at first contact. */
  readonly fraction: number;
  readonly point: Vec3;
  /** From the swept shape toward the collider. */
  readonly normal: Vec3;
}

export interface WorldSnapshot {
  readonly frame: number;
  readonly accumulator: number;
  readonly bodyIds: readonly number[];
  readonly bodies: Float64Array;
  readonly contacts: readonly {
    readonly pair: [number, number];
    readonly sub: number;
    readonly points: readonly { id: number; n: number; t1: number; t2: number }[];
  }[];
  readonly joints: readonly (readonly number[])[];
}

const BODY_STRIDE = 22;
/**
 * Separation at which a query cast counts as touching. Closest-point directions are only as
 * accurate as eps / separation, so the target is not zero: 1e-7 gives normals good to ~1e-8 and
 * reports distances early by at most 1e-7.
 */
const CAST_SKIN = 1e-7;
/** A point-sized sphere used as the probe for rays. */
const RAY_PROBE = new Sphere(1e-9);

/** Hash of the exact bit patterns of a list of doubles (two interleaved 32-bit lanes). */
function hashDoubles(values: readonly number[]): string {
  const f = new Float64Array(1);
  const u = new Uint32Array(f.buffer);
  let h1 = 0x811c9dc5;
  let h2 = 0x9e3779b9;
  for (const v of values) {
    f[0] = v;
    for (const w of u) {
      h1 = Math.imul(h1 ^ w, 0x01000193) >>> 0;
      h2 = Math.imul(h2 ^ ((w >>> 15) | (w << 17)), 0x85ebca6b) >>> 0;
      h2 = (h2 ^ (h2 >>> 13)) >>> 0;
    }
  }
  return h1.toString(16).padStart(8, '0') + h2.toString(16).padStart(8, '0');
}

/** Contact event for listeners: begin, persist, or end, per pair of colliders. */
export interface ContactEvent {
  readonly kind: 'begin' | 'persist' | 'end';
  readonly a: Collider;
  readonly b: Collider;
}

/**
 * Rigid-body world. Fixed-timestep stepping: integrate velocities, warm-start and solve contacts,
 * integrate poses, then run NGS position correction. Contacts persist across steps and reuse their
 * impulses per feature id.
 */
export class World {
  readonly gravity: Vec3;
  readonly bodies: RigidBody[] = [];
  readonly colliders: Collider[] = [];
  readonly joints: Joint[] = [];
  readonly broadphase = new BroadPhase(0.1);
  velocityIterations: number;
  /** Never stop before this many iterations, and stop once no impulse moves a speed more than `velocityTolerance` (m/s). */
  minVelocityIterations = 4;
  velocityTolerance = 1e-5;
  positionIterations: number;
  /** Position passes over contacts (joints use `positionIterations`); fewer calms dense piles. */
  contactPositionIterations = 4;
  restitutionThreshold: number;
  baumgarte: number;
  linearSlop: number;
  sleepTimeLimit: number;
  ccdMode: 'sweep' | 'speculative';
  /** Largest velocity (m/s or rad/s) that one joint position correction may feed back. */
  maxCorrectionSpeed = 1;
  /** Frames stepped so far. */
  frame = 0;
  private readonly contacts = new Map<number, ContactConstraint>();
  /** Contacts with mesh triangles: pair key, then triangle index. */
  private readonly meshContacts = new Map<number, Map<number, ContactConstraint>>();
  private readonly colliderProxy = new Map<Collider, number>();
  private readonly proxyCollider = new Map<number, Collider>();
  private readonly jointedPairs = new Set<number>();
  private nextBodyId = 1;
  /** Fixed-timestep driver state (see `advance`). */
  fixedDt = 1 / 120;
  maxSubSteps = 8;
  private accumulator = 0;
  private readonly listeners: ((e: ContactEvent) => void)[] = [];

  constructor(options: WorldOptions = {}) {
    this.gravity = options.gravity?.clone() ?? new Vec3(0, -9.81, 0);
    this.velocityIterations = options.velocityIterations ?? 40;
    this.positionIterations = options.positionIterations ?? 10;
    this.restitutionThreshold = options.restitutionThreshold ?? 1;
    this.baumgarte = options.baumgarte ?? 0.2;
    this.linearSlop = options.linearSlop ?? 0.0002;
    this.sleepTimeLimit = options.sleepTime ?? 0.5;
    this.ccdMode = options.ccdMode ?? 'sweep';
  }

  createBody(options: BodyOptions = {}): RigidBody {
    if (this.nextBodyId >= 0x4000000) throw new RangeError('too many bodies created');
    const body = new RigidBody(this.nextBodyId++, options);
    this.bodies.push(body);
    return body;
  }

  /** Attaches a shape to a body (mass properties are refreshed automatically). */
  createCollider(body: RigidBody, shape: ColliderShape, options: ColliderOptions = {}): Collider {
    if (!this.bodies.includes(body)) throw new RangeError('body does not belong to this world');
    for (const v of [options.friction, options.restitution]) {
      if (v !== undefined && !(v >= 0))
        throw new RangeError('friction and restitution must be >= 0');
    }
    if (shape instanceof TriangleMesh && body.type !== 'static') {
      throw new RangeError('a triangle mesh collider needs a static body');
    }
    const c: Collider = {
      body,
      shape,
      local: options.local?.clone() ?? new Transform(),
      friction: options.friction ?? 0.5,
      restitution: options.restitution ?? 0,
      layer: options.layer ?? 1,
      mask: options.mask ?? 0xffffffff,
    };
    body.colliders.push(c);
    try {
      body.refreshMassProperties();
    } catch (e) {
      body.colliders.pop();
      body.refreshMassProperties();
      throw e;
    }
    this.colliders.push(c);
    const proxy = this.broadphase.createProxy(this.worldBounds(c));
    this.colliderProxy.set(c, proxy);
    this.proxyCollider.set(proxy, c);
    return c;
  }

  /** Registers a joint. Unless `collideConnected`, its two bodies stop colliding with each other. */
  addJoint<J extends Joint>(joint: J): J {
    if (joint.bodyA === joint.bodyB) throw new RangeError('a joint needs two different bodies');
    if (!this.bodies.includes(joint.bodyA) || !this.bodies.includes(joint.bodyB))
      throw new RangeError('joint bodies must belong to this world');
    this.joints.push(joint);
    joint.bodyA.jointCount++;
    joint.bodyB.jointCount++;
    if (!joint.collideConnected) this.jointedPairs.add(bodyPairKey(joint.bodyA, joint.bodyB));
    return joint;
  }

  removeJoint(joint: Joint): void {
    const i = this.joints.indexOf(joint);
    if (i < 0) return;
    this.joints.splice(i, 1);
    joint.bodyA.jointCount--;
    joint.bodyB.jointCount--;
    this.jointedPairs.clear();
    for (const j of this.joints) {
      if (!j.collideConnected) this.jointedPairs.add(bodyPairKey(j.bodyA, j.bodyB));
    }
  }

  onContact(listener: (e: ContactEvent) => void): void {
    this.listeners.push(listener);
  }

  /**
   * Contacts in a canonical order (by broadphase pair key), so the solve does not depend on the
   * order in which contacts were created. Snapshot/restore and replays then match bit for bit.
   */
  private orderedContacts(): ContactConstraint[] {
    const all: { key: number; c: ContactConstraint }[] = [];
    for (const [key, c] of this.contacts) all.push({ key, c });
    for (const [key, subs] of this.meshContacts)
      for (const c of subs.values()) all.push({ key, c });
    return all.sort((x, y) => x.key - y.key || x.c.sub - y.c.sub).map((e) => e.c);
  }

  private forEachContact(fn: (c: ContactConstraint) => void): void {
    for (const c of this.contacts.values()) fn(c);
    for (const subs of this.meshContacts.values()) for (const c of subs.values()) fn(c);
  }

  /** Current contact constraints (for inspection and tests). */
  activeContacts(): ContactConstraint[] {
    return this.orderedContacts();
  }

  /** Removes a collider: its broadphase proxy and every contact it was part of (with end events). */
  destroyCollider(c: Collider): void {
    const proxy = this.colliderProxy.get(c);
    if (proxy === undefined) return;
    const ended = new Set<number>();
    for (const [key, k] of [...this.contacts.entries()]) {
      if (k.pair[0] !== proxy && k.pair[1] !== proxy) continue;
      this.contacts.delete(key);
      ended.add(key);
    }
    for (const [key, subs] of [...this.meshContacts.entries()]) {
      const any = subs.values().next().value as ContactConstraint | undefined;
      if (!any || (any.pair[0] !== proxy && any.pair[1] !== proxy)) continue;
      this.meshContacts.delete(key);
      ended.add(key);
    }
    for (const key of [...ended].sort((x, y) => x - y)) {
      const lo = Math.floor(key / 0x4000000);
      const hi = key - lo * 0x4000000;
      const first = lo === proxy;
      const other = this.proxyCollider.get(first ? hi : lo);
      if (other) {
        other.body.wake();
        this.emit({ kind: 'end', a: first ? c : other, b: first ? other : c });
      }
    }
    // Bodies resting on this one (their contacts were dropped while asleep) must wake.
    for (const other of this.broadphase.query(this.worldBounds(c))) {
      this.proxyCollider.get(other)?.body.wake();
    }
    this.broadphase.destroyProxy(proxy);
    this.broadphase.updatePairs();
    this.colliderProxy.delete(c);
    this.proxyCollider.delete(proxy);
    this.colliders.splice(this.colliders.indexOf(c), 1);
    c.body.colliders.splice(c.body.colliders.indexOf(c), 1);
    if (c.body.isDynamic && c.body.colliders.length > 0) c.body.refreshMassProperties();
    else if (c.body.isDynamic) c.body.makeMassless();
  }

  /** Removes a body with its colliders and joints; bodies that touched it or were jointed to it wake. */
  destroyBody(body: RigidBody): void {
    const at = this.bodies.indexOf(body);
    if (at < 0) return;
    for (const j of [...this.joints]) {
      if (j.bodyA !== body && j.bodyB !== body) continue;
      (j.bodyA === body ? j.bodyB : j.bodyA).wake();
      this.removeJoint(j);
    }
    for (const c of [...body.colliders]) this.destroyCollider(c);
    this.bodies.splice(this.bodies.indexOf(body), 1);
  }

  removeContactListener(listener: (e: ContactEvent) => void): void {
    const i = this.listeners.indexOf(listener);
    if (i >= 0) this.listeners.splice(i, 1);
  }

  /**
   * Fixed-timestep driver: adds `frameTime` to an accumulator, runs whole steps of `fixedDt`
   * (at most `maxSubSteps`; the rest of a long frame is dropped), and returns the interpolation
   * factor alpha in [0, 1) for rendering between the previous and current pose.
   */
  advance(frameTime: number): number {
    if (!Number.isFinite(frameTime)) throw new RangeError('frameTime must be finite');
    if (!(this.fixedDt > 0)) throw new RangeError('fixedDt must be positive');
    this.accumulator += Math.max(0, frameTime);
    let steps = 0;
    while (this.accumulator >= this.fixedDt && steps < this.maxSubSteps) {
      this.step(this.fixedDt);
      this.accumulator -= this.fixedDt;
      steps++;
    }
    if (this.accumulator >= this.fixedDt) this.accumulator = 0;
    return this.accumulator / this.fixedDt;
  }

  /** Pose of a body blended between the start of the last step and now. */
  interpolatedTransform(
    body: RigidBody,
    alpha: number,
    out: Transform = new Transform(),
  ): Transform {
    V.lerp(out.p, body.prevPosition, body.position, alpha);
    Q.slerp(out.q, body.prevOrientation, body.orientation, alpha);
    return out;
  }

  /** Closest collider hit by the ray, or null. A ray that starts inside a shape does not hit it. */
  raycast(
    origin: Vec3,
    direction: Vec3,
    maxDistance: number,
    filter: QueryFilter = {},
  ): RayHit | null {
    const len = V.length(direction);
    if (!(len > 0) || !(maxDistance > 0)) return null;
    const unit = V.scale(new Vec3(), direction, 1 / len);
    const cast = this.shapeCast(
      RAY_PROBE,
      new Transform(origin.clone()),
      V.scale(new Vec3(), unit, maxDistance),
      filter,
    );
    if (!cast) return null;
    return {
      collider: cast.collider,
      point: cast.point,
      normal: V.negate(new Vec3(), cast.normal),
      distance: cast.fraction * maxDistance,
    };
  }

  /**
   * Sweeps a convex shape along `translation` (rotation fixed) and returns the first collider it
   * touches. `normal` points from the swept shape toward the collider. Shapes already overlapping
   * at the start are ignored.
   */
  shapeCast(
    shape: ConvexShape,
    transform: Transform,
    translation: Vec3,
    filter: QueryFilter = {},
  ): ShapeCastHit | null {
    const start = placedBounds(shape, transform);
    const end = placedBounds(
      shape,
      new Transform(V.add(new Vec3(), transform.p, translation), transform.q),
    );
    const box = BB.union(new AABB(), start, end);
    let best: ShapeCastHit | null = null;
    for (const c of this.candidates(box, filter)) {
      for (const part of this.convexParts(c, box)) {
        const toi = timeOfImpact({ shape, transform }, part, translation, new Vec3(), {
          target: CAST_SKIN,
          tolerance: 0.1 * CAST_SKIN,
          maxIterations: 200,
        });
        if (toi && (!best || toi.t < best.fraction)) {
          best = { collider: c, fraction: toi.t, point: toi.pointB, normal: toi.normal };
        }
      }
    }
    return best;
  }

  /** Colliders whose shape overlaps (or touches) the given shape, in broadphase proxy order. */
  overlap(shape: ConvexShape, transform: Transform, filter: QueryFilter = {}): Collider[] {
    const out: Collider[] = [];
    for (const c of this.candidates(placedBounds(shape, transform), filter)) {
      const box = placedBounds(shape, transform);
      if (this.convexParts(c, box).some((part) => gjk({ shape, transform }, part).overlap)) {
        out.push(c);
      }
    }
    return out;
  }

  /**
   * The convex pieces of a collider near a world-space box: the collider's own shape, or for a
   * mesh the triangles whose bounds overlap the box (as thin convex shapes).
   */
  private convexParts(c: Collider, box: AABB): Placed[] {
    const t = c.body.colliderTransform(c, new Transform());
    const shape = c.shape;
    if (!(shape instanceof TriangleMesh)) return [{ shape, transform: t }];
    const tris: number[] = [];
    shape.bvh.query(toLocalBox(box, t), tris);
    tris.sort((x, y) => x - y);
    const pts: [Vec3, Vec3, Vec3] = [new Vec3(), new Vec3(), new Vec3()];
    return tris.map((i) => {
      shape.triangle(i, pts);
      return { shape: new Triangle(pts[0], pts[1], pts[2]), transform: t };
    });
  }

  private candidates(box: AABB, filter: QueryFilter): Collider[] {
    const out: Collider[] = [];
    for (const proxy of this.broadphase.query(box).sort((a, b) => a - b)) {
      const c = this.proxyCollider.get(proxy);
      if (!c) continue;
      if (filter.mask !== undefined && (filter.mask & c.layer) === 0) continue;
      if (filter.predicate && !filter.predicate(c)) continue;
      out.push(c);
    }
    return out;
  }

  /** Copies the complete simulation state (poses, velocities, sleep, warm-start impulses). */
  snapshot(): WorldSnapshot {
    const bodies = new Float64Array(this.bodies.length * BODY_STRIDE);
    this.bodies.forEach((b, i) => {
      const p = b.position;
      const q = b.orientation;
      const v = b.linearVelocity;
      const w = b.angularVelocity;
      const pp = b.prevPosition;
      const pq = b.prevOrientation;
      // prettier-ignore
      bodies.set([
        p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z,
        b.sleepTime, b.awake ? 1 : 0, pp.x, pp.y, pp.z, pq.x, pq.y, pq.z, pq.w,
      ], i * BODY_STRIDE);
    });
    return {
      frame: this.frame,
      accumulator: this.accumulator,
      bodyIds: this.bodies.map((b) => b.id),
      bodies,
      contacts: this.orderedContacts().map((c) => ({
        pair: [c.pair[0], c.pair[1]] as [number, number],
        sub: c.sub,
        points: c.points.map((p) => ({
          id: p.id,
          n: p.impulseN,
          t1: p.impulseT1,
          t2: p.impulseT2,
        })),
      })),
      joints: this.joints.map((j) => j.saveImpulses()),
    };
  }

  /** Restores a snapshot taken from this world (same bodies, colliders and joints). */
  restore(snap: WorldSnapshot): void {
    if (
      snap.bodyIds.length !== this.bodies.length ||
      snap.joints.length !== this.joints.length ||
      snap.bodyIds.some((id, i) => this.bodies[i]?.id !== id)
    ) {
      throw new Error('snapshot does not match this world (bodies or joints differ)');
    }
    this.bodies.forEach((b, i) => {
      const d = snap.bodies.subarray(i * BODY_STRIDE, (i + 1) * BODY_STRIDE);
      V.set(b.position, d[0] as number, d[1] as number, d[2] as number);
      Q.set(b.orientation, d[3] as number, d[4] as number, d[5] as number, d[6] as number);
      V.set(b.linearVelocity, d[7] as number, d[8] as number, d[9] as number);
      V.set(b.angularVelocity, d[10] as number, d[11] as number, d[12] as number);
      b.sleepTime = d[13] as number;
      b.awake = d[14] === 1;
      V.set(b.prevPosition, d[15] as number, d[16] as number, d[17] as number);
      Q.set(b.prevOrientation, d[18] as number, d[19] as number, d[20] as number, d[21] as number);
      b.updateWorldInertia();
    });
    this.frame = snap.frame;
    this.accumulator = snap.accumulator;
    snap.joints.forEach((data, i) => (this.joints[i] as Joint).loadImpulses(data));
    for (const c of this.colliders) {
      this.broadphase.moveProxy(this.colliderProxy.get(c) as number, this.worldBounds(c));
    }
    this.broadphase.updatePairs();
    this.contacts.clear();
    this.meshContacts.clear();
    for (const sc of snap.contacts) {
      const ca = this.proxyCollider.get(sc.pair[0]);
      const cb = this.proxyCollider.get(sc.pair[1]);
      if (!ca || !cb) continue;
      const zero = () => new Vec3();
      const pk = pairKey(sc.pair[0], sc.pair[1]);
      const put = (c: ContactConstraint): void => {
        if (sc.sub === 0) this.contacts.set(pk, c);
        else {
          let subs = this.meshContacts.get(pk);
          if (!subs) this.meshContacts.set(pk, (subs = new Map()));
          subs.set(sc.sub, c);
        }
      };
      put({
        bodyA: ca.body,
        bodyB: cb.body,
        normal: zero(),
        tangent1: zero(),
        tangent2: zero(),
        friction: 0,
        restitution: 0,
        pair: sc.pair,
        sub: sc.sub,
        normalK: new Float64Array(0),
        points: sc.points.map((p) => ({
          id: p.id,
          localA: zero(),
          localB: zero(),
          rA: zero(),
          rB: zero(),
          gap: 0,
          restitutionTarget: 0,
          normalMass: 0,
          tangentMass1: 0,
          tangentMass2: 0,
          impulseN: p.n,
          impulseT1: p.t1,
          impulseT2: p.t2,
        })),
      });
    }
  }

  /** 16-hex-digit hash of the complete simulation state; equal states hash equal, bit for bit. */
  stateHash(): string {
    const snap = this.snapshot();
    const words: number[] = [snap.frame];
    for (const v of snap.bodies) words.push(v);
    for (const c of snap.contacts) {
      words.push(c.pair[0], c.pair[1], c.sub);
      for (const p of c.points) words.push(p.id, p.n, p.t1, p.t2);
    }
    for (const j of snap.joints) for (const v of j) words.push(v);
    return hashDoubles(words);
  }

  /**
   * Time source for profiling, in milliseconds (the engine never reads a clock itself, which keeps
   * it deterministic; benchmarks inject `performance.now`). When set, `timings` holds the last
   * step's time per phase.
   */
  clock: (() => number) | null = null;
  readonly timings = { broadphase: 0, narrowphase: 0, solve: 0, integrate: 0 };
  private lapTime = 0;
  private readonly batch = new ContactBatch();

  private lap(phase: keyof World['timings']): void {
    if (!this.clock) return;
    const now = this.clock();
    this.timings[phase] += now - this.lapTime;
    this.lapTime = now;
  }

  /** Advances the simulation by dt seconds. */
  step(dt: number): void {
    if (!(dt > 0) || !Number.isFinite(dt)) throw new RangeError('dt must be positive and finite');
    if (this.clock) {
      const t = this.timings;
      t.broadphase = t.narrowphase = t.solve = t.integrate = 0;
      this.lapTime = this.clock();
    }
    for (const body of this.bodies) {
      body.snapshotPose();
      body.snapshotVelocities();
    }
    this.updateBroadphase(dt);
    this.updateContacts(dt);
    for (const j of this.joints) {
      if (j.bodyA.awake !== j.bodyB.awake && j.bodyA.isDynamic && j.bodyB.isDynamic) {
        j.bodyA.wake();
        j.bodyB.wake();
      }
    }
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      integrateVelocities(body, this.gravity, dt);
      if (body.jointCount === 0) integrateOrientation(body, dt);
    }
    this.lap('integrate');
    for (const j of this.joints) {
      if (!j.active) continue;
      j.prepare(dt);
      j.warmStart();
    }
    const ordered = this.orderedContacts();
    // Worlds without joints iterate on flat arrays (ContactBatch). Joints are solved on the
    // bodies directly, interleaved with the contacts, so jointed worlds keep the object solver.
    const batched = this.joints.length === 0;
    for (const c of ordered) {
      if (!this.constraintActive(c)) continue;
      prepareContact(c, this.restitutionThreshold, dt);
      buildNormalK(c);
      if (!batched) warmStart(c);
    }
    if (batched) {
      const batch = this.batch;
      batch.gather(ordered);
      batch.warmStart();
      // A resting pile with warm starting converges in a handful of iterations; the rest would
      // change nothing. Joints do not report convergence, so jointed worlds run every iteration.
      for (let it = 0; it < this.velocityIterations; it++) {
        batch.iterate();
        if (it >= this.minVelocityIterations && batch.maxChange < this.velocityTolerance) break;
      }
      batch.scatter();
    } else {
      for (let it = 0; it < this.velocityIterations; it++) {
        for (const j of this.joints) if (j.active) j.solveVelocity(dt);
        for (const c of ordered) if (this.constraintActive(c)) solveVelocity(c);
      }
    }
    this.lap('solve');
    // Jointed bodies rotate after the solve, from the same pose the joints were evaluated at;
    // free bodies were rotated before it (see integrateOrientation).
    for (const body of this.bodies)
      if (body.jointCount > 0) integrateOrientationPostSolve(body, dt);
    for (const body of this.bodies) if (body.type === 'kinematic') this.moveKinematic(body, dt);
    // CCD bodies move last, against the final poses of everything else.
    for (const body of this.bodies) if (!body.ccd) integratePosition(body, dt);
    for (const body of this.bodies) {
      if (!body.ccd) continue;
      if (this.ccdMode === 'sweep' && this.needsSweep(body, dt)) this.sweepBody(body, dt);
      else integratePosition(body, dt);
    }
    this.lap('integrate');
    const feedback = new Map<RigidBody, [Vec3, Vec3]>();
    for (let it = 0; it < this.positionIterations; it++) {
      if (it < this.contactPositionIterations) {
        for (const c of ordered) {
          if (this.constraintActive(c)) solvePosition(c, this.baumgarte, this.linearSlop);
        }
      }
      for (const j of this.joints) if (j.active) j.solveLimits();
      for (const j of this.joints) if (j.active) this.correctJoint(j, feedback);
    }
    this.applyFeedback(feedback, dt);
    this.lap('solve');
    for (const body of this.bodies) if (body.isSimulated) body.updateWorldInertia();
    this.updateSleep(dt);
    this.lap('integrate');
    this.frame++;
  }

  /** Kinematic bodies follow their velocities exactly; nothing pushes them. */
  private moveKinematic(body: RigidBody, dt: number): void {
    V.addScaled(body.position, body.position, body.linearVelocity, dt);
    const w = body.angularVelocity;
    const len = V.length(w);
    if (len > 0) {
      const dq = Q.fromAxisAngle(new Quat(), V.scale(new Vec3(), w, 1 / len), len * dt);
      Q.mul(body.orientation, dq, body.orientation);
      Q.normalize(body.orientation, body.orientation);
    }
  }

  /** True when the body would move far enough this step (relative to its thinnest collider) to skip over thin geometry. */
  private needsSweep(body: RigidBody, dt: number): boolean {
    if (!body.isSimulated) return false;
    const travel = V.length(body.linearVelocity) * dt;
    let core = Infinity;
    const box = new AABB();
    for (const c of body.colliders) {
      c.shape.localBounds(box);
      core = Math.min(core, box.max.x - box.min.x, box.max.y - box.min.y, box.max.z - box.min.z);
    }
    return travel > 0.25 * core;
  }

  /**
   * Moves a CCD body through one step, stopping at each impact found by conservative advancement:
   * at the time of impact the approach velocity is removed (with restitution), and the rest of the
   * step continues with the new velocity, up to a few events. Other bodies count as stationary and
   * rotation during the sweep is ignored (the contact solver handles rotation).
   */
  private sweepBody(body: RigidBody, dt: number): void {
    let remaining = 1;
    for (let event = 0; event < 4 && remaining > 1e-6; event++) {
      const d = V.scale(new Vec3(), body.linearVelocity, dt * remaining);
      let best: { toi: TimeOfImpact; ca: Collider; cb: Collider } | null = null;
      const swept = new AABB();
      BB.empty(swept);
      for (const ca of body.colliders) {
        const start = this.worldBounds(ca);
        const end = new AABB();
        BB.set(
          end,
          start.min.x + d.x,
          start.min.y + d.y,
          start.min.z + d.z,
          start.max.x + d.x,
          start.max.y + d.y,
          start.max.z + d.z,
        );
        BB.union(swept, start, end);
        const ta = body.colliderTransform(ca, new Transform());
        for (const proxy of this.broadphase.query(swept)) {
          const cb = this.proxyCollider.get(proxy);
          if (!cb || !this.shouldCollide(ca, cb)) continue;
          for (const part of this.convexParts(cb, swept)) {
            const toi = timeOfImpact(
              { shape: ca.shape as ConvexShape, transform: ta },
              part,
              d,
              new Vec3(),
              { target: this.linearSlop, tolerance: 0.25 * this.linearSlop },
            );
            if (toi && (!best || toi.t < best.toi.t)) best = { toi, ca, cb };
          }
        }
      }
      if (!best) {
        V.add(body.position, body.position, d);
        return;
      }
      V.addScaled(body.position, body.position, d, best.toi.t);
      this.resolveImpact(best.ca, best.cb, best.toi);
      remaining *= 1 - best.toi.t;
    }
  }

  /** Impulse at a time of impact: removes the approach speed along the normal, with restitution. */
  private resolveImpact(ca: Collider, cb: Collider, toi: TimeOfImpact): void {
    const a = ca.body;
    const b = cb.body;
    // Wake first: impulses are ignored on sleeping bodies.
    a.wake();
    b.wake();
    const rA = V.sub(new Vec3(), toi.pointA, a.position);
    const rB = V.sub(new Vec3(), toi.pointB, b.position);
    const vn = V.dot(
      V.sub(new Vec3(), b.velocityAt(rB, new Vec3()), a.velocityAt(rA, new Vec3())),
      toi.normal,
    );
    if (vn >= 0) return;
    const e = -vn > this.restitutionThreshold ? Math.max(ca.restitution, cb.restitution) : 0;
    const j = (-(1 + e) * vn) / effectiveMass(a, b, rA, rB, toi.normal);
    const P = V.scale(new Vec3(), toi.normal, j);
    applyImpulse(a, V.negate(new Vec3(), P), rA);
    applyImpulse(b, P, rB);
    b.wake();
  }

  /**
   * One NGS pass over a joint's equality locks. The displacement it applies is accumulated per
   * body and fed back into the velocities once per step (as in SHAKE): the position step moved the
   * body along the tangent, the correction is the missing centripetal part, and without it every
   * step dissipates energy at first order in dt. The total is capped per body so a badly violated
   * joint is pulled in gently rather than kicked.
   */
  private correctJoint(j: Joint, feedback: Map<RigidBody, [Vec3, Vec3]>): void {
    const a = j.bodyA;
    const b = j.bodyB;
    const pa = V.copy(new Vec3(), a.position);
    const pb = V.copy(new Vec3(), b.position);
    const qa = a.orientation.clone();
    const qb = b.orientation.clone();
    j.solvePosition();
    this.accumulate(a, pa, qa, feedback);
    this.accumulate(b, pb, qb, feedback);
  }

  private accumulate(
    body: RigidBody,
    p0: Vec3,
    q0: Quat,
    feedback: Map<RigidBody, [Vec3, Vec3]>,
  ): void {
    if (!body.isSimulated) return;
    let acc = feedback.get(body);
    if (!acc) {
      acc = [new Vec3(), new Vec3()];
      feedback.set(body, acc);
    }
    V.add(acc[0], acc[0], V.sub(new Vec3(), body.position, p0));
    // World-frame rotation vector taking q0 to the current orientation.
    const dq = Q.mul(new Quat(), body.orientation, Q.invert(new Quat(), q0));
    const k = dq.w < 0 ? -2 : 2;
    V.add(acc[1], acc[1], new Vec3(dq.x * k, dq.y * k, dq.z * k));
  }

  private applyFeedback(feedback: Map<RigidBody, [Vec3, Vec3]>, dt: number): void {
    // Bodies in contact are skipped: there the joint displacement is tangled with contact pushes,
    // and feeding it back pumps energy into limbs lying on surfaces (measured: 16 m/s forearm
    // whips on a staircase). Free-flying jointed bodies are where the dissipation matters.
    const touching = new Set<RigidBody>();
    this.forEachContact((c) => {
      touching.add(c.bodyA);
      touching.add(c.bodyB);
    });
    for (const [body, [dx, dtheta]] of feedback) {
      if (touching.has(body)) continue;
      for (const [target, delta] of [
        [body.linearVelocity, dx],
        [body.angularVelocity, dtheta],
      ] as const) {
        const dv = V.scale(new Vec3(), delta, 1 / dt);
        const len = V.length(dv);
        if (len > this.maxCorrectionSpeed) V.scale(dv, dv, this.maxCorrectionSpeed / len);
        V.add(target, target, dv);
      }
    }
  }

  private constraintActive(c: ContactConstraint): boolean {
    return c.bodyA.isSimulated || c.bodyB.isSimulated;
  }

  /** World-space bounding box of a collider at its current pose (for debug drawing and queries). */
  colliderBounds(c: Collider): AABB {
    return this.worldBounds(c);
  }

  private worldBounds(c: Collider): AABB {
    return placedBounds(c.shape, c.body.colliderTransform(c, new Transform()));
  }

  private updateBroadphase(dt: number): void {
    for (const c of this.colliders) {
      // Static and sleeping bodies do not move; their proxies stay as they are.
      if (!c.body.isSimulated && c.body.type !== 'kinematic') continue;
      const proxy = this.colliderProxy.get(c);
      if (proxy === undefined) continue;
      const box = this.worldBounds(c);
      const v = c.body.linearVelocity;
      if (c.body.ccd && this.ccdMode === 'speculative' && c.body.isSimulated) {
        // Speculative contacts need pairs for everything within this step's reach.
        BB.expandPoint(box, V.add(new Vec3(), box.min, new Vec3(v.x * dt, v.y * dt, v.z * dt)));
        BB.expandPoint(box, V.add(new Vec3(), box.max, new Vec3(v.x * dt, v.y * dt, v.z * dt)));
      }
      this.broadphase.moveProxy(proxy, box, v.x * dt, v.y * dt, v.z * dt);
    }
  }

  /** Re-runs the pair manifolds for overlapping collider pairs, keeping impulses by feature id. */
  private updateContacts(dt: number): void {
    const changes = this.broadphase.updatePairs();
    this.lap('broadphase');
    for (const ch of changes) {
      if (ch.kind !== 'end') continue;
      const ca = this.proxyCollider.get(ch.a);
      const cb = this.proxyCollider.get(ch.b);
      if (!ca || !cb) continue;
      const key = pairKey(ch.a, ch.b);
      const had = this.contacts.has(key) || this.meshContacts.has(key);
      this.dropPair(key);
      if (had) this.emit({ kind: 'end', a: ca, b: cb });
    }
    // Refresh manifolds for every overlapping pair.
    const wakesBefore = this.pairWakes;
    for (const [key, pair] of this.currentPairs()) {
      const [ia, ib] = pair;
      const ca = this.proxyCollider.get(ia);
      const cb = this.proxyCollider.get(ib);
      if (!ca || !cb) continue;
      if (!this.shouldCollide(ca, cb)) {
        this.skippedPairs.push(key, ia, ib);
        continue;
      }
      this.refreshPair(key, ca, cb, dt);
    }
    // A pair of sleepers is judged again once the pass above has woken whatever it touches, so a
    // body woken by a later pair does not miss its contacts for a step.
    const skipped = this.skippedPairs;
    const recheck = this.pairWakes !== wakesBefore;
    for (let i = 0; i < skipped.length; i += 3) {
      const key = skipped[i] as number;
      const ca = recheck ? this.proxyCollider.get(skipped[i + 1] as number) : undefined;
      const cb = recheck ? this.proxyCollider.get(skipped[i + 2] as number) : undefined;
      if (ca && cb && this.shouldCollide(ca, cb)) this.refreshPair(key, ca, cb, dt);
      else this.dropPair(key);
    }
    skipped.length = 0;
    this.lap('narrowphase');
  }

  /** Scratch for `updateContacts`: (pair key, proxy a, proxy b) triples that were not collided yet. */
  private readonly skippedPairs: number[] = [];
  private pairWakes = 0;

  private currentPairs(): [number, [number, number]][] {
    return this.broadphase.pairList().map(([a, b]) => [pairKey(a, b), [a, b]]);
  }

  private shouldCollide(a: Collider, b: Collider): boolean {
    if (a.body === b.body) return false;
    if (this.jointedPairs.has(bodyPairKey(a.body, b.body))) return false;
    if (!(a.body.isSimulated || b.body.isSimulated)) {
      // A moving kinematic body must still reach (and wake) a sleeping dynamic one.
      if (!(movesOnto(a.body, b.body) || movesOnto(b.body, a.body))) return false;
    }
    if ((a.mask & b.layer) === 0 || (b.mask & a.layer) === 0) return false;
    return true;
  }

  private dropPair(key: number): void {
    if (this.contacts.size > 0) this.contacts.delete(key);
    if (this.meshContacts.size > 0) this.meshContacts.delete(key);
  }

  /** Contacts between a static triangle mesh and a convex collider: one constraint per triangle. */
  private refreshMeshPair(key: number, ca: Collider, cb: Collider): void {
    const meshIsA = ca.shape instanceof TriangleMesh;
    const cm = meshIsA ? ca : cb;
    const co = meshIsA ? cb : ca;
    const mesh = cm.shape as TriangleMesh;
    if (co.shape instanceof TriangleMesh) return;
    const tm = cm.body.colliderTransform(cm, new Transform());
    const to = co.body.colliderTransform(co, new Transform());
    const tris: number[] = [];
    mesh.bvh.query(toLocalBox(placedBounds(co.shape, to), tm), tris);
    tris.sort((x, y) => x - y);
    const subs = this.meshContacts.get(key);
    const next = new Map<number, ContactConstraint>();
    const pts: [Vec3, Vec3, Vec3] = [new Vec3(), new Vec3(), new Vec3()];
    const nTri = new Vec3();
    for (const tri of tris) {
      mesh.triangle(tri, pts);
      const e1 = V.sub(new Vec3(), pts[1], pts[0]);
      const e2 = V.sub(new Vec3(), pts[2], pts[0]);
      V.cross(nTri, e1, e2);
      const area2 = V.length(nTri);
      if (!(area2 > 1e-12)) continue; // degenerate triangle
      V.scale(nTri, nTri, 1 / area2);
      Q.rotate(nTri, tm.q, nTri);
      const shapeO = { shape: co.shape, transform: to };
      // Fast path: face-normal contact from the shape's lowest points over this triangle.
      const fast = convexTriangleContact(
        shapeO,
        T.apply(new Vec3(), tm, pts[0]),
        T.apply(new Vec3(), tm, pts[1]),
        T.apply(new Vec3(), tm, pts[2]),
        0.5 * this.linearSlop,
      );
      if (fast === 'clear') continue;
      let m: ContactManifold | null;
      if (fast !== 'unsure') {
        m = meshIsA ? fast : swapManifold(fast);
      } else {
        // The shape overlaps the plane without a point over the triangle: an edge or vertex
        // contact. GJK/EPA decides; sideways normals belong to the neighbouring face and are
        // dropped rather than allowed to snag a sliding body.
        const shapeM = { shape: new Triangle(pts[0], pts[1], pts[2]), transform: tm };
        m = meshIsA ? manifoldOf(shapeM, shapeO) : manifoldOf(shapeO, shapeM);
        if (!m || Math.abs(V.dot(m.normal, nTri)) < 0.5) continue;
      }
      const old = subs?.get(tri + 1);
      const c = this.makeConstraint(ca, cb, m, old, tri + 1);
      next.set(tri + 1, c);
      if (!old && (ca.body.awake || cb.body.awake)) this.wakePair(ca.body, cb.body);
    }
    const had = subs !== undefined && subs.size > 0;
    if (next.size > 0) this.meshContacts.set(key, next);
    else this.meshContacts.delete(key);
    if (next.size > 0) this.emit({ kind: had ? 'persist' : 'begin', a: ca, b: cb });
    else if (had) this.emit({ kind: 'end', a: ca, b: cb });
  }

  private refreshPair(key: number, ca: Collider, cb: Collider, dt: number): void {
    if (ca.shape instanceof TriangleMesh || cb.shape instanceof TriangleMesh) {
      this.refreshMeshPair(key, ca, cb);
      return;
    }
    const ta = ca.body.colliderTransform(ca, new Transform());
    const tb = cb.body.colliderTransform(cb, new Transform());
    const placedA = { shape: ca.shape, transform: ta };
    const placedB = { shape: cb.shape, transform: tb };
    const m =
      manifoldOf(placedA, placedB) ??
      (this.ccdMode === 'speculative' ? this.speculative(ca, cb, placedA, placedB, dt) : null);
    const old = this.contacts.get(key);
    if (!m) {
      if (old) {
        this.contacts.delete(key);
        this.emit({ kind: 'end', a: ca, b: cb });
      }
      return;
    }
    const c = this.makeConstraint(ca, cb, m, old, 0);
    if (!old) this.emit({ kind: 'begin', a: ca, b: cb });
    else this.emit({ kind: 'persist', a: ca, b: cb });
    // A listener may have destroyed either collider; do not resurrect a contact for it.
    if (!this.colliderProxy.has(ca) || !this.colliderProxy.has(cb)) return;
    this.contacts.set(key, c);
    // A new contact wakes an island that touches an awake body. Persisting contacts do not wake
    // anything, so resting stacks can accumulate sleep time.
    if (!old && (ca.body.awake || cb.body.awake)) this.wakePair(ca.body, cb.body);
  }

  /** Wakes both bodies of a new contact, counting sleepers that woke (see `updateContacts`). */
  private wakePair(a: RigidBody, b: RigidBody): void {
    if (a.isDynamic && !a.awake) this.pairWakes++;
    if (b.isDynamic && !b.awake) this.pairWakes++;
    a.wake();
    b.wake();
  }

  /** Builds a solver constraint from a manifold, carrying impulses over from `old` by feature id. */
  private makeConstraint(
    ca: Collider,
    cb: Collider,
    m: ContactManifold,
    old: ContactConstraint | undefined,
    sub: number,
  ): ContactConstraint {
    const [t1, t2] = tangentBasis(m.normal);
    const points: SolverPoint[] = m.points.map((p) => {
      const prev = old?.points.find((o) => o.id === p.id);
      return {
        id: p.id,
        localA: localAnchor(ca.body, p.pointA),
        localB: localAnchor(cb.body, p.pointB),
        rA: new Vec3(),
        rB: new Vec3(),
        gap: Math.max(0, -p.depth),
        restitutionTarget: 0,
        normalMass: 0,
        tangentMass1: 0,
        tangentMass2: 0,
        impulseN: prev?.impulseN ?? 0,
        impulseT1: prev?.impulseT1 ?? 0,
        impulseT2: prev?.impulseT2 ?? 0,
      };
    });
    return {
      bodyA: ca.body,
      bodyB: cb.body,
      normal: V.copy(new Vec3(), m.normal),
      tangent1: t1,
      tangent2: t2,
      friction: Math.sqrt(ca.friction * cb.friction),
      restitution: Math.max(ca.restitution, cb.restitution),
      points,
      pair: [this.colliderProxy.get(ca) as number, this.colliderProxy.get(cb) as number],
      sub,
      normalK: new Float64Array(0),
    };
  }

  /**
   * Speculative contact for a CCD body: when the shapes are apart but will close the gap within
   * this step at their current speeds, a one-point manifold with negative depth lets the solver
   * stop the approach exactly at touching.
   */
  private speculative(
    ca: Collider,
    cb: Collider,
    A: { shape: ConvexShape; transform: Transform },
    B: { shape: ConvexShape; transform: Transform },
    dt: number,
  ): ContactManifold | null {
    if (!(ca.body.ccd || cb.body.ccd)) return null;
    const reach = (V.length(ca.body.linearVelocity) + V.length(cb.body.linearVelocity)) * dt;
    const g = gjk(A, B);
    if (g.overlap || !(g.distance <= reach)) return null;
    const normal = V.sub(new Vec3(), g.pointB, g.pointA);
    V.scale(normal, normal, 1 / g.distance);
    const closing = V.dot(
      V.sub(
        new Vec3(),
        ca.body.velocityAt(V.sub(new Vec3(), g.pointA, ca.body.position), new Vec3()),
        cb.body.velocityAt(V.sub(new Vec3(), g.pointB, cb.body.position), new Vec3()),
      ),
      normal,
    );
    if (closing * dt < g.distance - this.linearSlop) return null;
    const position = V.lerp(new Vec3(), g.pointA, g.pointB, 0.5);
    return {
      normal,
      points: [{ position, depth: -g.distance, id: -1, pointA: g.pointA, pointB: g.pointB }],
    };
  }

  private emit(e: ContactEvent): void {
    // Iterate a copy: a listener may remove itself or others.
    for (const l of [...this.listeners]) l(e);
  }

  /** Puts islands of slow bodies to sleep, using union-find over active contacts. */
  private updateSleep(dt: number): void {
    // Union-find over the bodies, with the parent links stored on the bodies themselves.
    const find = (b: RigidBody): RigidBody => {
      let r = b;
      while (r.islandParent !== r) {
        r.islandParent = r.islandParent.islandParent;
        r = r.islandParent;
      }
      return r;
    };
    for (const body of this.bodies) {
      body.islandParent = body;
      body.islandMin = Infinity;
    }
    // Islands connect through dynamic bodies only: a shared static floor or anchor must not merge
    // unrelated groups.
    const link = (a: RigidBody, b: RigidBody): void => {
      if (!a.isDynamic || !b.isDynamic) return;
      const ra = find(a);
      const rb = find(b);
      if (ra !== rb) ra.islandParent = rb;
    };
    this.forEachContact((c) => link(c.bodyA, c.bodyB));
    for (const j of this.joints) link(j.bodyA, j.bodyB);
    for (const body of this.bodies) {
      if (!body.isDynamic) continue;
      const slow =
        V.lengthSq(body.linearVelocity) < 1e-4 && V.lengthSq(body.angularVelocity) < 1e-4;
      body.sleepTime = slow ? body.sleepTime + dt : 0;
      const root = find(body);
      root.islandMin = Math.min(root.islandMin, body.sleepTime);
    }
    for (const body of this.bodies) {
      if (!body.isDynamic || !body.awake) continue;
      const ready = find(body).islandMin >= this.sleepTimeLimit;
      if (ready) {
        body.awake = false;
        body.linearVelocity.x = body.linearVelocity.y = body.linearVelocity.z = 0;
        body.angularVelocity.x = body.angularVelocity.y = body.angularVelocity.z = 0;
      }
    }
  }
}

/** Bounding box, in the frame of transform `t`, of a world-space box. */
function toLocalBox(box: AABB, t: Transform): AABB {
  const out = BB.empty(new AABB());
  for (const x of [box.min.x, box.max.x]) {
    for (const y of [box.min.y, box.max.y]) {
      for (const z of [box.min.z, box.max.z]) {
        BB.expandPoint(out, T.applyInverse(new Vec3(), t, new Vec3(x, y, z)));
      }
    }
  }
  return out;
}

/** World-space bounding box of a convex shape placed at `t`. */
function placedBounds(shape: ColliderShape, t: Transform): AABB {
  const local = shape.localBounds(new AABB());
  const out = BB.empty(new AABB());
  for (const x of [local.min.x, local.max.x]) {
    for (const y of [local.min.y, local.max.y]) {
      for (const z of [local.min.z, local.max.z]) {
        BB.expandPoint(out, T.apply(new Vec3(), t, new Vec3(x, y, z)));
      }
    }
  }
  return out;
}

function movesOnto(x: RigidBody, y: RigidBody): boolean {
  return (
    x.type === 'kinematic' &&
    y.isDynamic &&
    (V.lengthSq(x.linearVelocity) > 0 || V.lengthSq(x.angularVelocity) > 0)
  );
}

function bodyPairKey(a: RigidBody, b: RigidBody): number {
  return a.id < b.id ? a.id * 0x4000000 + b.id : b.id * 0x4000000 + a.id;
}

/** Anchor of a world point in a body's frame: the inverse rotation of (point − center of mass). */
function localAnchor(body: RigidBody, world: Vec3): Vec3 {
  const d = V.sub(new Vec3(), world, body.position);
  return Q.rotateInverse(new Vec3(), body.orientation, d);
}
