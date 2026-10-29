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

// Step-loop temporaries (PLAN rule 8: hot paths do not allocate). The engine is single-threaded and
// none of these is live across a call that could reuse it.
const S_PA = new Vec3();
const S_PB = new Vec3();
const S_QA = new Quat();
const S_QB = new Quat();
const S_DELTA = new Vec3();
const S_DQ = new Quat();
const S_DQI = new Quat();
const S_AXIS = new Vec3();
const S_DV = new Vec3();
const S_OT1 = new Vec3();
const S_OT2 = new Vec3();
const S_BOX = new AABB();
const S_WB = new AABB();
const S_TC = new Transform();
const S_TA = new Transform();
const S_TB = new Transform();
const S_TM = new Transform();
const S_PT = new Vec3();
const S_PT2 = new Vec3();
const S_SPEC = new Vec3();
const S_LB = new AABB();
const S_PLACED_A: { shape: ConvexShape; transform: Transform } = {
  shape: RAY_PROBE,
  transform: S_TA,
};
const S_PLACED_B: { shape: ConvexShape; transform: Transform } = {
  shape: RAY_PROBE,
  transform: S_TB,
};
const S_MESH_PTS: [Vec3, Vec3, Vec3] = [new Vec3(), new Vec3(), new Vec3()];
const S_E1 = new Vec3();
const S_E2 = new Vec3();
const S_NTRI = new Vec3();
const S_W0 = new Vec3();
const S_W1 = new Vec3();
const S_W2 = new Vec3();
/** Previous impulses of a constraint's points, indexed like the new points (see `makeConstraint`). */
let prevN = new Float64Array(16);
let prevT1 = new Float64Array(16);
let prevT2 = new Float64Array(16);
/** Mutable view of a constraint, for reusing last step's object in place. */
type Mutable<T> = { -readonly [K in keyof T]: T[K] };

interface ContactEntry {
  key: number;
  c: ContactConstraint;
}
function compareEntries(x: ContactEntry, y: ContactEntry): number {
  return x.key - y.key || x.c.sub - y.c.sub;
}

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
  /**
   * Penetration that contact position correction leaves alone (m). Below the residual of a
   * loaded heap, correcting makes bodies trade tiny hops with their neighbours forever; see
   * docs/ARCHITECTURE.md (Sleeping).
   */
  contactSlop = 0.005;
  /** Largest push-out (m) one contact point applies per position iteration. */
  maxContactCorrection = Infinity;
  /** Box-box contacts are kept this far before touching, so a resting box keeps all its corners. */
  contactMargin = 0.002;
  sleepTimeLimit: number;
  /** A body counts as at rest below these speeds (m/s, rad/s). */
  /** A sleeper wakes when a neighbour moves faster than this multiple of the sleep speeds. */
  wakeSpeedFactor = 3;
  sleepLinearSpeed = 0.04;
  sleepAngularSpeed = 0.15;
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
  /** Copy-on-write: `emit` iterates the array it started with, so listeners may (un)subscribe freely. */
  private listeners: readonly ((e: ContactEvent) => void)[] = [];

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

  /** Subscribes to contact events. Returns a function that unsubscribes the listener again. */
  onContact(listener: (e: ContactEvent) => void): () => void {
    this.listeners = [...this.listeners, listener];
    return () => this.removeContactListener(listener);
  }

  /**
   * Contacts in a canonical order (by broadphase pair key), so the solve does not depend on the
   * order in which contacts were created. Snapshot/restore and replays then match bit for bit.
   */
  private orderedContacts(): ContactConstraint[] {
    const entries = this.orderEntries;
    entries.length = 0;
    this.contacts.forEach(this.collectPlain);
    if (this.meshContacts.size > 0) this.meshContacts.forEach(this.collectMeshGroup);
    entries.sort(compareEntries);
    const out = this.orderOut;
    out.length = entries.length;
    for (let i = 0; i < entries.length; i++) out[i] = (entries[i] as ContactEntry).c;
    return out;
  }

  /** Scratch for `orderedContacts`: pooled sort entries, the entry list, and the shared result. */
  private readonly entryPool: ContactEntry[] = [];
  private readonly orderEntries: ContactEntry[] = [];
  private readonly orderOut: ContactConstraint[] = [];
  private collectKey = 0;
  /** Pairs that emitted `begin` and not yet `end`, for `restore`. */
  private readonly begun = new Map<number, { a: Collider; b: Collider }>();

  private pushEntry(key: number, c: ContactConstraint): void {
    const entries = this.orderEntries;
    let e = this.entryPool[entries.length];
    if (e) {
      e.key = key;
      e.c = c;
    } else {
      e = { key, c };
      this.entryPool.push(e);
    }
    entries.push(e);
  }
  private readonly collectPlain = (c: ContactConstraint, key: number): void =>
    this.pushEntry(key, c);
  private readonly collectMeshSub = (c: ContactConstraint): void =>
    this.pushEntry(this.collectKey, c);
  private readonly collectMeshGroup = (subs: Map<number, ContactConstraint>, key: number): void => {
    this.collectKey = key;
    subs.forEach(this.collectMeshSub);
  };

  /** Current contact constraints (for inspection and tests). */
  activeContacts(): ContactConstraint[] {
    return this.orderedContacts().slice();
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
    if (i < 0) return;
    const next = this.listeners.slice();
    next.splice(i, 1);
    this.listeners = next;
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
    // Pairs that began and never ended (including sleeping ones, whose contacts were dropped
    // silently) but are not in the snapshot have ended.
    const gone: { a: Collider; b: Collider; key: number }[] = [];
    for (const [key, pr] of this.begun) {
      if (!this.contacts.has(key) && !this.meshContacts.has(key)) gone.push({ ...pr, key });
    }
    for (const g of gone) if (this.begun.has(g.key)) this.emit({ kind: 'end', a: g.a, b: g.b });
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
    if (!batched) {
      for (const c of ordered) {
        if (!this.constraintActive(c)) continue;
        prepareContact(c, this.restitutionThreshold, dt);
        buildNormalK(c);
        warmStart(c);
      }
    }
    if (batched) {
      const batch = this.batch;
      batch.prepare(ordered, this.restitutionThreshold, dt);
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
    const stamp = ++this.stepStamp;
    this.feedbackBodies.length = 0;
    // Contact position passes run on the batch arrays (poses are unchanged by them except positions).
    if (batched) {
      const passes = Math.min(this.contactPositionIterations, this.positionIterations);
      this.batch.solvePositions(
        passes,
        this.baumgarte,
        this.contactSlop,
        this.maxContactCorrection,
      );
    }
    for (let it = 0; it < this.positionIterations; it++) {
      if (it < this.contactPositionIterations && !batched) {
        for (const c of ordered) {
          if (this.constraintActive(c))
            solvePosition(c, this.baumgarte, this.contactSlop, this.maxContactCorrection);
        }
      }
      for (const j of this.joints) if (j.active) j.solveLimits();
      for (const j of this.joints) if (j.active) this.correctJoint(j, stamp);
    }
    this.applyFeedback(ordered, stamp, dt);
    this.lap('solve');
    for (const body of this.bodies) if (body.isSimulated) body.updateWorldInertia();
    this.updateSleep(ordered, dt);
    this.lap('integrate');
    this.frame++;
  }

  /** Kinematic bodies follow their velocities exactly; nothing pushes them. */
  private moveKinematic(body: RigidBody, dt: number): void {
    V.addScaled(body.position, body.position, body.linearVelocity, dt);
    const w = body.angularVelocity;
    const len = V.length(w);
    if (len > 0) {
      Q.fromAxisAngle(S_DQ, V.scale(S_AXIS, w, 1 / len), len * dt);
      Q.mul(body.orientation, S_DQ, body.orientation);
      Q.normalize(body.orientation, body.orientation);
    }
  }

  /** True when the body would move far enough this step (relative to its thinnest collider) to skip over thin geometry. */
  private needsSweep(body: RigidBody, dt: number): boolean {
    if (!body.isSimulated) return false;
    const travel = V.length(body.linearVelocity) * dt;
    let core = Infinity;
    const box = S_BOX;
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
  private correctJoint(j: Joint, stamp: number): void {
    const a = j.bodyA;
    const b = j.bodyB;
    V.copy(S_PA, a.position);
    V.copy(S_PB, b.position);
    Q.copy(S_QA, a.orientation);
    Q.copy(S_QB, b.orientation);
    j.solvePosition();
    this.accumulate(a, S_PA, S_QA, stamp);
    this.accumulate(b, S_PB, S_QB, stamp);
  }

  /** Step counter that stamps per-body scratch (joint feedback, contact-touch marks). */
  private stepStamp = 0;
  /** Bodies with joint feedback this step, in first-touch order. */
  private readonly feedbackBodies: RigidBody[] = [];

  private accumulate(body: RigidBody, p0: Vec3, q0: Quat, stamp: number): void {
    if (!body.isSimulated) return;
    if (body.feedbackStamp !== stamp) {
      body.feedbackStamp = stamp;
      V.set(body.feedbackLinear, 0, 0, 0);
      V.set(body.feedbackAngular, 0, 0, 0);
      this.feedbackBodies.push(body);
    }
    V.add(body.feedbackLinear, body.feedbackLinear, V.sub(S_DELTA, body.position, p0));
    // World-frame rotation vector taking q0 to the current orientation.
    const dq = Q.mul(S_DQ, body.orientation, Q.invert(S_DQI, q0));
    const k = dq.w < 0 ? -2 : 2;
    const acc = body.feedbackAngular;
    acc.x = acc.x + dq.x * k;
    acc.y = acc.y + dq.y * k;
    acc.z = acc.z + dq.z * k;
  }

  private applyFeedback(contacts: readonly ContactConstraint[], stamp: number, dt: number): void {
    const bodies = this.feedbackBodies;
    if (bodies.length === 0) return;
    // Bodies in contact are skipped: there the joint displacement is tangled with contact pushes,
    // and feeding it back pumps energy into limbs lying on surfaces (measured: 16 m/s forearm
    // whips on a staircase). Free-flying jointed bodies are where the dissipation matters.
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i] as ContactConstraint;
      c.bodyA.touchStamp = stamp;
      c.bodyB.touchStamp = stamp;
    }
    for (let i = 0; i < bodies.length; i++) {
      const body = bodies[i] as RigidBody;
      if (body.touchStamp === stamp) continue;
      this.feedbackInto(body.linearVelocity, body.feedbackLinear, dt);
      this.feedbackInto(body.angularVelocity, body.feedbackAngular, dt);
    }
  }

  private feedbackInto(target: Vec3, delta: Vec3, dt: number): void {
    const dv = V.scale(S_DV, delta, 1 / dt);
    const len = V.length(dv);
    if (len > this.maxCorrectionSpeed) V.scale(dv, dv, this.maxCorrectionSpeed / len);
    V.add(target, target, dv);
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

  /** `worldBounds` into the shared scratch box (valid until the next call). */
  private worldBoundsScratch(c: Collider): AABB {
    return placedBoundsInto(S_WB, c.shape, c.body.colliderTransform(c, S_TC));
  }

  private updateBroadphase(dt: number): void {
    for (const c of this.colliders) {
      // Static and sleeping bodies do not move; their proxies stay as they are.
      if (!c.body.isSimulated && c.body.type !== 'kinematic') continue;
      const proxy = this.colliderProxy.get(c);
      if (proxy === undefined) continue;
      const box = this.worldBoundsScratch(c);
      const v = c.body.linearVelocity;
      if (c.body.ccd && this.ccdMode === 'speculative' && c.body.isSimulated) {
        // Speculative contacts need pairs for everything within this step's reach.
        V.set(S_SPEC, v.x * dt, v.y * dt, v.z * dt);
        BB.expandPoint(box, V.add(S_PT, box.min, S_SPEC));
        BB.expandPoint(box, V.add(S_PT, box.max, S_SPEC));
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

  /** Pooled (key, [a, b]) tuples and the view of them returned by `currentPairs`. */
  private readonly pairPool: [number, [number, number]][] = [];
  private readonly pairView: [number, [number, number]][] = [];

  private currentPairs(): [number, [number, number]][] {
    const list = this.broadphase.pairList();
    const view = this.pairView;
    view.length = 0;
    for (let i = 0; i < list.length; i++) {
      const [a, b] = list[i] as [number, number];
      let e = this.pairPool[i];
      if (e) {
        e[0] = pairKey(a, b);
        e[1][0] = a;
        e[1][1] = b;
      } else {
        e = [pairKey(a, b), [a, b]];
        this.pairPool.push(e);
      }
      view.push(e);
    }
    return view;
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
  /** Scratch for `refreshMeshPair`: triangle indices and the map being built for the pair. */
  private readonly meshTris: number[] = [];
  private meshScratch = new Map<number, ContactConstraint>();

  private refreshMeshPair(key: number, ca: Collider, cb: Collider): void {
    const meshIsA = ca.shape instanceof TriangleMesh;
    const cm = meshIsA ? ca : cb;
    const co = meshIsA ? cb : ca;
    const mesh = cm.shape as TriangleMesh;
    if (co.shape instanceof TriangleMesh) return;
    const tm = cm.body.colliderTransform(cm, S_TM);
    const to = co.body.colliderTransform(co, S_TA);
    const tris = this.meshTris;
    tris.length = 0;
    mesh.bvh.query(toLocalBoxInto(S_BOX, placedBoundsInto(S_WB, co.shape, to), tm), tris);
    tris.sort(ascending);
    const subs = this.meshContacts.get(key);
    const next = this.meshScratch;
    const pts = S_MESH_PTS;
    const nTri = S_NTRI;
    for (let ti = 0; ti < tris.length; ti++) {
      const tri = tris[ti] as number;
      mesh.triangle(tri, pts);
      const e1 = V.sub(S_E1, pts[1], pts[0]);
      const e2 = V.sub(S_E2, pts[2], pts[0]);
      V.cross(nTri, e1, e2);
      const area2 = V.length(nTri);
      if (!(area2 > 1e-12)) continue; // degenerate triangle
      V.scale(nTri, nTri, 1 / area2);
      Q.rotate(nTri, tm.q, nTri);
      const shapeO = S_PLACED_A;
      shapeO.shape = co.shape;
      // Fast path: face-normal contact from the shape's lowest points over this triangle.
      const fast = convexTriangleContact(
        shapeO,
        T.apply(S_W0, tm, pts[0]),
        T.apply(S_W1, tm, pts[1]),
        T.apply(S_W2, tm, pts[2]),
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
      if (!old) this.wakeOnNewContact(ca.body, cb.body);
    }
    const had = subs !== undefined && subs.size > 0;
    const keep = next.size > 0;
    if (keep) {
      // `next` becomes the pair's map; the previous map, emptied, is the scratch for the next call.
      this.meshContacts.set(key, next);
      const spare = subs ?? new Map<number, ContactConstraint>();
      spare.clear();
      this.meshScratch = spare;
    } else {
      this.meshContacts.delete(key);
      next.clear();
    }
    if (keep) this.emit({ kind: had ? 'persist' : 'begin', a: ca, b: cb });
    else if (had) this.emit({ kind: 'end', a: ca, b: cb });
  }

  private refreshPair(key: number, ca: Collider, cb: Collider, dt: number): void {
    if (ca.shape instanceof TriangleMesh || cb.shape instanceof TriangleMesh) {
      this.refreshMeshPair(key, ca, cb);
      return;
    }
    ca.body.colliderTransform(ca, S_TA);
    cb.body.colliderTransform(cb, S_TB);
    const placedA = S_PLACED_A;
    const placedB = S_PLACED_B;
    placedA.shape = ca.shape;
    placedB.shape = cb.shape;
    const m =
      manifoldOf(placedA, placedB, this.contactMargin) ??
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
    if (!old) this.wakeOnNewContact(ca.body, cb.body);
  }

  /**
   * Builds a solver constraint from a manifold, carrying impulses over from `old` by feature id.
   * When `old` exists its object (and point objects) are refilled in place rather than replaced.
   */
  private makeConstraint(
    ca: Collider,
    cb: Collider,
    m: ContactManifold,
    old: ContactConstraint | undefined,
    sub: number,
  ): ContactConstraint {
    const mp = m.points;
    const n = mp.length;
    if (n > prevN.length) {
      prevN = new Float64Array(2 * n);
      prevT1 = new Float64Array(2 * n);
      prevT2 = new Float64Array(2 * n);
    }
    // Read the old impulses first: the refill below overwrites the old points.
    for (let i = 0; i < n; i++) {
      const id = (mp[i] as { id: number }).id;
      prevN[i] = prevT1[i] = prevT2[i] = 0;
      if (!old) continue;
      const op = old.points;
      for (let j = 0; j < op.length; j++) {
        const o = op[j] as SolverPoint;
        if (o.id !== id) continue;
        prevN[i] = o.impulseN;
        prevT1[i] = o.impulseT1;
        prevT2[i] = o.impulseT2;
        break;
      }
    }
    const pa = this.colliderProxy.get(ca) as number;
    const pb = this.colliderProxy.get(cb) as number;
    const c: Mutable<ContactConstraint> = old ?? {
      bodyA: ca.body,
      bodyB: cb.body,
      normal: new Vec3(),
      tangent1: new Vec3(),
      tangent2: new Vec3(),
      friction: 0,
      restitution: 0,
      points: [],
      pair: [pa, pb],
      sub,
      normalK: new Float64Array(0),
    };
    c.bodyA = ca.body;
    c.bodyB = cb.body;
    if (old) {
      V.copy(S_OT1, c.tangent1);
      V.copy(S_OT2, c.tangent2);
    }
    V.copy(c.normal, m.normal);
    // Same basis as `tangentBasis`.
    V.perpendicular(c.tangent1, m.normal);
    V.cross(c.tangent2, m.normal, c.tangent1);
    // perpendicular() switches branch at |nx| = 0.577 and the basis then flips: carry the friction
    // impulse over as a world-space vector instead of component-wise.
    if (old && V.lengthSq(S_OT1) > 0.5 && V.dot(S_OT1, c.tangent1) < 0.5) {
      for (let i = 0; i < n; i++) {
        const a = prevT1[i] as number;
        const b = prevT2[i] as number;
        V.set(
          S_DV,
          a * S_OT1.x + b * S_OT2.x,
          a * S_OT1.y + b * S_OT2.y,
          a * S_OT1.z + b * S_OT2.z,
        );
        prevT1[i] = V.dot(S_DV, c.tangent1);
        prevT2[i] = V.dot(S_DV, c.tangent2);
      }
    }
    c.friction = Math.sqrt(ca.friction * cb.friction);
    c.restitution = Math.max(ca.restitution, cb.restitution);
    if (c.pair[0] !== pa || c.pair[1] !== pb) c.pair = [pa, pb];
    c.sub = sub;
    const pts = c.points;
    for (let i = 0; i < n; i++) {
      const p = mp[i] as ContactManifold['points'][number];
      let sp = pts[i] as Mutable<SolverPoint> | undefined;
      if (!sp) {
        sp = {
          id: 0,
          localA: new Vec3(),
          localB: new Vec3(),
          rA: new Vec3(),
          rB: new Vec3(),
          gap: 0,
          restitutionTarget: 0,
          normalMass: 0,
          tangentMass1: 0,
          tangentMass2: 0,
          impulseN: 0,
          impulseT1: 0,
          impulseT2: 0,
        };
        pts.push(sp);
      }
      sp.id = p.id;
      localAnchorInto(sp.localA, ca.body, p.pointA);
      localAnchorInto(sp.localB, cb.body, p.pointB);
      V.set(sp.rA, 0, 0, 0);
      V.set(sp.rB, 0, 0, 0);
      sp.gap = Math.max(0, -p.depth);
      sp.restitutionTarget = 0;
      sp.normalMass = 0;
      sp.tangentMass1 = 0;
      sp.tangentMass2 = 0;
      sp.impulseN = prevN[i] as number;
      sp.impulseT1 = prevT1[i] as number;
      sp.impulseT2 = prevT2[i] as number;
    }
    pts.length = n;
    return c;
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
    // The array is never mutated in place, so this is the "iterate a copy" a listener may rely on
    // when it removes itself or others.
    if (e.kind !== 'persist') {
      const pa = this.colliderProxy.get(e.a);
      const pb = this.colliderProxy.get(e.b);
      if (pa !== undefined && pb !== undefined) {
        const k = pairKey(pa, pb);
        if (e.kind === 'begin') this.begun.set(k, { a: e.a, b: e.b });
        else this.begun.delete(k);
      }
    }
    const ls = this.listeners;
    for (let i = 0; i < ls.length; i++) (ls[i] as (e: ContactEvent) => void)(e);
  }

  /** True when the body moves clearly faster than the sleep speeds (not just settling). */
  private isMoving(b: RigidBody): boolean {
    return (
      V.lengthSq(b.linearVelocity) > (this.wakeSpeedFactor * this.sleepLinearSpeed) ** 2 ||
      V.lengthSq(b.angularVelocity) > (this.wakeSpeedFactor * this.sleepAngularSpeed) ** 2
    );
  }

  /**
   * A new contact wakes a sleeper only when the awake body is a kinematic or moving one. Contacts
   * on a settling heap appear and vanish at the contact margin; waking on each of those kept the
   * whole heap cycling between sleep and wake. Persisting contacts never wake anything.
   */
  private wakeOnNewContact(a: RigidBody, b: RigidBody): void {
    if (a.awake === b.awake) return;
    const sleeper = a.awake ? b : a;
    const other = a.awake ? a : b;
    if (!sleeper.isDynamic) return;
    if (!other.isDynamic || this.isMoving(other)) {
      this.pairWakes++; // see `updateContacts`: a wake triggers a recheck of skipped pairs
      sleeper.wake();
    }
  }

  /**
   * Puts resting bodies to sleep. Jointed bodies sleep together (union-find over joints). A body
   * sleeps once it and every body it touches have been slow for `sleepTimeLimit`, so one restless
   * body only delays its own neighbours instead of a whole connected heap. A moving body that
   * touches a sleeping one wakes it, which keeps sleepers from hanging in the air when their
   * support moves.
   */
  private updateSleep(contacts: readonly ContactConstraint[], dt: number): void {
    // Union-find over the bodies, with the parent links stored on the bodies themselves.
    for (const body of this.bodies) {
      body.islandParent = body;
      body.islandMin = Infinity;
      if (!body.isDynamic) continue;
      const slow =
        V.lengthSq(body.linearVelocity) < this.sleepLinearSpeed ** 2 &&
        V.lengthSq(body.angularVelocity) < this.sleepAngularSpeed ** 2;
      body.sleepTime = slow ? body.sleepTime + dt : 0;
      body.islandMin = body.sleepTime;
    }
    // A shared static floor or anchor never couples bodies.
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i] as ContactConstraint;
      const a = c.bodyA;
      const b = c.bodyB;
      if (!a.isDynamic || !b.isDynamic) continue;
      // islandMin starts as the body's own sleepTime; neighbours only ever lower it.
      if (b.sleepTime < a.islandMin) a.islandMin = b.sleepTime;
      if (a.sleepTime < b.islandMin) b.islandMin = a.sleepTime;
    }
    // Waking is deferred (islandMin = -1 marks the sleeper) so a body woken in this pass does not
    // itself count as a mover.
    let anyWake = false;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i] as ContactConstraint;
      const a = c.bodyA;
      const b = c.bodyB;
      if (!a.isDynamic || !b.isDynamic || a.awake === b.awake) continue;
      const sleeper = a.awake ? b : a;
      const mover = a.awake ? a : b;
      // Only a clearly moving body wakes a sleeper; the faint twitching of a settling heap must not.
      if (mover.sleepTime === 0 && this.isMoving(mover)) {
        sleeper.islandMin = -1;
        anyWake = true;
      }
    }
    if (anyWake) {
      for (const body of this.bodies) {
        if (body.isDynamic && !body.awake && body.islandMin === -1) body.wake();
      }
    }
    for (const j of this.joints) {
      const a = j.bodyA;
      const b = j.bodyB;
      if (!a.isDynamic || !b.isDynamic) continue;
      linkIsland(a, b);
    }
    // Joint groups take the minimum over their members; a woken member keeps the group awake.
    for (const body of this.bodies) {
      if (!body.isDynamic) continue;
      const root = findIsland(body);
      if (root !== body) root.islandMin = Math.min(root.islandMin, body.islandMin);
    }
    for (const body of this.bodies) {
      if (!body.isDynamic || !body.awake) continue;
      const ready =
        (body.jointCount > 0 ? findIsland(body).islandMin : body.islandMin) >= this.sleepTimeLimit;
      if (ready) {
        body.awake = false;
        body.linearVelocity.x = body.linearVelocity.y = body.linearVelocity.z = 0;
        body.angularVelocity.x = body.angularVelocity.y = body.angularVelocity.z = 0;
      }
    }
  }
}

function findIsland(b: RigidBody): RigidBody {
  let r = b;
  while (r.islandParent !== r) {
    r.islandParent = r.islandParent.islandParent;
    r = r.islandParent;
  }
  return r;
}

function linkIsland(a: RigidBody, b: RigidBody): void {
  if (!a.isDynamic || !b.isDynamic) return;
  const ra = findIsland(a);
  const rb = findIsland(b);
  if (ra !== rb) ra.islandParent = rb;
}

/** Bounding box, in the frame of transform `t`, of a world-space box. */
function toLocalBox(box: AABB, t: Transform): AABB {
  return toLocalBoxInto(new AABB(), box, t);
}

/** `toLocalBox` into `out` (which must not be `box`). */
function toLocalBoxInto(out: AABB, box: AABB, t: Transform): AABB {
  BB.empty(out);
  for (let i = 0; i < 8; i++) {
    S_PT2.x = i & 4 ? box.max.x : box.min.x;
    S_PT2.y = i & 2 ? box.max.y : box.min.y;
    S_PT2.z = i & 1 ? box.max.z : box.min.z;
    BB.expandPoint(out, T.applyInverse(S_PT, t, S_PT2));
  }
  return out;
}

/** World-space bounding box of a convex shape placed at `t`. */
function placedBounds(shape: ColliderShape, t: Transform): AABB {
  return placedBoundsInto(new AABB(), shape, t);
}

/** `placedBounds` into `out`. Corner order (x slowest, z fastest) matches the allocating version. */
function placedBoundsInto(out: AABB, shape: ColliderShape, t: Transform): AABB {
  const local = shape.localBounds(S_LB);
  BB.empty(out);
  for (let i = 0; i < 8; i++) {
    S_PT2.x = i & 4 ? local.max.x : local.min.x;
    S_PT2.y = i & 2 ? local.max.y : local.min.y;
    S_PT2.z = i & 1 ? local.max.z : local.min.z;
    BB.expandPoint(out, T.apply(S_PT, t, S_PT2));
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
function localAnchorInto(out: Vec3, body: RigidBody, world: Vec3): Vec3 {
  V.sub(out, world, body.position);
  return Q.rotateInverse(out, body.orientation, out);
}

function ascending(x: number, y: number): number {
  return x - y;
}
