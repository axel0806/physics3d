import { AABB, Quat, Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import * as T from '../math/transform';
import * as Q from '../math/quat';
import { RigidBody, type BodyOptions, type Collider } from '../dynamics/body';
import {
  integratePosition,
  integrateOrientation,
  integrateOrientationPostSolve,
  integrateVelocities,
} from '../dynamics/integrate';
import type { ConvexShape } from '../shapes/shape';
import { BroadPhase, pairKey } from '../collision/broadphase';
import { Joint } from '../constraints/joint';
import { manifoldOf } from '../collision/pairs';
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
  positionIterations: number;
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
  private readonly colliderProxy = new Map<Collider, number>();
  private readonly proxyCollider = new Map<number, Collider>();
  private readonly jointedPairs = new Set<number>();
  private nextBodyId = 1;
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
    const body = new RigidBody(this.nextBodyId++, options);
    this.bodies.push(body);
    return body;
  }

  /** Attaches a convex shape to a body. Call refreshMass after adding colliders. */
  createCollider(body: RigidBody, shape: ConvexShape, options: ColliderOptions = {}): Collider {
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
    this.colliders.push(c);
    const proxy = this.broadphase.createProxy(this.worldBounds(c));
    this.colliderProxy.set(c, proxy);
    this.proxyCollider.set(proxy, c);
    body.refreshMassProperties();
    return c;
  }

  /** Registers a joint. Unless `collideConnected`, its two bodies stop colliding with each other. */
  addJoint<J extends Joint>(joint: J): J {
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

  /** Current contact constraints (for inspection and tests). */
  activeContacts(): ContactConstraint[] {
    return [...this.contacts.values()];
  }

  /** Advances the simulation by dt seconds. */
  step(dt: number): void {
    if (!(dt > 0)) throw new RangeError('dt must be positive');
    for (const body of this.bodies) body.snapshotVelocities();
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
    for (const j of this.joints) {
      if (!j.active) continue;
      j.prepare(dt);
      j.warmStart();
    }
    for (const c of this.contacts.values()) {
      if (!this.constraintActive(c)) continue;
      prepareContact(c, this.restitutionThreshold, dt);
      buildNormalK(c);
      warmStart(c);
    }
    for (let it = 0; it < this.velocityIterations; it++) {
      for (const j of this.joints) if (j.active) j.solveVelocity(dt);
      for (const c of this.contacts.values()) if (this.constraintActive(c)) solveVelocity(c);
    }
    // Jointed bodies rotate after the solve, from the same pose the joints were evaluated at;
    // free bodies were rotated before it (see integrateOrientation).
    for (const body of this.bodies)
      if (body.jointCount > 0) integrateOrientationPostSolve(body, dt);
    // CCD bodies move last, against the final poses of everything else.
    for (const body of this.bodies) if (!body.ccd) integratePosition(body, dt);
    for (const body of this.bodies) {
      if (!body.ccd) continue;
      if (this.ccdMode === 'sweep' && this.needsSweep(body, dt)) this.sweepBody(body, dt);
      else integratePosition(body, dt);
    }
    const feedback = new Map<RigidBody, [Vec3, Vec3]>();
    for (let it = 0; it < this.positionIterations; it++) {
      for (const c of this.contacts.values()) {
        if (this.constraintActive(c)) solvePosition(c, this.baumgarte, this.linearSlop);
      }
      for (const j of this.joints) if (j.active) j.solveLimits();
      for (const j of this.joints) if (j.active) this.correctJoint(j, feedback);
    }
    this.applyFeedback(feedback, dt);
    for (const body of this.bodies) body.updateWorldInertia();
    this.updateSleep(dt);
    this.frame++;
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
          const tb = cb.body.colliderTransform(cb, new Transform());
          const toi = timeOfImpact(
            { shape: ca.shape, transform: ta },
            { shape: cb.shape, transform: tb },
            d,
            new Vec3(),
            { target: this.linearSlop, tolerance: 0.25 * this.linearSlop },
          );
          if (toi && (!best || toi.t < best.toi.t)) best = { toi, ca, cb };
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
    for (const c of this.contacts.values()) {
      touching.add(c.bodyA);
      touching.add(c.bodyB);
    }
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

  private worldBounds(c: Collider): AABB {
    const t = c.body.colliderTransform(c, new Transform());
    const local = c.shape.localBounds(new AABB());
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

  private updateBroadphase(dt: number): void {
    for (const c of this.colliders) {
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
    for (const ch of changes) {
      if (ch.kind !== 'end') continue;
      const ca = this.proxyCollider.get(ch.a);
      const cb = this.proxyCollider.get(ch.b);
      if (!ca || !cb) continue;
      this.contacts.delete(pairKey(ch.a, ch.b));
      this.emit({ kind: 'end', a: ca, b: cb });
    }
    // Refresh manifolds for every overlapping pair (cheap for tests; a real engine would cache).
    for (const [key, pair] of this.currentPairs()) {
      const [ia, ib] = pair;
      const ca = this.proxyCollider.get(ia);
      const cb = this.proxyCollider.get(ib);
      if (!ca || !cb) continue;
      if (!this.shouldCollide(ca, cb)) {
        this.contacts.delete(key);
        continue;
      }
      this.refreshPair(key, ca, cb, dt);
    }
  }

  private currentPairs(): [number, [number, number]][] {
    return this.broadphase.pairList().map(([a, b]) => [pairKey(a, b), [a, b]]);
  }

  private shouldCollide(a: Collider, b: Collider): boolean {
    if (a.body === b.body) return false;
    if (this.jointedPairs.has(bodyPairKey(a.body, b.body))) return false;
    if (!(a.body.isSimulated || b.body.isSimulated)) return false;
    if ((a.mask & b.layer) === 0 || (b.mask & a.layer) === 0) return false;
    return true;
  }

  private refreshPair(key: number, ca: Collider, cb: Collider, dt: number): void {
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
    const c: ContactConstraint = {
      bodyA: ca.body,
      bodyB: cb.body,
      normal: V.copy(new Vec3(), m.normal),
      tangent1: t1,
      tangent2: t2,
      friction: Math.sqrt(ca.friction * cb.friction),
      restitution: Math.max(ca.restitution, cb.restitution),
      points,
      normalK: [],
    };
    if (!old) this.emit({ kind: 'begin', a: ca, b: cb });
    else this.emit({ kind: 'persist', a: ca, b: cb });
    this.contacts.set(key, c);
    // A new contact wakes an island that touches an awake body. Persisting contacts do not wake
    // anything, so resting stacks can accumulate sleep time.
    if (!old && (ca.body.awake || cb.body.awake)) {
      ca.body.wake();
      cb.body.wake();
    }
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
    for (const l of this.listeners) l(e);
  }

  /** Puts islands of slow bodies to sleep, using union-find over active contacts. */
  private updateSleep(dt: number): void {
    const parent = new Map<RigidBody, RigidBody>();
    const find = (b: RigidBody): RigidBody => {
      let r = b;
      while ((parent.get(r) ?? r) !== r) r = parent.get(r) as RigidBody;
      return r;
    };
    for (const body of this.bodies) parent.set(body, body);
    // Islands connect through dynamic bodies only: a shared static floor or anchor must not merge
    // unrelated groups.
    const link = (a: RigidBody, b: RigidBody): void => {
      if (!a.isDynamic || !b.isDynamic) return;
      const ra = find(a);
      const rb = find(b);
      if (ra !== rb) parent.set(ra, rb);
    };
    for (const c of this.contacts.values()) link(c.bodyA, c.bodyB);
    for (const j of this.joints) link(j.bodyA, j.bodyB);
    const islandMinSleep = new Map<RigidBody, number>();
    for (const body of this.bodies) {
      if (!body.isDynamic) continue;
      const slow =
        V.lengthSq(body.linearVelocity) < 1e-4 && V.lengthSq(body.angularVelocity) < 1e-4;
      body.sleepTime = slow ? body.sleepTime + dt : 0;
      const root = find(body);
      islandMinSleep.set(root, Math.min(islandMinSleep.get(root) ?? Infinity, body.sleepTime));
    }
    for (const body of this.bodies) {
      if (!body.isDynamic || !body.awake) continue;
      const ready = (islandMinSleep.get(find(body)) ?? 0) >= this.sleepTimeLimit;
      if (ready) {
        body.awake = false;
        body.linearVelocity.x = body.linearVelocity.y = body.linearVelocity.z = 0;
        body.angularVelocity.x = body.angularVelocity.y = body.angularVelocity.z = 0;
      }
    }
  }
}

function bodyPairKey(a: RigidBody, b: RigidBody): number {
  return a.id < b.id ? a.id * 1048576 + b.id : b.id * 1048576 + a.id;
}

/** Anchor of a world point in a body's frame: the inverse rotation of (point − center of mass). */
function localAnchor(body: RigidBody, world: Vec3): Vec3 {
  const d = V.sub(new Vec3(), world, body.position);
  return Q.rotateInverse(new Vec3(), body.orientation, d);
}
