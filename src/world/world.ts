import { AABB, Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import * as T from '../math/transform';
import * as Q from '../math/quat';
import { RigidBody, type BodyOptions, type Collider } from '../dynamics/body';
import {
  integratePosition,
  integrateOrientation,
  integrateVelocities,
} from '../dynamics/integrate';
import type { ConvexShape } from '../shapes/shape';
import { BroadPhase, pairKey } from '../collision/broadphase';
import { manifoldOf } from '../collision/pairs';
import {
  buildNormalK,
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
  readonly broadphase = new BroadPhase(0.1);
  velocityIterations: number;
  positionIterations: number;
  restitutionThreshold: number;
  baumgarte: number;
  linearSlop: number;
  sleepTimeLimit: number;
  /** Frames stepped so far. */
  frame = 0;
  private readonly contacts = new Map<number, ContactConstraint>();
  private readonly colliderProxy = new Map<Collider, number>();
  private readonly proxyCollider = new Map<number, Collider>();
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
    this.updateContacts();
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      integrateVelocities(body, this.gravity, dt);
      integrateOrientation(body, dt);
    }
    for (const c of this.contacts.values()) {
      if (!this.constraintActive(c)) continue;
      prepareContact(c, this.restitutionThreshold);
      buildNormalK(c);
      warmStart(c);
    }
    for (let it = 0; it < this.velocityIterations; it++) {
      for (const c of this.contacts.values()) if (this.constraintActive(c)) solveVelocity(c);
    }
    for (const body of this.bodies) integratePosition(body, dt);
    for (let it = 0; it < this.positionIterations; it++) {
      for (const c of this.contacts.values()) {
        if (this.constraintActive(c)) solvePosition(c, this.baumgarte, this.linearSlop);
      }
    }
    for (const body of this.bodies) body.updateWorldInertia();
    this.updateSleep(dt);
    this.frame++;
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
      this.broadphase.moveProxy(proxy, box, v.x * dt, v.y * dt, v.z * dt);
    }
  }

  /** Re-runs the pair manifolds for overlapping collider pairs, keeping impulses by feature id. */
  private updateContacts(): void {
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
      if (!this.shouldCollide(ca, cb)) continue;
      this.refreshPair(key, ca, cb);
    }
  }

  private currentPairs(): [number, [number, number]][] {
    return this.broadphase.pairList().map(([a, b]) => [pairKey(a, b), [a, b]]);
  }

  private shouldCollide(a: Collider, b: Collider): boolean {
    if (a.body === b.body) return false;
    if (!(a.body.isSimulated || b.body.isSimulated)) return false;
    if ((a.mask & b.layer) === 0 || (b.mask & a.layer) === 0) return false;
    return true;
  }

  private refreshPair(key: number, ca: Collider, cb: Collider): void {
    const ta = ca.body.colliderTransform(ca, new Transform());
    const tb = cb.body.colliderTransform(cb, new Transform());
    const m = manifoldOf({ shape: ca.shape, transform: ta }, { shape: cb.shape, transform: tb });
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
    for (const c of this.contacts.values()) {
      const ra = find(c.bodyA);
      const rb = find(c.bodyB);
      if (ra !== rb) parent.set(ra, rb);
    }
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

/** Anchor of a world point in a body's frame: the inverse rotation of (point − center of mass). */
function localAnchor(body: RigidBody, world: Vec3): Vec3 {
  const d = V.sub(new Vec3(), world, body.position);
  return Q.rotateInverse(new Vec3(), body.orientation, d);
}
