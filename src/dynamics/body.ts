import { Mat3, Quat, Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as M3 from '../math/mat3';
import * as Q from '../math/quat';
import type { ConvexShape } from '../shapes/shape';
import {
  combine,
  transformMassProperties,
  emptyMassProperties,
  type MassProperties,
} from '../shapes/mass';

export type BodyType = 'static' | 'kinematic' | 'dynamic';

/** A convex shape attached to a body. `local` is its frame relative to the body's center of mass. */
export interface Collider {
  readonly body: RigidBody;
  readonly shape: ConvexShape;
  readonly local: Transform;
  readonly friction: number;
  readonly restitution: number;
  /** Bit mask of categories this collider belongs to, and of categories it accepts. */
  readonly layer: number;
  readonly mask: number;
}

export interface BodyOptions {
  readonly type?: BodyType;
  readonly position?: Vec3;
  readonly orientation?: Quat;
  readonly linearVelocity?: Vec3;
  readonly angularVelocity?: Vec3;
  readonly linearDamping?: number;
  readonly angularDamping?: number;
  readonly gravityScale?: number;
  /** Sweep this body's motion against other colliders so fast movement cannot tunnel. */
  readonly ccd?: boolean;
}

/**
 * Rigid body. `position` is the center of mass in world space. Dynamic bodies have positive mass
 * and a local inertia tensor; static and kinematic bodies have zero inverse mass.
 */
export class RigidBody {
  type: BodyType;
  readonly position: Vec3;
  readonly orientation: Quat;
  readonly linearVelocity: Vec3;
  readonly angularVelocity: Vec3;
  linearDamping: number;
  angularDamping: number;
  gravityScale: number;
  /** Continuous collision detection: sweep fast motion for the earliest impact. */
  ccd: boolean;
  /** Mass and inertia, summed from colliders. Zero for non-dynamic bodies. */
  mass = 0;
  invMass = 0;
  /** Inverse inertia about the center of mass, in the body frame. */
  localInvInertia: Mat3 = new Mat3(new Array(9).fill(0));
  /** Inertia tensor about the center of mass, in the body frame (constant for the body's life). */
  localInertia: Mat3 = new Mat3(new Array(9).fill(0));
  /** Inverse inertia in world space, refreshed each step from the orientation. */
  readonly worldInvInertia: Mat3 = new Mat3(new Array(9).fill(0));
  readonly colliders: Collider[] = [];
  /** Velocities at the start of the step, before gravity and contacts; restitution uses these. */
  readonly stepStartLinear: Vec3 = new Vec3();
  readonly stepStartAngular: Vec3 = new Vec3();
  /** Pose at the start of the last step, for render interpolation. */
  readonly prevPosition: Vec3;
  readonly prevOrientation: Quat;
  /** Number of joints attached; jointed bodies rotate after the constraint solve. */
  jointCount = 0;
  /** Sleep bookkeeping. */
  awake = true;
  sleepTime = 0;
  readonly id: number;

  constructor(id: number, options: BodyOptions = {}) {
    this.id = id;
    this.type = options.type ?? 'dynamic';
    this.position = options.position?.clone() ?? new Vec3();
    this.orientation = options.orientation?.clone() ?? new Quat();
    this.prevPosition = this.position.clone();
    this.prevOrientation = this.orientation.clone();
    this.linearVelocity = options.linearVelocity?.clone() ?? new Vec3();
    this.angularVelocity = options.angularVelocity?.clone() ?? new Vec3();
    this.linearDamping = options.linearDamping ?? 0;
    this.angularDamping = options.angularDamping ?? 0;
    this.gravityScale = options.gravityScale ?? 1;
    this.ccd = options.ccd ?? false;
  }

  get isDynamic(): boolean {
    return this.type === 'dynamic';
  }

  /** True when the body moves under contact impulses (dynamic, and awake). */
  get isSimulated(): boolean {
    return this.type === 'dynamic' && this.awake;
  }

  /** Recomputes mass, center of mass, and inverse inertia from the colliders. Dynamic bodies only. */
  refreshMassProperties(): void {
    if (!this.isDynamic) {
      this.mass = 0;
      this.invMass = 0;
      this.localInvInertia.e.fill(0);
      this.localInertia.e.fill(0);
      this.worldInvInertia.e.fill(0);
      return;
    }
    const parts: MassProperties[] = [];
    for (const c of this.colliders) {
      const density = 1;
      parts.push(transformMassProperties(c.shape.massProperties(density), c.local));
    }
    const total = parts.length ? combine(parts) : emptyMassProperties();
    if (!(total.mass > 0)) throw new RangeError('dynamic body needs positive mass');
    this.mass = total.mass;
    this.invMass = 1 / total.mass;
    const inv = new Mat3();
    M3.invert(inv, total.inertia);
    this.localInvInertia.e.set(inv.e);
    this.localInertia.e.set(total.inertia.e);
    this.updateWorldInertia();
  }

  /** Recomputes the world inverse inertia from the current orientation. */
  updateWorldInertia(): void {
    this.updateWorldInertiaFrom(this.orientation);
  }

  /** Recomputes the world inverse inertia for an arbitrary orientation (e.g. the predicted mid-step pose). */
  updateWorldInertiaFrom(q: Quat): void {
    const rot = new Mat3();
    M3.fromQuat(rot, q);
    const tmp = M3.mul(new Mat3(), rot, this.localInvInertia);
    M3.mul(this.worldInvInertia, tmp, rot.transpose());
  }

  /** Velocity of the material point at world offset r from the center of mass. */
  velocityAt(r: Vec3, out: Vec3): Vec3 {
    const w = this.angularVelocity;
    out.x = this.linearVelocity.x + (w.y * r.z - w.z * r.y);
    out.y = this.linearVelocity.y + (w.z * r.x - w.x * r.z);
    out.z = this.linearVelocity.z + (w.x * r.y - w.y * r.x);
    return out;
  }

  /** Records the pose at the start of a step. */
  snapshotPose(): void {
    V.copy(this.prevPosition, this.position);
    Q.copy(this.prevOrientation, this.orientation);
  }

  /** Records the velocities at the start of a step. */
  snapshotVelocities(): void {
    V.copy(this.stepStartLinear, this.linearVelocity);
    V.copy(this.stepStartAngular, this.angularVelocity);
  }

  /** Transform of a collider in world space. */
  colliderTransform(c: Collider, out: Transform): Transform {
    Q.mul(out.q, this.orientation, c.local.q);
    const p = Q.rotate(new Vec3(), this.orientation, c.local.p);
    out.p.x = this.position.x + p.x;
    out.p.y = this.position.y + p.y;
    out.p.z = this.position.z + p.z;
    return out;
  }

  wake(): void {
    if (!this.isDynamic) return;
    this.awake = true;
    this.sleepTime = 0;
  }
}
