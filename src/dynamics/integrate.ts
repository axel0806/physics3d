import { Mat3, Quat, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as M3 from '../math/mat3';
import * as Q from '../math/quat';
import type { RigidBody } from './body';

const GYRO_ITERATIONS = 60;

/**
 * Semi-implicit (symplectic) Euler for a rigid body. Velocities first, then poses.
 *
 * The gyroscopic term is solved in the body frame, where the inertia tensor is constant, using the
 * implicit midpoint rule on the torque-free Euler equation I ω̇ = (I ω) × ω. The fixed point is
 * iterated and the result rotated back to world space. Freezing the world-space inertia instead
 * leaks angular momentum at first order in dt.
 */
export function integrateVelocities(body: RigidBody, gravity: Vec3, dt: number): void {
  if (!body.isSimulated) return;
  const v = body.linearVelocity;
  V.addScaled(v, v, gravity, body.gravityScale * dt);
  V.scale(v, v, Math.max(0, 1 - body.linearDamping * dt));

  const rot = new Mat3();
  M3.fromQuat(rot, body.orientation);
  const wBody = M3.mulVec(new Vec3(), rot.transpose(), body.angularVelocity);
  const I = body.localInertia;
  const L0 = M3.mulVec(new Vec3(), I, wBody);
  // Implicit midpoint: L_{n+1} − L_n = dt · (L_mid × ω_mid). Dotting with ω_mid and with L_mid
  // gives exact conservation of ω·Iω and |L| for torque-free bodies.
  let next = V.copy(new Vec3(), wBody);
  for (let k = 0; k < GYRO_ITERATIONS; k++) {
    const mid = V.lerp(new Vec3(), wBody, next, 0.5);
    const Lmid = M3.mulVec(new Vec3(), I, mid);
    const gyro = V.cross(new Vec3(), Lmid, mid);
    next = M3.mulVec(new Vec3(), body.localInvInertia, V.addScaled(new Vec3(), L0, gyro, dt));
  }
  const w = M3.mulVec(body.angularVelocity, rot, next);
  V.scale(w, w, Math.max(0, 1 - body.angularDamping * dt));
  body.updateWorldInertia();
}

/**
 * Rotates a body over dt before its contacts are solved. The rotation is exact about the
 * body-frame midpoint angular velocity, q ← q ⊗ exp(ω̄_b · dt), where ω̄_b averages the start-of-step
 * and post-gyroscopic velocities. The body-frame velocity is carried into the new orientation.
 * Solving contacts after this rotation applies impulses in the frame that angular momentum is
 * measured in, which keeps total angular momentum consistent with the impulses.
 */
export function integrateOrientation(body: RigidBody, dt: number): void {
  if (!body.isSimulated) return;
  const rot = new Mat3();
  M3.fromQuat(rot, body.orientation);
  const rt = rot.transpose();
  const wStart = M3.mulVec(new Vec3(), rt, body.stepStartAngular);
  const wEnd = M3.mulVec(new Vec3(), rt, body.angularVelocity);
  const mid = V.lerp(new Vec3(), wStart, wEnd, 0.5);
  const speed = V.length(mid);
  if (speed > 0) {
    const half = 0.5 * speed * dt;
    const s = Math.sin(half) / speed;
    const dq = new Quat(mid.x * s, mid.y * s, mid.z * s, Math.cos(half));
    Q.mul(body.orientation, body.orientation, dq);
    Q.normalize(body.orientation, body.orientation);
  }
  const rotNew = new Mat3();
  M3.fromQuat(rotNew, body.orientation);
  M3.mulVec(body.angularVelocity, rotNew, wEnd);
  body.updateWorldInertia();
}

/** Translates a body by its (post-solve) linear velocity over dt. */
export function integratePosition(body: RigidBody, dt: number): void {
  if (!body.isSimulated) return;
  const p = body.position;
  p.x += body.linearVelocity.x * dt;
  p.y += body.linearVelocity.y * dt;
  p.z += body.linearVelocity.z * dt;
}

/**
 * Rotates a jointed body by its post-solve world angular velocity, q <- exp(w*dt) (x) q. Joints are
 * evaluated at the pose the position integration starts from, so the orientation must advance
 * after the constraint solve (standard semi-implicit Euler) rather than before it.
 */
export function integrateOrientationPostSolve(body: RigidBody, dt: number): void {
  if (!body.isSimulated) return;
  const w = body.angularVelocity;
  const speed = V.length(w);
  if (speed > 0) {
    const half = 0.5 * speed * dt;
    const s = Math.sin(half) / speed;
    const dq = new Quat(w.x * s, w.y * s, w.z * s, Math.cos(half));
    Q.mul(body.orientation, dq, body.orientation);
    Q.normalize(body.orientation, body.orientation);
  }
  body.updateWorldInertia();
}
