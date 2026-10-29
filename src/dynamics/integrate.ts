import { Quat, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as Q from '../math/quat';
import type { RigidBody } from './body';

const GYRO_ITERATIONS = 60;

// Module-level temporaries: the step loop must not allocate (PLAN rule 8). Single-threaded and
// non-reentrant, so sharing them between calls is safe.
const dq = new Quat();

/**
 * Semi-implicit (symplectic) Euler for a rigid body. Velocities first, then poses.
 *
 * The gyroscopic term is solved in the body frame, where the inertia tensor is constant, using the
 * implicit midpoint rule on the torque-free Euler equation I ω̇ = (I ω) × ω. The fixed point is
 * iterated to machine precision and the result rotated back to world space. Freezing the world-space
 * inertia instead leaks angular momentum at first order in dt. Bodies with an isotropic inertia
 * tensor (cubes, spheres) have no gyroscopic term and skip the iteration. Scalar arithmetic only.
 */
export function integrateVelocities(body: RigidBody, gravity: Vec3, dt: number): void {
  if (!body.isSimulated) return;
  const v = body.linearVelocity;
  const gs = body.gravityScale * dt;
  const ld = Math.max(0, 1 - body.linearDamping * dt);
  v.x = (v.x + gravity.x * gs) * ld;
  v.y = (v.y + gravity.y * gs) * ld;
  v.z = (v.z + gravity.z * gs) * ld;

  const w = body.angularVelocity;
  if (w.x === 0 && w.y === 0 && w.z === 0) return;
  const I = body.localInertia.e;
  const i0 = I[0] as number;
  const i1 = I[1] as number;
  const i2 = I[2] as number;
  const i3 = I[3] as number;
  const i4 = I[4] as number;
  const i5 = I[5] as number;
  const i6 = I[6] as number;
  const i7 = I[7] as number;
  const i8 = I[8] as number;
  const ad = Math.max(0, 1 - body.angularDamping * dt);
  // Isotropic inertia: ω × Iω = 0, so the angular velocity only damps.
  if (
    i1 === 0 &&
    i2 === 0 &&
    i3 === 0 &&
    i5 === 0 &&
    i6 === 0 &&
    i7 === 0 &&
    i0 === i4 &&
    i4 === i8
  ) {
    if (ad !== 1) {
      w.x *= ad;
      w.y *= ad;
      w.z *= ad;
    }
    return;
  }
  const { x: qx, y: qy, z: qz, w: qw } = body.orientation;
  const r00 = 1 - 2 * (qy * qy + qz * qz);
  const r01 = 2 * (qx * qy - qz * qw);
  const r02 = 2 * (qx * qz + qy * qw);
  const r10 = 2 * (qx * qy + qz * qw);
  const r11 = 1 - 2 * (qx * qx + qz * qz);
  const r12 = 2 * (qy * qz - qx * qw);
  const r20 = 2 * (qx * qz - qy * qw);
  const r21 = 2 * (qy * qz + qx * qw);
  const r22 = 1 - 2 * (qx * qx + qy * qy);
  // Body-frame velocity Rᵀ ω.
  const bx = r00 * w.x + r10 * w.y + r20 * w.z;
  const by = r01 * w.x + r11 * w.y + r21 * w.z;
  const bz = r02 * w.x + r12 * w.y + r22 * w.z;
  const J = body.localInvInertia.e;
  const j0 = J[0] as number;
  const j1 = J[1] as number;
  const j2 = J[2] as number;
  const j3 = J[3] as number;
  const j4 = J[4] as number;
  const j5 = J[5] as number;
  const j6 = J[6] as number;
  const j7 = J[7] as number;
  const j8 = J[8] as number;
  const l0x = i0 * bx + i1 * by + i2 * bz;
  const l0y = i3 * bx + i4 * by + i5 * bz;
  const l0z = i6 * bx + i7 * by + i8 * bz;
  // Implicit midpoint: L_{n+1} − L_n = dt · (L_mid × ω_mid). Dotting with ω_mid and with L_mid
  // gives exact conservation of ω·Iω and |L| for torque-free bodies.
  let nx = bx;
  let ny = by;
  let nz = bz;
  const tol = 1e-16 * (Math.abs(bx) + Math.abs(by) + Math.abs(bz));
  for (let k = 0; k < GYRO_ITERATIONS; k++) {
    const mx = 0.5 * (bx + nx);
    const my = 0.5 * (by + ny);
    const mz = 0.5 * (bz + nz);
    const lx = i0 * mx + i1 * my + i2 * mz;
    const ly = i3 * mx + i4 * my + i5 * mz;
    const lz = i6 * mx + i7 * my + i8 * mz;
    const tx = l0x + dt * (ly * mz - lz * my);
    const ty = l0y + dt * (lz * mx - lx * mz);
    const tz = l0z + dt * (lx * my - ly * mx);
    const px = j0 * tx + j1 * ty + j2 * tz;
    const py = j3 * tx + j4 * ty + j5 * tz;
    const pz = j6 * tx + j7 * ty + j8 * tz;
    const d = Math.abs(px - nx) + Math.abs(py - ny) + Math.abs(pz - nz);
    nx = px;
    ny = py;
    nz = pz;
    if (d <= tol) break;
    if (k === GYRO_ITERATIONS - 1 || !(d < 1e30)) {
      // The fixed point diverges when dt·|ω| is large (fast spin of an elongated body), and the
      // iterates then blow up to thousands of rad/s. Fall back to one linearised implicit-Euler
      // (Newton) step, which is unconditionally stable.
      newtonStep(bx, by, bz, i0, i1, i2, i3, i4, i5, i6, i7, i8, dt);
      nx = gx;
      ny = gy;
      nz = gz;
      break;
    }
  }
  w.x = (r00 * nx + r01 * ny + r02 * nz) * ad;
  w.y = (r10 * nx + r11 * ny + r12 * nz) * ad;
  w.z = (r20 * nx + r21 * ny + r22 * nz) * ad;
}

// Result of `newtonStep`, and scratch of `jcol`.
let cx = 0;
let cy = 0;
let cz = 0;
let gx = 0;
let gy = 0;
let gz = 0;

/** Column of the Newton Jacobian for the unit vector e: I e + h (ω × I e − L × e), into cx, cy, cz. */
function jcol(
  wx: number,
  wy: number,
  wz: number,
  lx: number,
  ly: number,
  lz: number,
  ax: number,
  ay: number,
  az: number,
  ex: number,
  ey: number,
  ez: number,
  h: number,
): void {
  cx = ax + h * (wy * az - wz * ay - (ly * ez - lz * ey));
  cy = ay + h * (wz * ax - wx * az - (lz * ex - lx * ez));
  cz = az + h * (wx * ay - wy * ax - (lx * ey - ly * ex));
}

/**
 * One Newton step of the implicit Euler gyroscopic equation I(ω' − ω) + dt ω' × Iω' = 0 from ω'=ω:
 * solves (I + dt([ω]× I − [Iω]×)) Δ = −dt ω × Iω for Δ (Catto, GDC 2015). Result in gx, gy, gz.
 */
function newtonStep(
  wx: number,
  wy: number,
  wz: number,
  i0: number,
  i1: number,
  i2: number,
  i3: number,
  i4: number,
  i5: number,
  i6: number,
  i7: number,
  i8: number,
  h: number,
): void {
  const lx = i0 * wx + i1 * wy + i2 * wz;
  const ly = i3 * wx + i4 * wy + i5 * wz;
  const lz = i6 * wx + i7 * wy + i8 * wz;
  const rx = -h * (wy * lz - wz * ly);
  const ry = -h * (wz * lx - wx * lz);
  const rz = -h * (wx * ly - wy * lx);
  // Columns of J: J e_k = I e_k + h (ω × I e_k − L × e_k).
  jcol(wx, wy, wz, lx, ly, lz, i0, i3, i6, 1, 0, 0, h);
  const a00 = cx;
  const a10 = cy;
  const a20 = cz;
  jcol(wx, wy, wz, lx, ly, lz, i1, i4, i7, 0, 1, 0, h);
  const a01 = cx;
  const a11 = cy;
  const a21 = cz;
  jcol(wx, wy, wz, lx, ly, lz, i2, i5, i8, 0, 0, 1, h);
  const a02 = cx;
  const a12 = cy;
  const a22 = cz;
  const det =
    a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
  if (!(Math.abs(det) > 1e-300)) {
    gx = wx;
    gy = wy;
    gz = wz;
    return;
  }
  const inv = 1 / det;
  const dx =
    (rx * (a11 * a22 - a12 * a21) - a01 * (ry * a22 - a12 * rz) + a02 * (ry * a21 - a11 * rz)) *
    inv;
  const dy =
    (a00 * (ry * a22 - a12 * rz) - rx * (a10 * a22 - a12 * a20) + a02 * (a10 * rz - ry * a20)) *
    inv;
  const dz =
    (a00 * (a11 * rz - ry * a21) - a01 * (a10 * rz - ry * a20) + rx * (a10 * a21 - a11 * a20)) *
    inv;
  gx = wx + dx;
  gy = wy + dy;
  gz = wz + dz;
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
  const q = body.orientation;
  const s0 = body.stepStartAngular;
  const w = body.angularVelocity;
  if (s0.x === 0 && s0.y === 0 && s0.z === 0 && w.x === 0 && w.y === 0 && w.z === 0) {
    body.updateWorldInertia();
    return;
  }
  const { x: qx, y: qy, z: qz, w: qw } = q;
  const r00 = 1 - 2 * (qy * qy + qz * qz);
  const r01 = 2 * (qx * qy - qz * qw);
  const r02 = 2 * (qx * qz + qy * qw);
  const r10 = 2 * (qx * qy + qz * qw);
  const r11 = 1 - 2 * (qx * qx + qz * qz);
  const r12 = 2 * (qy * qz - qx * qw);
  const r20 = 2 * (qx * qz - qy * qw);
  const r21 = 2 * (qy * qz + qx * qw);
  const r22 = 1 - 2 * (qx * qx + qy * qy);
  const ex = w.x;
  const ey = w.y;
  const ez = w.z;
  // Body-frame start and end velocities (Rᵀ ω), averaged.
  const bx = r00 * ex + r10 * ey + r20 * ez;
  const by = r01 * ex + r11 * ey + r21 * ez;
  const bz = r02 * ex + r12 * ey + r22 * ez;
  const mx = 0.5 * (r00 * s0.x + r10 * s0.y + r20 * s0.z + bx);
  const my = 0.5 * (r01 * s0.x + r11 * s0.y + r21 * s0.z + by);
  const mz = 0.5 * (r02 * s0.x + r12 * s0.y + r22 * s0.z + bz);
  const speed = Math.sqrt(mx * mx + my * my + mz * mz);
  if (speed > 0) {
    const half = 0.5 * speed * dt;
    const s = Math.sin(half) / speed;
    const dx = mx * s;
    const dy = my * s;
    const dz = mz * s;
    const dw = Math.cos(half);
    // q ← normalize(q ⊗ dq)
    let a = qw * dx + qx * dw + qy * dz - qz * dy;
    let b = qw * dy - qx * dz + qy * dw + qz * dx;
    let c = qw * dz + qx * dy - qy * dx + qz * dw;
    let d = qw * dw - qx * dx - qy * dy - qz * dz;
    const len = Math.sqrt(a * a + b * b + c * c + d * d);
    if (len > 0) {
      const inv = 1 / len;
      a *= inv;
      b *= inv;
      c *= inv;
      d *= inv;
    }
    q.x = a;
    q.y = b;
    q.z = c;
    q.w = d;
    // Carry the body-frame end velocity into the new orientation: ω = R'·(Rᵀ ω_end).
    w.x = (1 - 2 * (b * b + c * c)) * bx + 2 * (a * b - c * d) * by + 2 * (a * c + b * d) * bz;
    w.y = 2 * (a * b + c * d) * bx + (1 - 2 * (a * a + c * c)) * by + 2 * (b * c - a * d) * bz;
    w.z = 2 * (a * c - b * d) * bx + 2 * (b * c + a * d) * by + (1 - 2 * (a * a + b * b)) * bz;
  }
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
    Q.set(dq, w.x * s, w.y * s, w.z * s, Math.cos(half));
    Q.mul(body.orientation, dq, body.orientation);
    Q.normalize(body.orientation, body.orientation);
  }
  body.updateWorldInertia();
}
