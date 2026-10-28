import { Vec3 } from './vec3';
import { Quat, rotate, rotateInverse, mul as qmul, invert as qinvert } from './quat';

const scratchQ = new Quat();

/** Rigid transform: rotate by `q`, then translate by `p`. */
export class Transform {
  constructor(
    public p = new Vec3(),
    public q = new Quat(),
  ) {}
  clone(): Transform {
    return new Transform(this.p.clone(), this.q.clone());
  }
}

export function identity(out: Transform): Transform {
  out.p.x = out.p.y = out.p.z = 0;
  out.q.x = out.q.y = out.q.z = 0;
  out.q.w = 1;
  return out;
}
export function copy(out: Transform, a: Transform): Transform {
  out.p.x = a.p.x;
  out.p.y = a.p.y;
  out.p.z = a.p.z;
  out.q.x = a.q.x;
  out.q.y = a.q.y;
  out.q.z = a.q.z;
  out.q.w = a.q.w;
  return out;
}
/** out = t * point (rotate then translate). Safe when out aliases point. */
export function apply(out: Vec3, t: Transform, point: Vec3): Vec3 {
  rotate(out, t.q, point);
  out.x += t.p.x;
  out.y += t.p.y;
  out.z += t.p.z;
  return out;
}
/** out = t⁻¹ * point. Safe when out aliases point. */
export function applyInverse(out: Vec3, t: Transform, point: Vec3): Vec3 {
  const x = point.x - t.p.x;
  const y = point.y - t.p.y;
  const z = point.z - t.p.z;
  out.x = x;
  out.y = y;
  out.z = z;
  return rotateInverse(out, t.q, out);
}
/** Rotation only (for directions). */
export function applyDirection(out: Vec3, t: Transform, d: Vec3): Vec3 {
  return rotate(out, t.q, d);
}
export function applyInverseDirection(out: Vec3, t: Transform, d: Vec3): Vec3 {
  return rotateInverse(out, t.q, d);
}
/** out = a * b (apply b first, then a). Safe when out aliases a or b. */
export function mul(out: Transform, a: Transform, b: Transform): Transform {
  const x = a.q.x,
    y = a.q.y,
    z = a.q.z,
    w = a.q.w;
  const px = a.p.x,
    py = a.p.y,
    pz = a.p.z;
  const bpx = b.p.x,
    bpy = b.p.y,
    bpz = b.p.z;
  // translation = a.q * b.p + a.p, using a's quaternion values captured above
  const tx = 2 * (y * bpz - z * bpy);
  const ty = 2 * (z * bpx - x * bpz);
  const tz = 2 * (x * bpy - y * bpx);
  const rx = bpx + w * tx + (y * tz - z * ty);
  const ry = bpy + w * ty + (z * tx - x * tz);
  const rz = bpz + w * tz + (x * ty - y * tx);
  qmul(out.q, a.q, b.q);
  out.p.x = rx + px;
  out.p.y = ry + py;
  out.p.z = rz + pz;
  return out;
}
/** Safe when out aliases a. */
export function invert(out: Transform, a: Transform): Transform {
  const px = -a.p.x,
    py = -a.p.y,
    pz = -a.p.z;
  qinvert(out.q, a.q);
  out.p.x = px;
  out.p.y = py;
  out.p.z = pz;
  rotate(out.p, out.q, out.p);
  return out;
}
/** out = a⁻¹ * b: b expressed in a's frame. */
export function mulInverse(out: Transform, a: Transform, b: Transform): Transform {
  const dx = b.p.x - a.p.x,
    dy = b.p.y - a.p.y,
    dz = b.p.z - a.p.z;
  const iq = qinvert(scratchQ, a.q);
  qmul(out.q, iq, b.q);
  out.p.x = dx;
  out.p.y = dy;
  out.p.z = dz;
  rotate(out.p, iq, out.p);
  return out;
}
