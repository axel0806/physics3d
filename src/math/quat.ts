import { Vec3 } from './vec3';
import { Mat3 } from './mat3';

/** Unit quaternion (x, y, z, w). */
export class Quat {
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public w = 1,
  ) {}

  clone(): Quat {
    return new Quat(this.x, this.y, this.z, this.w);
  }
  mul(b: Quat): Quat {
    return mul(new Quat(), this, b);
  }
  inverse(): Quat {
    return invert(new Quat(), this);
  }
  rotate(v: Vec3): Vec3 {
    return rotate(new Vec3(), this, v);
  }
  normalized(): Quat {
    return normalize(new Quat(), this);
  }
}

export function set(out: Quat, x: number, y: number, z: number, w: number): Quat {
  out.x = x;
  out.y = y;
  out.z = z;
  out.w = w;
  return out;
}
export function copy(out: Quat, a: Quat): Quat {
  return set(out, a.x, a.y, a.z, a.w);
}
export function identity(out: Quat): Quat {
  return set(out, 0, 0, 0, 1);
}
export function dot(a: Quat, b: Quat): number {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
/** Hamilton product a*b. Safe when out aliases a or b. */
export function mul(out: Quat, a: Quat, b: Quat): Quat {
  const x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  const y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  const z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  const w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  return set(out, x, y, z, w);
}
export function conjugate(out: Quat, a: Quat): Quat {
  return set(out, -a.x, -a.y, -a.z, a.w);
}
/** True inverse (conjugate / |q|^2), valid for non-unit quaternions too. */
export function invert(out: Quat, a: Quat): Quat {
  const n = dot(a, a);
  const inv = n === 0 ? 0 : 1 / n;
  return set(out, -a.x * inv, -a.y * inv, -a.z * inv, a.w * inv);
}
/** Normalizes; a zero quaternion becomes identity. */
export function normalize(out: Quat, a: Quat): Quat {
  const n = Math.sqrt(dot(a, a));
  if (n === 0) return identity(out);
  const inv = 1 / n;
  return set(out, a.x * inv, a.y * inv, a.z * inv, a.w * inv);
}
export function fromAxisAngle(out: Quat, axis: Vec3, angle: number): Quat {
  const s = Math.sin(angle * 0.5);
  return set(out, axis.x * s, axis.y * s, axis.z * s, Math.cos(angle * 0.5));
}
/** Rotates v by unit quaternion q. Safe when out aliases v. */
export function rotate(out: Vec3, q: Quat, v: Vec3): Vec3 {
  // t = 2 * (q.xyz × v); out = v + w*t + q.xyz × t
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  const x = v.x + q.w * tx + (q.y * tz - q.z * ty);
  const y = v.y + q.w * ty + (q.z * tx - q.x * tz);
  const z = v.z + q.w * tz + (q.x * ty - q.y * tx);
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
/** Rotates v by the inverse of unit quaternion q. */
export function rotateInverse(out: Vec3, q: Quat, v: Vec3): Vec3 {
  const tx = 2 * (-q.y * v.z + q.z * v.y);
  const ty = 2 * (-q.z * v.x + q.x * v.z);
  const tz = 2 * (-q.x * v.y + q.y * v.x);
  const x = v.x + q.w * tx + (-q.y * tz + q.z * ty);
  const y = v.y + q.w * ty + (-q.z * tx + q.x * tz);
  const z = v.z + q.w * tz + (-q.x * ty + q.y * tx);
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
/** q' = normalize(q + 0.5*dt*(0,ω)*q), ω in world space. Safe when out aliases q. */
export function integrate(out: Quat, q: Quat, omega: Vec3, dt: number): Quat {
  const h = 0.5 * dt;
  const x = q.x + h * (omega.x * q.w + omega.y * q.z - omega.z * q.y);
  const y = q.y + h * (omega.y * q.w + omega.z * q.x - omega.x * q.z);
  const z = q.z + h * (omega.z * q.w + omega.x * q.y - omega.y * q.x);
  const w = q.w + h * (-omega.x * q.x - omega.y * q.y - omega.z * q.z);
  return normalize(out, set(out, x, y, z, w));
}
/** Shortest-arc spherical interpolation; falls back to nlerp for near-parallel inputs. */
export function slerp(out: Quat, a: Quat, b: Quat, t: number): Quat {
  let d = dot(a, b);
  let bx = b.x;
  let by = b.y;
  let bz = b.z;
  let bw = b.w;
  if (d < 0) {
    d = -d;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }
  let s0: number;
  let s1: number;
  if (d > 0.9995) {
    s0 = 1 - t;
    s1 = t;
  } else {
    const theta = Math.acos(d);
    const inv = 1 / Math.sin(theta);
    s0 = Math.sin((1 - t) * theta) * inv;
    s1 = Math.sin(t * theta) * inv;
  }
  return normalize(
    out,
    set(out, s0 * a.x + s1 * bx, s0 * a.y + s1 * by, s0 * a.z + s1 * bz, s0 * a.w + s1 * bw),
  );
}
/** Rotation matrix to quaternion (Shepperd's method). */
export function fromMat3(out: Quat, m: Mat3): Quat {
  const e = m.e;
  const m00 = e[0] as number,
    m01 = e[1] as number,
    m02 = e[2] as number;
  const m10 = e[3] as number,
    m11 = e[4] as number,
    m12 = e[5] as number;
  const m20 = e[6] as number,
    m21 = e[7] as number,
    m22 = e[8] as number;
  const tr = m00 + m11 + m22;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    set(out, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s);
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    set(out, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    set(out, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s);
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    set(out, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s);
  }
  return normalize(out, out);
}
/** Shortest rotation taking unit vector `from` to unit vector `to`. */
export function fromUnitVectors(out: Quat, from: Vec3, to: Vec3): Quat {
  const d = from.x * to.x + from.y * to.y + from.z * to.z;
  if (d < -1 + 1e-12) {
    // 180°: pick any axis perpendicular to `from`.
    let ax = -from.y;
    let ay = from.x;
    let az = 0;
    if (Math.abs(from.x) < 0.57735) {
      ax = 0;
      ay = -from.z;
      az = from.y;
    }
    const l = Math.sqrt(ax * ax + ay * ay + az * az);
    return set(out, ax / l, ay / l, az / l, 0);
  }
  const cx = from.y * to.z - from.z * to.y;
  const cy = from.z * to.x - from.x * to.z;
  const cz = from.x * to.y - from.y * to.x;
  return normalize(out, set(out, cx, cy, cz, 1 + d));
}
