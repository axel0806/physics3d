import { Vec3 } from './vec3';
import type { Quat } from './quat';

/** 4x4 matrix, column-major (WebGL layout): e[4*col + row]. */
export class Mat4 {
  readonly e: Float64Array;
  constructor(e?: ArrayLike<number>) {
    this.e = new Float64Array(16);
    if (e) this.e.set(e);
    else this.e[0] = this.e[5] = this.e[10] = this.e[15] = 1;
  }
  clone(): Mat4 {
    return new Mat4(this.e);
  }
}

export function identity(out: Mat4): Mat4 {
  out.e.fill(0);
  out.e[0] = out.e[5] = out.e[10] = out.e[15] = 1;
  return out;
}
export function copy(out: Mat4, a: Mat4): Mat4 {
  out.e.set(a.e);
  return out;
}
/** Safe when out aliases a or b. */
export function mul(out: Mat4, a: Mat4, b: Mat4): Mat4 {
  const A = a.e,
    B = b.e;
  const r = scratch;
  for (let c = 0; c < 4; c++) {
    for (let row = 0; row < 4; row++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += (A[4 * k + row] as number) * (B[4 * c + k] as number);
      r[4 * c + row] = s;
    }
  }
  out.e.set(r);
  return out;
}
const scratch = new Float64Array(16);

export function fromTranslation(out: Mat4, x: number, y: number, z: number): Mat4 {
  identity(out);
  out.e[12] = x;
  out.e[13] = y;
  out.e[14] = z;
  return out;
}
export function fromScale(out: Mat4, x: number, y: number, z: number): Mat4 {
  identity(out);
  out.e[0] = x;
  out.e[5] = y;
  out.e[10] = z;
  return out;
}
/** Rotation q, then translation p, with optional non-uniform scale applied first. */
export function compose(out: Mat4, p: Vec3, q: Quat, sx = 1, sy = 1, sz = 1): Mat4 {
  const { x, y, z, w } = q;
  const x2 = x + x,
    y2 = y + y,
    z2 = z + z;
  const xx = x * x2,
    xy = x * y2,
    xz = x * z2;
  const yy = y * y2,
    yz = y * z2,
    zz = z * z2;
  const wx = w * x2,
    wy = w * y2,
    wz = w * z2;
  const e = out.e;
  e[0] = (1 - (yy + zz)) * sx;
  e[1] = (xy + wz) * sx;
  e[2] = (xz - wy) * sx;
  e[3] = 0;
  e[4] = (xy - wz) * sy;
  e[5] = (1 - (xx + zz)) * sy;
  e[6] = (yz + wx) * sy;
  e[7] = 0;
  e[8] = (xz + wy) * sz;
  e[9] = (yz - wx) * sz;
  e[10] = (1 - (xx + yy)) * sz;
  e[11] = 0;
  e[12] = p.x;
  e[13] = p.y;
  e[14] = p.z;
  e[15] = 1;
  return out;
}
/** Right-handed perspective, clip z in [-1, 1] (WebGL). */
export function perspective(
  out: Mat4,
  fovY: number,
  aspect: number,
  near: number,
  far: number,
): Mat4 {
  const f = 1 / Math.tan(fovY / 2);
  const nf = 1 / (near - far);
  out.e.fill(0);
  out.e[0] = f / aspect;
  out.e[5] = f;
  out.e[10] = (far + near) * nf;
  out.e[11] = -1;
  out.e[14] = 2 * far * near * nf;
  return out;
}
export function ortho(
  out: Mat4,
  l: number,
  r: number,
  b: number,
  t: number,
  n: number,
  f: number,
): Mat4 {
  out.e.fill(0);
  out.e[0] = 2 / (r - l);
  out.e[5] = 2 / (t - b);
  out.e[10] = -2 / (f - n);
  out.e[12] = -(r + l) / (r - l);
  out.e[13] = -(t + b) / (t - b);
  out.e[14] = -(f + n) / (f - n);
  out.e[15] = 1;
  return out;
}
/** Right-handed view matrix looking from `eye` toward `target`. */
export function lookAt(out: Mat4, eye: Vec3, target: Vec3, up: Vec3): Mat4 {
  let zx = eye.x - target.x,
    zy = eye.y - target.y,
    zz = eye.z - target.z;
  let l = Math.hypot(zx, zy, zz);
  zx /= l;
  zy /= l;
  zz /= l;
  let xx = up.y * zz - up.z * zy,
    xy = up.z * zx - up.x * zz,
    xz = up.x * zy - up.y * zx;
  l = Math.hypot(xx, xy, xz);
  xx /= l;
  xy /= l;
  xz /= l;
  const yx = zy * xz - zz * xy,
    yy = zz * xx - zx * xz,
    yz = zx * xy - zy * xx;
  const e = out.e;
  e[0] = xx;
  e[1] = yx;
  e[2] = zx;
  e[3] = 0;
  e[4] = xy;
  e[5] = yy;
  e[6] = zy;
  e[7] = 0;
  e[8] = xz;
  e[9] = yz;
  e[10] = zz;
  e[11] = 0;
  e[12] = -(xx * eye.x + xy * eye.y + xz * eye.z);
  e[13] = -(yx * eye.x + yy * eye.y + yz * eye.z);
  e[14] = -(zx * eye.x + zy * eye.y + zz * eye.z);
  e[15] = 1;
  return out;
}
export function transpose(out: Mat4, a: Mat4): Mat4 {
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) scratch[4 * c + r] = a.e[4 * r + c] as number;
  }
  out.e.set(scratch);
  return out;
}
/** General 4x4 inverse (cofactor expansion). Returns false if singular. Safe when out aliases a. */
export function invert(out: Mat4, a: Mat4): boolean {
  const m = a.e;
  const a00 = m[0] as number,
    a01 = m[1] as number,
    a02 = m[2] as number,
    a03 = m[3] as number;
  const a10 = m[4] as number,
    a11 = m[5] as number,
    a12 = m[6] as number,
    a13 = m[7] as number;
  const a20 = m[8] as number,
    a21 = m[9] as number,
    a22 = m[10] as number,
    a23 = m[11] as number;
  const a30 = m[12] as number,
    a31 = m[13] as number,
    a32 = m[14] as number,
    a33 = m[15] as number;
  const b00 = a00 * a11 - a01 * a10,
    b01 = a00 * a12 - a02 * a10;
  const b02 = a00 * a13 - a03 * a10,
    b03 = a01 * a12 - a02 * a11;
  const b04 = a01 * a13 - a03 * a11,
    b05 = a02 * a13 - a03 * a12;
  const b06 = a20 * a31 - a21 * a30,
    b07 = a20 * a32 - a22 * a30;
  const b08 = a20 * a33 - a23 * a30,
    b09 = a21 * a32 - a22 * a31;
  const b10 = a21 * a33 - a23 * a31,
    b11 = a22 * a33 - a23 * a32;
  const det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
  if (det === 0 || !Number.isFinite(det)) return false;
  const inv = 1 / det;
  const o = out.e;
  o[0] = (a11 * b11 - a12 * b10 + a13 * b09) * inv;
  o[1] = (a02 * b10 - a01 * b11 - a03 * b09) * inv;
  o[2] = (a31 * b05 - a32 * b04 + a33 * b03) * inv;
  o[3] = (a22 * b04 - a21 * b05 - a23 * b03) * inv;
  o[4] = (a12 * b08 - a10 * b11 - a13 * b07) * inv;
  o[5] = (a00 * b11 - a02 * b08 + a03 * b07) * inv;
  o[6] = (a32 * b02 - a30 * b05 - a33 * b01) * inv;
  o[7] = (a20 * b05 - a22 * b02 + a23 * b01) * inv;
  o[8] = (a10 * b10 - a11 * b08 + a13 * b06) * inv;
  o[9] = (a01 * b08 - a00 * b10 - a03 * b06) * inv;
  o[10] = (a30 * b04 - a31 * b02 + a33 * b00) * inv;
  o[11] = (a21 * b02 - a20 * b04 - a23 * b00) * inv;
  o[12] = (a11 * b07 - a10 * b09 - a12 * b06) * inv;
  o[13] = (a00 * b09 - a01 * b07 + a02 * b06) * inv;
  o[14] = (a31 * b01 - a30 * b03 - a32 * b00) * inv;
  o[15] = (a20 * b03 - a21 * b01 + a22 * b00) * inv;
  return true;
}
/** Transforms a point (w = 1) with perspective divide skipped (affine use). */
export function transformPoint(out: Vec3, m: Mat4, v: Vec3): Vec3 {
  const e = m.e;
  const x =
    (e[0] as number) * v.x + (e[4] as number) * v.y + (e[8] as number) * v.z + (e[12] as number);
  const y =
    (e[1] as number) * v.x + (e[5] as number) * v.y + (e[9] as number) * v.z + (e[13] as number);
  const z =
    (e[2] as number) * v.x + (e[6] as number) * v.y + (e[10] as number) * v.z + (e[14] as number);
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
/** Writes into a Float32Array (e.g. an instance buffer) at `offset`. */
export function toFloat32(out: Float32Array, m: Mat4, offset = 0): void {
  for (let i = 0; i < 16; i++) out[offset + i] = m.e[i] as number;
}
