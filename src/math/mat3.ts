import { Vec3 } from './vec3';
import type { Quat } from './quat';

/** 3x3 matrix, row-major: e[3*row + col]. */
export class Mat3 {
  readonly e: Float64Array;
  constructor(e?: ArrayLike<number>) {
    this.e = new Float64Array(9);
    if (e) this.e.set(e);
    else this.e[0] = this.e[4] = this.e[8] = 1;
  }
  clone(): Mat3 {
    return new Mat3(this.e);
  }
  mul(b: Mat3): Mat3 {
    return mul(new Mat3(), this, b);
  }
  transpose(): Mat3 {
    return transpose(new Mat3(), this);
  }
  inverse(): Mat3 | null {
    const out = new Mat3();
    return invert(out, this) ? out : null;
  }
  mulVec(v: Vec3): Vec3 {
    return mulVec(new Vec3(), this, v);
  }
}

export function set(out: Mat3, a: ArrayLike<number>): Mat3 {
  out.e.set(a);
  return out;
}
export function copy(out: Mat3, a: Mat3): Mat3 {
  out.e.set(a.e);
  return out;
}
export function identity(out: Mat3): Mat3 {
  out.e.fill(0);
  out.e[0] = out.e[4] = out.e[8] = 1;
  return out;
}
export function fromDiagonal(out: Mat3, x: number, y: number, z: number): Mat3 {
  out.e.fill(0);
  out.e[0] = x;
  out.e[4] = y;
  out.e[8] = z;
  return out;
}
/** Matrix whose columns are the given vectors. */
export function fromColumns(out: Mat3, c0: Vec3, c1: Vec3, c2: Vec3): Mat3 {
  const e = out.e;
  e[0] = c0.x;
  e[3] = c0.y;
  e[6] = c0.z;
  e[1] = c1.x;
  e[4] = c1.y;
  e[7] = c1.z;
  e[2] = c2.x;
  e[5] = c2.y;
  e[8] = c2.z;
  return out;
}
export function getColumn(out: Vec3, m: Mat3, c: number): Vec3 {
  out.x = m.e[c] as number;
  out.y = m.e[3 + c] as number;
  out.z = m.e[6 + c] as number;
  return out;
}
/** Skew-symmetric matrix so that skew(v) * w = v × w. */
export function skew(out: Mat3, v: Vec3): Mat3 {
  const e = out.e;
  e[0] = 0;
  e[1] = -v.z;
  e[2] = v.y;
  e[3] = v.z;
  e[4] = 0;
  e[5] = -v.x;
  e[6] = -v.y;
  e[7] = v.x;
  e[8] = 0;
  return out;
}
export function fromQuat(out: Mat3, q: Quat): Mat3 {
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
  e[0] = 1 - (yy + zz);
  e[1] = xy - wz;
  e[2] = xz + wy;
  e[3] = xy + wz;
  e[4] = 1 - (xx + zz);
  e[5] = yz - wx;
  e[6] = xz - wy;
  e[7] = yz + wx;
  e[8] = 1 - (xx + yy);
  return out;
}
export function add(out: Mat3, a: Mat3, b: Mat3): Mat3 {
  for (let i = 0; i < 9; i++) out.e[i] = (a.e[i] as number) + (b.e[i] as number);
  return out;
}
export function scale(out: Mat3, a: Mat3, s: number): Mat3 {
  for (let i = 0; i < 9; i++) out.e[i] = (a.e[i] as number) * s;
  return out;
}
/** Safe when out aliases a or b. */
export function mul(out: Mat3, a: Mat3, b: Mat3): Mat3 {
  const A = a.e,
    B = b.e;
  const a0 = A[0] as number,
    a1 = A[1] as number,
    a2 = A[2] as number;
  const a3 = A[3] as number,
    a4 = A[4] as number,
    a5 = A[5] as number;
  const a6 = A[6] as number,
    a7 = A[7] as number,
    a8 = A[8] as number;
  const b0 = B[0] as number,
    b1 = B[1] as number,
    b2 = B[2] as number;
  const b3 = B[3] as number,
    b4 = B[4] as number,
    b5 = B[5] as number;
  const b6 = B[6] as number,
    b7 = B[7] as number,
    b8 = B[8] as number;
  const e = out.e;
  e[0] = a0 * b0 + a1 * b3 + a2 * b6;
  e[1] = a0 * b1 + a1 * b4 + a2 * b7;
  e[2] = a0 * b2 + a1 * b5 + a2 * b8;
  e[3] = a3 * b0 + a4 * b3 + a5 * b6;
  e[4] = a3 * b1 + a4 * b4 + a5 * b7;
  e[5] = a3 * b2 + a4 * b5 + a5 * b8;
  e[6] = a6 * b0 + a7 * b3 + a8 * b6;
  e[7] = a6 * b1 + a7 * b4 + a8 * b7;
  e[8] = a6 * b2 + a7 * b5 + a8 * b8;
  return out;
}
export function mulVec(out: Vec3, m: Mat3, v: Vec3): Vec3 {
  const e = m.e;
  const x = (e[0] as number) * v.x + (e[1] as number) * v.y + (e[2] as number) * v.z;
  const y = (e[3] as number) * v.x + (e[4] as number) * v.y + (e[5] as number) * v.z;
  const z = (e[6] as number) * v.x + (e[7] as number) * v.y + (e[8] as number) * v.z;
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
export function mulVecTransposed(out: Vec3, m: Mat3, v: Vec3): Vec3 {
  const e = m.e;
  const x = (e[0] as number) * v.x + (e[3] as number) * v.y + (e[6] as number) * v.z;
  const y = (e[1] as number) * v.x + (e[4] as number) * v.y + (e[7] as number) * v.z;
  const z = (e[2] as number) * v.x + (e[5] as number) * v.y + (e[8] as number) * v.z;
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
/** Safe when out aliases a. */
export function transpose(out: Mat3, a: Mat3): Mat3 {
  const e = a.e,
    o = out.e;
  const e1 = e[1] as number,
    e2 = e[2] as number,
    e5 = e[5] as number;
  o[0] = e[0] as number;
  o[4] = e[4] as number;
  o[8] = e[8] as number;
  o[1] = e[3] as number;
  o[2] = e[6] as number;
  o[5] = e[7] as number;
  o[3] = e1;
  o[6] = e2;
  o[7] = e5;
  return out;
}
export function determinant(m: Mat3): number {
  const e = m.e;
  const a = e[0] as number,
    b = e[1] as number,
    c = e[2] as number;
  const d = e[3] as number,
    f = e[4] as number,
    g = e[5] as number;
  const h = e[6] as number,
    i = e[7] as number,
    j = e[8] as number;
  return a * (f * j - g * i) - b * (d * j - g * h) + c * (d * i - f * h);
}
export function trace(m: Mat3): number {
  return (m.e[0] as number) + (m.e[4] as number) + (m.e[8] as number);
}
/** Inverts `a` into `out`. Returns false (leaving out untouched) if singular. Safe when out aliases a. */
export function invert(out: Mat3, a: Mat3): boolean {
  const e = a.e;
  const m0 = e[0] as number,
    m1 = e[1] as number,
    m2 = e[2] as number;
  const m3 = e[3] as number,
    m4 = e[4] as number,
    m5 = e[5] as number;
  const m6 = e[6] as number,
    m7 = e[7] as number,
    m8 = e[8] as number;
  const c0 = m4 * m8 - m5 * m7;
  const c1 = m5 * m6 - m3 * m8;
  const c2 = m3 * m7 - m4 * m6;
  const det = m0 * c0 + m1 * c1 + m2 * c2;
  if (det === 0 || !Number.isFinite(det)) return false;
  const inv = 1 / det;
  const o = out.e;
  o[0] = c0 * inv;
  o[1] = (m2 * m7 - m1 * m8) * inv;
  o[2] = (m1 * m5 - m2 * m4) * inv;
  o[3] = c1 * inv;
  o[4] = (m0 * m8 - m2 * m6) * inv;
  o[5] = (m2 * m3 - m0 * m5) * inv;
  o[6] = c2 * inv;
  o[7] = (m1 * m6 - m0 * m7) * inv;
  o[8] = (m0 * m4 - m1 * m3) * inv;
  return true;
}

const scratchA = new Float64Array(9);

/**
 * Symmetric eigen-decomposition by cyclic Jacobi rotations.
 * `m` must be symmetric. Eigenvalues go to `values` (unsorted) and unit eigenvectors
 * to the columns of `vectors`, so m = V diag(values) Vᵀ. Allocation-free.
 */
export function symmetricEigen(values: Vec3, vectors: Mat3, m: Mat3): void {
  const a = scratchA;
  a.set(m.e);
  const v = vectors.e;
  v.fill(0);
  v[0] = v[4] = v[8] = 1;
  for (let sweep = 0; sweep < 32; sweep++) {
    const off = Math.abs(a[1] as number) + Math.abs(a[2] as number) + Math.abs(a[5] as number);
    const diag = Math.abs(a[0] as number) + Math.abs(a[4] as number) + Math.abs(a[8] as number);
    if (off <= 1e-300 || off <= 1e-17 * diag) break;
    for (let p = 0; p < 2; p++) {
      for (let q = p + 1; q < 3; q++) {
        const apq = a[3 * p + q] as number;
        if (apq === 0) continue;
        const app = a[3 * p + p] as number;
        const aqq = a[3 * q + q] as number;
        const theta = (aqq - app) / (2 * apq);
        const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        // A <- Jᵀ A J on rows/cols p,q
        for (let k = 0; k < 3; k++) {
          const akp = a[3 * k + p] as number;
          const akq = a[3 * k + q] as number;
          a[3 * k + p] = c * akp - s * akq;
          a[3 * k + q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[3 * p + k] as number;
          const aqk = a[3 * q + k] as number;
          a[3 * p + k] = c * apk - s * aqk;
          a[3 * q + k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = v[3 * k + p] as number;
          const vkq = v[3 * k + q] as number;
          v[3 * k + p] = c * vkp - s * vkq;
          v[3 * k + q] = s * vkp + c * vkq;
        }
      }
    }
  }
  values.x = a[0] as number;
  values.y = a[4] as number;
  values.z = a[8] as number;
}
