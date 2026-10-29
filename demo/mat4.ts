/** Column-major 4x4 matrices in Float32Array(16), as WebGL expects. */
export type Mat4 = Float32Array;

export const create = (): Mat4 => identity(new Float32Array(16));

export function identity(m: Mat4): Mat4 {
  m.fill(0);
  m[0] = m[5] = m[10] = m[15] = 1;
  return m;
}

/** out = a * b (apply b first). `out` may alias neither input. */
export function multiply(out: Mat4, a: Mat4, b: Mat4): Mat4 {
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += (a[k * 4 + r] as number) * (b[c * 4 + k] as number);
      out[c * 4 + r] = s;
    }
  }
  return out;
}

export function perspective(
  out: Mat4,
  fovy: number,
  aspect: number,
  near: number,
  far: number,
): Mat4 {
  const f = 1 / Math.tan(fovy / 2);
  out.fill(0);
  out[0] = f / aspect;
  out[5] = f;
  out[10] = (far + near) / (near - far);
  out[11] = -1;
  out[14] = (2 * far * near) / (near - far);
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
  out.fill(0);
  out[0] = 2 / (r - l);
  out[5] = 2 / (t - b);
  out[10] = -2 / (f - n);
  out[12] = -(r + l) / (r - l);
  out[13] = -(t + b) / (t - b);
  out[14] = -(f + n) / (f - n);
  out[15] = 1;
  return out;
}

export function lookAt(
  out: Mat4,
  eye: readonly number[],
  target: readonly number[],
  up: readonly number[],
): Mat4 {
  let zx = (eye[0] as number) - (target[0] as number);
  let zy = (eye[1] as number) - (target[1] as number);
  let zz = (eye[2] as number) - (target[2] as number);
  let l = Math.hypot(zx, zy, zz) || 1;
  zx /= l;
  zy /= l;
  zz /= l;
  let xx = (up[1] as number) * zz - (up[2] as number) * zy;
  let xy = (up[2] as number) * zx - (up[0] as number) * zz;
  let xz = (up[0] as number) * zy - (up[1] as number) * zx;
  l = Math.hypot(xx, xy, xz) || 1;
  xx /= l;
  xy /= l;
  xz /= l;
  const yx = zy * xz - zz * xy;
  const yy = zz * xx - zx * xz;
  const yz = zx * xy - zy * xx;
  out[0] = xx;
  out[1] = yx;
  out[2] = zx;
  out[3] = 0;
  out[4] = xy;
  out[5] = yy;
  out[6] = zy;
  out[7] = 0;
  out[8] = xz;
  out[9] = yz;
  out[10] = zz;
  out[11] = 0;
  out[12] = -(xx * (eye[0] as number) + xy * (eye[1] as number) + xz * (eye[2] as number));
  out[13] = -(yx * (eye[0] as number) + yy * (eye[1] as number) + yz * (eye[2] as number));
  out[14] = -(zx * (eye[0] as number) + zy * (eye[1] as number) + zz * (eye[2] as number));
  out[15] = 1;
  return out;
}

/**
 * Model matrix from a unit quaternion (x, y, z, w), a translation and a non-uniform scale, written
 * into `out` at `offset` (so instance data can be packed straight into one array).
 */
export function compose(
  out: Float32Array,
  offset: number,
  px: number,
  py: number,
  pz: number,
  qx: number,
  qy: number,
  qz: number,
  qw: number,
  sx: number,
  sy: number,
  sz: number,
): void {
  const x2 = qx + qx;
  const y2 = qy + qy;
  const z2 = qz + qz;
  const xx = qx * x2;
  const xy = qx * y2;
  const xz = qx * z2;
  const yy = qy * y2;
  const yz = qy * z2;
  const zz = qz * z2;
  const wx = qw * x2;
  const wy = qw * y2;
  const wz = qw * z2;
  out[offset] = (1 - (yy + zz)) * sx;
  out[offset + 1] = (xy + wz) * sx;
  out[offset + 2] = (xz - wy) * sx;
  out[offset + 3] = 0;
  out[offset + 4] = (xy - wz) * sy;
  out[offset + 5] = (1 - (xx + zz)) * sy;
  out[offset + 6] = (yz + wx) * sy;
  out[offset + 7] = 0;
  out[offset + 8] = (xz + wy) * sz;
  out[offset + 9] = (yz - wx) * sz;
  out[offset + 10] = (1 - (xx + yy)) * sz;
  out[offset + 11] = 0;
  out[offset + 12] = px;
  out[offset + 13] = py;
  out[offset + 14] = pz;
  out[offset + 15] = 1;
}

/** Inverse of a general 4x4 (returns false if singular). */
export function invert(out: Mat4, m: Mat4): boolean {
  const a = m;
  const b00 = (a[0] as number) * (a[5] as number) - (a[1] as number) * (a[4] as number);
  const b01 = (a[0] as number) * (a[6] as number) - (a[2] as number) * (a[4] as number);
  const b02 = (a[0] as number) * (a[7] as number) - (a[3] as number) * (a[4] as number);
  const b03 = (a[1] as number) * (a[6] as number) - (a[2] as number) * (a[5] as number);
  const b04 = (a[1] as number) * (a[7] as number) - (a[3] as number) * (a[5] as number);
  const b05 = (a[2] as number) * (a[7] as number) - (a[3] as number) * (a[6] as number);
  const b06 = (a[8] as number) * (a[13] as number) - (a[9] as number) * (a[12] as number);
  const b07 = (a[8] as number) * (a[14] as number) - (a[10] as number) * (a[12] as number);
  const b08 = (a[8] as number) * (a[15] as number) - (a[11] as number) * (a[12] as number);
  const b09 = (a[9] as number) * (a[14] as number) - (a[10] as number) * (a[13] as number);
  const b10 = (a[9] as number) * (a[15] as number) - (a[11] as number) * (a[13] as number);
  const b11 = (a[10] as number) * (a[15] as number) - (a[11] as number) * (a[14] as number);
  let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
  if (!det) return false;
  det = 1 / det;
  const g = (i: number) => a[i] as number;
  out[0] = (g(5) * b11 - g(6) * b10 + g(7) * b09) * det;
  out[1] = (g(2) * b10 - g(1) * b11 - g(3) * b09) * det;
  out[2] = (g(13) * b05 - g(14) * b04 + g(15) * b03) * det;
  out[3] = (g(10) * b04 - g(9) * b05 - g(11) * b03) * det;
  out[4] = (g(6) * b08 - g(4) * b11 - g(7) * b07) * det;
  out[5] = (g(0) * b11 - g(2) * b08 + g(3) * b07) * det;
  out[6] = (g(14) * b02 - g(12) * b05 - g(15) * b01) * det;
  out[7] = (g(8) * b05 - g(10) * b02 + g(11) * b01) * det;
  out[8] = (g(4) * b10 - g(5) * b08 + g(7) * b06) * det;
  out[9] = (g(1) * b08 - g(0) * b10 - g(3) * b06) * det;
  out[10] = (g(12) * b04 - g(13) * b02 + g(15) * b00) * det;
  out[11] = (g(9) * b02 - g(8) * b04 - g(11) * b00) * det;
  out[12] = (g(5) * b07 - g(4) * b09 - g(6) * b06) * det;
  out[13] = (g(0) * b09 - g(1) * b07 + g(2) * b06) * det;
  out[14] = (g(13) * b01 - g(12) * b03 - g(14) * b00) * det;
  out[15] = (g(8) * b03 - g(9) * b01 + g(10) * b00) * det;
  return true;
}

/** Transforms a point (w = 1) and returns the homogeneous result. */
export function transformPoint(
  m: Mat4,
  x: number,
  y: number,
  z: number,
): [number, number, number, number] {
  const g = (i: number) => m[i] as number;
  return [
    g(0) * x + g(4) * y + g(8) * z + g(12),
    g(1) * x + g(5) * y + g(9) * z + g(13),
    g(2) * x + g(6) * y + g(10) * z + g(14),
    g(3) * x + g(7) * y + g(11) * z + g(15),
  ];
}
