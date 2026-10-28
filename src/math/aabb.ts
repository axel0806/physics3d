import { Vec3 } from './vec3';
import { Quat } from './quat';

/** Axis-aligned bounding box. */
export class AABB {
  constructor(
    public min = new Vec3(Infinity, Infinity, Infinity),
    public max = new Vec3(-Infinity, -Infinity, -Infinity),
  ) {}
  clone(): AABB {
    return new AABB(this.min.clone(), this.max.clone());
  }
}

export function set(
  out: AABB,
  minx: number,
  miny: number,
  minz: number,
  maxx: number,
  maxy: number,
  maxz: number,
): AABB {
  out.min.x = minx;
  out.min.y = miny;
  out.min.z = minz;
  out.max.x = maxx;
  out.max.y = maxy;
  out.max.z = maxz;
  return out;
}
export function copy(out: AABB, a: AABB): AABB {
  return set(out, a.min.x, a.min.y, a.min.z, a.max.x, a.max.y, a.max.z);
}
/** Reset to the empty box (unions with it are the identity). */
export function empty(out: AABB): AABB {
  return set(out, Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity);
}
export function isValid(a: AABB): boolean {
  return a.min.x <= a.max.x && a.min.y <= a.max.y && a.min.z <= a.max.z;
}
export function union(out: AABB, a: AABB, b: AABB): AABB {
  return set(
    out,
    Math.min(a.min.x, b.min.x),
    Math.min(a.min.y, b.min.y),
    Math.min(a.min.z, b.min.z),
    Math.max(a.max.x, b.max.x),
    Math.max(a.max.y, b.max.y),
    Math.max(a.max.z, b.max.z),
  );
}
export function expandPoint(out: AABB, p: Vec3): AABB {
  out.min.x = Math.min(out.min.x, p.x);
  out.min.y = Math.min(out.min.y, p.y);
  out.min.z = Math.min(out.min.z, p.z);
  out.max.x = Math.max(out.max.x, p.x);
  out.max.y = Math.max(out.max.y, p.y);
  out.max.z = Math.max(out.max.z, p.z);
  return out;
}
/** Grow by `margin` on every side. */
export function fatten(out: AABB, a: AABB, margin: number): AABB {
  return set(
    out,
    a.min.x - margin,
    a.min.y - margin,
    a.min.z - margin,
    a.max.x + margin,
    a.max.y + margin,
    a.max.z + margin,
  );
}
export function overlaps(a: AABB, b: AABB): boolean {
  return (
    a.min.x <= b.max.x &&
    a.max.x >= b.min.x &&
    a.min.y <= b.max.y &&
    a.max.y >= b.min.y &&
    a.min.z <= b.max.z &&
    a.max.z >= b.min.z
  );
}
/** True if `outer` fully contains `inner`. */
export function contains(outer: AABB, inner: AABB): boolean {
  return (
    outer.min.x <= inner.min.x &&
    outer.max.x >= inner.max.x &&
    outer.min.y <= inner.min.y &&
    outer.max.y >= inner.max.y &&
    outer.min.z <= inner.min.z &&
    outer.max.z >= inner.max.z
  );
}
export function containsPoint(a: AABB, p: Vec3): boolean {
  return (
    p.x >= a.min.x &&
    p.x <= a.max.x &&
    p.y >= a.min.y &&
    p.y <= a.max.y &&
    p.z >= a.min.z &&
    p.z <= a.max.z
  );
}
export function center(out: Vec3, a: AABB): Vec3 {
  out.x = 0.5 * (a.min.x + a.max.x);
  out.y = 0.5 * (a.min.y + a.max.y);
  out.z = 0.5 * (a.min.z + a.max.z);
  return out;
}
export function halfExtents(out: Vec3, a: AABB): Vec3 {
  out.x = 0.5 * (a.max.x - a.min.x);
  out.y = 0.5 * (a.max.y - a.min.y);
  out.z = 0.5 * (a.max.z - a.min.z);
  return out;
}
export function surfaceArea(a: AABB): number {
  const dx = a.max.x - a.min.x;
  const dy = a.max.y - a.min.y;
  const dz = a.max.z - a.min.z;
  return 2 * (dx * dy + dy * dz + dz * dx);
}
export function volume(a: AABB): number {
  return (a.max.x - a.min.x) * (a.max.y - a.min.y) * (a.max.z - a.min.z);
}
/** Bounds of a box (center c, half extents h) rotated by q. Safe when out aliases nothing else. */
export function fromOrientedBox(out: AABB, c: Vec3, h: Vec3, q: Quat): AABB {
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
  const ex = Math.abs(1 - (yy + zz)) * h.x + Math.abs(xy - wz) * h.y + Math.abs(xz + wy) * h.z;
  const ey = Math.abs(xy + wz) * h.x + Math.abs(1 - (xx + zz)) * h.y + Math.abs(yz - wx) * h.z;
  const ez = Math.abs(xz - wy) * h.x + Math.abs(yz + wx) * h.y + Math.abs(1 - (xx + yy)) * h.z;
  return set(out, c.x - ex, c.y - ey, c.z - ez, c.x + ex, c.y + ey, c.z + ez);
}
// Slab clip state shared by `slab` (module-level to stay allocation-free).
let slabMin = 0;
let slabMax = 0;

/** Clips [slabMin, slabMax] against one axis slab. Returns false if the ray misses it. */
function slab(o: number, d: number, lo: number, hi: number): boolean {
  if (d === 0) return o >= lo && o <= hi;
  const inv = 1 / d;
  let t1 = (lo - o) * inv;
  let t2 = (hi - o) * inv;
  if (t1 > t2) {
    const t = t1;
    t1 = t2;
    t2 = t;
  }
  if (t1 > slabMin) slabMin = t1;
  if (t2 < slabMax) slabMax = t2;
  return slabMin <= slabMax;
}

/**
 * Slab ray test. Returns entry parameter t in [0, maxT], or -1 on miss.
 * An origin inside the box returns 0.
 */
export function raycast(
  a: AABB,
  ox: number,
  oy: number,
  oz: number,
  dx: number,
  dy: number,
  dz: number,
  maxT: number,
): number {
  slabMin = 0;
  slabMax = maxT;
  if (!slab(ox, dx, a.min.x, a.max.x)) return -1;
  if (!slab(oy, dy, a.min.y, a.max.y)) return -1;
  if (!slab(oz, dz, a.min.z, a.max.z)) return -1;
  return slabMin;
}
