/** Mutable 3-vector. Free functions take an `out` parameter and never allocate. */
export class Vec3 {
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
  ) {}

  clone(): Vec3 {
    return new Vec3(this.x, this.y, this.z);
  }
  add(b: Vec3): Vec3 {
    return add(new Vec3(), this, b);
  }
  sub(b: Vec3): Vec3 {
    return sub(new Vec3(), this, b);
  }
  scale(s: number): Vec3 {
    return scale(new Vec3(), this, s);
  }
  cross(b: Vec3): Vec3 {
    return cross(new Vec3(), this, b);
  }
  dot(b: Vec3): number {
    return dot(this, b);
  }
  length(): number {
    return length(this);
  }
  normalized(): Vec3 {
    return normalize(new Vec3(), this);
  }
  negated(): Vec3 {
    return negate(new Vec3(), this);
  }
}

export function set(out: Vec3, x: number, y: number, z: number): Vec3 {
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
export function copy(out: Vec3, a: Vec3): Vec3 {
  out.x = a.x;
  out.y = a.y;
  out.z = a.z;
  return out;
}
export function add(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  out.x = a.x + b.x;
  out.y = a.y + b.y;
  out.z = a.z + b.z;
  return out;
}
export function sub(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  out.z = a.z - b.z;
  return out;
}
export function scale(out: Vec3, a: Vec3, s: number): Vec3 {
  out.x = a.x * s;
  out.y = a.y * s;
  out.z = a.z * s;
  return out;
}
/** out = a + b * s */
export function addScaled(out: Vec3, a: Vec3, b: Vec3, s: number): Vec3 {
  out.x = a.x + b.x * s;
  out.y = a.y + b.y * s;
  out.z = a.z + b.z * s;
  return out;
}
export function negate(out: Vec3, a: Vec3): Vec3 {
  out.x = -a.x;
  out.y = -a.y;
  out.z = -a.z;
  return out;
}
export function dot(a: Vec3, b: Vec3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
/** Safe when out aliases a or b. */
export function cross(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  const x = a.y * b.z - a.z * b.y;
  const y = a.z * b.x - a.x * b.z;
  const z = a.x * b.y - a.y * b.x;
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}
export function lengthSq(a: Vec3): number {
  return a.x * a.x + a.y * a.y + a.z * a.z;
}
export function length(a: Vec3): number {
  return Math.sqrt(lengthSq(a));
}
export function distanceSq(a: Vec3, b: Vec3): number {
  const x = a.x - b.x;
  const y = a.y - b.y;
  const z = a.z - b.z;
  return x * x + y * y + z * z;
}
export function distance(a: Vec3, b: Vec3): number {
  return Math.sqrt(distanceSq(a, b));
}
/** Normalizes; a zero-length input yields the zero vector. */
export function normalize(out: Vec3, a: Vec3): Vec3 {
  const l = length(a);
  if (l === 0) return set(out, 0, 0, 0);
  const inv = 1 / l;
  return set(out, a.x * inv, a.y * inv, a.z * inv);
}
export function lerp(out: Vec3, a: Vec3, b: Vec3, t: number): Vec3 {
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.z = a.z + (b.z - a.z) * t;
  return out;
}
export function min(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  out.x = Math.min(a.x, b.x);
  out.y = Math.min(a.y, b.y);
  out.z = Math.min(a.z, b.z);
  return out;
}
export function max(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  out.x = Math.max(a.x, b.x);
  out.y = Math.max(a.y, b.y);
  out.z = Math.max(a.z, b.z);
  return out;
}
export function abs(out: Vec3, a: Vec3): Vec3 {
  out.x = Math.abs(a.x);
  out.y = Math.abs(a.y);
  out.z = Math.abs(a.z);
  return out;
}
export function equals(a: Vec3, b: Vec3): boolean {
  return a.x === b.x && a.y === b.y && a.z === b.z;
}
export function isFinite3(a: Vec3): boolean {
  return Number.isFinite(a.x) && Number.isFinite(a.y) && Number.isFinite(a.z);
}
/** Writes a unit vector perpendicular to the unit vector `n` (Duff et al. branchless basis). */
export function perpendicular(out: Vec3, n: Vec3): Vec3 {
  if (Math.abs(n.x) >= 0.57735) return normalize(out, set(out, n.y, -n.x, 0));
  return normalize(out, set(out, 0, n.z, -n.y));
}
