import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import type { ContactManifold, ContactPoint } from './contact';

/** Exact contact between two spheres (A at `ca`, radius `ra`; B at `cb`, radius `rb`). */
export function sphereSphere(ca: Vec3, ra: number, cb: Vec3, rb: number): ContactManifold | null {
  const d = V.sub(new Vec3(), cb, ca);
  const dist = V.length(d);
  const depth = ra + rb - dist;
  if (depth < 0) return null;
  const normal = dist > 0 ? V.scale(new Vec3(), d, 1 / dist) : new Vec3(1, 0, 0);
  const pointA = V.addScaled(new Vec3(), ca, normal, ra);
  const pointB = V.addScaled(new Vec3(), cb, normal, -rb);
  return { normal, points: [point(pointA, pointB, depth, 0)] };
}

/**
 * Exact contact between a sphere (center `c`, radius `r`) and an oriented box given by its
 * world-to-local rotation columns (ax, ay, az), center `cb`, and half extents `h`.
 * `normal` is from the sphere (A) toward the box (B).
 */
export function sphereBox(
  c: Vec3,
  r: number,
  cb: Vec3,
  axes: readonly [Vec3, Vec3, Vec3],
  h: Vec3,
): ContactManifold | null {
  const rel = V.sub(new Vec3(), c, cb);
  const local = [V.dot(rel, axes[0]), V.dot(rel, axes[1]), V.dot(rel, axes[2])] as const;
  const hs = [h.x, h.y, h.z] as const;
  const clamped = local.map((v, i) => Math.max(-(hs[i] as number), Math.min(hs[i] as number, v)));
  const inside = local.every((v, i) => Math.abs(v) <= (hs[i] as number));
  let closestLocal: [number, number, number];
  let outward: Vec3; // from box surface toward the sphere center
  let depth: number;
  if (!inside) {
    const delta = local.map((v, i) => v - (clamped[i] as number)) as [number, number, number];
    const dist = Math.hypot(delta[0], delta[1], delta[2]);
    depth = r - dist;
    if (depth < 0) return null;
    outward = new Vec3(0, 0, 0);
    V.addScaled(outward, outward, axes[0], delta[0]);
    V.addScaled(outward, outward, axes[1], delta[1]);
    V.addScaled(outward, outward, axes[2], delta[2]);
    V.normalize(outward, outward);
    closestLocal = clamped as [number, number, number];
  } else {
    // Center inside: push out through the nearest face.
    let best = 0;
    let bestGap = Infinity;
    for (let i = 0; i < 3; i++) {
      const gap = (hs[i] as number) - Math.abs(local[i] as number);
      if (gap < bestGap) {
        bestGap = gap;
        best = i;
      }
    }
    const sgn = (local[best] as number) >= 0 ? 1 : -1;
    outward = V.scale(new Vec3(), axes[best] as Vec3, sgn);
    depth = r + bestGap;
    closestLocal = [0, 0, 0];
    closestLocal[best] = (hs[best] as number) * sgn;
    for (let i = 0; i < 3; i++) if (i !== best) closestLocal[i] = local[i] as number;
  }
  const boxPoint = new Vec3(0, 0, 0);
  for (let i = 0; i < 3; i++)
    V.addScaled(boxPoint, boxPoint, axes[i] as Vec3, closestLocal[i] as number);
  V.add(boxPoint, boxPoint, cb);
  // A is the sphere, B the box: the normal from A to B is the negated outward direction.
  const normal = V.negate(new Vec3(), outward);
  const pointA = V.addScaled(new Vec3(), c, outward, -r);
  return { normal, points: [point(pointA, boxPoint, depth, 0)] };
}

/** Closest points between segments [p1, q1] and [p2, q2] (Ericson, §5.1.9). Returns [s, t] parameters. */
export function closestSegmentParams(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3): [number, number] {
  const d1 = V.sub(new Vec3(), q1, p1);
  const d2 = V.sub(new Vec3(), q2, p2);
  const r = V.sub(new Vec3(), p1, p2);
  const a = V.dot(d1, d1);
  const e = V.dot(d2, d2);
  const f = V.dot(d2, r);
  let s: number;
  let t: number;
  if (a <= 1e-300 && e <= 1e-300) return [0, 0];
  if (a <= 1e-300) {
    s = 0;
    t = clamp01(f / e);
  } else {
    const c = V.dot(d1, r);
    if (e <= 1e-300) {
      t = 0;
      s = clamp01(-c / a);
    } else {
      const b = V.dot(d1, d2);
      const denom = a * e - b * b;
      s = denom > 1e-300 ? clamp01((b * f - c * e) / denom) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp01(-c / a);
      } else if (t > 1) {
        t = 1;
        s = clamp01((b - c) / a);
      }
    }
  }
  return [s, t];
}

export function clamp01(x: number): number {
  return x < 0 ? 0 : x > 1 ? 1 : x;
}

/** Capsule-capsule: segments are the core lines (p, q) with radii ra and rb. */
export function capsuleCapsule(
  pa: Vec3,
  qa: Vec3,
  ra: number,
  pb: Vec3,
  qb: Vec3,
  rb: number,
): ContactManifold | null {
  const [s, t] = closestSegmentParams(pa, qa, pb, qb);
  const ca = V.lerp(new Vec3(), pa, qa, s);
  const cb = V.lerp(new Vec3(), pb, qb, t);
  const d = V.sub(new Vec3(), cb, ca);
  const dist = V.length(d);
  const depth = ra + rb - dist;
  if (depth < 0) return null;
  const normal =
    dist > 1e-12 ? V.scale(new Vec3(), d, 1 / dist) : perpendicularTo(V.sub(new Vec3(), qa, pa));
  const pointA = V.addScaled(new Vec3(), ca, normal, ra);
  const pointB = V.addScaled(new Vec3(), cb, normal, -rb);
  return { normal, points: [point(pointA, pointB, depth, 0)] };
}

function perpendicularTo(v: Vec3): Vec3 {
  if (V.lengthSq(v) === 0) return new Vec3(1, 0, 0);
  return V.perpendicular(new Vec3(), V.normalize(new Vec3(), v));
}

function point(pointA: Vec3, pointB: Vec3, depth: number, id: number): ContactPoint {
  return {
    position: V.lerp(new Vec3(), pointA, pointB, 0.5),
    depth,
    id,
    pointA,
    pointB,
  };
}

export { point as makeContactPoint };
