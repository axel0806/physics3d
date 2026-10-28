import { Vec3, Transform } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as T from '../../src/math/transform';
import { quickhull } from '../../src/shapes/quickhull';

/** World-space corners of a box with half extents h placed by transform t. */
export function boxCorners(h: [number, number, number], t: Transform): Vec3[] {
  const out: Vec3[] = [];
  for (const sx of [-1, 1])
    for (const sy of [-1, 1])
      for (const sz of [-1, 1]) {
        out.push(T.apply(new Vec3(), t, new Vec3(sx * h[0], sy * h[1], sz * h[2])));
      }
  return out;
}

/**
 * Independent reference for two convex polytopes: the Minkowski difference A − B is the convex
 * hull of all pairwise vertex differences. Returns the origin's signed relation to it:
 * `depth` > 0 if the origin is inside (distance to the nearest face), otherwise `distance` to the
 * hull surface.
 */
export function referenceMinkowski(
  cornersA: Vec3[],
  cornersB: Vec3[],
): { inside: boolean; depth: number; distance: number } {
  const pts: Vec3[] = [];
  for (const a of cornersA) for (const b of cornersB) pts.push(V.sub(new Vec3(), a, b));
  const hull = quickhull(pts);
  const tri = hull.indices;
  let minFaceDist = Infinity;
  let inside = true;
  for (let t = 0; t < tri.length; t += 3) {
    const a = hull.vertices[tri[t] as number] as Vec3;
    const b = hull.vertices[tri[t + 1] as number] as Vec3;
    const c = hull.vertices[tri[t + 2] as number] as Vec3;
    const n = V.normalize(
      new Vec3(),
      V.cross(new Vec3(), V.sub(new Vec3(), b, a), V.sub(new Vec3(), c, a)),
    );
    const offset = V.dot(n, a); // origin is inside the half-space n·x ≤ offset
    if (offset < 0) inside = false;
    minFaceDist = Math.min(minFaceDist, Math.abs(offset));
  }
  if (inside) return { inside: true, depth: minFaceDist, distance: 0 };
  let best = Infinity;
  for (let t = 0; t < tri.length; t += 3) {
    const a = hull.vertices[tri[t] as number] as Vec3;
    const b = hull.vertices[tri[t + 1] as number] as Vec3;
    const c = hull.vertices[tri[t + 2] as number] as Vec3;
    best = Math.min(best, pointTriangleDistance(new Vec3(), a, b, c));
  }
  return { inside: false, depth: 0, distance: best };
}

/** Distance from the origin to triangle (a, b, c): closest point via Ericson, §5.1.5, then its norm. */
export function pointTriangleDistance(_out: Vec3, a: Vec3, b: Vec3, c: Vec3): number {
  return V.length(closestPointOnTriangle(new Vec3(), new Vec3(0, 0, 0), a, b, c));
}

/** Ericson, Real-Time Collision Detection §5.1.5: closest point on triangle abc to p. */
export function closestPointOnTriangle(out: Vec3, p: Vec3, a: Vec3, b: Vec3, c: Vec3): Vec3 {
  const ab = V.sub(new Vec3(), b, a);
  const ac = V.sub(new Vec3(), c, a);
  const ap = V.sub(new Vec3(), p, a);
  const d1 = V.dot(ab, ap);
  const d2 = V.dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return V.copy(out, a);
  const bp = V.sub(new Vec3(), p, b);
  const d3 = V.dot(ab, bp);
  const d4 = V.dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return V.copy(out, b);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return V.addScaled(out, a, ab, d1 / (d1 - d3));
  const cp = V.sub(new Vec3(), p, c);
  const d5 = V.dot(ab, cp);
  const d6 = V.dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return V.copy(out, c);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return V.addScaled(out, a, ac, d2 / (d2 - d6));
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / (d4 - d3 + (d5 - d6));
    return V.addScaled(out, b, V.sub(new Vec3(), c, b), w);
  }
  const denom = 1 / (va + vb + vc);
  const v = vb * denom;
  const w = vc * denom;
  const q = V.addScaled(new Vec3(), a, ab, v);
  return V.addScaled(out, q, ac, w);
}
