import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import * as T from '../../math/transform';
import * as Q from '../../math/quat';
import { Box, Capsule, ConvexHull, Sphere } from '../../shapes';
import type { ContactManifold, ContactPoint } from './contact';
import type { Placed } from './support';

/** Result of the fast path: a manifold, nothing (the shape is clear of the triangle), or "unsure". */
export type TriangleContact = ContactManifold | 'clear' | 'unsure';

/** Candidate contact points are accepted this far outside the triangle's edges (fraction of size). */
const EDGE_MARGIN = 1e-3;

/**
 * Contact between a convex shape and one triangle of a static mesh, using the triangle's face
 * normal. Candidate points are the convex shape's lowest points relative to the triangle plane
 * (box corners, sphere and capsule deepest points, hull vertices); those that lie over the
 * triangle and below its plane become contact points, so a box lying on flat terrain gets its four
 * corners spread over the triangles beneath it.
 *
 * `normal` points from the triangle toward the shape; point A is on the triangle, point B on the
 * shape. Returns 'clear' when the shape lies wholly above the plane (no contact possible) and
 * 'unsure' when the shape overlaps the plane but none of its points lies over the triangle (an
 * edge or vertex of the triangle poking into it, or a shape beside it), which the caller settles
 * with the general GJK/EPA path.
 */
export function convexTriangleContact(
  shape: Placed,
  a: Vec3,
  b: Vec3,
  c: Vec3,
  skin: number,
): TriangleContact {
  const e1 = V.sub(new Vec3(), b, a);
  const e2 = V.sub(new Vec3(), c, a);
  const n = V.cross(new Vec3(), e1, e2);
  const area2 = V.length(n);
  if (!(area2 > 1e-14)) return 'clear';
  V.scale(n, n, 1 / area2);
  if (V.dot(n, V.sub(new Vec3(), shape.transform.p, a)) < 0) V.negate(n, n);

  const candidates = candidatePoints(shape, n);
  let minSd = Infinity;
  for (const p of candidates) minSd = Math.min(minSd, V.dot(n, V.sub(new Vec3(), p.point, a)));
  if (minSd > skin) return 'clear';

  // Barycentric coordinates of each point's projection, tested with a small margin.
  const d00 = V.dot(e1, e1);
  const d01 = V.dot(e1, e2);
  const d11 = V.dot(e2, e2);
  const denom = d00 * d11 - d01 * d01;
  const margin = EDGE_MARGIN;
  const found: ContactPoint[] = [];
  for (const p of candidates) {
    const rel = V.sub(new Vec3(), p.point, a);
    const sd = V.dot(n, rel);
    if (sd > skin) continue;
    const d20 = V.dot(rel, e1);
    const d21 = V.dot(rel, e2);
    const v = (d11 * d20 - d01 * d21) / denom;
    const w = (d00 * d21 - d01 * d20) / denom;
    const u = 1 - v - w;
    if (u < -margin || v < -margin || w < -margin) continue;
    const onPlane = V.addScaled(new Vec3(), p.point, n, -sd);
    found.push({
      position: V.lerp(new Vec3(), onPlane, p.point, 0.5),
      depth: -sd,
      id: p.id,
      pointA: onPlane,
      pointB: p.point,
    });
  }
  if (found.length === 0) return 'unsure';
  found.sort((x, y) => y.depth - x.depth || x.id - y.id);
  return { normal: n, points: found.slice(0, 4) };
}

interface Candidate {
  readonly point: Vec3;
  readonly id: number;
}

/** The shape's lowest points relative to the plane with normal `n` pointing toward the shape. */
function candidatePoints(shape: Placed, n: Vec3): Candidate[] {
  const s = shape.shape;
  const t = shape.transform;
  if (s instanceof Sphere) {
    return [{ point: V.addScaled(new Vec3(), t.p, n, -s.radius), id: 0 }];
  }
  if (s instanceof Capsule) {
    const up = Q.rotate(new Vec3(), t.q, new Vec3(0, s.halfHeight, 0));
    const lo = V.sub(new Vec3(), t.p, up);
    const hi = V.add(new Vec3(), t.p, up);
    return [
      { point: V.addScaled(new Vec3(), lo, n, -s.radius), id: 0 },
      { point: V.addScaled(new Vec3(), hi, n, -s.radius), id: 1 },
    ];
  }
  if (s instanceof Box) {
    const h = s.halfExtents;
    const out: Candidate[] = [];
    let id = 0;
    for (const sx of [-1, 1]) {
      for (const sy of [-1, 1]) {
        for (const sz of [-1, 1]) {
          out.push({
            point: T.apply(new Vec3(), t, new Vec3(sx * h.x, sy * h.y, sz * h.z)),
            id: id++,
          });
        }
      }
    }
    return out;
  }
  if (s instanceof ConvexHull) {
    return s.vertices.map((v, i) => ({ point: T.apply(new Vec3(), t, v), id: i }));
  }
  // Unknown convex shape: its support point opposite the normal.
  const local = T.applyInverseDirection(new Vec3(), t, V.negate(new Vec3(), n));
  return [{ point: T.apply(new Vec3(), t, s.support(local, new Vec3())), id: 0 }];
}
