import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import type { ContactPoint } from './contact';

/**
 * Reduces a contact set to at most four points, favoring large contact area:
 *  1. the deepest point,
 *  2. the point farthest from it,
 *  3. the point that makes the largest triangle with those two,
 *  4. the point farthest from the diagonal (p1, p3) on the side opposite p2, so the four points
 *     form a quadrilateral that spans the largest area available.
 * Ties resolve to the lower input index, so the result is deterministic.
 */
export function reduceToFour(points: readonly ContactPoint[]): ContactPoint[] {
  if (points.length <= 4) return [...points];
  const pos = (i: number): Vec3 => (points[i] as ContactPoint).position;
  let i1 = 0;
  for (let i = 1; i < points.length; i++) {
    if ((points[i] as ContactPoint).depth > (points[i1] as ContactPoint).depth) i1 = i;
  }
  const p1 = pos(i1);
  let i2 = i1 === 0 ? 1 : 0;
  for (let i = 0; i < points.length; i++) {
    if (i !== i1 && V.distanceSq(pos(i), p1) > V.distanceSq(pos(i2), p1)) i2 = i;
  }
  const p2 = pos(i2);
  const e = V.sub(new Vec3(), p2, p1);
  let i3 = -1;
  let bestArea = -1;
  for (let i = 0; i < points.length; i++) {
    if (i === i1 || i === i2) continue;
    const area = V.lengthSq(V.cross(new Vec3(), e, V.sub(new Vec3(), pos(i), p1)));
    if (area > bestArea) {
      bestArea = area;
      i3 = i;
    }
  }
  const p3 = pos(i3);
  // Diagonal p1→p3 and the normal of the contact plane (through p1, p2, p3).
  const diag = V.sub(new Vec3(), p3, p1);
  const planeN = V.cross(new Vec3(), e, diag);
  const sideOf2 =
    Math.sign(V.dot(V.cross(new Vec3(), diag, V.sub(new Vec3(), p2, p1)), planeN)) || 1;
  let i4 = -1;
  let bestScore = -Infinity;
  for (let i = 0; i < points.length; i++) {
    if (i === i1 || i === i2 || i === i3) continue;
    // Signed distance from the diagonal, measured on the side opposite p2 (so the quad is convex).
    const side = V.dot(V.cross(new Vec3(), diag, V.sub(new Vec3(), pos(i), p1)), planeN);
    const score = -sideOf2 * side;
    if (score > bestScore) {
      bestScore = score;
      i4 = i;
    }
  }
  const chosen = i4 >= 0 ? [i1, i2, i3, i4] : [i1, i2, i3];
  return chosen.map((i) => points[i] as ContactPoint);
}
