import { Transform, Vec3 } from '../../math';
import * as T from '../../math/transform';
import type { ConvexShape } from '../../shapes/shape';

/** A convex shape placed in world space. */
export interface Placed {
  readonly shape: ConvexShape;
  readonly transform: Transform;
}

/** World-space support point of a placed shape along world direction `dir`. */
export function supportWorld(p: Placed, dir: Vec3, out: Vec3): Vec3 {
  const local = T.applyInverseDirection(new Vec3(), p.transform, dir);
  const s = p.shape.support(local, new Vec3());
  return T.apply(out, p.transform, s);
}

/** A point of the Minkowski difference A − B, with the witnesses on A and B that produced it. */
export interface SupportPoint {
  readonly w: Vec3;
  readonly a: Vec3;
  readonly b: Vec3;
}

/** Support point of A − B along `dir`: argmax of (a·dir) − (b·(−dir)) over the two shapes. */
export function supportMinkowski(A: Placed, B: Placed, dir: Vec3): SupportPoint {
  const a = supportWorld(A, dir, new Vec3());
  const negDir = new Vec3(-dir.x, -dir.y, -dir.z);
  const b = supportWorld(B, negDir, new Vec3());
  return { w: new Vec3(a.x - b.x, a.y - b.y, a.z - b.z), a, b };
}
