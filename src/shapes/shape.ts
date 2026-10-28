import type { AABB, Vec3 } from '../math';
import type { MassProperties } from './mass';

/** A convex shape supplies a support mapping (used by GJK/EPA) and analytic or exact mass properties. */
export interface ConvexShape {
  readonly kind: 'sphere' | 'box' | 'capsule' | 'hull';
  /** Writes the local-frame point of the shape furthest along `dir` into `out`. */
  support(dir: Vec3, out: Vec3): Vec3;
  massProperties(density: number): MassProperties;
  /** Writes the local-frame bounding box into `out`. */
  localBounds(out: AABB): AABB;
}
