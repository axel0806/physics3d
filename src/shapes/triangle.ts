import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import type { ConvexShape } from './shape';
import { type MassProperties } from './mass';

/** Triangle as a degenerate convex shape, used for convex-vs-mesh queries. */
export class Triangle implements ConvexShape {
  readonly kind = 'hull' as const;
  readonly corners: readonly [Vec3, Vec3, Vec3];

  constructor(a: Vec3, b: Vec3, c: Vec3) {
    this.corners = [a.clone(), b.clone(), c.clone()];
  }

  support(dir: Vec3, out: Vec3): Vec3 {
    const [a, b, c] = this.corners;
    let best = a;
    let bestDot = V.dot(a, dir);
    for (const p of [b, c]) {
      const d = V.dot(p, dir);
      if (d > bestDot) {
        bestDot = d;
        best = p;
      }
    }
    return V.copy(out, best);
  }

  /** Triangles are zero-volume; they have no mass. Use only as static geometry. */
  massProperties(): MassProperties {
    throw new RangeError('a triangle has no volume; use it only as static geometry');
  }

  localBounds(out: AABB): AABB {
    BB.empty(out);
    for (const c of this.corners) BB.expandPoint(out, c);
    return out;
  }

  get normal(): Vec3 {
    const [a, b, c] = this.corners;
    return V.normalize(
      new Vec3(),
      V.cross(new Vec3(), V.sub(new Vec3(), b, a), V.sub(new Vec3(), c, a)),
    );
  }
}
