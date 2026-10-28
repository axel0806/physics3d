import { AABB, Transform, Vec3 } from '../math';
import * as BB from '../math/aabb';
import { apply } from '../math/transform';
import type { ConvexShape } from './shape';
import { combine, transformMassProperties, type MassProperties } from './mass';

export interface CompoundChild {
  readonly shape: ConvexShape;
  /** Child frame expressed in the compound's frame. */
  readonly local: Transform;
}

/** Rigid set of convex children. Mass and inertia use the parallel-axis theorem. */
export class Compound {
  readonly kind = 'compound' as const;
  readonly children: readonly CompoundChild[];

  constructor(children: readonly CompoundChild[]) {
    if (children.length === 0) throw new RangeError('compound needs at least one child');
    this.children = children.map((c) => ({ shape: c.shape, local: c.local.clone() }));
  }

  massProperties(density: number): MassProperties {
    return combine(
      this.children.map((c) => transformMassProperties(c.shape.massProperties(density), c.local)),
    );
  }

  localBounds(out: AABB): AABB {
    BB.empty(out);
    const local = new AABB();
    const corner = new Vec3();
    for (const c of this.children) {
      c.shape.localBounds(local);
      for (const x of [local.min.x, local.max.x]) {
        for (const y of [local.min.y, local.max.y]) {
          for (const z of [local.min.z, local.max.z]) {
            BB.expandPoint(out, apply(corner, c.local, new Vec3(x, y, z)));
          }
        }
      }
    }
    return out;
  }
}
