import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import type { ConvexShape } from './shape';
import { diagonalInertia, type MassProperties } from './mass';

/** Axis-aligned in local space, centered at the origin. `halfExtents` are half the full sizes. */
export class Box implements ConvexShape {
  readonly kind = 'box' as const;
  readonly halfExtents: Vec3;
  constructor(hx: number, hy: number, hz: number) {
    if (!(hx > 0 && hy > 0 && hz > 0)) throw new RangeError('box half extents must be positive');
    this.halfExtents = new Vec3(hx, hy, hz);
  }
  support(dir: Vec3, out: Vec3): Vec3 {
    const h = this.halfExtents;
    return V.set(out, dir.x >= 0 ? h.x : -h.x, dir.y >= 0 ? h.y : -h.y, dir.z >= 0 ? h.z : -h.z);
  }
  massProperties(density: number): MassProperties {
    const { x: hx, y: hy, z: hz } = this.halfExtents;
    const mass = 8 * hx * hy * hz * density;
    const ix = (mass / 3) * (hy * hy + hz * hz);
    const iy = (mass / 3) * (hx * hx + hz * hz);
    const iz = (mass / 3) * (hx * hx + hy * hy);
    return { mass, centerOfMass: new Vec3(), inertia: diagonalInertia(ix, iy, iz) };
  }
  localBounds(out: AABB): AABB {
    const h = this.halfExtents;
    return BB.set(out, -h.x, -h.y, -h.z, h.x, h.y, h.z);
  }
}
