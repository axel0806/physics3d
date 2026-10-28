import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import type { ConvexShape } from './shape';
import { combine, diagonalInertia, type MassProperties } from './mass';

/** Segment from (0,−h,0) to (0,h,0) swept by a sphere of `radius`. */
export class Capsule implements ConvexShape {
  readonly kind = 'capsule' as const;
  constructor(
    readonly radius: number,
    readonly halfHeight: number,
  ) {
    if (!(radius > 0) || !(halfHeight >= 0)) throw new RangeError('invalid capsule dimensions');
  }
  support(dir: Vec3, out: Vec3): Vec3 {
    const r = this.radius;
    const endY = dir.y >= 0 ? this.halfHeight : -this.halfHeight;
    const l = V.length(dir);
    if (l === 0) return V.set(out, r, endY, 0);
    return V.set(out, (dir.x / l) * r, endY + (dir.y / l) * r, (dir.z / l) * r);
  }
  massProperties(density: number): MassProperties {
    const r = this.radius;
    const h = this.halfHeight;
    const L = 2 * h;
    // Cylinder along Y, centered at the origin.
    const mc = Math.PI * r * r * L * density;
    const cylinder: MassProperties = {
      mass: mc,
      centerOfMass: new Vec3(),
      inertia: diagonalInertia(
        (mc * (3 * r * r + L * L)) / 12,
        (mc * r * r) / 2,
        (mc * (3 * r * r + L * L)) / 12,
      ),
    };
    // Two hemispheres; each has the sphere's 2/5 m r² about its flat face's center (on the axis),
    // and its COM sits 3r/8 further out along the axis.
    const mh = (2 / 3) * Math.PI * r * r * r * density;
    const a = (3 * r) / 8;
    const transverse = (2 / 5 - (a / r) * (a / r)) * mh * r * r;
    const axial = (2 / 5) * mh * r * r;
    const hemi = (sign: 1 | -1): MassProperties => ({
      mass: mh,
      centerOfMass: new Vec3(0, sign * (h + a), 0),
      inertia: diagonalInertia(transverse, axial, transverse),
    });
    return combine([cylinder, hemi(1), hemi(-1)]);
  }
  localBounds(out: AABB): AABB {
    const r = this.radius;
    const e = this.halfHeight + r;
    return BB.set(out, -r, -e, -r, r, e, r);
  }
}
