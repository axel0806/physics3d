import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import type { ConvexShape } from './shape';
import { diagonalInertia, type MassProperties } from './mass';

export class Sphere implements ConvexShape {
  readonly kind = 'sphere' as const;
  constructor(readonly radius: number) {
    if (!(radius > 0)) throw new RangeError('sphere radius must be positive');
  }
  support(dir: Vec3, out: Vec3): Vec3 {
    const l = V.length(dir);
    if (l === 0) return V.set(out, this.radius, 0, 0);
    return V.scale(out, dir, this.radius / l);
  }
  massProperties(density: number): MassProperties {
    const r = this.radius;
    const mass = (4 / 3) * Math.PI * r * r * r * density;
    const i = (2 / 5) * mass * r * r;
    return { mass, centerOfMass: new Vec3(), inertia: diagonalInertia(i, i, i) };
  }
  localBounds(out: AABB): AABB {
    const r = this.radius;
    return BB.set(out, -r, -r, -r, r, r, r);
  }
}
