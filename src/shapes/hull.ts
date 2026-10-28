import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';
import type { ConvexShape } from './shape';
import { polyhedronMassProperties, type MassProperties } from './mass';
import { quickhull } from './quickhull';

/** Convex hull of a point cloud, stored as outward-oriented triangles in local space. */
export class ConvexHull implements ConvexShape {
  readonly kind = 'hull' as const;
  readonly vertices: readonly Vec3[];
  readonly indices: Uint32Array;

  constructor(points: readonly Vec3[]) {
    const data = quickhull(points);
    this.vertices = data.vertices;
    this.indices = data.indices;
  }

  get triangleCount(): number {
    return this.indices.length / 3;
  }

  support(dir: Vec3, out: Vec3): Vec3 {
    let best = this.vertices[0] as Vec3;
    let bestDot = -Infinity;
    for (const v of this.vertices) {
      const d = V.dot(v, dir);
      if (d > bestDot) {
        bestDot = d;
        best = v;
      }
    }
    return V.copy(out, best);
  }

  massProperties(density: number): MassProperties {
    return polyhedronMassProperties(this.vertices, this.indices, density);
  }

  localBounds(out: AABB): AABB {
    BB.empty(out);
    for (const v of this.vertices) BB.expandPoint(out, v);
    return out;
  }
}
