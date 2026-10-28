import { AABB, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as BB from '../math/aabb';

/** Static, non-convex triangle mesh. Has no mass; use with static bodies only. */
export class TriangleMesh {
  readonly kind = 'mesh' as const;
  readonly vertices: readonly Vec3[];
  readonly indices: Uint32Array;
  readonly bounds: AABB;

  constructor(vertices: readonly Vec3[], indices: ArrayLike<number>) {
    if (indices.length % 3 !== 0) throw new RangeError('mesh index count must be a multiple of 3');
    for (let i = 0; i < indices.length; i++) {
      if ((indices[i] as number) >= vertices.length)
        throw new RangeError('mesh index out of range');
    }
    this.vertices = vertices.map((v) => new Vec3(v.x, v.y, v.z));
    this.indices = Uint32Array.from(indices);
    this.bounds = BB.empty(new AABB());
    for (const v of this.vertices) BB.expandPoint(this.bounds, v);
  }

  get triangleCount(): number {
    return this.indices.length / 3;
  }

  /** Writes the three corners of triangle `i` into `out`. */
  triangle(i: number, out: [Vec3, Vec3, Vec3]): [Vec3, Vec3, Vec3] {
    out[0] = this.vertices[this.indices[3 * i] as number] as Vec3;
    out[1] = this.vertices[this.indices[3 * i + 1] as number] as Vec3;
    out[2] = this.vertices[this.indices[3 * i + 2] as number] as Vec3;
    return out;
  }

  triangleNormal(i: number, out: Vec3): Vec3 {
    const [a, b, c] = this.triangle(i, [new Vec3(), new Vec3(), new Vec3()]);
    return V.normalize(out, V.cross(out, V.sub(new Vec3(), b, a), V.sub(new Vec3(), c, a)));
  }
}
