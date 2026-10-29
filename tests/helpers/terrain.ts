import { Vec3 } from '../../src/math';
import { TriangleMesh } from '../../src/shapes';

/** Regular grid heightfield over [-half, half]² with n×n cells, two triangles per cell. */
export function heightfield(n: number, half: number, height: (x: number, z: number) => number) {
  const vertices: Vec3[] = [];
  for (let j = 0; j <= n; j++) {
    for (let i = 0; i <= n; i++) {
      const x = -half + (2 * half * i) / n;
      const z = -half + (2 * half * j) / n;
      vertices.push(new Vec3(x, height(x, z), z));
    }
  }
  const indices: number[] = [];
  const at = (i: number, j: number) => j * (n + 1) + i;
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      // Counter-clockwise seen from +y, so face normals point up.
      indices.push(at(i, j), at(i, j + 1), at(i + 1, j));
      indices.push(at(i + 1, j), at(i, j + 1), at(i + 1, j + 1));
    }
  }
  return new TriangleMesh(vertices, indices);
}
