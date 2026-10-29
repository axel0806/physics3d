import type { Vec3 } from '../src/math';

/** Indexed triangle mesh with per-vertex normals, ready to upload. */
export interface MeshData {
  readonly positions: Float32Array;
  readonly normals: Float32Array;
  readonly indices: Uint32Array;
}

/** Unit cube [-1, 1]³ with flat normals; scale by the half extents. */
export function boxMesh(): MeshData {
  const positions: number[] = [];
  const normals: number[] = [];
  const indices: number[] = [];
  const faces: [number[], number[], number[], number[]][] = [
    [
      [1, 0, 0],
      [1, -1, -1],
      [1, 1, -1],
      [1, 1, 1],
    ],
    [
      [-1, 0, 0],
      [-1, -1, 1],
      [-1, 1, 1],
      [-1, 1, -1],
    ],
    [
      [0, 1, 0],
      [-1, 1, -1],
      [-1, 1, 1],
      [1, 1, 1],
    ],
    [
      [0, -1, 0],
      [-1, -1, 1],
      [-1, -1, -1],
      [1, -1, -1],
    ],
    [
      [0, 0, 1],
      [-1, -1, 1],
      [1, -1, 1],
      [1, 1, 1],
    ],
    [
      [0, 0, -1],
      [1, -1, -1],
      [-1, -1, -1],
      [-1, 1, -1],
    ],
  ];
  for (const [n, a, b, c] of faces) {
    const d = [a[0]! + c[0]! - b[0]!, a[1]! + c[1]! - b[1]!, a[2]! + c[2]! - b[2]!];
    const base = positions.length / 3;
    for (const p of [a, b, c, d]) {
      positions.push(p[0]!, p[1]!, p[2]!);
      normals.push(n[0]!, n[1]!, n[2]!);
    }
    indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
  }
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    indices: Uint32Array.from(indices),
  };
}

/** Unit sphere with smooth normals. */
export function sphereMesh(rings = 16, segments = 32): MeshData {
  return capsuleMesh(1, 0, rings, segments);
}

/**
 * Capsule along y: radius r, cylinder half-height h (h = 0 gives a sphere). Normals are smooth.
 * `rings` counts latitude bands per hemisphere.
 */
export function capsuleMesh(r: number, h: number, rings = 8, segments = 24): MeshData {
  const positions: number[] = [];
  const normals: number[] = [];
  const indices: number[] = [];
  const rows: { y: number; ny: number; ring: number }[] = [];
  for (let i = 0; i <= rings; i++) {
    const phi = (i / rings) * (Math.PI / 2); // 0 at the pole
    rows.push({ y: h + r * Math.cos(phi), ny: Math.cos(phi), ring: r * Math.sin(phi) });
  }
  for (let i = rings; i >= 0; i--) {
    const phi = (i / rings) * (Math.PI / 2);
    rows.push({ y: -h - r * Math.cos(phi), ny: -Math.cos(phi), ring: r * Math.sin(phi) });
  }
  for (const row of rows) {
    const nr = r > 0 ? row.ring / r : 0;
    for (let s = 0; s <= segments; s++) {
      const a = (s / segments) * Math.PI * 2;
      const cx = Math.cos(a);
      const cz = Math.sin(a);
      positions.push(cx * row.ring, row.y, cz * row.ring);
      normals.push(cx * nr, row.ny, cz * nr);
    }
  }
  const stride = segments + 1;
  for (let i = 0; i < rows.length - 1; i++) {
    for (let s = 0; s < segments; s++) {
      const a = i * stride + s;
      const b = a + stride;
      indices.push(a, a + 1, b, a + 1, b + 1, b);
    }
  }
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    indices: Uint32Array.from(indices),
  };
}

/** Flat-shaded mesh from outward-wound triangles (hulls, terrain). */
export function flatMesh(vertices: readonly Vec3[], tri: ArrayLike<number>): MeshData {
  const n = tri.length;
  const positions = new Float32Array(n * 3);
  const normals = new Float32Array(n * 3);
  const indices = new Uint32Array(n);
  for (let t = 0; t < n; t += 3) {
    const a = vertices[tri[t] as number] as Vec3;
    const b = vertices[tri[t + 1] as number] as Vec3;
    const c = vertices[tri[t + 2] as number] as Vec3;
    const ux = b.x - a.x;
    const uy = b.y - a.y;
    const uz = b.z - a.z;
    const vx = c.x - a.x;
    const vy = c.y - a.y;
    const vz = c.z - a.z;
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l;
    ny /= l;
    nz /= l;
    [a, b, c].forEach((p, k) => {
      const o = (t + k) * 3;
      positions[o] = p.x;
      positions[o + 1] = p.y;
      positions[o + 2] = p.z;
      normals[o] = nx;
      normals[o + 1] = ny;
      normals[o + 2] = nz;
      indices[t + k] = t + k;
    });
  }
  return { positions, normals, indices };
}
