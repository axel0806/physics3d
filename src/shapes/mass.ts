import { Mat3, Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as M3 from '../math/mat3';
import * as T from '../math/transform';

/** Mass, center of mass, and inertia about the center of mass (in the owning shape's local frame). */
export interface MassProperties {
  mass: number;
  centerOfMass: Vec3;
  inertia: Mat3;
}

export function emptyMassProperties(): MassProperties {
  return { mass: 0, centerOfMass: new Vec3(), inertia: new Mat3(new Array(9).fill(0)) };
}

/** Adds I += m (|d|² 1 − d dᵀ): the parallel-axis shift of a body-frame inertia by offset d. */
export function addParallelAxis(out: Mat3, m: number, d: Vec3): void {
  const e = out.e;
  const dd = V.dot(d, d);
  e[0] = (e[0] as number) + m * (dd - d.x * d.x);
  e[4] = (e[4] as number) + m * (dd - d.y * d.y);
  e[8] = (e[8] as number) + m * (dd - d.z * d.z);
  e[1] = (e[1] as number) - m * d.x * d.y;
  e[3] = e[1];
  e[2] = (e[2] as number) - m * d.x * d.z;
  e[6] = e[2];
  e[5] = (e[5] as number) - m * d.y * d.z;
  e[7] = e[5];
}

/** Combines parts (already expressed in one frame) into one body's mass properties. */
export function combine(parts: readonly MassProperties[]): MassProperties {
  let mass = 0;
  const com = new Vec3();
  for (const p of parts) {
    mass += p.mass;
    V.addScaled(com, com, p.centerOfMass, p.mass);
  }
  if (mass > 0) V.scale(com, com, 1 / mass);
  const inertia = new Mat3(new Array(9).fill(0));
  const d = new Vec3();
  for (const p of parts) {
    M3.add(inertia, inertia, p.inertia);
    V.sub(d, p.centerOfMass, com);
    addParallelAxis(inertia, p.mass, d);
  }
  return { mass, centerOfMass: com, inertia };
}

/** Re-expresses a child's mass properties in its parent frame given the child's local transform. */
export function transformMassProperties(mp: MassProperties, t: Transform): MassProperties {
  const rot = new Mat3();
  M3.fromQuat(rot, t.q);
  const rotated = M3.mul(new Mat3(), M3.mul(new Mat3(), rot, mp.inertia), rot.transpose());
  return {
    mass: mp.mass,
    centerOfMass: T.apply(new Vec3(), t, mp.centerOfMass),
    inertia: rotated,
  };
}

/** Inertia tensor diag(x, y, z) as a matrix. */
export function diagonalInertia(x: number, y: number, z: number): Mat3 {
  return M3.fromDiagonal(new Mat3(), x, y, z);
}

/**
 * Exact mass properties of a closed, outward-oriented triangle mesh with uniform density.
 *
 * Each triangle (a, b, c) forms a signed tetrahedron with the origin. The simplex moment
 * identities ∫x_i x_j dV = V/20 (Σ_k p_ki p_kj + Σ_k p_ki Σ_l p_lj) give exact volume,
 * first and second moments, which are then shifted to the centroid.
 */
export function polyhedronMassProperties(
  vertices: readonly Vec3[],
  indices: ArrayLike<number>,
  density: number,
): MassProperties {
  let vol = 0;
  const first = new Vec3();
  const second = new Float64Array(9); // ∫ x_i x_j dV, row-major
  for (let t = 0; t < indices.length; t += 3) {
    const a = vertices[indices[t] as number] as Vec3;
    const b = vertices[indices[t + 1] as number] as Vec3;
    const c = vertices[indices[t + 2] as number] as Vec3;
    const det =
      a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
    const dV = det / 6;
    const sx = a.x + b.x + c.x;
    const sy = a.y + b.y + c.y;
    const sz = a.z + b.z + c.z;
    vol += dV;
    first.x += dV * sx;
    first.y += dV * sy;
    first.z += dV * sz;
    const s = [sx, sy, sz];
    const p = [
      [a.x, a.y, a.z],
      [b.x, b.y, b.z],
      [c.x, c.y, c.z],
    ];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let sumSq = (s[i] as number) * (s[j] as number);
        for (const pk of p) sumSq += (pk[i] as number) * (pk[j] as number);
        second[3 * i + j] = (second[3 * i + j] as number) + (dV / 20) * sumSq;
      }
    }
  }
  if (!(vol > 0))
    throw new RangeError('polyhedron must be closed and outward-oriented (volume ≤ 0)');
  const mass = vol * density;
  const com = new Vec3(first.x / vol / 4, first.y / vol / 4, first.z / vol / 4);
  // Inertia about the origin: tr(M) 1 − M, with M = ρ ∫ x xᵀ dV.
  const M = Array.from(second, (v) => v * density);
  const tr = (M[0] as number) + (M[4] as number) + (M[8] as number);
  const I = new Mat3(M.map((v) => -v));
  I.e[0] = (I.e[0] as number) + tr;
  I.e[4] = (I.e[4] as number) + tr;
  I.e[8] = (I.e[8] as number) + tr;
  // Shift from origin to centroid: I_c = I_o − m(|c|²1 − c cᵀ).
  addParallelAxis(I, -mass, com);
  return { mass, centerOfMass: com, inertia: I };
}
