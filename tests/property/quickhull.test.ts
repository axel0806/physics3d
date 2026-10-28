import { describe, it } from 'vitest';
import fc from 'fast-check';
import { Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import { quickhull, polyhedronMassProperties, type HullData } from '../../src/shapes';

const coord = fc.double({ min: -10, max: 10, noNaN: true });
const cloud = fc.array(
  fc.tuple(coord, coord, coord).map(([x, y, z]) => new Vec3(x, y, z)),
  { minLength: 4, maxLength: 80 },
);

const RUNS = { numRuns: 1000 };
const SCALE = 10;
/**
 * "On or inside" tolerance. quickhull drops points within its own tolerance (1e-8 × max |coord|),
 * so the check allows 2× that to absorb rounding at the boundary.
 */
const TOL = 2e-8 * SCALE;

/** Returns the hull, or null for degenerate clouds (the property is skipped via fc.pre). */
function tryHull(pts: Vec3[]): HullData | null {
  try {
    return quickhull(pts);
  } catch (e) {
    if (e instanceof RangeError) return null;
    throw e;
  }
}

/**
 * Precondition for the geometric checks: the hull's volume must be at least 1e-3 of the cube of
 * its largest axis extent. Error analysis: the normal of a face is accurate to ~ε·D²/area and
 * the membership decisions to ~tol, so thickness ratios of 1e-3 keep every check well inside its
 * band. Thinner hulls are exercised by the Phase 4 adversarial thin-shape tests.
 */
function isWellConditioned(h: HullData): boolean {
  const lo = new Vec3(Infinity, Infinity, Infinity);
  const hi = new Vec3(-Infinity, -Infinity, -Infinity);
  for (const v of h.vertices) {
    V.min(lo, lo, v);
    V.max(hi, hi, v);
  }
  const diameter = Math.max(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
  let volume: number;
  try {
    volume = polyhedronMassProperties(h.vertices, h.indices, 1).mass;
  } catch (e) {
    // Rounding can make a flat hull's signed volume non-positive; that is maximally thin.
    if (e instanceof RangeError) return false;
    throw e;
  }
  return volume > 1e-3 * diameter ** 3;
}

/**
 * Outward unit plane of triangle t, or null for slivers. The normal's rounding error is about
 * ε·scale²/area; with area ≥ 1e-4 at scale 10 that is ≈1e-9, below TOL. Smaller slivers have
 * ill-conditioned normals and are skipped.
 */
function trianglePlane(h: HullData, t: number): { a: Vec3; n: Vec3 } | null {
  const a = h.vertices[h.indices[t] as number] as Vec3;
  const b = h.vertices[h.indices[t + 1] as number] as Vec3;
  const c = h.vertices[h.indices[t + 2] as number] as Vec3;
  const cross = V.cross(new Vec3(), V.sub(new Vec3(), b, a), V.sub(new Vec3(), c, a));
  if (V.length(cross) < 1e-4) return null;
  return { a, n: V.normalize(new Vec3(), cross) };
}

describe('quickhull properties', () => {
  it('every input point is inside or on the hull', () => {
    fc.assert(
      fc.property(cloud, (pts) => {
        const h = tryHull(pts);
        fc.pre(h !== null && isWellConditioned(h));
        for (let t = 0; t < h.indices.length; t += 3) {
          const plane = trianglePlane(h, t);
          if (!plane) continue;
          for (const p of pts) {
            if (V.dot(plane.n, V.sub(new Vec3(), p, plane.a)) > TOL) return false;
          }
        }
        return true;
      }),
      RUNS,
    );
  });

  it('the hull is convex: every face plane has all hull vertices on or behind it', () => {
    fc.assert(
      fc.property(cloud, (pts) => {
        const h = tryHull(pts);
        fc.pre(h !== null && isWellConditioned(h));
        for (let t = 0; t < h.indices.length; t += 3) {
          const plane = trianglePlane(h, t);
          if (!plane) continue;
          for (const v of h.vertices) {
            if (V.dot(plane.n, V.sub(new Vec3(), v, plane.a)) > TOL) return false;
          }
        }
        return true;
      }),
      RUNS,
    );
  });

  it("Euler's formula V − E + F = 2 holds, and every edge is shared by exactly two faces", () => {
    fc.assert(
      fc.property(cloud, (pts) => {
        const h = tryHull(pts);
        fc.pre(h !== null);
        const F = h.indices.length / 3;
        const edges = new Map<string, number>();
        for (let t = 0; t < h.indices.length; t += 3) {
          for (let k = 0; k < 3; k++) {
            const u = h.indices[t + k] as number;
            const v = h.indices[t + ((k + 1) % 3)] as number;
            const key = u < v ? `${u},${v}` : `${v},${u}`;
            edges.set(key, (edges.get(key) ?? 0) + 1);
          }
        }
        for (const count of edges.values()) if (count !== 2) return false;
        return h.vertices.length - edges.size + F === 2;
      }),
      RUNS,
    );
  });

  it('hull volume is positive and no larger than its bounding box', () => {
    fc.assert(
      fc.property(cloud, (pts) => {
        const h = tryHull(pts);
        fc.pre(h !== null && isWellConditioned(h));
        const mp = polyhedronMassProperties(h.vertices, h.indices, 1);
        const lo = new Vec3(Infinity, Infinity, Infinity);
        const hi = new Vec3(-Infinity, -Infinity, -Infinity);
        for (const v of h.vertices) {
          V.min(lo, lo, v);
          V.max(hi, hi, v);
        }
        const box = (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z);
        return mp.mass > 0 && mp.mass <= box * (1 + 1e-9);
      }),
      RUNS,
    );
  });
});
