import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import { supportMinkowski, type Placed, type SupportPoint } from './support';

export interface GjkResult {
  /** True if the shapes overlap (or touch). */
  readonly overlap: boolean;
  /** Distance between the shapes; 0 when they overlap. */
  readonly distance: number;
  /** Closest points on A and B (world space). For overlapping shapes these are approximate. */
  readonly pointA: Vec3;
  readonly pointB: Vec3;
  /** Simplex on the Minkowski difference at termination (≤ 4 points). */
  readonly simplex: readonly SupportPoint[];
  /** True when overlap is reported with a tetrahedron that strictly contains the origin (EPA-ready). */
  readonly interior: boolean;
  readonly iterations: number;
}

export interface GjkOptions {
  /** Stop when the simplex can no longer approach the origin by more than this (absolute). */
  readonly tolerance?: number;
  readonly maxIterations?: number;
  /** Starting direction (world space). Defaults to the centers' difference. */
  readonly initialDirection?: Vec3;
}

const MAX_ITER = 64;

/**
 * Closest point to the origin of the convex hull of `pts` (≤4 points), found by testing every
 * subset: the closest point lies in the relative interior of some subset, and it is the valid
 * (non-negative barycentric) candidate with the least norm. Degenerate subsets are skipped.
 */
export function closestOnSimplex(pts: readonly Vec3[]): {
  readonly indices: readonly number[];
  readonly lambdas: readonly number[];
  readonly point: Vec3;
} {
  const n = pts.length;
  let best: { indices: number[]; lambdas: number[]; point: Vec3; d2: number } | null = null;
  for (let mask = 1; mask < 1 << n; mask++) {
    const idx: number[] = [];
    for (let i = 0; i < n; i++) if (mask & (1 << i)) idx.push(i);
    const sol = affineClosest(idx.map((i) => pts[i] as Vec3));
    if (!sol) continue;
    if (sol.lambdas.some((l) => l < -1e-12)) continue;
    const d2 = V.lengthSq(sol.point);
    if (!best || d2 < best.d2 - 1e-300) {
      best = {
        indices: idx,
        lambdas: sol.lambdas.map((l) => Math.max(0, l)),
        point: sol.point,
        d2,
      };
    }
  }
  if (!best) throw new Error('closestOnSimplex: no valid subset');
  return { indices: best.indices, lambdas: best.lambdas, point: best.point };
}

/** Closest point to the origin on the affine hull of `p`, with barycentric weights. Null if degenerate. */
function affineClosest(p: Vec3[]): { lambdas: number[]; point: Vec3 } | null {
  const k = p.length;
  if (k === 1) return { lambdas: [1], point: V.copy(new Vec3(), p[0] as Vec3) };
  const p0 = p[0] as Vec3;
  const m = k - 1;
  const d: Vec3[] = [];
  for (let i = 1; i < k; i++) d.push(V.sub(new Vec3(), p[i] as Vec3, p0));
  // Gram system G μ = −Dᵀ p0 for the coefficients μ of minimizing |p0 + Σ μ_i d_i|².
  const G: number[][] = [];
  const rhs: number[] = [];
  let maxDiag = 0;
  for (let i = 0; i < m; i++) {
    G.push([]);
    for (let j = 0; j < m; j++) (G[i] as number[]).push(V.dot(d[i] as Vec3, d[j] as Vec3));
    rhs.push(-V.dot(d[i] as Vec3, p0));
    maxDiag = Math.max(maxDiag, G[i]?.[i] ?? 0);
  }
  if (!(maxDiag > 0)) return null;
  const mu = solveSmall(G, rhs, maxDiag * 1e-12);
  if (!mu) return null;
  const point = V.copy(new Vec3(), p0);
  let sumMu = 0;
  for (let i = 0; i < m; i++) {
    V.addScaled(point, point, d[i] as Vec3, mu[i] as number);
    sumMu += mu[i] as number;
  }
  return { lambdas: [1 - sumMu, ...mu], point };
}

/** Gaussian elimination with partial pivoting for tiny systems; null when (near-)singular. */
function solveSmall(
  A: readonly (readonly number[])[],
  b: readonly number[],
  pivotTol: number,
): number[] | null {
  const n = b.length;
  const M: number[][] = A.map((row, i) => [...row, b[i] ?? 0]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) {
      if (Math.abs(M[r]?.[c] ?? 0) > Math.abs(M[p]?.[c] ?? 0)) p = r;
    }
    const pivotRow = M[p] ?? [];
    if (Math.abs(pivotRow[c] ?? 0) <= pivotTol) return null;
    M[p] = M[c] ?? [];
    M[c] = pivotRow;
    for (let r = c + 1; r < n; r++) {
      const row = M[r] ?? [];
      const f = (row[c] ?? 0) / (pivotRow[c] ?? 1);
      for (let k = c; k <= n; k++) row[k] = (row[k] ?? 0) - f * (pivotRow[k] ?? 0);
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    const row = M[r] ?? [];
    let s = row[n] ?? 0;
    for (let k = r + 1; k < n; k++) s -= (row[k] ?? 0) * (x[k] ?? 0);
    x[r] = s / (row[r] ?? 1);
  }
  return x;
}

/**
 * GJK on the Minkowski difference A − B. Returns the distance between the shapes, or
 * overlap = true with a simplex enclosing the origin (input to EPA).
 */
export function gjk(A: Placed, B: Placed, options: GjkOptions = {}): GjkResult {
  const tol = options.tolerance ?? 1e-12;
  const maxIter = options.maxIterations ?? MAX_ITER;
  let simplex: SupportPoint[] = [
    supportMinkowski(A, B, options.initialDirection ?? initialDirection(A, B)),
  ];
  let v = V.copy(new Vec3(), (simplex[0] as SupportPoint).w);
  for (let iter = 1; iter <= maxIter; iter++) {
    const vv = V.lengthSq(v);
    // The origin lies on the simplex. With four points it is strictly enclosed; with fewer it is
    // on a lower-dimensional face, and EPA completes the tetrahedron. Either way: overlap.
    const originScale = Math.max(1, scaleOf(simplex));
    if (vv <= 1e-24 * originScale * originScale) return finish(simplex, true, 0, iter);
    const w = supportMinkowski(A, B, V.negate(new Vec3(), v));
    // No progress toward the origin: v is the closest point of the Minkowski difference.
    if (vv - V.dot(v, w.w) <= tol * vv) return finish(simplex, false, Math.sqrt(vv), iter);
    if (simplex.some((s) => V.distanceSq(s.w, w.w) === 0))
      return finish(simplex, false, Math.sqrt(vv), iter);
    simplex.push(w);
    const sol = closestOnSimplex(simplex.map((s) => s.w));
    simplex = sol.indices.map((i) => simplex[i] as SupportPoint);
    v = sol.point;
  }
  return finish(simplex, false, V.length(v), maxIter + 1);
}

function scaleOf(s: readonly SupportPoint[]): number {
  let m = 0;
  for (const p of s) m = Math.max(m, V.length(p.w));
  return m;
}

function initialDirection(A: Placed, B: Placed): Vec3 {
  const d = V.sub(new Vec3(), A.transform.p, B.transform.p);
  return V.lengthSq(d) > 0 ? d : new Vec3(1, 0, 0);
}

function finish(
  simplex: SupportPoint[],
  overlap: boolean,
  distance: number,
  iterations: number,
): GjkResult {
  // Witnesses: barycentric combination of the simplex used for the closest point.
  const pts = simplex.map((s) => s.w);
  const sol = closestOnSimplex(pts);
  const pa = new Vec3();
  const pb = new Vec3();
  sol.indices.forEach((idx, k) => {
    const s = simplex[idx] as SupportPoint;
    const l = sol.lambdas[k] as number;
    V.addScaled(pa, pa, s.a, l);
    V.addScaled(pb, pb, s.b, l);
  });
  return {
    overlap,
    distance: overlap ? 0 : distance,
    pointA: pa,
    pointB: pb,
    simplex: [...simplex],
    interior: overlap && simplex.length === 4,
    iterations,
  };
}
