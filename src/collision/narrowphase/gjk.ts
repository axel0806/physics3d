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
  for (let i = 0; i < n; i++) {
    const p = pts[i] as Vec3;
    CS_PTS[3 * i] = p.x;
    CS_PTS[3 * i + 1] = p.y;
    CS_PTS[3 * i + 2] = p.z;
  }
  closestCore(n);
  const indices: number[] = [];
  const lambdas: number[] = [];
  for (let k = 0; k < CS_N; k++) {
    indices.push(CS_IDX[k] as number);
    lambdas.push(CS_LAM[k] as number);
  }
  return {
    indices,
    lambdas,
    point: new Vec3(CS_POINT[0] as number, CS_POINT[1] as number, CS_POINT[2] as number),
  };
}

/** Scratch for `closestCore`: input points, and the chosen subset with its weights and point. */
const CS_PTS = new Float64Array(12);
const CS_IDX = new Int32Array(4);
const CS_LAM = new Float64Array(4);
const CS_POINT = new Float64Array(3);
let CS_N = 0;
// Candidate under evaluation.
const C_IDX = new Int32Array(4);
const C_LAM = new Float64Array(4);
const C_POINT = new Float64Array(3);
const C_D = new Float64Array(9);
const C_G = new Float64Array(12);

/** Allocation-free `closestOnSimplex` over the first `n` points of CS_PTS; results in CS_*. */
function closestCore(n: number): void {
  let bestD2 = Infinity;
  let found = false;
  for (let mask = 1; mask < 1 << n; mask++) {
    let k = 0;
    for (let i = 0; i < n; i++) if (mask & (1 << i)) C_IDX[k++] = i;
    if (!affineCore(k)) continue;
    let valid = true;
    for (let i = 0; i < k; i++) if ((C_LAM[i] as number) < -1e-12) valid = false;
    if (!valid) continue;
    const px = C_POINT[0] as number;
    const py = C_POINT[1] as number;
    const pz = C_POINT[2] as number;
    const d2 = px * px + py * py + pz * pz;
    if (!found || d2 < bestD2 - 1e-300) {
      found = true;
      bestD2 = d2;
      CS_N = k;
      for (let i = 0; i < k; i++) {
        CS_IDX[i] = C_IDX[i] as number;
        CS_LAM[i] = Math.max(0, C_LAM[i] as number);
      }
      CS_POINT[0] = px;
      CS_POINT[1] = py;
      CS_POINT[2] = pz;
    }
  }
  if (!found) throw new Error('closestOnSimplex: no valid subset');
}

/**
 * Closest point to the origin on the affine hull of the k points C_IDX[0..k) of CS_PTS, with
 * barycentric weights in C_LAM and the point in C_POINT. False if degenerate.
 */
function affineCore(k: number): boolean {
  const i0 = 3 * (C_IDX[0] as number);
  const p0x = CS_PTS[i0] as number;
  const p0y = CS_PTS[i0 + 1] as number;
  const p0z = CS_PTS[i0 + 2] as number;
  if (k === 1) {
    C_LAM[0] = 1;
    C_POINT[0] = p0x;
    C_POINT[1] = p0y;
    C_POINT[2] = p0z;
    return true;
  }
  const m = k - 1;
  for (let i = 0; i < m; i++) {
    const j = 3 * (C_IDX[i + 1] as number);
    C_D[3 * i] = (CS_PTS[j] as number) - p0x;
    C_D[3 * i + 1] = (CS_PTS[j + 1] as number) - p0y;
    C_D[3 * i + 2] = (CS_PTS[j + 2] as number) - p0z;
  }
  // Gram system G μ = −Dᵀ p0 for the coefficients μ of minimizing |p0 + Σ μ_i d_i|², stored as
  // an augmented m×(m+1) matrix.
  const w = m + 1;
  let maxDiag = 0;
  for (let i = 0; i < m; i++) {
    const dix = C_D[3 * i] as number;
    const diy = C_D[3 * i + 1] as number;
    const diz = C_D[3 * i + 2] as number;
    for (let j = 0; j < m; j++) {
      C_G[i * w + j] =
        dix * (C_D[3 * j] as number) +
        diy * (C_D[3 * j + 1] as number) +
        diz * (C_D[3 * j + 2] as number);
    }
    C_G[i * w + m] = -(dix * p0x + diy * p0y + diz * p0z);
    maxDiag = Math.max(maxDiag, C_G[i * w + i] as number);
  }
  if (!(maxDiag > 0)) return false;
  if (!eliminateSmall(m, maxDiag * 1e-12)) return false;
  let px = p0x;
  let py = p0y;
  let pz = p0z;
  let sumMu = 0;
  for (let i = 0; i < m; i++) {
    const mu = C_G[i * w + m] as number;
    px += (C_D[3 * i] as number) * mu;
    py += (C_D[3 * i + 1] as number) * mu;
    pz += (C_D[3 * i + 2] as number) * mu;
    sumMu += mu;
    C_LAM[i + 1] = mu;
  }
  C_LAM[0] = 1 - sumMu;
  C_POINT[0] = px;
  C_POINT[1] = py;
  C_POINT[2] = pz;
  return true;
}

/** Gaussian elimination with partial pivoting on C_G (m×(m+1)); solution in the last column. */
function eliminateSmall(m: number, pivotTol: number): boolean {
  const w = m + 1;
  for (let c = 0; c < m; c++) {
    let p = c;
    for (let r = c + 1; r < m; r++) {
      if (Math.abs(C_G[r * w + c] as number) > Math.abs(C_G[p * w + c] as number)) p = r;
    }
    if (Math.abs(C_G[p * w + c] as number) <= pivotTol) return false;
    if (p !== c) {
      for (let k = 0; k < w; k++) {
        const t = C_G[c * w + k] as number;
        C_G[c * w + k] = C_G[p * w + k] as number;
        C_G[p * w + k] = t;
      }
    }
    const pc = C_G[c * w + c] as number;
    for (let r = c + 1; r < m; r++) {
      const f = (C_G[r * w + c] as number) / pc;
      for (let k = c; k <= m; k++) {
        C_G[r * w + k] = (C_G[r * w + k] as number) - f * (C_G[c * w + k] as number);
      }
    }
  }
  for (let r = m - 1; r >= 0; r--) {
    let s = C_G[r * w + m] as number;
    for (let k = r + 1; k < m; k++) s -= (C_G[r * w + k] as number) * (C_G[k * w + m] as number);
    C_G[r * w + m] = s / (C_G[r * w + r] as number);
  }
  return true;
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
    loadSimplex(simplex);
    closestCore(simplex.length);
    const next: SupportPoint[] = [];
    for (let k = 0; k < CS_N; k++) next.push(simplex[CS_IDX[k] as number] as SupportPoint);
    simplex = next;
    v = new Vec3(CS_POINT[0] as number, CS_POINT[1] as number, CS_POINT[2] as number);
  }
  return finish(simplex, false, V.length(v), maxIter + 1);
}

function loadSimplex(simplex: readonly SupportPoint[]): void {
  for (let i = 0; i < simplex.length; i++) {
    const w = (simplex[i] as SupportPoint).w;
    CS_PTS[3 * i] = w.x;
    CS_PTS[3 * i + 1] = w.y;
    CS_PTS[3 * i + 2] = w.z;
  }
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
  loadSimplex(simplex);
  closestCore(simplex.length);
  const pa = new Vec3();
  const pb = new Vec3();
  for (let k = 0; k < CS_N; k++) {
    const s = simplex[CS_IDX[k] as number] as SupportPoint;
    const l = CS_LAM[k] as number;
    V.addScaled(pa, pa, s.a, l);
    V.addScaled(pb, pb, s.b, l);
  }
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
