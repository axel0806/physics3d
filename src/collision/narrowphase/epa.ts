import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import { supportMinkowski, type Placed, type SupportPoint } from './support';

export interface EpaResult {
  /** Penetration depth (≥ 0). */
  readonly depth: number;
  /** Unit contact normal pointing from A toward B: push A along −normal and B along +normal to separate. */
  readonly normal: Vec3;
  readonly pointA: Vec3;
  readonly pointB: Vec3;
  readonly iterations: number;
  /** False if the iteration cap was reached before the tolerance was met (best estimate returned). */
  readonly converged: boolean;
}

interface Face {
  readonly i: number;
  readonly j: number;
  readonly k: number;
  readonly n: Vec3;
  readonly d: number;
}

const MAX_ITER = 256;

/**
 * Expanding Polytope Algorithm (van den Bergen). Starts from a tetrahedron enclosing the origin
 * on the Minkowski difference and repeatedly splits the face nearest the origin toward the
 * support point along its normal. Returns null if no enclosing tetrahedron can be built (the
 * shapes only touch at a point or along a degenerate set).
 */
export function epa(
  A: Placed,
  B: Placed,
  simplex: readonly SupportPoint[],
  scale: number,
): EpaResult | null {
  const verts = completeTetrahedron(A, B, simplex);
  if (!verts) return null;
  // Termination tolerance (on the reach of the nearest face) is loose; visibility must be tight.
  // A loose visibility test keeps faces that barely see the new point, which folds the polytope
  // after ~130 splits on rounded shapes and corrupts the normal (measured on a sphere-box case).
  const tol = 1e-6 * Math.max(scale, 1);
  const visTol = 1e-12 * Math.max(scale, 1);
  let faces: Face[] = [];
  const pts = verts;
  const quads = [
    [1, 2, 3, 0],
    [0, 2, 3, 1],
    [0, 1, 3, 2],
    [0, 1, 2, 3],
  ];
  for (const [i, j, k, opp] of quads as [number, number, number, number][]) {
    faces.push(makeFace(pts, i, j, k, opp));
  }
  for (let iter = 1; iter <= MAX_ITER; iter++) {
    let nearest = faces[0] as Face;
    for (const f of faces) if (f.d < nearest.d) nearest = f;
    const w = supportMinkowski(A, B, nearest.n);
    const reach = V.dot(nearest.n, w.w) - nearest.d;
    if (reach <= tol) return result(pts, nearest, iter, true);
    // Split: remove faces that see w, and connect the horizon edges to w.
    pts.push(w);
    const wi = pts.length - 1;
    const visible: Face[] = [];
    const kept: Face[] = [];
    for (const f of faces) {
      if (V.dot(f.n, w.w) - f.d > visTol) visible.push(f);
      else kept.push(f);
    }
    const edges = new Map<string, [number, number]>();
    for (const f of visible) {
      for (const [a, b] of [
        [f.i, f.j],
        [f.j, f.k],
        [f.k, f.i],
      ] as const) {
        const rev = `${b},${a}`;
        if (edges.has(rev)) edges.delete(rev);
        else edges.set(`${a},${b}`, [a, b]);
      }
    }
    const created: Face[] = [];
    for (const [a, b] of edges.values()) {
      const f = makeFace(pts, a, b, wi, -1);
      if (V.lengthSq(f.n) > 0) created.push(f);
    }
    faces = kept.concat(created);
    if (faces.length === 0) return result(pts, nearest, iter, false);
  }
  let nearest = faces[0] as Face;
  for (const f of faces) if (f.d < nearest.d) nearest = f;
  return result(pts, nearest, MAX_ITER, false);
}

/**
 * Builds a face through (i, j, k) whose normal points away from the origin (the origin is inside
 * the polytope). `opp` is an opposite vertex used to orient the normal when given; with −1 the
 * orientation comes from the origin side of the face.
 */
function makeFace(
  pts: readonly SupportPoint[],
  i: number,
  j: number,
  k: number,
  opp: number,
): Face {
  const a = (pts[i] as SupportPoint).w;
  const b = (pts[j] as SupportPoint).w;
  const c = (pts[k] as SupportPoint).w;
  let n = V.normalize(
    new Vec3(),
    V.cross(new Vec3(), V.sub(new Vec3(), b, a), V.sub(new Vec3(), c, a)),
  );
  const ii = i;
  let jj = j;
  let kk = k;
  // Orient outward: away from the interior reference (opposite vertex if given, else the origin side).
  if (opp >= 0) {
    if (V.dot(n, V.sub(new Vec3(), (pts[opp] as SupportPoint).w, a)) > 0) {
      n = V.negate(n, n);
      [jj, kk] = [kk, jj];
    }
  } else if (V.dot(n, a) < 0) {
    n = V.negate(n, n);
    [jj, kk] = [kk, jj];
  }
  return { i: ii, j: jj, k: kk, n, d: V.dot(n, (pts[ii] as SupportPoint).w) };
}

function result(
  pts: readonly SupportPoint[],
  f: Face,
  iterations: number,
  converged: boolean,
): EpaResult {
  const a = pts[f.i] as SupportPoint;
  const b = pts[f.j] as SupportPoint;
  const c = pts[f.k] as SupportPoint;
  const [u, v, w] = barycentricOfProjection(a.w, b.w, c.w, V.scale(new Vec3(), f.n, f.d));
  const pointA = new Vec3(
    u * a.a.x + v * b.a.x + w * c.a.x,
    u * a.a.y + v * b.a.y + w * c.a.y,
    u * a.a.z + v * b.a.z + w * c.a.z,
  );
  const pointB = new Vec3(
    u * a.b.x + v * b.b.x + w * c.b.x,
    u * a.b.y + v * b.b.y + w * c.b.y,
    u * a.b.z + v * b.b.z + w * c.b.z,
  );
  return {
    depth: Math.max(0, f.d),
    normal: V.copy(new Vec3(), f.n),
    pointA,
    pointB,
    iterations,
    converged,
  };
}

/** Barycentric coordinates of point p projected into triangle (a, b, c). */
export function barycentricOfProjection(
  a: Vec3,
  b: Vec3,
  c: Vec3,
  p: Vec3,
): [number, number, number] {
  const v0 = V.sub(new Vec3(), b, a);
  const v1 = V.sub(new Vec3(), c, a);
  const v2 = V.sub(new Vec3(), p, a);
  const d00 = V.dot(v0, v0);
  const d01 = V.dot(v0, v1);
  const d11 = V.dot(v1, v1);
  const d20 = V.dot(v2, v0);
  const d21 = V.dot(v2, v1);
  const denom = d00 * d11 - d01 * d01;
  if (denom === 0) return [1, 0, 0];
  const v = (d11 * d20 - d01 * d21) / denom;
  const w = (d00 * d21 - d01 * d20) / denom;
  return [1 - v - w, v, w];
}

/** Extends a simplex of 1–3 points to a tetrahedron with non-zero volume that contains the origin. */
function completeTetrahedron(
  A: Placed,
  B: Placed,
  simplex: readonly SupportPoint[],
): SupportPoint[] | null {
  const pts = [...simplex];
  const probes: Vec3[] = [
    new Vec3(1, 0, 0),
    new Vec3(-1, 0, 0),
    new Vec3(0, 1, 0),
    new Vec3(0, -1, 0),
    new Vec3(0, 0, 1),
    new Vec3(0, 0, -1),
    new Vec3(1, 1, 1),
    new Vec3(-1, -1, -1),
    new Vec3(1, -1, 1),
    new Vec3(-1, 1, -1),
  ];
  for (let guard = 0; pts.length < 4 && guard < 4; guard++) {
    const dirs: Vec3[] = [];
    if (pts.length === 1) dirs.push(...probes);
    else if (pts.length === 2) {
      const e = V.sub(new Vec3(), (pts[1] as SupportPoint).w, (pts[0] as SupportPoint).w);
      for (const p of probes) dirs.push(V.cross(new Vec3(), e, p));
    } else {
      const n = V.cross(
        new Vec3(),
        V.sub(new Vec3(), (pts[1] as SupportPoint).w, (pts[0] as SupportPoint).w),
        V.sub(new Vec3(), (pts[2] as SupportPoint).w, (pts[0] as SupportPoint).w),
      );
      dirs.push(n, V.negate(new Vec3(), n));
    }
    let added = false;
    for (const d of dirs) {
      if (V.lengthSq(d) === 0) continue;
      const s = supportMinkowski(A, B, d);
      if (pts.length === 3 && planeDistance(pts, s.w) < 1e-12 * scaleOf(pts)) continue;
      if (pts.some((p) => V.distanceSq(p.w, s.w) === 0)) continue;
      pts.push(s);
      added = true;
      break;
    }
    if (!added) return null;
  }
  if (pts.length !== 4) return null;
  // Tetrahedron must have volume: reject flat ones.
  const a = (pts[0] as SupportPoint).w;
  const vol = V.dot(
    V.sub(new Vec3(), (pts[1] as SupportPoint).w, a),
    V.cross(
      new Vec3(),
      V.sub(new Vec3(), (pts[2] as SupportPoint).w, a),
      V.sub(new Vec3(), (pts[3] as SupportPoint).w, a),
    ),
  );
  return Math.abs(vol) > 1e-14 * scaleOf(pts) ** 3 ? pts : null;
}

function planeDistance(pts: readonly SupportPoint[], p: Vec3): number {
  const a = (pts[0] as SupportPoint).w;
  const n = V.cross(
    new Vec3(),
    V.sub(new Vec3(), (pts[1] as SupportPoint).w, a),
    V.sub(new Vec3(), (pts[2] as SupportPoint).w, a),
  );
  return Math.abs(V.dot(n, V.sub(new Vec3(), p, a))) / Math.max(V.length(n), 1e-300);
}

function scaleOf(pts: readonly SupportPoint[]): number {
  let m = 0;
  for (const p of pts) m = Math.max(m, V.length(p.w));
  return m;
}
