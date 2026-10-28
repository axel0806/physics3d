import { Transform, Vec3 } from '../../math';
import type { Box } from '../../shapes/box';
import * as V from '../../math/vec3';
import * as T from '../../math/transform';
import type { ContactManifold, ContactPoint } from './contact';
import { reduceToFour } from './manifold';
import { closestSegmentParams, makeContactPoint } from './analytic';

/** Oriented box in world space: center, unit axes (columns of the rotation), and half extents. */
export interface OrientedBox {
  readonly center: Vec3;
  readonly axes: readonly [Vec3, Vec3, Vec3];
  readonly h: readonly [number, number, number];
}

export function orientedBox(box: Box, t: Transform): OrientedBox {
  const axes: [Vec3, Vec3, Vec3] = [
    T.applyDirection(new Vec3(), t, new Vec3(1, 0, 0)),
    T.applyDirection(new Vec3(), t, new Vec3(0, 1, 0)),
    T.applyDirection(new Vec3(), t, new Vec3(0, 0, 1)),
  ];
  return {
    center: V.copy(new Vec3(), t.p),
    axes,
    h: [box.halfExtents.x, box.halfExtents.y, box.halfExtents.z],
  };
}

export type SatKind = 'faceA' | 'faceB' | 'edge';

export interface SatResult {
  /** True when some axis separates the boxes; then `depth` is the (negative) gap on that axis. */
  readonly separated: boolean;
  /** Overlap along the chosen axis (positive when overlapping). */
  readonly depth: number;
  /** Unit axis from A toward B. */
  readonly normal: Vec3;
  readonly kind: SatKind;
  readonly index: number;
}

/** Plane-clip tolerance (length units); corners within it of a clip plane count as inside. */
const CLIP_TOLERANCE = 1e-9;

/** Edge axes must beat a face axis by this margin to be chosen (keeps face contacts stable). */
const EDGE_BIAS = 1e-6;
/** Overlaps closer than this are ties; the earlier candidate (faces of A, then B, then edges) wins. */
const TIE_TOLERANCE = 1e-9;

/** Separating-axis test over the 15 candidate axes. For boxes the penetration depth is exact. */
export function satBoxBox(a: OrientedBox, b: OrientedBox): SatResult {
  const d = V.sub(new Vec3(), b.center, a.center);
  let best: SatResult | null = null;
  let bestScore = Infinity;
  const consider = (axis: Vec3, kind: SatKind, index: number): void => {
    const len = V.length(axis);
    if (len < 1e-9) return;
    const n = V.scale(new Vec3(), axis, 1 / len);
    const dn = V.dot(d, n);
    const overlap = projRadius(a, n) + projRadius(b, n) - Math.abs(dn);
    const score = kind === 'edge' ? overlap + EDGE_BIAS : overlap;
    // Strict improvement by a margin: rounding noise must not flip the reference face between frames.
    if (score >= bestScore - TIE_TOLERANCE) return;
    bestScore = score;
    const normal = dn >= 0 ? n : V.negate(new Vec3(), n);
    best = { separated: overlap < 0, depth: overlap, normal, kind, index };
  };
  for (let i = 0; i < 3; i++) consider(a.axes[i] as Vec3, 'faceA', i);
  for (let i = 0; i < 3; i++) consider(b.axes[i] as Vec3, 'faceB', i);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      consider(V.cross(new Vec3(), a.axes[i] as Vec3, b.axes[j] as Vec3), 'edge', 3 * i + j);
    }
  }
  if (best === null) throw new Error('SAT: no candidate axes');
  return best;
}

function projRadius(box: OrientedBox, n: Vec3): number {
  return (
    box.h[0] * Math.abs(V.dot(box.axes[0], n)) +
    box.h[1] * Math.abs(V.dot(box.axes[1], n)) +
    box.h[2] * Math.abs(V.dot(box.axes[2], n))
  );
}

/** Corners of the face of `box` with outward normal sign·axes[k], in cyclic order. */
function faceCorners(box: OrientedBox, k: number, sign: number): Vec3[] {
  const u = (k + 1) % 3;
  const w = (k + 2) % 3;
  const c = V.addScaled(new Vec3(), box.center, box.axes[k] as Vec3, sign * (box.h[k] as number));
  const eu = V.scale(new Vec3(), box.axes[u] as Vec3, box.h[u] as number);
  const ew = V.scale(new Vec3(), box.axes[w] as Vec3, box.h[w] as number);
  return [
    V.add(new Vec3(), V.add(new Vec3(), c, eu), ew),
    V.sub(new Vec3(), V.add(new Vec3(), c, eu), ew),
    V.sub(new Vec3(), V.sub(new Vec3(), c, eu), ew),
    V.add(new Vec3(), V.sub(new Vec3(), c, eu), ew),
  ];
}

interface ClipVertex {
  readonly p: Vec3;
  readonly id: number;
}

/**
 * Box-box manifold. SAT picks the separating axis. Face contacts clip the incident face against
 * the reference face's side planes and keep the points that penetrate it. Edge contacts yield one
 * closest-point contact. Feature ids depend only on the geometry, so they persist across frames.
 */
export function boxBoxManifold(a: OrientedBox, b: OrientedBox): ContactManifold | null {
  const sat = satBoxBox(a, b);
  if (sat.separated) return null;
  const N = sat.normal;
  if (sat.kind === 'edge') return edgeContact(a, b, N, sat.depth);

  const refIsA = sat.kind === 'faceA';
  const ref = refIsA ? a : b;
  const inc = refIsA ? b : a;
  // Outward normal of the reference face points toward the other box.
  const outRef = refIsA ? N : V.negate(new Vec3(), N);
  const k = argmaxAbsDot(ref.axes, outRef);
  const n = V.scale(
    new Vec3(),
    ref.axes[k] as Vec3,
    V.dot(ref.axes[k] as Vec3, outRef) >= 0 ? 1 : -1,
  );
  const faceCenter = V.addScaled(new Vec3(), ref.center, n, ref.h[k] as number);
  const planeOffset = V.dot(n, faceCenter);

  // Incident face: the face of the other box most anti-parallel to the reference normal.
  const kI = argmaxAbsDot(inc.axes, n);
  const sInc = V.dot(inc.axes[kI] as Vec3, n) >= 0 ? -1 : 1;
  let poly: ClipVertex[] = faceCorners(inc, kI, sInc).map((p, i) => ({
    p,
    id: 8 * kI + sInc + i * 0 + 2 * i,
  }));

  // Clip against the four side planes of the reference face.
  for (let j = 0; j < 3; j++) {
    if (j === k) continue;
    for (const sgn of [1, -1] as const) {
      const planeN = V.scale(new Vec3(), ref.axes[j] as Vec3, sgn);
      const planeD = V.dot(planeN, V.addScaled(new Vec3(), ref.center, planeN, ref.h[j] as number));
      poly = clipPolygon(poly, planeN, planeD, 16 * (2 * j + (sgn > 0 ? 0 : 1)) + 100);
    }
  }

  // Keep points that lie behind the reference face (penetrating) and map each onto that face.
  const points: ContactPoint[] = [];
  for (const v of poly) {
    const depth = planeOffset - V.dot(n, v.p);
    if (depth < -1e-9) continue;
    const dd = Math.max(0, depth);
    const q = V.addScaled(new Vec3(), v.p, n, dd);
    const pointA = refIsA ? q : v.p;
    const pointB = refIsA ? v.p : q;
    points.push(makeContactPoint(pointA, pointB, dd, v.id));
  }
  if (points.length === 0) return edgeContact(a, b, N, sat.depth);
  return { normal: V.copy(new Vec3(), N), points: reduceToFour(points) };
}

function argmaxAbsDot(axes: readonly [Vec3, Vec3, Vec3], n: Vec3): number {
  let best = 0;
  let bestAbs = -1;
  for (let i = 0; i < 3; i++) {
    const v = Math.abs(V.dot(axes[i] as Vec3, n));
    if (v > bestAbs) {
      bestAbs = v;
      best = i;
    }
  }
  return best;
}

/**
 * Sutherland–Hodgman clip of a convex polygon to the half-space dot(n, p) ≤ d. Vertices within
 * CLIP_TOLERANCE of the plane count as inside and keep their id; only a genuine crossing creates a
 * new vertex. Without the tolerance, corners lying exactly on a plane (aligned stacks) flip between
 * inside and outside with rounding, which changes the feature ids and defeats warm starting.
 */
function clipPolygon(poly: ClipVertex[], n: Vec3, d: number, idBase: number): ClipVertex[] {
  const out: ClipVertex[] = [];
  for (let i = 0; i < poly.length; i++) {
    const cur = poly[i] as ClipVertex;
    const nxt = poly[(i + 1) % poly.length] as ClipVertex;
    const dc = V.dot(n, cur.p) - d;
    const dn = V.dot(n, nxt.p) - d;
    if (dc <= CLIP_TOLERANCE) out.push(cur);
    if (dc > CLIP_TOLERANCE && dn < -CLIP_TOLERANCE) {
      out.push({ p: V.lerp(new Vec3(), cur.p, nxt.p, dc / (dc - dn)), id: idBase + i });
    } else if (dc < -CLIP_TOLERANCE && dn > CLIP_TOLERANCE) {
      out.push({ p: V.lerp(new Vec3(), cur.p, nxt.p, dc / (dc - dn)), id: idBase + i });
    }
  }
  return out;
}

/** Single contact at the closest points of the two support edges along the SAT edge axis. */
function edgeContact(a: OrientedBox, b: OrientedBox, N: Vec3, depth: number): ContactManifold {
  const ea = supportEdge(a, N);
  const eb = supportEdge(b, V.negate(new Vec3(), N));
  const [s] = closestSegmentParams(ea[0], ea[1], eb[0], eb[1]);
  const dd = Math.max(0, depth);
  // A's surface point is the closest point on its edge; B's is pushed back by −N·depth.
  const pointA = V.lerp(new Vec3(), ea[0], ea[1], s);
  const pointB = V.addScaled(new Vec3(), pointA, N, -dd);
  return { normal: V.copy(new Vec3(), N), points: [makeContactPoint(pointA, pointB, dd, 1000)] };
}

/** Edge of `box` most extreme along n, parallel to the axis least aligned with n. */
function supportEdge(box: OrientedBox, n: Vec3): [Vec3, Vec3] {
  const corner = V.copy(new Vec3(), box.center);
  for (let i = 0; i < 3; i++) {
    const s = V.dot(box.axes[i] as Vec3, n) >= 0 ? 1 : -1;
    V.addScaled(corner, corner, box.axes[i] as Vec3, s * (box.h[i] as number));
  }
  let k = 0;
  for (let i = 1; i < 3; i++) {
    if (Math.abs(V.dot(box.axes[i] as Vec3, n)) < Math.abs(V.dot(box.axes[k] as Vec3, n))) k = i;
  }
  const h = box.h[k] as number;
  const p = V.addScaled(new Vec3(), corner, box.axes[k] as Vec3, -h);
  const q = V.addScaled(new Vec3(), corner, box.axes[k] as Vec3, h);
  return [p, q];
}
