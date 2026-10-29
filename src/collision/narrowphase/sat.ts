import { Transform, Vec3 } from '../../math';
import type { Box } from '../../shapes/box';
import * as V from '../../math/vec3';
import * as Q from '../../math/quat';
import type { ContactManifold, ContactPoint } from './contact';
import { closestSegmentParams, makeContactPoint } from './analytic';

/** Oriented box in world space: center, unit axes (columns of the rotation), and half extents. */
export interface OrientedBox {
  readonly center: Vec3;
  readonly axes: readonly [Vec3, Vec3, Vec3];
  readonly h: readonly [number, number, number];
}

const UNIT_X = new Vec3(1, 0, 0);
const UNIT_Y = new Vec3(0, 1, 0);
const UNIT_Z = new Vec3(0, 0, 1);

/** Mutable oriented box that can be refilled without allocating. */
export interface MutableOrientedBox {
  readonly center: Vec3;
  readonly axes: [Vec3, Vec3, Vec3];
  readonly h: [number, number, number];
}

export function createOrientedBox(): MutableOrientedBox {
  return { center: new Vec3(), axes: [new Vec3(), new Vec3(), new Vec3()], h: [0, 0, 0] };
}

/** Fills `out` from a box and its world transform (no allocation). */
export function orientedBoxInto(out: MutableOrientedBox, box: Box, t: Transform): OrientedBox {
  Q.rotate(out.axes[0], t.q, UNIT_X);
  Q.rotate(out.axes[1], t.q, UNIT_Y);
  Q.rotate(out.axes[2], t.q, UNIT_Z);
  V.copy(out.center, t.p);
  out.h[0] = box.halfExtents.x;
  out.h[1] = box.halfExtents.y;
  out.h[2] = box.halfExtents.z;
  return out;
}

export function orientedBox(box: Box, t: Transform): OrientedBox {
  return orientedBoxInto(createOrientedBox(), box, t);
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

// ---- Scalar SAT state (module-level scratch; the narrowphase is synchronous and not re-entrant) ----
let sBestScore = Infinity;
let sBestOverlap = 0;
let sBestNx = 0;
let sBestNy = 0;
let sBestNz = 0;
let sBestKind = 0; // 0 faceA, 1 faceB, 2 edge
let sBestIndex = 0;
let sEarlyOut = false;
let sMargin = 0;
let sSeparated = false;
const KIND_NAMES: readonly SatKind[] = ['faceA', 'faceB', 'edge'];

// Relative rotation |R_ij| = |a_i . b_j| for the current pair, and the center offset in each frame.
const absR = new Float64Array(9);
const tA = new Float64Array(3);
const tB = new Float64Array(3);
const hA = new Float64Array(3);
const hB = new Float64Array(3);

/** Records an axis if it is the best so far (or flags separation in early-out mode). */
function offer(
  score: number,
  overlap: number,
  dn: number,
  nx: number,
  ny: number,
  nz: number,
  kind: number,
  index: number,
): void {
  // A score below -margin guarantees the final best axis separates by more than the margin
  // (its overlap <= score < -margin).
  if (sEarlyOut && score < -sMargin) {
    sSeparated = true;
    return;
  }
  // Strict improvement by a margin: rounding noise must not flip the reference face between frames.
  if (score >= sBestScore - TIE_TOLERANCE) return;
  sBestScore = score;
  sBestOverlap = overlap;
  if (dn >= 0) {
    sBestNx = nx;
    sBestNy = ny;
    sBestNz = nz;
  } else {
    sBestNx = -nx;
    sBestNy = -ny;
    sBestNz = -nz;
  }
  sBestKind = kind;
  sBestIndex = index;
}

/**
 * Runs the 15-axis test into the module state. Returns false when separated (early-out mode).
 * Uses the relative rotation R = A^T B, so each axis costs a handful of multiplications.
 */
function runSat(a: OrientedBox, b: OrientedBox, earlyOut: boolean, margin = 0): boolean {
  const ax0 = a.axes[0];
  const ax1 = a.axes[1];
  const ax2 = a.axes[2];
  const bx0 = b.axes[0];
  const bx1 = b.axes[1];
  const bx2 = b.axes[2];
  const dx = b.center.x - a.center.x;
  const dy = b.center.y - a.center.y;
  const dz = b.center.z - a.center.z;
  sBestScore = Infinity;
  sBestOverlap = 0;
  sEarlyOut = earlyOut;
  sMargin = margin;
  sSeparated = false;
  hA[0] = a.h[0];
  hA[1] = a.h[1];
  hA[2] = a.h[2];
  hB[0] = b.h[0];
  hB[1] = b.h[1];
  hB[2] = b.h[2];
  tA[0] = dx * ax0.x + dy * ax0.y + dz * ax0.z;
  tA[1] = dx * ax1.x + dy * ax1.y + dz * ax1.z;
  tA[2] = dx * ax2.x + dy * ax2.y + dz * ax2.z;
  tB[0] = dx * bx0.x + dy * bx0.y + dz * bx0.z;
  tB[1] = dx * bx1.x + dy * bx1.y + dz * bx1.z;
  tB[2] = dx * bx2.x + dy * bx2.y + dz * bx2.z;
  for (let i = 0; i < 3; i++) {
    const u = a.axes[i] as Vec3;
    absR[3 * i] = Math.abs(u.x * bx0.x + u.y * bx0.y + u.z * bx0.z);
    absR[3 * i + 1] = Math.abs(u.x * bx1.x + u.y * bx1.y + u.z * bx1.z);
    absR[3 * i + 2] = Math.abs(u.x * bx2.x + u.y * bx2.y + u.z * bx2.z);
  }
  // Faces of A.
  for (let i = 0; i < 3; i++) {
    const t = tA[i] as number;
    const overlap =
      (hA[i] as number) +
      (hB[0] as number) * (absR[3 * i] as number) +
      (hB[1] as number) * (absR[3 * i + 1] as number) +
      (hB[2] as number) * (absR[3 * i + 2] as number) -
      Math.abs(t);
    const v = a.axes[i] as Vec3;
    offer(overlap, overlap, t, v.x, v.y, v.z, 0, i);
    if (sSeparated) return false;
  }
  // Faces of B.
  for (let j = 0; j < 3; j++) {
    const t = tB[j] as number;
    const overlap =
      (hB[j] as number) +
      (hA[0] as number) * (absR[j] as number) +
      (hA[1] as number) * (absR[3 + j] as number) +
      (hA[2] as number) * (absR[6 + j] as number) -
      Math.abs(t);
    const v = b.axes[j] as Vec3;
    offer(overlap, overlap, t, v.x, v.y, v.z, 1, j);
    if (sSeparated) return false;
  }
  // Edge axes a_i x b_j. The projection radii follow from R: a_i1 . (a_i x b_j) = +-R(i2, j), etc.
  for (let i = 0; i < 3; i++) {
    const u = a.axes[i] as Vec3;
    const i1 = i === 2 ? 0 : i + 1;
    const i2 = i === 0 ? 2 : i - 1;
    for (let j = 0; j < 3; j++) {
      const v = b.axes[j] as Vec3;
      const ux = u.y * v.z - u.z * v.y;
      const uy = u.z * v.x - u.x * v.z;
      const uz = u.x * v.y - u.y * v.x;
      const len = Math.sqrt(ux * ux + uy * uy + uz * uz);
      if (len < 1e-9) continue;
      const j1 = j === 2 ? 0 : j + 1;
      const j2 = j === 0 ? 2 : j - 1;
      const rad =
        (hA[i1] as number) * (absR[3 * i2 + j] as number) +
        (hA[i2] as number) * (absR[3 * i1 + j] as number) +
        (hB[j1] as number) * (absR[3 * i + j2] as number) +
        (hB[j2] as number) * (absR[3 * i + j1] as number);
      const s = 1 / len;
      const dn = (dx * ux + dy * uy + dz * uz) * s;
      const overlap = rad * s - Math.abs(dn);
      const score = overlap + EDGE_BIAS;
      offer(score, overlap, dn, ux * s, uy * s, uz * s, 2, 3 * i + j);
      if (sSeparated) return false;
    }
  }
  return true;
}

/** Separating-axis test over the 15 candidate axes. For boxes the penetration depth is exact. */
export function satBoxBox(a: OrientedBox, b: OrientedBox): SatResult {
  runSat(a, b, false, 0);
  if (sBestScore === Infinity) throw new Error('SAT: no candidate axes');
  return {
    separated: sBestOverlap < 0,
    depth: sBestOverlap,
    normal: new Vec3(sBestNx, sBestNy, sBestNz),
    kind: KIND_NAMES[sBestKind] as SatKind,
    index: sBestIndex,
  };
}

// ---- Clipping scratch ----
const POLY_CAP = 16;
let polyIn = new Float64Array(POLY_CAP * 4); // x, y, z, id
let polyOut = new Float64Array(POLY_CAP * 4);
let polyN = 0;
// Contact candidates: pointA xyz, pointB xyz, position xyz, depth, id (stride 11)
const CS = 11;
const cand = new Float64Array(POLY_CAP * CS);
const chosen = new Int32Array(4);

/** Sutherland-Hodgman clip of polyIn to dot(n, p) <= d. Corners within tolerance count as inside. */
function clipScalar(nx: number, ny: number, nz: number, d: number, idBase: number): void {
  const inp = polyIn;
  const out = polyOut;
  let m = 0;
  const cnt = polyN;
  for (let i = 0; i < cnt; i++) {
    const ci = i * 4;
    const ni = (i + 1 === cnt ? 0 : i + 1) * 4;
    const cx = inp[ci] as number;
    const cy = inp[ci + 1] as number;
    const cz = inp[ci + 2] as number;
    const px = inp[ni] as number;
    const py = inp[ni + 1] as number;
    const pz = inp[ni + 2] as number;
    const dc = nx * cx + ny * cy + nz * cz - d;
    const dn = nx * px + ny * py + nz * pz - d;
    if (dc <= CLIP_TOLERANCE) {
      const o = m * 4;
      out[o] = cx;
      out[o + 1] = cy;
      out[o + 2] = cz;
      out[o + 3] = inp[ci + 3] as number;
      m++;
    }
    if (
      (dc > CLIP_TOLERANCE && dn < -CLIP_TOLERANCE) ||
      (dc < -CLIP_TOLERANCE && dn > CLIP_TOLERANCE)
    ) {
      const t = dc / (dc - dn);
      const o = m * 4;
      out[o] = cx + (px - cx) * t;
      out[o + 1] = cy + (py - cy) * t;
      out[o + 2] = cz + (pz - cz) * t;
      out[o + 3] = idBase + i;
      m++;
    }
  }
  polyIn = out;
  polyOut = inp;
  polyN = m;
}

/**
 * Box-box manifold. SAT picks the separating axis (with an early exit on the first axis that
 * separates). Face contacts clip the incident face against the reference face's side planes and
 * keep the points that penetrate it. Edge contacts yield one closest-point contact. Feature ids
 * depend only on the geometry, so they persist across frames.
 */
export function boxBoxManifold(a: OrientedBox, b: OrientedBox, margin = 0): ContactManifold | null {
  return boxBox(a, b, margin, false);
}

// Reused result of `boxBoxManifoldPooled`.
interface MutablePoint {
  position: Vec3;
  depth: number;
  id: number;
  pointA: Vec3;
  pointB: Vec3;
}
const POOL_NORMAL = new Vec3();
const POOL_POINTS: MutablePoint[] = [];
for (let i = 0; i < 4; i++) {
  POOL_POINTS.push({
    position: new Vec3(),
    depth: 0,
    id: 0,
    pointA: new Vec3(),
    pointB: new Vec3(),
  });
}
const POOL_LIST: MutablePoint[] = [];
const POOL_MANIFOLD: ContactManifold = { normal: POOL_NORMAL, points: POOL_LIST };

/**
 * Same result as `boxBoxManifold`, but face contacts are written into a shared manifold that is
 * only valid until the next call (no allocation). Edge contacts are returned as fresh objects.
 */
export function boxBoxManifoldPooled(
  a: OrientedBox,
  b: OrientedBox,
  margin = 0,
): ContactManifold | null {
  return boxBox(a, b, margin, true);
}

function boxBox(
  a: OrientedBox,
  b: OrientedBox,
  margin: number,
  pooled: boolean,
): ContactManifold | null {
  if (!runSat(a, b, true, margin)) return null;
  if (sBestScore === Infinity) throw new Error('SAT: no candidate axes');
  if (sBestOverlap < -margin) return null;
  const Nx = sBestNx;
  const Ny = sBestNy;
  const Nz = sBestNz;
  const depthSat = sBestOverlap;
  if (sBestKind === 2) return edgeContact(a, b, new Vec3(Nx, Ny, Nz), depthSat, margin > 0);

  const refIsA = sBestKind === 0;
  const ref = refIsA ? a : b;
  const inc = refIsA ? b : a;
  // Outward normal of the reference face points toward the other box.
  const ox = refIsA ? Nx : -Nx;
  const oy = refIsA ? Ny : -Ny;
  const oz = refIsA ? Nz : -Nz;
  const k = argmaxAbsDot(ref.axes, ox, oy, oz);
  const rk = ref.axes[k] as Vec3;
  const sr = rk.x * ox + rk.y * oy + rk.z * oz >= 0 ? 1 : -1;
  const nx = rk.x * sr;
  const ny = rk.y * sr;
  const nz = rk.z * sr;
  const hk = ref.h[k] as number;
  const fcx = ref.center.x + nx * hk;
  const fcy = ref.center.y + ny * hk;
  const fcz = ref.center.z + nz * hk;
  const planeOffset = nx * fcx + ny * fcy + nz * fcz;

  // Incident face: the face of the other box most anti-parallel to the reference normal.
  const kI = argmaxAbsDot(inc.axes, nx, ny, nz);
  const ik = inc.axes[kI] as Vec3;
  const sInc = ik.x * nx + ik.y * ny + ik.z * nz >= 0 ? -1 : 1;
  {
    const u = (kI + 1) % 3;
    const w = (kI + 2) % 3;
    const iu = inc.axes[u] as Vec3;
    const iw = inc.axes[w] as Vec3;
    const sh = sInc * (inc.h[kI] as number);
    const cx = inc.center.x + ik.x * sh;
    const cy = inc.center.y + ik.y * sh;
    const cz = inc.center.z + ik.z * sh;
    const hu = inc.h[u] as number;
    const hw = inc.h[w] as number;
    const eux = iu.x * hu;
    const euy = iu.y * hu;
    const euz = iu.z * hu;
    const ewx = iw.x * hw;
    const ewy = iw.y * hw;
    const ewz = iw.z * hw;
    const p = polyIn;
    const idb = 8 * kI + sInc;
    p[0] = cx + eux + ewx;
    p[1] = cy + euy + ewy;
    p[2] = cz + euz + ewz;
    p[3] = idb;
    p[4] = cx + eux - ewx;
    p[5] = cy + euy - ewy;
    p[6] = cz + euz - ewz;
    p[7] = idb + 2;
    p[8] = cx - eux - ewx;
    p[9] = cy - euy - ewy;
    p[10] = cz - euz - ewz;
    p[11] = idb + 4;
    p[12] = cx - eux + ewx;
    p[13] = cy - euy + ewy;
    p[14] = cz - euz + ewz;
    p[15] = idb + 6;
    polyN = 4;
  }

  // Clip against the four side planes of the reference face.
  for (let j = 0; j < 3; j++) {
    if (j === k) continue;
    const rj = ref.axes[j] as Vec3;
    const hj = ref.h[j] as number;
    for (let q = 0; q < 2; q++) {
      const sgn = q === 0 ? 1 : -1;
      const pnx = rj.x * sgn;
      const pny = rj.y * sgn;
      const pnz = rj.z * sgn;
      const planeD =
        pnx * (ref.center.x + pnx * hj) +
        pny * (ref.center.y + pny * hj) +
        pnz * (ref.center.z + pnz * hj);
      clipScalar(pnx, pny, pnz, planeD, 16 * (2 * j + q) + 100);
    }
  }

  // Keep points that lie behind the reference face (penetrating) and map each onto that face.
  let nc = 0;
  const poly = polyIn;
  for (let i = 0; i < polyN; i++) {
    const px = poly[i * 4] as number;
    const py = poly[i * 4 + 1] as number;
    const pz = poly[i * 4 + 2] as number;
    const depth = planeOffset - (nx * px + ny * py + nz * pz);
    if (depth < -Math.max(1e-9, margin)) continue;
    // Within the margin the depth stays negative: a speculative point that is still apart.
    const dd = margin > 0 ? depth : Math.max(0, depth);
    const qx = px + nx * dd;
    const qy = py + ny * dd;
    const qz = pz + nz * dd;
    const o = nc * CS;
    if (refIsA) {
      cand[o] = qx;
      cand[o + 1] = qy;
      cand[o + 2] = qz;
      cand[o + 3] = px;
      cand[o + 4] = py;
      cand[o + 5] = pz;
    } else {
      cand[o] = px;
      cand[o + 1] = py;
      cand[o + 2] = pz;
      cand[o + 3] = qx;
      cand[o + 4] = qy;
      cand[o + 5] = qz;
    }
    const ax = cand[o] as number;
    const ay = cand[o + 1] as number;
    const az = cand[o + 2] as number;
    cand[o + 6] = ax + ((cand[o + 3] as number) - ax) * 0.5;
    cand[o + 7] = ay + ((cand[o + 4] as number) - ay) * 0.5;
    cand[o + 8] = az + ((cand[o + 5] as number) - az) * 0.5;
    cand[o + 9] = dd;
    cand[o + 10] = poly[i * 4 + 3] as number;
    nc++;
  }
  if (nc === 0) return edgeContact(a, b, new Vec3(Nx, Ny, Nz), depthSat, margin > 0);

  const reduced = nc > 4;
  const nOut = reduced ? reduceCandidates(nc) : nc;
  if (pooled) {
    V.set(POOL_NORMAL, Nx, Ny, Nz);
    POOL_LIST.length = nOut;
    for (let i = 0; i < nOut; i++) {
      const o = (reduced ? (chosen[i] as number) : i) * CS;
      const pt = POOL_POINTS[i] as MutablePoint;
      V.set(pt.position, cand[o + 6] as number, cand[o + 7] as number, cand[o + 8] as number);
      pt.depth = cand[o + 9] as number;
      pt.id = cand[o + 10] as number;
      V.set(pt.pointA, cand[o] as number, cand[o + 1] as number, cand[o + 2] as number);
      V.set(pt.pointB, cand[o + 3] as number, cand[o + 4] as number, cand[o + 5] as number);
      POOL_LIST[i] = pt;
    }
    return POOL_MANIFOLD;
  }
  const points: ContactPoint[] = new Array<ContactPoint>(nOut);
  for (let i = 0; i < nOut; i++) {
    const o = (reduced ? (chosen[i] as number) : i) * CS;
    points[i] = {
      position: new Vec3(cand[o + 6], cand[o + 7], cand[o + 8]),
      depth: cand[o + 9] as number,
      id: cand[o + 10] as number,
      pointA: new Vec3(cand[o], cand[o + 1], cand[o + 2]),
      pointB: new Vec3(cand[o + 3], cand[o + 4], cand[o + 5]),
    };
  }
  return { normal: new Vec3(Nx, Ny, Nz), points };
}

/**
 * Scalar equivalent of `reduceToFour` (manifold.ts) over `cand` (positions at +6, depth at +9);
 * fills `chosen` and returns the count (4, or 3 when no fourth point exists).
 */
function reduceCandidates(n: number): number {
  const P = (i: number, c: number): number => cand[i * CS + 6 + c] as number;
  let i1 = 0;
  for (let i = 1; i < n; i++) {
    if ((cand[i * CS + 9] as number) > (cand[i1 * CS + 9] as number)) i1 = i;
  }
  const p1x = P(i1, 0);
  const p1y = P(i1, 1);
  const p1z = P(i1, 2);
  const dsq = (i: number): number => {
    const x = P(i, 0) - p1x;
    const y = P(i, 1) - p1y;
    const z = P(i, 2) - p1z;
    return x * x + y * y + z * z;
  };
  let i2 = i1 === 0 ? 1 : 0;
  let best2 = dsq(i2);
  for (let i = 0; i < n; i++) {
    if (i === i1) continue;
    const d = dsq(i);
    if (d > best2) {
      i2 = i;
      best2 = d;
    }
  }
  const ex = P(i2, 0) - p1x;
  const ey = P(i2, 1) - p1y;
  const ez = P(i2, 2) - p1z;
  let i3 = -1;
  let bestArea = -1;
  for (let i = 0; i < n; i++) {
    if (i === i1 || i === i2) continue;
    const vx = P(i, 0) - p1x;
    const vy = P(i, 1) - p1y;
    const vz = P(i, 2) - p1z;
    const cx = ey * vz - ez * vy;
    const cy = ez * vx - ex * vz;
    const cz = ex * vy - ey * vx;
    const area = cx * cx + cy * cy + cz * cz;
    if (area > bestArea) {
      bestArea = area;
      i3 = i;
    }
  }
  const dx = P(i3, 0) - p1x;
  const dy = P(i3, 1) - p1y;
  const dz = P(i3, 2) - p1z;
  const pnx = ey * dz - ez * dy;
  const pny = ez * dx - ex * dz;
  const pnz = ex * dy - ey * dx;
  // sideOf2 = sign(cross(diag, p2 - p1) . planeN)
  const s2x = dy * ez - dz * ey;
  const s2y = dz * ex - dx * ez;
  const s2z = dx * ey - dy * ex;
  const sideOf2 = Math.sign(s2x * pnx + s2y * pny + s2z * pnz) || 1;
  let i4 = -1;
  let bestScore = -Infinity;
  for (let i = 0; i < n; i++) {
    if (i === i1 || i === i2 || i === i3) continue;
    const vx = P(i, 0) - p1x;
    const vy = P(i, 1) - p1y;
    const vz = P(i, 2) - p1z;
    const cx = dy * vz - dz * vy;
    const cy = dz * vx - dx * vz;
    const cz = dx * vy - dy * vx;
    const score = -sideOf2 * (cx * pnx + cy * pny + cz * pnz);
    if (score > bestScore) {
      bestScore = score;
      i4 = i;
    }
  }
  chosen[0] = i1;
  chosen[1] = i2;
  chosen[2] = i3;
  if (i4 >= 0) {
    chosen[3] = i4;
    return 4;
  }
  return 3;
}

function argmaxAbsDot(
  axes: readonly [Vec3, Vec3, Vec3],
  nx: number,
  ny: number,
  nz: number,
): number {
  let best = 0;
  let bestAbs = -1;
  for (let i = 0; i < 3; i++) {
    const a = axes[i] as Vec3;
    const v = Math.abs(a.x * nx + a.y * ny + a.z * nz);
    if (v > bestAbs) {
      bestAbs = v;
      best = i;
    }
  }
  return best;
}

/** Single contact at the closest points of the two support edges along the SAT edge axis. */
function edgeContact(
  a: OrientedBox,
  b: OrientedBox,
  N: Vec3,
  depth: number,
  signed = false,
): ContactManifold {
  const ea = supportEdge(a, N);
  const eb = supportEdge(b, V.negate(new Vec3(), N));
  const [s] = closestSegmentParams(ea[0], ea[1], eb[0], eb[1]);
  const dd = signed ? depth : Math.max(0, depth);
  // A's surface point is the closest point on its edge; B's is pushed back by -N*depth.
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
