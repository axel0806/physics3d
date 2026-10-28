import { Vec3 } from '../math';
import * as V from '../math/vec3';

export interface HullData {
  vertices: Vec3[];
  /** Outward-oriented (counter-clockwise seen from outside) triangles. */
  indices: Uint32Array;
}

interface Face {
  v: [number, number, number];
  n: Vec3;
  adj: [Face | null, Face | null, Face | null];
  outside: number[];
  far: number;
  farDist: number;
  alive: boolean;
  visible: boolean;
}

/**
 * 3D quickhull (Barber, Dobkin & Huhdanpaa). Faces keep outside-point lists; each iteration
 * adds the farthest outside point, deletes the faces it can see, and stitches new faces to
 * the horizon. Throws on degenerate (coplanar, collinear, or coincident) input.
 */
export function quickhull(points: readonly Vec3[]): HullData {
  if (points.length < 4) throw new RangeError('quickhull needs at least 4 points');
  const pts = points;
  let scale = 0;
  for (const p of pts) scale = Math.max(scale, Math.abs(p.x), Math.abs(p.y), Math.abs(p.z));
  const tol = 1e-12 * scale;

  // Initial simplex from extreme points.
  const { i0, i1, i2, i3 } = initialSimplex(pts, tol);
  const p0 = pts[i0] as Vec3;
  const p1 = pts[i1] as Vec3;
  const p2 = pts[i2] as Vec3;
  const p3 = pts[i3] as Vec3;
  const tetra = [
    makeFace(pts, i0, i1, i2, p3),
    makeFace(pts, i0, i1, i3, p2),
    makeFace(pts, i0, i2, i3, p1),
    makeFace(pts, i1, i2, i3, p0),
  ];
  linkFaces(tetra, pts.length);

  const tetraSet = new Set([i0, i1, i2, i3]);
  const live = new Set<Face>(tetra);
  const pending: Face[] = [];
  for (let i = 0; i < pts.length; i++) {
    if (tetraSet.has(i)) continue;
    assignPoint(i, tetra, pts, tol, pending);
  }

  while (pending.length > 0) {
    const f = pending.pop() as Face;
    if (!f.alive || f.outside.length === 0) continue;
    const eye = f.far;
    const eyeP = pts[eye] as Vec3;

    // Visible region: faces that see the eye point, found by BFS over adjacency.
    const visible: Face[] = [f];
    f.visible = true;
    for (let qi = 0; qi < visible.length; qi++) {
      const g = visible[qi] as Face;
      for (const h of g.adj) {
        if (h && !h.visible && h.alive && signedDist(h, pts, eyeP) > tol) {
          h.visible = true;
          visible.push(h);
        }
      }
    }

    // Horizon: edges between a visible face and a hidden one. Each yields a new face (a, b, eye).
    const created: Face[] = [];
    for (const g of visible) {
      for (let k = 0; k < 3; k++) {
        const h = g.adj[k] as Face;
        if (h.visible) continue;
        const a = g.v[k] as number;
        const b = g.v[(k + 1) % 3] as number;
        const nf = makeFaceFromEye(pts, a, b, eye);
        nf.adj[0] = h;
        const j = h.adj.indexOf(g);
        h.adj[j] = nf;
        created.push(nf);
      }
    }
    linkFaces(created, pts.length);

    // Orphaned outside points from deleted faces are reassigned against every live face. Checking
    // only the new faces is not enough: an orphan can lie beyond an old, non-visible face.
    const orphans: number[] = [];
    for (const g of visible) {
      for (const idx of g.outside) if (idx !== eye) orphans.push(idx);
      g.alive = false;
      g.visible = false;
      live.delete(g);
    }
    for (const nf of created) live.add(nf);
    const candidates = [...live];
    for (const idx of orphans) assignPoint(idx, candidates, pts, tol, pending);
  }

  // Collect surviving faces and compact the vertex list.
  const remap = new Map<number, number>();
  const vertices: Vec3[] = [];
  const triangles: number[] = [];
  for (const f of live) {
    for (const vi of f.v) {
      let r = remap.get(vi);
      if (r === undefined) {
        r = vertices.length;
        remap.set(vi, r);
        const p = pts[vi] as Vec3;
        vertices.push(new Vec3(p.x, p.y, p.z));
      }
      triangles.push(r);
    }
  }
  return { vertices, indices: Uint32Array.from(triangles) };
}

function initialSimplex(
  pts: readonly Vec3[],
  tol: number,
): { i0: number; i1: number; i2: number; i3: number } {
  // Extremes along each axis, then the farthest pair among them.
  const ext: number[] = [];
  for (const axis of ['x', 'y', 'z'] as const) {
    let lo = 0;
    let hi = 0;
    for (let i = 1; i < pts.length; i++) {
      if ((pts[i] as Vec3)[axis] < (pts[lo] as Vec3)[axis]) lo = i;
      if ((pts[i] as Vec3)[axis] > (pts[hi] as Vec3)[axis]) hi = i;
    }
    ext.push(lo, hi);
  }
  let i0 = ext[0] as number;
  let i1 = ext[1] as number;
  let best = -1;
  for (const a of ext) {
    for (const b of ext) {
      const d = V.distanceSq(pts[a] as Vec3, pts[b] as Vec3);
      if (d > best) {
        best = d;
        i0 = a;
        i1 = b;
      }
    }
  }
  if (!(Math.sqrt(best) > tol)) throw new RangeError('degenerate hull: coincident points');
  const a = pts[i0] as Vec3;
  const b = pts[i1] as Vec3;
  const ab = V.sub(new Vec3(), b, a);
  let i2 = -1;
  let d2 = -1;
  const ap = new Vec3();
  for (let i = 0; i < pts.length; i++) {
    V.sub(ap, pts[i] as Vec3, a);
    const d = V.lengthSq(V.cross(new Vec3(), ap, ab));
    if (d > d2) {
      d2 = d;
      i2 = i;
    }
  }
  if (!(Math.sqrt(d2) / Math.sqrt(best) > 1e-10))
    throw new RangeError('degenerate hull: collinear points');
  const c = pts[i2] as Vec3;
  // Fourth point maximizes |det(e1, e2, e3)|, i.e. its distance from the base plane (scaled by
  // |e1 × e2|). Height is measured relative to the diameter |e1|, so it is scale-invariant and
  // catches flat simplices no matter how small the coordinates are. The determinant's error is
  // ~ε·|e1||e2||e3|, far below the 1e-9 bound.
  const e1 = V.sub(new Vec3(), pts[i1] as Vec3, a);
  const e2 = V.sub(new Vec3(), c, a);
  const normalArea = V.length(V.cross(new Vec3(), e1, e2));
  let i3 = -1;
  let det3 = -1;
  for (let i = 0; i < pts.length; i++) {
    const e3 = V.sub(new Vec3(), pts[i] as Vec3, a);
    const det = Math.abs(V.dot(e1, V.cross(new Vec3(), e2, e3)));
    if (det > det3) {
      det3 = det;
      i3 = i;
    }
  }
  // Two checks. The height test is scale-invariant. The noise floor matters when |e1×e2| is tiny:
  // there the determinant's rounding (~ε·|e1||e2||e3|) divided by |e1×e2| can look like a real
  // height, so the determinant itself must also clear ~1e-12 of the product of edge lengths.
  const e3Len = V.length(V.sub(new Vec3(), pts[i3] as Vec3, a));
  const scaleProduct = V.length(e1) * V.length(e2) * e3Len;
  const coplanar = !(det3 > 1e-9 * V.length(e1) * normalArea) || !(det3 > 1e-12 * scaleProduct);
  if (coplanar) throw new RangeError('degenerate hull: coplanar points');
  return { i0, i1, i2, i3 };
}

/** Face (a, b, c) with the normal flipped, if needed, to point away from `inside`. */
function makeFace(pts: readonly Vec3[], a: number, b: number, c: number, inside: Vec3): Face {
  const pa = pts[a] as Vec3;
  let vb = b;
  let vc = c;
  let n = V.normalize(
    new Vec3(),
    V.cross(
      new Vec3(),
      V.sub(new Vec3(), pts[b] as Vec3, pa),
      V.sub(new Vec3(), pts[c] as Vec3, pa),
    ),
  );
  if (V.dot(n, V.sub(new Vec3(), inside, pa)) > 0) {
    vb = c;
    vc = b;
    n = V.negate(n, n);
  }
  return newFace(a, vb, vc, n);
}

/** New face (a, b, eye) whose orientation follows the horizon edge a→b. */
function makeFaceFromEye(pts: readonly Vec3[], a: number, b: number, eye: number): Face {
  const pa = pts[a] as Vec3;
  const n = V.normalize(
    new Vec3(),
    V.cross(
      new Vec3(),
      V.sub(new Vec3(), pts[b] as Vec3, pa),
      V.sub(new Vec3(), pts[eye] as Vec3, pa),
    ),
  );
  return newFace(a, b, eye, n);
}

function newFace(a: number, b: number, c: number, n: Vec3): Face {
  return {
    v: [a, b, c],
    n,
    adj: [null, null, null],
    outside: [],
    far: -1,
    farDist: 0,
    alive: true,
    visible: false,
  };
}

function signedDist(f: Face, pts: readonly Vec3[], p: Vec3): number {
  return V.dot(f.n, V.sub(new Vec3(), p, pts[f.v[0]] as Vec3));
}

/** Pairs up directed edges among `faces` to set adjacency. Unmatched (horizon) edges are left alone. */
function linkFaces(faces: readonly Face[], nVerts: number): void {
  const edges = new Map<number, [Face, number]>();
  for (const f of faces) {
    for (let k = 0; k < 3; k++) {
      edges.set((f.v[k] as number) * nVerts + (f.v[(k + 1) % 3] as number), [f, k]);
    }
  }
  for (const f of faces) {
    for (let k = 0; k < 3; k++) {
      const partner = edges.get((f.v[(k + 1) % 3] as number) * nVerts + (f.v[k] as number));
      if (partner) f.adj[k] = partner[0];
    }
  }
}

function assignPoint(
  idx: number,
  faces: readonly Face[],
  pts: readonly Vec3[],
  tol: number,
  pending: Face[],
): void {
  let best: Face | null = null;
  let bestDist = tol;
  const p = pts[idx] as Vec3;
  for (const f of faces) {
    if (!f.alive) continue;
    const d = signedDist(f, pts, p);
    if (d > bestDist) {
      bestDist = d;
      best = f;
    }
  }
  if (!best) return;
  if (best.outside.length === 0) pending.push(best);
  best.outside.push(idx);
  if (bestDist > best.farDist) {
    best.farDist = bestDist;
    best.far = idx;
  }
}
