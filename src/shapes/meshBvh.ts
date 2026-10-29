import { AABB } from '../math';

const LEAF_SIZE = 4;

/**
 * Static bounding-volume hierarchy over a triangle soup. Built top-down by splitting at the median
 * triangle centroid along the longest axis of the node's centroid bounds; stored in flat arrays.
 */
export class MeshBvh {
  /** Per node: min xyz, max xyz. */
  private readonly bounds: Float64Array;
  /** Per node: indices of the two children, or −1 for a leaf. */
  private readonly left: Int32Array;
  private readonly right: Int32Array;
  /** Per leaf: first slot and count in `order`. */
  private readonly start: Int32Array;
  private readonly count: Int32Array;
  /** Triangle indices, grouped by leaf. */
  private readonly order: Uint32Array;
  private nodeCount = 0;
  private readonly stack = new Int32Array(128);

  constructor(
    vertices: ArrayLike<{ x: number; y: number; z: number }>,
    indices: ArrayLike<number>,
  ) {
    const n = indices.length / 3;
    const maxNodes = Math.max(1, 2 * Math.ceil(n / (LEAF_SIZE / 2)));
    this.bounds = new Float64Array(maxNodes * 6);
    this.left = new Int32Array(maxNodes).fill(-1);
    this.right = new Int32Array(maxNodes).fill(-1);
    this.start = new Int32Array(maxNodes);
    this.count = new Int32Array(maxNodes);
    this.order = new Uint32Array(n);
    const tb = new Float64Array(n * 6);
    const cen = new Float64Array(n * 3);
    for (let t = 0; t < n; t++) {
      const a = vertices[indices[3 * t] as number] as { x: number; y: number; z: number };
      const b = vertices[indices[3 * t + 1] as number] as { x: number; y: number; z: number };
      const c = vertices[indices[3 * t + 2] as number] as { x: number; y: number; z: number };
      const x0 = Math.min(a.x, b.x, c.x);
      const y0 = Math.min(a.y, b.y, c.y);
      const z0 = Math.min(a.z, b.z, c.z);
      const x1 = Math.max(a.x, b.x, c.x);
      const y1 = Math.max(a.y, b.y, c.y);
      const z1 = Math.max(a.z, b.z, c.z);
      tb.set([x0, y0, z0, x1, y1, z1], 6 * t);
      cen.set([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], 3 * t);
      this.order[t] = t;
    }
    if (n > 0) this.build(0, n, tb, cen);
  }

  private build(lo: number, hi: number, tb: Float64Array, cen: Float64Array): number {
    const node = this.nodeCount++;
    const b = this.bounds;
    const o = node * 6;
    b[o] = b[o + 1] = b[o + 2] = Infinity;
    b[o + 3] = b[o + 4] = b[o + 5] = -Infinity;
    const cmin = [Infinity, Infinity, Infinity];
    const cmax = [-Infinity, -Infinity, -Infinity];
    for (let i = lo; i < hi; i++) {
      const t = this.order[i] as number;
      for (let k = 0; k < 3; k++) {
        b[o + k] = Math.min(b[o + k] as number, tb[6 * t + k] as number);
        b[o + 3 + k] = Math.max(b[o + 3 + k] as number, tb[6 * t + 3 + k] as number);
        const c = cen[3 * t + k] as number;
        cmin[k] = Math.min(cmin[k] as number, c);
        cmax[k] = Math.max(cmax[k] as number, c);
      }
    }
    if (hi - lo <= LEAF_SIZE) {
      this.start[node] = lo;
      this.count[node] = hi - lo;
      return node;
    }
    let axis = 0;
    for (let k = 1; k < 3; k++) {
      if (
        (cmax[k] as number) - (cmin[k] as number) >
        (cmax[axis] as number) - (cmin[axis] as number)
      ) {
        axis = k;
      }
    }
    const slice = Array.from(this.order.subarray(lo, hi));
    slice.sort((x, y) => (cen[3 * x + axis] as number) - (cen[3 * y + axis] as number) || x - y);
    this.order.set(slice, lo);
    const mid = (lo + hi) >> 1;
    const l = this.build(lo, mid, tb, cen);
    const r = this.build(mid, hi, tb, cen);
    this.left[node] = l;
    this.right[node] = r;
    return node;
  }

  /** Appends the indices of triangles whose boxes overlap `box` to `out`, in a fixed order. */
  query(box: AABB, out: number[]): number[] {
    if (this.nodeCount === 0) return out;
    const b = this.bounds;
    const stack = this.stack;
    let sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
      const node = stack[--sp] as number;
      const o = node * 6;
      if (
        (b[o] as number) > box.max.x ||
        (b[o + 3] as number) < box.min.x ||
        (b[o + 1] as number) > box.max.y ||
        (b[o + 4] as number) < box.min.y ||
        (b[o + 2] as number) > box.max.z ||
        (b[o + 5] as number) < box.min.z
      ) {
        continue;
      }
      const l = this.left[node] as number;
      if (l < 0) {
        const s = this.start[node] as number;
        const c = this.count[node] as number;
        for (let i = 0; i < c; i++) out.push(this.order[s + i] as number);
      } else {
        stack[sp++] = this.right[node] as number;
        stack[sp++] = l;
      }
    }
    return out;
  }
}
