import { AABB } from '../../math';
import * as BB from '../../math/aabb';

/** Result of a closest-hit raycast: proxy id and ray parameter t (world distance along the direction). */
export interface TreeRayHit {
  readonly id: number;
  readonly t: number;
}

const NULL = -1;
const STRIDE = 6;

/**
 * Dynamic AABB tree (Ericson, Real-Time Collision Detection ch. 6; Box2D-style insertion and
 * AVL-like rotations). Leaves are proxies identified by their node index, which stays stable
 * until destroyed. Each leaf stores a tight box and a fat box (tight grown by `margin`); the
 * tree only changes when a tight box escapes its fat box.
 */
export class DynamicTree {
  private cap = 0;
  private fat: Float64Array<ArrayBuffer> = new Float64Array(0);
  private tight: Float64Array<ArrayBuffer> = new Float64Array(0);
  private parent: Int32Array<ArrayBuffer> = new Int32Array(0);
  private child1: Int32Array<ArrayBuffer> = new Int32Array(0);
  private child2: Int32Array<ArrayBuffer> = new Int32Array(0);
  private height: Int32Array<ArrayBuffer> = new Int32Array(0);
  private leaf: Uint8Array<ArrayBuffer> = new Uint8Array(0);
  private freeHead = NULL;
  private nodeCount = 0;
  private root = NULL;
  private activeLeaves = 0;

  /** @param margin how far fat boxes extend beyond tight boxes (world units). */
  constructor(private readonly margin = 0.1) {
    if (!(margin >= 0)) throw new RangeError('margin must be non-negative');
    this.grow(64);
  }

  get leafCount(): number {
    return this.activeLeaves;
  }

  get rootIndex(): number {
    return this.root;
  }

  /** Height of the tree in edges (a single leaf has height 0; empty tree is -1). */
  get treeHeight(): number {
    return this.root === NULL ? -1 : (this.height[this.root] as number);
  }

  /** Visits every node (internal and leaf) with its stored box and depth, root first. */
  forEachNode(visit: (box: AABB, depth: number, isLeaf: boolean) => void): void {
    if (this.root === NULL) return;
    const box = new AABB();
    const stack: [number, number][] = [[this.root, 0]];
    while (stack.length > 0) {
      const [node, depth] = stack.pop() as [number, number];
      visit(readBox(this.fat, node, box), depth, this.leaf[node] === 1);
      if (this.leaf[node] !== 1) {
        stack.push(
          [this.child2[node] as number, depth + 1],
          [this.child1[node] as number, depth + 1],
        );
      }
    }
  }

  /** Tight AABB of a proxy (copy into `out`). */
  tightBox(id: number, out: AABB): AABB {
    this.assertLeaf(id);
    return readBox(this.tight, id, out);
  }

  /** Fat AABB of a proxy as stored in the tree (copy into `out`). */
  fatBox(id: number, out: AABB): AABB {
    this.assertLeaf(id);
    return readBox(this.fat, id, out);
  }

  createProxy(box: AABB): number {
    const id = this.allocate();
    this.leaf[id] = 1;
    this.setBox(this.tight, id, box);
    this.setFat(id, box, 0, 0, 0);
    this.height[id] = 0;
    this.insertLeaf(id);
    this.activeLeaves++;
    return id;
  }

  destroyProxy(id: number): void {
    this.assertLeaf(id);
    this.removeLeaf(id);
    this.release(id);
    this.activeLeaves--;
  }

  /**
   * Updates a proxy's tight box. Returns true if the fat box had to change (the tree was
   * modified), false if the new tight box still fits inside the existing fat box.
   * `dx, dy, dz` is the expected displacement; the fat box is extended along it.
   */
  moveProxy(id: number, box: AABB, dx = 0, dy = 0, dz = 0): boolean {
    this.assertLeaf(id);
    this.setBox(this.tight, id, box);
    if (contains(this.fat, id, box)) return false;
    this.removeLeaf(id);
    this.setFat(id, box, dx, dy, dz);
    this.insertLeaf(id);
    return true;
  }

  /** Ids of all proxies whose fat box overlaps `box`, in ascending id order. */
  query(box: AABB): number[] {
    const hits: number[] = [];
    if (this.root === NULL) return hits;
    const stack: number[] = [this.root];
    while (stack.length > 0) {
      const n = stack.pop() as number;
      if (!overlapsNode(this.fat, n, box)) continue;
      if (this.leaf[n]) hits.push(n);
      else {
        stack.push(this.child1[n] as number, this.child2[n] as number);
      }
    }
    return hits.sort((a, b) => a - b);
  }

  /** Closest proxy hit by the ray origin + t·dir for t in [0, maxT], tested against fat boxes. */
  raycast(
    ox: number,
    oy: number,
    oz: number,
    dx: number,
    dy: number,
    dz: number,
    maxT: number,
  ): TreeRayHit | null {
    if (this.root === NULL) return null;
    let best: TreeRayHit | null = null;
    let limit = maxT;
    const box = new AABB();
    const stack: number[] = [this.root];
    while (stack.length > 0) {
      const n = stack.pop() as number;
      readBox(this.fat, n, box);
      const t = BB.raycast(box, ox, oy, oz, dx, dy, dz, limit);
      if (t < 0) continue;
      if (this.leaf[n]) {
        best = { id: n, t };
        limit = t;
      } else {
        stack.push(this.child1[n] as number, this.child2[n] as number);
      }
    }
    return best;
  }

  /** Checks structural invariants; throws on violation. For tests and debugging. */
  validate(): void {
    if (this.root === NULL) {
      if (this.activeLeaves !== 0) throw new Error('empty root but leaves remain');
      return;
    }
    if (this.parent[this.root] !== NULL) throw new Error('root has a parent');
    let leaves = 0;
    const visit = (n: number): number => {
      if (this.leaf[n]) {
        leaves++;
        if (this.height[n] !== 0) throw new Error('leaf height must be 0');
        return 0;
      }
      const a = this.child1[n] as number;
      const b = this.child2[n] as number;
      if (this.parent[a] !== n || this.parent[b] !== n) throw new Error('child parent link broken');
      const ha = visit(a);
      const hb = visit(b);
      if (Math.abs(ha - hb) > 1) throw new Error('unbalanced node');
      const h = 1 + Math.max(ha, hb);
      if (this.height[n] !== h) throw new Error('stale height');
      for (let k = 0; k < 3; k++) {
        if (
          (this.fat[n * STRIDE + k] as number) >
          Math.min(this.fat[a * STRIDE + k] as number, this.fat[b * STRIDE + k] as number)
        ) {
          throw new Error('parent box does not contain child');
        }
        if (
          (this.fat[n * STRIDE + 3 + k] as number) <
          Math.max(this.fat[a * STRIDE + 3 + k] as number, this.fat[b * STRIDE + 3 + k] as number)
        ) {
          throw new Error('parent box does not contain child');
        }
      }
      return h;
    };
    visit(this.root);
    if (leaves !== this.activeLeaves) throw new Error('leaf count mismatch');
  }

  // ---- internal -------------------------------------------------------------

  private assertLeaf(id: number): void {
    if (id < 0 || id >= this.cap || !this.leaf[id]) throw new RangeError(`not a live proxy: ${id}`);
  }

  private allocate(): number {
    if (this.freeHead === NULL) {
      if (this.nodeCount === this.cap) this.grow(this.cap * 2);
      const id = this.nodeCount++;
      this.parent[id] = NULL;
      this.child1[id] = NULL;
      this.child2[id] = NULL;
      return id;
    }
    const id = this.freeHead;
    this.freeHead = this.child1[id] as number;
    this.parent[id] = NULL;
    this.child1[id] = NULL;
    this.child2[id] = NULL;
    return id;
  }

  private release(id: number): void {
    this.leaf[id] = 0;
    this.height[id] = -1;
    this.child1[id] = this.freeHead;
    this.freeHead = id;
  }

  private grow(newCap: number): void {
    const copyI = (src: Int32Array<ArrayBuffer>): Int32Array<ArrayBuffer> => {
      const out = new Int32Array(newCap) as Int32Array<ArrayBuffer>;
      out.set(src);
      return out;
    };
    const copyF = (src: Float64Array<ArrayBuffer>): Float64Array<ArrayBuffer> => {
      const out = new Float64Array(newCap * STRIDE) as Float64Array<ArrayBuffer>;
      out.set(src);
      return out;
    };
    const leafNew = new Uint8Array(newCap) as Uint8Array<ArrayBuffer>;
    leafNew.set(this.leaf);
    this.fat = copyF(this.fat);
    this.tight = copyF(this.tight);
    this.parent = copyI(this.parent);
    this.child1 = copyI(this.child1);
    this.child2 = copyI(this.child2);
    this.height = copyI(this.height);
    this.leaf = leafNew;
    this.cap = newCap;
  }

  private setBox(store: Float64Array<ArrayBuffer>, id: number, b: AABB): void {
    const o = id * STRIDE;
    store[o] = b.min.x;
    store[o + 1] = b.min.y;
    store[o + 2] = b.min.z;
    store[o + 3] = b.max.x;
    store[o + 4] = b.max.y;
    store[o + 5] = b.max.z;
  }

  /** Fat box = tight box fattened by margin, extended along the displacement (dx, dy, dz). */
  private setFat(id: number, b: AABB, dx: number, dy: number, dz: number): void {
    const m = this.margin;
    const o = id * STRIDE;
    this.fat[o] = Math.min(b.min.x, b.min.x - dx) - m;
    this.fat[o + 1] = Math.min(b.min.y, b.min.y - dy) - m;
    this.fat[o + 2] = Math.min(b.min.z, b.min.z - dz) - m;
    this.fat[o + 3] = Math.max(b.max.x, b.max.x - dx) + m;
    this.fat[o + 4] = Math.max(b.max.y, b.max.y - dy) + m;
    this.fat[o + 5] = Math.max(b.max.z, b.max.z - dz) + m;
  }

  private insertLeaf(leafId: number): void {
    if (this.root === NULL) {
      this.root = leafId;
      this.parent[leafId] = NULL;
      return;
    }
    const sibling = this.findBestSibling(leafId);
    const oldParent = this.parent[sibling] as number;
    const newParent = this.allocate();
    this.height[newParent] = 1 + (this.height[sibling] as number);
    this.parent[newParent] = oldParent;
    this.child1[newParent] = sibling;
    this.child2[newParent] = leafId;
    this.parent[sibling] = newParent;
    this.parent[leafId] = newParent;
    if (oldParent === NULL) this.root = newParent;
    else this.replaceChild(oldParent, sibling, newParent);
    this.walkUp(newParent);
  }

  /**
   * Branch-and-bound search for the node whose sibling-insertion cost is lowest. The cost of
   * making a new parent above node n is area(union(leaf, n)) plus the enlargement inherited from
   * its ancestors; the bound SA(leaf) + inherited prunes subtrees that cannot improve.
   */
  private findBestSibling(leafId: number): number {
    const leafBox = readBox(this.fat, leafId, new AABB());
    const leafArea = BB.surfaceArea(leafBox);
    const box = new AABB();
    const union = new AABB();
    let best = this.root;
    let bestCost = BB.surfaceArea(BB.union(union, leafBox, readBox(this.fat, this.root, box)));
    const stack: { node: number; inherited: number }[] = [{ node: this.root, inherited: 0 }];
    while (stack.length > 0) {
      const { node, inherited } = stack.pop() as { node: number; inherited: number };
      readBox(this.fat, node, box);
      const combinedArea = BB.surfaceArea(BB.union(union, leafBox, box));
      const directCost = combinedArea + inherited;
      if (directCost < bestCost) {
        bestCost = directCost;
        best = node;
      }
      const childInherited = inherited + combinedArea - BB.surfaceArea(box);
      if (!this.leaf[node] && childInherited + leafArea < bestCost) {
        stack.push({ node: this.child1[node] as number, inherited: childInherited });
        stack.push({ node: this.child2[node] as number, inherited: childInherited });
      }
    }
    return best;
  }

  private removeLeaf(leafId: number): void {
    if (leafId === this.root) {
      this.root = NULL;
      this.parent[leafId] = NULL;
      return;
    }
    const parent = this.parent[leafId] as number;
    const grand = this.parent[parent] as number;
    const sibling =
      this.child1[parent] === leafId
        ? (this.child2[parent] as number)
        : (this.child1[parent] as number);
    if (grand === NULL) {
      this.root = sibling;
      this.parent[sibling] = NULL;
    } else {
      this.replaceChild(grand, parent, sibling);
      this.parent[sibling] = grand;
    }
    this.release(parent);
    this.parent[leafId] = NULL;
    this.walkUp(grand);
  }

  private replaceChild(p: number, oldChild: number, newChild: number): void {
    if (this.child1[p] === oldChild) this.child1[p] = newChild;
    else this.child2[p] = newChild;
    this.parent[newChild] = p;
  }

  /** Refits and rebalances every ancestor of `start`, including `start` itself. */
  private walkUp(start: number): void {
    let i = start;
    while (i !== NULL) {
      i = this.balance(i);
      this.refit(i);
      i = this.parent[i] as number;
    }
  }

  private refit(n: number): void {
    const a = this.child1[n] as number;
    const b = this.child2[n] as number;
    const o = n * STRIDE;
    for (let k = 0; k < 3; k++) {
      this.fat[o + k] = Math.min(
        this.fat[a * STRIDE + k] as number,
        this.fat[b * STRIDE + k] as number,
      );
      this.fat[o + 3 + k] = Math.max(
        this.fat[a * STRIDE + 3 + k] as number,
        this.fat[b * STRIDE + 3 + k] as number,
      );
    }
    this.height[n] = 1 + Math.max(this.height[a] as number, this.height[b] as number);
  }

  /** AVL-style rotation keeping child heights within one of each other. Returns the subtree root. */
  private balance(a: number): number {
    // Decide from the children's heights only: this node's own stored height may be stale
    // while ancestors are being rebalanced.
    if (this.leaf[a]) return a;
    const b = this.child1[a] as number;
    const c = this.child2[a] as number;
    const diff = (this.height[c] as number) - (this.height[b] as number);
    if (diff > 1) return this.rotate(a, c, b);
    if (diff < -1) return this.rotate(a, b, c);
    return a;
  }

  /** Lifts child x above a, with a keeping `other` and the shorter grandchild. */
  private rotate(a: number, x: number, other: number): number {
    const f = this.child1[x] as number;
    const g = this.child2[x] as number;
    const p = this.parent[a] as number;
    // x takes a's place
    this.parent[x] = p;
    if (p === NULL) this.root = x;
    else this.replaceChild(p, a, x);
    // Keep the taller grandchild under x; move the shorter one under a.
    const keep = (this.height[f] as number) > (this.height[g] as number) ? f : g;
    const move = keep === f ? g : f;
    this.child1[x] = a;
    this.child2[x] = keep;
    this.parent[a] = x;
    this.parent[keep] = x;
    this.child1[a] = other;
    this.child2[a] = move;
    this.parent[other] = a;
    this.parent[move] = a;
    this.refit(a);
    this.refit(x);
    // A new parent can sit above a much taller sibling, so one rotation may leave a height gap
    // larger than 1 on either node. Rebalance both; each call only recurses into shallower trees.
    const newA = this.balance(a);
    if (newA !== a) this.refit(x);
    const newX = this.balance(x);
    return newX;
  }
}

function readBox(store: Float64Array<ArrayBuffer>, id: number, out: AABB): AABB {
  const o = id * STRIDE;
  return BB.set(
    out,
    store[o] as number,
    store[o + 1] as number,
    store[o + 2] as number,
    store[o + 3] as number,
    store[o + 4] as number,
    store[o + 5] as number,
  );
}

function contains(store: Float64Array<ArrayBuffer>, id: number, b: AABB): boolean {
  const o = id * STRIDE;
  return (
    (store[o] as number) <= b.min.x &&
    (store[o + 1] as number) <= b.min.y &&
    (store[o + 2] as number) <= b.min.z &&
    (store[o + 3] as number) >= b.max.x &&
    (store[o + 4] as number) >= b.max.y &&
    (store[o + 5] as number) >= b.max.z
  );
}

function overlapsNode(store: Float64Array<ArrayBuffer>, id: number, b: AABB): boolean {
  const o = id * STRIDE;
  return (
    (store[o] as number) <= b.max.x &&
    (store[o + 3] as number) >= b.min.x &&
    (store[o + 1] as number) <= b.max.y &&
    (store[o + 4] as number) >= b.min.y &&
    (store[o + 2] as number) <= b.max.z &&
    (store[o + 5] as number) >= b.min.z
  );
}
