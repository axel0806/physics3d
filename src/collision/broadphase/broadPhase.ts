import { AABB } from '../../math';
import { DynamicTree, type TreeRayHit } from './dynamicTree';

/** Packs an unordered proxy pair into one number key (ids must be below 2^26). */
export function pairKey(a: number, b: number): number {
  const lo = a < b ? a : b;
  const hi = a < b ? b : a;
  return lo * 0x4000000 + hi;
}

/** Splits a `pairKey` into its smaller and larger proxy id. */
export function pairLo(key: number): number {
  return Math.floor(key / 0x4000000);
}
export function pairHi(key: number): number {
  return key - Math.floor(key / 0x4000000) * 0x4000000;
}

export interface PairChange {
  readonly kind: 'begin' | 'end';
  /** Smaller proxy id first. */
  readonly a: number;
  readonly b: number;
}

/**
 * Broadphase: a dynamic tree plus a deduplicated set of overlapping proxy pairs (by fat AABB).
 * Only proxies whose fat box changed are re-queried, so the pair set stays exact with
 * incremental work. `updatePairs` reports begin and end events in deterministic order.
 */
export class BroadPhase {
  readonly tree: DynamicTree;
  private readonly pairs = new Map<number, [number, number]>();
  private readonly partners = new Map<number, Set<number>>();
  private readonly dirty: number[] = [];
  private readonly isDirty = new Set<number>();
  private changes: PairChange[] = [];

  constructor(margin = 0.1) {
    this.tree = new DynamicTree(margin);
  }

  get pairCount(): number {
    return this.pairs.size;
  }

  createProxy(box: AABB): number {
    const id = this.tree.createProxy(box);
    this.partners.set(id, new Set());
    this.markDirty(id);
    return id;
  }

  destroyProxy(id: number): void {
    for (const other of [...(this.partners.get(id) ?? [])]) this.removePair(id, other);
    this.partners.delete(id);
    this.isDirty.delete(id);
    const at = this.dirty.indexOf(id);
    if (at >= 0) this.dirty.splice(at, 1);
    this.tree.destroyProxy(id);
  }

  moveProxy(id: number, box: AABB, dx = 0, dy = 0, dz = 0): void {
    if (this.tree.moveProxy(id, box, dx, dy, dz)) this.markDirty(id);
  }

  /** Re-queries dirty proxies and returns the begin/end changes since the previous call. */
  updatePairs(): PairChange[] {
    const fat = new AABB();
    for (const id of this.dirty.splice(0)) {
      this.isDirty.delete(id);
      const current = this.partners.get(id);
      if (!current) continue; // destroyed after being queued
      // Emit only the real transitions: pairs that stop overlapping and pairs that start.
      this.tree.fatBox(id, fat);
      const overlapping = new Set(this.tree.query(fat).filter((other) => other !== id));
      for (const other of [...current]) if (!overlapping.has(other)) this.removePair(id, other);
      for (const other of overlapping) if (!current.has(other)) this.addPair(id, other);
    }
    const out = this.changes;
    this.changes = [];
    return out;
  }

  /** All current pairs (a < b), sorted for determinism. */
  pairList(): [number, number][] {
    return [...this.pairs.values()]
      .map(([a, b]) => [a, b] as [number, number])
      .sort((x, y) => x[0] - y[0] || x[1] - y[1]);
  }

  private sortedKeys = new Float64Array(1024);
  private sortedCount = 0;
  private sortedValid = false;

  /**
   * Keys (see `pairKey`) of all current pairs in ascending order, which equals (a, b) order.
   * The returned array is reused: read only the first `pairKeyCount()` entries, before the next change.
   */
  sortedPairKeys(): Float64Array {
    if (!this.sortedValid && this.sortedReady && this.opLog.length <= 48) {
      this.replayOps();
    } else if (!this.sortedValid) {
      const n = this.pairs.size;
      if (n > this.sortedKeys.length) this.sortedKeys = new Float64Array(2 * n);
      const keys = this.sortedKeys;
      let i = 0;
      for (const k of this.pairs.keys()) keys[i++] = k;
      keys.subarray(0, n).sort();
      this.sortedCount = n;
      this.sortedValid = true;
      this.sortedReady = true;
      this.opLog.length = 0;
    }
    return this.sortedKeys;
  }

  /** Pair additions (positive) and removals (negative key) since the sorted array was valid. */
  private readonly opLog: number[] = [];
  private sortedReady = false;

  /** Applies the logged pair changes to the sorted array by binary search and shifting. */
  private replayOps(): void {
    const log = this.opLog;
    if (this.sortedCount + log.length > this.sortedKeys.length) {
      const grown = new Float64Array(2 * (this.sortedCount + log.length));
      grown.set(this.sortedKeys.subarray(0, this.sortedCount));
      this.sortedKeys = grown;
    }
    const keys = this.sortedKeys;
    let n = this.sortedCount;
    for (let i = 0; i < log.length; i++) {
      const op = log[i] as number;
      const key = op < 0 ? -op - 1 : op;
      let lo = 0;
      let hi = n;
      while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if ((keys[mid] as number) < key) lo = mid + 1;
        else hi = mid;
      }
      const present = lo < n && keys[lo] === key;
      if (op < 0) {
        if (present) {
          keys.copyWithin(lo, lo + 1, n);
          n--;
        }
      } else if (!present) {
        keys.copyWithin(lo + 1, lo, n);
        keys[lo] = key;
        n++;
      }
    }
    this.sortedCount = n;
    this.sortedValid = true;
    log.length = 0;
  }

  pairKeyCount(): number {
    return this.sortedCount;
  }

  hasPair(a: number, b: number): boolean {
    return this.pairs.has(pairKey(a, b));
  }

  /** Visits every node of the underlying tree (for debug drawing). */
  forEachNode(visit: (box: AABB, depth: number, isLeaf: boolean) => void): void {
    this.tree.forEachNode(visit);
  }

  /** Proxies whose fat box overlaps `box`. */
  query(box: AABB): number[] {
    return this.tree.query(box);
  }

  raycast(
    ox: number,
    oy: number,
    oz: number,
    dx: number,
    dy: number,
    dz: number,
    maxT: number,
  ): TreeRayHit | null {
    return this.tree.raycast(ox, oy, oz, dx, dy, dz, maxT);
  }

  private markDirty(id: number): void {
    if (this.isDirty.has(id)) return;
    this.isDirty.add(id);
    this.dirty.push(id);
  }

  private addPair(a: number, b: number): void {
    const key = pairKey(a, b);
    if (this.pairs.has(key)) return;
    const lo = a < b ? a : b;
    const hi = a < b ? b : a;
    this.pairs.set(key, [lo, hi]);
    this.sortedValid = false;
    if (this.opLog.length <= 48) this.opLog.push(key);
    this.partners.get(a)?.add(b);
    this.partners.get(b)?.add(a);
    this.changes.push({ kind: 'begin', a: lo, b: hi });
  }

  private removePair(a: number, b: number): void {
    const key = pairKey(a, b);
    if (!this.pairs.has(key)) return;
    this.pairs.delete(key);
    this.sortedValid = false;
    if (this.opLog.length <= 48) this.opLog.push(-key - 1);
    this.partners.get(a)?.delete(b);
    this.partners.get(b)?.delete(a);
    this.changes.push({ kind: 'end', a: Math.min(a, b), b: Math.max(a, b) });
  }
}
