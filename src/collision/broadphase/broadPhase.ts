import { AABB } from '../../math';
import { DynamicTree, type TreeRayHit } from './dynamicTree';

/** Packs an unordered proxy pair into one number key (ids must be below 2^26). */
export function pairKey(a: number, b: number): number {
  const lo = a < b ? a : b;
  const hi = a < b ? b : a;
  return lo * 0x4000000 + hi;
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

  hasPair(a: number, b: number): boolean {
    return this.pairs.has(pairKey(a, b));
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
    this.partners.get(a)?.add(b);
    this.partners.get(b)?.add(a);
    this.changes.push({ kind: 'begin', a: lo, b: hi });
  }

  private removePair(a: number, b: number): void {
    const key = pairKey(a, b);
    if (!this.pairs.has(key)) return;
    this.pairs.delete(key);
    this.partners.get(a)?.delete(b);
    this.partners.get(b)?.delete(a);
    this.changes.push({ kind: 'end', a: Math.min(a, b), b: Math.max(a, b) });
  }
}
