import { describe, expect, it } from 'vitest';
import { AABB, Vec3 } from '../../src/math';
import { DynamicTree, BroadPhase, pairKey } from '../../src/collision/broadphase';

const box = (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number) =>
  new AABB(new Vec3(x0, y0, z0), new Vec3(x1, y1, z1));

describe('DynamicTree', () => {
  it('empty tree answers queries and raycasts with nothing', () => {
    const t = new DynamicTree();
    expect(t.treeHeight).toBe(-1);
    expect(t.query(box(0, 0, 0, 1, 1, 1))).toEqual([]);
    expect(t.raycast(0, 0, 0, 1, 0, 0, 10)).toBeNull();
    t.validate();
  });

  it('create, query, and destroy with fat boxes', () => {
    const t = new DynamicTree(0.5);
    const a = t.createProxy(box(0, 0, 0, 1, 1, 1));
    const b = t.createProxy(box(3, 0, 0, 4, 1, 1));
    expect(t.leafCount).toBe(2);
    // Fat box of a reaches x = 1.5, so a query starting at 1.4 hits a but not b.
    expect(t.query(box(1.4, 0, 0, 1.45, 1, 1))).toEqual([a]);
    expect(t.query(box(0, 0, 0, 10, 1, 1))).toEqual([a, b]);
    const tight = t.tightBox(a, new AABB());
    expect(tight.max).toEqual(new Vec3(1, 1, 1));
    t.destroyProxy(a);
    expect(t.leafCount).toBe(1);
    expect(t.query(box(0, 0, 0, 10, 1, 1))).toEqual([b]);
    t.validate();
    expect(() => t.tightBox(a, new AABB())).toThrow(RangeError);
    expect(() => t.destroyProxy(a)).toThrow(RangeError);
  });

  it('moveProxy is a no-op while the tight box stays inside the fat box', () => {
    const t = new DynamicTree(1);
    const a = t.createProxy(box(0, 0, 0, 1, 1, 1));
    expect(t.moveProxy(a, box(0.2, 0, 0, 1.2, 1, 1))).toBe(false);
    expect(t.moveProxy(a, box(5, 0, 0, 6, 1, 1))).toBe(true);
    expect(t.query(box(5.5, 0, 0, 5.6, 1, 1))).toEqual([a]);
    t.validate();
  });

  it('raycast returns the closest fat-box hit', () => {
    const t = new DynamicTree(0);
    const near = t.createProxy(box(2, -1, -1, 3, 1, 1));
    t.createProxy(box(5, -1, -1, 6, 1, 1));
    const hit = t.raycast(0, 0, 0, 1, 0, 0, 100);
    expect(hit).toEqual({ id: near, t: 2 });
    expect(t.raycast(0, 0, 0, 1, 0, 0, 1)).toBeNull();
    expect(t.raycast(0, 5, 0, 1, 0, 0, 100)).toBeNull();
  });

  it('rejects invalid margins', () => {
    expect(() => new DynamicTree(-1)).toThrow(RangeError);
    expect(() => new DynamicTree(Number.NaN)).toThrow(RangeError);
  });

  it('grows past its initial capacity and reuses freed nodes', () => {
    const t = new DynamicTree(0);
    const ids: number[] = [];
    for (let i = 0; i < 300; i++) ids.push(t.createProxy(box(i, 0, 0, i + 0.5, 1, 1)));
    for (const id of ids.slice(0, 150)) t.destroyProxy(id);
    for (let i = 0; i < 150; i++) t.createProxy(box(i, 2, 2, i + 0.5, 3, 3));
    expect(t.leafCount).toBe(300);
    t.validate();
  });

  it('removing the root leaf empties the tree', () => {
    const t = new DynamicTree(0);
    const only = t.createProxy(box(0, 0, 0, 1, 1, 1));
    t.destroyProxy(only);
    expect(t.rootIndex).toBe(-1);
    t.validate();
  });
});

describe('BroadPhase pair manager', () => {
  it('reports begin and end events and keeps a deduplicated pair set', () => {
    const bp = new BroadPhase(0);
    const a = bp.createProxy(box(0, 0, 0, 1, 1, 1));
    const b = bp.createProxy(box(0.5, 0, 0, 1.5, 1, 1));
    const c = bp.createProxy(box(10, 0, 0, 11, 1, 1));
    expect(bp.updatePairs()).toEqual([{ kind: 'begin', a, b }]);
    expect(bp.hasPair(a, b)).toBe(true);
    expect(bp.hasPair(c, a)).toBe(false);
    bp.moveProxy(b, box(9.5, 0, 0, 10.5, 1, 1));
    expect(bp.updatePairs()).toEqual([
      { kind: 'end', a, b },
      { kind: 'begin', a: b, b: c },
    ]);
    expect(bp.pairList()).toEqual([[b, c]]);
    bp.destroyProxy(c);
    expect(bp.updatePairs()).toEqual([{ kind: 'end', a: b, b: c }]);
    expect(bp.pairCount).toBe(0);
  });

  it('pairKey is symmetric and injective on small ids', () => {
    expect(pairKey(3, 9)).toBe(pairKey(9, 3));
    expect(pairKey(3, 9)).not.toBe(pairKey(3, 10));
  });

  it('query and raycast are forwarded to the tree', () => {
    const bp = new BroadPhase(0);
    const a = bp.createProxy(box(0, 0, 0, 1, 1, 1));
    expect(bp.query(box(0.5, 0.5, 0.5, 0.6, 0.6, 0.6))).toEqual([a]);
    expect(bp.raycast(-1, 0.5, 0.5, 1, 0, 0, 10)).toEqual({ id: a, t: 1 });
  });

  it('a destroyed proxy that was queued for update is skipped', () => {
    const bp = new BroadPhase(0);
    const a = bp.createProxy(box(0, 0, 0, 1, 1, 1));
    bp.destroyProxy(a);
    expect(bp.updatePairs()).toEqual([]);
  });
});
