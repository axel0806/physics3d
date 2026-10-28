import { describe, expect, it } from 'vitest';
import { AABB, Vec3 } from '../../src/math';
import * as BB from '../../src/math/aabb';
import { BroadPhase, DynamicTree, pairKey } from '../../src/collision/broadphase';
import { mulberry32 } from '../helpers/rng';

const rand = mulberry32(0x5eed);
const range = (lo: number, hi: number) => lo + (hi - lo) * rand();

function randomBox(size: number): AABB {
  const x = range(-100, 100);
  const y = range(-100, 100);
  const z = range(-100, 100);
  const w = range(0.1, size);
  const h = range(0.1, size);
  const d = range(0.1, size);
  return new AABB(new Vec3(x, y, z), new Vec3(x + w, y + h, z + d));
}

describe('broadphase fuzz: pair set equals brute force', () => {
  it('5,000 random AABBs undergoing 1,000 random moves, exact at every step', () => {
    const N = 5000;
    const STEPS = 1000;
    const bp = new BroadPhase(0.1);
    const ids: number[] = [];
    const tight: AABB[] = [];
    for (let i = 0; i < N; i++) {
      const b = randomBox(12);
      tight.push(b);
      ids.push(bp.createProxy(b));
    }
    bp.updatePairs();
    const fatOf = (i: number) => bp.tree.fatBox(ids[i] as number, new AABB());
    const fatAll = () => Array.from({ length: N }, (_, i) => fatOf(i));
    const key = (i: number, j: number) => pairKey(ids[i] as number, ids[j] as number);

    // Reference pair set, kept by brute force over the moved proxy's row (O(n) per move) and
    // fully recomputed (O(n²)) every 50 steps, so the check is exact at every step.
    const ref = new Set<number>();
    for (let i = 0; i < N; i++) {
      for (let j = i + 1; j < N; j++) {
        if (BB.overlaps(fatOf(i), fatOf(j))) ref.add(key(i, j));
      }
    }
    expect(bp.pairList().length).toBe(ref.size);

    for (let step = 0; step < STEPS; step++) {
      const i = Math.floor(range(0, N));
      tight[i] = randomBox(12);
      const before = fatOf(i);
      bp.moveProxy(ids[i] as number, tight[i] as AABB);
      const after = fatOf(i);
      bp.updatePairs();
      if (BB.contains(before, after) && BB.contains(after, before)) continue; // fat box unchanged

      for (let j = 0; j < N; j++) {
        if (j === i) continue;
        const k = key(i, j);
        if (BB.overlaps(after, fatOf(j))) ref.add(k);
        else ref.delete(k);
      }

      if (step % 50 === 0) {
        const all = fatAll();
        const full = new Set<number>();
        for (let a = 0; a < N; a++) {
          for (let b = a + 1; b < N; b++) {
            if (BB.overlaps(all[a] as AABB, all[b] as AABB)) full.add(key(a, b));
          }
        }
        expect(full.size).toBe(ref.size);
        for (const k of full) expect(ref.has(k)).toBe(true);
        bp.tree.validate();
      }
      if (step % 10 === 0) {
        const tree = new Set(bp.pairList().map(([a, b]) => pairKey(a, b)));
        expect(tree.size).toBe(ref.size);
        for (const k of ref) expect(tree.has(k)).toBe(true);
      }
    }
    // Final full comparison against the O(n²) brute force.
    const all = fatAll();
    const expected: [number, number][] = [];
    for (let a = 0; a < N; a++) {
      for (let b = a + 1; b < N; b++) {
        if (BB.overlaps(all[a] as AABB, all[b] as AABB)) {
          expected.push([
            Math.min(ids[a] as number, ids[b] as number),
            Math.max(ids[a] as number, ids[b] as number),
          ]);
        }
      }
    }
    expected.sort((x, y) => x[0] - y[0] || x[1] - y[1]);
    expect(bp.pairList()).toEqual(expected);
    bp.tree.validate();
  }, 120_000);
});

describe('dynamic tree height', () => {
  it('stays within 2× of log₂ n after randomized insertion and removal', () => {
    for (const seed of [1, 2, 3, 4, 5]) {
      const r = mulberry32(seed * 7919);
      const tree = new DynamicTree(0);
      const live: number[] = [];
      const mk = () => {
        const x = r() * 1000;
        const y = r() * 1000;
        const z = r() * 1000;
        return tree.createProxy(new AABB(new Vec3(x, y, z), new Vec3(x + 1, y + 1, z + 1)));
      };
      for (let i = 0; i < 2000; i++) live.push(mk());
      for (let i = 0; i < 1000; i++) {
        const k = Math.floor(r() * live.length);
        tree.destroyProxy(live[k] as number);
        live.splice(k, 1);
      }
      for (let i = 0; i < 1000; i++) live.push(mk());
      tree.validate();
      const n = tree.leafCount;
      expect(tree.treeHeight).toBeLessThanOrEqual(2 * Math.log2(n));
    }
  });
});

describe('dynamic tree queries match brute force', () => {
  it('10,000 random rays: closest hit distance matches brute force', () => {
    const r = mulberry32(42);
    const tree = new DynamicTree(0);
    const boxes: AABB[] = [];
    for (let i = 0; i < 400; i++) {
      const x = r() * 20 - 10;
      const y = r() * 20 - 10;
      const z = r() * 20 - 10;
      const b = new AABB(new Vec3(x, y, z), new Vec3(x + r() * 2, y + r() * 2, z + r() * 2));
      boxes.push(b);
      tree.createProxy(b);
    }
    for (let k = 0; k < 10000; k++) {
      const o = [r() * 30 - 15, r() * 30 - 15, r() * 30 - 15];
      const d = [r() * 2 - 1, r() * 2 - 1, r() * 2 - 1];
      const maxT = 40;
      let bestT = Infinity;
      for (const b of boxes) {
        const t = BB.raycast(
          b,
          o[0] as number,
          o[1] as number,
          o[2] as number,
          d[0] as number,
          d[1] as number,
          d[2] as number,
          maxT,
        );
        if (t >= 0 && t < bestT) bestT = t;
      }
      const hit = tree.raycast(
        o[0] as number,
        o[1] as number,
        o[2] as number,
        d[0] as number,
        d[1] as number,
        d[2] as number,
        maxT,
      );
      if (bestT === Infinity) expect(hit).toBeNull();
      else {
        expect(hit).not.toBeNull();
        expect(Math.abs((hit as { t: number }).t - bestT)).toBeLessThan(1e-12);
      }
    }
  });

  it('random AABB queries return exactly the brute-force overlap set', () => {
    const r = mulberry32(99);
    const tree = new DynamicTree(0.2);
    const ids: number[] = [];
    for (let i = 0; i < 500; i++) {
      const x = r() * 50;
      const y = r() * 50;
      const z = r() * 50;
      ids.push(
        tree.createProxy(
          new AABB(new Vec3(x, y, z), new Vec3(x + r() * 3, y + r() * 3, z + r() * 3)),
        ),
      );
    }
    for (let k = 0; k < 2000; k++) {
      const x = r() * 50;
      const y = r() * 50;
      const z = r() * 50;
      const q = new AABB(new Vec3(x, y, z), new Vec3(x + r() * 5, y + r() * 5, z + r() * 5));
      const expected: number[] = [];
      for (const id of ids) {
        if (BB.overlaps(tree.fatBox(id, new AABB()), q)) expected.push(id);
      }
      expect(tree.query(q)).toEqual(expected.sort((a, b) => a - b));
    }
  });
});
