import { describe, expect, it } from 'vitest';
import { Vec3, Quat, Transform } from '../../src/math';
import { Sphere } from '../../src/shapes';
import { collideConvex } from '../../src/collision/narrowphase';
import { mulberry32 } from '../helpers/rng';

/**
 * Sphere pairs make GJK terminate on a 2-point simplex (origin on the center line). EPA started
 * from the flat tetrahedron around it used to run all 256 iterations on a degenerate, exploding
 * polytope (~500 ms per sample, ~10% of overlapping pairs) before the restart direction succeeded.
 */
describe('EPA: degenerate GJK simplex on sphere pairs', () => {
  it('replays seed 1001 sample 152 with bounded work and exact answers', () => {
    const r = mulberry32(1001);
    for (let k = 0; ; k++) {
      const ra = 0.1 + r() * 3;
      const rb = 0.1 + r() * 3;
      const dir = new Vec3(r() * 2 - 1, r() * 2 - 1, r() * 2 - 1).normalized();
      const dist = (ra + rb) * (0.2 + r() * 1.6);
      if (k < 152) continue;
      const A = { shape: new Sphere(ra), transform: new Transform(new Vec3(), new Quat()) };
      const B = { shape: new Sphere(rb), transform: new Transform(dir.scale(dist), new Quat()) };
      const c = collideConvex(A, B);
      expect(c.overlap).toBe(true);
      if (!c.overlap) return;
      expect(c.converged).toBe(true);
      expect(c.iterations).toBeLessThan(100);
      expect(Math.abs(c.depth - (ra + rb - dist))).toBeLessThan(1e-4);
      expect(Math.acos(Math.min(1, c.normal.dot(dir)))).toBeLessThan((0.5 * Math.PI) / 180);
      return;
    }
  });

  it('never needs the iteration cap over 400 overlapping sphere pairs', () => {
    const r = mulberry32(1001);
    let worst = 0;
    const t0 = performance.now();
    for (let k = 0; k < 800; k++) {
      const ra = 0.1 + r() * 3;
      const rb = 0.1 + r() * 3;
      const dir = new Vec3(r() * 2 - 1, r() * 2 - 1, r() * 2 - 1).normalized();
      const dist = (ra + rb) * (0.2 + r() * 1.6);
      const A = { shape: new Sphere(ra), transform: new Transform(new Vec3(), new Quat()) };
      const B = { shape: new Sphere(rb), transform: new Transform(dir.scale(dist), new Quat()) };
      const c = collideConvex(A, B);
      if (c.overlap) worst = Math.max(worst, c.iterations);
    }
    expect(worst).toBeLessThan(256);
    // Was ~30 s before the fix; generous bound to stay machine independent.
    expect(performance.now() - t0).toBeLessThan(5000);
  });
});
