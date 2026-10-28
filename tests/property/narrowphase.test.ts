import { describe, expect, it } from 'vitest';
import { Vec3, Quat, Transform } from '../../src/math';
import { Box, Sphere, Capsule } from '../../src/shapes';
import { collideConvex } from '../../src/collision/narrowphase';
import { orientedBox, boxBoxManifold, satBoxBox } from '../../src/collision/narrowphase/sat';
import {
  sphereBox,
  capsuleCapsule,
  makeContactPoint,
} from '../../src/collision/narrowphase/analytic';
import { boxCorners, referenceMinkowski } from '../helpers/reference';
import { mulberry32 } from '../helpers/rng';

/** Sample count per pair type. `pnpm bench:accuracy` runs the full 100,000 configurations. */
const N = Number(process.env.NARROWPHASE_SAMPLES ?? 1500);
const DEG = Math.PI / 180;

function randomRotation(r: () => number): Quat {
  // Uniform random unit quaternion (Shoemake).
  const u1 = r();
  const u2 = r() * 2 * Math.PI;
  const u3 = r() * 2 * Math.PI;
  const a = Math.sqrt(1 - u1);
  const b = Math.sqrt(u1);
  return new Quat(a * Math.sin(u2), a * Math.cos(u2), b * Math.sin(u3), b * Math.cos(u3));
}

function angleBetween(a: Vec3, b: Vec3): number {
  return Math.acos(Math.min(1, Math.max(-1, a.dot(b))));
}

describe('narrowphase: GJK/EPA against analytic answers', () => {
  it('sphere-sphere: depth within 1e-4, normal within 0.5°, distance within 1e-6', () => {
    const r = mulberry32(1001);
    let worstDepth = 0;
    let worstAngle = 0;
    let worstDist = 0;
    for (let k = 0; k < N; k++) {
      const ra = 0.1 + r() * 3;
      const rb = 0.1 + r() * 3;
      const dir = new Vec3(r() * 2 - 1, r() * 2 - 1, r() * 2 - 1).normalized();
      const dist = (ra + rb) * (0.2 + r() * 1.6);
      const A = { shape: new Sphere(ra), transform: new Transform(new Vec3(), new Quat()) };
      const B = { shape: new Sphere(rb), transform: new Transform(dir.scale(dist), new Quat()) };
      const c = collideConvex(A, B);
      const trueDepth = ra + rb - dist;
      if (trueDepth >= 0) {
        expect(c.overlap).toBe(true);
        if (!c.overlap) continue;
        worstDepth = Math.max(worstDepth, Math.abs(c.depth - trueDepth) / Math.max(1, ra + rb));
        worstAngle = Math.max(worstAngle, angleBetween(c.normal, dir));
      } else {
        expect(c.overlap).toBe(false);
        if (c.overlap) continue;
        worstDist = Math.max(worstDist, Math.abs(c.distance + trueDepth));
        worstAngle = Math.max(worstAngle, angleBetween(c.normal, dir));
      }
    }
    expect(worstDepth).toBeLessThan(1e-4);
    expect(worstDist).toBeLessThan(1e-6);
    expect(worstAngle).toBeLessThan(0.5 * DEG);
  }, 3_600_000);

  it('sphere-box: analytic path and GJK/EPA agree (depth 1e-4, normal 0.5°, distance 1e-6)', () => {
    const r = mulberry32(2002);
    let worstDepth = 0;
    let worstAngle = 0;
    let worstDist = 0;
    for (let k = 0; k < N; k++) {
      const rs = 0.05 + r() * 0.8;
      const h = [0.2 + r(), 0.2 + r(), 0.2 + r()] as const;
      const S = {
        shape: new Sphere(rs),
        transform: new Transform(
          new Vec3(3 * (r() * 2 - 1), 3 * (r() * 2 - 1), 3 * (r() * 2 - 1)),
          new Quat(),
        ),
      };
      const q = randomRotation(r);
      const Bx = { shape: new Box(h[0], h[1], h[2]), transform: new Transform(new Vec3(), q) };
      const ob = orientedBox(Bx.shape, Bx.transform);
      const analytic = sphereBox(S.transform.p, rs, ob.center, ob.axes, new Vec3(h[0], h[1], h[2]));
      const g = collideConvex(S, Bx);
      if (!analytic) {
        expect(g.overlap).toBe(false);
        if (g.overlap) continue;
        // Separated: the distance must match the analytic sphere-to-box surface distance.
        worstDist = Math.max(
          worstDist,
          Math.abs(g.distance - sphereBoxGap(S.transform.p, rs, Bx.transform, h)),
        );
        continue;
      }
      expect(g.overlap).toBe(true);
      if (!g.overlap) continue;
      const pt = analytic.points[0];
      if (!pt) throw new Error('sphere-box analytic contact missing');
      worstDepth = Math.max(worstDepth, Math.abs(pt.depth - g.depth));
      worstAngle = Math.max(worstAngle, angleBetween(analytic.normal, g.normal));
    }
    expect(worstDepth).toBeLessThan(1e-4);
    expect(worstAngle).toBeLessThan(0.5 * DEG);
    expect(worstDist).toBeLessThan(1e-6);
  }, 3_600_000);

  it('box-box: manifold depth and normal agree with GJK/EPA; separated distance matches the Minkowski hull', () => {
    const r = mulberry32(3003);
    let worstDepth = 0;
    let worstAngle = 0;
    let checkedSeparated = 0;
    let worstDist = 0;
    for (let k = 0; k < N; k++) {
      const ha = [0.2 + r(), 0.2 + r(), 0.2 + r()] as [number, number, number];
      const hb = [0.2 + r(), 0.2 + r(), 0.2 + r()] as [number, number, number];
      const ta = new Transform(
        new Vec3(0.3 * (r() * 2 - 1), 0.3 * (r() * 2 - 1), 0.3 * (r() * 2 - 1)),
        randomRotation(r),
      );
      const tb = new Transform(
        new Vec3(2.4 * (r() * 2 - 1), 2.4 * (r() * 2 - 1), 2.4 * (r() * 2 - 1)),
        randomRotation(r),
      );
      const A = { shape: new Box(ha[0], ha[1], ha[2]), transform: ta };
      const B = { shape: new Box(hb[0], hb[1], hb[2]), transform: tb };
      const oa = orientedBox(A.shape, ta);
      const ob = orientedBox(B.shape, tb);
      const m = boxBoxManifold(oa, ob);
      const g = collideConvex(A, B);
      const ref = referenceMinkowski(boxCorners(ha, ta), boxCorners(hb, tb));
      expect(ref.inside).toBe(g.overlap);
      if (!g.overlap) {
        expect(m).toBeNull();
        if (checkedSeparated < 300) {
          worstDist = Math.max(worstDist, Math.abs(g.distance - ref.distance));
          checkedSeparated++;
        }
        continue;
      }
      expect(m).not.toBeNull();
      if (!m) continue;
      const depth = Math.max(...m.points.map((p) => p.depth));
      worstDepth = Math.max(worstDepth, Math.abs(depth - ref.depth));
      worstAngle = Math.max(worstAngle, angleBetween(m.normal, g.normal));
    }
    expect(worstDepth).toBeLessThan(1e-4);
    expect(worstAngle).toBeLessThan(0.5 * DEG);
    expect(worstDist).toBeLessThan(1e-6);
    expect(checkedSeparated).toBeGreaterThan(0);
  }, 3_600_000);
});

describe('narrowphase: specialized paths agree with GJK/EPA', () => {
  it('capsule-capsule closed form matches GJK/EPA on random pairs', () => {
    const r = mulberry32(4004);
    let worstDepth = 0;
    for (let k = 0; k < Math.min(N, 800); k++) {
      const ra = 0.05 + r() * 0.5;
      const rb = 0.05 + r() * 0.5;
      const ha = 0.2 + r();
      const hb = 0.2 + r();
      const qa = randomRotation(r);
      const qb = randomRotation(r);
      const ta = new Transform(
        new Vec3(0.2 * (r() * 2 - 1), 0.2 * (r() * 2 - 1), 0.2 * (r() * 2 - 1)),
        qa,
      );
      const tb = new Transform(
        new Vec3(1.2 * (r() * 2 - 1), 1.2 * (r() * 2 - 1), 1.2 * (r() * 2 - 1)),
        qb,
      );
      const A = { shape: new Capsule(ra, ha), transform: ta };
      const B = { shape: new Capsule(rb, hb), transform: tb };
      const segA = segmentOf(ta, ha);
      const segB = segmentOf(tb, hb);
      const m = capsuleCapsule(segA[0], segA[1], ra, segB[0], segB[1], rb);
      const g = collideConvex(A, B);
      expect(m !== null).toBe(g.overlap);
      if (!m || !g.overlap) continue;
      worstDepth = Math.max(worstDepth, Math.abs((m.points[0]?.depth ?? 0) - g.depth));
    }
    expect(worstDepth).toBeLessThan(1e-4);
  }, 3_600_000);

  it('box-box SAT depth equals the independent Minkowski hull depth on random pairs', () => {
    const r = mulberry32(5005);
    let worst = 0;
    for (let k = 0; k < Math.min(N, 800); k++) {
      const ha = [0.2 + r(), 0.2 + r(), 0.2 + r()] as [number, number, number];
      const hb = [0.2 + r(), 0.2 + r(), 0.2 + r()] as [number, number, number];
      const ta = new Transform(
        new Vec3(0.2 * (r() * 2 - 1), 0.2 * (r() * 2 - 1), 0.2 * (r() * 2 - 1)),
        randomRotation(r),
      );
      const tb = new Transform(
        new Vec3(1.5 * (r() * 2 - 1), 1.5 * (r() * 2 - 1), 1.5 * (r() * 2 - 1)),
        randomRotation(r),
      );
      const ref = referenceMinkowski(boxCorners(ha, ta), boxCorners(hb, tb));
      const sat = satBoxBox(
        orientedBox(new Box(ha[0], ha[1], ha[2]), ta),
        orientedBox(new Box(hb[0], hb[1], hb[2]), tb),
      );
      if (!ref.inside) {
        expect(sat.separated).toBe(true);
        continue;
      }
      worst = Math.max(worst, Math.abs(sat.depth - ref.depth));
    }
    expect(worst).toBeLessThan(1e-6);
  }, 3_600_000);
});

describe('narrowphase: stable manifolds', () => {
  it('a box resting on a box gives a stable four-point manifold with persistent feature ids', () => {
    const ground = new Box(5, 0.5, 5);
    const crate = new Box(0.5, 0.5, 0.5);
    const groundT = new Transform(new Vec3(0, 0, 0), new Quat());
    const frames: { ids: number[]; depths: number[] }[] = [];
    for (let f = 0; f < 5; f++) {
      const crateT = new Transform(new Vec3(0.1, 1.0 - 0.01, -0.05), new Quat());
      const m = boxBoxManifold(orientedBox(crate, crateT), orientedBox(ground, groundT));
      if (!m) throw new Error('resting crate must be in contact');
      frames.push({
        ids: m.points.map((p) => p.id),
        depths: m.points.map((p) => p.depth),
      });
    }
    const first = frames[0];
    expect(first?.ids.length).toBe(4);
    for (const fr of frames) {
      expect(fr.ids).toEqual(first?.ids);
      expect(fr.depths).toEqual(first?.depths);
    }
    for (const d of first?.depths ?? []) expect(d).toBeCloseTo(0.01, 9);
  });

  it('makeContactPoint averages the surface points', () => {
    const p = makeContactPoint(new Vec3(0, 0, 0), new Vec3(2, 0, 0), 0.5, 7);
    expect(p.position).toEqual(new Vec3(1, 0, 0));
    expect(p.id).toBe(7);
  });
});

describe('narrowphase: adversarial inputs terminate and stay finite', () => {
  it('touching, coincident, thin, and huge-ratio shapes never produce NaN or throw', () => {
    const r = mulberry32(6006);
    const shapesFor = (k: number) => {
      const scale = k % 3 === 0 ? 1e4 : k % 3 === 1 ? 1e-3 : 1;
      return scale;
    };
    for (let k = 0; k < Math.min(N, 600); k++) {
      const s = shapesFor(k);
      const A = {
        shape: new Box(s * (0.5 + r()), s * 1e-3 * (1 + r()), s * (0.5 + r())),
        transform: new Transform(new Vec3(), randomRotation(r)),
      };
      // Touching: B's face at A's face plane.
      const B = {
        shape: new Box(s * (0.5 + r()), s * 0.5, s * 0.5),
        transform: new Transform(
          new Vec3(0, s * 1e-3 + s * 0.5, 0),
          k % 2 ? new Quat() : randomRotation(r),
        ),
      };
      const g = collideConvex(A, B);
      expect(Number.isFinite(g.overlap ? g.depth : g.distance)).toBe(true);
      expect(Number.isFinite(g.normal.x + g.normal.y + g.normal.z)).toBe(true);
      const C = { shape: new Sphere(s), transform: new Transform(new Vec3(), new Quat()) };
      const D = { shape: new Sphere(s), transform: new Transform(new Vec3(0, 0, 0), new Quat()) }; // coincident centers
      const c = collideConvex(C, D);
      expect(c.overlap).toBe(true);
      expect(Number.isFinite(c.overlap ? c.depth : c.distance)).toBe(true);
    }
  }, 3_600_000);
});

/** Analytic gap between a sphere (center c, radius r) and an oriented box (transform t, half extents h). */
function sphereBoxGap(
  c: Vec3,
  r: number,
  t: Transform,
  h: readonly [number, number, number],
): number {
  const rel = new Vec3(c.x - t.p.x, c.y - t.p.y, c.z - t.p.z);
  const inv = new Quat(-t.q.x, -t.q.y, -t.q.z, t.q.w);
  const local = rotate(inv, rel);
  const dx = Math.max(0, Math.abs(local.x) - h[0]);
  const dy = Math.max(0, Math.abs(local.y) - h[1]);
  const dz = Math.max(0, Math.abs(local.z) - h[2]);
  return Math.hypot(dx, dy, dz) - r;
}

function rotate(q: Quat, v: Vec3): Vec3 {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return new Vec3(
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx),
  );
}

/** World-space endpoints of a capsule's core segment (local axis Y, half height h). */
function segmentOf(t: Transform, h: number): [Vec3, Vec3] {
  const a = t.p.clone();
  const dir = new Vec3(0, 1, 0);
  const rotated = new Vec3(0, 0, 0);
  // Apply rotation: rotate the Y axis by t.q.
  const q = t.q;
  const tx = 2 * (q.y * dir.z - q.z * dir.y);
  const ty = 2 * (q.z * dir.x - q.x * dir.z);
  const tz = 2 * (q.x * dir.y - q.y * dir.x);
  rotated.x = dir.x + q.w * tx + (q.y * tz - q.z * ty);
  rotated.y = dir.y + q.w * ty + (q.z * tx - q.x * tz);
  rotated.z = dir.z + q.w * tz + (q.x * ty - q.y * tx);
  const p = new Vec3(a.x - rotated.x * h, a.y - rotated.y * h, a.z - rotated.z * h);
  const e = new Vec3(a.x + rotated.x * h, a.y + rotated.y * h, a.z + rotated.z * h);
  return [p, e];
}
