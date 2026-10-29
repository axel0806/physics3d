import { describe, it, expect } from 'vitest';
import fc from 'fast-check';
import { FC_SEED } from '../helpers/fc';
import { Vec3, Quat, Mat3, Transform } from '../../src/math';
import * as Q from '../../src/math/quat';
import * as M3 from '../../src/math/mat3';
import {
  Box,
  Sphere,
  Capsule,
  Compound,
  ConvexHull,
  quickhull,
  polyhedronMassProperties,
  type MassProperties,
} from '../../src/shapes';

const NUM = { numRuns: 300, seed: FC_SEED };
const d = fc.double({ min: -1, max: 1, noNaN: true });
const quat = fc
  .tuple(d, d, d, d)
  .filter(([x, y, z, w]) => x * x + y * y + z * z + w * w > 1e-2)
  .map(([x, y, z, w]) => Q.normalize(new Quat(), new Quat(x, y, z, w)));
const vec = fc.tuple(d, d, d).map(([x, y, z]) => new Vec3(x, y, z));

const rotV = (q: Quat, v: Vec3) => Q.rotate(new Vec3(), q, v);
const add = (p: Vec3, t: Vec3) => new Vec3(p.x + t.x, p.y + t.y, p.z + t.z);
/** R diag(x, y, z) Rᵀ */
function rotInertia(q: Quat, x: number, y: number, z: number): Mat3 {
  const R = M3.fromQuat(new Mat3(), q);
  const RD = M3.mul(new Mat3(), R, M3.fromDiagonal(new Mat3(), x, y, z));
  return M3.mul(new Mat3(), RD, M3.transpose(new Mat3(), R));
}
function corners(hx: number, hy: number, hz: number): Vec3[] {
  const out: Vec3[] = [];
  for (const x of [-hx, hx])
    for (const y of [-hy, hy]) for (const z of [-hz, hz]) out.push(new Vec3(x, y, z));
  return out;
}
/** Largest elementwise error, relative to the largest entry of b. */
function matErr(a: Mat3, b: Mat3): number {
  const s = Math.max(...Array.from(b.e, Math.abs), Number.MIN_VALUE);
  let worst = 0;
  for (let i = 0; i < 9; i++)
    worst = Math.max(worst, Math.abs((a.e[i] as number) - (b.e[i] as number)) / s);
  return worst;
}
function hullOrNull(pts: Vec3[]): ConvexHull | null {
  try {
    return new ConvexHull(pts);
  } catch (e) {
    if (e instanceof RangeError) return null;
    throw e;
  }
}
/** Every directed edge has its reverse; V - E + F = 2 with E = 3F/2; all vertices used. */
function checkManifold(h: ConvexHull): void {
  const F = h.indices.length / 3;
  const directed = new Set<string>();
  for (let t = 0; t < F; t++)
    for (let k = 0; k < 3; k++) {
      const a = h.indices[3 * t + k] as number;
      const b = h.indices[3 * t + ((k + 1) % 3)] as number;
      expect(directed.has(`${a},${b}`)).toBe(false);
      directed.add(`${a},${b}`);
    }
  for (const e of directed) expect(directed.has(e.split(',').reverse().join(','))).toBe(true);
  const used = new Set(h.indices);
  expect(used.size - directed.size / 2 + F).toBe(2);
  expect(used.size).toBe(h.vertices.length);
}
function expectMassClose(a: MassProperties, b: MassProperties, rel: number): void {
  expect(Math.abs(a.mass - b.mass)).toBeLessThanOrEqual(rel * Math.abs(b.mass));
  expect(a.centerOfMass.x).toBeCloseTo(b.centerOfMass.x, 9);
  expect(a.centerOfMass.y).toBeCloseTo(b.centerOfMass.y, 9);
  expect(a.centerOfMass.z).toBeCloseTo(b.centerOfMass.z, 9);
  expect(matErr(a.inertia, b.inertia)).toBeLessThanOrEqual(rel);
}

describe('box hulls vs analytic Box', () => {
  for (const s of [1e-3, 1, 1e3]) {
    it(`corners at scale ${s} match Box to 1e-9`, () => {
      const hx = 0.7 * s,
        hy = 1.3 * s,
        hz = 2.1 * s;
      expectMassClose(
        new ConvexHull(corners(hx, hy, hz)).massProperties(1000),
        new Box(hx, hy, hz).massProperties(1000),
        1e-9,
      );
    });
  }
  it('rotated, translated hull: mass exact, COM = t, inertia = R I Rᵀ', () => {
    fc.assert(
      fc.property(quat, vec, (q, v) => {
        const t = new Vec3(v.x * 10, v.y * 10, v.z * 10);
        const mp = new ConvexHull(corners(0.5, 1, 2).map((p) => add(rotV(q, p), t))).massProperties(
          1,
        );
        expect(Math.abs(mp.mass - 8)).toBeLessThan(1e-9 * 8);
        expect(
          Math.hypot(mp.centerOfMass.x - t.x, mp.centerOfMass.y - t.y, mp.centerOfMass.z - t.z),
        ).toBeLessThan(1e-9);
        expect(
          matErr(mp.inertia, rotInertia(q, (8 / 3) * 5, (8 / 3) * 4.25, (8 / 3) * 1.25)),
        ).toBeLessThan(1e-9);
      }),
      NUM,
    );
  });
  it('offset hulls: offset 1e2 meets 1e-9; offset 1e4 does not', () => {
    const err = (off: number) =>
      matErr(
        new ConvexHull(corners(1, 1, 1).map((p) => add(p, new Vec3(off, off, off)))).massProperties(
          1,
        ).inertia,
        new Box(1, 1, 1).massProperties(1).inertia,
      );
    expect(err(1e2)).toBeLessThan(1e-9);
    expect(err(1e4)).toBeLessThan(1e-9); // FAILS: origin-based sums cancel ~eps*|c|^2
  });
});

describe('polyhedron mass: analytic checks', () => {
  it('unit tetrahedron: volume 1/6, centroid (1/4,1/4,1/4), I_xx = 1/80, I_xy = 1/480', () => {
    const pts = [new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
    const mp = polyhedronMassProperties(pts, [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3], 1);
    expect(mp.mass).toBeCloseTo(1 / 6, 14);
    for (const c of [mp.centerOfMass.x, mp.centerOfMass.y, mp.centerOfMass.z])
      expect(c).toBeCloseTo(0.25, 14);
    for (const i of [0, 4, 8]) expect(mp.inertia.e[i]).toBeCloseTo(1 / 80, 14);
    expect(mp.inertia.e[1]).toBeCloseTo(1 / 480, 14);
    expect(mp.inertia.e[5]).toBeCloseTo(1 / 480, 14);
  });
  it('regular octahedron (vertices ±e_i): 8 faces, mass 4/3, I = m/5 per axis', () => {
    const h = new ConvexHull([
      new Vec3(1, 0, 0),
      new Vec3(-1, 0, 0),
      new Vec3(0, 1, 0),
      new Vec3(0, -1, 0),
      new Vec3(0, 0, 1),
      new Vec3(0, 0, -1),
    ]);
    checkManifold(h);
    expect(h.triangleCount).toBe(8);
    const mp = h.massProperties(1);
    expect(mp.mass).toBeCloseTo(4 / 3, 12);
    for (const i of [0, 4, 8]) expect(mp.inertia.e[i]).toBeCloseTo(4 / 3 / 5, 12);
  });
  it('inward-wound triangles are rejected', () => {
    const pts = [new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
    expect(() => polyhedronMassProperties(pts, [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2], 1)).toThrow(
      RangeError,
    );
  });
  it('64-segment cylinder hull is within 0.5% of analytic mass and inertia', () => {
    const n = 64,
      r = 1,
      h = 1,
      pts: Vec3[] = [];
    for (let i = 0; i < n; i++) {
      const a = (2 * Math.PI * i) / n;
      pts.push(
        new Vec3(r * Math.cos(a), h, r * Math.sin(a)),
        new Vec3(r * Math.cos(a), -h, r * Math.sin(a)),
      );
    }
    const mp = new ConvexHull(pts).massProperties(1);
    const m = Math.PI * r * r * 2 * h;
    expect(Math.abs(mp.mass / m - 1)).toBeLessThan(0.005);
    expect(Math.abs(mp.inertia.e[4]! / ((m * r * r) / 2) - 1)).toBeLessThan(0.005);
    expect(Math.abs(mp.inertia.e[0]! / ((m * (3 * r * r + 4 * h * h)) / 12) - 1)).toBeLessThan(
      0.005,
    );
  });
  it('capsule matches Simpson slice integration of the radial profile', () => {
    const r = 0.8,
      h = 1.5,
      H = h + r,
      N = 20000;
    const R2 = (y: number) => (Math.abs(y) <= h ? r * r : r * r - (Math.abs(y) - h) ** 2);
    const simpson = (f: (y: number) => number, lo: number, hi: number) => {
      const dy = (hi - lo) / N;
      let s = f(lo) + f(hi);
      for (let i = 1; i < N; i++) s += f(lo + i * dy) * (i % 2 ? 4 : 2);
      return (s * dy) / 3;
    };
    let m = 0,
      ixx = 0,
      iyy = 0;
    for (const [lo, hi] of [
      [-H, -h],
      [-h, h],
      [h, H],
    ] as const) {
      m += simpson((y) => Math.PI * R2(y), lo, hi);
      ixx += simpson((y) => Math.PI * (R2(y) * y * y + (R2(y) * R2(y)) / 4), lo, hi);
      iyy += simpson((y) => (Math.PI * R2(y) * R2(y)) / 2, lo, hi);
    }
    const mp = new Capsule(r, h).massProperties(1);
    expect(Math.abs(mp.mass / m - 1)).toBeLessThan(1e-9);
    expect(Math.abs(mp.inertia.e[0]! / ixx - 1)).toBeLessThan(1e-8);
    expect(Math.abs(mp.inertia.e[4]! / iyy - 1)).toBeLessThan(1e-8);
  });
  it('capsule with zero half-height equals a sphere', () => {
    expectMassClose(new Capsule(2, 0).massProperties(1), new Sphere(2).massProperties(1), 1e-12);
  });
});

describe('quickhull: spheres, degenerate and random inputs', () => {
  it('Fibonacci-sphere hulls approach 4/3 pi r^3 from below, Euler holds', () => {
    const r = 3;
    let prev = 0;
    for (const n of [200, 1000, 4000]) {
      const pts: Vec3[] = [];
      for (let i = 0; i < n; i++) {
        const y = 1 - (2 * (i + 0.5)) / n,
          rad = Math.sqrt(1 - y * y),
          th = i * Math.PI * (3 - Math.sqrt(5));
        pts.push(new Vec3(r * Math.cos(th) * rad, r * y, r * Math.sin(th) * rad));
      }
      const h = new ConvexHull(pts);
      checkManifold(h);
      const ratio = h.massProperties(1).mass / ((4 / 3) * Math.PI * r ** 3);
      expect(ratio).toBeLessThan(1);
      expect(ratio).toBeGreaterThan(prev);
      prev = ratio;
    }
    expect(prev).toBeGreaterThan(0.998);
  }, 60_000);
  it('cube with corners tripled plus interior points: 12 faces, exact volume', () => {
    const base = corners(1, 1, 1);
    const h = new ConvexHull([
      ...base,
      ...base,
      ...base,
      new Vec3(0, 0, 0),
      new Vec3(0.5, -0.2, 0.1),
    ]);
    checkManifold(h);
    expect(h.triangleCount).toBe(12);
    expect(h.massProperties(1).mass).toBeCloseTo(8, 9);
  });
  it('coplanar, collinear and coincident inputs throw RangeError', () => {
    const v = (...c: number[][]) => c.map(([x, y, z]) => new Vec3(x, y, z));
    expect(() => quickhull(v([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]))).toThrow(RangeError);
    expect(() => quickhull(v([0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]))).toThrow(RangeError);
    expect(() => quickhull(v([1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]))).toThrow(RangeError);
  });
  it('needle tetrahedron (aspect 1e-5) accepted at scales 1e-6 .. 1e3 (1e-6 FAILS)', () => {
    for (const s of [1e-6, 1e-3, 1, 1e3]) {
      const pts = [
        new Vec3(0, 0, 0),
        new Vec3(s, 0, 0),
        new Vec3(0.5 * s, 1e-5 * s, 0),
        new Vec3(0.5 * s, 0, 1e-5 * s),
      ];
      const h = hullOrNull(pts);
      expect(h, `scale ${s} rejected as degenerate`).not.toBeNull();
      // exact volume |det(AB, AC, AD)| / 6 = s^3 * 1e-10 / 6
      expect(
        Math.abs((h?.massProperties(1).mass ?? NaN) / ((s ** 3 * 1e-10) / 6) - 1),
      ).toBeLessThan(1e-9);
    }
  });
  it('random clouds scaled 1e-3 .. 1e3: manifold, inside-or-on, valid inertia', () => {
    fc.assert(
      fc.property(
        fc.array(vec, { minLength: 4, maxLength: 40 }),
        fc.constantFrom(1e-3, 1, 1e3),
        (raw, s) => {
          const pts = raw.map((p) => new Vec3(p.x * s, p.y * s, p.z * s));
          const h = hullOrNull(pts);
          if (!h) return;
          checkManifold(h);
          const mp = h.massProperties(1),
            I = mp.inertia.e;
          expect(mp.mass).toBeGreaterThan(0);
          // I_xx + I_yy - I_zz = 2 ∫z² dm >= 0 for any physical body
          expect(I[0]! + I[4]! + 1e-9 * (I[0]! + I[4]! + I[8]!)).toBeGreaterThanOrEqual(I[8]!);
          for (let t = 0; t < h.triangleCount; t++) {
            const a = h.vertices[h.indices[3 * t] as number] as Vec3;
            const b = h.vertices[h.indices[3 * t + 1] as number] as Vec3;
            const c = h.vertices[h.indices[3 * t + 2] as number] as Vec3;
            const n = new Vec3(
              (b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y),
              (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z),
              (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x),
            );
            const L = Math.hypot(n.x, n.y, n.z);
            if (L < 1e-4 * s * s) continue; // slivers: normal too noisy to test
            for (const p of pts) {
              expect(
                ((p.x - a.x) * n.x + (p.y - a.y) * n.y + (p.z - a.z) * n.z) / L,
              ).toBeLessThanOrEqual(1e-7 * s);
            }
          }
        },
      ),
      NUM,
    );
  });
  it('uniform scaling by 1000: mass ~ s^3, inertia ~ s^5', () => {
    fc.assert(
      fc.property(fc.array(vec, { minLength: 8, maxLength: 30 }), (raw) => {
        const base = hullOrNull(raw);
        if (!base) return;
        const s = 1000;
        const big = new ConvexHull(
          raw.map((p) => new Vec3(p.x * s, p.y * s, p.z * s)),
        ).massProperties(1);
        const small = base.massProperties(1);
        expect(Math.abs(big.mass / (s ** 3 * small.mass) - 1)).toBeLessThan(1e-9);
        expect(matErr(big.inertia, new Mat3(small.inertia.e.map((x) => x * s ** 5)))).toBeLessThan(
          1e-9,
        );
      }),
      NUM,
    );
  });
});

describe('compounds and support functions', () => {
  it('two touching unit cubes equal a box of half extents (2,1,1)', () => {
    const c = new Compound([
      { shape: new Box(1, 1, 1), local: new Transform(new Vec3(-1, 0, 0)) },
      { shape: new Box(1, 1, 1), local: new Transform(new Vec3(1, 0, 0)) },
    ]);
    expectMassClose(c.massProperties(1), new Box(2, 1, 1).massProperties(1), 1e-12);
  });
  it('rotated, translated child equals the hull of its transformed corners', () => {
    fc.assert(
      fc.property(quat, vec, (q, t) => {
        const comp = new Compound([
          { shape: new Box(0.5, 1, 2), local: new Transform(t, q) },
        ]).massProperties(1);
        const hull = new ConvexHull(
          corners(0.5, 1, 2).map((p) => add(rotV(q, p), t)),
        ).massProperties(1);
        expectMassClose(comp, hull, 1e-9);
      }),
      NUM,
    );
  });
  it('support functions attain the directional maximum (sphere, box, capsule)', () => {
    fc.assert(
      fc.property(vec, (dir) => {
        const len = Math.hypot(dir.x, dir.y, dir.z);
        if (len < 1e-3) return;
        const dot = (p: Vec3) => p.x * dir.x + p.y * dir.y + p.z * dir.z;
        const out = new Vec3();
        expect(dot(new Sphere(1.5).support(dir, out))).toBeCloseTo(1.5 * len, 9);
        expect(dot(new Box(0.5, 1, 2).support(dir, out))).toBeCloseTo(
          Math.max(...corners(0.5, 1, 2).map(dot)),
          12,
        );
        const cmax = Math.max(dot(new Vec3(0, 1.2, 0)), dot(new Vec3(0, -1.2, 0))) + 0.7 * len;
        expect(dot(new Capsule(0.7, 1.2).support(dir, out))).toBeCloseTo(cmax, 9);
      }),
      NUM,
    );
  });
});
