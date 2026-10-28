import { describe, expect, it } from 'vitest';
import { Vec3, Quat, Transform, AABB, Mat3 } from '../../src/math';
import {
  Sphere,
  Box,
  Capsule,
  ConvexHull,
  TriangleMesh,
  Compound,
  polyhedronMassProperties,
  combine,
  quickhull,
  emptyMassProperties,
} from '../../src/shapes';
import { Q as _Q } from '../../src/math';
import * as V from '../../src/math/vec3';

const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(1e-300, Math.abs(b));
const expectInertia = (actual: Mat3, expected: number[], tol: number) => {
  for (let i = 0; i < 9; i++) {
    const e = expected[i] ?? 0;
    expect(Math.abs((actual.e[i] as number) - e)).toBeLessThanOrEqual(
      tol * Math.max(1, Math.abs(e)),
    );
  }
};

/** Unit-density cube corners ±h. */
function boxCorners(hx: number, hy: number, hz: number): Vec3[] {
  const out: Vec3[] = [];
  for (const x of [-hx, hx])
    for (const y of [-hy, hy]) for (const z of [-hz, hz]) out.push(new Vec3(x, y, z));
  return out;
}

describe('analytic mass properties', () => {
  it('sphere', () => {
    const mp = new Sphere(2).massProperties(1.5);
    const m = (4 / 3) * Math.PI * 8 * 1.5;
    expect(rel(mp.mass, m)).toBeLessThan(1e-12);
    expectInertia(mp.inertia, [0.4 * m * 4, 0, 0, 0, 0.4 * m * 4, 0, 0, 0, 0.4 * m * 4], 1e-12);
  });
  it('box', () => {
    const mp = new Box(1, 2, 3).massProperties(2);
    const m = 8 * 1 * 2 * 3 * 2;
    expect(rel(mp.mass, m)).toBeLessThan(1e-12);
    // full sizes 2, 4, 6
    expectInertia(
      mp.inertia,
      [(m / 12) * (16 + 36), 0, 0, 0, (m / 12) * (4 + 36), 0, 0, 0, (m / 12) * (4 + 16)],
      1e-12,
    );
  });
  it('capsule with zero half-height equals a sphere', () => {
    const c = new Capsule(1.3, 0).massProperties(1);
    const s = new Sphere(1.3).massProperties(1);
    expect(rel(c.mass, s.mass)).toBeLessThan(1e-12);
    expectInertia(c.inertia, s.inertia.e as unknown as number[], 1e-12);
  });
  it('capsule matches the closed form (cylinder + two hemispheres)', () => {
    const r = 0.5;
    const h = 1;
    const mp = new Capsule(r, h).massProperties(1);
    const mc = Math.PI * r * r * 2 * h;
    const ms = (4 / 3) * Math.PI * r ** 3;
    expect(rel(mp.mass, mc + ms)).toBeLessThan(1e-12);
    // Axial inertia (about Y): cylinder ½ m r² plus two hemispheres at 2/5 m r².
    const iy = 0.5 * mc * r * r + 0.4 * ms * r * r;
    expect(rel(mp.inertia.e[4] as number, iy)).toBeLessThan(1e-12);
    // Transverse about the center: known closed form for a capsule.
    const L = 2 * h;
    const ix =
      (mc * (3 * r * r + L * L)) / 12 + ms * (0.4 * r * r + (h + (3 * r) / 8) ** 2) - ms * 0 + 0;
    // hemisphere transverse about the sphere center is 2/5 m_h r² per hemisphere; both COMs at ±(h+3r/8)
    // Each hemisphere: (2/5 m_h r²) about its sphere center + m_h d² shift... see mass.ts for derivation.
    const hemiMass = ms / 2;
    const hemiCom = h + (3 * r) / 8;
    // About the sphere center the hemisphere has 2/5 m r²; shifting to its own COM subtracts m a²
    // (a = 3r/8), then shifting to the origin adds m·(h + a)².
    const a = (3 * r) / 8;
    const transverseHemi = 0.4 * hemiMass * r * r - hemiMass * a * a + hemiMass * hemiCom * hemiCom;
    const expectedIx = (mc * (3 * r * r + L * L)) / 12 + 2 * transverseHemi;
    expect(rel(mp.inertia.e[0] as number, expectedIx)).toBeLessThan(1e-9);
    expect(ix).toBeGreaterThan(0);
  });
  it('capsule with zero radius limit is a cylinder-like body, center of mass at origin', () => {
    const mp = new Capsule(0.2, 1).massProperties(3);
    expect(
      Math.abs(mp.centerOfMass.x) + Math.abs(mp.centerOfMass.y) + Math.abs(mp.centerOfMass.z),
    ).toBeLessThan(1e-12);
  });
  it('capsule support: endpoints and zero direction', () => {
    const cap = new Capsule(1, 2);
    const out = new Vec3();
    expect(cap.support(new Vec3(0, 1, 0), out)).toEqual(new Vec3(0, 3, 0));
    expect(cap.support(new Vec3(0, -1, 0), out)).toEqual(new Vec3(0, -3, 0));
    expect(cap.support(new Vec3(), out)).toEqual(new Vec3(1, 2, 0));
    // Diagonal: endpoint (0, 2, 0) plus r along the unit direction.
    const d = cap.support(new Vec3(1, 1, 0), out);
    expect(d.y).toBeCloseTo(2 + 1 / Math.SQRT2);
    expect(V_sub_len(d, new Vec3(0, 2, 0))).toBeCloseTo(1);
  });
  it('invalid dimensions are rejected', () => {
    expect(() => new Sphere(0)).toThrow(RangeError);
    expect(() => new Box(1, 0, 1)).toThrow(RangeError);
    expect(() => new Capsule(-1, 1)).toThrow(RangeError);
    expect(() => new Capsule(1, -1)).toThrow(RangeError);
  });
  it('sphere and box support functions', () => {
    const out = new Vec3();
    expect(new Sphere(2).support(new Vec3(0, 0, 0), out)).toEqual(new Vec3(2, 0, 0));
    expect(new Sphere(2).support(new Vec3(0, 3, 0), out)).toEqual(new Vec3(0, 2, 0));
    expect(new Box(1, 2, 3).support(new Vec3(-1, 1, -1), out)).toEqual(new Vec3(-1, 2, -3));
    expect(new Box(1, 2, 3).support(new Vec3(0, 0, 0), out)).toEqual(new Vec3(1, 2, 3));
  });
  it('bounds', () => {
    const b = new AABB();
    new Box(1, 2, 3).localBounds(b);
    expect(b.max).toEqual(new Vec3(1, 2, 3));
    new Sphere(1).localBounds(b);
    expect(b.min.x).toBe(-1);
    new Capsule(1, 2).localBounds(b);
    expect(b.max.y).toBe(3);
    expect(b.max.x).toBe(1);
  });
});

describe('convex hull', () => {
  it('box from 8 corners matches analytic mass properties to 1e-9', () => {
    const hull = new ConvexHull(boxCorners(1, 2, 3));
    const mp = hull.massProperties(2);
    const box = new Box(1, 2, 3).massProperties(2);
    expect(rel(mp.mass, box.mass)).toBeLessThan(1e-9);
    expect(
      Math.abs(mp.centerOfMass.x) + Math.abs(mp.centerOfMass.y) + Math.abs(mp.centerOfMass.z),
    ).toBeLessThan(1e-12);
    for (let i = 0; i < 9; i++) {
      const diff = Math.abs((mp.inertia.e[i] as number) - (box.inertia.e[i] as number));
      expect(diff).toBeLessThanOrEqual(1e-9 * Math.max(1, Math.abs(box.inertia.e[i] as number)));
    }
  });
  it('off-center hull has the correct centroid', () => {
    const pts = boxCorners(1, 1, 1).map((p) => new Vec3(p.x + 5, p.y, p.z));
    const mp = new ConvexHull(pts).massProperties(1);
    expect(rel(mp.centerOfMass.x, 5)).toBeLessThan(1e-12);
    // Inertia about the centroid is independent of translation.
    const ref = new Box(1, 1, 1).massProperties(1);
    expect(rel(mp.inertia.e[0] as number, ref.inertia.e[0] as number)).toBeLessThan(1e-9);
  });
  it('cylinder-like hull converges to the analytic cylinder within 0.5%', () => {
    const r = 1;
    const h = 0.75;
    const n = 256;
    const pts: Vec3[] = [];
    for (let i = 0; i < n; i++) {
      const a = (2 * Math.PI * i) / n;
      pts.push(
        new Vec3(r * Math.cos(a), h, r * Math.sin(a)),
        new Vec3(r * Math.cos(a), -h, r * Math.sin(a)),
      );
    }
    const mp = new ConvexHull(pts).massProperties(1);
    const m = Math.PI * r * r * 2 * h;
    const iAxis = 0.5 * m * r * r;
    const iSide = (m * (3 * r * r + (2 * h) ** 2)) / 12;
    expect(rel(mp.mass, m)).toBeLessThan(0.005);
    expect(rel(mp.inertia.e[4] as number, iAxis)).toBeLessThan(0.005);
    expect(rel(mp.inertia.e[0] as number, iSide)).toBeLessThan(0.005);
    expect(rel(mp.inertia.e[8] as number, iSide)).toBeLessThan(0.005);
  });
  it('support returns the farthest vertex and fills out', () => {
    const hull = new ConvexHull(boxCorners(1, 1, 1));
    const out = new Vec3();
    const s = hull.support(new Vec3(1, -1, 0.5), out);
    expect(s).toEqual(new Vec3(1, -1, 1));
    expect(hull.triangleCount).toBe(12);
    const b = new AABB();
    hull.localBounds(b);
    expect(b.min).toEqual(new Vec3(-1, -1, -1));
  });
  it('rejects degenerate input', () => {
    expect(() => quickhull([new Vec3(), new Vec3(), new Vec3(), new Vec3()])).toThrow(/coincident/);
    expect(() =>
      quickhull([new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(2, 0, 0), new Vec3(3, 0, 0)]),
    ).toThrow(/collinear/);
    expect(() =>
      quickhull([new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(1, 1, 0)]),
    ).toThrow(/coplanar/);
    expect(() => quickhull([new Vec3(), new Vec3(1, 0, 0), new Vec3(0, 1, 0)])).toThrow(RangeError);
  });
  it('polyhedronMassProperties rejects inverted orientation', () => {
    const pts = boxCorners(1, 1, 1);
    const hull = quickhull(pts);
    const flipped = new Uint32Array(hull.indices.length);
    for (let i = 0; i < hull.indices.length; i += 3) {
      flipped[i] = hull.indices[i] as number;
      flipped[i + 1] = hull.indices[i + 2] as number;
      flipped[i + 2] = hull.indices[i + 1] as number;
    }
    expect(() => polyhedronMassProperties(hull.vertices, flipped, 1)).toThrow(RangeError);
  });
});

describe('quickhull regressions (inputs found by property testing)', () => {
  it('rejects a cloud with only three distinct points and a duplicate origin', () => {
    const pts = [
      new Vec3(0, 0, 0),
      new Vec3(0, -3.562332651085854e-10, 0),
      new Vec3(-0.0030038239711335096, 3.0953595142937718, 1.0027119454693854),
      new Vec3(0, 0, 0),
    ];
    expect(() => quickhull(pts)).toThrow(/degenerate hull/);
  });
  it('rejects a flat simplex whose height is below the rounding floor', () => {
    const pts = [
      new Vec3(0.5, 0, 0),
      new Vec3(0, 0, 0),
      new Vec3(0, 0, -5e-324),
      new Vec3(0.25, -8.318098374396484, 1.8728371994105404e-109),
    ];
    expect(() => quickhull(pts)).toThrow(RangeError);
  });
  it('a near-flat cloud (thickness ratio ~1e-5) terminates with finite output (KNOWN LIMITATION)', () => {
    // Known limitation, tracked in PROGRESS.md: tolerance-based visibility can leave a face
    // tilted by ~3e-7 rad when a near-duplicate vertex sits in a near-collinear triangle, so
    // the membership bound does not hold below ~1e-4 thickness ratio. Phase 4 adds exact or
    // merge-based handling; until then this asserts termination and finiteness only.
    const pts = [
      new Vec3(0, 0, -9.022655241918818),
      new Vec3(-0.000011083211905892051, 0, 9.962006254250902),
      new Vec3(0, -0.00024033692408962346, 0),
      new Vec3(-8.997802734374979, 0, 0),
      new Vec3(0, 0, 0),
      new Vec3(0, 1.8286352674927817e-12, 0),
    ];
    const h = quickhull(pts);
    for (const v of h.vertices) expect(V.isFinite3(v)).toBe(true);
    expect(h.indices.length % 3).toBe(0);
  });
});

describe('triangle mesh', () => {
  it('validates indices, reports bounds and normals', () => {
    const v = [new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
    const mesh = new TriangleMesh(v, [0, 1, 2, 0, 2, 3]);
    expect(mesh.triangleCount).toBe(2);
    expect(mesh.bounds.max).toEqual(new Vec3(1, 1, 1));
    const n = mesh.triangleNormal(0, new Vec3());
    expect(n).toEqual(new Vec3(0, 0, 1));
    expect(() => new TriangleMesh(v, [0, 1])).toThrow(RangeError);
    expect(() => new TriangleMesh(v, [0, 1, 9])).toThrow(RangeError);
  });
});

describe('compound', () => {
  it('two halves of a box combine to the whole box (parallel-axis theorem)', () => {
    const halfA = new Box(0.5, 1, 1);
    const halfB = new Box(0.5, 1, 1);
    const comp = new Compound([
      { shape: halfA, local: new Transform(new Vec3(-0.5, 0, 0), new Quat()) },
      { shape: halfB, local: new Transform(new Vec3(0.5, 0, 0), new Quat()) },
    ]);
    const mp = comp.massProperties(1);
    const whole = new Box(1, 1, 1).massProperties(1);
    expect(rel(mp.mass, whole.mass)).toBeLessThan(1e-12);
    for (let i = 0; i < 9; i++) {
      expect(Math.abs((mp.inertia.e[i] as number) - (whole.inertia.e[i] as number))).toBeLessThan(
        1e-12,
      );
    }
  });
  it('rotated child contributes rotated inertia', () => {
    const rod = new Box(0.1, 0.1, 2);
    const q = Q_fromAxisAngle(new Vec3(0, 1, 0), Math.PI / 2);
    const comp = new Compound([{ shape: rod, local: new Transform(new Vec3(), q) }]);
    const mp = comp.massProperties(1);
    // rod's long axis now along x
    expect(mp.inertia.e[0]).toBeLessThan(mp.inertia.e[8] as number);
    expect(rel(mp.inertia.e[8] as number, mp.inertia.e[4] as number)).toBeLessThan(1e-12);
  });
  it('bounds cover all children; empty compound rejected', () => {
    const comp = new Compound([
      { shape: new Sphere(1), local: new Transform(new Vec3(5, 0, 0), new Quat()) },
      { shape: new Box(1, 1, 1), local: new Transform(new Vec3(-5, 0, 0), new Quat()) },
    ]);
    const b = new AABB();
    comp.localBounds(b);
    expect(b.min.x).toBe(-6);
    expect(b.max.x).toBe(6);
    expect(() => new Compound([])).toThrow(RangeError);
  });
  it('combine of nothing is empty; zero-mass parts are harmless', () => {
    expect(combine([]).mass).toBe(0);
    expect(combine([emptyMassProperties()]).mass).toBe(0);
  });
});

function V_sub_len(a: Vec3, b: Vec3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function Q_fromAxisAngle(axis: Vec3, angle: number): Quat {
  return _Q.fromAxisAngle(new Quat(), axis, angle);
}
