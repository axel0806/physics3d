import { describe, expect, it } from 'vitest';
import * as M from '../../src/math';
import { Vec3, Quat, Mat3, Mat4, Transform, AABB } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as Q from '../../src/math/quat';
import * as M3 from '../../src/math/mat3';
import * as M4 from '../../src/math/mat4';
import * as T from '../../src/math/transform';
import * as B from '../../src/math/aabb';

const near = (a: number, b: number, t = 1e-12) => expect(Math.abs(a - b)).toBeLessThan(t);

describe('util', () => {
  it('works', () => {
    expect(M.clamp(5, 0, 1)).toBe(1);
    expect(M.clamp(-5, 0, 1)).toBe(0);
    expect(M.clamp(0.5, 0, 1)).toBe(0.5);
    expect(M.lerp(0, 10, 0.25)).toBe(2.5);
    expect(M.sign(-2)).toBe(-1);
    expect(M.sign(2)).toBe(1);
    expect(M.approxEqual(1, 1 + 1e-12)).toBe(true);
    expect(M.approxEqual(1, 2)).toBe(false);
    expect(M.RAD2DEG * M.DEG2RAD).toBeCloseTo(1);
  });
});

describe('Vec3', () => {
  it('basic ops, allocating and out-param', () => {
    const a = new Vec3(1, 2, 3);
    const b = new Vec3(4, 5, 6);
    expect(a.clone()).toEqual(a);
    expect(a.add(b)).toEqual(new Vec3(5, 7, 9));
    expect(b.sub(a)).toEqual(new Vec3(3, 3, 3));
    expect(a.scale(2)).toEqual(new Vec3(2, 4, 6));
    expect(a.cross(b)).toEqual(new Vec3(-3, 6, -3));
    expect(a.dot(b)).toBe(32);
    near(new Vec3(3, 4, 0).length(), 5);
    near(new Vec3(0, 0, 9).normalized().z, 1);
    expect(a.negated()).toEqual(new Vec3(-1, -2, -3));
    const o = new Vec3();
    V.set(o, 1, 2, 3);
    V.copy(o, b);
    expect(o).toEqual(b);
    V.addScaled(o, a, b, 2);
    expect(o).toEqual(new Vec3(9, 12, 15));
    V.lerp(o, a, b, 0.5);
    expect(o).toEqual(new Vec3(2.5, 3.5, 4.5));
    V.min(o, a, new Vec3(0, 9, 0));
    expect(o).toEqual(new Vec3(0, 2, 0));
    V.max(o, a, new Vec3(0, 9, 0));
    expect(o).toEqual(new Vec3(1, 9, 3));
    V.abs(o, new Vec3(-1, 2, -3));
    expect(o).toEqual(new Vec3(1, 2, 3));
    expect(V.lengthSq(a)).toBe(14);
    expect(V.distanceSq(a, b)).toBe(27);
    near(V.distance(a, a), 0);
    expect(V.equals(a, a.clone())).toBe(true);
    expect(V.equals(a, b)).toBe(false);
    expect(V.isFinite3(a)).toBe(true);
    expect(V.isFinite3(new Vec3(NaN, 0, 0))).toBe(false);
    expect(V.isFinite3(new Vec3(0, Infinity, 0))).toBe(false);
    expect(V.isFinite3(new Vec3(0, 0, NaN))).toBe(false);
  });
  it('cross is alias-safe; normalize of zero is zero', () => {
    const a = new Vec3(1, 0, 0);
    V.cross(a, a, new Vec3(0, 1, 0));
    expect(a).toEqual(new Vec3(0, 0, 1));
    expect(V.normalize(new Vec3(1, 1, 1), new Vec3())).toEqual(new Vec3());
  });
  it('perpendicular covers both branches', () => {
    const o = new Vec3();
    V.perpendicular(o, new Vec3(1, 0, 0));
    near(o.dot(new Vec3(1, 0, 0)), 0);
    V.perpendicular(o, new Vec3(0, 1, 0));
    near(o.dot(new Vec3(0, 1, 0)), 0);
    near(o.length(), 1);
  });
});

describe('Quat', () => {
  const axisAngle = (x: number, y: number, z: number, a: number) =>
    Q.fromAxisAngle(new Quat(), new Vec3(x, y, z), a);
  it('basics', () => {
    const q = new Quat();
    expect(q.clone()).toEqual(q);
    Q.set(q, 1, 2, 3, 4);
    const c = Q.copy(new Quat(), q);
    expect(c).toEqual(q);
    expect(Q.identity(c)).toEqual(new Quat());
    expect(Q.conjugate(new Quat(), q)).toEqual(new Quat(-1, -2, -3, 4));
    expect(q.normalized().mul(new Quat()).w).toBeCloseTo(4 / Math.sqrt(30));
    expect(Q.normalize(new Quat(5, 5, 5, 5), new Quat(0, 0, 0, 0))).toEqual(new Quat());
    expect(Q.invert(new Quat(), new Quat(0, 0, 0, 0))).toEqual(new Quat(-0, -0, -0, 0));
  });
  it('rotates 90° about z', () => {
    const q = axisAngle(0, 0, 1, Math.PI / 2);
    const r = q.rotate(new Vec3(1, 0, 0));
    near(r.x, 0);
    near(r.y, 1);
    const b = Q.rotateInverse(new Vec3(), q, r);
    near(b.x, 1);
  });
  it('integrate matches exact rotation for small steps', () => {
    let q = new Quat();
    const w = new Vec3(0, 0, 2);
    for (let i = 0; i < 1000; i++) Q.integrate(q, q, w, 1e-3);
    const exact = axisAngle(0, 0, 1, 2);
    near(Math.abs(Q.dot(q, exact)), 1, 1e-6);
    q = Q.integrate(new Quat(), new Quat(), new Vec3(), 0.1);
    expect(q).toEqual(new Quat());
  });
  it('slerp: midpoint, shortest arc, near-parallel', () => {
    const a = new Quat();
    const b = axisAngle(0, 0, 1, Math.PI / 2);
    const m = Q.slerp(new Quat(), a, b, 0.5);
    near(Q.dot(m, axisAngle(0, 0, 1, Math.PI / 4)), 1);
    const nb = new Quat(-b.x, -b.y, -b.z, -b.w);
    const m2 = Q.slerp(new Quat(), a, nb, 0.5);
    near(Math.abs(Q.dot(m2, m)), 1);
    const close = axisAngle(0, 0, 1, 1e-4);
    const m3 = Q.slerp(new Quat(), a, close, 0.5);
    near(Q.dot(m3, m3), 1);
  });
  it('fromMat3 covers all four branches', () => {
    const cases: Quat[] = [
      axisAngle(0, 0, 1, 0.3), // trace > 0
      axisAngle(1, 0, 0, Math.PI * 0.9), // m00 largest
      axisAngle(0, 1, 0, Math.PI * 0.9), // m11 largest
      axisAngle(0, 0, 1, Math.PI * 0.9), // m22 largest
    ];
    for (const q of cases) {
      const r = Q.fromMat3(new Quat(), M3.fromQuat(new Mat3(), q));
      near(Math.abs(Q.dot(q, r)), 1);
    }
  });
  it('fromUnitVectors', () => {
    const cases: [Vec3, Vec3][] = [
      [new Vec3(1, 0, 0), new Vec3(0, 1, 0)],
      [new Vec3(1, 0, 0), new Vec3(-1, 0, 0)],
      [new Vec3(0, 1, 0), new Vec3(0, -1, 0)],
      [new Vec3(0, 0, 1), new Vec3(0, 0, -1)],
      [new Vec3(0, 0, 1), new Vec3(0, 0, 1)],
    ];
    for (const [a, b] of cases) {
      const r = Q.fromUnitVectors(new Quat(), a, b).rotate(a);
      near(r.x, b.x, 1e-9);
      near(r.y, b.y, 1e-9);
      near(r.z, b.z, 1e-9);
    }
  });
});

describe('Mat3', () => {
  it('construction, identity, copy, column ops', () => {
    const m = new Mat3();
    expect(Array.from(m.e)).toEqual([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    const n = new Mat3([1, 2, 3, 4, 5, 6, 7, 8, 9]);
    expect(n.clone().e).toEqual(n.e);
    M3.copy(m, n);
    expect(m.e).toEqual(n.e);
    M3.identity(m);
    M3.set(m, [9, 8, 7, 6, 5, 4, 3, 2, 1]);
    expect(m.e[0]).toBe(9);
    expect(M3.getColumn(new Vec3(), n, 1)).toEqual(new Vec3(2, 5, 8));
    const fc = M3.fromColumns(new Mat3(), new Vec3(1, 4, 7), new Vec3(2, 5, 8), new Vec3(3, 6, 9));
    expect(fc.e).toEqual(n.e);
    expect(M3.trace(n)).toBe(15);
    expect(M3.determinant(new Mat3([2, 0, 0, 0, 3, 0, 0, 0, 4]))).toBe(24);
  });
  it('skew implements the cross product', () => {
    const a = new Vec3(1, 2, 3);
    const b = new Vec3(-4, 5, 6);
    const r = M3.skew(new Mat3(), a).mulVec(b);
    expect(r).toEqual(a.cross(b));
  });
  it('mulVecTransposed, transpose in place, inverse', () => {
    const m = new Mat3([1, 2, 3, 4, 5, 6, 7, 8, 10]);
    const v = new Vec3(1, -1, 2);
    expect(M3.mulVecTransposed(new Vec3(), m, v)).toEqual(m.transpose().mulVec(v));
    const t = m.clone();
    M3.transpose(t, t);
    expect(t.e).toEqual(m.transpose().e);
    const inv = m.inverse() as Mat3;
    const id = m.mul(inv);
    near(id.e[0] as number, 1, 1e-12);
    near(id.e[1] as number, 0, 1e-12);
    const alias = m.clone();
    expect(M3.invert(alias, alias)).toBe(true);
    near(alias.e[0] as number, inv.e[0] as number);
    expect(new Mat3([1, 2, 3, 2, 4, 6, 1, 1, 1]).inverse()).toBeNull();
    expect(M3.invert(new Mat3(), new Mat3([Infinity, 0, 0, 0, 1, 0, 0, 0, 1]))).toBe(false);
  });
  it('eigen: diagonal, repeated, and degenerate inputs', () => {
    const vals = new Vec3();
    const vecs = new Mat3();
    M3.symmetricEigen(vals, vecs, M3.fromDiagonal(new Mat3(), 3, 1, 2));
    expect([vals.x, vals.y, vals.z]).toEqual([3, 1, 2]);
    M3.symmetricEigen(vals, vecs, M3.fromDiagonal(new Mat3(), 5, 5, 5));
    expect(vals.x).toBe(5);
    M3.symmetricEigen(vals, vecs, new Mat3(new Array(9).fill(0)));
    expect(vals.x).toBe(0);
    // Dense symmetric with known eigenvalues 1, 2, 4 (rotated).
    const r = M3.fromQuat(new Mat3(), Q.normalize(new Quat(), new Quat(0.3, -0.5, 0.2, 0.8)));
    const a = r.mul(M3.fromDiagonal(new Mat3(), 1, 2, 4)).mul(r.transpose());
    M3.symmetricEigen(vals, vecs, a);
    expect([vals.x, vals.y, vals.z].sort()).toEqual(
      expect.arrayContaining([expect.closeTo(1, 10), expect.closeTo(2, 10), expect.closeTo(4, 10)]),
    );
  });
  it('add/scale', () => {
    const a = new Mat3();
    expect(M3.add(new Mat3(), a, a).e[0]).toBe(2);
    expect(M3.scale(new Mat3(), a, 3).e[4]).toBe(3);
  });
});

describe('Mat4', () => {
  it('identity/copy/translation/scale/mul', () => {
    const m = new Mat4();
    expect(m.clone().e).toEqual(m.e);
    const t = M4.fromTranslation(new Mat4(), 1, 2, 3);
    const s = M4.fromScale(new Mat4(), 2, 2, 2);
    const ts = M4.mul(new Mat4(), t, s);
    expect(M4.transformPoint(new Vec3(), ts, new Vec3(1, 1, 1))).toEqual(new Vec3(3, 4, 5));
    M4.mul(t, t, s); // aliasing
    expect(t.e[12]).toBe(1);
    expect(M4.copy(new Mat4(), s).e).toEqual(s.e);
    expect(M4.identity(s).e[0]).toBe(1);
  });
  it('transpose, invert singular, compose with scale', () => {
    const m = M4.compose(new Mat4(), new Vec3(1, 2, 3), new Quat(), 2, 3, 4);
    const t = M4.transpose(new Mat4(), m);
    expect(t.e[3]).toBe(1);
    expect(M4.invert(new Mat4(), new Mat4(new Array(16).fill(0)))).toBe(false);
    const inv = new Mat4();
    expect(M4.invert(inv, m)).toBe(true);
    expect(M4.transformPoint(new Vec3(), inv, new Vec3(3, 5, 7)).x).toBeCloseTo(1);
  });
  it('perspective/ortho/lookAt', () => {
    const p = M4.perspective(new Mat4(), Math.PI / 2, 2, 1, 100);
    expect(p.e[5]).toBeCloseTo(1);
    expect(p.e[0]).toBeCloseTo(0.5);
    expect(p.e[11]).toBe(-1);
    const o = M4.ortho(new Mat4(), -1, 1, -1, 1, 0, 2);
    expect(o.e[10]).toBe(-1);
    const v = M4.lookAt(new Mat4(), new Vec3(0, 0, 5), new Vec3(), new Vec3(0, 1, 0));
    expect(M4.transformPoint(new Vec3(), v, new Vec3()).z).toBeCloseTo(-5);
  });
  it('toFloat32', () => {
    const out = new Float32Array(20);
    M4.toFloat32(out, M4.fromTranslation(new Mat4(), 7, 8, 9), 4);
    expect(out[16]).toBe(7);
    expect(out[19]).toBe(1);
    M4.toFloat32(out, new Mat4());
    expect(out[0]).toBe(1);
  });
});

describe('Transform', () => {
  it('identity/copy/apply/inverse/mul/mulInverse', () => {
    const t = new Transform(
      new Vec3(1, 2, 3),
      Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), Math.PI / 2),
    );
    const c = t.clone();
    expect(T.copy(new Transform(), t)).toEqual(c);
    const p = T.apply(new Vec3(), t, new Vec3(1, 0, 0));
    near(p.x, 1);
    near(p.y, 3);
    const d = T.applyDirection(new Vec3(), t, new Vec3(1, 0, 0));
    near(d.y, 1);
    near(T.applyInverseDirection(new Vec3(), t, d).x, 1);
    near(T.applyInverse(new Vec3(), t, p).x, 1);
    const inv = T.invert(new Transform(), t);
    const id = T.mul(new Transform(), t, inv);
    near(id.p.x, 0);
    const a = new Transform(new Vec3(5, 0, 0), new Quat());
    const rel = T.mulInverse(new Transform(), a, t);
    near(rel.p.x, -4);
    T.invert(c, c);
    near(c.p.x, inv.p.x);
    T.mul(c, c, t);
    near(c.p.x, 0);
    T.identity(c);
    expect(c.q.w).toBe(1);
  });
});

describe('AABB', () => {
  const box = (a: number, b: number, c: number, d: number, e: number, f: number) =>
    B.set(new AABB(), a, b, c, d, e, f);
  it('queries', () => {
    const a = box(0, 0, 0, 1, 2, 3);
    const e = B.empty(new AABB());
    expect(B.isValid(e)).toBe(false);
    expect(B.isValid(a)).toBe(true);
    expect(B.copy(new AABB(), a)).toEqual(a);
    expect(a.clone()).toEqual(a);
    const u = B.union(new AABB(), a, box(-1, 0, 0, 0.5, 5, 1));
    expect(u.min.x).toBe(-1);
    expect(u.max.y).toBe(5);
    expect(B.surfaceArea(a)).toBe(2 * (2 + 6 + 3));
    expect(B.volume(a)).toBe(6);
    expect(B.overlaps(a, box(1, 0, 0, 2, 1, 1))).toBe(true);
    expect(B.overlaps(a, box(1.1, 0, 0, 2, 1, 1))).toBe(false);
    expect(B.overlaps(a, box(0, 2.1, 0, 1, 3, 1))).toBe(false);
    expect(B.overlaps(a, box(0, 0, 3.1, 1, 1, 4))).toBe(false);
    expect(B.overlaps(a, box(-3, 0, 0, -1, 1, 1))).toBe(false);
    expect(B.overlaps(a, box(0, -3, 0, 1, -1, 1))).toBe(false);
    expect(B.overlaps(a, box(0, 0, -3, 1, 1, -1))).toBe(false);
    expect(B.contains(u, a)).toBe(true);
    expect(B.contains(a, u)).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(-1, 0, 0, 1, 1, 1))).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(0, -1, 0, 1, 1, 1))).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(0, 0, -1, 1, 1, 1))).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(0, 0, 0, 2, 1, 1))).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(0, 0, 0, 1, 2, 1))).toBe(false);
    expect(B.contains(box(0, 0, 0, 1, 1, 1), box(0, 0, 0, 1, 1, 2))).toBe(false);
    expect(B.containsPoint(a, new Vec3(0.5, 1, 1))).toBe(true);
    expect(B.containsPoint(a, new Vec3(-1, 1, 1))).toBe(false);
    expect(B.containsPoint(a, new Vec3(0.5, -1, 1))).toBe(false);
    expect(B.containsPoint(a, new Vec3(0.5, 1, 9))).toBe(false);
    expect(B.center(new Vec3(), a)).toEqual(new Vec3(0.5, 1, 1.5));
    expect(B.halfExtents(new Vec3(), a)).toEqual(new Vec3(0.5, 1, 1.5));
    const f = B.fatten(new AABB(), a, 1);
    expect(f.min.x).toBe(-1);
    expect(f.max.z).toBe(4);
    const g = B.empty(new AABB());
    B.expandPoint(g, new Vec3(1, 2, 3));
    B.expandPoint(g, new Vec3(-1, 5, 0));
    expect(g.min).toEqual(new Vec3(-1, 2, 0));
    expect(g.max).toEqual(new Vec3(1, 5, 3));
  });
  it('fromOrientedBox is tight for axis-aligned and 45° cases', () => {
    const q = Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), Math.PI / 4);
    const b = B.fromOrientedBox(new AABB(), new Vec3(1, 1, 1), new Vec3(1, 1, 1), q);
    near(b.max.x, 1 + Math.SQRT2);
    near(b.max.z, 2);
    const id = B.fromOrientedBox(new AABB(), new Vec3(), new Vec3(1, 2, 3), new Quat());
    expect(id.max).toEqual(new Vec3(1, 2, 3));
  });
  it('raycast: hit, miss, inside, parallel, reversed', () => {
    const a = box(-1, -1, -1, 1, 1, 1);
    near(B.raycast(a, -5, 0, 0, 1, 0, 0, 100), 4);
    near(B.raycast(a, 5, 0, 0, -1, 0, 0, 100), 4);
    expect(B.raycast(a, -5, 0, 0, 1, 0, 0, 3)).toBe(-1);
    expect(B.raycast(a, 0, 0, 0, 1, 0, 0, 100)).toBe(0);
    expect(B.raycast(a, -5, 3, 0, 1, 0, 0, 100)).toBe(-1); // parallel outside (y)
    expect(B.raycast(a, -5, 0, 3, 1, 0, 0, 100)).toBe(-1); // parallel outside (z)
    expect(B.raycast(a, -5, 0, 0, -1, 0, 0, 100)).toBe(-1); // pointing away
    expect(B.raycast(a, -5, 5, 0, 1, -0.5, 0, 100)).toBe(-1); // misses via y slab
    expect(B.raycast(a, -5, 0, 5, 1, 0, -0.5, 100)).toBe(-1); // misses via z slab
  });
});
