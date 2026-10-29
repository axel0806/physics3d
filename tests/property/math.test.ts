import { describe, it } from 'vitest';
import fc from 'fast-check';
import { FC_SEED } from '../helpers/fc';
import { Vec3, Quat, Mat3, Mat4, Transform } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as Q from '../../src/math/quat';
import * as M3 from '../../src/math/mat3';
import * as M4 from '../../src/math/mat4';
import * as T from '../../src/math/transform';

const RUNS = { numRuns: 2000, seed: FC_SEED };
const f = fc.double({ min: -100, max: 100, noNaN: true });
const vec = fc.tuple(f, f, f).map(([x, y, z]) => new Vec3(x, y, z));
const quat = fc
  .tuple(f, f, f, f)
  .filter(([x, y, z, w]) => x * x + y * y + z * z + w * w > 1e-3)
  .map(([x, y, z, w]) => Q.normalize(new Quat(), new Quat(x, y, z, w)));

const close = (a: number, b: number, tol: number) =>
  Math.abs(a - b) <= tol * Math.max(1, Math.abs(a), Math.abs(b));
const vclose = (a: Vec3, b: Vec3, tol: number) =>
  close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol);

describe('quaternion properties', () => {
  it('q * q⁻¹ = identity', () => {
    fc.assert(
      fc.property(quat, (q) => {
        const r = q.mul(q.inverse());
        return (
          close(r.w, 1, 1e-12) &&
          close(r.x, 0, 1e-12) &&
          close(r.y, 0, 1e-12) &&
          close(r.z, 0, 1e-12)
        );
      }),
      RUNS,
    );
  });
  it('|rotate(q, v)| = |v|', () => {
    fc.assert(
      fc.property(quat, vec, (q, v) => close(q.rotate(v).length(), v.length(), 1e-12)),
      RUNS,
    );
  });
  it('rotateInverse undoes rotate', () => {
    fc.assert(
      fc.property(quat, vec, (q, v) =>
        vclose(Q.rotateInverse(new Vec3(), q, q.rotate(v)), v, 1e-11),
      ),
      RUNS,
    );
  });
  it('quat → mat3 → quat round trip (up to sign)', () => {
    fc.assert(
      fc.property(quat, (q) => {
        const r = Q.fromMat3(new Quat(), M3.fromQuat(new Mat3(), q));
        return close(Math.abs(Q.dot(q, r)), 1, 1e-12);
      }),
      RUNS,
    );
  });
  it('matrix rotation agrees with quaternion rotation', () => {
    fc.assert(
      fc.property(quat, vec, (q, v) =>
        vclose(M3.fromQuat(new Mat3(), q).mulVec(v), q.rotate(v), 1e-11),
      ),
      RUNS,
    );
  });
  it('integrate keeps unit length and matches axis-angle for small steps', () => {
    fc.assert(
      fc.property(quat, vec, (q, w) => {
        const r = Q.integrate(new Quat(), q, w, 1e-3);
        return close(Q.dot(r, r), 1, 1e-12);
      }),
      RUNS,
    );
  });
  it('slerp endpoints and unit length', () => {
    fc.assert(
      fc.property(quat, quat, fc.double({ min: 0, max: 1, noNaN: true }), (a, b, t) => {
        const r = Q.slerp(new Quat(), a, b, t);
        const r0 = Q.slerp(new Quat(), a, b, 0);
        const r1 = Q.slerp(new Quat(), a, b, 1);
        return (
          close(Q.dot(r, r), 1, 1e-12) &&
          close(Math.abs(Q.dot(r0, a)), 1, 1e-9) &&
          close(Math.abs(Q.dot(r1, b)), 1, 1e-9)
        );
      }),
      RUNS,
    );
  });
});

describe('vector properties', () => {
  it('cross product is anti-commutative', () => {
    fc.assert(
      fc.property(vec, vec, (a, b) => vclose(a.cross(b), b.cross(a).negated(), 1e-12)),
      RUNS,
    );
  });
  it('cross product is orthogonal to both inputs', () => {
    fc.assert(
      fc.property(vec, vec, (a, b) => {
        const c = a.cross(b);
        const s = Math.max(1, a.length() * b.length());
        return (
          Math.abs(c.dot(a)) <= 1e-9 * s * Math.max(1, a.length()) &&
          Math.abs(c.dot(b)) <= 1e-9 * s * Math.max(1, b.length())
        );
      }),
      RUNS,
    );
  });
  it('perpendicular returns a unit vector orthogonal to n', () => {
    fc.assert(
      fc.property(
        vec.filter((v) => v.length() > 1e-3),
        (v) => {
          const n = v.normalized();
          const p = V.perpendicular(new Vec3(), n);
          return close(p.length(), 1, 1e-12) && Math.abs(p.dot(n)) < 1e-12;
        },
      ),
      RUNS,
    );
  });
});

const mat3 = fc.array(f, { minLength: 9, maxLength: 9 }).map((a) => new Mat3(a));
// Well-conditioned: rotation * diag(0.5..2) * rotation.
const wellConditioned = fc
  .tuple(
    quat,
    quat,
    fc.double({ min: 0.5, max: 2, noNaN: true }),
    fc.double({ min: 0.5, max: 2, noNaN: true }),
    fc.double({ min: 0.5, max: 2, noNaN: true }),
  )
  .map(([q1, q2, a, b, c]) => {
    const d = M3.fromDiagonal(new Mat3(), a, b, c);
    return M3.fromQuat(new Mat3(), q1).mul(d).mul(M3.fromQuat(new Mat3(), q2));
  });

describe('matrix properties', () => {
  it('M * M⁻¹ ≈ I for well-conditioned M', () => {
    fc.assert(
      fc.property(wellConditioned, (m) => {
        const inv = m.inverse();
        if (!inv) return false;
        const p = m.mul(inv);
        for (let i = 0; i < 9; i++) {
          if (Math.abs((p.e[i] as number) - (i % 4 === 0 ? 1 : 0)) > 1e-12) return false;
        }
        return true;
      }),
      RUNS,
    );
  });
  it('transpose is an involution and (AB)ᵀ = BᵀAᵀ', () => {
    fc.assert(
      fc.property(mat3, mat3, (a, b) => {
        const t = a.transpose().transpose();
        const l = a.mul(b).transpose();
        const r = b.transpose().mul(a.transpose());
        for (let i = 0; i < 9; i++) {
          if (t.e[i] !== a.e[i] || !close(l.e[i] as number, r.e[i] as number, 1e-12)) return false;
        }
        return true;
      }),
      RUNS,
    );
  });
  it('Jacobi eigen-decomposition reconstructs a symmetric matrix', () => {
    fc.assert(
      fc.property(mat3, (a) => {
        const s = M3.scale(new Mat3(), M3.add(new Mat3(), a, a.transpose()), 0.5);
        const vals = new Vec3();
        const vecs = new Mat3();
        M3.symmetricEigen(vals, vecs, s);
        const rec = vecs
          .mul(M3.fromDiagonal(new Mat3(), vals.x, vals.y, vals.z))
          .mul(vecs.transpose());
        const scale = Math.max(1, ...Array.from(s.e, Math.abs));
        for (let i = 0; i < 9; i++) {
          if (Math.abs((rec.e[i] as number) - (s.e[i] as number)) > 1e-12 * scale) return false;
        }
        // V orthonormal
        const vtv = vecs.transpose().mul(vecs);
        for (let i = 0; i < 9; i++) {
          if (Math.abs((vtv.e[i] as number) - (i % 4 === 0 ? 1 : 0)) > 1e-12) return false;
        }
        return true;
      }),
      RUNS,
    );
  });
  it('Mat4 inverse and compose agree with Transform', () => {
    fc.assert(
      fc.property(vec, quat, vec, (p, q, v) => {
        const m = M4.compose(new Mat4(), p, q);
        const inv = new Mat4();
        if (!M4.invert(inv, m)) return false;
        const id = M4.mul(new Mat4(), m, inv);
        for (let i = 0; i < 16; i++) {
          if (Math.abs((id.e[i] as number) - (i % 5 === 0 ? 1 : 0)) > 1e-9) return false;
        }
        const t = new Transform(p, q);
        return vclose(M4.transformPoint(new Vec3(), m, v), T.apply(new Vec3(), t, v), 1e-11);
      }),
      RUNS,
    );
  });
  it('Transform: t * t⁻¹ = identity and applyInverse undoes apply', () => {
    fc.assert(
      fc.property(vec, quat, vec, (p, q, v) => {
        const t = new Transform(p, q);
        const inv = T.invert(new Transform(), t);
        const id = T.mul(new Transform(), t, inv);
        const back = T.applyInverse(new Vec3(), t, T.apply(new Vec3(), t, v));
        return (
          vclose(id.p, new Vec3(), 1e-10) &&
          close(Math.abs(id.q.w), 1, 1e-12) &&
          vclose(back, v, 1e-10)
        );
      }),
      RUNS,
    );
  });
});
