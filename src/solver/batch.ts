import type { RigidBody } from '../dynamics/body';
import type { ContactConstraint } from './contactSolver';

/**
 * Struct-of-arrays copy of the contact constraints for the velocity iterations. The object
 * solver (`solveVelocity`) chases a pointer for every body, vector and point it touches; here the
 * hot loop reads only flat Float64Arrays, which is several times faster. Arithmetic matches the
 * object solver, so results agree to rounding.
 *
 * Usage per step: `gather` (after `prepareContact`/`buildNormalK`), `warmStart`, `iterate` any
 * number of times, then `scatter` to write velocities and accumulated impulses back.
 *
 * Body slot 0 is reserved for "no body" and never used; bodies that do not move (static,
 * sleeping) have zero inverse mass and inertia, so applying impulses to them changes nothing.
 */
export class ContactBatch {
  // Bodies: linear and angular velocity (3 each), inverse mass, inverse inertia (9).
  private lin = new Float64Array(3 * 64);
  private ang = new Float64Array(3 * 64);
  private invMass = new Float64Array(64);
  private invInertia = new Float64Array(9 * 64);
  private bodyCount = 0;
  private bodies: RigidBody[] = [];
  private stamp = 0;

  // Constraints.
  private cBodyA = new Int32Array(64);
  private cBodyB = new Int32Array(64);
  private cNormal = new Float64Array(3 * 64);
  private cTan1 = new Float64Array(3 * 64);
  private cTan2 = new Float64Array(3 * 64);
  private cFriction = new Float64Array(64);
  private cPointStart = new Int32Array(64);
  private cPointCount = new Int32Array(64);
  private cKStart = new Int32Array(64);
  private K = new Float64Array(16 * 64);
  private constraints: ContactConstraint[] = [];
  private constraintCount = 0;

  // Points.
  private pRA = new Float64Array(3 * 256);
  private pRB = new Float64Array(3 * 256);
  private pMass = new Float64Array(3 * 256); // normal, tangent 1, tangent 2
  private pImpulse = new Float64Array(3 * 256);
  private pTarget = new Float64Array(256);

  /** Largest speed change caused by an impulse update since the last `iterate` began. */
  maxChange = 0;

  private growBodies(n: number): void {
    if (n <= this.invMass.length) return;
    const cap = Math.max(n, this.invMass.length * 2);
    this.lin = grow(this.lin, 3 * cap);
    this.ang = grow(this.ang, 3 * cap);
    this.invMass = grow(this.invMass, cap);
    this.invInertia = grow(this.invInertia, 9 * cap);
  }

  private growConstraints(n: number, kn: number): void {
    if (n > this.cFriction.length) {
      const cap = Math.max(n, this.cFriction.length * 2);
      this.cBodyA = grow(this.cBodyA, cap);
      this.cBodyB = grow(this.cBodyB, cap);
      this.cNormal = grow(this.cNormal, 3 * cap);
      this.cTan1 = grow(this.cTan1, 3 * cap);
      this.cTan2 = grow(this.cTan2, 3 * cap);
      this.cFriction = grow(this.cFriction, cap);
      this.cPointStart = grow(this.cPointStart, cap);
      this.cPointCount = grow(this.cPointCount, cap);
      this.cKStart = grow(this.cKStart, cap);
    }
    if (kn > this.K.length) this.K = grow(this.K, Math.max(kn, this.K.length * 2));
  }

  private growPoints(n: number): void {
    if (n <= this.pTarget.length) return;
    const cap = Math.max(n, this.pTarget.length * 2);
    this.pRA = grow(this.pRA, 3 * cap);
    this.pRB = grow(this.pRB, 3 * cap);
    this.pMass = grow(this.pMass, 3 * cap);
    this.pImpulse = grow(this.pImpulse, 3 * cap);
    this.pTarget = grow(this.pTarget, cap);
  }

  private slot(b: RigidBody): number {
    if (b.batchStamp === this.stamp) return b.batchIndex;
    b.batchStamp = this.stamp;
    const i = this.bodyCount++;
    b.batchIndex = i;
    this.bodies[i] = b;
    this.growBodies(i + 1);
    const lv = b.linearVelocity;
    const av = b.angularVelocity;
    this.lin[3 * i] = lv.x;
    this.lin[3 * i + 1] = lv.y;
    this.lin[3 * i + 2] = lv.z;
    this.ang[3 * i] = av.x;
    this.ang[3 * i + 1] = av.y;
    this.ang[3 * i + 2] = av.z;
    if (b.isSimulated) {
      this.invMass[i] = b.invMass;
      this.invInertia.set(b.worldInvInertia.e, 9 * i);
    } else {
      this.invMass[i] = 0;
      this.invInertia.fill(0, 9 * i, 9 * i + 9);
    }
    return i;
  }

  /** Copies the (already prepared) constraints into the arrays. */
  gather(contacts: readonly ContactConstraint[]): void {
    this.stamp++;
    this.bodyCount = 1; // slot 0 unused
    this.constraints.length = 0;
    let points = 0;
    let kSize = 0;
    let active = 0;
    for (const c of contacts) {
      if (!(c.bodyA.isSimulated || c.bodyB.isSimulated)) continue;
      active++;
      points += c.points.length;
      kSize += c.points.length * c.points.length;
    }
    this.growConstraints(active, kSize);
    this.growPoints(points);
    let k = 0;
    let q = 0;
    let kOff = 0;
    for (const c of contacts) {
      if (!(c.bodyA.isSimulated || c.bodyB.isSimulated)) continue;
      this.constraints.push(c);
      this.cBodyA[k] = this.slot(c.bodyA);
      this.cBodyB[k] = this.slot(c.bodyB);
      this.cNormal.set([c.normal.x, c.normal.y, c.normal.z], 3 * k);
      this.cTan1.set([c.tangent1.x, c.tangent1.y, c.tangent1.z], 3 * k);
      this.cTan2.set([c.tangent2.x, c.tangent2.y, c.tangent2.z], 3 * k);
      this.cFriction[k] = c.friction;
      const n = c.points.length;
      this.cPointStart[k] = q;
      this.cPointCount[k] = n;
      this.cKStart[k] = kOff;
      if (c.normalK.length === n * n) this.K.set(c.normalK, kOff);
      kOff += n * n;
      for (const p of c.points) {
        this.pRA.set([p.rA.x, p.rA.y, p.rA.z], 3 * q);
        this.pRB.set([p.rB.x, p.rB.y, p.rB.z], 3 * q);
        this.pMass[3 * q] = p.normalMass;
        this.pMass[3 * q + 1] = p.tangentMass1;
        this.pMass[3 * q + 2] = p.tangentMass2;
        this.pImpulse[3 * q] = p.impulseN;
        this.pImpulse[3 * q + 1] = p.impulseT1;
        this.pImpulse[3 * q + 2] = p.impulseT2;
        this.pTarget[q] = p.restitutionTarget;
        q++;
      }
      k++;
    }
    this.constraintCount = k;
  }

  /** Re-applies the accumulated impulses from the previous step. */
  warmStart(): void {
    for (let k = 0; k < this.constraintCount; k++) {
      const a = this.cBodyA[k] as number;
      const b = this.cBodyB[k] as number;
      const nx = this.cNormal[3 * k] as number;
      const ny = this.cNormal[3 * k + 1] as number;
      const nz = this.cNormal[3 * k + 2] as number;
      const t1x = this.cTan1[3 * k] as number;
      const t1y = this.cTan1[3 * k + 1] as number;
      const t1z = this.cTan1[3 * k + 2] as number;
      const t2x = this.cTan2[3 * k] as number;
      const t2y = this.cTan2[3 * k + 1] as number;
      const t2z = this.cTan2[3 * k + 2] as number;
      const start = this.cPointStart[k] as number;
      const end = start + (this.cPointCount[k] as number);
      for (let q = start; q < end; q++) {
        const jn = this.pImpulse[3 * q] as number;
        const j1 = this.pImpulse[3 * q + 1] as number;
        const j2 = this.pImpulse[3 * q + 2] as number;
        this.applyPair(
          a,
          b,
          q,
          nx * jn + t1x * j1 + t2x * j2,
          ny * jn + t1y * j1 + t2y * j2,
          nz * jn + t1z * j1 + t2z * j2,
        );
      }
    }
  }

  /** One sequential-impulse pass over every constraint (normals as a block, then friction). */
  iterate(): void {
    this.maxChange = 0;
    for (let k = 0; k < this.constraintCount; k++) this.solveConstraint(k);
  }

  private solveConstraint(k: number): void {
    const a = this.cBodyA[k] as number;
    const b = this.cBodyB[k] as number;
    const nx = this.cNormal[3 * k] as number;
    const ny = this.cNormal[3 * k + 1] as number;
    const nz = this.cNormal[3 * k + 2] as number;
    const start = this.cPointStart[k] as number;
    const n = this.cPointCount[k] as number;
    const end = start + n;
    if (n > 1 && n <= BLOCK_MAX && this.solveBlock(k, a, b, nx, ny, nz, start, n)) {
      // Block solution applied.
    } else {
      for (let q = start; q < end; q++) {
        const vn = this.speedAlong(a, b, q, nx, ny, nz);
        const old = this.pImpulse[3 * q] as number;
        const mass = this.pMass[3 * q] as number;
        const next = Math.max(0, old + ((this.pTarget[q] as number) - vn) * mass);
        const d = next - old;
        this.pImpulse[3 * q] = next;
        if (mass > 0) this.track(Math.abs(d) / mass);
        this.applyPair(a, b, q, nx * d, ny * d, nz * d);
      }
    }
    const mu = this.cFriction[k] as number;
    const t1x = this.cTan1[3 * k] as number;
    const t1y = this.cTan1[3 * k + 1] as number;
    const t1z = this.cTan1[3 * k + 2] as number;
    const t2x = this.cTan2[3 * k] as number;
    const t2y = this.cTan2[3 * k + 1] as number;
    const t2z = this.cTan2[3 * k + 2] as number;
    const lin = this.lin;
    const ang = this.ang;
    const pI = this.pImpulse;
    const pM = this.pMass;
    const pRA = this.pRA;
    const pRB = this.pRB;
    for (let q = start; q < end; q++) {
      const acc1 = pI[3 * q + 1] as number;
      const acc2 = pI[3 * q + 2] as number;
      const maxF = mu * (pI[3 * q] as number);
      // No normal load and nothing accumulated: the clamped impulse is exactly zero.
      if (maxF === 0 && acc1 === 0 && acc2 === 0) continue;
      const m1 = pM[3 * q + 1] as number;
      const m2 = pM[3 * q + 2] as number;
      const rx = pRA[3 * q] as number;
      const ry = pRA[3 * q + 1] as number;
      const rz = pRA[3 * q + 2] as number;
      const sx = pRB[3 * q] as number;
      const sy = pRB[3 * q + 1] as number;
      const sz = pRB[3 * q + 2] as number;
      // Relative velocity of the point once, then both tangent speeds from it.
      const wax = ang[3 * a] as number;
      const way = ang[3 * a + 1] as number;
      const waz = ang[3 * a + 2] as number;
      const wbx = ang[3 * b] as number;
      const wby = ang[3 * b + 1] as number;
      const wbz = ang[3 * b + 2] as number;
      const vx =
        (lin[3 * b] as number) +
        (wby * sz - wbz * sy) -
        (lin[3 * a] as number) -
        (way * rz - waz * ry);
      const vy =
        (lin[3 * b + 1] as number) +
        (wbz * sx - wbx * sz) -
        (lin[3 * a + 1] as number) -
        (waz * rx - wax * rz);
      const vz =
        (lin[3 * b + 2] as number) +
        (wbx * sy - wby * sx) -
        (lin[3 * a + 2] as number) -
        (wax * ry - way * rx);
      // Both tangent velocities are cancelled together, then the 2D impulse is bounded by the
      // Coulomb cone (norm <= mu N), so friction is isotropic.
      let n1 = acc1 - (vx * t1x + vy * t1y + vz * t1z) * m1;
      let n2 = acc2 - (vx * t2x + vy * t2y + vz * t2z) * m2;
      const lenSq = n1 * n1 + n2 * n2;
      if (lenSq > maxF * maxF) {
        const sc = maxF / Math.sqrt(lenSq);
        n1 *= sc;
        n2 *= sc;
      }
      const d1 = n1 - acc1;
      const d2 = n2 - acc2;
      if (d1 === 0 && d2 === 0) continue;
      if (m1 > 0) this.track(Math.abs(d1) / m1);
      if (m2 > 0) this.track(Math.abs(d2) / m2);
      pI[3 * q + 1] = n1;
      pI[3 * q + 2] = n2;
      this.applyPair(a, b, q, t1x * d1 + t2x * d2, t1y * d1 + t2y * d2, t1z * d1 + t2z * d2);
    }
  }

  private track(dv: number): void {
    if (dv > this.maxChange) this.maxChange = dv;
  }

  /** Speed of body B relative to body A at point q along (dx, dy, dz). */
  private speedAlong(a: number, b: number, q: number, dx: number, dy: number, dz: number): number {
    const lin = this.lin;
    const ang = this.ang;
    const rx = this.pRA[3 * q] as number;
    const ry = this.pRA[3 * q + 1] as number;
    const rz = this.pRA[3 * q + 2] as number;
    const sx = this.pRB[3 * q] as number;
    const sy = this.pRB[3 * q + 1] as number;
    const sz = this.pRB[3 * q + 2] as number;
    const wax = ang[3 * a] as number;
    const way = ang[3 * a + 1] as number;
    const waz = ang[3 * a + 2] as number;
    const wbx = ang[3 * b] as number;
    const wby = ang[3 * b + 1] as number;
    const wbz = ang[3 * b + 2] as number;
    const vx =
      (lin[3 * b] as number) +
      (wby * sz - wbz * sy) -
      (lin[3 * a] as number) -
      (way * rz - waz * ry);
    const vy =
      (lin[3 * b + 1] as number) +
      (wbz * sx - wbx * sz) -
      (lin[3 * a + 1] as number) -
      (waz * rx - wax * rz);
    const vz =
      (lin[3 * b + 2] as number) +
      (wbx * sy - wby * sx) -
      (lin[3 * a + 2] as number) -
      (wax * ry - way * rx);
    return vx * dx + vy * dy + vz * dz;
  }

  /** Applies +P to body B and −P to body A at the offsets of point q. */
  private applyPair(a: number, b: number, q: number, px: number, py: number, pz: number): void {
    this.apply(
      a,
      -px,
      -py,
      -pz,
      this.pRA[3 * q] as number,
      this.pRA[3 * q + 1] as number,
      this.pRA[3 * q + 2] as number,
    );
    this.apply(
      b,
      px,
      py,
      pz,
      this.pRB[3 * q] as number,
      this.pRB[3 * q + 1] as number,
      this.pRB[3 * q + 2] as number,
    );
  }

  private apply(
    i: number,
    px: number,
    py: number,
    pz: number,
    rx: number,
    ry: number,
    rz: number,
  ): void {
    const im = this.invMass[i] as number;
    if (im === 0) return;
    const lin = this.lin;
    lin[3 * i] = (lin[3 * i] as number) + px * im;
    lin[3 * i + 1] = (lin[3 * i + 1] as number) + py * im;
    lin[3 * i + 2] = (lin[3 * i + 2] as number) + pz * im;
    const tx = ry * pz - rz * py;
    const ty = rz * px - rx * pz;
    const tz = rx * py - ry * px;
    const e = this.invInertia;
    const o = 9 * i;
    const ang = this.ang;
    ang[3 * i] =
      (ang[3 * i] as number) +
      (e[o] as number) * tx +
      (e[o + 1] as number) * ty +
      (e[o + 2] as number) * tz;
    ang[3 * i + 1] =
      (ang[3 * i + 1] as number) +
      (e[o + 3] as number) * tx +
      (e[o + 4] as number) * ty +
      (e[o + 5] as number) * tz;
    ang[3 * i + 2] =
      (ang[3 * i + 2] as number) +
      (e[o + 6] as number) * tx +
      (e[o + 7] as number) * ty +
      (e[o + 8] as number) * tz;
  }

  /**
   * Normal impulses of a multi-point manifold as a linear complementarity problem (see
   * `solveNormalBlock` in contactSolver.ts); returns false when no active set is consistent.
   */
  private solveBlock(
    k: number,
    a: number,
    b: number,
    nx: number,
    ny: number,
    nz: number,
    start: number,
    n: number,
  ): boolean {
    const K = this.K;
    const ko = this.cKStart[k] as number;
    for (let i = 0; i < n; i++) {
      let kl = 0;
      for (let j = 0; j < n; j++)
        kl += (K[ko + i * n + j] as number) * (this.pImpulse[3 * (start + j)] as number);
      BQ[i] =
        this.speedAlong(a, b, start + i, nx, ny, nz) - (this.pTarget[start + i] as number) - kl;
    }
    const EPS = 1e-12;
    for (let mask = (1 << n) - 1; mask >= 0; mask--) {
      let m = 0;
      for (let i = 0; i < n; i++) if (mask & (1 << i)) BS[m++] = i;
      BL.fill(0, 0, n);
      if (m > 0) {
        for (let r = 0; r < m; r++) {
          const si = BS[r] as number;
          for (let c = 0; c < m; c++)
            BM[r * (m + 1) + c] = K[ko + si * n + (BS[c] as number)] as number;
          BM[r * (m + 1) + m] = -(BQ[si] as number);
        }
        if (!eliminate(m)) continue;
        let ok = true;
        for (let r = 0; r < m; r++) {
          const v = BM[r * (m + 1) + m] as number;
          if (v < -EPS) ok = false;
          BL[BS[r] as number] = Math.max(0, v);
        }
        if (!ok) continue;
      }
      let feasible = true;
      for (let i = 0; i < n && feasible; i++) {
        if (mask & (1 << i)) continue;
        let w = BQ[i] as number;
        for (let j = 0; j < n; j++) w += (K[ko + i * n + j] as number) * (BL[j] as number);
        if (w < -1e-9) feasible = false;
      }
      if (!feasible) continue;
      this.applyBlock(a, b, nx, ny, nz, start, n);
      return true;
    }
    return false;
  }

  /** Applies the block solution in BL as incremental normal impulses. */
  private applyBlock(
    a: number,
    b: number,
    nx: number,
    ny: number,
    nz: number,
    start: number,
    n: number,
  ): void {
    for (let i = 0; i < n; i++) {
      const q = start + i;
      const lam = BL[i] as number;
      const d = lam - (this.pImpulse[3 * q] as number);
      this.pImpulse[3 * q] = lam;
      const mass = this.pMass[3 * q] as number;
      if (mass > 0) this.track(Math.abs(d) / mass);
      this.applyPair(a, b, q, nx * d, ny * d, nz * d);
    }
  }

  /** Writes velocities back to the bodies and impulses back to the constraint points. */
  scatter(): void {
    for (let i = 1; i < this.bodyCount; i++) {
      const b = this.bodies[i] as RigidBody;
      if (!b.isSimulated) continue;
      b.linearVelocity.x = this.lin[3 * i] as number;
      b.linearVelocity.y = this.lin[3 * i + 1] as number;
      b.linearVelocity.z = this.lin[3 * i + 2] as number;
      b.angularVelocity.x = this.ang[3 * i] as number;
      b.angularVelocity.y = this.ang[3 * i + 1] as number;
      b.angularVelocity.z = this.ang[3 * i + 2] as number;
    }
    let q = 0;
    for (const c of this.constraints) {
      for (const p of c.points) {
        p.impulseN = this.pImpulse[3 * q] as number;
        p.impulseT1 = this.pImpulse[3 * q + 1] as number;
        p.impulseT2 = this.pImpulse[3 * q + 2] as number;
        q++;
      }
    }
  }
}

const BLOCK_MAX = 4;
const BQ = new Float64Array(BLOCK_MAX);
const BL = new Float64Array(BLOCK_MAX);
const BS = new Int32Array(BLOCK_MAX);
const BM = new Float64Array(BLOCK_MAX * (BLOCK_MAX + 1));

/** Gaussian elimination with partial pivoting on the augmented system in BM; false if singular. */
function eliminate(m: number): boolean {
  const w = m + 1;
  for (let col = 0; col < m; col++) {
    let piv = col;
    for (let r = col + 1; r < m; r++) {
      if (Math.abs(BM[r * w + col] as number) > Math.abs(BM[piv * w + col] as number)) piv = r;
    }
    if (Math.abs(BM[piv * w + col] as number) < 1e-14) return false;
    if (piv !== col) {
      for (let c = 0; c < w; c++) {
        const t = BM[col * w + c] as number;
        BM[col * w + c] = BM[piv * w + c] as number;
        BM[piv * w + c] = t;
      }
    }
    const pc = BM[col * w + col] as number;
    for (let r = col + 1; r < m; r++) {
      const f = (BM[r * w + col] as number) / pc;
      for (let c = col; c <= m; c++) {
        BM[r * w + c] = (BM[r * w + c] as number) - f * (BM[col * w + c] as number);
      }
    }
  }
  for (let r = m - 1; r >= 0; r--) {
    let s = BM[r * w + m] as number;
    for (let c = r + 1; c < m; c++) s -= (BM[r * w + c] as number) * (BM[c * w + m] as number);
    BM[r * w + m] = s / (BM[r * w + r] as number);
  }
  return true;
}

function grow<T extends Float64Array | Int32Array>(a: T, size: number): T {
  const out = new (a.constructor as new (n: number) => T)(size);
  out.set(a);
  return out;
}
