import type { RigidBody } from '../dynamics/body';
import type { ContactConstraint, SolverPoint } from './contactSolver';

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
  private Kinv = new Float64Array(16 * 64);
  private cKValid = new Int32Array(64);
  private constraints: ContactConstraint[] = [];
  private constraintCount = 0;

  // Points.
  // Per point and row (normal, tangent 1, tangent 2): 12 doubles, JA (3), JB (3), IJA (3), IJB (3),
  // where J = r x d is the angular Jacobian and IJ = I^-1 J, so a row costs a few multiply-adds.
  private pJ = new Float64Array(36 * 256);
  private pMass = new Float64Array(3 * 256); // normal, tangent 1, tangent 2
  private pImpulse = new Float64Array(3 * 256);
  private pTarget = new Float64Array(256);
  private pSA = new Float64Array(3 * 256); // surface anchor offsets (position solve)
  private pSB = new Float64Array(3 * 256);
  private pos = new Float64Array(3 * 64);

  // Per-body change counters and per-constraint snapshots: a constraint whose two bodies were not
  // moved by any other constraint since it last settled cannot change anything and is skipped.
  private bodyMod = new Int32Array(64);
  private cSeen = new Int32Array(64);

  /** An impulse change that moves a speed by less than this (m/s) counts as converged. */
  skipTolerance = 1e-5;

  /** Largest speed change caused by an impulse update since the last `iterate` began. */
  maxChange = 0;

  private growBodies(n: number): void {
    if (n <= this.invMass.length) return;
    const cap = Math.max(n, this.invMass.length * 2);
    this.lin = grow(this.lin, 3 * cap);
    this.ang = grow(this.ang, 3 * cap);
    this.pos = grow(this.pos, 3 * cap);
    this.bodyMod = grow(this.bodyMod, cap);
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
      this.cSeen = grow(this.cSeen, cap);
      this.cKValid = grow(this.cKValid, cap);
    }
    if (kn > this.K.length) {
      const cap = Math.max(kn, this.K.length * 2);
      this.K = grow(this.K, cap);
      this.Kinv = grow(this.Kinv, cap);
    }
  }

  private growPoints(n: number): void {
    if (n <= this.pTarget.length) return;
    const cap = Math.max(n, this.pTarget.length * 2);
    this.pJ = grow(this.pJ, 36 * cap);
    this.pMass = grow(this.pMass, 3 * cap);
    this.pImpulse = grow(this.pImpulse, 3 * cap);
    this.pTarget = grow(this.pTarget, cap);
    this.pSA = grow(this.pSA, 3 * cap);
    this.pSB = grow(this.pSB, 3 * cap);
  }

  private slot(b: RigidBody): number {
    if (b.batchStamp === this.stamp) return b.batchIndex;
    b.batchStamp = this.stamp;
    const i = this.bodyCount++;
    this.growBodies(i + 1);
    this.bodyMod[i] = 0;
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

  /**
   * Builds the arrays straight from the constraints and the current poses: offsets from the
   * centers of mass, the Jacobian rows (angular Jacobians and their inverse-inertia products),
   * effective masses along normal and tangents, the coupled normal matrix and the restitution
   * target. Also writes `rA`/`rB` back to the points for debug drawing.
   */
  prepare(contacts: readonly ContactConstraint[], threshold: number, dt: number): void {
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
    const J = this.pJ;
    let k = 0;
    let q = 0;
    let kOff = 0;
    for (const c of contacts) {
      if (!(c.bodyA.isSimulated || c.bodyB.isSimulated)) continue;
      const bodyA = c.bodyA;
      const bodyB = c.bodyB;
      this.constraints.push(c);
      this.cBodyA[k] = this.slot(bodyA);
      this.cBodyB[k] = this.slot(bodyB);
      this.cSeen[k] = -1;
      const nx = c.normal.x;
      const ny = c.normal.y;
      const nz = c.normal.z;
      const t1 = c.tangent1;
      const t2 = c.tangent2;
      this.cNormal[3 * k] = nx;
      this.cNormal[3 * k + 1] = ny;
      this.cNormal[3 * k + 2] = nz;
      this.cTan1[3 * k] = t1.x;
      this.cTan1[3 * k + 1] = t1.y;
      this.cTan1[3 * k + 2] = t1.z;
      this.cTan2[3 * k] = t2.x;
      this.cTan2[3 * k + 1] = t2.y;
      this.cTan2[3 * k + 2] = t2.z;
      this.cFriction[k] = c.friction;
      const pts = c.points;
      const n = pts.length;
      this.cPointStart[k] = q;
      this.cPointCount[k] = n;
      this.cKStart[k] = kOff;
      const aSim = bodyA.isSimulated;
      const bSim = bodyB.isSimulated;
      const ea = bodyA.worldInvInertia.e;
      const eb = bodyB.worldInvInertia.e;
      const qa = bodyA.orientation;
      const qb = bodyB.orientation;
      const pa = bodyA.position;
      const pb = bodyB.position;
      const base = (aSim ? bodyA.invMass : 0) + (bSim ? bodyB.invMass : 0);
      const sla = bodyA.stepStartLinear;
      const saa = bodyA.stepStartAngular;
      const slb = bodyB.stepStartLinear;
      const sab = bodyB.stepStartAngular;
      for (let i = 0; i < n; i++) {
        const p = pts[i] as SolverPoint;
        // Surface anchors as world offsets (the body-frame anchors rotated by the orientation).
        rot(qa, p.localA, 0);
        rot(qb, p.localB, 3);
        const sax = R[0] as number;
        const say = R[1] as number;
        const saz = R[2] as number;
        const sbx = R[3] as number;
        const sby = R[4] as number;
        const sbz = R[5] as number;
        const o = 3 * (q + i);
        this.pSA[o] = sax;
        this.pSA[o + 1] = say;
        this.pSA[o + 2] = saz;
        this.pSB[o] = sbx;
        this.pSB[o + 1] = sby;
        this.pSB[o + 2] = sbz;
        // Impulses act at the midpoint of the two surface anchors.
        const midx = 0.5 * (pa.x + sax + pb.x + sbx);
        const midy = 0.5 * (pa.y + say + pb.y + sby);
        const midz = 0.5 * (pa.z + saz + pb.z + sbz);
        const rax = midx - pa.x;
        const ray = midy - pa.y;
        const raz = midz - pa.z;
        const rbx = midx - pb.x;
        const rby = midy - pb.y;
        const rbz = midz - pb.z;
        p.rA.x = rax;
        p.rA.y = ray;
        p.rA.z = raz;
        p.rB.x = rbx;
        p.rB.y = rby;
        p.rB.z = rbz;
        const jo = 36 * (q + i);
        const mN =
          base + jacobianRow(J, jo, aSim, ea, bSim, eb, rax, ray, raz, rbx, rby, rbz, nx, ny, nz);
        const m1 =
          base +
          jacobianRow(
            J,
            jo + 12,
            aSim,
            ea,
            bSim,
            eb,
            rax,
            ray,
            raz,
            rbx,
            rby,
            rbz,
            t1.x,
            t1.y,
            t1.z,
          );
        const m2 =
          base +
          jacobianRow(
            J,
            jo + 24,
            aSim,
            ea,
            bSim,
            eb,
            rax,
            ray,
            raz,
            rbx,
            rby,
            rbz,
            t2.x,
            t2.y,
            t2.z,
          );
        this.pMass[o] = mN > 0 ? 1 / mN : 0;
        this.pMass[o + 1] = m1 > 0 ? 1 / m1 : 0;
        this.pMass[o + 2] = m2 > 0 ? 1 / m2 : 0;
        // Restitution from the start-of-step velocities.
        const vx = slb.x + (sab.y * rbz - sab.z * rby) - sla.x - (saa.y * raz - saa.z * ray);
        const vy = slb.y + (sab.z * rbx - sab.x * rbz) - sla.y - (saa.z * rax - saa.x * raz);
        const vz = slb.z + (sab.x * rby - sab.y * rbx) - sla.z - (saa.x * ray - saa.y * rax);
        const vn0 = vx * nx + vy * ny + vz * nz;
        let target = vn0 < -threshold ? -c.restitution * vn0 : 0;
        if (p.gap > 0 && dt > 0) target = -p.gap / dt;
        this.pTarget[q + i] = target;
        this.pImpulse[o] = p.impulseN;
        this.pImpulse[o + 1] = p.impulseT1;
        this.pImpulse[o + 2] = p.impulseT2;
      }
      // Coupled normal matrix (symmetric), with the small diagonal term that keeps coplanar blocks
      // well posed: K_ij = invMass terms + JA_i . IJA_j + JB_i . IJB_j on the normal rows.
      let maxDiag = 0;
      for (let i = 0; i < n; i++) {
        const ji = 36 * (q + i);
        for (let j = i; j < n; j++) {
          const jj = 36 * (q + j);
          const kk =
            base +
            (J[ji] as number) * (J[jj + 6] as number) +
            (J[ji + 1] as number) * (J[jj + 7] as number) +
            (J[ji + 2] as number) * (J[jj + 8] as number) +
            (J[ji + 3] as number) * (J[jj + 9] as number) +
            (J[ji + 4] as number) * (J[jj + 10] as number) +
            (J[ji + 5] as number) * (J[jj + 11] as number);
          this.K[kOff + i * n + j] = kk;
          this.K[kOff + j * n + i] = kk;
          if (i === j && kk > maxDiag) maxDiag = kk;
        }
      }
      const eps = 1e-9 * maxDiag;
      for (let i = 0; i < n; i++)
        this.K[kOff + i * n + i] = (this.K[kOff + i * n + i] as number) + eps;
      this.cKValid[k] = n > 1 && n <= BLOCK_MAX && invert(this.K, kOff, n, this.Kinv) ? 1 : 0;
      kOff += n * n;
      q += n;
      k++;
    }
    this.constraintCount = k;
  }

  /**
   * Nonlinear Gauss-Seidel position correction along the contact normals, linear only (see
   * `solvePosition` in contactSolver.ts). Reads the bodies' current positions, writes them back.
   */
  solvePositions(passes: number, baumgarte: number, slop: number, maxStep = Infinity): void {
    const pos = this.pos;
    for (let i = 1; i < this.bodyCount; i++) {
      const b = this.bodies[i] as RigidBody;
      pos[3 * i] = b.position.x;
      pos[3 * i + 1] = b.position.y;
      pos[3 * i + 2] = b.position.z;
    }
    for (let pass = 0; pass < passes; pass++) {
      for (let k = 0; k < this.constraintCount; k++) {
        const a = this.cBodyA[k] as number;
        const b = this.cBodyB[k] as number;
        const imA = this.invMass[a] as number;
        const imB = this.invMass[b] as number;
        const kn = imA + imB;
        if (!(kn > 0)) continue;
        const nx = this.cNormal[3 * k] as number;
        const ny = this.cNormal[3 * k + 1] as number;
        const nz = this.cNormal[3 * k + 2] as number;
        const start = this.cPointStart[k] as number;
        const end = start + (this.cPointCount[k] as number);
        for (let q = start; q < end; q++) {
          const o = 3 * q;
          const sep =
            ((pos[3 * b] as number) +
              (this.pSB[o] as number) -
              (pos[3 * a] as number) -
              (this.pSA[o] as number)) *
              nx +
            ((pos[3 * b + 1] as number) +
              (this.pSB[o + 1] as number) -
              (pos[3 * a + 1] as number) -
              (this.pSA[o + 1] as number)) *
              ny +
            ((pos[3 * b + 2] as number) +
              (this.pSB[o + 2] as number) -
              (pos[3 * a + 2] as number) -
              (this.pSA[o + 2] as number)) *
              nz;
          const error = Math.max(-maxStep / baumgarte, Math.min(0, sep + slop));
          if (error === 0) continue;
          const s = (-baumgarte * error) / kn;
          if (imA !== 0) {
            pos[3 * a] = (pos[3 * a] as number) + -nx * s * imA;
            pos[3 * a + 1] = (pos[3 * a + 1] as number) + -ny * s * imA;
            pos[3 * a + 2] = (pos[3 * a + 2] as number) + -nz * s * imA;
          }
          if (imB !== 0) {
            pos[3 * b] = (pos[3 * b] as number) + nx * s * imB;
            pos[3 * b + 1] = (pos[3 * b + 1] as number) + ny * s * imB;
            pos[3 * b + 2] = (pos[3 * b + 2] as number) + nz * s * imB;
          }
        }
      }
    }
    for (let i = 1; i < this.bodyCount; i++) {
      const b = this.bodies[i] as RigidBody;
      if (!b.isSimulated) continue;
      b.position.x = pos[3 * i] as number;
      b.position.y = pos[3 * i + 1] as number;
      b.position.z = pos[3 * i + 2] as number;
    }
  }

  /** Re-applies the accumulated impulses from the previous step. */
  warmStart(): void {
    const lin = this.lin;
    const ang = this.ang;
    const J = this.pJ;
    for (let k = 0; k < this.constraintCount; k++) {
      const a = this.cBodyA[k] as number;
      const b = this.cBodyB[k] as number;
      const imA = this.invMass[a] as number;
      const imB = this.invMass[b] as number;
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
        const px = nx * jn + t1x * j1 + t2x * j2;
        const py = ny * jn + t1y * j1 + t2y * j2;
        const pz = nz * jn + t1z * j1 + t2z * j2;
        const o = 36 * q;
        lin[3 * a] = (lin[3 * a] as number) - px * imA;
        lin[3 * a + 1] = (lin[3 * a + 1] as number) - py * imA;
        lin[3 * a + 2] = (lin[3 * a + 2] as number) - pz * imA;
        lin[3 * b] = (lin[3 * b] as number) + px * imB;
        lin[3 * b + 1] = (lin[3 * b + 1] as number) + py * imB;
        lin[3 * b + 2] = (lin[3 * b + 2] as number) + pz * imB;
        for (let c = 0; c < 3; c++) {
          ang[3 * a + c] =
            (ang[3 * a + c] as number) -
            ((J[o + 6 + c] as number) * jn +
              (J[o + 18 + c] as number) * j1 +
              (J[o + 30 + c] as number) * j2);
          ang[3 * b + c] =
            (ang[3 * b + c] as number) +
            ((J[o + 9 + c] as number) * jn +
              (J[o + 21 + c] as number) * j1 +
              (J[o + 33 + c] as number) * j2);
        }
      }
    }
  }

  /**
   * One sequential-impulse pass over every constraint (normals as a block, then friction).
   * Constraints whose bodies no other constraint has moved since they last settled are skipped.
   * `maxChange` is the largest speed change of the pass.
   */
  iterate(): void {
    const lin = this.lin;
    const ang = this.ang;
    const invMass = this.invMass;
    const J = this.pJ;
    const pI = this.pImpulse;
    const pM = this.pMass;
    const pT = this.pTarget;
    const cBodyA = this.cBodyA;
    const cBodyB = this.cBodyB;
    const cNormal = this.cNormal;
    const cTan1 = this.cTan1;
    const cTan2 = this.cTan2;
    const cPointStart = this.cPointStart;
    const cPointCount = this.cPointCount;
    const mod = this.bodyMod;
    const seen = this.cSeen;
    const cKValid = this.cKValid;
    const eps = this.skipTolerance;
    let maxChange = 0;
    for (let k = 0; k < this.constraintCount; k++) {
      const a = cBodyA[k] as number;
      const b = cBodyB[k] as number;
      const sig = (mod[a] as number) + (mod[b] as number);
      if (seen[k] === sig) continue;
      const imA = invMass[a] as number;
      const imB = invMass[b] as number;
      const a3 = 3 * a;
      const b3 = 3 * b;
      let lax = lin[a3] as number;
      let lay = lin[a3 + 1] as number;
      let laz = lin[a3 + 2] as number;
      let wax = ang[a3] as number;
      let way = ang[a3 + 1] as number;
      let waz = ang[a3 + 2] as number;
      let lbx = lin[b3] as number;
      let lby = lin[b3 + 1] as number;
      let lbz = lin[b3 + 2] as number;
      let wbx = ang[b3] as number;
      let wby = ang[b3 + 1] as number;
      let wbz = ang[b3 + 2] as number;
      const k3 = 3 * k;
      const nx = cNormal[k3] as number;
      const ny = cNormal[k3 + 1] as number;
      const nz = cNormal[k3 + 2] as number;
      const start = cPointStart[k] as number;
      const n = cPointCount[k] as number;
      const end = start + n;
      let changed = false;

      // Normal rows. Multi-point manifolds are first tried as one linear complementarity problem.
      let blocked = false;
      if (n > 1 && n <= BLOCK_MAX) {
        const ko = this.cKStart[k] as number;
        const K = this.K;
        const base = (lbx - lax) * nx + (lby - lay) * ny + (lbz - laz) * nz;
        for (let i = 0; i < n; i++) {
          const q = start + i;
          const o = 36 * q;
          // Speed error along the normal; BQ also holds it for the active-set fallback below.
          BE[i] =
            base +
            ((J[o + 3] as number) * wbx + (J[o + 4] as number) * wby + (J[o + 5] as number) * wbz) -
            ((J[o] as number) * wax + (J[o + 1] as number) * way + (J[o + 2] as number) * waz) -
            (pT[q] as number);
        }
        // Start from the points that carried load last time. When that is all of them:
        // lambda = lambda_old - K^-1 (speed error), valid when none is negative.
        let full = 0;
        for (let i = 0; i < n; i++) if ((pI[3 * (start + i)] as number) > 0) full |= 1 << i;
        if (full === 0) full = (1 << n) - 1;
        if (cKValid[k] === 1 && full === (1 << n) - 1) {
          const Ki = this.Kinv;
          blocked = true;
          for (let i = 0; i < n; i++) {
            let l = pI[3 * (start + i)] as number;
            for (let j = 0; j < n; j++) l -= (Ki[ko + i * n + j] as number) * (BE[j] as number);
            if (l < -1e-12) {
              blocked = false;
              break;
            }
            BL[i] = l > 0 ? l : 0;
          }
        }
        if (!blocked) {
          for (let i = 0; i < n; i++) {
            let kl = 0;
            for (let j = 0; j < n; j++)
              kl += (K[ko + i * n + j] as number) * (pI[3 * (start + j)] as number);
            BQ[i] = (BE[i] as number) - kl;
          }
          blocked = solveBlock(K, ko, n, full);
        }
      }
      for (let q = start; q < end; q++) {
        const o = 36 * q;
        const old = pI[3 * q] as number;
        let next: number;
        if (blocked) {
          next = BL[q - start] as number;
        } else {
          const vn =
            (lbx - lax) * nx +
            (lby - lay) * ny +
            (lbz - laz) * nz +
            ((J[o + 3] as number) * wbx + (J[o + 4] as number) * wby + (J[o + 5] as number) * wbz) -
            ((J[o] as number) * wax + (J[o + 1] as number) * way + (J[o + 2] as number) * waz);
          next = old + ((pT[q] as number) - vn) * (pM[3 * q] as number);
          if (next < 0) next = 0;
        }
        const d = next - old;
        if (d === 0) continue;
        pI[3 * q] = next;
        const mass = pM[3 * q] as number;
        const ad = d < 0 ? -d : d;
        if (ad > eps * mass) {
          changed = true;
          const dv = ad / mass;
          if (dv > maxChange) maxChange = dv;
        }
        lax -= nx * d * imA;
        lay -= ny * d * imA;
        laz -= nz * d * imA;
        lbx += nx * d * imB;
        lby += ny * d * imB;
        lbz += nz * d * imB;
        wax -= (J[o + 6] as number) * d;
        way -= (J[o + 7] as number) * d;
        waz -= (J[o + 8] as number) * d;
        wbx += (J[o + 9] as number) * d;
        wby += (J[o + 10] as number) * d;
        wbz += (J[o + 11] as number) * d;
      }

      // Friction: both tangent speeds are cancelled together, then the 2D impulse is bounded by the
      // Coulomb cone (norm <= mu N), so friction is isotropic.
      const mu = this.cFriction[k] as number;
      const t1x = cTan1[k3] as number;
      const t1y = cTan1[k3 + 1] as number;
      const t1z = cTan1[k3 + 2] as number;
      const t2x = cTan2[k3] as number;
      const t2y = cTan2[k3 + 1] as number;
      const t2z = cTan2[k3 + 2] as number;
      for (let q = start; q < end; q++) {
        const acc1 = pI[3 * q + 1] as number;
        const acc2 = pI[3 * q + 2] as number;
        const maxF = mu * (pI[3 * q] as number);
        // No normal load and nothing accumulated: the clamped impulse is exactly zero.
        if (maxF === 0 && acc1 === 0 && acc2 === 0) continue;
        const m1 = pM[3 * q + 1] as number;
        const m2 = pM[3 * q + 2] as number;
        const o1 = 36 * q + 12;
        const o2 = o1 + 12;
        const dx = lbx - lax;
        const dy = lby - lay;
        const dz = lbz - laz;
        const v1 =
          dx * t1x +
          dy * t1y +
          dz * t1z +
          ((J[o1 + 3] as number) * wbx +
            (J[o1 + 4] as number) * wby +
            (J[o1 + 5] as number) * wbz) -
          ((J[o1] as number) * wax + (J[o1 + 1] as number) * way + (J[o1 + 2] as number) * waz);
        const v2 =
          dx * t2x +
          dy * t2y +
          dz * t2z +
          ((J[o2 + 3] as number) * wbx +
            (J[o2 + 4] as number) * wby +
            (J[o2 + 5] as number) * wbz) -
          ((J[o2] as number) * wax + (J[o2 + 1] as number) * way + (J[o2 + 2] as number) * waz);
        let n1 = acc1 - v1 * m1;
        let n2 = acc2 - v2 * m2;
        const lenSq = n1 * n1 + n2 * n2;
        if (lenSq > maxF * maxF) {
          const sc = maxF / Math.sqrt(lenSq);
          n1 *= sc;
          n2 *= sc;
        }
        const d1 = n1 - acc1;
        const d2 = n2 - acc2;
        if (d1 === 0 && d2 === 0) continue;
        const ad1 = d1 < 0 ? -d1 : d1;
        const ad2 = d2 < 0 ? -d2 : d2;
        if (ad1 > eps * m1 || ad2 > eps * m2) {
          changed = true;
          const dv1 = m1 > 0 ? ad1 / m1 : 0;
          const dv2 = m2 > 0 ? ad2 / m2 : 0;
          if (dv1 > maxChange) maxChange = dv1;
          if (dv2 > maxChange) maxChange = dv2;
        }
        pI[3 * q + 1] = n1;
        pI[3 * q + 2] = n2;
        const px = t1x * d1 + t2x * d2;
        const py = t1y * d1 + t2y * d2;
        const pz = t1z * d1 + t2z * d2;
        lax -= px * imA;
        lay -= py * imA;
        laz -= pz * imA;
        lbx += px * imB;
        lby += py * imB;
        lbz += pz * imB;
        wax -= (J[o1 + 6] as number) * d1 + (J[o2 + 6] as number) * d2;
        way -= (J[o1 + 7] as number) * d1 + (J[o2 + 7] as number) * d2;
        waz -= (J[o1 + 8] as number) * d1 + (J[o2 + 8] as number) * d2;
        wbx += (J[o1 + 9] as number) * d1 + (J[o2 + 9] as number) * d2;
        wby += (J[o1 + 10] as number) * d1 + (J[o2 + 10] as number) * d2;
        wbz += (J[o1 + 11] as number) * d1 + (J[o2 + 11] as number) * d2;
      }

      if (imA > 0) {
        lin[a3] = lax;
        lin[a3 + 1] = lay;
        lin[a3 + 2] = laz;
        ang[a3] = wax;
        ang[a3 + 1] = way;
        ang[a3 + 2] = waz;
      }
      if (imB > 0) {
        lin[b3] = lbx;
        lin[b3 + 1] = lby;
        lin[b3 + 2] = lbz;
        ang[b3] = wbx;
        ang[b3 + 1] = wby;
        ang[b3 + 2] = wbz;
      }
      if (changed) {
        seen[k] = -1;
        if (imA > 0) mod[a] = (mod[a] as number) + 1;
        if (imB > 0) mod[b] = (mod[b] as number) + 1;
      } else {
        seen[k] = sig;
      }
    }
    this.maxChange = maxChange;
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

const R = new Float64Array(6);

/** Rotates v by q into R[off..off+2]. */
function rot(
  q: { x: number; y: number; z: number; w: number },
  v: { x: number; y: number; z: number },
  off: number,
): void {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  R[off] = v.x + q.w * tx + (q.y * tz - q.z * ty);
  R[off + 1] = v.y + q.w * ty + (q.z * tx - q.x * tz);
  R[off + 2] = v.z + q.w * tz + (q.x * ty - q.y * tx);
}

/**
 * Writes one Jacobian row at J[o..o+11]: JA = rA x d, JB = rB x d, IJA = I_A^-1 JA, IJB = I_B^-1 JB
 * (zero for bodies that do not move), and returns JA . IJA + JB . IJB.
 */
function jacobianRow(
  J: Float64Array,
  o: number,
  aSim: boolean,
  ea: ArrayLike<number>,
  bSim: boolean,
  eb: ArrayLike<number>,
  rax: number,
  ray: number,
  raz: number,
  rbx: number,
  rby: number,
  rbz: number,
  dx: number,
  dy: number,
  dz: number,
): number {
  const ax = ray * dz - raz * dy;
  const ay = raz * dx - rax * dz;
  const az = rax * dy - ray * dx;
  const bx = rby * dz - rbz * dy;
  const by = rbz * dx - rbx * dz;
  const bz = rbx * dy - rby * dx;
  J[o] = ax;
  J[o + 1] = ay;
  J[o + 2] = az;
  J[o + 3] = bx;
  J[o + 4] = by;
  J[o + 5] = bz;
  let m = 0;
  if (aSim) {
    const ix = (ea[0] as number) * ax + (ea[1] as number) * ay + (ea[2] as number) * az;
    const iy = (ea[3] as number) * ax + (ea[4] as number) * ay + (ea[5] as number) * az;
    const iz = (ea[6] as number) * ax + (ea[7] as number) * ay + (ea[8] as number) * az;
    J[o + 6] = ix;
    J[o + 7] = iy;
    J[o + 8] = iz;
    m += ax * ix + ay * iy + az * iz;
  } else {
    J[o + 6] = 0;
    J[o + 7] = 0;
    J[o + 8] = 0;
  }
  if (bSim) {
    const ix = (eb[0] as number) * bx + (eb[1] as number) * by + (eb[2] as number) * bz;
    const iy = (eb[3] as number) * bx + (eb[4] as number) * by + (eb[5] as number) * bz;
    const iz = (eb[6] as number) * bx + (eb[7] as number) * by + (eb[8] as number) * bz;
    J[o + 9] = ix;
    J[o + 10] = iy;
    J[o + 11] = iz;
    m += bx * ix + by * iy + bz * iz;
  } else {
    J[o + 9] = 0;
    J[o + 10] = 0;
    J[o + 11] = 0;
  }
  return m;
}

const BLOCK_MAX = 4;
const BQ = new Float64Array(BLOCK_MAX);
const BE = new Float64Array(BLOCK_MAX);
const BI = new Float64Array(BLOCK_MAX * 2 * BLOCK_MAX);
const BL = new Float64Array(BLOCK_MAX);
const BS = new Int32Array(BLOCK_MAX);
const BM = new Float64Array(BLOCK_MAX * (BLOCK_MAX + 1));

/**
 * Normal impulses of a multi-point manifold as a linear complementarity problem (see
 * `solveNormalBlock` in contactSolver.ts). Reads the residual speeds in BQ, writes the new
 * accumulated impulses to BL; returns false when no active set is consistent.
 */
function solveBlock(K: Float64Array, ko: number, n: number, start: number): boolean {
  const EPS = 1e-12;
  // K is positive definite, so the solution is unique and any active-set path finds it. Principal
  // pivoting (drop the most negative impulse, add the most violated point) usually ends in one or
  // two solves; the exhaustive search below is the fallback if it cycles.
  let set = start;
  for (let round = 0; round < 6; round++) {
    let m = 0;
    for (let i = 0; i < n; i++) if (set & (1 << i)) BS[m++] = i;
    BL.fill(0, 0, n);
    if (m > 0) {
      for (let r = 0; r < m; r++) {
        const si = BS[r] as number;
        for (let c = 0; c < m; c++)
          BM[r * (m + 1) + c] = K[ko + si * n + (BS[c] as number)] as number;
        BM[r * (m + 1) + m] = -(BQ[si] as number);
      }
      if (!eliminate(m)) break;
      let worst = -EPS;
      let drop = -1;
      for (let r = 0; r < m; r++) {
        const v = BM[r * (m + 1) + m] as number;
        if (v < worst) {
          worst = v;
          drop = BS[r] as number;
        }
        BL[BS[r] as number] = v > 0 ? v : 0;
      }
      if (drop >= 0) {
        set &= ~(1 << drop);
        continue;
      }
    }
    let add = -1;
    let low = -1e-9;
    for (let i = 0; i < n; i++) {
      if (set & (1 << i)) continue;
      let w = BQ[i] as number;
      for (let j = 0; j < n; j++) w += (K[ko + i * n + j] as number) * (BL[j] as number);
      if (w < low) {
        low = w;
        add = i;
      }
    }
    if (add < 0) return true;
    set |= 1 << add;
  }
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
        BL[BS[r] as number] = v > 0 ? v : 0;
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
    return true;
  }
  return false;
}

/**
 * Inverts the symmetric n x n block at K[ko..] into out[ko..] (Gauss-Jordan, partial pivoting);
 * false when it is singular.
 */
function invert(K: Float64Array, ko: number, n: number, out: Float64Array): boolean {
  const w = 2 * n;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      BI[i * w + j] = K[ko + i * n + j] as number;
      BI[i * w + n + j] = i === j ? 1 : 0;
    }
  }
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(BI[r * w + col] as number) > Math.abs(BI[piv * w + col] as number)) piv = r;
    }
    const pv = BI[piv * w + col] as number;
    if (Math.abs(pv) < 1e-14) return false;
    if (piv !== col) {
      for (let c = 0; c < w; c++) {
        const t = BI[col * w + c] as number;
        BI[col * w + c] = BI[piv * w + c] as number;
        BI[piv * w + c] = t;
      }
    }
    const inv = 1 / pv;
    for (let c = 0; c < w; c++) BI[col * w + c] = (BI[col * w + c] as number) * inv;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = BI[r * w + col] as number;
      if (f === 0) continue;
      for (let c = 0; c < w; c++)
        BI[r * w + c] = (BI[r * w + c] as number) - f * (BI[col * w + c] as number);
    }
  }
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) out[ko + i * n + j] = BI[i * w + n + j] as number;
  return true;
}

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
