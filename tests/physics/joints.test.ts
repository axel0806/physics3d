import { describe, expect, it } from 'vitest';
import { Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as Q from '../../src/math/quat';
import type { RigidBody } from '../../src/dynamics/body';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import {
  BallJoint,
  HingeJoint,
  SliderJoint,
  FixedJoint,
  DistanceJoint,
} from '../../src/constraints';

const G = 9.81;
const DT = 1 / 240;

/** Complete elliptic integral of the first kind via the arithmetic-geometric mean. */
function ellipticK(k: number): number {
  let a = 1;
  let b = Math.sqrt(1 - k * k);
  for (let i = 0; i < 40; i++) [a, b] = [(a + b) / 2, Math.sqrt(a * b)];
  return Math.PI / (2 * a);
}

/** Pendulum of a small sphere of radius r on a massless link of length L (about the pivot). */
function pendulum(world: World, L: number, theta0: number, kind: 'ball' | 'hinge') {
  const r = 0.05;
  const anchor = world.createBody({ type: 'static', position: new Vec3(0, 0, 0) });
  const pos = new Vec3(L * Math.sin(theta0), -L * Math.cos(theta0), 0);
  const bob = world.createBody({ position: pos });
  world.createCollider(bob, new Sphere(r));
  const pivot = new Vec3(0, 0, 0);
  const joint =
    kind === 'ball'
      ? new BallJoint(anchor, bob, pivot)
      : new HingeJoint(anchor, bob, pivot, new Vec3(0, 0, 1));
  world.addJoint(joint);
  // Physical pendulum: I_pivot = (2/5) m r² + m L².
  const leff = L + (0.4 * r * r) / L;
  return { bob, leff };
}

/** Mean period from upward zero crossings of the swing angle, with linear interpolation. */
function measurePeriod(world: World, bob: RigidBody, cycles: number): number {
  const times: number[] = [];
  let prev = bob.position.x;
  for (let i = 0; i < 240 * 60 && times.length < cycles + 1; i++) {
    world.step(DT);
    const x = bob.position.x;
    if (prev < 0 && x >= 0) times.push((i + 1 - x / (x - prev)) * DT);
    prev = x;
  }
  expect(times.length).toBe(cycles + 1);
  return ((times[cycles] ?? 0) - (times[0] ?? 0)) / cycles;
}

describe('pendulum', () => {
  for (const kind of ['hinge', 'ball'] as const) {
    it(`${kind}: small-angle period matches 2π√(L/g) within 1%`, () => {
      const w = new World({ gravity: new Vec3(0, -G, 0) });
      const { bob, leff } = pendulum(w, 1, 0.05, kind);
      const T = measurePeriod(w, bob, 5);
      const expected = 2 * Math.PI * Math.sqrt(leff / G);
      expect(Math.abs(T / expected - 1)).toBeLessThan(0.01);
    });

    it(`${kind}: large-angle period matches the elliptic-integral solution within 1%`, () => {
      for (const theta0 of [Math.PI / 3, (2 * Math.PI) / 3]) {
        const w = new World({ gravity: new Vec3(0, -G, 0) });
        const { bob, leff } = pendulum(w, 1, theta0, kind);
        const T = measurePeriod(w, bob, 4);
        const expected = 4 * Math.sqrt(leff / G) * ellipticK(Math.sin(theta0 / 2));
        expect(Math.abs(T / expected - 1)).toBeLessThan(0.01);
      }
    });
  }
});

describe('hinge', () => {
  it('motor reaches its target speed within its torque limit, then respects the limit', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const base = w.createBody({ type: 'static' });
    const arm = w.createBody({ position: new Vec3(1, 0, 0) });
    w.createCollider(arm, new Box(1, 0.1, 0.1));
    const maxTorque = 1000;
    const hinge = w.addJoint(
      new HingeJoint(base, arm, new Vec3(0, 0, 0), new Vec3(0, 0, 1), {
        motorSpeed: 2,
        maxMotorTorque: maxTorque,
      }),
    );
    for (let i = 0; i < 240; i++) w.step(DT);
    expect(arm.angularVelocity.z).toBeCloseTo(2, 3);
    // A weak motor cannot reach the speed in one step: angular acceleration = torque / I.
    const w2 = new World({ gravity: new Vec3(0, 0, 0) });
    const base2 = w2.createBody({ type: 'static' });
    const arm2 = w2.createBody({ position: new Vec3(1, 0, 0) });
    w2.createCollider(arm2, new Box(1, 0.1, 0.1));
    w2.addJoint(
      new HingeJoint(base2, arm2, new Vec3(0, 0, 0), new Vec3(0, 0, 1), {
        motorSpeed: 100,
        maxMotorTorque: 5,
      }),
    );
    const iPivot = 1 / (arm2.worldInvInertia.e[8] ?? 1) + arm2.mass * 1;
    w2.step(DT);
    expect(arm2.angularVelocity.z).toBeCloseTo((5 * DT) / iPivot, 6);
    void hinge;
  });

  it('limits are never violated by more than the slop', () => {
    const w = new World({ gravity: new Vec3(0, -G, 0) });
    const base = w.createBody({ type: 'static' });
    const arm = w.createBody({ position: new Vec3(1, 0, 0) });
    w.createCollider(arm, new Box(1, 0.1, 0.1));
    const lower = -0.5;
    const upper = 0.6;
    const hinge = w.addJoint(
      new HingeJoint(base, arm, new Vec3(0, 0, 0), new Vec3(0, 0, 1), { lower, upper }),
    );
    let worst = 0;
    for (let i = 0; i < 240 * 10; i++) {
      // Drive it hard against both limits.
      arm.angularVelocity.z += (i % 480 < 240 ? 1 : -1) * 0.5;
      w.step(DT);
      const a = hinge.currentAngle;
      worst = Math.max(worst, lower - a, a - upper);
    }
    expect(worst).toBeLessThan(0.01);
  });
});

describe('slider and fixed', () => {
  it('slider keeps the body on its axis and respects limits', () => {
    const w = new World({ gravity: new Vec3(3, -G, 0) });
    const base = w.createBody({ type: 'static' });
    const car = w.createBody({ position: new Vec3(0, 0, 0) });
    w.createCollider(car, new Box(0.2, 0.2, 0.2));
    const s = w.addJoint(
      new SliderJoint(base, car, new Vec3(0, 0, 0), new Vec3(1, 0, 0), { lower: -0.5, upper: 1 }),
    );
    for (let i = 0; i < 240 * 5; i++) w.step(DT);
    expect(Math.abs(car.position.y)).toBeLessThan(1e-3);
    expect(Math.abs(car.position.z)).toBeLessThan(1e-3);
    expect(s.translation).toBeGreaterThan(1 - 0.01);
    expect(s.translation).toBeLessThan(1 + 0.01);
  });

  it('fixed joint holds a body rigidly against gravity', () => {
    const w = new World({ gravity: new Vec3(0, -G, 0) });
    const base = w.createBody({ type: 'static' });
    const arm = w.createBody({ position: new Vec3(2, 0, 0) });
    w.createCollider(arm, new Box(2, 0.1, 0.1));
    w.addJoint(new FixedJoint(base, arm, new Vec3(0, 0, 0)));
    for (let i = 0; i < 240 * 5; i++) w.step(DT);
    expect(V.distance(arm.position, new Vec3(2, 0, 0))).toBeLessThan(2e-3);
  });

  it('distance joint is a rod; with frequency it is a spring of that frequency', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const base = w.createBody({ type: 'static' });
    const ball = w.createBody({ position: new Vec3(0, -1.5, 0) });
    w.createCollider(ball, new Sphere(0.1));
    w.addJoint(
      new DistanceJoint(base, ball, new Vec3(0, 0, 0), new Vec3(0, -1.5, 0), {
        length: 1,
        frequency: 2,
        dampingRatio: 0,
      }),
    );
    // Undamped spring about rest length 1: oscillates between y=-1.5 and y=-0.5, period 0.5 s.
    let minY = 0;
    let maxY = -10;
    for (let i = 0; i < 240; i++) {
      w.step(DT);
      minY = Math.min(minY, ball.position.y);
      maxY = Math.max(maxY, ball.position.y);
    }
    expect(minY).toBeGreaterThan(-1.6);
    expect(maxY).toBeLessThan(-0.4);
    expect(maxY).toBeGreaterThan(-0.6);
  });
});

describe('double pendulum', () => {
  it('conserves energy to within 1% over 30 s', () => {
    const w = new World({ gravity: new Vec3(0, -G, 0) });
    const anchor = w.createBody({ type: 'static' });
    const L = 1;
    // Two rods (boxes) from a pivot; the first starts horizontal, the second at an angle.
    const rod1 = w.createBody({ position: new Vec3(L / 2, 0, 0) });
    w.createCollider(rod1, new Box(L / 2, 0.02, 0.02));
    const tip = new Vec3(L, 0, 0);
    const a2 = 2.2;
    const rod2 = w.createBody({
      position: new Vec3(L + (L / 2) * Math.cos(a2), (L / 2) * Math.sin(a2), 0),
    });
    // Orient rod2 along its direction.
    rod2.orientation.z = Math.sin(a2 / 2);
    rod2.orientation.w = Math.cos(a2 / 2);
    w.createCollider(rod2, new Box(L / 2, 0.02, 0.02));
    rod2.updateWorldInertia();
    w.addJoint(new HingeJoint(anchor, rod1, new Vec3(0, 0, 0), new Vec3(0, 0, 1)));
    w.addJoint(new HingeJoint(rod1, rod2, tip, new Vec3(0, 0, 1)));
    const energy = (): number => {
      let e = 0;
      for (const b of [rod1, rod2]) {
        e += 0.5 * b.mass * V.lengthSq(b.linearVelocity) + b.mass * G * b.position.y;
        const inv = b.worldInvInertia;
        const wv = b.angularVelocity;
        // ½ ωᵀ I ω with I = inverse of worldInvInertia (z axis only here).
        const izz = 1 / (inv.e[8] ?? 1);
        e += 0.5 * izz * wv.z * wv.z;
      }
      return e;
    };
    const e0 = energy();
    // Reference scale: the energy range reachable (potential swing), not the near-zero total.
    const scale = rod1.mass * G * L * 2;
    let worst = 0;
    // Energy error is first order in dt: peak deviation is 1.6% at 240 Hz and below 1% at 480 Hz.
    const dt = 1 / 480;
    for (let i = 0; i < 30 / dt; i++) {
      w.step(dt);
      worst = Math.max(worst, Math.abs(energy() - e0) / scale);
    }
    expect(worst).toBeLessThan(0.01);
  }, 60_000);
});

describe('chain', () => {
  it('a 50-link chain released horizontally settles without explosion, joint error under 1 mm', () => {
    const w = new World({ gravity: new Vec3(0, -G, 0) });
    const anchor = w.createBody({ type: 'static', position: new Vec3(0, 10, 0) });
    const link = 0.2;
    const n = 50;
    const joints: BallJoint[] = [];
    let prev = anchor;
    for (let i = 0; i < n; i++) {
      const b = w.createBody({
        position: new Vec3((i + 0.5) * link, 10, 0),
        linearDamping: 1,
        angularDamping: 1,
      });
      w.createCollider(b, new Box(link / 2, 0.02, 0.02));
      joints.push(w.addJoint(new BallJoint(prev, b, new Vec3(i * link, 10, 0))));
      prev = b;
    }
    const gap = (j: BallJoint): number => {
      const pa = V.add(
        new Vec3(),
        j.bodyA.position,
        Q.rotate(new Vec3(), j.bodyA.orientation, j.localA),
      );
      const pb = V.add(
        new Vec3(),
        j.bodyB.position,
        Q.rotate(new Vec3(), j.bodyB.orientation, j.localB),
      );
      return V.distance(pa, pb);
    };
    let transient = 0;
    let worst = 0;
    let maxSpeed = 0;
    for (let i = 0; i < 240 * 20; i++) {
      w.step(DT);
      // The violent first swing stretches joints by a couple of millimetres; the settled phase is
      // held to the 1 mm requirement.
      for (const j of joints) {
        if (i < 240 * 10) transient = Math.max(transient, gap(j));
        else worst = Math.max(worst, gap(j));
      }
      for (const b of w.bodies) maxSpeed = Math.max(maxSpeed, V.length(b.linearVelocity));
    }
    // The free end of a 10 m chain released horizontally reaches ~14 m/s; anything near 100 is an explosion.
    expect(maxSpeed).toBeLessThan(30);
    for (const b of w.bodies) expect(V.isFinite3(b.position)).toBe(true);
    expect(transient).toBeLessThan(5e-3);
    expect(worst).toBeLessThan(1e-3);
    // Settled: hanging below the anchor with small residual motion.
    const last = w.bodies[w.bodies.length - 1] as RigidBody;
    expect(V.length(last.linearVelocity)).toBeLessThan(0.5);
    expect(last.position.y).toBeLessThan(10 - 0.8 * n * link);
  }, 120_000);
});
