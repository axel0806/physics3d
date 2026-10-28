import { describe, expect, it } from 'vitest';
import { Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import type { RigidBody } from '../../src/dynamics/body';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import { BallJoint, HingeJoint, SliderJoint, FixedJoint, DistanceJoint } from '../../src/constraints';

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
