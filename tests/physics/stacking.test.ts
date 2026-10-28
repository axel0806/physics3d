import { describe, expect, it } from 'vitest';
import { Transform, Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as Q from '../../src/math/quat';
import type { RigidBody } from '../../src/dynamics/body';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';

const G = 9.81;

function floorWorld(): { w: World } {
  const w = new World({ gravity: new Vec3(0, -G, 0) });
  const floor = w.createBody({ type: 'static' });
  w.createCollider(floor, new Box(50, 0.5, 50), {
    local: new Transform(new Vec3(0, -0.5, 0)),
    friction: 0.6,
    restitution: 0,
  });
  return { w };
}

describe('stacking', () => {
  it('a 20-box tower stands for 10 s with drift under 1 cm and goes to sleep', () => {
    const { w } = floorWorld();
    const boxes = [];
    for (let i = 0; i < 20; i++) {
      const b = w.createBody({ position: new Vec3(0, 0.5 + i, 0) });
      w.createCollider(b, new Box(0.5, 0.5, 0.5), { friction: 0.6, restitution: 0 });
      boxes.push(b);
    }
    const start = boxes.map((b) => b.position.clone());
    const dt = 1 / 120;
    for (let i = 0; i < 10 * 120; i++) w.step(dt);
    let drift = 0;
    boxes.forEach((b, i) => {
      drift = Math.max(drift, b.position.sub(start[i] as Vec3).length());
    });
    expect(drift).toBeLessThan(0.01);
    expect(boxes.every((b) => !b.awake)).toBe(true);
  }, 600_000);

  it('a 10-level pyramid of 55 boxes stays stable for 10 s', () => {
    const { w } = floorWorld();
    const boxes: { body: ReturnType<World['createBody']>; start: Vec3 }[] = [];
    const levels = 10;
    for (let level = 0; level < levels; level++) {
      const n = levels - level;
      for (let i = 0; i < n; i++) {
        const x = (i - (n - 1) / 2) * 1.0;
        const y = 0.5 + level * 1.0;
        const body = w.createBody({ position: new Vec3(x, y, 0) });
        w.createCollider(body, new Box(0.5, 0.5, 0.5), { friction: 0.6, restitution: 0 });
        boxes.push({ body, start: new Vec3(x, y, 0) });
      }
    }
    expect(boxes.length).toBe(55);
    const dt = 1 / 120;
    for (let i = 0; i < 10 * 120; i++) w.step(dt);
    let drift = 0;
    for (const { body, start } of boxes) drift = Math.max(drift, body.position.sub(start).length());
    expect(drift).toBeLessThan(0.1);
  }, 600_000);
});

describe("Newton's cradle", () => {
  it('five touching spheres: momentum passes through the chain and only the end ball leaves', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const balls = [];
    for (let i = 0; i < 5; i++) {
      const b = w.createBody({
        // 1 mm gaps: impacts then occur one after another, each detected in its own step.
        position: new Vec3(2.001 * i, 0, 0),
        linearVelocity: i === 0 ? new Vec3(2, 0, 0) : new Vec3(),
      });
      w.createCollider(b, new Sphere(1), { restitution: 1, friction: 0 });
      balls.push(b);
    }
    // Discrete impacts need a fine step: each pair is detected before it has overlapped by more than v·dt.
    for (let i = 0; i < 4800; i++) w.step(1 / 2400);
    const v = balls.map((b) => b.linearVelocity.x);
    // The first ball stops and the last ball carries the speed away. The impact speed (2 m/s) is kept
    // above the restitution threshold (1 m/s) so the bounce term applies to each impact.
    expect(Math.abs((v[0] ?? 1) - 0)).toBeLessThan(0.02);
    expect(Math.abs((v[4] ?? 0) - 2)).toBeLessThan(0.02);
    for (const i of [1, 2, 3]) expect(Math.abs(v[i] ?? 1)).toBeLessThan(0.02);
  });
});

/** Body-frame angular velocity: the world vector rotated into the body's principal frame. */
function bodyFrame(body: RigidBody): { vec: Vec3; get: (k: 0 | 1 | 2) => number } {
  const vec = Q.rotateInverse(new Vec3(), body.orientation, body.angularVelocity);
  return { vec, get: (k) => (k === 0 ? vec.x : k === 1 ? vec.y : vec.z) };
}

describe('Dzhanibekov effect', () => {
  /**
   * Spins a T-shaped body about one principal axis and reports the body-frame behaviour: sign flips
   * of the spin component along that axis, and the largest misalignment of the body-frame angular
   * velocity from the axis. Body-frame quantities are the meaningful ones: world components mix
   * with the rotating orientation.
   */
  function spinAbout(axisIndex: 0 | 1 | 2): { flips: number; maxMisalignment: number } {
    const w = new World({ gravity: new Vec3() });
    const body = w.createBody();
    // Bar along x, and a stem along z touching it, so the two boxes do not overlap.
    w.createCollider(body, new Box(2, 0.25, 0.25));
    w.createCollider(body, new Box(0.25, 0.25, 1.25), {
      local: new Transform(new Vec3(0, 0, 1.5)),
    });
    const spin = [0.02, 0.02, 0.02];
    spin[axisIndex] = 3;
    body.angularVelocity.x = spin[0] as number;
    body.angularVelocity.y = spin[1] as number;
    body.angularVelocity.z = spin[2] as number;
    let flips = 0;
    let maxMisalignment = 0;
    let prev = Math.sign(bodyFrame(body).get(axisIndex));
    const dt = 1 / 480;
    for (let i = 0; i < 40 * 480; i++) {
      w.step(dt);
      const wb = bodyFrame(body);
      const along = wb.get(axisIndex);
      const s = Math.sign(along);
      if (s !== prev) flips++;
      prev = s;
      const cos = Math.abs(along) / V.length(wb.vec);
      maxMisalignment = Math.max(maxMisalignment, 1 - cos);
    }
    return { flips, maxMisalignment };
  }

  it('the intermediate principal axis flips periodically; the min and max axes stay stable', () => {
    // Principal moments of the T body, from a probe world.
    const probe = new World({ gravity: new Vec3() });
    const body = probe.createBody();
    probe.createCollider(body, new Box(2, 0.25, 0.25));
    probe.createCollider(body, new Box(0.25, 0.25, 1.25), {
      local: new Transform(new Vec3(0, 0, 1.5)),
    });
    const I = [0, 1, 2].map((k) => body.localInertia.e[4 * k] as number);
    // Diagonal entries of the inertia tensor are the moments about x, y, z; the T's products are zero.
    const order = [0, 1, 2].sort((a, b) => (I[a] as number) - (I[b] as number));
    const [minAxis, midAxis, maxAxis] = order as [0 | 1 | 2, 0 | 1 | 2, 0 | 1 | 2];
    expect(Math.abs((I[midAxis] as number) - (I[minAxis] as number))).toBeGreaterThan(0.05);
    const mid = spinAbout(midAxis);
    expect(mid.flips).toBeGreaterThan(0);
    expect(spinAbout(minAxis).maxMisalignment).toBeLessThan(0.05);
    expect(spinAbout(maxAxis).maxMisalignment).toBeLessThan(0.05);
  }, 600_000);
});
