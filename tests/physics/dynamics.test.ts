import { describe, expect, it } from 'vitest';
import { Mat3, Quat, Transform, Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as M3 from '../../src/math/mat3';
import type { RigidBody } from '../../src/dynamics/body';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import { mulberry32 } from '../helpers/rng';

const G = 9.81;

/** Orbital angular momentum m·(x × v) about the origin. */
function orbital(body: RigidBody): Vec3 {
  return body.position.cross(body.linearVelocity).scale(body.mass);
}

/** Spin angular momentum I_world·ω about the body's center of mass. */
function spinAngularMomentum(body: RigidBody): Vec3 {
  const inv = new Mat3();
  M3.invert(inv, body.worldInvInertia);
  return M3.mulVec(new Vec3(), inv, body.angularVelocity);
}

describe('free fall', () => {
  it('position after t matches ½gt² within the integrator’s O(dt) error', () => {
    // Energy drift is first order in dt (4.9e-3 at 240 Hz, 6.1e-4 at 1920 Hz).
    const dt = 1 / 1920;
    const w = new World({ gravity: new Vec3(0, -G, 0) });
    const b = w.createBody({ position: new Vec3(0, 100, 0) });
    w.createCollider(b, new Sphere(0.5));
    const steps = 240; // 1 s
    for (let i = 0; i < steps; i++) w.step(dt);
    const t = steps * dt;
    const analytic = 100 - 0.5 * G * t * t;
    // Symplectic Euler on constant acceleration: error = ½·g·dt·t.
    expect(Math.abs(b.position.y - analytic)).toBeLessThan(0.5 * G * dt * t + 1e-9);
  });
});

describe('restitution', () => {
  it('bounce height ratio is e² within 2% for e ∈ {0.2, 0.5, 0.8}', () => {
    for (const e of [0.2, 0.5, 0.8]) {
      // Contacts are found after the step has moved the ball by v·dt, so the apex error is O(v·dt/h).
      const dt = 1 / 1920;
      const w = new World({ gravity: new Vec3(0, -G, 0), linearSlop: 0.001 });
      const floor = w.createBody({ type: 'static' });
      w.createCollider(floor, new Box(50, 0.5, 50), {
        local: new Transform(new Vec3(0, -0.5, 0)),
        restitution: e,
      });
      const ball = w.createBody({ position: new Vec3(0, 5, 0) });
      w.createCollider(ball, new Sphere(0.5), { restitution: e, friction: 0 });
      // Drop height of the center above its resting height (contact at y = 0.5).
      const dropHeight = 5 - 0.5;
      let bounced = false;
      let apexAboveRest = -1;
      let prev = ball.linearVelocity.y;
      for (let i = 0; i < 40000 && apexAboveRest < 0; i++) {
        w.step(dt);
        const vy = ball.linearVelocity.y;
        if (!bounced && prev < 0 && vy > 0) bounced = true;
        if (bounced && prev > 0 && vy <= 0) apexAboveRest = ball.position.y - 0.5;
        prev = vy;
      }
      expect(bounced).toBe(true);
      expect(Math.abs(apexAboveRest / dropHeight - e * e) / (e * e)).toBeLessThan(0.02);
    }
  }, 600_000);
});

describe('conservation', () => {
  it('bouncing ball keeps total energy within 0.1% over 60 s (e = 1)', () => {
    // Energy drift is first order in dt (4.9e-3 at 240 Hz, 6.1e-4 at 1920 Hz).
    const dt = 1 / 1920;
    const w = new World({ gravity: new Vec3(0, -G, 0), linearSlop: 0.001 });
    const floor = w.createBody({ type: 'static' });
    w.createCollider(floor, new Box(50, 0.5, 50), {
      local: new Transform(new Vec3(0, -0.5, 0)),
      restitution: 1,
    });
    const ball = w.createBody({ position: new Vec3(0, 3, 0) });
    w.createCollider(ball, new Sphere(0.5), { restitution: 1, friction: 0 });
    const energy = () => 0.5 * V.lengthSq(ball.linearVelocity) + G * ball.position.y;
    const e0 = energy();
    let worst = 0;
    for (let i = 0; i < 60 * 1920; i++) {
      w.step(dt);
      worst = Math.max(worst, Math.abs(energy() - e0) / e0);
    }
    expect(worst).toBeLessThan(1e-3);
  }, 600_000);

  it('two-body collisions conserve linear momentum to 1e-9 and angular momentum about the origin to 1e-6', () => {
    const r = mulberry32(77);
    let worstLin = 0;
    let worstAng = 0;
    for (let trial = 0; trial < 1000; trial++) {
      // Position correction moves bodies by design, which changes Σ x×mv; the conservation check
      // therefore runs with it disabled. Velocity-level impulses alone must conserve both quantities.
      const w = new World({ gravity: new Vec3(), velocityIterations: 20, positionIterations: 0 });
      const a = w.createBody({
        position: new Vec3(-1 + 0.2 * r(), 0.1 * r(), 0.1 * r()),
        linearVelocity: new Vec3(1 + r(), r() - 0.5, 0.2 * r()),
        angularVelocity: new Vec3(0.3 * r(), 0.3 * r(), 0.3 * r()),
      });
      const b = w.createBody({
        position: new Vec3(1 + 0.2 * r(), 0.1 * r(), 0.1 * r()),
        linearVelocity: new Vec3(-1 - r(), r() - 0.5, 0.2 * r()),
        angularVelocity: new Vec3(0.3 * r(), 0.3 * r(), 0.3 * r()),
      });
      w.createCollider(a, new Box(0.4, 0.3, 0.5), { restitution: 0.5, friction: 0.3 });
      w.createCollider(b, new Box(0.5, 0.4, 0.3), { restitution: 0.5, friction: 0.3 });
      const momentum = () => {
        // Mass-weighted: the two boxes have different masses.
        return a.linearVelocity.scale(a.mass).add(b.linearVelocity.scale(b.mass));
      };
      const angular = () =>
        spinAngularMomentum(a).add(orbital(a)).add(spinAngularMomentum(b)).add(orbital(b));
      const p0 = momentum();
      const l0 = angular();
      for (let i = 0; i < 40; i++) w.step(1 / 120);
      const p1 = momentum();
      const l1 = angular();
      const scaleP = Math.max(1, p0.length());
      worstLin = Math.max(worstLin, p1.sub(p0).length() / scaleP);
      worstAng = Math.max(worstAng, l1.sub(l0).length() / Math.max(1, l0.length()));
    }
    expect(worstLin).toBeLessThan(1e-9);
    expect(worstAng).toBeLessThan(1e-6);
  }, 600_000);
});

describe('friction ramp', () => {
  it('a box slides on an incline if and only if tan θ > μ, for 20 angles', () => {
    const mu = 0.4;
    for (let k = 1; k <= 20; k++) {
      const theta = (k / 21) * (Math.PI / 2 - 0.05);
      const expectSlide = Math.tan(theta) > mu;
      const w = new World({ gravity: new Vec3(0, -G, 0) });
      const ramp = w.createBody({
        type: 'static',
        orientation: new Quat(0, 0, Math.sin(-theta / 2), Math.cos(-theta / 2)),
      });
      w.createCollider(ramp, new Box(20, 0.5, 20), {
        local: new Transform(new Vec3(0, -0.5, 0)),
        friction: mu,
        restitution: 0,
      });
      // The ramp's top normal is (sin θ, cos θ, 0). The box shares the ramp's orientation, so its
      // half-extent along that normal is 0.5; place it 0.02 above the surface.
      const along = 0.5 + 0.02;
      const start = new Vec3(along * Math.sin(theta), along * Math.cos(theta), 0);
      const box = w.createBody({
        position: start.clone(),
        orientation: new Quat(0, 0, Math.sin(-theta / 2), Math.cos(-theta / 2)),
      });
      w.createCollider(box, new Box(0.5, 0.5, 0.5), { friction: mu, restitution: 0 });
      for (let i = 0; i < 480; i++) w.step(1 / 120);
      // Displacement from the placement point (the resting height itself is not motion).
      const slid = box.position.sub(start).length() > 0.2 || box.linearVelocity.length() > 0.05;
      expect(slid, `theta=${((theta * 180) / Math.PI).toFixed(2)}°`).toBe(expectSlide);
    }
  }, 600_000);
});

describe('gyroscopic integration at large dt·|ω|', () => {
  it('stays bounded for a fast-spinning elongated body (the fixed point alone diverges)', () => {
    // dt·|ω| is about 4 on a long thin box: the implicit-midpoint fixed-point iteration diverges here,
    // and used to blow the angular velocity up to the thousands of rad/s (ragdoll heap, 4 mm margin).
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const b = w.createBody({ angularVelocity: new Vec3(200, 140, 100) });
    w.createCollider(b, new Box(0.6, 0.05, 0.05));
    const energy = (): number => {
      const L = spinAngularMomentum(b);
      return 0.5 * L.dot(b.angularVelocity);
    };
    const e0 = energy();
    const speed0 = b.angularVelocity.length();
    for (let i = 0; i < 600; i++) {
      w.step(1 / 60);
      expect(b.angularVelocity.length()).toBeLessThan(1.5 * speed0);
    }
    expect(Number.isFinite(energy())).toBe(true);
    expect(energy()).toBeLessThan(2 * e0);
  });
});
