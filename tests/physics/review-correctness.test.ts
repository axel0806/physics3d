import { describe, expect, it } from 'vitest';
import { Quat, Transform, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import * as M3 from '../../src/math/mat3';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import type { RigidBody } from '../../src/dynamics/body';

const DT = 1 / 120;

function angularMomentumWorld(b: RigidBody): Vec3 {
  const wb = Q.rotateInverse(new Vec3(), b.orientation, b.angularVelocity);
  const Lb = M3.mulVec(new Vec3(), b.localInertia, wb);
  return Q.rotate(new Vec3(), b.orientation, Lb);
}

function restingBoxOnFloor(w: World, x: number, y: number): RigidBody {
  const box = w.createBody({ position: new Vec3(x, y, 0) });
  w.createCollider(box, new Box(0.5, 0.5, 0.5));
  return box;
}

describe('review: physical correctness', () => {
  it('friction is isotropic: a block on a 33 deg slope (mu 0.5) slides for every azimuth', () => {
    // tan(33 deg) = 0.649 > mu = 0.5, so the block must slide. A square (pyramid) friction
    // cone raises the limit to mu*sqrt(2) = 0.707 along the diagonal of the tangent frame.
    const mu = 0.5;
    const slope = (33 * Math.PI) / 180;
    const speeds: number[] = [];
    for (let deg = 0; deg < 360; deg += 15) {
      const phi = (deg * Math.PI) / 180;
      const w = new World();
      // Slope surface normal: tilt up-axis by `slope` toward azimuth phi.
      const tiltAxis = new Vec3(-Math.sin(phi), 0, Math.cos(phi)); // axis in the xz plane
      const qTilt = Q.fromAxisAngle(new Quat(), tiltAxis, slope);
      const ground = w.createBody({ type: 'static', orientation: qTilt });
      w.createCollider(ground, new Box(20, 0.5, 20), { friction: mu });
      const n = Q.rotate(new Vec3(), qTilt, new Vec3(0, 1, 0));
      const box = w.createBody({
        position: new Vec3(n.x * 1.0, n.y * 1.0, n.z * 1.0),
        orientation: qTilt,
      });
      w.createCollider(box, new Box(0.5, 0.5, 0.5), { friction: mu });
      for (let i = 0; i < 240; i++) w.step(DT);
      // Downhill speed expected ~ g (sin - mu cos) * 1s ~= 1.2 m/s minus startup.
      speeds.push(Math.hypot(box.linearVelocity.x, box.linearVelocity.y, box.linearVelocity.z));
    }
    const min = Math.min(...speeds);
    const max = Math.max(...speeds);
    expect(min).toBeGreaterThan(0.8 * max);
  });

  it('a CCD bullet hitting a sleeping box conserves momentum', () => {
    const w = new World();
    const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
    w.createCollider(floor, new Box(50, 0.5, 50), { friction: 0 });
    const box = w.createBody({ position: new Vec3(0, 0.5, 0) });
    w.createCollider(box, new Box(0.5, 0.5, 0.5), { friction: 0 });
    for (let i = 0; i < 400 && box.awake; i++) w.step(DT);
    expect(box.awake).toBe(false);
    const bullet = w.createBody({
      position: new Vec3(-5, 0.5, 0),
      linearVelocity: new Vec3(100, 0, 0),
      gravityScale: 0,
      ccd: true,
    });
    w.createCollider(bullet, new Sphere(0.1), { friction: 0 });
    const p0 = bullet.mass * 100;
    for (let i = 0; i < 20; i++) w.step(DT);
    const p1 = bullet.mass * bullet.linearVelocity.x + box.mass * box.linearVelocity.x;
    expect(p1).toBeGreaterThan(0.9 * p0);
  });

  it('torque-free slender body spin: world L drift is second order in dt', () => {
    // 60 rad/s at 120 Hz is 0.5 rad per step; a second-order integrator cannot hold L to 1e-6 there.
    // Measured drift over 1 s: 240 Hz 6.1e-3, 480 Hz 1.6e-3, 960 Hz 4.1e-4 (ratio ~3.8, i.e. O(dt^2)).
    const drift = (hz: number): number => {
      const w = new World({ gravity: new Vec3(0, 0, 0) });
      const b = w.createBody({ angularVelocity: new Vec3(0.3, 60, 0.3) });
      w.createCollider(b, new Box(0.1, 0.5, 2));
      const l0 = angularMomentumWorld(b);
      for (let i = 0; i < hz; i++) w.step(1 / hz);
      const l1 = angularMomentumWorld(b);
      return Math.hypot(l1.x - l0.x, l1.y - l0.y, l1.z - l0.z) / Math.hypot(l0.x, l0.y, l0.z);
    };
    const d240 = drift(240);
    const d480 = drift(480);
    const d960 = drift(960);
    expect(d240 / d480).toBeGreaterThan(3);
    expect(d480 / d960).toBeGreaterThan(3);
    expect(d960).toBeLessThan(1e-3);
  });

  it('dropping a ball on a sleeping stack does not sink the bottom box into the floor', () => {
    const w = new World();
    const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
    w.createCollider(floor, new Box(50, 0.5, 50));
    const boxes: RigidBody[] = [];
    for (let i = 0; i < 6; i++) boxes.push(restingBoxOnFloor(w, 0, 0.5 + i * 1.0));
    for (let i = 0; i < 600 && boxes.some((b) => b.awake); i++) w.step(DT);
    expect(boxes.every((b) => !b.awake)).toBe(true);
    const ball = w.createBody({ position: new Vec3(0, 8, 0) });
    w.createCollider(ball, new Sphere(0.2));
    let minBottom = Infinity;
    for (let i = 0; i < 240; i++) {
      w.step(DT);
      minBottom = Math.min(minBottom, (boxes[0] as RigidBody).position.y);
    }
    // Bottom box rests at y = 0.5; allow only the solver slop.
    expect(minBottom).toBeGreaterThan(0.5 - 0.002);
  });

  it('snapshot/restore reproduces a mixed sleeping + ccd + kinematic scene bit for bit', () => {
    const build = (): World => {
      const w = new World({ ccdMode: 'speculative' });
      const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
      w.createCollider(floor, new Box(50, 0.5, 50));
      restingBoxOnFloor(w, 0, 0.5);
      const kin = w.createBody({
        type: 'kinematic',
        position: new Vec3(-3, 0.5, 2),
        linearVelocity: new Vec3(0.5, 0, 0),
        angularVelocity: new Vec3(0, 0.3, 0),
      });
      w.createCollider(kin, new Box(0.5, 0.5, 0.5));
      const bl = w.createBody({
        position: new Vec3(-6, 0.5, 0),
        linearVelocity: new Vec3(60, 0, 0),
        ccd: true,
      });
      w.createCollider(bl, new Sphere(0.1), { restitution: 0.5 });
      return w;
    };
    const w = build();
    for (let i = 0; i < 300; i++) w.step(DT);
    const snap = w.snapshot();
    const hashes: string[] = [];
    for (let i = 0; i < 100; i++) {
      w.step(DT);
      hashes.push(w.stateHash());
    }
    w.restore(snap);
    for (let i = 0; i < 100; i++) {
      w.step(DT);
      expect(w.stateHash()).toBe(hashes[i]);
    }
  });
});
void Transform;
