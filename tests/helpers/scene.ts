import { Quat, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import * as V from '../../src/math/vec3';
import { Box, Capsule, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import { BallJoint, HingeJoint } from '../../src/constraints';
import { mulberry32 } from './rng';

/**
 * A seeded random scene: a floor with walls, a pile of mixed shapes dropped from random poses and
 * velocities, a few jointed pairs, and one fast CCD projectile. Same seed, same scene, every time.
 */
export function randomScene(seed: number, bodies = 14): World {
  const rand = mulberry32(seed);
  const world = new World({ ccdMode: seed % 5 === 4 ? 'speculative' : 'sweep' });
  const floor = world.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
  world.createCollider(floor, new Box(8, 0.5, 8), { friction: 0.6 });
  for (const [x, z] of [
    [-8, 0],
    [8, 0],
    [0, -8],
    [0, 8],
  ] as const) {
    const wall = world.createBody({ type: 'static', position: new Vec3(x, 1, z) });
    world.createCollider(wall, x === 0 ? new Box(8, 1.5, 0.5) : new Box(0.5, 1.5, 8));
  }
  const made = [];
  for (let i = 0; i < bodies; i++) {
    const kind = Math.floor(rand() * 3);
    const q = Q.normalize(
      new Quat(),
      new Quat(rand() - 0.5, rand() - 0.5, rand() - 0.5, rand() + 0.1),
    );
    const body = world.createBody({
      position: new Vec3((rand() - 0.5) * 8, 0.5 + i * 0.7 + rand(), (rand() - 0.5) * 8),
      orientation: q,
      linearVelocity: new Vec3((rand() - 0.5) * 4, 0, (rand() - 0.5) * 4),
      angularVelocity: new Vec3((rand() - 0.5) * 3, (rand() - 0.5) * 3, (rand() - 0.5) * 3),
    });
    const shape =
      kind === 0
        ? new Sphere(0.2 + rand() * 0.2)
        : kind === 1
          ? new Box(0.2 + rand() * 0.2, 0.2 + rand() * 0.2, 0.2 + rand() * 0.2)
          : new Capsule(0.15 + rand() * 0.1, 0.2 + rand() * 0.2);
    world.createCollider(body, shape, { friction: 0.3 + rand() * 0.5, restitution: rand() * 0.4 });
    made.push(body);
  }
  for (let i = 0; i + 1 < made.length && i < 6; i += 2) {
    const a = made[i]!;
    const b = made[i + 1]!;
    const mid = V.lerp(new Vec3(), a.position, b.position, 0.5);
    world.addJoint(
      i % 4 === 0
        ? new BallJoint(a, b, mid)
        : new HingeJoint(a, b, mid, new Vec3(0, 1, 0), { lower: -1, upper: 1 }),
    );
  }
  const bullet = world.createBody({
    position: new Vec3(-7, 1, 0),
    linearVelocity: new Vec3(120, 2, (rand() - 0.5) * 10),
    ccd: true,
  });
  world.createCollider(bullet, new Sphere(0.05));
  return world;
}
