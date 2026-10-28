import { describe, expect, it } from 'vitest';
import { Quat, Transform, Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import * as Q from '../../src/math/quat';
import { Box } from '../../src/shapes';
import { World } from '../../src/world/world';
import type { BallJoint } from '../../src/constraints';
import { anchorGap, createRagdoll } from '../helpers/ragdoll';
import { mulberry32 } from '../helpers/rng';

const DROPS = Number(process.env.RAGDOLL_DROPS ?? 40);

/** Staircase descending in +x: 8 steps of 0.5 m depth and 0.2 m rise, over a floor. */
function stairs(world: World): void {
  const floor = world.createBody({ type: 'static' });
  world.createCollider(floor, new Box(20, 0.5, 6), { local: new Transform(new Vec3(5, -0.5, 0)) });
  const steps = 8;
  for (let i = 0; i < steps; i++) {
    const top = (steps - i) * 0.2;
    const s = world.createBody({ type: 'static' });
    world.createCollider(s, new Box(0.25, top / 2, 2), {
      local: new Transform(new Vec3(0.25 + i * 0.5, top / 2, 0)),
    });
  }
}

describe('ragdoll', () => {
  it(
    `survives ${DROPS} randomized drops down a staircase without NaNs, explosions or limb separation`,
    { timeout: 3_600_000 },
    () => {
      const rng = mulberry32(2024);
      let worstGap = 0;
      let worstSpeed = 0;
      for (let drop = 0; drop < DROPS; drop++) {
        const w = new World({ gravity: new Vec3(0, -9.81, 0), velocityIterations: 20 });
        stairs(w);
        const axis = V.normalize(new Vec3(), new Vec3(rng() - 0.5, rng() - 0.5, rng() - 0.5));
        const q = Q.fromAxisAngle(new Quat(), axis, rng() * Math.PI * 2);
        const origin = new Vec3(0.2 + rng() * 1.5, 3 + rng(), (rng() - 0.5) * 0.6);
        const doll = createRagdoll(w, origin, q);
        for (const b of doll.bodies) {
          b.linearVelocity.x = rng() * 2;
          b.angularVelocity.x = (rng() - 0.5) * 6;
          b.angularVelocity.y = (rng() - 0.5) * 6;
          b.angularVelocity.z = (rng() - 0.5) * 6;
        }
        for (let i = 0; i < 120 * 4; i++) {
          w.step(1 / 120);
          for (const b of doll.bodies) {
            if (!V.isFinite3(b.position) || !V.isFinite3(b.linearVelocity)) {
              throw new Error(`non-finite state in drop ${drop} at step ${i}`);
            }
            worstSpeed = Math.max(worstSpeed, V.length(b.linearVelocity));
          }
          for (const j of doll.joints) {
            worstGap = Math.max(worstGap, anchorGap(j as BallJoint));
          }
        }
      }
      expect(worstSpeed).toBeLessThan(25);
      console.log(
        `ragdoll: ${DROPS} drops, worst joint gap ${worstGap.toFixed(4)} m, worst speed ${worstSpeed.toFixed(1)} m/s`,
      );
      expect(worstGap).toBeLessThan(0.05);
    },
  );
});
