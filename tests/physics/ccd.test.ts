import { describe, expect, it } from 'vitest';
import { Quat, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import * as V from '../../src/math/vec3';
import { Box, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import { mulberry32 } from '../helpers/rng';

const DT = 1 / 60;

/**
 * Fires a 5 cm sphere at 500 m/s at a 1 cm thick wall (randomly tilted, random aim point and
 * approach angle) and counts shots that end up on the far side.
 */
function fire(
  shots: number,
  ccd: boolean,
  seed: number,
  ccdMode: 'sweep' | 'speculative' = 'sweep',
): number {
  const rand = mulberry32(seed);
  let tunnelled = 0;
  for (let i = 0; i < shots; i++) {
    const w = new World({ gravity: new Vec3(0, 0, 0), ccdMode });
    const tilt = Q.fromAxisAngle(
      new Quat(),
      V.normalize(new Vec3(), new Vec3(rand() - 0.5, rand() - 0.5, rand() - 0.5)),
      (rand() - 0.5) * 2 * (Math.PI / 3),
    );
    const wall = w.createBody({ type: 'static', orientation: tilt });
    w.createCollider(wall, new Box(2, 2, 0.005));
    const n = Q.rotate(new Vec3(), tilt, new Vec3(0, 0, 1));
    const u = Q.rotate(new Vec3(), tilt, new Vec3(1, 0, 0));
    const v = Q.rotate(new Vec3(), tilt, new Vec3(0, 1, 0));
    // Aim point on the wall, start 1-6 m in front, direction within 60 degrees of the normal.
    const aim = V.add(
      new Vec3(),
      V.scale(new Vec3(), u, (rand() - 0.5) * 3),
      V.scale(new Vec3(), v, (rand() - 0.5) * 3),
    );
    const side = rand() < 0.5 ? 1 : -1;
    const angle = rand() * (Math.PI / 3);
    const az = rand() * 2 * Math.PI;
    const dir = V.add(
      new Vec3(),
      V.scale(new Vec3(), n, -side * Math.cos(angle)),
      V.add(
        new Vec3(),
        V.scale(new Vec3(), u, Math.sin(angle) * Math.cos(az)),
        V.scale(new Vec3(), v, Math.sin(angle) * Math.sin(az)),
      ),
    );
    const dist = 1 + rand() * 5;
    const start = V.addScaled(new Vec3(), aim, dir, -dist);
    const ball = w.createBody({
      position: start,
      linearVelocity: V.scale(new Vec3(), dir, 500),
      ccd,
    });
    w.createCollider(ball, new Sphere(0.05));
    const side0 = Math.sign(V.dot(V.sub(new Vec3(), ball.position, wall.position), n));
    let crossed = false;
    for (let s = 0; s < 12 && !crossed; s++) {
      w.step(DT);
      const sd = V.dot(V.sub(new Vec3(), ball.position, wall.position), n);
      // Past the wall plane inside the wall footprint = tunnelled.
      const lu = V.dot(V.sub(new Vec3(), ball.position, wall.position), u);
      const lv = V.dot(V.sub(new Vec3(), ball.position, wall.position), v);
      if (Math.sign(sd) !== side0 && Math.abs(lu) < 2 && Math.abs(lv) < 2) crossed = true;
    }
    if (crossed) tunnelled++;
  }
  return tunnelled;
}

describe('continuous collision detection', () => {
  it('without CCD the fast sphere tunnels (the test has teeth)', () => {
    expect(fire(200, false, 1)).toBeGreaterThan(50);
  });

  it('a 500 m/s sphere never tunnels a 1 cm wall over 10,000 randomized shots at 60 Hz', () => {
    expect(fire(10_000, true, 2)).toBe(0);
  }, 600_000);

  it('speculative contacts cut tunnelling by about 8x but do not eliminate it (2,000 shots)', () => {
    // Measured 111 of 2,000 (5.5%); the discrete solver loses ~46%. The misses are oblique shots
    // whose closest feature at the start of an 8 m step is the wall's edge, not its face.
    const n = fire(2000, true, 3, 'speculative');
    expect(n).toBeLessThan(160);
  }, 300_000);

  it('a CCD ball dropped on a floor bounces and comes to rest with no time loss', () => {
    const run = (ccd: boolean): { y: number; t: number } => {
      const w = new World();
      const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
      w.createCollider(floor, new Box(10, 0.5, 10), { restitution: 0.5 });
      const ball = w.createBody({ position: new Vec3(0, 3, 0), ccd });
      w.createCollider(ball, new Sphere(0.25), { restitution: 0.5 });
      let t = 0;
      while (t < 10 && ball.awake) {
        w.step(DT);
        t += DT;
      }
      return { y: ball.position.y, t };
    };
    const a = run(true);
    const b = run(false);
    expect(a.y).toBeGreaterThan(0.24);
    expect(a.y).toBeLessThan(0.26);
    // Same settling time as the discrete solver (within one sleep interval of 0.5 s).
    expect(Math.abs(a.t - b.t)).toBeLessThan(0.5);
  });

  it('a slow CCD sphere is not swept at all (matches the discrete result exactly)', () => {
    const run = (ccd: boolean): number => {
      const w = new World();
      const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
      w.createCollider(floor, new Box(10, 0.5, 10));
      const ball = w.createBody({ position: new Vec3(0, 0.3, 0), ccd });
      w.createCollider(ball, new Sphere(0.25));
      for (let i = 0; i < 120; i++) w.step(DT);
      return ball.position.y;
    };
    expect(run(true)).toBe(run(false));
  });
});
