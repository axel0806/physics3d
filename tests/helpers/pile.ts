import { Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import { Box } from '../../src/shapes';
import { World } from '../../src/world/world';
import { mulberry32 } from './rng';

/** Randomly oriented boxes dropped into a walled 9 x 9 m arena. */
export function buildPile(seed: number, count: number, span = 8): World {
  const w = new World();
  const half = 4.5;
  const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
  w.createCollider(floor, new Box(half + 1, 0.5, half + 1), { friction: 0.6 });
  for (const [x, z, sx, sz] of [
    [-half - 0.5, 0, 0.5, half + 1],
    [half + 0.5, 0, 0.5, half + 1],
    [0, -half - 0.5, half + 1, 0.5],
    [0, half + 0.5, half + 1, 0.5],
  ] as const) {
    const wall = w.createBody({ type: 'static', position: new Vec3(x, 4, z) });
    w.createCollider(wall, new Box(sx, 4.5, sz));
  }
  const r = mulberry32(seed);
  for (let i = 0; i < count; i++) {
    const q = Q.normalize(new Q.Quat(), new Q.Quat(r() - 0.5, r() - 0.5, r() - 0.5, r() + 0.1));
    const b = w.createBody({
      position: new Vec3((r() - 0.5) * span, 0.5 + (i % 8) * 0.7 + r() * 0.2, (r() - 0.5) * span),
      orientation: q,
    });
    w.createCollider(b, new Box(0.2 + r() * 0.15, 0.2 + r() * 0.15, 0.2 + r() * 0.15), {
      friction: 0.5,
    });
  }
  return w;
}
