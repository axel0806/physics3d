import { Vec3 } from '../src/math';
import { Box, Sphere } from '../src/shapes';
import { World } from '../src/world/world';
import { createRagdoll } from '../tests/helpers/ragdoll';
import { heightfield } from '../tests/helpers/terrain';

export interface BenchScene {
  readonly name: string;
  readonly description: string;
  /** Builds a fresh world; every scene is seeded and deterministic. */
  build(): World;
}

/** Small deterministic generator (LCG), local so scenes do not depend on test helpers. */
function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 4294967296;
}

/** A walled square arena of half-size `half`, so piles stay together. */
function arena(world: World, half: number): void {
  const floor = world.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
  world.createCollider(floor, new Box(half + 1, 0.5, half + 1), { friction: 0.6 });
  for (const [x, z, sx, sz] of [
    [-half - 0.5, 0, 0.5, half + 1],
    [half + 0.5, 0, 0.5, half + 1],
    [0, -half - 0.5, half + 1, 0.5],
    [0, half + 0.5, half + 1, 0.5],
  ] as const) {
    const wall = world.createBody({ type: 'static', position: new Vec3(x, 4, z) });
    world.createCollider(wall, new Box(sx, 4.5, sz));
  }
}

export const scenes: BenchScene[] = [
  {
    name: 'pile-1000-boxes',
    description: '1,000 boxes (0.5 m) dropped in ten layers of 100 into a walled arena',
    build() {
      const w = new World({ velocityIterations: 10 });
      arena(w, 4);
      const r = rng(1);
      for (let layer = 0; layer < 10; layer++) {
        for (let i = 0; i < 100; i++) {
          const b = w.createBody({
            position: new Vec3(
              -3.6 + (i % 10) * 0.8 + (r() - 0.5) * 0.1,
              0.4 + layer * 0.75,
              -3.6 + Math.floor(i / 10) * 0.8 + (r() - 0.5) * 0.1,
            ),
          });
          w.createCollider(b, new Box(0.25, 0.25, 0.25), { friction: 0.5 });
        }
      }
      return w;
    },
  },
  {
    name: 'pile-2000-spheres',
    description: '2,000 spheres (0.2 m radius) dropped in ten layers of 200 into a walled arena',
    build() {
      const w = new World();
      arena(w, 4);
      const r = rng(2);
      for (let layer = 0; layer < 10; layer++) {
        for (let i = 0; i < 200; i++) {
          const b = w.createBody({
            position: new Vec3(
              -3.7 + (i % 15) * 0.5 + (r() - 0.5) * 0.05,
              0.3 + layer * 0.45,
              -3.7 + Math.floor(i / 15) * 0.55 + (r() - 0.5) * 0.05,
            ),
          });
          w.createCollider(b, new Sphere(0.2), { friction: 0.4 });
        }
      }
      return w;
    },
  },
  {
    name: 'ragdolls-100',
    description: '100 ten-body ragdolls (1,000 bodies, 900 joints) dropped on a floor',
    build() {
      const w = new World();
      arena(w, 9);
      const r = rng(3);
      for (let i = 0; i < 100; i++) {
        createRagdoll(
          w,
          new Vec3(
            -8 + (i % 10) * 1.8,
            1.2 + Math.floor(i / 10) * 0.2,
            -8 + Math.floor(i / 10) * 1.8,
          ),
        );
        void r;
      }
      return w;
    },
  },
  {
    name: 'stacks-30x30',
    description: 'a 30 x 30 grid of four-box stacks (3,600 boxes) at rest',
    build() {
      const w = new World();
      const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
      w.createCollider(floor, new Box(25, 0.5, 25), { friction: 0.7 });
      for (let i = 0; i < 30; i++) {
        for (let j = 0; j < 30; j++) {
          for (let k = 0; k < 4; k++) {
            const b = w.createBody({
              position: new Vec3(-24 + i * 1.65, 0.2 + k * 0.4, -24 + j * 1.65),
            });
            w.createCollider(b, new Box(0.2, 0.2, 0.2), { friction: 0.7 });
          }
        }
      }
      return w;
    },
  },
  {
    name: 'terrain-500',
    description: 'a static 100k-triangle heightfield with 500 boxes and spheres dropped on it',
    build() {
      const w = new World();
      const g = w.createBody({ type: 'static' });
      const height = (x: number, z: number) =>
        0.5 * Math.sin(x * 0.3) * Math.cos(z * 0.25) + 0.01 * (x * x + z * z);
      w.createCollider(g, heightfield(224, 40, height), { friction: 0.7 });
      const r = rng(4);
      for (let i = 0; i < 500; i++) {
        const b = w.createBody({
          position: new Vec3((r() - 0.5) * 24, 3 + r() * 6, (r() - 0.5) * 24),
        });
        w.createCollider(b, i % 2 ? new Sphere(0.3) : new Box(0.3, 0.3, 0.3), { friction: 0.7 });
      }
      return w;
    },
  },
];
