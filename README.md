# physics3d

A from-scratch 3D rigid-body physics engine in TypeScript with no runtime dependencies, plus a raw
WebGL2 renderer and demo.

- Shapes: sphere, box, capsule, convex hull, static triangle mesh (BVH). A body with several colliders acts as a compound.
- Collision: dynamic AABB tree broadphase, GJK/EPA with analytic fast paths, box–box SAT manifolds.
- Dynamics: sequential impulses with warm starting, block normal solve, friction cones, NGS position
  correction, sleeping islands, gyroscopic integration.
- Joints: ball, hinge (limits, motor), slider, fixed, distance/spring, cone-twist.
- Continuous collision detection (sweep time of impact, or speculative contacts).
- World API: fixed-step accumulator with interpolation, ray/shape/overlap queries, layers, contact
  events, snapshot/restore, bit-exact state hashing (deterministic).

## Quick start

```bash
npx pnpm@9 install
npx pnpm@9 test            # unit, property, physics and determinism tests
npx pnpm@9 dev             # demo at http://localhost:5173
npx pnpm@9 bench           # benchmark scenes
```

```ts
import { World } from './src/world/world';
import { Box, Sphere } from './src/shapes';
import { Vec3 } from './src/math';

const world = new World();
const floor = world.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
world.createCollider(floor, new Box(10, 0.5, 10));
const ball = world.createBody({ position: new Vec3(0, 4, 0) });
world.createCollider(ball, new Sphere(0.5), { restitution: 0.5 });
for (let i = 0; i < 240; i++) world.step(1 / 120);
```

See [docs/API.md](docs/API.md), [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md),
[docs/VALIDATION.md](docs/VALIDATION.md) and [PROGRESS.md](PROGRESS.md) (status, benchmark numbers,
known limitations).

## Demo scenes

Validation: bounce, Newton's cradle, Dzhanibekov flip, friction ramp, 20-box tower, 55-box pyramid,
120° pendulum, double pendulum, 50-link chain, ragdoll pile, 500 m/s bullet vs a 1 cm wall,
terrain mesh, 1,000-box pile. Playground: Jenga, domino spiral, wrecking ball, vehicle, ragdoll
cannon, click to throw. Controls: drag to orbit, right-drag or shift-drag to pan, wheel to zoom,
`F` for fly mode (WASD, Q/E), click to throw (shift-click selects a body for the inspector).
Screenshots are in `docs/screens`.

## Scripts

`test`, `test:physics`, `test:ragdoll` (500 drops), `test:e2e` (headless Chromium smoke test over every
scene), `bench`, `bench:quick`, `bench:check`, `bench:alloc`, `check` (tsc, eslint, prettier), `verify`.

## Status

All phases in [PLAN.md](PLAN.md) are implemented and verified except one criterion: the 1,000-box pile
steps in 17 ms median, not the planned 8 ms. See [PROGRESS.md](PROGRESS.md) for numbers, known
limitations and the suggested next steps. On Windows run `test:ragdoll` and `bench:accuracy` from Git Bash
(they use POSIX env syntax). The demo needs a browser with WebGL2 and hardware acceleration enabled.
