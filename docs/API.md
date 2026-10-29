# API

Everything is exported from the module files under `src/`; the main entry is `World`
(`src/world/world.ts`). Units are SI (metres, kilograms, seconds, radians). The engine has no
runtime dependencies and uses no randomness or clocks, so a scene with the same inputs always
produces the same bits.

## Quick start

```ts
import { Vec3 } from './src/math';
import { Box, Sphere } from './src/shapes';
import { World } from './src/world/world';

const world = new World({ gravity: new Vec3(0, -9.81, 0) });

const floor = world.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
world.createCollider(floor, new Box(20, 0.5, 20), { friction: 0.6 });

const ball = world.createBody({ position: new Vec3(0, 5, 0) });
world.createCollider(ball, new Sphere(0.5), { restitution: 0.4 });

for (let i = 0; i < 600; i++) world.step(1 / 120);
console.log(ball.position.y);
```

## Bodies and colliders

`world.createBody({ type, position, orientation, linearVelocity, angularVelocity, linearDamping,
angularDamping, gravityScale, ccd })` with `type` `'dynamic'` (default), `'static'` or
`'kinematic'`. Mass, centre of mass and inertia come from the colliders (density 1); a body may
have several colliders (a compound). `createCollider(body, shape, { local, friction, restitution,
layer, mask })` where `local` is the collider frame relative to the body.

Shapes: `Sphere(r)`, `Box(hx, hy, hz)` (half extents), `Capsule(r, halfHeight)` (along y),
`ConvexHull(points)` (quickhull), and `TriangleMesh(vertices, indices)` (static bodies only,
BVH-accelerated). `world.destroyCollider(c)` and `world.destroyBody(b)` remove things, wake what
they touched and emit `end` events.

Filtering: two colliders collide when `a.mask & b.layer` and `b.mask & a.layer` are non-zero.

## Stepping

```ts
world.step(dt);                       // one fixed step
const alpha = world.advance(frameTime); // accumulator: runs whole world.fixedDt steps, returns 0..1
const pose = world.interpolatedTransform(body, alpha); // blend previous and current pose for drawing
```

`fixedDt` (default 1/120), `maxSubSteps` (default 8, extra time is dropped). Quality knobs:
`velocityIterations` (40), `positionIterations` (10, joints), `contactPositionIterations` (4),
`baumgarte`, `linearSlop`, `restitutionThreshold`, `sleepTimeLimit`. Worlds without joints run the
contact iterations on flat arrays and stop early once impulses stop changing.

## Joints

```ts
import { BallJoint, HingeJoint, SliderJoint, FixedJoint, DistanceJoint, ConeTwistJoint } from './src/constraints';

world.addJoint(new HingeJoint(a, b, anchor, axis, { lower: -1, upper: 1, motorSpeed: 3, maxMotorTorque: 50 }));
world.addJoint(new DistanceJoint(a, b, anchorA, anchorB, { frequency: 3, dampingRatio: 0.3 })); // spring
world.addJoint(new ConeTwistJoint(a, b, anchor, axis, { swingSpan: 0.8, twistSpan: 0.4 }));
```

Anchors and axes are given in world space at creation. Jointed bodies do not collide with each other
unless `joint.collideConnected = true`.

## Continuous collision detection

Set `ccd: true` on a body. `world.ccdMode` is `'sweep'` (default: time of impact by conservative
advancement; other bodies count as stationary, rotation is not swept) or `'speculative'`.

## Queries

```ts
const hit = world.raycast(origin, direction, maxDistance, { mask: 0b10, predicate: (c) => c.body !== me });
// { collider, point, normal, distance } | null; a ray that starts inside a shape does not hit it
const cast = world.shapeCast(new Sphere(0.3), new Transform(start), translation); // first hit
const near = world.overlap(new Box(1, 1, 1), new Transform(center));               // Collider[]
```

## Events

```ts
world.onContact((e) => console.log(e.kind, e.a.body.id, e.b.body.id)); // 'begin' | 'persist' | 'end'
world.removeContactListener(listener);
```

## Snapshots and hashing

```ts
const snap = world.snapshot();   // poses, velocities, sleep, warm-start impulses
world.restore(snap);             // same bodies/colliders/joints
world.stateHash();               // 16 hex digits over the exact state bits
```

## Profiling

Set `world.clock = () => performance.now()` and read `world.timings` (`broadphase`, `narrowphase`,
`solve`, `integrate`, ms) after each step. The engine never reads a clock itself.
