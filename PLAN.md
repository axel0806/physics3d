# PLAN.md — 3D Rigid-Body Physics Engine (TypeScript, from scratch)

## Mission

Build a production-quality 3D rigid-body physics engine in TypeScript with no runtime dependencies, plus a raw WebGL2 renderer, an interactive demo sandbox, a physical-correctness test suite, and a benchmark harness. Work through the phases below in order. Do not stop until every acceptance criterion in every phase is met and verified by an automated test or benchmark. Iterate as many times as needed.

### Ground rules for the agent

1. Read this file at the start of every session. Keep `PROGRESS.md` updated: current phase, what's done, what's failing, what's next, and any design decisions with their reasoning.
2. No runtime libraries. No Three.js, gl-matrix, cannon, rapier, ammo, etc. Dev tooling only: TypeScript, Vite, Vitest, fast-check, ESLint, Prettier, Playwright (for visual smoke tests).
3. TypeScript strict mode (`strict`, `noUncheckedIndexedAccess`, `exactOptionalPropertyTypes`). No `any`.
4. Every phase ends with a green test run (`pnpm test`) and a lint/typecheck pass (`pnpm check`). Never mark a phase done with failing or skipped tests.
5. Commit at the end of each sub-step with a descriptive message.
6. Use subagents for review. After each phase, spawn a reviewer subagent that has not seen the implementation process to audit the phase's code against this plan and against the referenced papers, and to write additional adversarial tests. Fix everything it finds.
7. When a test fails, find the root cause. Never loosen a tolerance to make a test pass without writing a justification in `PROGRESS.md` based on the numerical method's known error bounds.
8. Hot paths must not allocate. Use preallocated pools and `Float64Array`/`Float32Array` scratch buffers. Verify with an allocation-tracking benchmark (Phase 9).

### Repository layout

```
/src
  /math        vec3, quat, mat3, mat4, transform, aabb, utilities
  /shapes      sphere, box, capsule, convex hull, triangle mesh (static), compound
  /collision
    /broadphase   dynamic AABB tree, pair manager
    /narrowphase  GJK, EPA, SAT (box-box), specialized pairs, manifold builder
    /ccd          time-of-impact (conservative advancement)
  /dynamics    rigid body, mass properties, integrator, island builder, sleeping
  /constraints contact, friction, ball, hinge, slider, fixed, limits, motors
  /solver      sequential impulse solver, warm starting, position correction
  /world       World API, stepping, events, raycasts, queries
  /render      WebGL2 renderer (no libs), debug draw, camera controls
/demo          Vite app: scene picker, inspector, play/pause/step, debug toggles
/tests
  /unit        per-module tests
  /property    fast-check property tests
  /physics     conservation and validation scenarios
  /determinism determinism tests
/bench         benchmark scenes + runner + results JSON
/docs          ARCHITECTURE.md, API.md, VALIDATION.md (generated report)
```

### Key references (implement from the ideas, not by copying code)

- Erin Catto: Iterative Dynamics with Temporal Coherence (2005), GDC talks on sequential impulses, warm starting, and Numerical Methods (2015, the gyroscopic implicit solve)
- Gino van den Bergen: GJK and EPA papers / Collision Detection in Interactive 3D Environments
- Dirk Gregorius: GDC 2013/2015 talks on SAT for convex polyhedra and contact manifolds
- Christer Ericson: Real-Time Collision Detection (dynamic AABB trees, closest-point routines)
- Brian Mirtich: mass properties of polyhedra
- Ian Millington: Game Physics Engine Development (general architecture)

## Phase 0 — Scaffolding

- pnpm + TypeScript + Vite + Vitest + fast-check + ESLint + Prettier.
- Scripts: `dev`, `build`, `test`, `test:physics`, `bench`, `check`.
- CI-style script `pnpm verify` runs check + all tests + a quick benchmark smoke run.
- Create `PROGRESS.md` and `docs/ARCHITECTURE.md` (stub).

Done when: `pnpm verify` passes on an empty project with a trivial test.

## Phase 1 — Math library

- `Vec3`, `Quat`, `Mat3`, `Mat4`, `Transform`, `AABB`. Provide both allocating convenience functions and allocation-free `out`-parameter versions for hot paths.
- Quaternion integration from angular velocity, normalization, slerp, conversion to and from matrices.
- Mat3 inverse, transpose, symmetric eigen-decomposition (Jacobi), for inertia tensors.

Acceptance:

- Property tests (≥ 1,000 runs each): `q * q⁻¹ = identity`, `|rotate(q, v)| = |v|`, `M * M⁻¹ ≈ I` for well-conditioned M, quat↔mat round trip, cross product anti-commutativity, Jacobi eigen-decomposition reconstructs the input.
- 100% line coverage on `/math`.

## Phase 2 — Shapes and mass properties

- Sphere, box, capsule, convex hull (built with 3D Quickhull from a point cloud), static triangle mesh, compound shapes.
- Support functions for all convex shapes (needed for GJK).
- Mass, center of mass, and inertia tensor for every shape. Convex hull uses Mirtich's polyhedral mass properties. Compounds use the parallel-axis theorem.

Acceptance:

- Analytic inertia matches for sphere, box, capsule, and cylinder-approximating hull (high segment count converges to the analytic cylinder within 0.5%).
- A convex hull built from a box's 8 corners matches the box's mass properties to 1e-9.
- Quickhull property tests: every input point is inside or on the hull; the hull is convex; Euler's formula V − E + F = 2 holds.

## Phase 3 — Broadphase

- Dynamic AABB tree with fat AABBs, insert, remove, move, and tree rotations for balance (Ericson / Box2D style).
- Pair manager producing deduplicated overlapping pairs.
- Raycast and AABB query against the tree.

Acceptance:

- Fuzz test: for 5,000 random AABBs undergoing 1,000 random moves, the tree's pair set equals the brute-force O(n²) pair set at every step.
- Tree height stays within 2× of optimal (log₂ n) after randomized insertion and removal.
- Raycast results match brute force on 10,000 random rays.

## Phase 4 — Narrowphase

- GJK for boolean overlap and closest points, with robust termination and degenerate-simplex handling.
- EPA for penetration depth and normal when overlapping.
- Specialized fast paths: sphere-sphere, sphere-box, sphere-capsule, capsule-capsule, box-box (SAT with face/edge clipping), convex-vs-triangle-mesh.
- Contact manifold generation: up to 4 points per pair, reduced by area maximization; persistent manifolds with feature IDs for warm starting.

Acceptance:

- GJK/EPA vs analytic answers for sphere-sphere, sphere-box, and box-box across 100,000 random configurations: distance within 1e-6, penetration depth within 1e-4, normal within 0.5°.
- Specialized paths agree with GJK/EPA on randomized inputs.
- No infinite loops or NaNs on adversarial inputs: touching shapes, coincident centers, near-parallel faces, very thin boxes (1e-3), huge scale ratios (1e4). Fuzz for 1M iterations.
- Box resting on box produces a stable 4-point manifold whose feature IDs persist across frames when nothing moves.

## Phase 5 — Dynamics and the contact solver

- Rigid body with linear and angular state, world-space inverse inertia tensor, damping, gravity scale, and body types (static, kinematic, dynamic).
- Semi-implicit (symplectic) Euler integration.
- Implicit gyroscopic torque solve (Catto 2015) so torque-free spinning bodies behave correctly.
- Sequential impulse solver: normal impulses with accumulated clamping, Coulomb friction with two tangent directions, restitution with a velocity threshold, warm starting, configurable iteration counts.
- Position correction: split impulses or NGS (pick one, justify it in `ARCHITECTURE.md`), with slop.
- Island building via union-find, plus sleeping with velocity thresholds and timers.

Acceptance (all in `/tests/physics`, each with a short write-up in `docs/VALIDATION.md`):

- Free fall: position after t seconds matches ½gt² within the integrator's expected O(dt) error.
- Projectile energy: in a frictionless, dragless world, total energy drift stays below 0.1% over 60 s for a bouncing ball with restitution 1 on a static floor.
- Momentum: in 1,000 random two-body collisions in zero gravity, total linear momentum is conserved to 1e-9 relative error and angular momentum about the origin to 1e-6.
- Newton's cradle: 5 spheres, restitution 1. Momentum transfers through the chain, and only the end ball leaves.
- Dzhanibekov effect: a T-shaped compound spun about its intermediate axis flips periodically, and rotation about the min and max axes stays stable.
- Friction ramp: a box on an incline slides if and only if tan(θ) > μ, tested across 20 angles.
- Stacking: a 20-box tower stands for 10 s with total drift under 1 cm and goes to sleep. A 10-level pyramid (55 boxes) is also stable.
- Restitution: bounce height ratio ≈ e² within 2% across e ∈ {0.2, 0.5, 0.8}.

## Phase 6 — Joints

- Ball-socket, hinge (with limits and motor), slider/prismatic (with limits and motor), fixed, distance/spring, and cone-twist (for ragdolls).
- All joints are solved in the same sequential-impulse framework, with warm starting.

Acceptance:

- Pendulum: the small-angle period matches 2π√(L/g) within 1%. The large-angle period matches the elliptic-integral solution within 1%.
- Double pendulum: energy drift stays below 1% over 30 s, and the motion is visibly chaotic in the demo.
- Chain: a 50-link chain hanging from a fixed point settles without explosion, with joint error under 1 mm.
- Hinge motor: reaches its target speed within its max torque. Limits are never violated by more than slop.
- Ragdoll: falls down a staircase without NaNs, explosions, or limb separation over 500 randomized drops.

## Phase 7 — Continuous collision detection

- Time of impact via conservative advancement for fast-moving convex bodies.
- Per-body CCD flag, plus a speculative-contact option. Compare both in `ARCHITECTURE.md`.

Acceptance:

- A 5 cm sphere fired at 500 m/s at a 1 cm-thick wall never tunnels across 10,000 randomized shots at 60 Hz.
- CCD-enabled bodies come to rest without visible stalls ("time loss") in the demo.

## Phase 8 — World API, queries, determinism

- Public `World` API: create/destroy bodies, shapes, and joints; fixed-timestep stepping with an accumulator and interpolation alpha for rendering.
- Raycast, shape cast, overlap query, and contact begin/persist/end events.
- Collision filtering by layer and mask.
- Snapshot/restore of the full world state (needed for determinism tests and rollback).

Acceptance:

- Determinism: the same scene with the same inputs produces bit-identical state hashes after 10,000 steps across 50 randomized scenes, run twice in the same process and in two separate processes.
- Snapshot/restore: after restoring a snapshot and re-simulating, hashes match the original run exactly.
- No use of `Math.random` inside the engine. Iteration order is never dependent on `Map`/`Set` insertion order from nondeterministic sources.

## Phase 9 — Performance

- Benchmark scenes: 1,000 boxes falling into a pile; a 2,000-sphere pile; 100 ragdolls; a 30×30 grid of stacks; a static 100k-triangle terrain with 500 bodies.
- The runner reports the median and p99 step time per scene, broken down by phase (broadphase, narrowphase, solve, integrate), and writes JSON to `/bench/results`.
- Optimize with struct-of-arrays for solver data where it helps, cache-friendly constraint ordering, an allocation-free hot loop, and pair/manifold caching.
- Allocation check: run each benchmark with `--expose-gc` and assert that heap growth across 1,000 steps after warm-up is ≈ 0.

Acceptance:

- The 1,000-box pile steps in under 8 ms median in Node on a modern laptop (leaving headroom for rendering at 60 fps).
- No regressions: record a baseline, and fail `pnpm bench:check` if any scene gets more than 10% slower.

## Phase 10 — WebGL2 renderer and demo

- Raw WebGL2: instanced rendering of primitives and hulls, directional light with shadow mapping, an orbit/fly camera, and a debug-draw layer (contacts, normals, AABBs, broadphase tree, joints, islands colored by sleep state).
- Demo app with scenes for every validation test above, plus these playground scenes: Jenga, a dominoes spiral, a wrecking ball, a vehicle made of hinge joints, a ragdoll cannon, and "click to throw".
- UI panel: play/pause/step, timestep and iteration sliders, debug toggles, a live perf graph, and a body inspector on click.

Acceptance:

- All scenes run at 60 fps in Chrome for the scene sizes listed in Phase 9.
- A Playwright smoke test loads every scene, runs it for 5 s, and asserts no console errors and no NaN body positions.

## Phase 11 — Final audit and documentation

- Spawn two independent reviewer subagents: one audits physical correctness against the references, and one audits API ergonomics and code quality. Fix every finding.
- Finish `docs/ARCHITECTURE.md` (pipeline diagram, design decisions and tradeoffs), `docs/API.md` (with examples), and `docs/VALIDATION.md` (each physics test: setup, expected result, measured result, and plots exported from the demo as PNGs or CSV).
- `README.md` with a quick start and GIF-worthy scene list.

Done when: `pnpm verify` and `pnpm bench:check` are green, every acceptance criterion above has a passing automated check, and `PROGRESS.md` says "COMPLETE" with a summary of final benchmark numbers.

## Stretch goals (only after Phase 11 is complete)

1. Soft constraints / XPBD alternative solver, compared against sequential impulses in `VALIDATION.md`.
2. A parallel solver using Web Workers and graph coloring.
3. WebGPU renderer path.
4. Vehicle controller with raycast suspension and a Pacejka tire model.
5. Cloth via particles and distance constraints, coupled with rigid bodies.
