# Validation

Each physics test: what is set up, what the physics says should happen, what was measured, and where
the check lives. All tests are deterministic (seeded) and run with `pnpm test`. Tolerances were set
from measured behaviour and are written next to the assertion with the reason; none was loosened
silently. Known gaps are listed at the end.

## Collision detection (Phase 4)

| Check                                  | Setup                                                        | Expected                         | Measured                                                |
| -------------------------------------- | ------------------------------------------------------------ | -------------------------------- | ------------------------------------------------------- |
| GJK/EPA against analytic pair formulas | 1,500 random pairs per type (sphere, box, capsule), 100k run | depth, normal and distance agree | default run passes; 100k run result is in `PROGRESS.md` |
| Robustness fuzz                        | 1,000,000 random near-degenerate pairs                       | no throw, no NaN, EPA converges  | 0 non-finite, 0 throws, 0 EPA non-convergence           |
| Box–box distance                       | 300 separated pairs vs a Minkowski-hull reference            | equal within 1e-9                | passes                                                  |

`tests/property/narrowphase.test.ts`, `bench/fuzz-narrowphase.ts`.

## Rigid-body dynamics (Phase 5)

| Test            | Setup                                                    | Expected                                | Measured                                                            |
| --------------- | -------------------------------------------------------- | --------------------------------------- | ------------------------------------------------------------------- |
| Free fall       | sphere dropped, no contacts                              | `y = y0 − ½gt²` to O(dt)                | passes at the tolerance in the test                                 |
| Bounce          | spheres, e = 0.2 / 0.5 / 0.8, 1,920 Hz                   | rebound height ratio e²                 | within 2%; energy drift 6e-4 over 60 s (bound 1e-3)                 |
| Momentum        | 1,000 random collisions, position correction off         | Σp conserved, ΣL conserved              | 1e-9 and 1e-6                                                       |
| Newton's cradle | five elastic spheres, 1 mm gaps, 2,400 Hz                | end ball leaves at the impact speed     | first ball stops, last leaves at 2 m/s within 0.02                  |
| Dzhanibekov     | T-shaped body spun about each principal axis, no gravity | intermediate axis flips; min/max stable | flips only about the intermediate axis; \|L\|, E conserved to 1e-11 |
| Friction ramp   | 20 ramp angles                                           | slide starts at `tan θ = μ`             | matches the Coulomb angle within the tested step                    |
| 20-box tower    | friction 0.6, 120 Hz, 10 s                               | drift < 1 cm, then sleeps               | passes                                                              |
| 55-box pyramid  | 10 levels, 120 Hz, 10 s                                  | stays up, drift < 10 cm                 | passes (needs 40 velocity iterations; 20 is not enough)             |

`tests/physics/dynamics.test.ts`, `tests/physics/stacking.test.ts`.

## Joints (Phase 6)

| Test                  | Setup                                                | Expected                                                                   | Measured                                                                                    |
| --------------------- | ---------------------------------------------------- | -------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| Pendulum              | ball and hinge, small angle and 60° / 120° release   | physical-pendulum period with the large-angle elliptic-integral correction | within 1%                                                                                   |
| Hinge motor, limits   | motor to a limit; limit stop                         | reaches limit, holds                                                       | passes                                                                                      |
| Slider, fixed, spring | each joint type                                      | constraint held; spring period                                             | passes                                                                                      |
| Double pendulum       | 30 s, 480 Hz                                         | energy within 1%                                                           | < 1% at 480 Hz; peak 1.6% at 240 Hz (first order in dt)                                     |
| 50-link chain         | released horizontally                                | no explosion; joint error < 1 mm settled                                   | settled < 1 mm; first-swing transient ≈ 1.7 mm (bound 5 mm)                                 |
| Ragdoll               | ten-body ragdolls dropped at random poses, 500 drops | no explosion, no torn limb                                                 | passed 500 drops: worst transient gap ≈ 0.2 m, worst speed ≈ 15 m/s, one rest gap of 4.5 cm |

`tests/physics/joints.test.ts`, `tests/physics/ragdoll.test.ts` (`RAGDOLL_DROPS=500`, about 25 min).
Thresholds (speed < 25 m/s, transient gap < 0.25 m, resting gap < 6 cm) were set from the measured data.

## Continuous collision detection (Phase 7)

A 5 cm sphere at 500 m/s against a 1 cm wall (random tilt, aim point and approach angle, 60 Hz):
**0 of 10,000** shots tunnel with sweep CCD; without CCD 46% do; speculative contacts 11.8% (they are
documented as approximate). A CCD ball dropped on a floor settles in the same time as a non-CCD ball.
`tests/physics/ccd.test.ts`.

## World API, meshes, determinism (Phases 8–9)

| Test               | Setup                                                       | Expected                          | Measured                                |
| ------------------ | ----------------------------------------------------------- | --------------------------------- | --------------------------------------- |
| Ray casts          | 2,000 random rays vs 25 analytic spheres                    | distances equal to 1e-5           | all agree                               |
| Mesh BVH           | 300 random queries over 700 triangles; a 100k-triangle mesh | superset of overlapping triangles | passes                                  |
| Bodies on meshes   | box, ramp, bumpy bowl with 24 bodies                        | rest on the surface               | passes                                  |
| Determinism        | random scenes, state hashes of exact float bits             | identical across runs/processes   | see `PROGRESS.md` for the full-size run |
| Snapshot / restore | restore and re-simulate 300 steps                           | every later hash equal            | passes, also into a rebuilt world       |

`tests/physics/world-api.test.ts`, `tests/physics/mesh.test.ts`, `tests/determinism/`.

## Demo (Phase 10)

`tests/demo/scenes.test.ts` runs every scene in Node (no NaN, no runaway bodies). `pnpm test:e2e`
loads every scene in headless Chromium for 300 frames and asserts no console errors and finite
positions; screenshots are in `docs/screens`.

## Phase 11 review round

Two independent reviewers (physical correctness; API and code quality) ran. Regression tests are in
`tests/physics/review-correctness.test.ts` and `tests/unit/validation.test.ts`.

| Finding                     | Before                                               | After                                     |
| --------------------------- | ---------------------------------------------------- | ----------------------------------------- |
| Friction cone               | square cone; slope speed 0.35-2.52 m/s by azimuth    | joint 2D solve, norm clamp; 2.52-2.64 m/s |
| CCD impact, sleeping body   | momentum lost                                        | both bodies woken first                   |
| Body woken by later contact | contacts missed one step; bottom box sank 5.7 mm     | sleeping pairs re-checked after the pass  |
| Input validation            | bad damping, joint bodies, mass-setup throw accepted | rejected / rolled back                    |

Not a bug: torque-free spin drift of world L is second order in dt. Measured 1 s drift at 60 rad/s:
240 Hz 6.1e-3, 480 Hz 1.6e-3, 960 Hz 4.1e-4 (120 Hz turns 0.5 rad per step, 2%). The 60-iteration
gyroscopic fixed point is converged (600 iterations give the same error). The test tolerance follows
from this scaling.

## Known gaps

- Dense random piles of boxes stay lively for a long time and do not sleep (see `PROGRESS.md`).
- Near-duplicate hull input points are merged within 1e-10 of the cloud size; exactly flat clouds are rejected.
- Mesh contacts use the triangle face normal; edge-on contacts fall back to GJK/EPA.
- Speculative CCD is approximate; sweep CCD ignores rotation during the sweep.
- Open, unconfirmed reviewer suspicions: tangent basis flip at |nx| = 0.577 in `perpendicular()`
  (warm start in a rotated frame); `timeOfImpact` returns null after 64 non-converged iterations;
  sweep impact has no friction and does not sweep rotation; `restore()` emits no end events;
  orientation lags the solve by one step; sleep threshold uses squared speed.
- The 1,000-box pile benchmark (62-106 ms against the 8 ms target) is not met.
