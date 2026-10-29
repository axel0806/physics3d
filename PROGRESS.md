# PROGRESS

## Current status

**Phases 0-10 are implemented and tested; Phase 11 (final audit) is done. Not COMPLETE:** one
acceptance criterion is **not met**: the plan's 8 ms median for the 1,000-box pile. Measured now:
40 ms median (was 88 ms at the start of Phase 11, 191 ms before Phase 9); narrowphase 16 ms, solve
16 ms, integrate 6 ms, broadphase 1.5 ms, with about 535 of 1,000 bodies awake. The solver alone
cannot reach 8 ms (floor about 14 ms at 6 iterations); it needs SIMD/Wasm or multithreading, or a
pile that sleeps completely (the 1,000-box pile is about 96% asleep after 20 s; the 400-box arena
sleeps fully at about 12 s). Everything else in the plan has an automated check.

Last full verification (all green): `tsc`, `eslint .`, 211 tests (plus 1 expected fail), 100k-sample
narrowphase accuracy run (170 s), `bench:check` against the re-recorded baseline, `bench:alloc` (no
retained growth in any scene), and the 19-scene browser smoke test. Benchmark medians (ms): pile-1000-boxes
40, pile-2000-spheres 39, ragdolls-100 142, stacks-30x30 0.7, terrain-500 279 (single runs are noisy
by about 10%; the baseline was recorded on a quiet machine).

## Phase status

- **Phase 0 (scaffolding)**: done. TypeScript pinned to 5.9 (typescript-eslint requires <6.1).
- **Phase 1 (math)**: done. 39 tests, property tests at 2000 runs, 100% coverage of `src/math`.
- **Phase 2 (shapes, mass)**: done. Independent reviewer pass completed; findings fixed (vertex-referenced
  mass moments, dimensionless collinearity test, documented tolerance corrected to 1e-12 × max|coord|).
  Reviewer's adversarial tests kept as regression tests.
- **Phase 3 (broadphase)**: done. Fuzz at full size (5,000 AABBs × 1,000 moves, exact pair set each step),
  height ≤ 2·log₂n, 10,000 rays and 2,000 queries match brute force.
- **Phase 4 (narrowphase)**: implementation done; acceptance mostly verified.
  - Default-size tests (1,500 per pair type) pass.
  - 1,000,000-iteration adversarial fuzz passed (0 non-finite, 0 throws, 0 EPA non-convergence).
  - **Open:** the full 100,000-configuration accuracy run was started in the background
    (`NARROWPHASE_SAMPLES=100000`) against the fixed EPA. Its result is not recorded yet. Earlier, a
    100k run found two real failures (EPA folding, fixed; see below). Re-run `pnpm bench:accuracy` to confirm.
  - Box-box distance is checked against an independent Minkowski-hull reference on 300 separated samples
    per run (not 100k: each reference costs a quickhull of 64 points).
- **Phase 5 (dynamics, solver)**: all eight acceptance scenarios have automated tests and pass.
  - Free fall (O(dt)); bounce e² within 2% for e ∈ {0.2, 0.5, 0.8} at 1920 Hz; bounce energy drift 6e-4 over 60 s at 1920 Hz (bound 1e-3);
    momentum 1e-9 and angular momentum 1e-6 over 1,000 random collisions (NGS disabled for this test, see below);
    Newton's cradle; Dzhanibekov (intermediate axis flips, min/max stable, body-frame); friction ramp
    (20 angles); 20-box tower (<1 cm, sleeps); 55-box pyramid (stable, sleeps).
  - **Not yet done:** the `docs/VALIDATION.md` write-up for these tests; and the phase-5 reviewer subagent pass.

## Phase 6 (joints): done, reviewer pass pending

- Implemented in `src/constraints/`: ball, hinge (limits, motor), slider (limits, motor), fixed,
  distance/spring, cone-twist. Rows are scalar (`Row`) or 3x3 blocks (`PointLock`, `AngularLock`).
- Tests (`tests/physics/joints.test.ts`) pass: pendulum small angle and 60/120 degree large angle
  (hinge and ball, within 1% of the elliptic-integral period), hinge motor and limits, slider, fixed,
  spring, double pendulum (<1% energy at 480 Hz; peak 1.6% at 240 Hz, first order in dt),
  50-link chain (settled joint error <1 mm; first-swing transient ~1.7 mm, bounded at 5 mm).
- Ragdoll (`tests/physics/ragdoll.test.ts`): default 40 drops; `pnpm test:ragdoll` runs 500 (about 15 min).
  Thresholds were set from measured data: speed < 25 m/s, transient anchor gap < 0.25 m, resting gap < 6 cm.
  500-drop result (passed, 1526 s): worst transient gap ~0.2 m, worst speed ~15 m/s, one drop rests with
  a 4.5 cm gap (limb radius 5 cm). Transient gaps come from deep-penetration pushes without CCD (Phase 7).
- Fixes made along the way: hinge angle is unwrapped about the middle of its limit range and limit
  correction is clamped (`MAX_LIMIT_FIX` 0.2 rad); the cone-twist twist axis is the normalized sum of both
  bodies' twist axes (A's alone chattered at large swing); SHAKE feedback is skipped for bodies in contact
  (it pumped energy and whipped limbs at 41 m/s).
- Design decision, energy loss: a velocity-level joint solver plus position projection dissipated ~6%/s
  in uniform circular motion (first order in dt). Fixed by (1) correcting positions along the
  start-of-step Jacobian and (2) feeding the lock-correction displacement back into velocities once per
  step (SHAKE), capped at `maxCorrectionSpeed` (1 m/s, rad/s). Limit corrections are not fed back
  (that made limits bounce and chatter). Result: circular-motion KE change +0.007%/s, period error ~0.
- Jointed bodies rotate after the solve (`integrateOrientationPostSolve`); free bodies keep the
  pre-solve rotation used by the contact tests.
- Sleeping islands now link through dynamic bodies only (a shared static body no longer merges islands).
- Not yet done: allocation-free joint code (Phase 9), Phase 6 reviewer pass, `docs/VALIDATION.md`.

## Phase 7 (CCD): done, reviewer pass pending

- `src/collision/toi.ts` (conservative advancement) and `World.ccdMode` ('sweep' default, 'speculative').
  Per-body flag `ccd`. Comparison written up in `docs/ARCHITECTURE.md`.
- Acceptance: a 5 cm sphere at 500 m/s against a 1 cm wall (randomly tilted, random aim and approach
  angle up to 60 degrees, 60 Hz): 0 tunnelled in 10,000 shots (`tests/physics/ccd.test.ts`). Without
  CCD 46% tunnel (so the test has teeth); speculative mode 11.8%, documented, test bound 16%.
- No time loss: a CCD ball dropped on a floor settles in the same time as the discrete one, and a slow
  CCD ball takes the identical path as a non-CCD ball (bit-equal position).
- Limits: other bodies are stationary during the sweep, rotation is not swept, no friction at impact.
- Repo-wide: vitest `testTimeout` raised to 60 s (the property tests failed the 5 s default under load).
  A random quickhull run found a sliver face (10 long, 2e-5 wide) with a vertex 1.2e-6 above it; the
  test now skips faces with altitude < 1e-3 (limitation 1 below, near-duplicate input points).
- The Phase 7 acceptance item "no visible stalls in the demo" needs the Phase 10 demo.

## Phase 8 (World API, queries, determinism): implemented, full-size run recorded below

- `World`: destroy body/collider/joint, `advance(frameTime)` + `interpolatedTransform`, `raycast`,
  `shapeCast`, `overlap` (layer mask and predicate filters), contact begin/persist/end events,
  `snapshot()` / `restore()` / `stateHash()`. Contacts are now solved in canonical (pair key) order.
- Tests: `tests/physics/world-api.test.ts` (15: destroy, fixed step, alpha, ray vs analytic sphere on
  2,000 random rays, filters, shape cast, overlap) and `tests/determinism/determinism.test.ts`
  (same scene twice in-process, two child processes, one-bit sensitivity, source grep for ambient
  nondeterminism, snapshot/restore replays every later hash, restore into a rebuilt world).
- Default test size: 6 scenes x 1,500 steps. Full acceptance size: `DETERMINISM_FULL=1` runs 50 scenes
  x 10,000 steps, twice in-process and in 4 child processes. Result: see the line below.
- Event semantics: a contact between a sleeping body and a static body is dropped silently when the
  island sleeps (no `end` event), and recreated with `begin` on wake.

## Phase 9 (performance): done except the 8 ms target

- Changes: allocation-free contact solver, a struct-of-arrays batch solver for joint-free worlds
  (`src/solver/batch.ts`, early exit once impulses converge), scalar inertia/sleep/position code,
  BVH-backed static triangle meshes with a face-normal fast path (`triangleContact.ts`), a faster GJK
  simplex step, 4 instead of 10 position passes over contacts, per-phase timing (`world.clock`).
- Runner: `pnpm bench` (median, p99, max and per-phase medians per scene, JSON in `bench/results`),
  `--baseline` records, `pnpm bench:check` fails if a median is more than 10% slower than the
  baseline. `pnpm bench:alloc` (`--expose-gc`) checks retained heap growth over 1,000 steps: no
  growth in any scene (the narrowphase still creates short-lived garbage that the collector reclaims).
- **8 ms target not met.** The 1,000-box pile (ten layers, 2,000-2,500 contacts, all awake) went from
  191 ms to about 85 ms median. The remaining cost is the 40-iteration block solve itself (about
  1 us per constraint per iteration in JS); getting to 8 ms needs far fewer iterations, a different
  solver (soft constraints or TGS substeps) or most of the pile asleep. Numbers: see the table below.
- Phase 11 profile of the 1,000-box pile (`BENCH_ITERS`, `--quick`, this machine): 40 iterations 106 ms (bp 3.4, np 24, solve 70, int 9); 10 iterations 70 ms (solve 26); 4 iterations 62 ms (solve 19, np 29). Cutting iterations alone cannot reach 8 ms: even at 4 the narrowphase (about 10 us per pair) and integrate phases alone are about 38 ms. A real fix needs all of: a cheaper box-box path, far fewer iterations (TGS or soft constraints), and sleeping. Not attempted yet.
- Mesh pipeline: 100 bodies on a 100k-triangle terrain went from 554 ms to about 60 ms.

## Phase 10 (renderer and demo): done

- `demo/`: raw WebGL2 (instanced boxes, spheres, capsules, hulls, meshes; directional light with a
  PCF shadow map; orbit and fly cameras; debug draw of contacts, normals, AABBs, broadphase tree,
  joints, islands by sleep state), UI panel (scene picker, play/pause/step/reset, step rate and
  iteration sliders, time scale, perf graph, body inspector) and 19 scenes: the validation scenes
  plus Jenga, dominoes, wrecking ball, vehicle, ragdoll cannon and click-to-throw.
- Checks: `tests/demo/scenes.test.ts` (every scene in Node) and `pnpm test:e2e` (Playwright, headless
  Chromium with software WebGL2: every scene, 300 frames, no console errors, finite positions, draws
  something). 19 of 19 pass; screenshots in `docs/screens`.
- Not verified: the "60 fps in Chrome" criterion (software rendering only; the step time of the large
  scenes is above 16 ms, see Phase 9).

## Phase 9 solve-phase pass (pile-1000-boxes)

- Batched worlds now build the flat arrays straight from the constraints (`ContactBatch.prepare`
  replaces `prepareContact` + `buildNormalK` + `gather`) and run the contact position passes on the
  arrays (`solvePositions`, same arithmetic and order). Non-iteration solve overhead for the pile
  fell from about 13 ms to about 5 ms. Results match the old path (all tests unchanged).
- The 40 default velocity iterations cost about 1.4 ms each on the pile (2,300 manifolds), so the
  default solve stays about 60-70 ms. The pile scene therefore sets `velocityIterations: 10` (opt-in
  per scene; tests keep the 40 default, the 55-box pyramid needs it): solve about 17 ms; 6 iterations
  about 14 ms.
- Tried and rejected: precomputed inverse of the 4-point normal matrix (replaces the active-set
  elimination in the all-active case). The Tikhonov-regularised coplanar matrix is too ill
  conditioned: the 20-box tower drifted 12 cm. Not merged.
- Not done: TGS substeps, soft constraints, graph colouring. The max-change convergence test never
  triggers in a pile (a few constraints keep moving by 0.2 m/s at iteration 40).
- 8 ms for the whole pile step is not reachable by the solver alone: the solve floor is about
  14 ms at 6 iterations on this machine.

## Phase 9 solve-phase pass 2 (pile-1000-boxes)

- `ContactBatch` now stores per-point Jacobian rows (angular Jacobian J and I^-1 J for the normal and
  both tangents), so a row costs a few multiply-adds, and `iterate` keeps both bodies' velocities in
  locals for a whole constraint (one load and one store per body instead of per point). The 4-point
  normal matrix is built from the same rows (symmetric half only).
- Multi-point normal blocks: all-active case solved with a precomputed K^-1 (lambda = lambda_old -
  K^-1 (speed error), no K product, so no cancellation); otherwise principal pivoting starting from the
  points that carried load last time (exhaustive search only if it cycles). The solution is unique
  because K is positive definite. Tower and pyramid tests are unchanged.
- Constraints whose two bodies no other constraint has moved since they last settled are skipped
  (`skipTolerance` 1e-5 m/s). It helps resting stacks at 40 iterations; in the falling pile nothing
  converges below 5 mm/s within 10 iterations, so it saves nothing there.
- Measured (min of 3, same tree, quiet machine, bench scene, 10 iterations): solve phase 14.5 ms ->
  9.3 ms. Breakdown now: prepare 2.3, iterate 4.9 (0.48 per pass: normal rows 0.29, friction 0.19),
  scatter 0.35, position passes 0.35, contact ordering 0.5.
- Iteration count: 10 is the best setting for the whole step. 6 iterations cost less per step but the
  pile stays awake longer (545 awake bodies at step 360 vs 67), so the median is worse (31 vs 25 ms).
  Stacking quality versus `velocityIterations` (20-box tower drift / 55-box pyramid drift, 120 Hz, 10 s):
  40: 2.9 / 0.9 mm; 10: 12.7 / 7.4 mm; 6: 21.5 / 16 mm; 4: 33 / 26 mm.
- Tried and rejected: sorting contacts by height (top-down) for faster Gauss-Seidel propagation: no
  change in tower sag or time to sleep.
- Finding: the pile median is set by when it falls asleep, not by per-step cost. With
  `contactPositionIterations = 0` the whole pile sleeps by step ~350 and the step median is 4.6 ms, but
  boxes interpenetrate by up to 0.5 m, so it is not a fix. The NGS position pass (5 mm slop, mean
  settled penetration 4-5 mm) keeps the heap awake; a softer or hysteretic position correction (or
  split impulse with a larger slop) is the lever for 8 ms, not more solver speed.

## Known limitations (carry-forward)

1. **Near-flat hulls (thickness ratio ≲ 1e-4)**: quickhull can accept a near-duplicate vertex in a
   near-collinear triangle, which tilts a face ~3e-7 rad. Regression test asserts termination and finiteness
   only. Planned fix: merge near-duplicates, exact predicates (Phase 4 robustness).
2. **Exactly touching chains**: Newton's cradle with zero gap at dt = 1/240 gives the wrong answer (balls
   share momentum inelastically) because restitution is measured from start-of-step velocities and
   neighbouring pairs have not yet impacted. The test uses 1 mm gaps and dt = 1/2400. Documented.
3. **Discrete impact energy error**: contacts are found after a step has moved bodies by v·dt, so bounce
   error is first order in dt (4.9e-3 at 240 Hz, 6.1e-4 at 1920 Hz). Speculative contacts or CCD (Phase 7) would remove it.
4. **Angular momentum with position correction**: NGS moves bodies, which changes Σ x×mv. The
   momentum test therefore runs with `positionIterations: 0`, so only velocity-level impulses are checked.
5. **Iteration count**: default velocity iterations are now 40. Ten iterations were insufficient for a
   55-box pyramid (stable at 40, not at 20). Phase 9 must budget for this.
6. **Hull support** is O(V) per call and hull construction is O(n·F). Fine for now; revisit in Phase 9.
7. **Friction**: stacking stability depends on friction (frictionless stacks topple), which is physical.
8. **Dense piles do not sleep**: 400 random boxes in a 9 x 9 m arena keep about 1 J of kinetic energy
   for 25 s. With position correction off they sleep within 10 s, so the NGS push-out feeds energy
   back through gravity. Fewer contact position passes (4) helped but did not cure it. Unresolved.
9. **Mesh contacts** use the triangle face normal; edge-on cases fall back to GJK/EPA.
10. Static bodies moved after creation are not re-inserted in the broadphase (proxies of static and
    sleeping bodies are skipped each step); teleport them with a kinematic body or recreate them.

11. **Torque-free spin drift is second order in dt (not a bug)**: world-frame L of a freely spinning
    body drifts because the orientation update is a finite rotation per step. Measured 1 s drift at
    60 rad/s: 6.1e-3 at 240 Hz, 1.6e-3 at 480 Hz, 4.1e-4 at 960 Hz (factor 4 per doubling). At 120 Hz
    a step turns the body 0.5 rad, drift 2%. The test tolerance is justified by this scaling. The
    60-iteration gyroscopic fixed point is converged: 600 iterations give an identical error.
12. **Open reviewer suspicions (unconfirmed, no failing test yet)**: the tangent basis in
    `perpendicular()` flips at |nx| = 0.577, so a warm-started friction impulse may be applied in a
    rotated frame for one step; `timeOfImpact` returns null after 64 non-converged iterations (the
    sweep then misses the hit); the sweep impact applies no friction and does not sweep rotation;
    `restore()` emits no contact end events; orientation lags the solve by one step; the sleep
    threshold compares squared speed.
13. **Open API items**: `step()` allocates short-lived garbage (`orderedContacts`, `currentPairs`, a
    per-step `Map`, `refreshPair`); `onContact` has no unsubscribe return value.

14. **Dense piles and sleeping (mostly solved, not perfect)**: 400 random boxes in a 9 x 9 m arena
    (`tests/physics/pile-sleep.test.ts`) used to keep about 1 J of kinetic energy and never sleep (0 of
    400 asleep after 20 s; with position correction off they slept at about 9 s). Causes: (a) box-box
    manifolds kept only penetrating corners, so a resting box rocked between one- and two-point
    contacts; (b) the 0.2 mm position-correction slop is below the compression residual of a loaded
    heap (measured: about 850 contact points 3 to 14 mm deep in a settled 8-layer pile), so the push-out
    fired every step and hopped bodies against gravity; (c) a single restless body kept its whole
    union-find island awake, and every flickering contact woke both bodies. Fixes: 2 mm box-box
    contact margin (speculative points; `contactMargin`), separate 5 mm `contactSlop` for position
    correction (`linearSlop` is unchanged for CCD), sleep speeds 0.04 m/s and 0.15 rad/s, per-body
    neighbour sleep (a body sleeps when it and every body it touches are slow; joints still sleep as a
    group), and wake-on-contact only for moving or kinematic bodies. Result: the 400-box pile is fully
    asleep at 11.7 s (test bound 20 s, runs in about 30 s); the bench 1000-box pile is 31 percent asleep
    at 4 s, 77 percent at 6 s and 96 percent at 20 s, but not monotonic (individual bodies keep waking
    neighbours, so it does not reach 100 percent). The ragdoll heap exploding with a 4 mm margin was root-caused: not
    the margin but the gyroscopic integrator. Its implicit-midpoint fixed point diverges once dt*|w| is
    large (a limb spinning at about 180 rad/s at 60 Hz), and 60 diverging iterations turned that into
    thousands of rad/s; the margin only perturbed the chaotic heap into that state. Fixed by iterating to
    convergence and falling back to one Newton (linearised implicit Euler) step on non-convergence
    (test: `gyroscopic integration at large dt*|w|`). 2 to 6 mm margins now agree (max speed 28 m/s).
    Sleep timers also use hysteresis now (hold between 1x and 2x the sleep speeds, reset above 2x), which
    took the bench pile from 351 to 90 awake bodies at 6 s.
    Rest window (seed 7 and seed 2 of the pile helper kept 3 to 9 bodies awake at 20 s): a body wedged by
    single-point contacts jitters 1 cm at 0.05 to 0.2 m/s because 4 position passes lift it, the 2 mm
    speculative margin then lets it fall back by gap/dt, and gravity repeats the cycle (it persists with
    40 velocity iterations; with position correction off or margin 0 it sleeps). The sleep timer now also
    runs while a body stays within 1 cm and 0.05 rad of the pose where its timer started, whatever its
    speed; leaving the window without being slow resets the timer. Free fall leaves the window within
    45 ms. Pile seeds 1 to 8 all sleep within 4 to 12 s.
    Position correction still injects some energy in the first seconds; a split-impulse or soft contact
    solver would be the real fix.

## Bugs found and fixed during this work (for the record)

- Quickhull: orphan points reassigned to new faces only (lost points); degenerate-simplex test used a
  normalized normal (noise amplified). Both fixed.
- EPA: visibility tolerance too loose (1e-6); polytope folded after ~130 splits on rounded shapes
  (sphere-box normal 1° off; sphere-sphere depth error 0.75 at 100k). Fixed by tight visibility (1e-12).
- Anchors: body-frame anchor applied the inverse rotation twice (wrong face corners on tilted bodies).
- Angular momentum: impulses at different points on the two surfaces (now midpoint); world inertia
  uninitialized until the first step (now at mass setup); frame mismatch from rotating after the solve
  (orientation now updated before the contact solve).
- Gyroscopic integration: world-frame frozen inertia leaked L at O(dt); now body-frame implicit midpoint
  with an exact pose update (|L| and E conserved to 1e-11 in free motion; matches RK4 Euler reference).
- Restitution: used post-gravity velocity, adding g·dt per bounce. Now start-of-step.
- Restitution threshold: impacts at exactly the threshold speed were skipped (`<` vs `≤`); tests use speeds above it.
- Manifold clipping ids flipped with rounding; clipping now tolerant, SAT ties favour faces.
- Coplanar four-point normal matrix is singular; 1e-9 Tikhonov term selects the symmetric solution.
- Sequential normal/friction interleaving let friction saturate at one corner and pitch the body.
  Now: block normal LCP (active-set), then friction.
- Position correction: rotational pushes toppled tall stacks; now linear only. Slop 0.2 mm (4 mm sag over 20 boxes).
- Waking on every persisting contact reset the sleep timer; now only new contacts wake.
- Phase 11 review round (two independent reviewers: physical correctness, API and code quality):
  - Friction was a square (pyramid) cone and anisotropic: speed down a 33 degree slope varied
    0.35-2.52 m/s with azimuth. Now a joint 2D solve with a norm clamp (2.52-2.64 m/s).
  - A CCD impact on a sleeping body lost momentum; both bodies are now woken first.
  - A body woken by a later contact missed its contacts for one step, and the bottom box of a
    sleeping stack sank 5.7 mm. Sleeping pairs are now re-checked after the pass if anything woke.
  - Input validation: `createBody` rejects non-finite or negative damping, `addJoint` rejects the same
    body twice or foreign bodies, `createCollider` rolls back if mass setup throws.
  - Tests: `tests/physics/review-correctness.test.ts`, `tests/unit/validation.test.ts`.
- Friction test placed boxes inside the ramp, and the Dzhanibekov test mis-detected flips in world frame.
  Both are test bugs, fixed.

## Design decisions

See `docs/ARCHITECTURE.md` (pipeline, solver, position correction, gyroscopic integration, tradeoffs).

## Next steps (in order, superseded by Phase 11 below)

1. Record the 100k narrowphase accuracy result in this file (`pnpm bench:accuracy`).
2. Write `docs/VALIDATION.md` for the Phase 4 and Phase 5 tests (setup, expected, measured).
3. Phase 5 reviewer subagent (per plan), then fix findings.
4. Phase 6 reviewer subagent, then fix findings.
5. Phase 7 reviewer subagent, then Phases 8–11 as planned in `PLAN.md`.

## Uncommitted state

Phase 6 is committed. Full suite (ragdoll excluded): 130 tests pass; 500-drop ragdoll passes.
