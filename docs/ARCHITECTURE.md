# Architecture

## Pipeline (one `World.step(dt)`)

```
snapshot velocities (restitution uses start-of-step speeds)
  │
  ├─ broadphase: move dirty proxies in the dynamic AABB tree (fat AABBs), pair changes
  ├─ narrowphase: per overlapping pair → manifold (analytic or GJK/EPA), feature ids
  │     sphere–sphere, sphere–box, capsule–capsule, sphere–capsule: analytic
  │     box–box: SAT over 15 axes + incident-face clipping + 4-point reduction
  │     anything else: GJK (distance / overlap) + EPA (depth, normal)
  │
  ├─ integrate velocities: gravity, damping, gyroscopic term (body frame, implicit midpoint)
  ├─ integrate orientation: exact rotation about the body-frame midpoint ω
  ├─ contact solve: warm start → velocity iterations (block normal LCP, then friction)
  ├─ integrate position: translate by the post-solve velocity
  ├─ position correction: NGS, linear only, slop 0.2 mm
  └─ islands (union-find over contacts) → sleep when all bodies are slow for 0.5 s
```

Modules: `src/math` (vectors, quaternions, matrices, transforms, AABB), `src/shapes`
(mass properties, quickhull, support functions), `src/collision/broadphase` (dynamic tree, pair
manager), `src/collision/narrowphase` (support mapping, GJK, EPA, analytic paths, SAT, manifolds),
`src/dynamics` (bodies, integrator), `src/solver` (contact constraints), `src/world` (World API).

## World API, queries and determinism

`World` offers `createBody/createCollider/addJoint` and `destroyBody/destroyCollider/removeJoint`
(destroying wakes whatever was touching or jointed to the thing and emits `end` events).
`advance(frameTime)` runs whole `fixedDt` steps from an accumulator (capped at `maxSubSteps`, the
excess is dropped) and returns alpha for `interpolatedTransform`, which blends each body's pose at
the start of the last step with its current pose (slerp for orientation).

Queries are built on the same GJK and time-of-impact code as CCD: `shapeCast` is conservative
advancement against every collider in the swept bounds, `raycast` is a shape cast of a 1e-9 sphere,
and `overlap` is GJK on the candidates the broadphase returns. Candidates are visited in proxy-id
order so ties break the same way every run; `QueryFilter` takes a layer `mask` and a predicate. Casts
stop 1e-7 short of the surface (closest-point directions are only as accurate as eps over the gap).
A ray that starts inside a shape does not hit it.

Determinism rests on four rules. The engine uses no `Math.random`, clock or other ambient input (a
test greps the sources). Everything is iterated in arrays or in orders fixed by ids: contacts are
solved sorted by broadphase pair key, not by creation order, and the broadphase pair list is sorted.
Floating-point work follows one code path, with no parallelism and no data-dependent algorithm
changes. And the complete state is capturable: `snapshot()` stores poses, velocities, sleep timers,
the previous pose, the accumulator, the warm-start impulses of every contact point (by feature id)
and of every joint, and `restore()` rebuilds the contact cache and re-synchronises the broadphase.
`stateHash()` hashes the exact bit patterns of that state. The broadphase tree shape and which
fat-box pairs exist after a restore may differ from the original run, which does not matter:
contacts exist only for shapes that really overlap, and those are a function of the poses.

## Design decisions and tradeoffs

**Continuous collision detection.** Bodies with `ccd: true` use one of two modes (`World.ccdMode`).
_Sweep_ (default, `src/collision/toi.ts`): conservative advancement on GJK distance finds the first
time of impact of the body's translation against each collider in the swept bounds. Because the
separation of two translating convex shapes is convex in time, stepping by (gap / closing speed)
never overshoots. At the impact the body is placed at the target separation (`linearSlop`), an
impulse removes the approach speed (with restitution above the threshold), and the rest of the step
continues with the new velocity, up to four events. Bodies only sweep when they travel more than a
quarter of their thinnest collider extent per step, so slow bodies take the ordinary path
unchanged. Limits: other bodies count as stationary during the sweep, rotation is not swept, no
friction impulse at the impact. _Speculative_: while shapes are still apart but the closing speed
would cover the gap this step, a one-point contact with a negative depth lets the solver cancel the
approach beyond the gap. It needs no time-of-impact search and works with the whole solver
(friction, stacking), but it builds the contact from the closest feature at the start of the step.
For fast oblique motion that feature can be the wrong one (a wall's edge instead of its face), and
a contact made at the moment of impact has no restitution. Measured on 500 m/s shots at a 1 cm
wall: discrete 46% tunnel, speculative 11.8%, sweep 0% (10,000 shots).

**Joints.** `src/constraints` holds ball, hinge, slider, fixed, distance/spring and cone-twist joints.
Each is a set of scalar `Row`s plus 3x3 block locks (`PointLock`, `AngularLock`). Joints are solved
before contacts in every velocity iteration, with warm starting. Positions are corrected in the
position loop (NGS) along the start-of-step Jacobian. A velocity solver plus end-of-step projection
loses energy at first order in dt (about 6%/s in circular motion), so the lock correction is fed back
into the velocities once per step (SHAKE/RATTLE style), capped at `World.maxCorrectionSpeed`. Limit
corrections are not fed back (they bounce), and neither are corrections on bodies in contact (they pump
energy). Jointed bodies rotate after the solve; sleep islands connect only through dynamic bodies.
Known cost: the joint code, the narrowphase and per-step contact construction still allocate short-lived objects; only the contact solver iterations are allocation-free (the batch solver for joint-free worlds, scalar loops otherwise).

**Mass properties.** Polyhedron moments are integrated over signed tetrahedra with the origin and
then shifted to the centroid; moments are taken about a vertex, not the origin, so far-offset hulls
do not lose precision. The plan names Mirtich's projection method; the tetrahedral form is the same
exact polynomial integral and was easier to verify (the box-hull test matches analytic values to
1e-9).

**Convex hull.** Quickhull with per-face outside lists. Orphaned points are reassigned against all
live faces (checking only new faces loses points that lie beyond old faces). The initial simplex is
judged by scaled height and by a rounding noise floor. Tolerance: 1e-12 × max|coordinate|.

**GJK.** The closest point is found by enumerating the 15 subsets of the simplex and keeping the
valid one with least norm. This is exact and degeneracy-robust. A center-aligned pair of spheres
puts the origin on a segment of the Minkowski difference, which EPA cannot start from; GJK then
restarts from generic directions.

**EPA.** van den Bergen's expansion. Two tolerances matter: the termination tolerance on the
nearest face's reach (1e-6 × scale), and the visibility test, which must be tight (1e-12 × scale).
A loose visibility test keeps faces that barely see the new point; after roughly 130 splits on
rounded shapes this folded the polytope and corrupted the normal (found by the 100,000-configuration
sphere-box run). Convergence is linear on curved shapes, typically 50–150 iterations.

**Box–box manifolds.** SAT chooses the reference face; the incident face is clipped against the
reference side planes (Sutherland–Hodgman); points behind the reference face are kept and reduced to
four by area. Corners within 1e-9 of a clip plane count as inside and keep their ids, so ids are
stable across frames when nothing moves (required for warm starting).

**Contact impulses.** Applied at the midpoint of the two surface anchors, so both bodies' offsets
refer to one point and equal and opposite impulses cancel in torque (conservation of angular
momentum about the origin).

**Normal solver.** Multi-point manifolds are solved as a linear complementarity problem: an
active-set enumeration over the normal effective-mass matrix (at most 16 subsets for four points).
Four coplanar points make that matrix singular (translation plus two rotations spans three
dimensions), so it carries a 1e-9 Tikhonov term, which selects the symmetric minimum-norm solution.
Sequential per-point solving converged too slowly for stacks: a 20-box tower drifted and collapsed.
Friction is solved after all normal impulses, against the updated loads.

**Restitution.** The target is −e·v_n measured from the velocities at the start of the step. Using
the post-gravity velocity added g·dt to every rebound (measured: an exact 0.16 m/s per bounce at
dt = 1/60).

**Position correction: NGS, linear only, slop 0.2 mm.** Split impulses would keep the velocity solve
unchanged too, and I chose NGS because it is simpler and needs no second velocity pass. Correction
is translational. Rotational pushes at each point of a manifold made tall stacks topple, measured on
a 20-box tower. The steady sag of a tower equals the slop per contact (0.2 mm × 20 ≈ 4 mm), so the
slop sets the stacking accuracy directly. Tighter slop (1 mm) made the position pass chase near-zero
penetration and destabilized the stack.

**Gyroscopic term and integration.** The torque-free Euler equation I ω̇ = (I ω) × ω is solved in
the body frame with the implicit midpoint rule, so |L| and kinetic energy are conserved (1e-11 over
40 s, checked). The orientation is advanced by an exact rotation about the body-frame midpoint, and
the body-frame velocity is carried into the new orientation. Freezing the world inertia instead
leaked angular momentum at first order in dt. The implicit-midpoint solve reproduces an RK4 reference
of the Euler equations to four digits over 40 s.

**Time stepping and contacts.** Contacts are generated from positions at the start of a step, so
the discrete penetration at impact is up to v·dt. Energy error for a bouncing ball is first order
in dt: 4.9e-3 at 240 Hz, 6.1e-4 at 1920 Hz (measured). Speculative contacts would remove the error;
they are a candidate for the CCD work in Phase 7.

**Sleeping.** Islands come from union-find over contacts. A body sleeps once its island has stayed
below 1e-2 m/s and 1e-2 rad/s for 0.5 s. A new contact wakes its bodies; a persisting one does not
(waking on every step kept resting stacks awake).

**Friction.** Friction is a joint 2D solve over both tangent directions with a clamp on the norm of
the tangent impulse (a round cone, mu times the normal load). The earlier per-axis clamp gave a square
pyramid: the speed of a body on a 33 degree slope varied 0.35-2.52 m/s with azimuth; the round cone
gives 2.52-2.64 m/s. CCD is described above.

**Waking and sleeping pairs.** A CCD impact on a sleeping body wakes both bodies before the impulse.
If anything wakes during the contact pass, sleeping pairs are re-checked afterwards so that a body woken
by a later contact does not miss its contacts for a step (that miss let the bottom box of a sleeping
stack sink 5.7 mm).

**Spin drift.** The orientation advances by a finite rotation per step, so world-frame angular momentum
of a torque-free spinning body drifts at second order in dt (1 s at 60 rad/s: 6.1e-3 at 240 Hz, 1.6e-3
at 480 Hz, 4.1e-4 at 960 Hz). The 60-iteration gyroscopic fixed point is converged.
