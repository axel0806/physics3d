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

## Design decisions and tradeoffs

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

**Friction and CCD.** No continuous collision yet; fast bodies can tunnel. That is Phase 7.
