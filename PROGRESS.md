# PROGRESS

## Current phase

Phase 2 (shapes + mass properties) — implemented, tests green. Phase reviewer pass pending.

## Done

- **Phase 2 reviewer pass**: independent subagent audit. Findings fixed: (1) polyhedron moments now integrated about a vertex (precision for far-offset hulls); (2) quickhull collinearity test made dimensionless; (3) docs corrected to the code's real tolerance 1e-12·max|coord|. Its adversarial tests are kept (`tests/property/review-phase2.test.ts`). Known open item: `ConvexHull.support` is O(V) per call and hull construction is O(n·F); fine now, revisit in Phase 9.

- **Phase 3**: `src/collision/broadphase` — DynamicTree (AVL-style rotations, SAH-style branch-and-bound insertion, fat AABBs, closest-hit raycast, AABB query, `validate()`), BroadPhase pair manager (deduplicated pairs, incremental re-query of moved proxies, begin/end events emitted only on real transitions).
  - Fuzz: 5,000 AABBs × 1,000 moves, pair set equal to brute force at every step (incremental reference per move; full O(n²) recheck every 50 steps). Height ≤ 2·log₂ n after randomized insert/remove over 5 seeds. 10,000 random rays and 2,000 random queries match brute force.
  - Bugs found by tests and fixed: (1) balance skipped nodes using a stale stored height; (2) one rotation could leave a height gap >1 when a new parent sat above a much taller sibling, now rebalanced recursively; (3) event churn from removing and re-adding unchanged pairs.

- **Phase 0**: pnpm project, TypeScript (pinned 5.9 — typescript-eslint requires <6.1), Vite, Vitest, fast-check, ESLint, Prettier, Playwright. `pnpm verify` green.
- **Phase 1**: `src/math` — Vec3, Quat, Mat3 (incl. Jacobi `symmetricEigen`), Mat4, Transform, AABB, util.
  - 39 tests (property tests at 2000 runs each). 100% line/branch/function coverage on `src/math`.
- **Phase 2**: `src/shapes` — Sphere, Box, Capsule, ConvexHull (Quickhull), TriangleMesh (static), Compound.
  - Support functions for all convex shapes; mass/COM/inertia for every shape.
  - Quickhull property tests: inside-or-on, convexity, Euler V−E+F=2 with manifold edges, volume ≤ bbox.

## Failing

- none (all 66 tests green; property suites run with fresh random seeds each run — 4 stability runs green after the fixes below)

## Known limitations (carried into Phase 4)

- **Near-flat hulls (thickness ratio ≲ 1e-4)**: quickhull can accept a near-duplicate vertex in a near-collinear triangle. The resulting face tilts by ~3e-7 rad, so a far vertex ends up ~3e-6 outside it. Root cause: tolerance-based visibility is inconsistent with the new face's plane at that geometry. Regression test `a near-flat cloud … (KNOWN LIMITATION)` asserts termination and finiteness only. Planned fix (Phase 4 adversarial work): merge near-duplicate input points before hulling, and/or exact orientation predicates. Property tests exclude these hulls via the well-conditioned precondition below.

## Next

- Phase 2 reviewer subagent pass (per plan), then Phase 3 (dynamic AABB tree).

## Phase 4 status

- GJK: closest point by enumerating simplex subsets (exact, degeneracy-robust). Overlap is declared only with an EPA-ready simplex; if the origin lands on a lower-dimensional simplex, GJK restarts from generic directions. (Center-aligned sphere pairs otherwise always hit this.)
- EPA: van den Bergen expansion with tolerance 1e-6·scale and 128-iteration cap; typical sphere pairs need ~66 iterations because EPA converges linearly on curved shapes. Results report `converged`.
- Measured at default sample sizes (1500 per pair type): sphere-sphere depth error < 1e-4, normal < 0.5° (worst measured ~0.1°), distance < 1e-6; sphere-box analytic vs GJK depth < 1e-4; box-box manifold vs independent Minkowski-hull reference: depth ~1e-10 typical.
- Box-box reference: convex hull of the 64 pairwise vertex differences (exact Minkowski difference), built with the Phase 2 quickhull. This is independent of both SAT and EPA.
- Manifolds: reduce to 4 points (deepest, farthest, max triangle, far from diagonal). Feature ids are geometry-derived and persist across frames.
- **Pending for Phase 4 acceptance**: full 100,000-configuration accuracy run (`pnpm bench:accuracy`, est. ~12 min per pair type) and 1,000,000-iteration adversarial fuzz (`pnpm fuzz:narrowphase`). Results are recorded below once they finish.

## Design decisions

- **Polyhedral mass properties (Phase 2)**: `polyhedronMassProperties` integrates each oriented triangle against the origin (signed tetrahedra) using the exact simplex moment identities ∫x_i x_j dV = V/20·(Σ_k p_ki p_kj + Σ_k p_ki Σ_l p_lj). This is exact, the same polynomial integrals that Mirtich's projection method computes, and it is verified against analytic box inertia to 1e-9. **Deviation:** the plan names Mirtich's method; I used the tetrahedral form because it is a closed-form identity that I could check directly. Revisit only if profiling shows it matters.
- **Compounds**: parallel-axis theorem via `combine()`; children are re-expressed in the parent frame with `transformMassProperties`.
- **Capsule**: cylinder + two hemispheres. A hemisphere's inertia about its COM is (2/5 − 9/64)·m_h·r² transverse, 2/5·m_h·r² axial, with COM 3r/8 from its flat face (verified in the test against the derivation).
- **Quickhull tolerance**: `tol = 1e-12 · max|coord|` (the value in code; an earlier draft of this note said 1e-8 and was wrong). Points within `tol` of the hull are dropped, so the hull is within `tol` of every input point by construction. Two bugs were found and fixed while testing:
  1. Orphaned outside points were reassigned only to _new_ faces. A point can lie beyond an old, non-visible face and then be lost, so orphans are now reassigned against every live face.
  2. The initial-simplex "coplanar" test used the distance to a normalized normal. For nearly parallel edges the normal's direction error (~ε·|e|²/|e1×e2|) can exceed the distance tolerance, so a duplicate point looked non-coplanar. The test now uses the scaled determinant |det(e1,e2,e3)|/(|e1||e2||e3|) > 1e-9.
- **Property-test tolerances**: the "inside" and "convex" checks use `2e-8 · scale`, which is 2× the hull's own tolerance. The factor of 2 absorbs rounding for points exactly at the drop threshold; it is not a loosening of the algorithm's guarantee. Those checks use only hulls with volume > 1e-6 · bbox³ and only triangles with cross-product length ≥ 1e-4, because a sliver's normal has rounding error ≈ ε·scale²/area, which is ~1e-9 at area 1e-4 and dominates for smaller slivers. Flatter hulls are exercised by the Phase 4 adversarial thin-shape tests.
- **Quickhull complexity**: O(n·F) for orphan reassignment (scans live faces). Acceptable at Phase 2; revisit if Phase 9 benchmarks need it (the 100k-triangle terrain uses TriangleMesh, which does not build a hull).
- **Tooling note**: the vitest runs must be launched in the background with a bounded wait, since a full run plus property tests exceeds the foreground 120 s limit.
