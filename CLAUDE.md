# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A from-scratch 3D rigid-body physics engine in TypeScript (`src/`, zero runtime dependencies) plus a raw WebGL2 renderer/demo (`demo/`), a benchmark harness (`bench/`) and a physical-correctness test suite (`tests/`). `PLAN.md` is the original phased spec with acceptance criteria; `PROGRESS.md` tracks phase status, benchmark numbers and known limitations (read it before starting work); `docs/ARCHITECTURE.md`, `docs/API.md` and `docs/VALIDATION.md` hold design rationale, API and test evidence. Keep `PROGRESS.md` current when finishing work.

## Commands

Package manager is pnpm (`npx pnpm@9 ...` if not installed).

```bash
pnpm test                       # vitest run (unit, property, physics, determinism, demo scene tests)
pnpm vitest run tests/physics/ccd.test.ts            # single file
pnpm vitest run tests/physics/ccd.test.ts -t "name"  # single test
pnpm test:physics               # tests/physics only
pnpm test:e2e                   # headless Chromium smoke test over all demo scenes (needs `npx playwright install chromium`)
pnpm test:ragdoll               # 500-drop ragdoll run (slow)
pnpm check                      # tsc --noEmit + eslint + prettier --check
pnpm verify                     # check + test + bench:quick
pnpm bench / bench:quick / bench:check   # benchmark scenes; bench:check compares with bench/results/baseline.json
pnpm bench:alloc                # allocation-tracking run (node --expose-gc)
pnpm bench:accuracy             # 100k-sample narrowphase accuracy run
pnpm dev                        # vite demo at http://localhost:5173
```

The `test:ragdoll` and `bench:accuracy` scripts use POSIX env-var syntax; on Windows run them from Git Bash.

## Architecture (big picture)

`World.step(dt)` (`src/world/world.ts`) orchestrates the whole pipeline: broadphase update (dynamic AABB tree with fat AABBs, `src/collision/broadphase`) → narrowphase per pair (`src/collision/narrowphase`: analytic paths for sphere/capsule pairs, SAT + clipping for box–box, GJK/EPA for everything else, `triangleContact.ts` for mesh triangles) → velocity integration (gyroscopic term) → sequential-impulse contact solve with warm starting (`src/solver/contactSolver.ts`, `batch.ts`) and joint rows (`src/constraints`) → position integration → NGS position correction → union-find islands and sleeping. `World.advance(frameTime)` is the fixed-step accumulator with interpolation.

Cross-cutting pieces that span several files:

- **CCD** (`src/collision/toi.ts`): conservative-advancement sweep time of impact (default) or speculative contacts, selected by `World.ccdMode`; only bodies with `ccd: true`. Queries (`raycast`, `shapeCast`, `overlap`) reuse the same GJK/TOI code.
- **Shapes** (`src/shapes`): sphere, box, capsule, convex hull (quickhull), compound, static triangle mesh with BVH (`mesh.ts`, `meshBvh.ts`). Shapes expose support functions consumed by GJK/EPA.
- **Determinism**: engine must be bit-exact reproducible. No `Math.random`/clock/ambient input (a test greps the sources), contacts solved sorted by broadphase pair key, no `Map`/`Set` insertion-order dependence. `snapshot()/restore()/stateHash()` must cover any new state you add to bodies, contacts or joints, or the determinism tests will fail.

## Conventions that are enforced

- TypeScript strict with `noUncheckedIndexedAccess` and `exactOptionalPropertyTypes`; no `any`. `tsconfig` also covers `tests`, `bench`, `demo`.
- Hot paths (broadphase, narrowphase, solver, step) must not allocate: use preallocated pools and typed-array scratch buffers, and `out`-parameter math functions. Verify with `pnpm bench:alloc` after touching them.
- Never loosen a test tolerance to make it pass without a justification in `PROGRESS.md` grounded in the numerical method's error bounds; find the root cause instead.
- Performance regressions are caught by `pnpm bench:check` against `bench/results/baseline.json`.
- Math tests require 100% line coverage of `src/math` (`pnpm test:coverage`).

## Things that are easy to get wrong

- `pnpm check` runs prettier on the whole tree; on Windows with `core.autocrlf` it flags CRLF files you did not edit (`PLAN.md`, `tsconfig.json`). Run `npx prettier --write` only on files you changed.
- Timing benchmarks are noisy (up to 25% drift between sessions). Compare old and new code alternately in the same session, and only run one benchmark at a time.
- Sleeping is decided per body from its touching neighbours with hysteresis plus a small pose window (`updateSleep` in `world.ts`); box-box contacts use a 2 mm speculative margin and position correction has its own 5 mm `contactSlop`. Changing any of these interacts with `tests/physics/pile-sleep.test.ts` (400 boxes, seeds in `tests/helpers/pile.ts`), the tower and pyramid tests.
- The box-box narrowphase (`sat.ts`, `boxCache.ts`) is scalar and uses module-level scratch buffers: results must stay bit-exact between runs, and `restore()` relies on contacts being rebuilt on the next step.
- The demo needs WebGL2; with hardware acceleration off it shows a message instead of the scene.
