import { mkdirSync, readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { scenes } from './scenes';

/**
 * Benchmark runner. For each scene: build, step a warm-up, then time `steps` fixed steps at 60 Hz
 * and report median / p99 / max step time with a per-phase breakdown (medians of the phases).
 *
 *   tsx bench/run.ts                run all scenes, write bench/results/latest.json
 *   tsx bench/run.ts --quick        fewer steps (smoke)
 *   tsx bench/run.ts --scene NAME   one scene
 *   tsx bench/run.ts --baseline     also write bench/results/baseline.json
 *   tsx bench/run.ts --check        compare with the baseline; exit 1 if a scene's median got >10% slower
 */
const args = process.argv.slice(2);
const quick = args.includes('--quick');
const check = args.includes('--check');
const writeBaseline = args.includes('--baseline');
const only = args.includes('--scene') ? args[args.indexOf('--scene') + 1] : undefined;
const DT = 1 / 60;
const WARMUP = process.env.BENCH_WARMUP ? Number(process.env.BENCH_WARMUP) : quick ? 20 : 60;
const STEPS = quick ? 60 : 300;
const RESULTS = join(import.meta.dirname, 'results');

interface SceneResult {
  scene: string;
  bodies: number;
  steps: number;
  medianMs: number;
  p99Ms: number;
  maxMs: number;
  phases: { broadphase: number; narrowphase: number; solve: number; integrate: number };
  contactsAtEnd: number;
  awakeAtEnd: number;
}

function percentile(sorted: number[], p: number): number {
  return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))] as number;
}

function runScene(name: string): SceneResult {
  const scene = scenes.find((s) => s.name === name);
  if (!scene) throw new Error(`unknown scene ${name}`);
  const world = scene.build();
  world.clock = () => performance.now();
  if (process.env.BENCH_TOL) world.velocityTolerance = Number(process.env.BENCH_TOL);
  if (process.env.BENCH_ITERS) world.velocityIterations = Number(process.env.BENCH_ITERS);
  if (process.env.BENCH_POS) world.positionIterations = Number(process.env.BENCH_POS);
  for (let i = 0; i < WARMUP; i++) world.step(DT);
  const total: number[] = [];
  const phases = {
    broadphase: [] as number[],
    narrowphase: [] as number[],
    solve: [] as number[],
    integrate: [] as number[],
  };
  for (let i = 0; i < STEPS; i++) {
    const t0 = performance.now();
    world.step(DT);
    total.push(performance.now() - t0);
    for (const k of Object.keys(phases) as (keyof typeof phases)[])
      phases[k].push(world.timings[k]);
  }
  const sorted = [...total].sort((a, b) => a - b);
  const med = (xs: number[]) =>
    percentile(
      [...xs].sort((a, b) => a - b),
      0.5,
    );
  return {
    scene: name,
    bodies: world.bodies.length,
    steps: STEPS,
    medianMs: percentile(sorted, 0.5),
    p99Ms: percentile(sorted, 0.99),
    maxMs: sorted[sorted.length - 1] as number,
    phases: {
      broadphase: med(phases.broadphase),
      narrowphase: med(phases.narrowphase),
      solve: med(phases.solve),
      integrate: med(phases.integrate),
    },
    contactsAtEnd: world.activeContacts().length,
    awakeAtEnd: world.bodies.filter((b) => b.awake && b.isDynamic).length,
  };
}

const names = only ? [only] : scenes.map((s) => s.name);
const results: SceneResult[] = [];
for (const name of names) {
  const r = runScene(name);
  results.push(r);
  const f = (x: number) => x.toFixed(2).padStart(7);
  console.log(
    `${name.padEnd(20)} ${String(r.bodies).padStart(5)} bodies  median ${f(r.medianMs)} ms  p99 ${f(r.p99Ms)} ms  max ${f(r.maxMs)} ms` +
      `  [bp ${r.phases.broadphase.toFixed(2)} np ${r.phases.narrowphase.toFixed(2)} solve ${r.phases.solve.toFixed(2)} int ${r.phases.integrate.toFixed(2)}]` +
      `  contacts ${r.contactsAtEnd}, awake ${r.awakeAtEnd}`,
  );
}

mkdirSync(RESULTS, { recursive: true });
if (!quick && !only) {
  const out = { node: process.version, date: new Date().toISOString(), results };
  writeFileSync(join(RESULTS, 'latest.json'), JSON.stringify(out, null, 2));
  if (writeBaseline) writeFileSync(join(RESULTS, 'baseline.json'), JSON.stringify(out, null, 2));
}

if (check) {
  const path = join(RESULTS, 'baseline.json');
  if (!existsSync(path)) {
    console.error('no baseline: run `tsx bench/run.ts --baseline` first');
    process.exit(1);
  }
  const base = JSON.parse(readFileSync(path, 'utf8')) as { results: SceneResult[] };
  let failed = false;
  for (const r of results) {
    const b = base.results.find((x) => x.scene === r.scene);
    if (!b) continue;
    const ratio = r.medianMs / b.medianMs;
    const bad = ratio > 1.1;
    if (bad) failed = true;
    console.log(
      `${bad ? 'REGRESSION' : 'ok        '} ${r.scene}: ${r.medianMs.toFixed(2)} ms vs baseline ${b.medianMs.toFixed(2)} ms (${((ratio - 1) * 100).toFixed(1)}%)`,
    );
  }
  if (failed) process.exit(1);
}
