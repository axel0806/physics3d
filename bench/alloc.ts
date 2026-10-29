import { scenes } from './scenes';

/**
 * Allocation check: run with `node --expose-gc --import tsx bench/alloc.ts`. For each scene it warms
 * up, forces a GC, steps 1,000 times, forces another GC, and compares live heap. Garbage that the
 * collector reclaims does not count (the engine still allocates short-lived objects in the
 * narrowphase); what the check catches is retained growth, i.e. leaks in the step loop.
 */
const gc = (globalThis as { gc?: () => void }).gc;
if (!gc) {
  console.error('run with --expose-gc');
  process.exit(2);
}
const quick = process.argv.includes('--quick');
const STEPS = quick ? 200 : 1000;
// Allowed growth: 1% of the live heap plus 1 MB.
let failed = false;
for (const scene of scenes) {
  const world = scene.build();
  for (let i = 0; i < 120; i++) world.step(1 / 60);
  gc();
  const before = process.memoryUsage().heapUsed;
  for (let i = 0; i < STEPS; i++) world.step(1 / 60);
  gc();
  const after = process.memoryUsage().heapUsed;
  const growth = after - before;
  const allowed = 0.01 * before + 1_000_000;
  const ok = growth <= allowed;
  if (!ok) failed = true;
  console.log(
    `${ok ? 'ok  ' : 'LEAK'} ${scene.name.padEnd(20)} heap ${(before / 1e6).toFixed(1)} MB -> ${(after / 1e6).toFixed(1)} MB over ${STEPS} steps (growth ${(growth / 1e6).toFixed(2)} MB)`,
  );
}
if (failed) process.exit(1);
