import { randomScene } from '../helpers/scene';

/**
 * Child-process entry for the determinism test: runs scenes [first, first + count) for `steps`
 * steps at 120 Hz and prints "<seed> <hash>" per scene.
 */
const [first, count, steps] = process.argv.slice(2).map(Number) as [number, number, number];
for (let seed = first; seed < first + count; seed++) {
  const world = randomScene(seed);
  for (let i = 0; i < steps; i++) world.step(1 / 120);
  console.log(`${seed} ${world.stateHash()}`);
}
