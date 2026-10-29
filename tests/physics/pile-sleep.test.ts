import { describe, expect, it } from 'vitest';
import { buildPile } from '../helpers/pile';

/** Fraction of the dynamic bodies that are asleep. */
function asleepFraction(world: ReturnType<typeof buildPile>): number {
  let dyn = 0;
  let asleep = 0;
  for (const b of world.bodies) {
    if (!b.isDynamic) continue;
    dyn++;
    if (!b.awake) asleep++;
  }
  return asleep / dyn;
}

describe('dense pile sleeping', () => {
  it('400 random boxes in a 9 x 9 m arena settle and sleep within 20 s', () => {
    const w = buildPile(7, 400);
    const dt = 1 / 60;
    const log: string[] = [];
    let allAsleepAt = -1;
    for (let i = 1; i <= 20 * 60; i++) {
      w.step(dt);
      if (i % 120 === 0) log.push(`${(i * dt).toFixed(0)}s:${asleepFraction(w).toFixed(2)}`);
      if (allAsleepAt < 0 && asleepFraction(w) === 1) {
        allAsleepAt = i * dt;
        break;
      }
    }
    if (process.env.PILE_LOG) console.log(log.join(' '), 'allAsleepAt', allAsleepAt);
    expect(allAsleepAt).toBeGreaterThan(0);
    expect(allAsleepAt).toBeLessThanOrEqual(20);
  }, 180_000);
});
