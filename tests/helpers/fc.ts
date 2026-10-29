/**
 * Seed for fast-check property tests. Fixed by default so a failure reproduces and CI is stable;
 * set FC_SEED=<number> (or FC_SEED=random) to explore other inputs.
 */
export const FC_SEED: number =
  process.env.FC_SEED === 'random'
    ? Math.floor(Math.random() * 2 ** 31)
    : Number(process.env.FC_SEED ?? 20260101);
