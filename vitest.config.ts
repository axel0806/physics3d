import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['tests/**/*.test.ts'],
    // Property and physics tests run thousands of cases; the 5 s default fails under parallel load.
    testTimeout: 60_000,
    coverage: { provider: 'v8', include: ['src/math/**'], thresholds: { lines: 100 } },
  },
});
