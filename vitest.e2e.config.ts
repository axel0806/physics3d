import { defineConfig } from 'vitest/config';

// Browser smoke test: needs Chromium (`npx playwright install chromium`). Run with `pnpm test:e2e`.
export default defineConfig({
  test: { include: ['tests/e2e/**/*.e2e.ts'], testTimeout: 900_000, hookTimeout: 120_000 },
});
