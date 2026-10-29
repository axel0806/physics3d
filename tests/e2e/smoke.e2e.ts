import { mkdirSync } from 'node:fs';
import { join } from 'node:path';
import { chromium, type Browser } from 'playwright';
import { createServer, type ViteDevServer } from 'vite';
import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { scenes } from '../../demo/scenes';

/**
 * Loads every demo scene in headless Chromium (software WebGL2), lets it run for 300 frames of
 * 1/60 s (5 s of simulated time) in auto mode, and checks that the page logged no errors, that
 * every body position is finite, and that something was actually drawn. Set SCREENSHOTS=1 to save
 * a final-frame PNG per scene to docs/screens.
 */
let server: ViteDevServer;
let browser: Browser;
let base = '';

beforeAll(async () => {
  server = await createServer({
    configFile: join(process.cwd(), 'demo', 'vite.config.ts'),
    server: { port: 0, strictPort: false },
    logLevel: 'error',
  });
  await server.listen();
  const addr = server.httpServer?.address();
  const port = typeof addr === 'object' && addr ? addr.port : 5173;
  base = `http://localhost:${port}`;
  browser = await chromium.launch({
    headless: true,
    args: [
      '--use-angle=swiftshader',
      '--enable-unsafe-swiftshader',
      '--ignore-gpu-blocklist',
      '--enable-webgl',
    ],
  });
});

afterAll(async () => {
  await browser?.close();
  await server?.close();
});

describe('demo smoke test', () => {
  for (const spec of scenes) {
    it(`${spec.id}: runs 5 s with no console errors and no NaN`, async () => {
      const page = await browser.newPage({ viewport: { width: 960, height: 600 } });
      const errors: string[] = [];
      page.on('console', (m) => {
        if (m.type() === 'error') errors.push(m.text());
      });
      page.on('pageerror', (e) => errors.push(String(e)));
      await page.goto(`${base}/?scene=${spec.id}&auto=1`);
      await page.waitForFunction(() => document.title === 'done', undefined, { timeout: 600_000 });
      const report = (await page.evaluate(
        () => (window as unknown as { __report: unknown }).__report,
      )) as {
        frames: number;
        bodies: number;
        nonFinite: number;
        drawCalls: number;
        instances: number;
        simTime: number;
      };
      if (process.env.SCREENSHOTS === '1') {
        mkdirSync('docs/screens', { recursive: true });
        await page.screenshot({ path: `docs/screens/${spec.id}.png` });
      }
      await page.close();
      expect(errors).toEqual([]);
      expect(report.frames).toBe(300);
      expect(report.nonFinite).toBe(0);
      expect(report.bodies).toBeGreaterThan(0);
      expect(report.drawCalls).toBeGreaterThan(0);
      expect(report.instances).toBeGreaterThan(0);
      expect(report.simTime).toBeGreaterThan(4);
    });
  }
});
