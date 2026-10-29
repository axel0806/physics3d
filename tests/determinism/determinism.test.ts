import { spawn } from 'node:child_process';
import { readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { randomScene } from '../helpers/scene';

const FULL = process.env.DETERMINISM_FULL === '1';
const SCENES = FULL ? 50 : 6;
const STEPS = FULL ? 10_000 : 1500;
const DT = 1 / 120;
const SCRIPT = fileURLToPath(new URL('./run-scenes.ts', import.meta.url));

function runInProcess(seed: number, steps: number): string {
  const world = randomScene(seed);
  for (let i = 0; i < steps; i++) world.step(DT);
  return world.stateHash();
}

/** Runs the scene runner in a fresh Node process and parses its "<seed> <hash>" lines. */
function runInChild(first: number, count: number, steps: number): Promise<Map<number, string>> {
  return new Promise((resolve, reject) => {
    const child = spawn(
      process.execPath,
      ['--import', 'tsx', SCRIPT, String(first), String(count), String(steps)],
      { stdio: ['ignore', 'pipe', 'inherit'] },
    );
    let out = '';
    child.stdout.on('data', (d) => (out += d));
    child.on('error', reject);
    child.on('close', (code) => {
      if (code !== 0) return reject(new Error(`child exited with ${code}`));
      const m = new Map<number, string>();
      for (const line of out.trim().split('\n')) {
        const [seed, hash] = line.split(' ');
        m.set(Number(seed), hash as string);
      }
      resolve(m);
    });
  });
}

describe('determinism', () => {
  it(`${SCENES} randomized scenes give identical hashes after ${STEPS} steps, run twice in-process`, () => {
    for (let seed = 0; seed < SCENES; seed++) {
      expect(runInProcess(seed, STEPS), `scene ${seed}`).toBe(runInProcess(seed, STEPS));
    }
  }, 7_200_000);

  it('two separate processes agree with each other and with this process', async () => {
    const half = Math.ceil(SCENES / 2);
    const [a1, a2, b1, b2] = await Promise.all([
      runInChild(0, half, STEPS),
      runInChild(half, SCENES - half, STEPS),
      runInChild(0, half, STEPS),
      runInChild(half, SCENES - half, STEPS),
    ]);
    const first = new Map([...a1, ...a2]);
    const second = new Map([...b1, ...b2]);
    expect(first.size).toBe(SCENES);
    expect([...second]).toEqual([...first]);
    // And this process matches too (sampled: the in-process test above covers all scenes).
    for (const seed of [0, SCENES - 1]) expect(runInProcess(seed, STEPS)).toBe(first.get(seed));
  }, 7_200_000);

  it('the hash sees a one-bit change in any body', () => {
    const w = randomScene(3);
    const before = w.stateHash();
    const v = w.bodies[4]!.linearVelocity;
    v.x = Number.EPSILON + v.x;
    expect(w.stateHash()).not.toBe(before);
  });
});

describe('engine sources', () => {
  it('use no Math.random, clocks or other ambient nondeterminism', () => {
    const root = fileURLToPath(new URL('../../src', import.meta.url));
    const files: string[] = [];
    const walk = (dir: string): void => {
      for (const e of readdirSync(dir, { withFileTypes: true })) {
        if (e.isDirectory()) walk(join(dir, e.name));
        else if (e.name.endsWith('.ts')) files.push(join(dir, e.name));
      }
    };
    walk(root);
    expect(files.length).toBeGreaterThan(20);
    for (const f of files) {
      const text = readFileSync(f, 'utf8').replace(/\/\*[\s\S]*?\*\/|\/\/.*$/gm, '');
      expect(text, f).not.toMatch(/Math\.random|Date\.now|performance\.now|new Date\(|crypto\./);
    }
  });
});

describe('snapshot and restore', () => {
  it('re-simulating from a snapshot reproduces every later state hash exactly', () => {
    const world = randomScene(11);
    for (let i = 0; i < 400; i++) world.step(DT);
    const snap = world.snapshot();
    const reference: string[] = [];
    for (let i = 0; i < 300; i++) {
      world.step(DT);
      reference.push(world.stateHash());
    }
    world.restore(snap);
    const again: string[] = [];
    for (let i = 0; i < 300; i++) {
      world.step(DT);
      again.push(world.stateHash());
    }
    expect(again).toEqual(reference);
  });

  it('a snapshot restores into a separately built identical world', () => {
    const a = randomScene(12);
    for (let i = 0; i < 300; i++) a.step(DT);
    const snap = a.snapshot();
    const b = randomScene(12);
    b.restore(snap);
    expect(b.stateHash()).toBe(a.stateHash());
    for (let i = 0; i < 200; i++) {
      a.step(DT);
      b.step(DT);
    }
    expect(b.stateHash()).toBe(a.stateHash());
  });

  it('refuses a snapshot from a different world', () => {
    expect(() => randomScene(1, 10).restore(randomScene(1, 12).snapshot())).toThrow();
  });
});
