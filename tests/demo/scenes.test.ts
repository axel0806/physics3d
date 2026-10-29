import { describe, expect, it } from 'vitest';
import { scenes } from '../../demo/scenes';

/** Steps at most `maxSteps` (and at most 5 s of simulated time) and returns the first bad body. */
function runScene(id: string): { steps: number; bad: string | null; awake: number } {
  const spec = scenes.find((s) => s.id === id)!;
  const inst = spec.build();
  const dt = spec.fixedDt ?? 1 / 60;
  const maxSteps = Math.min(Math.round(5 / dt), 1500);
  let t = 0;
  for (let i = 0; i < maxSteps; i++) {
    inst.update?.(t, dt);
    inst.world.step(dt);
    t += dt;
  }
  for (const b of inst.world.bodies) {
    const p = b.position;
    const q = b.orientation;
    const v = b.linearVelocity;
    const ok = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z].every(Number.isFinite);
    if (!ok) return { steps: maxSteps, bad: `body ${b.id} not finite`, awake: 0 };
    if (Math.abs(p.x) > 1e3 || Math.abs(p.y) > 1e3 || Math.abs(p.z) > 1e3) {
      return { steps: maxSteps, bad: `body ${b.id} flew away: ${p.x}, ${p.y}, ${p.z}`, awake: 0 };
    }
  }
  return { steps: maxSteps, bad: null, awake: inst.world.bodies.filter((b) => b.awake).length };
}

describe('demo scenes', () => {
  it('has unique ids and the scenes the plan names', () => {
    const ids = scenes.map((s) => s.id);
    expect(new Set(ids).size).toBe(ids.length);
    for (const id of ['jenga', 'dominoes', 'wrecking-ball', 'vehicle', 'ragdoll-cannon', 'throw']) {
      expect(ids).toContain(id);
    }
  });

  for (const spec of scenes) {
    it(`${spec.id}: builds and runs without NaN or runaway bodies`, () => {
      const r = runScene(spec.id);
      expect(r.bad).toBeNull();
    }, 600_000);
  }

  it('the tower scene keeps standing after 5 s', () => {
    const inst = scenes.find((s) => s.id === 'tower')!.build();
    for (let i = 0; i < 600; i++) inst.world.step(1 / 120);
    const top = inst.world.bodies.at(-1)!;
    expect(top.position.y).toBeGreaterThan(19);
    expect(Math.hypot(top.position.x, top.position.z)).toBeLessThan(0.1);
  });

  it('the vehicle drives forward', () => {
    const inst = scenes.find((s) => s.id === 'vehicle')!.build();
    const chassis = inst.world.bodies.find((b) => b.isDynamic)!;
    for (let i = 0; i < 300; i++) inst.world.step(1 / 60);
    expect(Math.abs(chassis.position.x)).toBeGreaterThan(3);
  });
});
