import { describe, expect, it } from 'vitest';
import { Quat, Transform, Vec3 } from '../../src/math';
import * as V from '../../src/math/vec3';
import { gjk } from '../../src/collision/narrowphase';
import { timeOfImpact } from '../../src/collision/toi';
import { Box, Sphere } from '../../src/shapes';
import { World, type ContactEvent } from '../../src/world/world';

const DT = 1 / 120;

describe('suspicion 1: friction tangent basis continuity', () => {
  // CONFIRMED, not fixable inside perpendicular(): by the hairy-ball theorem no continuous tangent field
  // exists on the sphere, so the flip moves but cannot vanish. makeConstraint (world.ts) copies
  // impulseT1/T2 by component into the new basis, which rotates the warm-start friction impulse by up
  // to 90 degrees when the basis flips. The fix belongs there: re-project the old tangent impulse
  // vector onto the new basis. `it.fails` flips to a plain `it` once perpendicular is made smooth.
  it.fails('perpendicular() is locally continuous across |nx| = 0.57735 (flip)', () => {
    const a = Math.asin(0.57735);
    const n0 = new Vec3(Math.sin(a - 1e-6), Math.cos(a - 1e-6), 0);
    const n1 = new Vec3(Math.sin(a + 1e-6), Math.cos(a + 1e-6), 0);
    const t0 = V.perpendicular(new Vec3(), n0);
    const t1 = V.perpendicular(new Vec3(), n1);
    expect(V.length(V.sub(new Vec3(), t0, t1))).toBeLessThan(1e-3);
  });
});

describe('suspicion 2: timeOfImpact misses', () => {
  const opts = { target: 0.005, tolerance: 0.00125 };
  it('a fast sphere grazing along a box face still reports the hit', () => {
    const box = { shape: new Box(5, 0.5, 5), transform: new Transform() };
    // Sphere r = 0.1 whose lowest point sits 0.004 above the (target-inflated) top-face clearance,
    // then dips 0.006 into the target zone over a 40 m travel: a very shallow approach.
    for (const dip of [0.0001, 0.001, 0.006]) {
      const y = 0.5 + 0.1 + 0.005 + 0.002 - dip;
      const s = {
        shape: new Sphere(0.1),
        transform: new Transform(new Vec3(-20, y, 0), new Quat()),
      };
      const d = new Vec3(40, 0, 0);
      // Separation after the sweep is below the target, so a hit must be reported.
      const end = { shape: s.shape, transform: new Transform(new Vec3(20, y, 0), new Quat()) };
      const sepEnd = gjk(end, box).distance;
      if (!(sepEnd < opts.target)) continue;
      // Shallow sweep along the face: separation is constant, only close enough once inside.
      const toi = timeOfImpact(s, box, d, new Vec3(), opts);
      expect(toi, `dip ${dip}`).not.toBeNull();
    }
  });

  it('a sphere sweeping nearly parallel to a face converges within the cap', () => {
    const box = { shape: new Box(50, 0.5, 5), transform: new Transform() };
    // Slope 1e-5: descends 0.002 over 200 m, starting at clearance 0.0075 over the face.
    const s = {
      shape: new Sphere(0.1),
      transform: new Transform(new Vec3(-45, 0.5 + 0.1 + 0.0075, 0), new Quat()),
    };
    const toi = timeOfImpact(s, box, new Vec3(90, -0.0075 - 0.0001, 0), new Vec3(), opts);
    expect(toi).not.toBeNull();
  });

  it('already overlapping at t = 0 returns null by contract (the contact solver owns it)', () => {
    const a = { shape: new Sphere(0.5), transform: new Transform() };
    const b = { shape: new Sphere(0.5), transform: new Transform(new Vec3(0.9, 0, 0), new Quat()) };
    const toi = timeOfImpact(a, b, new Vec3(1, 0, 0), new Vec3(), opts);
    expect(toi).toBeNull();
  });
});

describe('suspicion 3: restore() end events', () => {
  it('restoring to a snapshot without the contact emits end for the vanished contact', () => {
    const w = new World();
    const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
    w.createCollider(floor, new Box(5, 0.5, 5));
    const box = w.createBody({ position: new Vec3(0, 3, 0) });
    w.createCollider(box, new Box(0.5, 0.5, 0.5));
    const snap = w.snapshot();
    const events: ContactEvent['kind'][] = [];
    w.onContact((e) => events.push(e.kind));
    for (let i = 0; i < 240; i++) w.step(DT);
    expect(events).toContain('begin');
    w.restore(snap);
    expect(events).toContain('end');
  });
});

describe('suspicion 4: sleep threshold', () => {
  it('a body drifting at 5 mm/s in zero gravity is put to sleep (velocity zeroed): by design, 1 cm/s', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const b = w.createBody({ linearVelocity: new Vec3(0.005, 0, 0) });
    w.createCollider(b, new Sphere(0.5));
    for (let i = 0; i < 120 * 2; i++) w.step(DT);
    // lengthSq < 1e-4 is a 1 cm/s linear threshold (the squared compare is correct), so slow drift freezes.
    expect(b.awake).toBe(false);
    expect(b.linearVelocity.x).toBe(0);
  });
  it('a body drifting at 5 cm/s stays awake', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const b = w.createBody({ linearVelocity: new Vec3(0.05, 0, 0) });
    w.createCollider(b, new Sphere(0.5));
    for (let i = 0; i < 240; i++) w.step(DT);
    expect(b.awake).toBe(true);
  });
});
