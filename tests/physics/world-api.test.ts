import { describe, expect, it } from 'vitest';
import { Quat, Transform, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import { Box, Capsule, Sphere } from '../../src/shapes';
import { World, type ContactEvent } from '../../src/world/world';
import { BallJoint } from '../../src/constraints';

const DT = 1 / 120;

describe('destroying things', () => {
  it('removing the support makes a resting box fall, with an end event', () => {
    const w = new World();
    const floor = w.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
    w.createCollider(floor, new Box(5, 0.5, 5));
    const box = w.createBody({ position: new Vec3(0, 0.5, 0) });
    w.createCollider(box, new Box(0.5, 0.5, 0.5));
    const events: ContactEvent['kind'][] = [];
    w.onContact((e) => events.push(e.kind));
    for (let i = 0; i < 30; i++) w.step(DT);
    expect(box.position.y).toBeCloseTo(0.5, 2);
    expect(events).toContain('begin');
    w.destroyBody(floor);
    expect(events).toContain('end');
    for (let i = 0; i < 120; i++) w.step(DT);
    expect(box.position.y).toBeLessThan(-3);
    expect(w.bodies).toEqual([box]);
    expect(w.colliders.length).toBe(1);
  });

  it('destroying a body removes its joints and wakes the partner', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const a = w.createBody({ position: new Vec3(0, 0, 0) });
    w.createCollider(a, new Sphere(0.2));
    const b = w.createBody({ position: new Vec3(1, 0, 0) });
    w.createCollider(b, new Sphere(0.2));
    w.addJoint(new BallJoint(a, b, new Vec3(0.5, 0, 0)));
    b.awake = false;
    w.destroyBody(a);
    expect(w.joints.length).toBe(0);
    expect(b.awake).toBe(true);
    expect(b.jointCount).toBe(0);
    w.step(DT);
  });

  it('a destroyed collider no longer shows in queries', () => {
    const w = new World();
    const b = w.createBody({ type: 'static' });
    const c = w.createCollider(b, new Sphere(1));
    expect(w.raycast(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 10)?.collider).toBe(c);
    w.destroyCollider(c);
    expect(w.raycast(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 10)).toBeNull();
  });
});

describe('fixed timestep and interpolation', () => {
  it('runs whole steps and returns the leftover fraction', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    expect(w.advance(0.0125)).toBeCloseTo(0.0125 / w.fixedDt - 1, 9);
    expect(w.frame).toBe(1);
    expect(w.advance(0.001)).toBeCloseTo((0.0125 + 0.001) / w.fixedDt - 1, 9);
    expect(w.frame).toBe(1);
    w.advance(0.06);
    expect(w.frame).toBe(1 + 7);
  });

  it('drops time beyond maxSubSteps so a long frame cannot spiral', () => {
    const w = new World();
    w.maxSubSteps = 4;
    const alpha = w.advance(10);
    expect(w.frame).toBe(4);
    expect(alpha).toBe(0);
  });

  it('interpolates between the previous and current pose', () => {
    const w = new World({ gravity: new Vec3(0, 0, 0) });
    const b = w.createBody({ linearVelocity: new Vec3(1.2, 0, 0) });
    w.createCollider(b, new Sphere(0.1));
    w.step(DT);
    const start = b.prevPosition.x;
    const end = b.position.x;
    expect(end - start).toBeCloseTo(1.2 * DT, 12);
    const mid = w.interpolatedTransform(b, 0.5);
    expect(mid.p.x).toBeCloseTo((start + end) / 2, 12);
    expect(w.interpolatedTransform(b, 0).p.x).toBe(start);
    expect(w.interpolatedTransform(b, 1).p.x).toBe(end);
  });
});

describe('queries', () => {
  const scene = () => {
    const w = new World();
    const ball = w.createBody({ type: 'static', position: new Vec3(0, 0, 0) });
    w.createCollider(ball, new Sphere(1), { layer: 1 });
    const box = w.createBody({ type: 'static', position: new Vec3(5, 0, 0) });
    w.createCollider(box, new Box(1, 1, 1), { layer: 2 });
    const cap = w.createBody({ type: 'static', position: new Vec3(10, 0, 0) });
    w.createCollider(cap, new Capsule(0.5, 1), { layer: 4 });
    return w;
  };

  it('raycast hits a sphere at the analytic distance and normal', () => {
    const w = scene();
    const hit = w.raycast(new Vec3(-4, 0.3, 0), new Vec3(2, 0, 0), 20);
    expect(hit).not.toBeNull();
    const x = -Math.sqrt(1 - 0.09);
    expect(hit!.point.x).toBeCloseTo(x, 5);
    expect(hit!.distance).toBeCloseTo(x + 4, 5);
    expect(hit!.normal.x).toBeCloseTo(x, 5);
    expect(hit!.normal.y).toBeCloseTo(0.3, 5);
  });

  it('raycast picks the closest of several hits and respects maxDistance', () => {
    const w = scene();
    const hit = w.raycast(new Vec3(-4, 0, 0), new Vec3(1, 0, 0), 20);
    expect(hit!.collider.shape).toBeInstanceOf(Sphere);
    expect(w.raycast(new Vec3(2, 0, 0), new Vec3(1, 0, 0), 1.5)).toBeNull();
    const boxHit = w.raycast(new Vec3(2, 0, 0), new Vec3(1, 0, 0), 20);
    expect(boxHit!.collider.shape).toBeInstanceOf(Box);
    expect(boxHit!.distance).toBeCloseTo(2, 5);
    expect(boxHit!.normal.x).toBeCloseTo(-1, 5);
  });

  it('raycast hits a capsule on its curved cap', () => {
    const w = scene();
    // Vertical ray down onto the top cap of the capsule at x = 10: apex at y = 1.5.
    const hit = w.raycast(new Vec3(10, 5, 0), new Vec3(0, -1, 0), 20);
    expect(hit!.distance).toBeCloseTo(3.5, 5);
    expect(hit!.normal.y).toBeCloseTo(1, 5);
  });

  it('mask filters by collider layer, predicate by collider', () => {
    const w = scene();
    const o = new Vec3(-4, 0, 0);
    const d = new Vec3(1, 0, 0);
    expect(w.raycast(o, d, 20, { mask: 2 })!.collider.shape).toBeInstanceOf(Box);
    expect(w.raycast(o, d, 20, { mask: 4 })!.collider.shape).toBeInstanceOf(Capsule);
    expect(w.raycast(o, d, 20, { mask: 8 })).toBeNull();
    const hit = w.raycast(o, d, 20, { predicate: (c) => c.shape instanceof Capsule });
    expect(hit!.collider.shape).toBeInstanceOf(Capsule);
  });

  it('a ray starting inside a shape reports no hit for it', () => {
    const w = scene();
    const hit = w.raycast(new Vec3(0, 0, 0), new Vec3(1, 0, 0), 20);
    expect(hit!.collider.shape).toBeInstanceOf(Box);
  });

  it('shapeCast of a sphere stops at the combined radius', () => {
    const w = scene();
    const hit = w.shapeCast(new Sphere(0.5), new Transform(new Vec3(-5, 0, 0)), new Vec3(10, 0, 0));
    expect(hit!.fraction * 10).toBeCloseTo(5 - 1.5, 5);
    expect(hit!.collider.shape).toBeInstanceOf(Sphere);
    expect(hit!.normal.x).toBeCloseTo(1, 5);
  });

  it('shapeCast of a rotated box finds the box target', () => {
    const w = scene();
    const q = Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), Math.PI / 4);
    const t = new Transform(new Vec3(5.5, 3, 0), q);
    const hit = w.shapeCast(new Box(0.5, 0.5, 0.5), t, new Vec3(0, -3, 0));
    // Diagonal box: lowest corner is 0.5*sqrt(2) below its center, box top is at y = 1.
    expect(hit!.fraction * 3).toBeCloseTo(3 - 1 - 0.5 * Math.SQRT2, 5);
  });

  it('overlap lists exactly the touching colliders', () => {
    const w = scene();
    const hits = w.overlap(new Sphere(1.5), new Transform(new Vec3(2.5, 0, 0)));
    expect(hits.map((c) => c.shape.kind).sort()).toEqual(['box', 'sphere']);
    expect(w.overlap(new Sphere(0.4), new Transform(new Vec3(2.5, 0, 0)))).toEqual([]);
    expect(w.overlap(new Sphere(1.5), new Transform(new Vec3(2.5, 0, 0)), { mask: 1 }).length).toBe(
      1,
    );
  });

  it('raycast agrees with brute-force analytic spheres on 2,000 random rays', () => {
    const w = new World();
    const spheres: { c: Vec3; r: number }[] = [];
    let s = 12345;
    const rnd = () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 4294967296;
    for (let i = 0; i < 25; i++) {
      const c = new Vec3(rnd() * 20 - 10, rnd() * 20 - 10, rnd() * 20 - 10);
      const r = 0.3 + rnd();
      const b = w.createBody({ type: 'static', position: c });
      w.createCollider(b, new Sphere(r));
      spheres.push({ c, r });
    }
    let hits = 0;
    for (let i = 0; i < 2000; i++) {
      const o = new Vec3(rnd() * 24 - 12, rnd() * 24 - 12, rnd() * 24 - 12);
      const d = new Vec3(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5);
      const len = Math.hypot(d.x, d.y, d.z);
      let best = Infinity;
      for (const { c, r } of spheres) {
        const m = new Vec3(o.x - c.x, o.y - c.y, o.z - c.z);
        const dd = new Vec3(d.x / len, d.y / len, d.z / len);
        const b = m.x * dd.x + m.y * dd.y + m.z * dd.z;
        const cc = m.x * m.x + m.y * m.y + m.z * m.z - r * r;
        if (cc > 0 && b <= 0) {
          const disc = b * b - cc;
          if (disc >= 0) best = Math.min(best, -b - Math.sqrt(disc));
        }
      }
      const hit = w.raycast(o, d, 30);
      if (best <= 30) {
        hits++;
        expect(hit, `ray ${i}`).not.toBeNull();
        expect(hit!.distance).toBeCloseTo(best, 5);
      } else {
        expect(hit, `ray ${i}`).toBeNull();
      }
    }
    expect(hits).toBeGreaterThan(50);
  });
});
