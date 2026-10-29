import { describe, expect, it } from 'vitest';
import { AABB, Quat, Transform, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import { Box, Sphere } from '../../src/shapes';
import { MeshBvh } from '../../src/shapes/meshBvh';
import { World } from '../../src/world/world';
import { mulberry32 } from '../helpers/rng';
import { heightfield } from '../helpers/terrain';

const DT = 1 / 120;

describe('mesh BVH', () => {
  it('returns a superset of the triangles whose boxes overlap a query, in 300 random queries', () => {
    const rand = mulberry32(5);
    const verts: Vec3[] = [];
    const idx: number[] = [];
    for (let t = 0; t < 700; t++) {
      const c = new Vec3(rand() * 20, rand() * 20, rand() * 20);
      for (let k = 0; k < 3; k++) {
        verts.push(new Vec3(c.x + rand(), c.y + rand(), c.z + rand()));
        idx.push(verts.length - 1);
      }
    }
    const bvh = new MeshBvh(verts, idx);
    for (let q = 0; q < 300; q++) {
      const lo = new Vec3(rand() * 20, rand() * 20, rand() * 20);
      const box = new AABB(lo, new Vec3(lo.x + rand() * 4, lo.y + rand() * 4, lo.z + rand() * 4));
      const found = new Set(bvh.query(box, []));
      for (let t = 0; t < 700; t++) {
        const ps = [0, 1, 2].map((k) => verts[idx[3 * t + k] as number] as Vec3);
        const overlaps =
          Math.min(...ps.map((p) => p.x)) <= box.max.x &&
          Math.max(...ps.map((p) => p.x)) >= box.min.x &&
          Math.min(...ps.map((p) => p.y)) <= box.max.y &&
          Math.max(...ps.map((p) => p.y)) >= box.min.y &&
          Math.min(...ps.map((p) => p.z)) <= box.max.z &&
          Math.max(...ps.map((p) => p.z)) >= box.min.z;
        if (overlaps) expect(found.has(t), `query ${q} triangle ${t}`).toBe(true);
      }
    }
  });

  it('builds a 100k-triangle mesh and queries it', () => {
    const mesh = heightfield(224, 50, (x, z) => Math.sin(x * 0.3) * Math.cos(z * 0.2));
    expect(mesh.triangleCount).toBeGreaterThan(100_000);
    const hits = mesh.bvh.query(new AABB(new Vec3(-1, -5, -1), new Vec3(1, 5, 1)), []);
    expect(hits.length).toBeGreaterThan(0);
    expect(hits.length).toBeLessThan(100);
  });
});

describe('bodies on meshes', () => {
  it('only static bodies can carry a mesh', () => {
    const w = new World();
    const b = w.createBody({});
    expect(() =>
      w.createCollider(
        b,
        heightfield(2, 1, () => 0),
      ),
    ).toThrow(RangeError);
  });

  it('a box dropped on a flat mesh rests at its half height and goes to sleep', () => {
    const w = new World();
    const ground = w.createBody({ type: 'static' });
    w.createCollider(
      ground,
      heightfield(8, 10, () => 0),
    );
    const box = w.createBody({ position: new Vec3(0.3, 2, -0.4) });
    w.createCollider(box, new Box(0.5, 0.5, 0.5));
    for (let i = 0; i < 600; i++) w.step(DT);
    expect(box.position.y).toBeGreaterThan(0.495);
    expect(box.position.y).toBeLessThan(0.505);
    expect(box.awake).toBe(false);
  });

  it('a sphere rolls down a 20 degree ramp mesh and stops on the flat', () => {
    const w = new World();
    const ground = w.createBody({ type: 'static' });
    const slope = Math.tan((20 * Math.PI) / 180);
    w.createCollider(
      ground,
      heightfield(120, 30, (x) => (x < 0 ? -x * slope : 0)),
      { friction: 0.8 },
    );
    const ball = w.createBody({ position: new Vec3(-4, 4 * slope + 0.6, 0) });
    w.createCollider(ball, new Sphere(0.5), { friction: 0.8 });
    for (let i = 0; i < 600; i++) w.step(DT);
    expect(ball.position.x).toBeGreaterThan(0.5);
    expect(ball.position.y).toBeGreaterThan(0.49);
    expect(ball.position.y).toBeLessThan(0.53);
  });

  it('24 boxes and spheres dropped on bumpy terrain end up on it, none below the surface', () => {
    // A bowl with bumps, so nothing rolls off the edge.
    const height = (x: number, z: number) =>
      0.4 * Math.sin(x * 0.7) * Math.cos(z * 0.6) + 0.05 * (x * x + z * z);
    const mesh = heightfield(60, 8, height);
    const w = new World();
    const ground = w.createBody({ type: 'static' });
    w.createCollider(ground, mesh, { friction: 0.7 });
    const rand = mulberry32(9);
    const bodies = [];
    for (let i = 0; i < 24; i++) {
      const b = w.createBody({
        position: new Vec3(rand() * 10 - 5, 2 + rand() * 4, rand() * 10 - 5),
        orientation: Q.fromAxisAngle(
          new Quat(),
          new Vec3(rand(), rand(), rand() + 0.1),
          rand() * 6,
        ),
      });
      w.createCollider(b, i % 2 ? new Sphere(0.3) : new Box(0.3, 0.3, 0.3), { friction: 0.7 });
      bodies.push(b);
    }
    for (let i = 0; i < 600; i++) w.step(DT);
    for (const b of bodies) {
      const surface = height(b.position.x, b.position.z);
      // Centre at least ~0.25 above the surface (a sphere is 0.3, a box 0.3-0.5, minus slope sag),
      // and still on the terrain, not fallen through or flung away.
      expect(b.position.y - surface, `body ${b.id}`).toBeGreaterThan(0.2);
      expect(b.position.y - surface).toBeLessThan(1.2);
    }
  }, 300_000);

  it('a mesh pair gives begin and end events and wakes a sleeping box when removed', () => {
    const w = new World();
    const ground = w.createBody({ type: 'static' });
    const gc = w.createCollider(
      ground,
      heightfield(4, 5, () => 0),
    );
    const box = w.createBody({ position: new Vec3(0, 0.6, 0) });
    w.createCollider(box, new Box(0.5, 0.5, 0.5));
    const kinds: string[] = [];
    w.onContact((e) => kinds.push(e.kind));
    for (let i = 0; i < 30; i++) w.step(DT);
    expect(kinds[0]).toBe('begin');
    w.destroyCollider(gc);
    expect(kinds).toContain('end');
    for (let i = 0; i < 120; i++) w.step(DT);
    expect(box.position.y).toBeLessThan(-3);
  });
});

describe('queries against meshes', () => {
  const world = () => {
    const w = new World();
    const ground = w.createBody({ type: 'static' });
    w.createCollider(
      ground,
      heightfield(20, 10, (x) => 0.1 * x),
    );
    return w;
  };

  it('raycast hits the sloped surface at the analytic distance', () => {
    const w = world();
    const hit = w.raycast(new Vec3(2, 5, 1), new Vec3(0, -1, 0), 20);
    expect(hit!.distance).toBeCloseTo(5 - 0.2, 5);
    expect(hit!.normal.y).toBeGreaterThan(0.99);
    expect(w.raycast(new Vec3(2, -1, 1), new Vec3(0, -1, 0), 20)).toBeNull();
  });

  it('shapeCast and overlap see the mesh', () => {
    const w = world();
    const hit = w.shapeCast(new Sphere(0.5), new Transform(new Vec3(0, 4, 0)), new Vec3(0, -6, 0));
    expect(hit).not.toBeNull();
    // Slope 0.1 -> surface normal tilted by atan(0.1); sphere centre stops 0.5 above the plane.
    expect(4 - hit!.fraction * 6).toBeCloseTo(0.5 / Math.cos(Math.atan(0.1)), 3);
    expect(w.overlap(new Sphere(0.5), new Transform(new Vec3(0, 0.3, 0)))).toHaveLength(1);
    expect(w.overlap(new Sphere(0.5), new Transform(new Vec3(0, 3, 0)))).toHaveLength(0);
  });
});

describe('mesh state', () => {
  it('snapshot and restore reproduce a terrain scene exactly', () => {
    const build = () => {
      const w = new World();
      const g = w.createBody({ type: 'static' });
      w.createCollider(
        g,
        heightfield(30, 6, (x, z) => 0.3 * Math.sin(x) * Math.cos(z)),
      );
      const rand = mulberry32(21);
      for (let i = 0; i < 20; i++) {
        const b = w.createBody({
          position: new Vec3(rand() * 8 - 4, 1 + rand() * 3, rand() * 8 - 4),
        });
        w.createCollider(b, i % 2 ? new Sphere(0.25) : new Box(0.25, 0.25, 0.25));
      }
      return w;
    };
    const a = build();
    for (let i = 0; i < 300; i++) a.step(DT);
    const snap = a.snapshot();
    const reference: string[] = [];
    for (let i = 0; i < 200; i++) {
      a.step(DT);
      reference.push(a.stateHash());
    }
    const b = build();
    b.restore(snap);
    const again: string[] = [];
    for (let i = 0; i < 200; i++) {
      b.step(DT);
      again.push(b.stateHash());
    }
    expect(again).toEqual(reference);
  });
});
