import { it, expect } from 'vitest';
import { Vec3, Transform, Quat } from '../../src/math';
import {
  pointTriangleDistance,
  closestPointOnTriangle,
  boxCorners,
  referenceMinkowski,
} from '../helpers/reference';
import { Vec3 as _V } from '../../src/math';
it('triangle distance regions', () => {
  const a = new Vec3(1, 0, 0),
    b = new Vec3(0, 1, 0),
    c = new Vec3(0, 0, 1);
  expect(pointTriangleDistance(new Vec3(), a, b, c)).toBeCloseTo(1 / Math.sqrt(3), 12); // origin projects inside face
  expect(
    pointTriangleDistance(new Vec3(), new Vec3(2, 0, 0), new Vec3(3, 0, 0), new Vec3(2, 1, 0)),
  ).toBeCloseTo(2, 12);
  const q = closestPointOnTriangle(new Vec3(), new Vec3(0.2, 0.2, 5), a, b, c);
  // (0.2, 0.2, 5) is closest to vertex c = (0, 0, 1).
  expect(q.x).toBeCloseTo(0, 12);
  expect(q.y).toBeCloseTo(0, 12);
  expect(q.z).toBeCloseTo(1, 12);
  expect(_V).toBeDefined();
  // box corners vs origin-centred unit cube: origin inside, depth 1 (distance to nearest face)
  const cube = boxCorners([1, 1, 1], new Transform(new Vec3(), new Quat()));
  const r = referenceMinkowski(
    cube,
    cube.map((p) => p),
  );
  expect(r.inside).toBe(true);
});
