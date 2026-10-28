// Adversarial narrowphase fuzz: touching, coincident centers, near-parallel faces, thin boxes
// (1e-3) and huge scale ratios (1e4). Asserts finite outputs; reports iteration caps hit.
import { Vec3, Quat, Transform } from '../src/math';
import { Box, Sphere } from '../src/shapes';
import { collideConvex } from '../src/collision/narrowphase';

const ITER = Number(process.env.FUZZ_ITERATIONS ?? 1_000_000);
let seed = 0x9e3779b9;
const rand = (): number => {
  seed ^= seed << 13;
  seed ^= seed >>> 17;
  seed ^= seed << 5;
  return ((seed >>> 0) % 1_000_000) / 1_000_000;
};
const randQuat = (): Quat => {
  const x = rand() * 2 - 1,
    y = rand() * 2 - 1,
    z = rand() * 2 - 1,
    w = rand() * 2 - 1;
  const l = Math.hypot(x, y, z, w) || 1;
  return new Quat(x / l, y / l, z / l, w / l);
};

let nonFinite = 0;
let capHits = 0;
let throws = 0;
const t0 = performance.now();
for (let k = 0; k < ITER; k++) {
  const scale = [1, 1e-3, 1e4][k % 3] as number;
  const kind = k % 4;
  let A: { shape: Box | Sphere; transform: Transform };
  let B: { shape: Box | Sphere; transform: Transform };
  if (kind === 0) {
    // Touching faces.
    A = {
      shape: new Box(scale * (0.5 + rand()), scale * 1e-3, scale * 0.5),
      transform: new Transform(new Vec3(), randQuat()),
    };
    B = {
      shape: new Box(scale * 0.5, scale * 0.5, scale * 0.5),
      transform: new Transform(new Vec3(0, scale * 1e-3 + scale * 0.5, 0), new Quat()),
    };
  } else if (kind === 1) {
    // Coincident centers.
    A = { shape: new Sphere(scale), transform: new Transform(new Vec3(), new Quat()) };
    B = {
      shape: new Box(scale, scale * 0.3, scale),
      transform: new Transform(new Vec3(), randQuat()),
    };
  } else if (kind === 2) {
    // Near-parallel faces, thin boxes.
    const tilt = 1e-6 * rand();
    A = {
      shape: new Box(scale, scale * 1e-3, scale),
      transform: new Transform(new Vec3(), new Quat(Math.sin(tilt), 0, 0, Math.cos(tilt))),
    };
    B = {
      shape: new Box(scale, scale * 1e-3, scale),
      transform: new Transform(new Vec3(scale * 0.1, scale * 1.5e-3, 0), new Quat()),
    };
  } else {
    // Large scale ratio between the two shapes.
    A = { shape: new Sphere(scale * 1e4), transform: new Transform(new Vec3(), new Quat()) };
    B = {
      shape: new Box(scale * 0.3, scale * 0.3, scale * 0.3),
      transform: new Transform(new Vec3(scale * 1e4 * (0.5 + rand()), 0, 0), randQuat()),
    };
  }
  try {
    const c = collideConvex(A, B);
    const values = c.overlap
      ? [c.depth, c.normal.x, c.normal.y, c.normal.z]
      : [c.distance, c.normal.x, c.normal.y, c.normal.z];
    if (!values.every(Number.isFinite)) nonFinite++;
    if (c.overlap && !c.converged) capHits++;
  } catch (e) {
    throws++;
    if (throws <= 5) console.error('throw at', k, (e as Error).message);
  }
}
const secs = (performance.now() - t0) / 1000;
console.log(
  JSON.stringify({
    iterations: ITER,
    nonFinite,
    epaNotConverged: capHits,
    throws,
    seconds: Math.round(secs),
  }),
);
if (nonFinite > 0 || throws > 0) process.exit(1);
