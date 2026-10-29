import { Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import { gjk, type Placed } from './narrowphase';

export interface TimeOfImpact {
  /** Fraction of the displacement at which the shapes first come within `target` of each other. */
  readonly t: number;
  /** Unit normal from A to B at that time. */
  readonly normal: Vec3;
  /** Closest points on A and B (world space) at that time. */
  readonly pointA: Vec3;
  readonly pointB: Vec3;
}

export interface ToiOptions {
  /** Separation at which the shapes count as touching. */
  readonly target: number;
  /** Accept a hit when the separation is within this much of `target`. */
  readonly tolerance: number;
  readonly maxIterations?: number;
}

function shifted(p: Placed, d: Vec3, t: number): Placed {
  const tr = new Transform(V.addScaled(new Vec3(), p.transform.p, d, t), p.transform.q);
  return { shape: p.shape, transform: tr };
}

/**
 * Time of impact of two convex shapes translating by displacements `dA` and `dB` over t ∈ [0, 1],
 * by conservative advancement: the separation d(t) is convex for pure translation, so advancing
 * by (d − target) / closing-speed never overshoots. Returns null when the shapes do not come
 * within `target` during the interval, or already overlap at t = 0 (the contact solver owns that).
 * Rotation during the interval is not swept.
 */
export function timeOfImpact(
  A: Placed,
  B: Placed,
  dA: Vec3,
  dB: Vec3,
  options: ToiOptions,
): TimeOfImpact | null {
  const rel = V.sub(new Vec3(), dA, dB);
  const maxIter = options.maxIterations ?? 64;
  let t = 0;
  for (let i = 0; i < maxIter; i++) {
    const r = gjk(shifted(A, dA, t), shifted(B, dB, t));
    if (r.overlap) {
      // Advancement cannot overshoot, so this is rounding at the target distance: report a hit
      // along the direction of relative motion.
      if (t === 0) return null;
      const n = V.normalize(new Vec3(), rel);
      return { t, normal: n, pointA: r.pointA, pointB: r.pointB };
    }
    if (!(r.distance > 0))
      return t === 0
        ? null
        : { t, normal: V.normalize(new Vec3(), rel), pointA: r.pointA, pointB: r.pointB };
    const gap = r.distance - options.target;
    const n = V.sub(new Vec3(), r.pointB, r.pointA);
    V.scale(n, n, 1 / r.distance);
    const closing = V.dot(rel, n);
    if (closing <= 1e-12) return null;
    if (gap <= options.tolerance) return { t, normal: n, pointA: r.pointA, pointB: r.pointB };
    t += gap / closing;
    if (t > 1) return null;
  }
  return null;
}
