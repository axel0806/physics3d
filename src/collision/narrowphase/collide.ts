import { Vec3 } from '../../math';
import * as V from '../../math/vec3';
import { epa, type EpaResult } from './epa';
import { gjk, type GjkResult } from './gjk';
import type { Placed } from './support';

export type ConvexContact =
  | {
      readonly overlap: false;
      /** Distance between the surfaces (> 0). */
      readonly distance: number;
      /** Unit direction from A's closest point toward B's closest point. */
      readonly normal: Vec3;
      readonly pointA: Vec3;
      readonly pointB: Vec3;
    }
  | ({ readonly overlap: true } & EpaResult);

/**
 * Starting directions for GJK. If the origin lands on a lower-dimensional simplex (common for
 * centrally symmetric shapes whose supports lie on the line through their centers), EPA cannot
 * start from a polytope with the origin strictly inside, so GJK restarts from the next direction.
 */
const RESTART_DIRECTIONS = [
  new Vec3(0.5773502691896258, 0.5773502691896258, 0.5773502691896258),
  new Vec3(-0.8164965809277261, 0.4082482904638631, 0.4082482904638631),
  new Vec3(0.2672612419124244, -0.5345224838248488, 0.8017837257372732),
];

/** Full convex-convex narrowphase: GJK for separation, EPA for penetration. */
export function collideConvex(A: Placed, B: Placed): ConvexContact {
  const first = gjk(A, B);
  if (!first.overlap) return separated(first);
  const starts: (Vec3 | undefined)[] = [undefined, ...RESTART_DIRECTIONS];
  let fallback: EpaResult | null = null;
  let fallbackGjk: GjkResult = first;
  for (const start of starts) {
    const g = start === undefined ? first : gjk(A, B, { initialDirection: start });
    if (!g.overlap) return separated(g);
    const scale = Math.max(...g.simplex.map((s) => V.length(s.w)), 1);
    const e = epa(A, B, g.simplex, scale);
    if (e && e.converged) return { overlap: true, ...e };
    if (e && (!fallback || g.simplex.length > fallbackGjk.simplex.length)) {
      fallback = e;
      fallbackGjk = g;
    }
  }
  if (fallback) return { overlap: true, ...fallback };
  // Touching without a volumetric overlap: zero depth, normal from the witness difference.
  const d = V.sub(new Vec3(), first.pointB, first.pointA);
  const len = V.length(d);
  const normal = len > 0 ? V.scale(new Vec3(), d, 1 / len) : new Vec3(0, 1, 0);
  return {
    overlap: true,
    depth: 0,
    normal,
    pointA: first.pointA,
    pointB: first.pointB,
    iterations: first.iterations,
    converged: true,
  };
}

function separated(g: GjkResult): ConvexContact {
  const d = V.sub(new Vec3(), g.pointB, g.pointA);
  const len = V.length(d);
  const normal = len > 0 ? V.scale(new Vec3(), d, 1 / len) : new Vec3(1, 0, 0);
  return { overlap: false, distance: g.distance, normal, pointA: g.pointA, pointB: g.pointB };
}
