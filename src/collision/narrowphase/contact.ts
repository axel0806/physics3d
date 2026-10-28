import type { Vec3 } from '../../math';

/** One contact point. `position` is the midpoint of the two surface points; `id` is a persistent feature id. */
export interface ContactPoint {
  readonly position: Vec3;
  /** Penetration depth along the manifold normal (positive when overlapping). */
  readonly depth: number;
  readonly id: number;
  /** Surface points on A and B. */
  readonly pointA: Vec3;
  readonly pointB: Vec3;
}

/** Contact manifold with at most four points. `normal` points from A toward B. */
export interface ContactManifold {
  readonly normal: Vec3;
  readonly points: readonly ContactPoint[];
}
