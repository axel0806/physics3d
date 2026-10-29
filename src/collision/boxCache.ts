import { Transform } from '../math';
import type { Collider } from '../dynamics/body';
import type { Box } from '../shapes/box';
import { createOrientedBox, orientedBoxInto, type MutableOrientedBox } from './narrowphase/sat';

/** A box collider's world pose for one narrowphase pass: oriented box plus its world AABB. */
export interface BoxPose {
  stamp: number;
  readonly ob: MutableOrientedBox;
  minX: number;
  minY: number;
  minZ: number;
  maxX: number;
  maxY: number;
  maxZ: number;
}

const S_T = new Transform();

/**
 * Per-pass cache of box colliders' oriented boxes, indexed by broadphase proxy id. A body that
 * is in many pairs has its pose built once instead of once per pair, and the world AABB lets
 * clearly separated pairs skip the separating-axis test.
 */
export class BoxPoseCache {
  private stamp = 0;
  private readonly poses: (BoxPose | undefined)[] = [];

  /** Invalidates every entry; call once per pass, before poses change. */
  begin(): void {
    this.stamp++;
  }

  get(proxy: number, c: Collider): BoxPose {
    let p = this.poses[proxy];
    if (p === undefined) {
      p = {
        stamp: 0,
        ob: createOrientedBox(),
        minX: 0,
        minY: 0,
        minZ: 0,
        maxX: 0,
        maxY: 0,
        maxZ: 0,
      };
      this.poses[proxy] = p;
    }
    if (p.stamp === this.stamp) return p;
    p.stamp = this.stamp;
    const ob = orientedBoxInto(p.ob, c.shape as Box, c.body.colliderTransform(c, S_T));
    const a0 = ob.axes[0];
    const a1 = ob.axes[1];
    const a2 = ob.axes[2];
    const h0 = ob.h[0];
    const h1 = ob.h[1];
    const h2 = ob.h[2];
    const ex = h0 * Math.abs(a0.x) + h1 * Math.abs(a1.x) + h2 * Math.abs(a2.x);
    const ey = h0 * Math.abs(a0.y) + h1 * Math.abs(a1.y) + h2 * Math.abs(a2.y);
    const ez = h0 * Math.abs(a0.z) + h1 * Math.abs(a1.z) + h2 * Math.abs(a2.z);
    p.minX = ob.center.x - ex;
    p.minY = ob.center.y - ey;
    p.minZ = ob.center.z - ez;
    p.maxX = ob.center.x + ex;
    p.maxY = ob.center.y + ey;
    p.maxZ = ob.center.z + ez;
    return p;
  }
}

/** Slack that keeps rounding in the AABB test from rejecting a pair the SAT would accept. */
const AABB_SLACK = 1e-6;

/** True when the world AABBs are apart by more than `margin` on some axis (then no contact exists). */
export function boxesSeparated(a: BoxPose, b: BoxPose, margin: number): boolean {
  const g = margin + AABB_SLACK;
  return (
    a.minX - b.maxX > g ||
    b.minX - a.maxX > g ||
    a.minY - b.maxY > g ||
    b.minY - a.maxY > g ||
    a.minZ - b.maxZ > g ||
    b.minZ - a.maxZ > g
  );
}
