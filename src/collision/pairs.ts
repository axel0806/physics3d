import { Transform, Vec3 } from '../math';
import * as V from '../math/vec3';
import * as T from '../math/transform';
import { Box, Capsule, Sphere } from '../shapes';
import type { ConvexShape } from '../shapes/shape';
import { collideConvex, type Placed } from './narrowphase';
import { boxBoxManifold, orientedBox, type OrientedBox } from './narrowphase/sat';
import { capsuleCapsule, sphereBox, sphereSphere } from './narrowphase/analytic';
import type { ContactManifold } from './narrowphase/contact';

/** Shape and world transform, as seen by the manifold dispatcher. */
export interface WorldShape {
  readonly shape: ConvexShape;
  readonly transform: Transform;
}

/** Capsule core segment in world space. */
function segment(c: Capsule, t: Transform): [Vec3, Vec3] {
  const up = T.applyDirection(new Vec3(), t, new Vec3(0, c.halfHeight, 0));
  return [V.sub(new Vec3(), t.p, up), V.add(new Vec3(), t.p, up)];
}

function swapManifold(m: ContactManifold): ContactManifold {
  return {
    normal: V.negate(new Vec3(), m.normal),
    points: m.points.map((p) => ({
      position: p.position,
      depth: p.depth,
      id: p.id,
      pointA: p.pointB,
      pointB: p.pointA,
    })),
  };
}

/**
 * Contact manifold between two world-placed convex shapes, normal from A to B. Analytic paths are
 * used for sphere, box, and capsule pairs; anything else goes through GJK/EPA with one point.
 */
export function manifoldOf(A: WorldShape, B: WorldShape): ContactManifold | null {
  const a = A.shape;
  const b = B.shape;
  if (a instanceof Sphere && b instanceof Sphere) {
    return sphereSphere(A.transform.p, a.radius, B.transform.p, b.radius);
  }
  if (a instanceof Sphere && b instanceof Box) {
    const ob = orientedBox(b, B.transform);
    return sphereBox(A.transform.p, a.radius, ob.center, ob.axes, new Vec3(...ob.h));
  }
  if (a instanceof Box && b instanceof Sphere) {
    const m = manifoldOf(B, A);
    return m ? swapManifold(m) : null;
  }
  if (a instanceof Box && b instanceof Box) {
    return boxBoxManifold(orientedBox(a, A.transform), orientedBox(b, B.transform));
  }
  if (a instanceof Capsule && b instanceof Capsule) {
    const [pa, qa] = segment(a, A.transform);
    const [pb, qb] = segment(b, B.transform);
    return capsuleCapsule(pa, qa, a.radius, pb, qb, b.radius);
  }
  if (a instanceof Sphere && b instanceof Capsule) {
    const [pb, qb] = segment(b, B.transform);
    return capsuleCapsule(A.transform.p, A.transform.p, a.radius, pb, qb, b.radius);
  }
  if (a instanceof Capsule && b instanceof Sphere) {
    const m = manifoldOf(B, A);
    return m ? swapManifold(m) : null;
  }
  const g = collideConvex(A as Placed, B as Placed);
  if (!g.overlap) return null;
  return {
    normal: V.copy(new Vec3(), g.normal),
    points: [
      {
        position: V.lerp(new Vec3(), g.pointA, g.pointB, 0.5),
        depth: g.depth,
        id: 0,
        pointA: g.pointA,
        pointB: g.pointB,
      },
    ],
  };
}

export type { OrientedBox };
