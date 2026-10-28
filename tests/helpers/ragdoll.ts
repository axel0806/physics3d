import { Quat, Transform, Vec3 } from '../../src/math';
import * as Q from '../../src/math/quat';
import type { RigidBody } from '../../src/dynamics/body';
import { Box, Capsule, Sphere } from '../../src/shapes';
import { World } from '../../src/world/world';
import { ConeTwistJoint, HingeJoint, type Joint } from '../../src/constraints';

export interface Ragdoll {
  readonly bodies: RigidBody[];
  readonly joints: Joint[];
}

/** Anchor separation (m) of a joint's two body-frame anchors; the joint is intact while it is small. */
export function anchorGap(j: {
  bodyA: RigidBody;
  bodyB: RigidBody;
  localA: Vec3;
  localB: Vec3;
}): number {
  const pa = Q.rotate(new Vec3(), j.bodyA.orientation, j.localA).add(j.bodyA.position);
  const pb = Q.rotate(new Vec3(), j.bodyB.orientation, j.localB).add(j.bodyB.position);
  return pa.sub(pb).length();
}

/**
 * Eleven-body ragdoll: torso, head, two arms (upper + lower), two legs (upper + lower). Shoulders,
 * hips and neck are cone-twist joints; elbows and knees are limited hinges. Standing pose, built
 * around `origin` (the pelvis centre) and then moved and spun by the caller.
 */
export function createRagdoll(world: World, origin: Vec3, orientation = new Quat()): Ragdoll {
  const bodies: RigidBody[] = [];
  const joints: Joint[] = [];
  const at = (x: number, y: number, z: number) => origin.add(new Vec3(x, y, z));
  const make = (pos: Vec3, shape: Box | Sphere | Capsule): RigidBody => {
    const rel = pos.sub(origin);
    const p = origin.add(Q.rotate(new Vec3(), orientation, rel));
    const b = world.createBody({ position: p, orientation: orientation.clone() });
    world.createCollider(b, shape, { friction: 0.6, local: new Transform() });
    bodies.push(b);
    return b;
  };
  const anchor = (v: Vec3): Vec3 => origin.add(Q.rotate(new Vec3(), orientation, v.sub(origin)));
  const dir = (v: Vec3): Vec3 => Q.rotate(new Vec3(), orientation, v);

  const torso = make(at(0, 0.35, 0), new Box(0.18, 0.3, 0.1));
  const head = make(at(0, 0.92, 0), new Sphere(0.11));
  joints.push(
    world.addJoint(
      new ConeTwistJoint(torso, head, anchor(at(0, 0.67, 0)), dir(new Vec3(0, 1, 0)), {
        swingSpan: 0.7,
        twistSpan: 0.7,
      }),
    ),
  );
  for (const side of [-1, 1]) {
    const upperArm = make(at(side * 0.3, 0.45, 0), new Capsule(0.05, 0.12));
    const foreArm = make(at(side * 0.3, 0.1, 0), new Capsule(0.045, 0.12));
    joints.push(
      world.addJoint(
        new ConeTwistJoint(
          torso,
          upperArm,
          anchor(at(side * 0.25, 0.62, 0)),
          dir(new Vec3(0, -1, 0)),
          {
            swingSpan: 1.4,
            twistSpan: 0.8,
          },
        ),
      ),
      world.addJoint(
        new HingeJoint(upperArm, foreArm, anchor(at(side * 0.3, 0.28, 0)), dir(new Vec3(1, 0, 0)), {
          lower: -2.2,
          upper: 0,
        }),
      ),
    );
    const thigh = make(at(side * 0.1, -0.3, 0), new Capsule(0.07, 0.17));
    const shin = make(at(side * 0.1, -0.78, 0), new Capsule(0.055, 0.17));
    joints.push(
      world.addJoint(
        new ConeTwistJoint(
          torso,
          thigh,
          anchor(at(side * 0.1, -0.05, 0)),
          dir(new Vec3(0, -1, 0)),
          {
            swingSpan: 1.0,
            twistSpan: 0.4,
          },
        ),
      ),
      world.addJoint(
        new HingeJoint(thigh, shin, anchor(at(side * 0.1, -0.54, 0)), dir(new Vec3(1, 0, 0)), {
          lower: 0,
          upper: 2.2,
        }),
      ),
    );
  }
  return { bodies, joints };
}
