import { describe, expect, it } from 'vitest';
import { World } from '../../src/world/world';
import { Sphere } from '../../src/shapes';
import { Vec3 } from '../../src/math';
import { BallJoint } from '../../src/constraints';

describe('input validation', () => {
  it('rejects non-finite body state', () => {
    const w = new World();
    expect(() => w.createBody({ position: new Vec3(NaN, 0, 0) })).toThrow(RangeError);
    expect(() => w.createBody({ linearDamping: -1 })).toThrow(RangeError);
  });

  it('rejects joints on one body or on foreign bodies', () => {
    const w = new World();
    const a = w.createBody();
    w.createCollider(a, new Sphere(0.5));
    const other = new World().createBody();
    expect(() => w.addJoint(new BallJoint(a, a, new Vec3()))).toThrow(RangeError);
    expect(() => w.addJoint(new BallJoint(a, other, new Vec3()))).toThrow(RangeError);
  });
});
