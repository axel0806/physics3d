import { describe, expect, it } from 'vitest';
import { Vec3 } from '../../src/math';
import { World } from '../../src/world/world';
import {
  BallJoint,
  ConeTwistJoint,
  DistanceJoint,
  FixedJoint,
  HingeJoint,
  SliderJoint,
} from '../../src/constraints';

function bodies() {
  const w = new World();
  const a = w.createBody({ type: 'static' });
  const b = w.createBody({ position: new Vec3(1, 0, 0) });
  return { a, b };
}

const zero = new Vec3(0, 0, 0);
const nan = new Vec3(NaN, 0, 1);
const inf = new Vec3(0, Infinity, 0);
const up = new Vec3(0, 1, 0);

describe('joint constructor validation', () => {
  it('rejects non-finite anchors', () => {
    const { a, b } = bodies();
    expect(() => new BallJoint(a, b, nan)).toThrow(RangeError);
    expect(() => new FixedJoint(a, b, inf)).toThrow(RangeError);
    expect(() => new HingeJoint(a, b, nan, up)).toThrow(RangeError);
    expect(() => new SliderJoint(a, b, inf, up)).toThrow(RangeError);
    expect(() => new DistanceJoint(a, b, zero, nan)).toThrow(RangeError);
    expect(() => new ConeTwistJoint(a, b, nan, up, { swingSpan: 1, twistSpan: 1 })).toThrow(
      RangeError,
    );
  });

  it('rejects zero-length or non-finite axes', () => {
    const { a, b } = bodies();
    for (const axis of [zero, nan, inf]) {
      expect(() => new HingeJoint(a, b, zero, axis)).toThrow(RangeError);
      expect(() => new SliderJoint(a, b, zero, axis)).toThrow(RangeError);
      expect(() => new ConeTwistJoint(a, b, zero, axis, { swingSpan: 1, twistSpan: 1 })).toThrow(
        RangeError,
      );
    }
  });

  it('rejects lower > upper limits', () => {
    const { a, b } = bodies();
    expect(() => new HingeJoint(a, b, zero, up, { lower: 1, upper: -1 })).toThrow(RangeError);
    expect(() => new HingeJoint(a, b, zero, up, { lower: 4 })).toThrow(RangeError);
    expect(() => new HingeJoint(a, b, zero, up, { lower: NaN })).toThrow(RangeError);
    expect(() => new SliderJoint(a, b, zero, up, { lower: 2, upper: 1 })).toThrow(RangeError);
  });

  it('rejects negative or non-finite distance length and negative cone spans', () => {
    const { a, b } = bodies();
    expect(() => new DistanceJoint(a, b, zero, up, { length: -1 })).toThrow(RangeError);
    expect(() => new DistanceJoint(a, b, zero, up, { length: NaN })).toThrow(RangeError);
    expect(() => new ConeTwistJoint(a, b, zero, up, { swingSpan: -1, twistSpan: 1 })).toThrow(
      RangeError,
    );
    expect(() => new ConeTwistJoint(a, b, zero, up, { swingSpan: 1, twistSpan: NaN })).toThrow(
      RangeError,
    );
  });

  it('still accepts valid input', () => {
    const { a, b } = bodies();
    expect(() => new BallJoint(a, b, zero)).not.toThrow();
    expect(() => new FixedJoint(a, b, zero)).not.toThrow();
    expect(() => new HingeJoint(a, b, zero, up, { lower: -1, upper: 1 })).not.toThrow();
    expect(() => new HingeJoint(a, b, zero, up, { lower: 1, upper: 1 })).not.toThrow();
    expect(() => new SliderJoint(a, b, zero, up, { lower: 0 })).not.toThrow();
    expect(() => new SliderJoint(a, b, zero, up)).not.toThrow();
    expect(() => new DistanceJoint(a, b, zero, up, { length: 0 })).not.toThrow();
    expect(() => new DistanceJoint(a, b, zero, up)).not.toThrow();
    expect(() => new ConeTwistJoint(a, b, zero, up, { swingSpan: 0, twistSpan: 0 })).not.toThrow();
  });
});
