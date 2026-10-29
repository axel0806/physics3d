import { Quat, Transform, Vec3 } from '../src/math';
import * as Q from '../src/math/quat';
import * as V from '../src/math/vec3';
import type { RigidBody } from '../src/dynamics/body';
import { Box, Capsule, Sphere } from '../src/shapes';
import { World } from '../src/world/world';
import { BallJoint, HingeJoint } from '../src/constraints';
import { createRagdoll } from '../tests/helpers/ragdoll';
import { heightfield } from '../tests/helpers/terrain';

/** Initial orbit-camera placement for a scene. */
export interface CameraStart {
  readonly target: [number, number, number];
  readonly distance: number;
  readonly yaw: number;
  readonly pitch: number;
}

/** Everything the host needs to run one scene. */
export interface SceneInstance {
  readonly world: World;
  readonly camera: CameraStart;
  /** Called once per rendered frame with the elapsed simulation time; may spawn or move bodies. */
  update?(time: number, dt: number): void;
  /** "Click to throw" and similar: called with a world-space ray when the scene wants clicks. */
  onClick?(origin: Vec3, direction: Vec3): void;
}

export interface SceneSpec {
  readonly id: string;
  readonly name: string;
  readonly group: 'validation' | 'playground';
  readonly description: string;
  /** Fixed timestep this scene is meant to run at (the host may override). */
  readonly fixedDt?: number;
  build(): SceneInstance;
}

function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 4294967296;
}

function floor(world: World, half = 30, friction = 0.6): RigidBody {
  const f = world.createBody({ type: 'static', position: new Vec3(0, -0.5, 0) });
  world.createCollider(f, new Box(half, 0.5, half), { friction });
  return f;
}

function addBox(
  world: World,
  pos: Vec3,
  half: Vec3,
  options: { friction?: number; restitution?: number; q?: Quat; v?: Vec3 } = {},
): RigidBody {
  const b = world.createBody({
    position: pos,
    ...(options.q ? { orientation: options.q } : {}),
    ...(options.v ? { linearVelocity: options.v } : {}),
  });
  world.createCollider(b, new Box(half.x, half.y, half.z), {
    friction: options.friction ?? 0.6,
    restitution: options.restitution ?? 0,
  });
  return b;
}

function addSphere(
  world: World,
  pos: Vec3,
  r: number,
  options: { friction?: number; restitution?: number; v?: Vec3; ccd?: boolean } = {},
): RigidBody {
  const b = world.createBody({
    position: pos,
    ...(options.v ? { linearVelocity: options.v } : {}),
    ...(options.ccd ? { ccd: true } : {}),
  });
  world.createCollider(b, new Sphere(r), {
    friction: options.friction ?? 0.5,
    restitution: options.restitution ?? 0,
  });
  return b;
}

/** A sphere thrown along a ray; used by every scene that reacts to clicks. */
function throwSphere(world: World, origin: Vec3, direction: Vec3, speed = 25): RigidBody {
  const d = V.normalize(new Vec3(), direction);
  const b = addSphere(world, V.addScaled(new Vec3(), origin, d, 1), 0.3, {
    v: V.scale(new Vec3(), d, speed),
    ccd: true,
    friction: 0.5,
  });
  b.linearDamping = 0;
  return b;
}

const clickToThrow = (world: World) => (o: Vec3, d: Vec3) => void throwSphere(world, o, d);

export const scenes: SceneSpec[] = [
  // ---- validation scenes -------------------------------------------------------------------
  {
    id: 'bounce',
    name: 'Bounce: restitution 0.2 / 0.5 / 0.8',
    group: 'validation',
    description: 'Three spheres dropped from 5 m; each rebound height is e squared times the drop.',
    fixedDt: 1 / 240,
    build() {
      const world = new World();
      floor(world);
      for (const [i, e] of [0.2, 0.5, 0.8].entries()) {
        addSphere(world, new Vec3(-2 + i * 2, 5.5, 0), 0.5, { restitution: e });
      }
      return { world, camera: { target: [0, 3, 0], distance: 14, yaw: 0.3, pitch: 0.25 } };
    },
  },
  {
    id: 'cradle',
    name: "Newton's cradle",
    group: 'validation',
    description: 'Five elastic spheres: momentum passes through the chain.',
    fixedDt: 1 / 2400,
    build() {
      const world = new World({ gravity: new Vec3(0, 0, 0) });
      for (let i = 0; i < 5; i++) {
        const b = world.createBody({
          position: new Vec3(2.001 * (i - 2), 2, 0),
          linearVelocity: i === 0 ? new Vec3(2, 0, 0) : new Vec3(),
        });
        world.createCollider(b, new Sphere(1), { restitution: 1, friction: 0 });
      }
      return { world, camera: { target: [0, 2, 0], distance: 14, yaw: 0, pitch: 0.1 } };
    },
  },
  {
    id: 'dzhanibekov',
    name: 'Dzhanibekov: intermediate axis flip',
    group: 'validation',
    description:
      'A T-shaped body spun about its intermediate axis tumbles and flips in zero gravity.',
    fixedDt: 1 / 480,
    build() {
      const world = new World({ gravity: new Vec3() });
      const b = world.createBody({
        position: new Vec3(0, 3, 0),
        angularVelocity: new Vec3(0.02, 3, 0.02),
      });
      world.createCollider(b, new Box(2, 0.25, 0.25));
      world.createCollider(b, new Box(0.25, 0.25, 1.25), {
        local: new Transform(new Vec3(0, 0, 1.5)),
      });
      return { world, camera: { target: [0, 3, 0], distance: 12, yaw: 0.6, pitch: 0.3 } };
    },
  },
  {
    id: 'friction-ramp',
    name: 'Friction ramp',
    group: 'validation',
    description: 'Boxes with friction 0.1 to 0.9 on a 25 degree ramp: the low-friction ones slide.',
    build() {
      const world = new World();
      const angle = (25 * Math.PI) / 180;
      const q = Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), -angle);
      const ramp = world.createBody({
        type: 'static',
        position: new Vec3(0, 3, 0),
        orientation: q,
      });
      world.createCollider(ramp, new Box(8, 0.25, 5), { friction: 0.5 });
      floor(world);
      for (let i = 0; i < 5; i++) {
        const mu = 0.1 + i * 0.2;
        const along = -3;
        const lz = -3.6 + i * 1.8;
        const up = new Vec3(Math.sin(angle), Math.cos(angle), 0);
        const p = new Vec3(
          along * Math.cos(angle) + up.x * 0.6,
          3 + along * -Math.sin(angle) + up.y * 0.6,
          lz,
        );
        addBox(world, p, new Vec3(0.4, 0.4, 0.4), { friction: mu, q });
      }
      return { world, camera: { target: [0, 3, 0], distance: 18, yaw: 0.5, pitch: 0.25 } };
    },
  },
  {
    id: 'tower',
    name: '20-box tower',
    group: 'validation',
    description: 'A tower of twenty boxes stands, then goes to sleep. Click to throw balls at it.',
    fixedDt: 1 / 120,
    build() {
      const world = new World();
      floor(world);
      for (let i = 0; i < 20; i++) addBox(world, new Vec3(0, 0.5 + i, 0), new Vec3(0.5, 0.5, 0.5));
      return {
        world,
        camera: { target: [0, 8, 0], distance: 28, yaw: 0.5, pitch: 0.2 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'pyramid',
    name: 'Pyramid of 55 boxes',
    group: 'validation',
    description: 'A ten-level pyramid that must stay up. Click to throw balls at it.',
    fixedDt: 1 / 120,
    build() {
      const world = new World();
      floor(world);
      for (let level = 0; level < 10; level++) {
        const n = 10 - level;
        for (let i = 0; i < n; i++) {
          addBox(world, new Vec3((i - (n - 1) / 2) * 1.0, 0.5 + level, 0), new Vec3(0.5, 0.5, 0.5));
        }
      }
      return {
        world,
        camera: { target: [0, 4, 0], distance: 26, yaw: 0.35, pitch: 0.2 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'pendulum',
    name: 'Pendulum, 120 degrees',
    group: 'validation',
    description:
      'A hinged pendulum released from 120 degrees; the period matches the elliptic-integral value.',
    fixedDt: 1 / 240,
    build() {
      const world = new World();
      const anchor = world.createBody({ type: 'static', position: new Vec3(0, 6, 0) });
      const L = 3;
      const th = (120 * Math.PI) / 180;
      const bob = addSphere(world, new Vec3(Math.sin(th) * L, 6 - Math.cos(th) * L, 0), 0.25);
      world.addJoint(new HingeJoint(anchor, bob, new Vec3(0, 6, 0), new Vec3(0, 0, 1)));
      return { world, camera: { target: [0, 4.5, 0], distance: 13, yaw: 0, pitch: 0.05 } };
    },
  },
  {
    id: 'double-pendulum',
    name: 'Double pendulum',
    group: 'validation',
    description: 'Chaotic double pendulum; total energy stays within 1 percent.',
    fixedDt: 1 / 480,
    build() {
      const world = new World();
      const anchor = world.createBody({ type: 'static', position: new Vec3(0, 8, 0) });
      const a = addSphere(world, new Vec3(2, 8, 0), 0.2);
      const b = addSphere(world, new Vec3(4, 8, 0), 0.2);
      world.addJoint(new HingeJoint(anchor, a, new Vec3(0, 8, 0), new Vec3(0, 0, 1)));
      world.addJoint(new HingeJoint(a, b, new Vec3(2, 8, 0), new Vec3(0, 0, 1)));
      return { world, camera: { target: [1, 6, 0], distance: 14, yaw: 0, pitch: 0.05 } };
    },
  },
  {
    id: 'chain',
    name: '50-link chain',
    group: 'validation',
    description: 'A fifty-link capsule chain released horizontally; the joints hold.',
    fixedDt: 1 / 240,
    build() {
      const world = new World();
      const anchor = world.createBody({ type: 'static', position: new Vec3(-12, 10, 0) });
      let prev = anchor;
      for (let i = 0; i < 50; i++) {
        const link = world.createBody({
          position: new Vec3(-12 + 0.5 + i * 0.5, 10, 0),
          orientation: Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), Math.PI / 2),
        });
        world.createCollider(link, new Capsule(0.08, 0.17), { friction: 0.4 });
        world.addJoint(new BallJoint(prev, link, new Vec3(-12 + i * 0.5, 10, 0)));
        prev = link;
      }
      return { world, camera: { target: [-6, 6, 0], distance: 24, yaw: 0, pitch: 0.1 } };
    },
  },
  {
    id: 'ragdolls',
    name: 'Ragdoll pile',
    group: 'validation',
    description: 'Twenty ragdolls dropped in a heap: no explosions, no torn limbs.',
    build() {
      const world = new World();
      floor(world);
      const r = rng(8);
      for (let i = 0; i < 20; i++) {
        createRagdoll(
          world,
          new Vec3((r() - 0.5) * 3, 1.5 + i * 1.3, (r() - 0.5) * 3),
          Q.fromAxisAngle(new Quat(), new Vec3(r(), r(), r() + 0.1), r() * 6),
        );
      }
      return {
        world,
        camera: { target: [0, 3, 0], distance: 16, yaw: 0.6, pitch: 0.3 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'bullet-wall',
    name: 'CCD: 500 m/s sphere vs a 1 cm wall',
    group: 'validation',
    description:
      'A fast sphere is fired at a thin wall every second; continuous collision detection stops it.',
    build() {
      const world = new World();
      floor(world);
      const wall = world.createBody({ type: 'static', position: new Vec3(0, 2, 0) });
      world.createCollider(wall, new Box(3, 2, 0.005));
      let last = -10;
      const r = rng(5);
      return {
        world,
        camera: { target: [0, 2, 0], distance: 14, yaw: 0.8, pitch: 0.15 },
        update(time) {
          if (time - last < 1) return;
          last = time;
          const target = new Vec3((r() - 0.5) * 4, 1 + r() * 2.5, 0);
          const from = new Vec3((r() - 0.5) * 4, 1 + r() * 2.5, 25);
          const dir = V.normalize(new Vec3(), V.sub(new Vec3(), target, from));
          addSphere(world, from, 0.05, {
            v: V.scale(new Vec3(), dir, 500),
            ccd: true,
            restitution: 0.2,
          });
        },
      };
    },
  },
  {
    id: 'terrain',
    name: 'Terrain mesh',
    group: 'validation',
    description: 'A static triangle-mesh heightfield with shapes raining onto it.',
    build() {
      const world = new World();
      const g = world.createBody({ type: 'static' });
      const height = (x: number, z: number) =>
        0.6 * Math.sin(x * 0.35) * Math.cos(z * 0.3) + 0.02 * (x * x + z * z);
      world.createCollider(g, heightfield(80, 20, height), { friction: 0.7 });
      const r = rng(11);
      let spawned = 0;
      return {
        world,
        camera: { target: [0, 2, 0], distance: 34, yaw: 0.5, pitch: 0.55 },
        update(time) {
          while (spawned < 150 && time > spawned * 0.15) {
            const p = new Vec3((r() - 0.5) * 18, 8 + r() * 4, (r() - 0.5) * 18);
            if (spawned % 2) addSphere(world, p, 0.35, { friction: 0.7 });
            else
              addBox(world, p, new Vec3(0.3, 0.3, 0.3), {
                friction: 0.7,
                q: Q.fromAxisAngle(new Quat(), new Vec3(r(), r(), r() + 0.1), r() * 6),
              });
            spawned++;
          }
        },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'pile-1000',
    name: '1,000-box pile',
    group: 'validation',
    description: 'One thousand boxes dropped into a walled arena (the benchmark scene).',
    build() {
      const world = new World();
      floor(world, 6);
      for (const [x, z, sx, sz] of [
        [-4.5, 0, 0.5, 5],
        [4.5, 0, 0.5, 5],
        [0, -4.5, 5, 0.5],
        [0, 4.5, 5, 0.5],
      ] as const) {
        const w = world.createBody({ type: 'static', position: new Vec3(x, 4, z) });
        world.createCollider(w, new Box(sx, 4.5, sz));
      }
      const r = rng(1);
      for (let layer = 0; layer < 10; layer++) {
        for (let i = 0; i < 100; i++) {
          addBox(
            world,
            new Vec3(
              -3.6 + (i % 10) * 0.8 + (r() - 0.5) * 0.1,
              0.4 + layer * 0.75,
              -3.6 + Math.floor(i / 10) * 0.8 + (r() - 0.5) * 0.1,
            ),
            new Vec3(0.25, 0.25, 0.25),
            { friction: 0.5 },
          );
        }
      }
      return {
        world,
        camera: { target: [0, 2, 0], distance: 18, yaw: 0.6, pitch: 0.45 },
        onClick: clickToThrow(world),
      };
    },
  },

  // ---- playground scenes -------------------------------------------------------------------
  {
    id: 'jenga',
    name: 'Jenga',
    group: 'playground',
    description: 'A tower of 54 blocks in alternating layers. Click to throw balls.',
    fixedDt: 1 / 120,
    build() {
      const world = new World();
      floor(world);
      const w = 0.5;
      const h = 0.3;
      const l = 1.5;
      for (let layer = 0; layer < 18; layer++) {
        for (let i = -1; i <= 1; i++) {
          const along = layer % 2 === 0;
          const pos = along
            ? new Vec3(i * w * 2.02, h + layer * h * 2.002, 0)
            : new Vec3(0, h + layer * h * 2.002, i * w * 2.02);
          addBox(world, pos, along ? new Vec3(w, h, l) : new Vec3(l, h, w), { friction: 0.7 });
        }
      }
      return {
        world,
        camera: { target: [0, 5, 0], distance: 16, yaw: 0.5, pitch: 0.2 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'dominoes',
    name: 'Domino spiral',
    group: 'playground',
    description:
      'Eighty dominoes along a spiral; a ball knocks over the first. Click to throw more.',
    fixedDt: 1 / 120,
    build() {
      const world = new World();
      floor(world);
      let angle = 0;
      let radius = 3;
      const heightHalf = 0.5;
      for (let i = 0; i < 80; i++) {
        const pos = new Vec3(Math.cos(angle) * radius, heightHalf, Math.sin(angle) * radius);
        const q = Q.fromAxisAngle(new Quat(), new Vec3(0, 1, 0), -angle);
        addBox(world, pos, new Vec3(0.06, heightHalf, 0.3), { friction: 0.5, q });
        const step = 0.62;
        angle += step / radius;
        radius += 0.014 * (step / radius) * 8;
      }
      addSphere(world, new Vec3(3.0, 1.8, -1.1), 0.3, { v: new Vec3(0.6, 0, 1.2) });
      return {
        world,
        camera: { target: [0, 0.5, 0], distance: 18, yaw: 0.2, pitch: 0.75 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'wrecking-ball',
    name: 'Wrecking ball',
    group: 'playground',
    description: 'A heavy ball swings on a chain into a wall of boxes. Click to throw more.',
    build() {
      const world = new World();
      floor(world);
      const top = new Vec3(0, 14, 0);
      const anchor = world.createBody({ type: 'static', position: top });
      let prev = anchor;
      const links = 10;
      const linkLen = 0.9;
      const swing = (55 * Math.PI) / 180;
      for (let i = 0; i < links; i++) {
        const d = (i + 0.5) * linkLen;
        const pos = new Vec3(-Math.sin(swing) * d, 14 - Math.cos(swing) * d, 0);
        const link = world.createBody({
          position: pos,
          orientation: Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), -swing),
        });
        world.createCollider(link, new Capsule(0.08, linkLen / 2 - 0.08));
        const jd = i * linkLen;
        world.addJoint(
          new BallJoint(prev, link, new Vec3(-Math.sin(swing) * jd, 14 - Math.cos(swing) * jd, 0)),
        );
        prev = link;
      }
      const R = 0.9;
      const dball = links * linkLen + R;
      const ball = addSphere(
        world,
        new Vec3(-Math.sin(swing) * dball, 14 - Math.cos(swing) * dball, 0),
        R,
        { friction: 0.4 },
      );
      world.addJoint(
        new BallJoint(
          prev,
          ball,
          new Vec3(-Math.sin(swing) * links * linkLen, 14 - Math.cos(swing) * links * linkLen, 0),
        ),
      );
      for (let y = 0; y < 7; y++) {
        for (let z = -3; z <= 3; z++) {
          addBox(world, new Vec3(5, 0.4 + y * 0.8, z * 0.8), new Vec3(0.4, 0.4, 0.4), {
            friction: 0.6,
          });
        }
      }
      return {
        world,
        camera: { target: [0, 6, 0], distance: 30, yaw: 0.05, pitch: 0.15 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'vehicle',
    name: 'Vehicle',
    group: 'playground',
    description:
      'A chassis on four motorised hinge wheels drives over ramps. Click to throw balls.',
    build() {
      const world = new World();
      floor(world, 60);
      const ramp = (x: number, a: number) => {
        const q = Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), a);
        const r = world.createBody({
          type: 'static',
          position: new Vec3(x, 0.3, 0),
          orientation: q,
        });
        world.createCollider(r, new Box(3, 0.2, 3), { friction: 0.9 });
      };
      ramp(14, 0.18);
      ramp(26, -0.18);
      const chassis = addBox(world, new Vec3(0, 1.1, 0), new Vec3(1.4, 0.25, 0.8), {
        friction: 0.2,
      });
      for (const sx of [-0.9, 0.9]) {
        for (const sz of [-1, 1]) {
          const wheel = world.createBody({
            position: new Vec3(sx, 0.55, sz * 1.0),
            orientation: Q.fromAxisAngle(new Quat(), new Vec3(1, 0, 0), Math.PI / 2),
          });
          world.createCollider(wheel, new Sphere(0.5), { friction: 1.2 });
          world.addJoint(
            new HingeJoint(chassis, wheel, new Vec3(sx, 0.55, sz * 1.0), new Vec3(0, 0, 1), {
              motorSpeed: -9,
              maxMotorTorque: 150,
            }),
          );
        }
      }
      return {
        world,
        camera: { target: [10, 1.5, 0], distance: 22, yaw: 0.35, pitch: 0.2 },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'ragdoll-cannon',
    name: 'Ragdoll cannon',
    group: 'playground',
    description: 'A cannon fires a ragdoll at a box wall every few seconds. Click to throw balls.',
    build() {
      const world = new World();
      floor(world, 50);
      for (let y = 0; y < 8; y++) {
        for (let z = -4; z <= 4; z++) {
          addBox(world, new Vec3(8, 0.4 + y * 0.8, z * 0.8), new Vec3(0.4, 0.4, 0.4), {
            friction: 0.6,
          });
        }
      }
      let last = -100;
      const r = rng(21);
      return {
        world,
        camera: { target: [3, 3, 0], distance: 26, yaw: 0.9, pitch: 0.2 },
        update(time) {
          if (time - last < 4) return;
          last = time;
          const doll = createRagdoll(
            world,
            new Vec3(-8, 2 + r(), (r() - 0.5) * 2),
            Q.fromAxisAngle(new Quat(), new Vec3(0, 0, 1), -Math.PI / 2),
          );
          const v = new Vec3(18 + r() * 4, 4 + r() * 3, (r() - 0.5) * 2);
          for (const b of doll.bodies) V.copy(b.linearVelocity, v);
        },
        onClick: clickToThrow(world),
      };
    },
  },
  {
    id: 'throw',
    name: 'Click to throw',
    group: 'playground',
    description: 'An open arena with a few stacks: click anywhere to throw a ball at it.',
    build() {
      const world = new World();
      floor(world);
      for (const x of [-4, 0, 4]) {
        for (let i = 0; i < 6; i++) {
          addBox(world, new Vec3(x, 0.5 + i, -3), new Vec3(0.5, 0.5, 0.5), { friction: 0.5 });
        }
      }
      return {
        world,
        camera: { target: [0, 3, -3], distance: 20, yaw: 0, pitch: 0.2 },
        onClick: clickToThrow(world),
      };
    },
  },
];
