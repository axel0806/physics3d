export const VERSION = '0.1.0';

export * from './math';
export * from './shapes';
export * from './constraints';
export { World } from './world/world';
export type {
  WorldOptions,
  ColliderOptions,
  ContactEvent,
  QueryFilter,
  RayHit,
  ShapeCastHit,
  WorldSnapshot,
} from './world/world';
export { RigidBody } from './dynamics/body';
export type { BodyOptions, BodyType, Collider, ColliderShape } from './dynamics/body';
