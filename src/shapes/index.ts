export * from './mass';
export * from './shape';
export * from './sphere';
export * from './box';
export * from './capsule';
export * from './hull';
export * from './quickhull';
export * from './mesh';
export * from './triangle';
export * from './compound';

import type { Sphere } from './sphere';
import type { Box } from './box';
import type { Capsule } from './capsule';
import type { ConvexHull } from './hull';
import type { TriangleMesh } from './mesh';
import type { Compound } from './compound';

/** Every shape the engine understands. */
export type Shape = Sphere | Box | Capsule | ConvexHull | TriangleMesh | Compound;
