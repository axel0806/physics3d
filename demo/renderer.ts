import { Quat, Vec3 } from '../src/math';
import * as Q from '../src/math/quat';
import type { Collider, RigidBody } from '../src/dynamics/body';
import { Box, Capsule, ConvexHull, Sphere, TriangleMesh } from '../src/shapes';
import type { World } from '../src/world/world';
import type { Camera } from './camera';
import { boxMesh, capsuleMesh, flatMesh, sphereMesh, type MeshData } from './geometry';
import * as M from './mat4';

export interface DebugFlags {
  contacts: boolean;
  normals: boolean;
  aabbs: boolean;
  broadphase: boolean;
  joints: boolean;
  /** Colour bodies by sleep state instead of by identity. */
  islands: boolean;
}

export const defaultDebug = (): DebugFlags => ({
  contacts: false,
  normals: false,
  aabbs: false,
  broadphase: false,
  joints: true,
  islands: false,
});

const STRIDE = 20; // 16 matrix floats + 4 colour floats per instance

const MAIN_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec4 aM0;
layout(location=3) in vec4 aM1;
layout(location=4) in vec4 aM2;
layout(location=5) in vec4 aM3;
layout(location=6) in vec4 aColor;
uniform mat4 uViewProj;
out vec3 vNormal;
out vec3 vWorld;
out vec4 vColor;
void main() {
  mat4 model = mat4(aM0, aM1, aM2, aM3);
  vec4 w = model * vec4(aPos, 1.0);
  vWorld = w.xyz;
  vec3 s2 = vec3(dot(aM0.xyz, aM0.xyz), dot(aM1.xyz, aM1.xyz), dot(aM2.xyz, aM2.xyz));
  vNormal = mat3(model) * (aNormal / max(s2, vec3(1e-8)));
  vColor = aColor;
  gl_Position = uViewProj * w;
}`;

const MAIN_FS = `#version 300 es
precision highp float;
precision highp sampler2DShadow;
in vec3 vNormal;
in vec3 vWorld;
in vec4 vColor;
uniform vec3 uLightDir;
uniform vec3 uEye;
uniform mat4 uLightViewProj;
uniform sampler2DShadow uShadow;
uniform float uShadowTexel;
out vec4 outColor;
float shadowAt(vec3 n) {
  vec4 lp = uLightViewProj * vec4(vWorld + n * 0.06, 1.0);
  vec3 p = lp.xyz / lp.w * 0.5 + 0.5;
  if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0) return 1.0;
  float sum = 0.0;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      sum += texture(uShadow, vec3(p.xy + vec2(float(x), float(y)) * uShadowTexel, p.z - 0.0015));
    }
  }
  return sum / 9.0;
}
void main() {
  vec3 n = normalize(vNormal);
  vec3 l = normalize(-uLightDir);
  float diff = max(dot(n, l), 0.0);
  float sh = shadowAt(n);
  vec3 sky = mix(vec3(0.22, 0.2, 0.18), vec3(0.42, 0.5, 0.62), n.y * 0.5 + 0.5);
  vec3 base = vColor.rgb;
  vec3 lit = base * (sky * 0.9 + vec3(1.0, 0.96, 0.9) * diff * sh * 0.95);
  vec3 v = normalize(uEye - vWorld);
  float spec = pow(max(dot(n, normalize(l + v)), 0.0), 40.0) * 0.18 * sh;
  lit += vec3(spec);
  if (vColor.a > 1.5) lit += vec3(0.25, 0.22, 0.05);
  float dist = length(uEye - vWorld);
  lit = mix(lit, vec3(0.62, 0.69, 0.78), clamp((dist - 80.0) / 220.0, 0.0, 0.8));
  outColor = vec4(pow(lit, vec3(1.0 / 2.2)), 1.0);
}`;

const SHADOW_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=2) in vec4 aM0;
layout(location=3) in vec4 aM1;
layout(location=4) in vec4 aM2;
layout(location=5) in vec4 aM3;
uniform mat4 uLightViewProj;
void main() {
  gl_Position = uLightViewProj * (mat4(aM0, aM1, aM2, aM3) * vec4(aPos, 1.0));
}`;
const SHADOW_FS = `#version 300 es
precision mediump float;
void main() {}`;

const LINE_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main() {
  vColor = aColor;
  gl_Position = uViewProj * vec4(aPos, 1.0);
}`;
const LINE_FS = `#version 300 es
precision mediump float;
in vec3 vColor;
out vec4 outColor;
void main() { outColor = vec4(vColor, 1.0); }`;

interface GpuMesh {
  vao: WebGLVertexArrayObject;
  instanceBuffer: WebGLBuffer;
  indexCount: number;
  data: Float32Array;
  count: number;
}

function compile(gl: WebGL2RenderingContext, type: number, src: string): WebGLShader {
  const sh = gl.createShader(type) as WebGLShader;
  gl.shaderSource(sh, src);
  gl.compileShader(sh);
  if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
    throw new Error(`shader compile failed: ${gl.getShaderInfoLog(sh)}`);
  }
  return sh;
}

function program(gl: WebGL2RenderingContext, vs: string, fs: string): WebGLProgram {
  const p = gl.createProgram() as WebGLProgram;
  gl.attachShader(p, compile(gl, gl.VERTEX_SHADER, vs));
  gl.attachShader(p, compile(gl, gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
    throw new Error(`program link failed: ${gl.getProgramInfoLog(p)}`);
  }
  return p;
}

const PALETTE: [number, number, number][] = [
  [0.89, 0.42, 0.35],
  [0.36, 0.69, 0.84],
  [0.93, 0.78, 0.33],
  [0.52, 0.78, 0.47],
  [0.74, 0.5, 0.84],
  [0.95, 0.6, 0.3],
  [0.4, 0.78, 0.7],
  [0.84, 0.5, 0.62],
];

/** Raw WebGL2 renderer: instanced meshes, a shadow-mapped sun, and a line layer for debug draw. */
export class Renderer {
  readonly gl: WebGL2RenderingContext;
  private readonly mainProg: WebGLProgram;
  private readonly shadowProg: WebGLProgram;
  private readonly lineProg: WebGLProgram;
  private readonly shadowTex: WebGLTexture;
  private readonly shadowFbo: WebGLFramebuffer;
  private readonly shadowSize = 2048;
  private readonly meshes = new Map<unknown, GpuMesh>();
  private readonly lineVao: WebGLVertexArrayObject;
  private readonly lineBuffer: WebGLBuffer;
  private lineData = new Float32Array(6 * 4096);
  private lineCount = 0;
  private readonly lightDir = new Float32Array([-0.45, -1, -0.35]);
  private readonly lightViewProj = M.create();
  private readonly lightView = M.create();
  private readonly lightProj = M.create();
  /** Number of draw calls and instances in the last frame (for the stats readout). */
  drawCalls = 0;
  instances = 0;

  constructor(readonly canvas: HTMLCanvasElement) {
    // Some browsers/GPUs refuse the first request (antialiasing, blocklisted GPU): relax it.
    const gl =
      canvas.getContext('webgl2', { antialias: true, alpha: false }) ??
      canvas.getContext('webgl2', { antialias: false, alpha: false }) ??
      canvas.getContext('webgl2', { antialias: false, alpha: false, powerPreference: 'low-power' });
    if (!gl) throw new Error('WebGL2 is not available');
    this.gl = gl;
    const l = this.lightDir;
    const n = Math.hypot(l[0] as number, l[1] as number, l[2] as number);
    for (let i = 0; i < 3; i++) l[i] = (l[i] as number) / n;
    this.mainProg = program(gl, MAIN_VS, MAIN_FS);
    this.shadowProg = program(gl, SHADOW_VS, SHADOW_FS);
    this.lineProg = program(gl, LINE_VS, LINE_FS);

    this.shadowTex = gl.createTexture() as WebGLTexture;
    gl.bindTexture(gl.TEXTURE_2D, this.shadowTex);
    gl.texImage2D(
      gl.TEXTURE_2D,
      0,
      gl.DEPTH_COMPONENT24,
      this.shadowSize,
      this.shadowSize,
      0,
      gl.DEPTH_COMPONENT,
      gl.UNSIGNED_INT,
      null,
    );
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_MODE, gl.COMPARE_REF_TO_TEXTURE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_FUNC, gl.LEQUAL);
    this.shadowFbo = gl.createFramebuffer() as WebGLFramebuffer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.shadowFbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, this.shadowTex, 0);
    gl.drawBuffers([gl.NONE]);
    gl.readBuffer(gl.NONE);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);

    this.lineVao = gl.createVertexArray() as WebGLVertexArrayObject;
    this.lineBuffer = gl.createBuffer() as WebGLBuffer;
    gl.bindVertexArray(this.lineVao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.lineBuffer);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 24, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 24, 12);
    gl.bindVertexArray(null);
  }

  /** Matches the canvas drawing buffer to its CSS size (and the device pixel ratio). */
  resize(): void {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.floor(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
  }

  private upload(key: unknown, data: MeshData): GpuMesh {
    const gl = this.gl;
    const vao = gl.createVertexArray() as WebGLVertexArrayObject;
    gl.bindVertexArray(vao);
    const pb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, pb);
    gl.bufferData(gl.ARRAY_BUFFER, data.positions, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    const nb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nb);
    gl.bufferData(gl.ARRAY_BUFFER, data.normals, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
    const ib = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, data.indices, gl.STATIC_DRAW);
    const instanceBuffer = gl.createBuffer() as WebGLBuffer;
    gl.bindBuffer(gl.ARRAY_BUFFER, instanceBuffer);
    for (let i = 0; i < 5; i++) {
      gl.enableVertexAttribArray(2 + i);
      gl.vertexAttribPointer(2 + i, 4, gl.FLOAT, false, STRIDE * 4, i * 16);
      gl.vertexAttribDivisor(2 + i, 1);
    }
    gl.bindVertexArray(null);
    const mesh: GpuMesh = {
      vao,
      instanceBuffer,
      indexCount: data.indices.length,
      data: new Float32Array(STRIDE * 64),
      count: 0,
    };
    this.meshes.set(key, mesh);
    return mesh;
  }

  private meshFor(c: Collider): { mesh: GpuMesh; scale: [number, number, number] } {
    const s = c.shape;
    if (s instanceof Box) {
      const m = this.meshes.get('box') ?? this.upload('box', boxMesh());
      return { mesh: m, scale: [s.halfExtents.x, s.halfExtents.y, s.halfExtents.z] };
    }
    if (s instanceof Sphere) {
      const m = this.meshes.get('sphere') ?? this.upload('sphere', sphereMesh());
      return { mesh: m, scale: [s.radius, s.radius, s.radius] };
    }
    if (s instanceof Capsule) {
      const key = `capsule:${s.radius}:${s.halfHeight}`;
      const m = this.meshes.get(key) ?? this.upload(key, capsuleMesh(s.radius, s.halfHeight));
      return { mesh: m, scale: [1, 1, 1] };
    }
    if (s instanceof ConvexHull) {
      const m = this.meshes.get(s) ?? this.upload(s, flatMesh(s.vertices, s.indices));
      return { mesh: m, scale: [1, 1, 1] };
    }
    if (s instanceof TriangleMesh) {
      const m = this.meshes.get(s) ?? this.upload(s, flatMesh(s.vertices, s.indices));
      return { mesh: m, scale: [1, 1, 1] };
    }
    // Unknown convex shape: draw its bounding box so nothing is invisible.
    const m = this.meshes.get('box') ?? this.upload('box', boxMesh());
    return { mesh: m, scale: [0.5, 0.5, 0.5] };
  }

  private colorOf(body: RigidBody, flags: DebugFlags, selected: RigidBody | null): number[] {
    if (body === selected) return [1, 0.92, 0.45, 2];
    if (body.type === 'static') return [0.56, 0.58, 0.62, 1];
    if (body.type === 'kinematic') return [0.4, 0.55, 0.9, 1];
    if (flags.islands) {
      return body.awake ? [1, 0.55, 0.2, 1] : [0.3, 0.45, 0.75, 1];
    }
    const base = PALETTE[body.id % PALETTE.length] as [number, number, number];
    return body.awake ? [...base, 1] : [base[0] * 0.55, base[1] * 0.55, base[2] * 0.6, 1];
  }

  render(
    world: World,
    alpha: number,
    camera: Camera,
    flags: DebugFlags,
    selected: RigidBody | null,
  ): void {
    const gl = this.gl;
    this.resize();
    const w = this.canvas.width;
    const h = this.canvas.height;
    camera.update(w / h);
    this.drawCalls = 0;
    this.instances = 0;

    // Pack instances per mesh.
    for (const m of this.meshes.values()) m.count = 0;
    const pos = new Vec3();
    const q = new Quat();
    const wq = new Quat();
    const lp = new Vec3();
    for (const c of world.colliders) {
      const body = c.body;
      const { mesh, scale } = this.meshFor(c);
      const bp =
        body.type === 'static'
          ? body.position
          : lerp3(pos, body.prevPosition, body.position, alpha);
      const bq =
        body.type === 'static'
          ? body.orientation
          : Q.slerp(q, body.prevOrientation, body.orientation, alpha);
      Q.rotate(lp, bq, c.local.p);
      Q.mul(wq, bq, c.local.q);
      if (mesh.data.length < (mesh.count + 1) * STRIDE) {
        const bigger = new Float32Array(mesh.data.length * 2);
        bigger.set(mesh.data);
        mesh.data = bigger;
      }
      const off = mesh.count * STRIDE;
      M.compose(
        mesh.data,
        off,
        bp.x + lp.x,
        bp.y + lp.y,
        bp.z + lp.z,
        wq.x,
        wq.y,
        wq.z,
        wq.w,
        scale[0],
        scale[1],
        scale[2],
      );
      const col = this.colorOf(body, flags, selected);
      mesh.data[off + 16] = col[0] as number;
      mesh.data[off + 17] = col[1] as number;
      mesh.data[off + 18] = col[2] as number;
      mesh.data[off + 19] = col[3] as number;
      mesh.count++;
    }
    for (const m of this.meshes.values()) {
      if (m.count === 0) continue;
      gl.bindBuffer(gl.ARRAY_BUFFER, m.instanceBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, m.data.subarray(0, m.count * STRIDE), gl.DYNAMIC_DRAW);
    }

    // Light fitted around the camera target.
    const target = camera.fly
      ? [
          camera.eye[0] + camera.forward[0] * 20,
          camera.eye[1] + camera.forward[1] * 20,
          camera.eye[2] + camera.forward[2] * 20,
        ]
      : camera.target;
    const R = Math.max(22, Math.min(70, camera.distance * 1.1));
    const ld = this.lightDir;
    const lightEye = [
      (target[0] as number) - (ld[0] as number) * R * 2,
      (target[1] as number) - (ld[1] as number) * R * 2,
      (target[2] as number) - (ld[2] as number) * R * 2,
    ];
    M.lookAt(this.lightView, lightEye, target, [0, 0, 1]);
    M.ortho(this.lightProj, -R, R, -R, R, 0.1, R * 4);
    M.multiply(this.lightViewProj, this.lightProj, this.lightView);

    // Shadow pass.
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.shadowFbo);
    gl.viewport(0, 0, this.shadowSize, this.shadowSize);
    gl.clear(gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.disable(gl.CULL_FACE);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(2, 4);
    gl.useProgram(this.shadowProg);
    gl.uniformMatrix4fv(
      gl.getUniformLocation(this.shadowProg, 'uLightViewProj'),
      false,
      this.lightViewProj,
    );
    this.drawMeshes();
    gl.disable(gl.POLYGON_OFFSET_FILL);

    // Main pass.
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.62, 0.69, 0.78, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.mainProg);
    const u = (name: string) => gl.getUniformLocation(this.mainProg, name);
    gl.uniformMatrix4fv(u('uViewProj'), false, camera.viewProj);
    gl.uniformMatrix4fv(u('uLightViewProj'), false, this.lightViewProj);
    gl.uniform3fv(u('uLightDir'), this.lightDir);
    gl.uniform3fv(u('uEye'), camera.eye);
    gl.uniform1f(u('uShadowTexel'), 1 / this.shadowSize);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.shadowTex);
    gl.uniform1i(u('uShadow'), 0);
    this.drawMeshes();

    // Debug lines.
    this.lineCount = 0;
    this.buildDebugLines(world, flags);
    if (this.lineCount > 0) {
      gl.useProgram(this.lineProg);
      gl.uniformMatrix4fv(
        gl.getUniformLocation(this.lineProg, 'uViewProj'),
        false,
        camera.viewProj,
      );
      gl.bindVertexArray(this.lineVao);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.lineBuffer);
      gl.bufferData(
        gl.ARRAY_BUFFER,
        this.lineData.subarray(0, this.lineCount * 6),
        gl.DYNAMIC_DRAW,
      );
      gl.drawArrays(gl.LINES, 0, this.lineCount);
      gl.bindVertexArray(null);
      this.drawCalls++;
    }
  }

  private drawMeshes(): void {
    const gl = this.gl;
    for (const m of this.meshes.values()) {
      if (m.count === 0) continue;
      gl.bindVertexArray(m.vao);
      gl.drawElementsInstanced(gl.TRIANGLES, m.indexCount, gl.UNSIGNED_INT, 0, m.count);
      this.drawCalls++;
      this.instances += m.count;
    }
    gl.bindVertexArray(null);
  }

  private line(a: readonly number[], b: readonly number[], c: readonly number[]): void {
    if ((this.lineCount + 2) * 6 > this.lineData.length) {
      const bigger = new Float32Array(this.lineData.length * 2);
      bigger.set(this.lineData);
      this.lineData = bigger;
    }
    const o = this.lineCount * 6;
    this.lineData.set(
      [
        a[0] as number,
        a[1] as number,
        a[2] as number,
        c[0] as number,
        c[1] as number,
        c[2] as number,
      ],
      o,
    );
    this.lineData.set(
      [
        b[0] as number,
        b[1] as number,
        b[2] as number,
        c[0] as number,
        c[1] as number,
        c[2] as number,
      ],
      o + 6,
    );
    this.lineCount += 2;
  }

  private boxLines(min: Vec3, max: Vec3, color: readonly number[]): void {
    const xs = [min.x, max.x];
    const ys = [min.y, max.y];
    const zs = [min.z, max.z];
    const p = (i: number, j: number, k: number) => [
      xs[i] as number,
      ys[j] as number,
      zs[k] as number,
    ];
    for (let a = 0; a < 2; a++) {
      for (let b = 0; b < 2; b++) {
        this.line(p(0, a, b), p(1, a, b), color);
        this.line(p(a, 0, b), p(a, 1, b), color);
        this.line(p(a, b, 0), p(a, b, 1), color);
      }
    }
  }

  private buildDebugLines(world: World, flags: DebugFlags): void {
    if (flags.contacts || flags.normals) {
      for (const c of world.activeContacts()) {
        for (const p of c.points) {
          const x = c.bodyA.position.x + p.rA.x;
          const y = c.bodyA.position.y + p.rA.y;
          const z = c.bodyA.position.z + p.rA.z;
          if (flags.contacts) {
            const s = 0.06;
            const col = p.impulseN > 0 ? [1, 0.2, 0.2] : [1, 1, 0.3];
            this.line([x - s, y, z], [x + s, y, z], col);
            this.line([x, y - s, z], [x, y + s, z], col);
            this.line([x, y, z - s], [x, y, z + s], col);
          }
          if (flags.normals) {
            this.line(
              [x, y, z],
              [x + c.normal.x * 0.5, y + c.normal.y * 0.5, z + c.normal.z * 0.5],
              [0.3, 1, 0.4],
            );
          }
        }
      }
    }
    if (flags.aabbs) {
      for (const c of world.colliders) {
        if (c.shape instanceof TriangleMesh) continue;
        const b = world.colliderBounds(c);
        this.boxLines(b.min, b.max, c.body.awake ? [0.2, 0.9, 0.9] : [0.35, 0.45, 0.6]);
      }
    }
    if (flags.broadphase) {
      world.broadphase.forEachNode((box, depth, leaf) => {
        if (leaf) return;
        const t = Math.min(1, depth / 10);
        this.boxLines(box.min, box.max, [0.9 - t * 0.5, 0.4 + t * 0.4, 0.9]);
      });
    }
    if (flags.joints) {
      for (const j of world.joints) {
        const a = j.bodyA;
        const b = j.bodyB;
        const la = (j as unknown as { localA?: Vec3 }).localA;
        const lb = (j as unknown as { localB?: Vec3 }).localB;
        const rot = new Vec3();
        let pa: number[] = [a.position.x, a.position.y, a.position.z];
        let pb: number[] = [b.position.x, b.position.y, b.position.z];
        if (la) {
          Q.rotate(rot, a.orientation, la);
          pa = [a.position.x + rot.x, a.position.y + rot.y, a.position.z + rot.z];
          this.line([a.position.x, a.position.y, a.position.z], pa, [0.1, 0.9, 0.9]);
        }
        if (lb) {
          Q.rotate(rot, b.orientation, lb);
          pb = [b.position.x + rot.x, b.position.y + rot.y, b.position.z + rot.z];
          this.line([b.position.x, b.position.y, b.position.z], pb, [0.1, 0.9, 0.9]);
        }
        this.line(pa, pb, [1, 0.3, 0.9]);
      }
    }
  }
}

function lerp3(out: Vec3, a: Vec3, b: Vec3, t: number): Vec3 {
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.z = a.z + (b.z - a.z) * t;
  return out;
}
