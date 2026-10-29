import * as M from './mat4';
import type { CameraStart } from './scenes';

/** Orbit camera with an optional free-fly mode. Angles in radians; y is up. */
export class Camera {
  target: [number, number, number] = [0, 0, 0];
  distance = 15;
  yaw = 0.4;
  pitch = 0.3;
  fov = (55 * Math.PI) / 180;
  /** Free-fly mode keeps its own eye position. */
  fly = false;
  private flyEye: [number, number, number] = [0, 5, 15];
  readonly view = M.create();
  readonly proj = M.create();
  readonly viewProj = M.create();
  private readonly inv = M.create();

  reset(start: CameraStart): void {
    this.target = [...start.target];
    this.distance = start.distance;
    this.yaw = start.yaw;
    this.pitch = start.pitch;
    this.fly = false;
  }

  get eye(): [number, number, number] {
    if (this.fly) return this.flyEye;
    const cp = Math.cos(this.pitch);
    return [
      this.target[0] + this.distance * cp * Math.sin(this.yaw),
      this.target[1] + this.distance * Math.sin(this.pitch),
      this.target[2] + this.distance * cp * Math.cos(this.yaw),
    ];
  }

  /** Unit forward vector. */
  get forward(): [number, number, number] {
    const cp = Math.cos(this.pitch);
    return [-cp * Math.sin(this.yaw), -Math.sin(this.pitch), -cp * Math.cos(this.yaw)];
  }

  toggleFly(): void {
    if (!this.fly) this.flyEye = this.eye;
    else {
      const f = this.forward;
      this.target = [
        this.flyEye[0] + f[0] * this.distance,
        this.flyEye[1] + f[1] * this.distance,
        this.flyEye[2] + f[2] * this.distance,
      ];
    }
    this.fly = !this.fly;
  }

  orbit(dx: number, dy: number): void {
    this.yaw -= dx * 0.005;
    this.pitch = Math.max(-1.5, Math.min(1.5, this.pitch + dy * 0.005));
  }

  zoom(delta: number): void {
    this.distance = Math.max(1, Math.min(300, this.distance * Math.exp(delta * 0.001)));
  }

  pan(dx: number, dy: number): void {
    const f = this.forward;
    // right = forward x up, up' = right x forward
    let rx = f[1] * 0 - f[2] * 1;
    let rz = f[0] * 1 - f[1] * 0;
    const rl = Math.hypot(rx, rz) || 1;
    rx /= rl;
    rz /= rl;
    const k = this.distance * 0.0015;
    this.target[0] -= rx * dx * k;
    this.target[2] -= rz * dx * k;
    this.target[1] += dy * k;
  }

  /** Free-fly movement along forward/right/up for `dt` seconds. */
  move(forward: number, right: number, up: number, dt: number, speed = 12): void {
    if (!this.fly) return;
    const f = this.forward;
    let rx = -f[2];
    let rz = f[0];
    const rl = Math.hypot(rx, rz) || 1;
    rx /= rl;
    rz /= rl;
    const s = speed * dt;
    this.flyEye[0] += (f[0] * forward + rx * right) * s;
    this.flyEye[1] += (f[1] * forward + up) * s;
    this.flyEye[2] += (f[2] * forward + rz * right) * s;
  }

  update(aspect: number): void {
    const eye = this.eye;
    const f = this.forward;
    const look = this.fly ? [eye[0] + f[0], eye[1] + f[1], eye[2] + f[2]] : this.target;
    M.lookAt(this.view, eye, look, [0, 1, 0]);
    M.perspective(this.proj, this.fov, aspect, 0.1, 500);
    M.multiply(this.viewProj, this.proj, this.view);
  }

  /** World-space ray through a pixel (origin, unit direction). */
  ray(px: number, py: number, width: number, height: number): { origin: number[]; dir: number[] } {
    M.invert(this.inv, this.viewProj);
    const x = (px / width) * 2 - 1;
    const y = 1 - (py / height) * 2;
    const near = M.transformPoint(this.inv, x, y, -1);
    const far = M.transformPoint(this.inv, x, y, 1);
    const n = [near[0] / near[3], near[1] / near[3], near[2] / near[3]];
    const f = [far[0] / far[3], far[1] / far[3], far[2] / far[3]];
    const d = [f[0]! - n[0]!, f[1]! - n[1]!, f[2]! - n[2]!];
    const l = Math.hypot(d[0]!, d[1]!, d[2]!) || 1;
    return { origin: n, dir: [d[0]! / l, d[1]! / l, d[2]! / l] };
  }
}
