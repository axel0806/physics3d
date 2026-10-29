import { Vec3 } from '../src/math';
import type { RigidBody } from '../src/dynamics/body';
import { Camera } from './camera';
import { defaultDebug, Renderer, type DebugFlags } from './renderer';
import { scenes, type SceneInstance, type SceneSpec } from './scenes';

const $ = <T extends HTMLElement>(id: string): T => document.getElementById(id) as T;
const params = new URLSearchParams(location.search);
const AUTO = params.get('auto') === '1';

const canvas = $<HTMLCanvasElement>('view');
const renderer = new Renderer(canvas);
const camera = new Camera();
const debug: DebugFlags = defaultDebug();

let spec: SceneSpec = scenes.find((s) => s.id === params.get('scene')) ?? (scenes[0] as SceneSpec);
let inst: SceneInstance = spec.build();
let playing = true;
let stepOnce = false;
let simTime = 0;
let alpha = 0;
let selected: RigidBody | null = null;
let speed = 1;
let frames = 0;
const stepMs: number[] = [];

// ---- controls ----------------------------------------------------------------------------------
const sceneSelect = $<HTMLSelectElement>('scene');
for (const group of ['validation', 'playground'] as const) {
  const og = document.createElement('optgroup');
  og.label = group === 'validation' ? 'Validation scenes' : 'Playground';
  for (const s of scenes.filter((x) => x.group === group)) {
    const o = document.createElement('option');
    o.value = s.id;
    o.textContent = s.name;
    og.appendChild(o);
  }
  sceneSelect.appendChild(og);
}
const hz = $<HTMLInputElement>('hz');
const vit = $<HTMLInputElement>('vit');
const pit = $<HTMLInputElement>('pit');
const speedInput = $<HTMLInputElement>('speed');

function bindSlider(
  input: HTMLInputElement,
  out: string,
  format: (v: number) => string,
  apply: (v: number) => void,
): void {
  const update = () => {
    const v = Number(input.value);
    $(out).textContent = format(v);
    apply(v);
  };
  input.addEventListener('input', update);
  update();
}

bindSlider(
  hz,
  'hzOut',
  (v) => `${v} Hz`,
  (v) => {
    inst.world.fixedDt = 1 / v;
    inst.world.maxSubSteps = Math.max(8, Math.ceil(v / 60) * 2);
  },
);
bindSlider(
  vit,
  'vitOut',
  (v) => `${v}`,
  (v) => (inst.world.velocityIterations = v),
);
bindSlider(
  pit,
  'pitOut',
  (v) => `${v}`,
  (v) => (inst.world.positionIterations = v),
);
bindSlider(
  speedInput,
  'speedOut',
  (v) => `${v.toFixed(1)}x`,
  (v) => (speed = v),
);

const checks = $('checks');
for (const key of Object.keys(debug) as (keyof DebugFlags)[]) {
  const label = document.createElement('label');
  const box = document.createElement('input');
  box.type = 'checkbox';
  box.checked = debug[key];
  box.addEventListener('change', () => (debug[key] = box.checked));
  label.append(box, ` ${key}`);
  checks.appendChild(label);
}

function loadScene(id: string): void {
  spec = scenes.find((s) => s.id === id) ?? (scenes[0] as SceneSpec);
  inst = spec.build();
  inst.world.clock = () => performance.now();
  camera.reset(inst.camera);
  simTime = 0;
  alpha = 0;
  selected = null;
  stepMs.length = 0;
  sceneSelect.value = spec.id;
  $('desc').textContent = spec.description;
  hz.value = String(Math.round(1 / (spec.fixedDt ?? 1 / 120)));
  vit.value = String(inst.world.velocityIterations);
  pit.value = String(inst.world.positionIterations);
  hz.dispatchEvent(new Event('input'));
  vit.dispatchEvent(new Event('input'));
  pit.dispatchEvent(new Event('input'));
  history.replaceState(null, '', `?scene=${spec.id}${AUTO ? '&auto=1' : ''}`);
}

sceneSelect.addEventListener('change', () => loadScene(sceneSelect.value));
$('play').addEventListener('click', () => {
  playing = !playing;
  $('play').textContent = playing ? 'Pause' : 'Play';
});
$('step').addEventListener('click', () => {
  playing = false;
  $('play').textContent = 'Play';
  stepOnce = true;
});
$('reset').addEventListener('click', () => loadScene(spec.id));

// ---- input -------------------------------------------------------------------------------------
const keys = new Set<string>();
addEventListener('keydown', (e) => {
  if ((e.target as HTMLElement).tagName === 'SELECT') return;
  keys.add(e.key.toLowerCase());
  if (e.key.toLowerCase() === 'f') camera.toggleFly();
  if (e.key === ' ') {
    e.preventDefault();
    $('play').click();
  }
});
addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));

let drag: { x: number; y: number; moved: number; button: number; shift: boolean } | null = null;
canvas.addEventListener('contextmenu', (e) => e.preventDefault());
canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  drag = { x: e.clientX, y: e.clientY, moved: 0, button: e.button, shift: e.shiftKey };
});
canvas.addEventListener('pointermove', (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x;
  const dy = e.clientY - drag.y;
  drag.x = e.clientX;
  drag.y = e.clientY;
  drag.moved += Math.abs(dx) + Math.abs(dy);
  if (drag.button === 2 || drag.shift) camera.pan(dx, dy);
  else camera.orbit(dx, dy);
});
canvas.addEventListener('pointerup', (e) => {
  const d = drag;
  drag = null;
  if (!d || d.moved > 5 || d.button !== 0) return;
  const rect = canvas.getBoundingClientRect();
  const r = camera.ray(e.clientX - rect.left, e.clientY - rect.top, rect.width, rect.height);
  const origin = new Vec3(r.origin[0], r.origin[1], r.origin[2]);
  const dir = new Vec3(r.dir[0], r.dir[1], r.dir[2]);
  if (inst.onClick && !e.shiftKey) {
    inst.onClick(origin, dir);
    return;
  }
  const hit = inst.world.raycast(origin, dir, 500);
  selected = hit ? hit.collider.body : null;
});
canvas.addEventListener(
  'wheel',
  (e) => {
    e.preventDefault();
    camera.zoom(e.deltaY);
  },
  { passive: false },
);

// ---- stats -------------------------------------------------------------------------------------
const graph = $<HTMLCanvasElement>('graph');
const g2 = graph.getContext('2d') as CanvasRenderingContext2D;

function drawGraph(): void {
  g2.clearRect(0, 0, graph.width, graph.height);
  const max = Math.max(16.7, ...stepMs);
  g2.strokeStyle = 'rgba(255,255,255,0.25)';
  g2.beginPath();
  const y60 = graph.height - (16.7 / max) * (graph.height - 4);
  g2.moveTo(0, y60);
  g2.lineTo(graph.width, y60);
  g2.stroke();
  g2.fillStyle = '#5eb0ff';
  const w = graph.width / 120;
  stepMs.forEach((v, i) => {
    const h = (v / max) * (graph.height - 4);
    g2.fillRect(i * w, graph.height - h, Math.max(1, w - 1), h);
  });
}

function updateStats(): void {
  const w = inst.world;
  const sorted = [...stepMs].sort((a, b) => a - b);
  const med = sorted.length ? (sorted[Math.floor(sorted.length / 2)] as number) : 0;
  const t = w.timings;
  const dyn = w.bodies.filter((b) => b.isDynamic);
  $('stats').textContent =
    `bodies   ${dyn.length} (${dyn.filter((b) => b.awake).length} awake)\n` +
    `contacts ${w.activeContacts().length}   joints ${w.joints.length}\n` +
    `step     ${med.toFixed(2)} ms median\n` +
    `  bp ${t.broadphase.toFixed(2)}  np ${t.narrowphase.toFixed(2)}  solve ${t.solve.toFixed(2)}  int ${t.integrate.toFixed(2)}\n` +
    `draws    ${renderer.drawCalls} calls, ${renderer.instances} instances\n` +
    `time     ${simTime.toFixed(2)} s   dt 1/${Math.round(1 / w.fixedDt)}`;
  if (selected) {
    const b = selected;
    const f = (v: Vec3) => `${v.x.toFixed(2)}, ${v.y.toFixed(2)}, ${v.z.toFixed(2)}`;
    $('inspector').textContent =
      `body ${b.id}  ${b.type}${b.awake ? '' : ' (asleep)'}\n` +
      `mass     ${b.mass.toFixed(3)} kg\n` +
      `pos      ${f(b.position)}\n` +
      `vel      ${f(b.linearVelocity)}\n` +
      `spin     ${f(b.angularVelocity)}\n` +
      `shapes   ${b.colliders.map((c) => c.shape.kind).join(', ')}`;
  } else $('inspector').textContent = 'click a body (shift-click in throw scenes)';
}

// ---- main loop ---------------------------------------------------------------------------------
let last = performance.now();
let statsClock = 0;
const AUTO_FRAMES = 300;

function frame(now: number): void {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  const w = inst.world;
  const fwd = (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0);
  const right = (keys.has('d') ? 1 : 0) - (keys.has('a') ? 1 : 0);
  const up = (keys.has('e') ? 1 : 0) - (keys.has('q') ? 1 : 0);
  camera.move(fwd, right, up, dt);

  const frameTime = AUTO ? 1 / 60 : dt * speed;
  const t0 = performance.now();
  if (playing || stepOnce) {
    const before = w.frame;
    if (stepOnce) {
      w.step(w.fixedDt);
      stepOnce = false;
      alpha = 1;
    } else {
      alpha = w.advance(frameTime);
    }
    const stepsDone = w.frame - before;
    if (stepsDone > 0) {
      simTime += stepsDone * w.fixedDt;
      inst.update?.(simTime, w.fixedDt);
      stepMs.push((performance.now() - t0) / stepsDone);
      if (stepMs.length > 120) stepMs.shift();
    }
  }
  renderer.render(w, playing ? alpha : 1, camera, debug, selected);
  frames++;
  statsClock += dt;
  if (statsClock > 0.2 || AUTO) {
    statsClock = 0;
    updateStats();
    drawGraph();
  }
  if (AUTO && frames === AUTO_FRAMES) finishAuto();
  else requestAnimationFrame(frame);
}

function finishAuto(): void {
  let bad = 0;
  for (const b of inst.world.bodies) {
    const p = b.position;
    if (![p.x, p.y, p.z, b.orientation.w].every(Number.isFinite)) bad++;
  }
  (window as unknown as { __report: unknown }).__report = {
    scene: spec.id,
    frames,
    simTime,
    bodies: inst.world.bodies.length,
    nonFinite: bad,
    drawCalls: renderer.drawCalls,
    instances: renderer.instances,
  };
  document.title = 'done';
}

loadScene(spec.id);
requestAnimationFrame(frame);
