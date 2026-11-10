import {
  BoxGeometry, BufferGeometry, ConeGeometry, CylinderGeometry, DirectionalLight,
  Float32BufferAttribute, GridHelper, Group, HemisphereLight, Line, LineBasicMaterial,
  LineLoop, MathUtils, Mesh, MeshBasicMaterial, MeshLambertMaterial, OctahedronGeometry,
  PerspectiveCamera, Plane, PlaneGeometry, Raycaster, Scene, ShadowMaterial, SphereGeometry,
  Vector2, Vector3, WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Mat4 } from '../robotics/math';
import type { Vec3 } from '../robotics/models';
import { stagedDhTransform } from '../robotics/kinematics';
import { multiply } from '../robotics/transforms';
import type { CameraView, ViewportProps } from './RobotViewport';

const axisNames = ['X', 'Y', 'Z'] as const;
const axisColors = [0xff817c, 0x79dfa2, 0x76b8ff] as const;
const axes = [new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)] as const;
const directions: Record<CameraView, readonly [number, number, number]> = {
  perspective: [1.35, -2, 1.45], front: [0, -1, 0], back: [0, 1, 0], left: [-1, 0, 0],
  right: [1, 0, 0], top: [0, -0.00001, 1], bottom: [0, -0.00001, -1], isometric: [1, -1, 1],
};
const background = 0x10151e;

export function createRobotScene(host: HTMLDivElement, overlay: HTMLDivElement, diagnostics: HTMLDivElement, antialias: boolean) {
  const renderer = new WebGLRenderer({ antialias, powerPreference: 'low-power' });
  const canvas = renderer.domElement;
  renderer.setClearColor(background);
  canvas.tabIndex = 0;
  canvas.setAttribute('aria-label', 'Robot model. Drag to orbit, right-drag or use arrow keys to pan, and scroll to zoom.');
  host.prepend(canvas);

  const scene = new Scene();
  const camera = new PerspectiveCamera(44, 1, 0.001, 1000);
  camera.up.set(0, 0, 1);
  camera.position.set(...directions.perspective);
  const controls = new OrbitControls(camera, canvas);
  controls.listenToKeyEvents(canvas);

  const resources: { dispose(): void }[] = [];
  const own = <T extends { dispose(): void }>(resource: T) => {
    resources.push(resource);
    return resource;
  };

  const ambient = new HemisphereLight(0xe5f1ff, 0x566379, 2.1);
  const sun = new DirectionalLight(0xffffff, 2.4);
  sun.shadow.mapSize.set(512, 512);
  sun.shadow.bias = -0.0002;
  const grid = own(new GridHelper(10, 20, 0x4e6682, 0x283647));
  grid.rotation.x = Math.PI / 2;
  const floor = new Mesh(own(new PlaneGeometry(10, 10)), own(new ShadowMaterial({ opacity: 0.3 })));
  floor.receiveShadow = true;
  scene.add(ambient, sun, sun.target, grid, floor);

  const rodGeometry = own(new CylinderGeometry(1, 1, 1, 10));
  const revoluteGeometry = own(new CylinderGeometry(1, 1, 1, 14).rotateX(Math.PI / 2));
  const prismaticGeometry = own(new BoxGeometry(1.8, 1.8, 1));
  const tipGeometry = own(new OctahedronGeometry(1));
  const markerGeometry = own(new SphereGeometry(1, 12, 8));
  const armMaterial = own(new MeshLambertMaterial({ color: 0x718cab }));
  const pivotMaterial = own(new MeshLambertMaterial({ color: 0xaabfd3 }));
  const baseMaterial = own(new MeshLambertMaterial({ color: 0x47617f }));
  const selectedMaterial = own(new MeshLambertMaterial({ color: 0xffab59 }));
  const tipMaterial = own(new MeshLambertMaterial({ color: 0x7debd3 }));
  const markerMaterial = own(new MeshBasicMaterial({ color: 0xffcb6d, transparent: true, opacity: 0.45, depthWrite: false }));
  const traceMaterial = own(new LineBasicMaterial({ color: 0x70e9d0 }));
  const jointAxisGeometry = own(new BufferGeometry().setFromPoints([new Vector3(0, 0, -0.7), new Vector3(0, 0, 1.2)]));
  const jointAxisMaterial = own(new LineBasicMaterial({ color: 0xffce72, depthTest: false }));
  const axisGeometries = axes.map((axis) => own(new BufferGeometry().setFromPoints([new Vector3(), axis])));
  const axisLines = axisColors.map((color) => own(new LineBasicMaterial({ color })));
  const arrowGeometry = own(new ConeGeometry(0.075, 0.21, 6));
  const arrowMaterials = axisColors.map((color) => own(new MeshBasicMaterial({ color })));
  const yAxis = axes[1];

  const base = new Mesh(revoluteGeometry, baseMaterial);
  const tip = new Mesh(tipGeometry, tipMaterial);
  const target = new Mesh(markerGeometry, markerMaterial);
  const targetOutlineGeometry = own(new BufferGeometry().setFromPoints(Array.from({ length: 32 }, (_, i) => {
    const angle = i * 2 * Math.PI / 32;
    return new Vector3(Math.cos(angle), Math.sin(angle), 0);
  })));
  const targetOutline = new LineLoop(targetOutlineGeometry, own(new LineBasicMaterial({ color: 0xffd391, depthTest: false })));
  target.add(targetOutline);
  base.castShadow = base.receiveShadow = tip.castShadow = true;
  scene.add(base, tip, target);

  function createFrame(index: number | 'DH') {
    const root = new Group();
    root.matrixAutoUpdate = false;
    const triad = new Group();
    const cones = new Group();
    const shaft = new Group();
    const jointAxis = new Line(jointAxisGeometry, jointAxisMaterial);
    const labels = axisNames.map((axis, i) => {
      const label = document.createElement('span');
      label.style.color = '#' + axisColors[i]!.toString(16);
      label.textContent = axis + index;
      overlay.append(label);
      return label;
    });
    axes.forEach((axis, i) => {
      shaft.add(new Line(axisGeometries[i], axisLines[i]));
      const cone = new Mesh(arrowGeometry, arrowMaterials[i]);
      cone.quaternion.setFromUnitVectors(yAxis, axis);
      cone.position.copy(axis).multiplyScalar(0.91);
      cones.add(cone);
    });
    triad.add(shaft, cones);
    root.add(triad, jointAxis);
    scene.add(root);
    return { root, triad, cones, jointAxis, labels };
  }

  function createJoint(index: number) {
    const housing = new Mesh<BufferGeometry, typeof pivotMaterial>(revoluteGeometry, pivotMaterial);
    const offset = new Mesh(rodGeometry, armMaterial);
    const length = new Mesh(rodGeometry, armMaterial);
    for (const part of [housing, offset, length]) {
      part.userData.joint = index;
      part.castShadow = part.receiveShadow = true;
    }
    scene.add(housing, offset, length);
    return { housing, offset, length };
  }

  const dhFrame = createFrame('DH');
  dhFrame.root.visible = false;
  const frameViews: ReturnType<typeof createFrame>[] = [];
  const jointViews: ReturnType<typeof createJoint>[] = [];
  const pickable: Mesh[] = [];

  let trace: Line | undefined;
  let props: ViewportProps | undefined;
  let frames: readonly Mat4[] = [];
  let size = 1;
  let width = 0;
  let height = 0;
  let requestedFrame = 0;
  let lastDraw = 0;
  let dragging: number | undefined;
  let pendingTarget: Vec3 | undefined;
  let downX = 0;
  let downY = 0;
  const dragPlane = new Plane();
  const dragPoint = new Vector3();
  const dragOffset = new Vector3();
  const raycaster = new Raycaster();
  const pointer = new Vector2();
  const origin = new Vector3();
  const end = new Vector3();
  const elbow = new Vector3();
  const direction = new Vector3();
  const zAxis = new Vector3();
  const center = new Vector3();
  const min = new Vector3();
  const max = new Vector3();
  const projected = new Vector3();
  const viewOffset = new Vector3();

  function invalidate() {
    if (requestedFrame === 0) requestedFrame = requestAnimationFrame(draw);
  }

  function projectLabels(frame: ReturnType<typeof createFrame>) {
    const visible = !!props?.settings.labels && frame.root.visible && frame.triad.visible;
    frame.labels.forEach((label, i) => {
      if (!visible) {
        label.hidden = true;
        return;
      }
      projected.copy(axes[i]!).multiplyScalar(1.13).applyMatrix4(frame.triad.matrixWorld).project(camera);
      label.hidden = projected.z < -1 || projected.z > 1 || Math.abs(projected.x) > 1 || Math.abs(projected.y) > 1;
      if (!label.hidden) {
        const x = Math.round((projected.x + 1) * width / 2);
        const y = Math.round((1 - projected.y) * height / 2);
        label.style.transform = 'translate(' + x + 'px, ' + y + 'px)';
      }
    });
  }

  function draw(time: number) {
    const nextTarget = pendingTarget;
    pendingTarget = undefined;
    if (nextTarget) props?.onTargetChange?.(nextTarget);
    requestedFrame = 0;
    if (width < 1 || height < 1) return;
    const started = performance.now();
    if (target.visible) target.quaternion.copy(camera.quaternion);
    renderer.render(scene, camera);
    frameViews.forEach(projectLabels);
    projectLabels(dhFrame);
    if (!diagnostics.hidden) {
      const elapsed = time - lastDraw;
      const cadence = lastDraw && elapsed > 0 && elapsed < 500 ? (1000 / elapsed).toFixed(0) + ' FPS · ' + elapsed.toFixed(1) + ' ms/frame' : 'Demand rendering';
      const { render, memory } = renderer.info;
      diagnostics.textContent = cadence + ' · ' + (performance.now() - started).toFixed(1) + ' ms CPU\n'
        + render.calls + ' draws · ' + render.triangles + ' triangles\n'
        + memory.geometries + ' geometries · ' + memory.textures + ' textures';
    }
    lastDraw = time;
  }

  function readPosition(matrix: Mat4, vector: Vector3) {
    return vector.set(matrix[3], matrix[7], matrix[11]);
  }

  function setRod(rod: Mesh, from: Vector3, to: Vector3, radius: number) {
    direction.subVectors(to, from);
    const length = direction.length();
    rod.visible = length > 1e-9;
    if (!rod.visible) return;
    rod.position.copy(from).addScaledVector(direction, 0.5);
    rod.quaternion.setFromUnitVectors(yAxis, direction.divideScalar(length));
    rod.scale.set(radius, length, radius);
  }

  function setFrameView(view: ReturnType<typeof createFrame>, transform: Mat4, triad: boolean, jointAxis: boolean) {
    view.root.matrix.set(...transform);
    view.root.matrixWorldNeedsUpdate = true;
    view.root.visible = triad || jointAxis;
    view.triad.visible = triad;
    view.cones.visible = props?.settings.arrows ?? true;
    view.jointAxis.visible = jointAxis;
    view.triad.scale.setScalar(size * 0.19);
    view.jointAxis.scale.setScalar(size * 0.24);
  }

  function applyPose() {
    const dh = props?.dh;
    const settings = props?.settings;
    const endEffector = frames.at(-1);
    const frameZero = frames[0];
    base.visible = !!frameZero;
    tip.visible = !!endEffector;
    if (frameZero) readPosition(frameZero, base.position).z -= size * 0.031;
    if (endEffector) readPosition(endEffector, tip.position);
    base.scale.set(size * 0.09, size * 0.09, size * 0.055);
    tip.scale.setScalar(size * 0.042);
    target.scale.setScalar(size * 0.065);

    frames.forEach((transform, i) => {
      const adjacent = !!dh && (i === dh.joint || i === dh.joint + 1);
      const showTriad = dh
        ? (dh.allFrames || adjacent) && (i !== dh.joint + 1 || dh.progress >= 1)
        : i === 0 || !!settings?.frames;
      setFrameView(frameViews[i]!, transform, showTriad, !!settings?.jointAxes && i < jointViews.length);

      const next = frames[i + 1];
      if (!next) return;
      const joint = jointViews[i]!;
      const selected = (dh?.joint ?? props?.selectedJoint) === i;
      const prismatic = props?.jointTypes?.[i] === 'prismatic';
      joint.housing.geometry = prismatic ? prismaticGeometry : revoluteGeometry;
      joint.housing.material = selected ? selectedMaterial : pivotMaterial;
      joint.offset.material = joint.length.material = selected ? selectedMaterial : armMaterial;
      readPosition(transform, origin);
      readPosition(next, end);
      zAxis.set(transform[2], transform[6], transform[10]);
      joint.housing.position.copy(origin);
      joint.housing.quaternion.setFromUnitVectors(axes[2], zAxis);
      const radius = size * 0.041;
      joint.housing.scale.set(radius, radius, radius * (prismatic ? 2.1 : 1.2));
      const displacementZ = direction.subVectors(end, origin).dot(zAxis);
      elbow.copy(origin).addScaledVector(zAxis, displacementZ);
      setRod(joint.offset, origin, elbow, size * (prismatic ? 0.027 : 0.023));
      setRod(joint.length, elbow, end, size * 0.021);
    });

    const previous = dh && frames[dh.joint];
    dhFrame.root.visible = false;
    if (dh && previous && dh.progress < 1) {
      const partial = stagedDhTransform(dh.angle, dh.offset, dh.length, dh.twist, dh.progress);
      setFrameView(dhFrame, multiply(previous, partial), true, false);
    }
    invalidate();
  }

  function setFrames(next: readonly Mat4[]) {
    const countChanged = next.length !== frames.length;
    const firstPose = frames.length === 0 && next.length > 0;
    frames = next;
    if (countChanged) {
      while (frameViews.length > frames.length) {
        const removed = frameViews.pop()!;
        scene.remove(removed.root);
        removed.labels.forEach((label) => label.remove());
      }
      while (frameViews.length < frames.length) frameViews.push(createFrame(frameViews.length));
      const jointCount = Math.max(0, frames.length - 1);
      while (jointViews.length > jointCount) {
        const removed = jointViews.pop()!;
        scene.remove(removed.housing, removed.offset, removed.length);
      }
      while (jointViews.length < jointCount) jointViews.push(createJoint(jointViews.length));
      pickable.length = 0;
      jointViews.forEach(({ housing, offset, length }) => pickable.push(housing, offset, length));
    }
    if (firstPose) setView('perspective');
    else applyPose();
  }

  function fitBounds() {
    min.set(0, 0, 0);
    max.set(0, 0, 0);
    let reach = 0;
    frames.forEach((frame, i) => {
      readPosition(frame, origin);
      min.min(origin);
      max.max(origin);
      const next = frames[i + 1];
      if (!next) return;
      readPosition(next, end);
      zAxis.set(frame[2], frame[6], frame[10]);
      const displacementZ = direction.subVectors(end, origin).dot(zAxis);
      elbow.copy(origin).addScaledVector(zAxis, displacementZ);
      min.min(elbow);
      max.max(elbow);
      reach += Math.abs(displacementZ) + elbow.distanceTo(end);
    });
    center.addVectors(min, max).multiplyScalar(0.5);
    size = Math.max(0.05, min.distanceTo(max), reach * 0.55);
    grid.scale.setScalar(size * 0.38);
    floor.scale.copy(grid.scale);
    grid.position.z = -size * 0.065;
    floor.position.z = grid.position.z - size * 0.001;
    sun.target.position.copy(center);
    sun.position.copy(center).addScaledVector(direction.set(-2, -4, 6), size);
    const shadowCamera = sun.shadow.camera;
    shadowCamera.left = shadowCamera.bottom = -size * 2;
    shadowCamera.right = shadowCamera.top = size * 2;
    shadowCamera.near = size * 0.1;
    shadowCamera.far = size * 12;
    shadowCamera.updateProjectionMatrix();

    camera.near = size * 0.001;
    camera.far = size * 160;
    camera.updateProjectionMatrix();
    controls.minDistance = size * 0.09;
    controls.maxDistance = size * 40;
    applyPose();
  }

  function fit() {
    viewOffset.subVectors(camera.position, controls.target).normalize();
    fitBounds();
    const vertical = MathUtils.degToRad(camera.fov / 2);
    const horizontal = Math.atan(Math.tan(vertical) * camera.aspect);
    const radius = Math.max(size * 0.25, min.distanceTo(max) * 0.56);
    const distance = radius / Math.sin(Math.min(vertical, horizontal));
    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(viewOffset, distance);
    controls.update();
    invalidate();
  }

  function setView(view: CameraView) {
    camera.position.copy(controls.target).add(viewOffset.set(...directions[view]));
    fit();
  }

  function resize() {
    width = host.clientWidth;
    height = host.clientHeight;
    if (width < 1 || height < 1) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    invalidate();
  }

  function update(next: ViewportProps) {
    const previous = props;
    props = next;
    const { settings } = next;
    const cap = settings.profile === 'low' ? 1 : settings.profile === 'high' ? 2 : 1.5;
    const pixelRatio = Math.min(window.devicePixelRatio || 1, MathUtils.clamp(settings.pixelRatio ?? cap, 0.5, 2));
    if (renderer.getPixelRatio() !== pixelRatio) renderer.setPixelRatio(pixelRatio);
    grid.visible = settings.grid;
    floor.visible = sun.castShadow = renderer.shadowMap.enabled = settings.profile === 'high' && settings.shadows;
    diagnostics.hidden = !(import.meta.env.DEV && settings.diagnostics);

    target.visible = next.target !== undefined;
    if (next.target && dragging === undefined) target.position.set(...next.target);

    if (next.trace !== previous?.trace) {
      if (trace) {
        scene.remove(trace);
        trace.geometry.dispose();
        trace = undefined;
      }
      if (next.trace && next.trace.length > 1) {
        const positions = new Float32Array(next.trace.length * 3);
        next.trace.forEach((point, i) => positions.set(point, i * 3));
        const geometry = new BufferGeometry().setAttribute('position', new Float32BufferAttribute(positions, 3));
        trace = new Line(geometry, traceMaterial);
        scene.add(trace);
      }
    }

    if (next.frames !== previous?.frames) setFrames(next.frames);
    else applyPose();
  }

  function pointRay(event: PointerEvent) {
    const rect = canvas.getBoundingClientRect();
    pointer.set(
      2 * (event.clientX - rect.left) / rect.width - 1,
      1 - 2 * (event.clientY - rect.top) / rect.height,
    );
    raycaster.setFromCamera(pointer, camera);
  }

  function pointerDown(event: PointerEvent) {
    if (event.button !== 0 || !event.isPrimary) return;
    downX = event.clientX;
    downY = event.clientY;
    if (!target.visible || !props?.onTargetChange) return;
    pointRay(event);
    if (!raycaster.intersectObject(target, false).length) return;
    dragPlane.setFromNormalAndCoplanarPoint(camera.getWorldDirection(direction), target.position);
    if (!raycaster.ray.intersectPlane(dragPlane, dragPoint)) return;
    dragOffset.subVectors(target.position, dragPoint);
    dragging = event.pointerId;
    canvas.setPointerCapture(event.pointerId);
    controls.enabled = false;
    canvas.style.cursor = 'grabbing';
    event.stopImmediatePropagation();
  }

  function pointerMove(event: PointerEvent) {
    if (dragging !== event.pointerId) return;
    pointRay(event);
    if (!raycaster.ray.intersectPlane(dragPlane, dragPoint)) return;
    target.position.copy(dragPoint).add(dragOffset);
    pendingTarget = [target.position.x, target.position.y, target.position.z];
    invalidate();
  }

  function releaseDrag() {
    if (dragging === undefined) return;
    const pointerId = dragging;
    dragging = undefined;
    controls.enabled = true;
    if (canvas.hasPointerCapture(pointerId)) canvas.releasePointerCapture(pointerId);
    canvas.style.cursor = '';
  }

  function pointerUp(event: PointerEvent) {
    if (dragging === event.pointerId) {
      releaseDrag();
      return;
    }
    if (event.button !== 0 || !event.isPrimary || Math.hypot(event.clientX - downX, event.clientY - downY) > 4) return;
    if (!props?.onSelectJoint) return;
    pointRay(event);
    const hit = raycaster.intersectObjects(pickable, false).find(({ object }) => object.visible);
    if (hit) props.onSelectJoint(hit.object.userData.joint as number);
  }

  const observer = new ResizeObserver(resize);
  observer.observe(host);
  controls.addEventListener('change', invalidate);
  canvas.addEventListener('pointerdown', pointerDown, true);
  canvas.addEventListener('pointermove', pointerMove);
  canvas.addEventListener('pointerup', pointerUp);
  canvas.addEventListener('pointercancel', releaseDrag);
  canvas.addEventListener('lostpointercapture', releaseDrag);
  canvas.addEventListener('webglcontextrestored', invalidate);
  resize();

  return {
    update, setFrames, setView, fit,
    dispose() {
      observer.disconnect();
      cancelAnimationFrame(requestedFrame);
      controls.dispose();
      canvas.removeEventListener('pointerdown', pointerDown, true);
      canvas.removeEventListener('pointermove', pointerMove);
      canvas.removeEventListener('pointerup', pointerUp);
      canvas.removeEventListener('pointercancel', releaseDrag);
      canvas.removeEventListener('lostpointercapture', releaseDrag);
      canvas.removeEventListener('webglcontextrestored', invalidate);
      releaseDrag();
      trace?.geometry.dispose();
      sun.shadow.dispose();
      resources.forEach((resource) => resource.dispose());
      renderer.dispose();
      renderer.forceContextLoss();
      canvas.remove();
      overlay.replaceChildren();
    },
  };
}
