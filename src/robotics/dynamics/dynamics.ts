import { computeFrames } from '../kinematics';
import { cross, failure, subtract, success, type Result } from '../math';
import { validateRobot, type Mat3, type Robot, type Vec3 } from '../models';
import { orientationOf, transposeRotation } from '../transforms';
import type { MotionSample } from '../trajectories/trajectory';

interface DynamicsState {
  q: readonly number[];
  velocity: readonly number[];
  gravity?: Vec3;
}

export interface InverseDynamicsInput extends DynamicsState {
  acceleration: readonly number[];
}

export interface ForwardDynamicsInput extends DynamicsState {
  effort: readonly number[];
}

export interface SimulationInput extends ForwardDynamicsInput {
  duration: number;
  timeStep?: number;
}

const zero: Vec3 = [0, 0, 0];
const defaultGravity: Vec3 = [0, 0, -9.80665];
const maxSimulationSteps = 5000;

function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(v: Vec3, factor: number): Vec3 {
  return [factor * v[0], factor * v[1], factor * v[2]];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function applyMatrix(matrix: Mat3, [x, y, z]: Vec3): Vec3 {
  return [
    matrix[0] * x + matrix[1] * y + matrix[2] * z,
    matrix[3] * x + matrix[4] * y + matrix[5] * z,
    matrix[6] * x + matrix[7] * y + matrix[8] * z,
  ];
}

function prepareLinks(robot: Robot, q: readonly number[]) {
  const { frames, positions } = computeFrames(robot, q);
  return robot.joints.map((joint, index) => {
    const parentFrame = frames[index]!;
    const orientation = orientationOf(frames[index + 1]!);
    return {
      joint,
      axis: [parentFrame[2], parentFrame[6], parentFrame[10]] as Vec3,
      offset: subtract(positions[index + 1]!, positions[index]!),
      comOffset: applyMatrix(orientation, joint.centerOfMass),
      orientation,
      inverseOrientation: transposeRotation(orientation),
    };
  });
}

// Recursive Newton–Euler, with all twists, accelerations and wrenches expressed in world axes.
function jointEffort(links: ReturnType<typeof prepareLinks>, velocity: readonly number[], acceleration: readonly number[], gravity: Vec3): number[] {
  let angularVelocity = zero;
  let angularAcceleration = zero;
  let originAcceleration = scale(gravity, -1);
  const wrenches = links.map(({ joint, axis, offset, comOffset, orientation, inverseOrientation }, index) => {
    const jointVelocity = scale(axis, velocity[index]!);
    const jointAcceleration = scale(axis, acceleration[index]!);
    if (joint.type === 'revolute') {
      angularAcceleration = add(add(angularAcceleration, jointAcceleration), cross(angularVelocity, jointVelocity));
      angularVelocity = add(angularVelocity, jointVelocity);
    }

    originAcceleration = add(add(originAcceleration, cross(angularAcceleration, offset)),
      cross(angularVelocity, cross(angularVelocity, offset)));
    if (joint.type === 'prismatic') {
      originAcceleration = add(add(originAcceleration, jointAcceleration), scale(cross(angularVelocity, jointVelocity), 2));
    }

    const comAcceleration = add(add(originAcceleration, cross(angularAcceleration, comOffset)),
      cross(angularVelocity, cross(angularVelocity, comOffset)));
    const bodyAngularVelocity = applyMatrix(inverseOrientation, angularVelocity);
    const bodyAngularAcceleration = applyMatrix(inverseOrientation, angularAcceleration);
    const bodyMoment = add(applyMatrix(joint.inertia, bodyAngularAcceleration),
      cross(bodyAngularVelocity, applyMatrix(joint.inertia, bodyAngularVelocity)));
    return { force: scale(comAcceleration, joint.mass), moment: applyMatrix(orientation, bodyMoment) };
  });

  let force = zero;
  let moment = zero;
  const effort = new Array<number>(links.length);
  for (let index = links.length - 1; index >= 0; index--) {
    const { joint, axis, offset, comOffset } = links[index]!;
    const wrench = wrenches[index]!;
    force = add(force, wrench.force);
    moment = add(add(add(moment, wrench.moment), cross(comOffset, wrench.force)), cross(offset, force));
    effort[index] = dot(joint.type === 'revolute' ? moment : force, axis);
  }
  return effort;
}

function solveMassMatrix(columns: readonly number[][], rhs: number[]): number[] | undefined {
  const size = columns.length;
  const lower = Array.from({ length: size }, (_, row) => new Float64Array(row + 1));
  const solved = Array<number>(size);
  for (let row = 0; row < size; row++) {
    const entries = lower[row]!;
    for (let col = 0; col <= row; col++) {
      let pivot = columns[col]![row]!;
      for (let index = 0; index < col; index++) pivot -= entries[index]! * lower[col]![index]!;
      if (row === col) {
        const tolerance = 64 * Number.EPSILON * Math.abs(columns[row]![row]!);
        if (!Number.isFinite(pivot) || pivot <= tolerance) return undefined;
        entries[col] = Math.sqrt(pivot);
      } else {
        entries[col] = pivot / lower[col]![col]!;
      }
    }
    let value = rhs[row]!;
    for (let col = 0; col < row; col++) value -= entries[col]! * solved[col]!;
    solved[row] = value / entries[row]!;
  }
  for (let row = size - 1; row >= 0; row--) {
    let value = solved[row]!;
    for (let col = row + 1; col < size; col++) value -= lower[col]![row]! * solved[col]!;
    solved[row] = value / lower[row]![row]!;
  }
  return solved.every(Number.isFinite) ? solved : undefined;
}

function solveAcceleration(robot: Robot, q: readonly number[], velocity: readonly number[], effort: readonly number[], gravity: Vec3): Result<number[]> {
  const links = prepareLinks(robot, q);
  const resting = Array<number>(links.length).fill(0);
  const bias = jointEffort(links, velocity, resting, gravity);
  const unitAcceleration = resting.slice();
  const columns = links.map((_, col) => {
    unitAcceleration[col] = 1;
    const column = jointEffort(links, resting, unitAcceleration, zero);
    unitAcceleration[col] = 0;
    return column;
  });
  const acceleration = solveMassMatrix(columns, bias.map((value, index) => effort[index]! - value));
  return acceleration
    ? success(acceleration)
    : failure('Dynamics mass matrix is singular, ill-conditioned, or outside the supported numeric range. Check link mass and inertia.');
}

function validateState(robot: Robot, state: DynamicsState, values: readonly number[]): string | undefined {
  if ([state.q, state.velocity, values].some((vector) => vector.length !== robot.joints.length ||
    Array.from(vector).some((value) => !Number.isFinite(value)))) {
    return 'Dynamics requires one finite value per joint for position, velocity, and acceleration or effort.';
  }
  if (state.gravity !== undefined && (state.gravity.length !== 3 || Array.from(state.gravity).some((value) => !Number.isFinite(value)))) {
    return 'Gravity must be a finite three-dimensional vector.';
  }
  const joint = outsideLimits(robot, state.q);
  if (joint >= 0) return `Joint ${joint + 1} is outside its position limits.`;
}

function outsideLimits(robot: Robot, q: readonly number[]): number {
  return robot.joints.findIndex(({ min, max }, index) => q[index]! < min || q[index]! > max);
}

export function inverseDynamics(robot: Robot, input: InverseDynamicsInput): Result<number[]> {
  const validated = validateRobot(robot);
  if (!validated.success) return validated;
  const error = validateState(validated.data, input, input.acceleration);
  if (error) return failure(error);
  const efforts = jointEffort(prepareLinks(validated.data, input.q), input.velocity, input.acceleration, input.gravity ?? defaultGravity);
  return efforts.every(Number.isFinite) ? success(efforts) : failure('Joint effort exceeds the supported numeric range.');
}

export function forwardDynamics(robot: Robot, input: ForwardDynamicsInput): Result<number[]> {
  const validated = validateRobot(robot);
  if (!validated.success) return validated;
  const error = validateState(validated.data, input, input.effort);
  if (error) return failure(error);
  return solveAcceleration(validated.data, input.q, input.velocity, input.effort, input.gravity ?? defaultGravity);
}

function addScaled(state: readonly number[], derivative: readonly number[], step: number): number[] {
  return state.map((value, index) => value + step * derivative[index]!);
}

function rk4Update(state: readonly number[], k1: readonly number[], k2: readonly number[], k3: readonly number[], k4: readonly number[], step: number): number[] {
  return state.map((value, index) => value + step / 6 * (k1[index]! + 2 * k2[index]! + 2 * k3[index]! + k4[index]!));
}

export function simulateDynamics(robot: Robot, input: SimulationInput): Result<MotionSample[]> {
  const validated = validateRobot(robot);
  if (!validated.success) return validated;
  const error = validateState(validated.data, input, input.effort);
  if (error) return failure(error);
  const { duration, timeStep = 0.01, effort, gravity = defaultGravity } = input;
  const steps = Math.ceil(duration / timeStep);
  if (!Number.isFinite(duration) || duration <= 0 || !Number.isFinite(timeStep) || timeStep <= 0 || steps < 1 || steps > maxSimulationSteps) {
    return failure(`Enter a positive duration and time step that require at most ${maxSimulationSteps} integration steps.`);
  }

  const step = duration / steps;
  const samples: MotionSample[] = [];
  let position = Array.from(input.q);
  let velocity = Array.from(input.velocity);
  const accelerationAt = (q: readonly number[], v: readonly number[]) => solveAcceleration(validated.data, q, v, effort, gravity);
  for (let index = 0; index <= steps; index++) {
    const time = index === steps ? duration : index * step;
    if (!position.every(Number.isFinite) || !velocity.every(Number.isFinite)) return failure(`Simulation exceeded the numeric range at t=${time} s.`);
    const joint = outsideLimits(validated.data, position);
    if (joint >= 0) return failure(`Joint ${joint + 1} exceeded its position limits at t=${time} s.`);

    const a1 = accelerationAt(position, velocity);
    if (!a1.success) return a1;
    samples.push({ time, position, velocity, acceleration: a1.data });
    if (index === steps) break;

    const v2 = addScaled(velocity, a1.data, step / 2);
    const a2 = accelerationAt(addScaled(position, velocity, step / 2), v2);
    if (!a2.success) return a2;
    const v3 = addScaled(velocity, a2.data, step / 2);
    const a3 = accelerationAt(addScaled(position, v2, step / 2), v3);
    if (!a3.success) return a3;
    const v4 = addScaled(velocity, a3.data, step);
    const a4 = accelerationAt(addScaled(position, v3, step), v4);
    if (!a4.success) return a4;

    position = rk4Update(position, velocity, v2, v3, v4, step);
    velocity = rk4Update(velocity, a1.data, a2.data, a3.data, a4.data, step);
  }
  return success(samples);
}
