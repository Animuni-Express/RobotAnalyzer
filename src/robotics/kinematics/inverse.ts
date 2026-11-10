import { clamp, failure, finiteVector, solveLinearSystem, subtract, success } from "../math";
import type { Mat3, Result, Vec3 } from "../math";
import { validateRobot } from "../models";
import type { Joint, Robot } from "../models";
import { isRotation, multiplyRotation, orientationOf, rotationLog, rpyFromRotation, transposeRotation } from "../transforms";
import { computeFrames, jacobianFromFrames } from "./index";
import type { ForwardFrames } from "./index";

export interface IKTarget {
  position: Vec3;
  orientation?: Mat3;
}

export interface IKOptions {
  maxIterations?: number;
  positionTolerance?: number;
  orientationTolerance?: number;
  damping?: number;
}

export interface IKSolution {
  solutions: number[][];
  iterations: number;
  residual: number;
}

interface PoseError {
  vector: number[];
  position: number;
  orientation: number;
  norm: number;
}

function poseError(target: IKTarget, frames: ForwardFrames): PoseError {
  const position = subtract(target.position, frames.positions[frames.positions.length - 1]!);
  const orientation = target.orientation
    ? rotationLog(multiplyRotation(target.orientation, transposeRotation(orientationOf(frames.endEffector))))
    : [];
  const vector = [...position, ...orientation];
  return { vector, position: Math.hypot(...position), orientation: Math.hypot(...orientation), norm: Math.hypot(...vector) };
}

function withinTolerance(error: PoseError, options: Required<IKOptions>): boolean {
  return error.position <= options.positionTolerance && error.orientation <= options.orientationTolerance;
}

function equivalentWithinLimits(angle: number, joint: Joint, seed: number): number | undefined {
  const turn = 2 * Math.PI;
  const offset = angle - joint.theta;
  const min = Math.ceil((joint.min - offset - 1e-12) / turn);
  const max = Math.floor((joint.max - offset + 1e-12) / turn);
  if (min > max) return;
  const turns = clamp(Math.round((seed - offset) / turn), min, max);
  return clamp(offset + turns * turn, joint.min, joint.max);
}

function analyticPlanar(robot: Robot, target: IKTarget, seed: readonly number[], options: Required<IKOptions>): Result<IKSolution> | undefined {
  const joints = robot.joints;
  if (joints.length > 3 || joints.some(joint => joint.type !== "revolute" || joint.alpha !== 0 || joint.a <= 0)) return;
  if (joints.length === 3 && !target.orientation) return;
  const [x, y, z] = target.position;
  const yaw = target.orientation ? rpyFromRotation(target.orientation)[2] : undefined;
  const height = joints.reduce((height, joint) => height + joint.d, 0);
  if (Math.abs(z - height) > options.positionTolerance) return failure("The target is outside the plane of this robot.");

  const first = joints[0]!;
  const second = joints[1];
  const third = joints[2];
  const candidates: number[][] = [];
  const warnings: string[] = [];
  if (!second) {
    candidates.push([Math.atan2(y, x)]);
  } else {
    const wristX = third ? x - third.a * Math.cos(yaw!) : x;
    const wristY = third ? y - third.a * Math.sin(yaw!) : y;
    const radius = Math.hypot(wristX, wristY);
    const inner = Math.abs(first.a - second.a);
    const outer = first.a + second.a;
    if (radius < inner - options.positionTolerance || radius > outer + options.positionTolerance) {
      return failure("The target is outside the robot's planar reach.");
    }
    const scale = Math.max(first.a, second.a);
    const a = first.a / scale;
    const b = second.a / scale;
    const normalizedRadius = radius / scale;
    const cosine = clamp((normalizedRadius ** 2 - a ** 2 - b ** 2) / (2 * a * b), -1, 1);
    if (!Number.isFinite(cosine)) return;
    const elbow = Math.acos(cosine);
    if (Math.abs(Math.sin(elbow)) < 1e-7) warnings.push("The planar arm is at or near a singular configuration.");
    for (const angle2 of [elbow, -elbow]) {
      let angle1 = Math.atan2(wristY, wristX) - Math.atan2(b * Math.sin(angle2), a + b * Math.cos(angle2));
      if (radius <= 1e-12 * scale && inner <= 1e-12 * scale) {
        angle1 = yaw !== undefined && !third ? yaw - angle2 : first.theta + seed[0]!;
      }
      candidates.push(third ? [angle1, angle2, yaw! - angle1 - angle2] : [angle1, angle2]);
    }
  }

  const solutions: { q: number[]; error: number; distance: number }[] = [];
  for (const candidate of candidates) {
    const q = candidate.map((angle, i) => equivalentWithinLimits(angle, joints[i]!, seed[i]!));
    if (q.some(value => value === undefined)) continue;
    const positions = q as number[];
    const error = poseError(target, computeFrames(robot, positions));
    if (!withinTolerance(error, options) || solutions.some(solution =>
      Math.hypot(...positions.map((value, i) => value - solution.q[i]!)) < 1e-8)) continue;
    const distance = Math.hypot(...positions.map((value, i) => value - seed[i]!));
    solutions.push({ q: positions, error: error.norm, distance });
  }
  if (!solutions.length) {
    // A folded equal-length arm has a continuous family of solutions that a seed can miss.
    if (third) return;
    return failure("No planar solution satisfies the target pose and joint limits.", warnings);
  }
  solutions.sort((a, b) => a.distance - b.distance);
  return success({ solutions: solutions.map(solution => solution.q), iterations: 0, residual: solutions[0]!.error }, warnings);
}

function dampedStep(rows: number[][], error: number[], damping: number): number[] | undefined {
  const system = rows.map((row, i) => rows.map((column, j) => row.reduce((sum, value, k) =>
    sum + value * column[k]!, i === j ? damping ** 2 : 0)));
  const dual = solveLinearSystem(system, error);
  return dual && rows[0]!.map((_, col) => rows.reduce((sum, row, i) => sum + row[col]! * dual[i]!, 0));
}

function numericIK(robot: Robot, target: IKTarget, seed: readonly number[], options: Required<IKOptions>): Result<IKSolution> {
  const joints = robot.joints;
  const reach = joints.reduce((sum, joint) => {
    const offset = joint.type === "revolute" ? Math.abs(joint.d)
      : Math.max(Math.abs(joint.d + joint.min), Math.abs(joint.d + joint.max));
    return sum + Math.hypot(joint.a, offset);
  }, 0);
  if (Math.hypot(...target.position) > reach + options.positionTolerance) return failure("The target is beyond the robot's maximum reach.");

  let q = seed.map((value, i) => clamp(value, joints[i]!.min, joints[i]!.max));
  let frames = computeFrames(robot, q);
  let error = poseError(target, frames);
  let best = error.norm;
  let restarted = false;
  let iterations = 0;
  for (; iterations <= options.maxIterations; iterations++) {
    if (withinTolerance(error, options)) return success({ solutions: [q], iterations, residual: error.norm });
    if (!Number.isFinite(error.norm)) return failure("Inverse kinematics overflowed. Check the magnitude of the DH parameters.");
    if (iterations === options.maxIterations) break;

    const rows = jacobianFromFrames(robot, frames).slice(0, error.vector.length);
    let step = dampedStep(rows, error.vector, options.damping);
    if (!step) return failure("The damped Jacobian system could not be solved. Increase damping or check the DH parameters.");
    const blocked = joints.map((joint, i) => (q[i]! <= joint.min && step![i]! < 0) || (q[i]! >= joint.max && step![i]! > 0));
    if (blocked.some(Boolean)) {
      step = dampedStep(rows.map(row => row.map((value, i) => blocked[i] ? 0 : value)), error.vector, options.damping);
      if (!step) return failure("The damped Jacobian system could not be solved at the joint limits.");
    }

    const boundedStep = step.map((value, i) => {
      const joint = joints[i]!;
      const limit = joint.type === "revolute" ? 0.4 : Math.max(0.4, (joint.max - joint.min) / 6);
      return clamp(value, -limit, limit);
    });
    let improved = false;
    for (let scale = 1; scale >= 1 / 64; scale /= 2) {
      const next = q.map((value, i) => clamp(value + boundedStep[i]! * scale, joints[i]!.min, joints[i]!.max));
      const nextFrames = computeFrames(robot, next);
      const nextError = poseError(target, nextFrames);
      if (nextError.norm < error.norm || withinTolerance(nextError, options)) {
        q = next;
        frames = nextFrames;
        error = nextError;
        best = Math.min(best, error.norm);
        improved = true;
        break;
      }
    }
    if (improved) continue;
    if (restarted) return failure(`IK stalled at a singular configuration or joint limit. Best residual ${best.toExponential(3)}. The pose may be unreachable.`);

    // A straight chain can have zero first-order motion toward a reachable target.
    restarted = true;
    q = q.map((value, i) => {
      const joint = joints[i]!;
      const amount = joint.type === "revolute" ? 0.24 : 0.15 * (joint.max - joint.min);
      const signed = i % 2 === 0 ? amount : -amount;
      const next = clamp(value + signed, joint.min, joint.max);
      return next === value ? clamp(value - signed, joint.min, joint.max) : next;
    });
    frames = computeFrames(robot, q);
    error = poseError(target, frames);
    best = Math.min(best, error.norm);
  }
  return failure(`IK reached its ${iterations}-iteration limit. Best residual ${best.toExponential(3)}. The pose may be unreachable.`);
}

export function solveIK(robot: Robot, target: IKTarget, seed: readonly number[], options: IKOptions = {}): Result<IKSolution> {
  const validated = validateRobot(robot);
  if (!validated.success) return validated;
  if (!finiteVector(seed, robot.joints.length)) return failure("Provide one finite starting value per joint.");
  if (!target || !finiteVector(target.position, 3)) return failure("Provide a finite target X, Y and Z.");
  if (target.orientation !== undefined && !isRotation(target.orientation)) return failure("Target orientation must be a proper rotation matrix.");
  const settings = {
    maxIterations: options.maxIterations ?? 100,
    positionTolerance: options.positionTolerance ?? 1e-5,
    orientationTolerance: options.orientationTolerance ?? 1e-4,
    damping: options.damping ?? 1e-3,
  };
  if (!Number.isInteger(settings.maxIterations) || settings.maxIterations < 1 || settings.maxIterations > 1000 ||
      ![settings.positionTolerance, settings.orientationTolerance, settings.damping].every(value => Number.isFinite(value) && value > 0)) {
    return failure("IK needs positive finite tolerances and damping, and a 1–1000 iteration limit.");
  }
  return analyticPlanar(robot, target, seed, settings) ?? numericIK(robot, target, seed, settings);
}
