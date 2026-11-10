import { clamp, cross, failure, finiteVector, subtract, success } from "../math";
import type { Mat4, Result, Vec3 } from "../math";
import { validateRobot } from "../models";
import type { Joint, Robot } from "../models";
import { identity, multiply, positionOf } from "../transforms";

export { solveIK } from "./inverse";
export type { IKOptions, IKSolution, IKTarget } from "./inverse";

function dhMatrix(angle: number, offset: number, length: number, twist: number): Mat4 {
  const ct = Math.cos(angle);
  const st = Math.sin(angle);
  const ca = Math.cos(twist);
  const sa = Math.sin(twist);
  return [ct, -st * ca, st * sa, length * ct, st, ct * ca, -ct * sa, length * st,
    0, sa, ca, offset, 0, 0, 0, 1];
}

export function dhTransform(joint: Joint, q: number): Mat4 {
  return dhMatrix(joint.theta + (joint.type === "revolute" ? q : 0),
    joint.d + (joint.type === "prismatic" ? q : 0), joint.a, joint.alpha);
}

export function stagedDhTransform(angle: number, offset: number, length: number, twist: number, progress: number): Mat4 {
  const step = progress * 4;
  return dhMatrix(angle * clamp(step, 0, 1), offset * clamp(step - 1, 0, 1),
    length * clamp(step - 2, 0, 1), twist * clamp(step - 3, 0, 1));
}

export function dhSteps(joint: Joint, q: number): Mat4[] {
  const angle = joint.theta + (joint.type === "revolute" ? q : 0);
  const offset = joint.d + (joint.type === "prismatic" ? q : 0);
  return [0.25, 0.5, 0.75, 1].map(progress => stagedDhTransform(angle, offset, joint.a, joint.alpha, progress));
}

export interface ForwardFrames {
  relative: Mat4[];
  frames: Mat4[];
  positions: Vec3[];
  endEffector: Mat4;
}

export function computeFrames(robot: Robot, q: readonly number[]): ForwardFrames {
  const frames = [identity];
  const positions: Vec3[] = [[0, 0, 0]];
  const relative: Mat4[] = [];
  let endEffector = identity;
  for (let i = 0; i < robot.joints.length; i++) {
    const local = dhTransform(robot.joints[i]!, q[i]!);
    endEffector = multiply(endEffector, local);
    relative.push(local);
    frames.push(endEffector);
    positions.push(positionOf(endEffector));
  }
  return { relative, frames, positions, endEffector };
}

export function forwardKinematics(robot: Robot, q: readonly number[]): Result<ForwardFrames> {
  const validated = validateRobot(robot);
  if (!validated.success) return validated;
  if (!finiteVector(q, robot.joints.length)) return failure("Provide one finite position per joint.");
  const outOfBounds = robot.joints.findIndex((joint, i) => q[i]! < joint.min || q[i]! > joint.max);
  if (outOfBounds >= 0) return failure(`Joint ${outOfBounds + 1} is outside its limits.`);
  const data = computeFrames(robot, q);
  return data.frames.every(frame => frame.every(Number.isFinite))
    ? success(data) : failure("Forward kinematics overflowed. Check the magnitude of the DH parameters.");
}

export function jacobianFromFrames(robot: Robot, { frames, positions }: ForwardFrames): number[][] {
  const rows = Array.from({ length: 6 }, () => new Array<number>(robot.joints.length));
  const tip = positions[positions.length - 1]!;
  for (let i = 0; i < robot.joints.length; i++) {
    const frame = frames[i]!;
    const axis: Vec3 = [frame[2], frame[6], frame[10]];
    const revolute = robot.joints[i]!.type === "revolute";
    const linear = revolute ? cross(axis, subtract(tip, positions[i]!)) : axis;
    for (let row = 0; row < 3; row++) {
      rows[row]![i] = linear[row]!;
      rows[row + 3]![i] = revolute ? axis[row]! : 0;
    }
  }
  return rows;
}

export function geometricJacobian(robot: Robot, q: readonly number[]): Result<number[][]> {
  const frames = forwardKinematics(robot, q);
  if (!frames.success) return frames;
  const matrix = jacobianFromFrames(robot, frames.data);
  return matrix.every(row => row.every(Number.isFinite))
    ? success(matrix) : failure("Jacobian overflowed. Check the magnitude of the DH parameters.");
}
