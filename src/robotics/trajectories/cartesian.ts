import { computeFrames, forwardKinematics, solveIK } from '../kinematics';
import { failure, finiteVector, subtract, success, type Result } from '../math';
import type { Mat3, Robot, Vec3 } from '../models';
import { isRotation, multiplyRotation, orientationOf, rotationExp, rotationLog, transposeRotation } from '../transforms';
import type { Trajectory } from './trajectory';

interface CartesianPose {
  position: Vec3;
  orientation?: Mat3;
}

export interface CartesianTrajectorySpec {
  seed: readonly number[];
  start?: CartesianPose;
  target: CartesianPose;
  duration: number;
  segments?: number;
}

const positionTolerance = 0.0002;
const orientationTolerance = 0.002;

export function createCartesianTrajectory(robot: Robot, spec: CartesianTrajectorySpec): Result<Trajectory> {
  const { start, target, seed, duration, segments = 64 } = spec;
  const step = duration / segments;
  if (!Number.isFinite(duration) || duration <= 0 || step <= 0 || !Number.isInteger(segments) || segments < 2 || segments > 256) {
    return failure('Cartesian motion requires a positive duration and 2–256 path segments.');
  }
  if ([start, target].some((pose) => pose && (!finiteVector(pose.position, 3) ||
    (pose.orientation !== undefined && !isRotation(pose.orientation))))) {
    return failure('Cartesian path poses must contain finite coordinates and valid rotations.');
  }

  let q = Array.from(seed);
  if (start) {
    const solved = solveIK(robot, start, q);
    if (!solved.success) return failure(`Cartesian start pose: ${solved.errors.join(' ')}`);
    q = solved.data.solutions[0]!;
  }
  const fk = forwardKinematics(robot, q);
  if (!fk.success) return fk;
  const origin = start?.position ?? fk.data.positions.at(-1)!;
  const displacement = subtract(target.position, origin);
  if (!displacement.every(Number.isFinite)) return failure('Cartesian displacement exceeds the supported numeric range.');
  const from = start?.orientation ?? (target.orientation ? orientationOf(fk.data.endEffector) : undefined);
  const rotation = from && target.orientation
    ? rotationLog(multiplyRotation(target.orientation, transposeRotation(from)))
    : undefined;

  function poseAt(fraction: number): CartesianPose {
    return {
      position: [
        origin[0] + fraction * displacement[0],
        origin[1] + fraction * displacement[1],
        origin[2] + fraction * displacement[2],
      ],
      orientation: from && rotation
        ? multiplyRotation(rotationExp([rotation[0] * fraction, rotation[1] * fraction, rotation[2] * fraction]), from)
        : from,
    };
  }

  const knots = [q];
  const velocities: number[][] = [];
  for (let segment = 1; segment <= segments; segment++) {
    const result = solveIK(robot, poseAt(segment / segments), q);
    if (!result.success) return failure(`Cartesian segment ${segment}/${segments}: ${result.errors.join(' ')}`);
    const next = result.data.solutions[0]!;
    const midpoint = q.map((value, index) => value + (next[index]! - value) / 2);
    const actual = computeFrames(robot, midpoint);
    const desired = poseAt((segment - 0.5) / segments);
    const positionError = Math.hypot(...subtract(desired.position, actual.positions.at(-1)!));
    const orientationError = desired.orientation
      ? Math.hypot(...rotationLog(multiplyRotation(desired.orientation, transposeRotation(orientationOf(actual.endEffector)))))
      : 0;
    if (positionError > positionTolerance || orientationError > orientationTolerance) {
      return failure(`Cartesian segment ${segment}/${segments} deviates from the requested line. Increase segment resolution or choose a path farther from joint limits and singularities.`);
    }
    const velocity = next.map((value, index) => (value - q[index]!) / step);
    if (!velocity.every(Number.isFinite)) return failure('Cartesian joint velocities exceed the supported numeric range.');
    velocities.push(velocity);
    knots.push(next);
    q = next;
  }

  return success({
    startTime: 0, endTime: duration, jointCount: seed.length,
    sample(requestedTime) {
      if (!Number.isFinite(requestedTime)) throw new RangeError('Sample time must be finite.');
      const time = Math.min(duration, Math.max(0, requestedTime));
      const knot = time / duration * segments;
      const segment = Math.min(segments - 1, Math.floor(knot));
      const fraction = knot - segment;
      const position = knots[segment]!.map((value, index) => value + fraction * (knots[segment + 1]![index]! - value));
      // Joint velocity is piecewise constant and can jump at the sampled poses.
      return { time, position, velocity: velocities[segment]!.slice(), acceleration: position.map(() => 0) };
    },
  });
}
