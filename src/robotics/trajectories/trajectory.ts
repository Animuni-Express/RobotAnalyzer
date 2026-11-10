import { failure, success, type Result } from '../math';

export type TrajectoryKind = 'linear' | 'cubic' | 'quintic' | 'cycloidal';

export interface TrajectorySpec {
  kind: TrajectoryKind;
  startTime: number;
  endTime: number;
  initial: readonly number[];
  final: readonly number[];
  initialVelocity?: readonly number[];
  finalVelocity?: readonly number[];
  initialAcceleration?: readonly number[];
  finalAcceleration?: readonly number[];
}

export interface MotionSample {
  time: number;
  position: number[];
  velocity: number[];
  acceleration: number[];
}

export interface Trajectory {
  startTime: number;
  endTime: number;
  jointCount: number;
  sample(time: number): MotionSample;
}

function polynomialAt(coefficients: readonly number[], time: number): number {
  return coefficients.reduceRight((value, coefficient) => value * time + coefficient, 0);
}

function derivative(coefficients: number[], duration: number): number[] {
  return coefficients.slice(1).map((coefficient, index) => (index + 1) * (coefficient / duration));
}

export function createTrajectory(spec: TrajectorySpec): Result<Trajectory> {
  const { kind, startTime, endTime, initial, final, initialVelocity, finalVelocity, initialAcceleration, finalAcceleration } = spec;
  const duration = endTime - startTime;
  if (!Number.isFinite(duration) || duration <= 0) {
    return failure('Final time must be finite and greater than initial time.');
  }
  if (!['linear', 'cubic', 'quintic', 'cycloidal'].includes(kind)) {
    return failure('Unknown trajectory interpolation.');
  }
  const jointCount = initial.length;
  const vectors = [initial, final, initialVelocity, finalVelocity, initialAcceleration, finalAcceleration];
  if (jointCount === 0 || vectors.some((vector) => vector !== undefined && (
    vector.length !== jointCount || Array.from(vector).some((value) => !Number.isFinite(value))
  ))) {
    return failure('Trajectory states must contain one finite value per joint.');
  }

  const cycloidal = kind === 'cycloidal';
  const hasVelocity = kind === 'cubic' || kind === 'quintic';
  const hasAcceleration = kind === 'quintic';
  if ((!hasVelocity && (initialVelocity || finalVelocity)) ||
      (!hasAcceleration && (initialAcceleration || finalAcceleration))) {
    return failure('Boundary velocities are supported by cubic and quintic interpolation. Boundary accelerations require quintic interpolation.');
  }

  const coefficients = Array.from(initial, (start, index) => {
    const displacement = final[index]! - start;
    if (!hasVelocity) return [start, displacement];
    const v0 = (initialVelocity?.[index] ?? 0) * duration;
    const v1 = (finalVelocity?.[index] ?? 0) * duration;
    if (!hasAcceleration) return [start, v0, 3 * displacement - 2 * v0 - v1, -2 * displacement + v0 + v1];
    const a0 = (initialAcceleration?.[index] ?? 0) * duration * duration;
    const a1 = (finalAcceleration?.[index] ?? 0) * duration * duration;
    const positionError = displacement - v0 - a0 / 2;
    const velocityError = v1 - v0 - a0;
    const accelerationError = a1 - a0;
    return [start, v0, a0 / 2,
      10 * positionError - 4 * velocityError + accelerationError / 2,
      -15 * positionError + 7 * velocityError - accelerationError,
      6 * positionError - 3 * velocityError + accelerationError / 2,
    ];
  });
  const velocities = coefficients.map((coefficient) => derivative(coefficient, duration));
  const accelerations = velocities.map((coefficient, index) => cycloidal
    ? [(coefficient[0]! / duration) * 2 * Math.PI]
    : derivative(velocities[index]!, duration));
  if ([...coefficients, ...velocities, ...accelerations].some((vector) =>
    !Number.isFinite(vector.reduce((sum, value) => sum + Math.abs(value), 0))) ||
    (cycloidal && velocities.some(([value]) => !Number.isFinite(2 * value!)))) {
    return failure('Trajectory derivatives exceed the supported numeric range.');
  }

  return success({
    startTime, endTime, jointCount,
    sample(requestedTime) {
      if (!Number.isFinite(requestedTime)) throw new RangeError('Sample time must be finite.');
      const time = Math.min(endTime, Math.max(startTime, requestedTime));
      const fraction = (time - startTime) / duration;
      if (cycloidal) {
        const angle = 2 * Math.PI * fraction;
        const sine = fraction === 1 ? 0 : Math.sin(angle);
        const positionFraction = fraction - sine / (2 * Math.PI);
        const velocityFraction = 1 - Math.cos(angle);
        return {
          time,
          position: coefficients.map(([start, displacement]) => start! + displacement! * positionFraction),
          velocity: velocities.map(([value]) => value! * velocityFraction),
          acceleration: accelerations.map(([value]) => value! * sine),
        };
      }
      return {
        time,
        position: coefficients.map((values) => polynomialAt(values, fraction)),
        velocity: velocities.map((values) => polynomialAt(values, fraction)),
        acceleration: accelerations.map((values) => polynomialAt(values, fraction)),
      };
    },
  });
}
