import { describe, expect, test } from 'bun:test';
import { createTrajectory, sampledTrajectory } from '../../src/robotics/trajectories/trajectory';
import { createCartesianTrajectory } from '../../src/robotics/trajectories/cartesian';
import { builtInRobots } from '../../src/robotics/models';
import { computeFrames } from '../../src/robotics/kinematics';
import { positionOf } from '../../src/robotics/transforms';
describe('motion contracts', () => {
  for (const kind of ['linear', 'cubic', 'quintic', 'cycloidal'] as const) test(`${kind} endpoints, finite samples, and clamping`, () => {
    const result = createTrajectory({ kind, startTime: 2, endTime: 5, initial: [.3, -.2], final: [.7, 1] });
    expect(result.success).toBe(true); if (!result.success) return;
    result.data.sample(-10).position.forEach((v, i) => expect(v).toBeCloseTo([.3, -.2][i]!, 10));
    result.data.sample(20).position.forEach((v, i) => expect(v).toBeCloseTo([.7, 1][i]!, 10));
    expect(result.data.sample(3).velocity.every(Number.isFinite)).toBe(true);
    if (kind !== 'linear') { expect(result.data.sample(2).velocity[0]).toBeCloseTo(0, 10); expect(result.data.sample(5).velocity[0]).toBeCloseTo(0, 10); }
  });
  test('quintic satisfies nonzero velocity and acceleration boundary conditions', () => {
    const result = createTrajectory({ kind: 'quintic', startTime: 1, endTime: 4, initial: [0], final: [1], initialVelocity: [.2], finalVelocity: [-.1], initialAcceleration: [.3], finalAcceleration: [-.4] });
    expect(result.success).toBe(true); if (!result.success) return;
    expect(result.data.sample(1).velocity[0]).toBeCloseTo(.2, 9); expect(result.data.sample(4).velocity[0]).toBeCloseTo(-.1, 9);
    expect(result.data.sample(1).acceleration[0]).toBeCloseTo(.3, 9); expect(result.data.sample(4).acceleration[0]).toBeCloseTo(-.4, 9);
  });
  test('rejects invalid times and incompatible boundary conditions', () => {
    const spec = { kind: 'linear' as const, startTime: 0, endTime: 1, initial: [0], final: [1] };
    for (const startTime of [NaN, Infinity, -Infinity, 1]) expect(createTrajectory({ ...spec, startTime }).success).toBe(false);
    expect(createTrajectory({ ...spec, initialVelocity: [1] }).success).toBe(false);
  });
  test('Cartesian position motion can change orientation on a planar arm', () => {
    const robot = builtInRobots[1]!; const seed = robot.joints.map(j => j.initial);
    const start = positionOf(computeFrames(robot, seed).endEffector);
    const target = [start[0] - .03, start[1] + .03, start[2]] as const;
    const result = createCartesianTrajectory(robot, { seed, start: { position: start }, target: { position: target }, duration: 2 });
    expect(result.success).toBe(true); if (!result.success) return;
    for (const time of [0, .5, 1, 1.5, 2]) {
      const actual = positionOf(computeFrames(robot, result.data.sample(time).position).endEffector);
      actual.forEach((v, i) => expect(v).toBeCloseTo(start[i]! + (target[i]! - start[i]!) * time / 2, 3));
    }
  });
  test('sampled simulation interpolates irregular times and preserves endpoints', () => {
    const samples = [0, .1, .35].map(time => ({ time, position: [time * 2], velocity: [2], acceleration: [0] }));
    const path = sampledTrajectory(samples); expect(path.sample(.2).position[0]).toBeCloseTo(.4, 10);
    expect(path.sample(10).position[0]).toBe(.7); expect(() => path.sample(NaN)).toThrow();
    expect(() => sampledTrajectory([samples[1]!, samples[0]!])).toThrow();
  });
});

test('joint limits catch interior overshoot despite legal endpoints', () => {
  const spec = { kind: 'cubic' as const, startTime: 0, endTime: 1, initial: [0], final: [0], initialVelocity: [10], finalVelocity: [-10], limits: [{ min: -1, max: 1 }] };
  expect(createTrajectory(spec).success).toBe(false);
  expect(createTrajectory({ ...spec, initialVelocity: [0], finalVelocity: [0] }).success).toBe(true);
  expect(createTrajectory({ ...spec, kind: 'quintic' }).success).toBe(false);
});
