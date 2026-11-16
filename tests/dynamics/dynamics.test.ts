import { describe, expect, test } from 'bun:test';
import { createJoint, builtInRobots } from '../../src/robotics/models';
import { forwardDynamics, inverseDynamics, simulateDynamics } from '../../src/robotics/dynamics/dynamics';

const slide = { name: 'Vertical slide', joints: [createJoint({ type: 'prismatic', mass: 2, min: -100, max: 100 })] };
describe('physical dynamics references', () => {
  test('vertical slide effort equals mass times gravity plus acceleration', () => {
    const result = inverseDynamics(slide, { q: [0], velocity: [3], acceleration: [2] });
    expect(result.success).toBe(true);
    if (result.success) expect(result.data[0]).toBeCloseTo(2 * (9.80665 + 2), 10);
  });
  test('single pendulum gravity torque matches the hand calculation', () => {
    const link = createJoint({ a: 1, mass: 2, centerOfMass: [-0.5, 0, 0] });
    const result = inverseDynamics({ name: 'Pendulum', joints: [link] }, { q: [0], velocity: [0], acceleration: [0], gravity: [0, -9.80665, 0] });
    expect(result.success).toBe(true);
    if (result.success) expect(result.data[0]).toBeCloseTo(9.80665, 9);
  });
  test('single link acceleration includes parallel-axis inertia', () => {
    const link = createJoint({ a: 1, mass: 2, centerOfMass: [-0.5, 0, 0] });
    const result = inverseDynamics({ name: 'Link', joints: [link] }, { q: [0.2], velocity: [0], acceleration: [3], gravity: [0, 0, 0] });
    expect(result.success).toBe(true);
    if (result.success) expect(result.data[0]).toBeCloseTo((link.inertia[8] + 2 * 0.5 ** 2) * 3, 10);
  });
  for (const robot of builtInRobots) test(`forward/inverse round trip: ${robot.name}`, () => {
    const q = robot.joints.map(j => j.initial), velocity = q.map((_, i) => .07 * (i + 1));
    const acceleration = q.map((_, i) => -.2 + .03 * i);
    const inverse = inverseDynamics(robot, { q, velocity, acceleration });
    expect(inverse.success).toBe(true);
    if (!inverse.success) return;
    const forward = forwardDynamics(robot, { q, velocity, effort: inverse.data });
    expect(forward.success).toBe(true);
    if (forward.success) forward.data.forEach((v, i) => expect(v).toBeCloseTo(acceleration[i]!, 7));
  });
  test('RK4 free fall agrees with analytic position and velocity', () => {
    const result = simulateDynamics(slide, { q: [0], velocity: [0], effort: [0], duration: 1, timeStep: .01 });
    expect(result.success).toBe(true);
    if (result.success) { expect(result.data.at(-1)!.position[0]).toBeCloseTo(-9.80665 / 2, 8); expect(result.data.at(-1)!.velocity[0]).toBeCloseTo(-9.80665, 8); }
  });
  test('rejects nonfinite state, oversized runs and joint-limit crossings', () => {
    expect(forwardDynamics(slide, { q: [NaN], velocity: [0], effort: [0] }).success).toBe(false);
    expect(simulateDynamics(slide, { q: [0], velocity: [0], effort: [0], duration: 100 }).success).toBe(false);
    expect(simulateDynamics({ ...slide, joints: [createJoint({ type: 'prismatic' })] }, { q: [0], velocity: [0], effort: [0], duration: 1 }).success).toBe(false);
  });
});
