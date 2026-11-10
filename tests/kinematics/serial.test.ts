import { describe, expect, test } from "bun:test";
import { computeFrames, dhSteps, dhTransform, forwardKinematics, geometricJacobian, solveIK, stagedDhTransform } from "../../src/robotics/kinematics";
import type { Result, Vec3 } from "../../src/robotics/math";
import { builtInRobots, createJoint, validateRobot } from "../../src/robotics/models";
import type { Robot } from "../../src/robotics/models";
import { identity, multiplyRotation, orientationOf, rotationLog, rotationFromRPY, transposeRotation } from "../../src/robotics/transforms";

function unwrap<T>(result: Result<T>): T {
  if (!result.success) throw new Error(result.errors.join(" "));
  return result.data;
}

function expectVector(actual: readonly number[], expected: readonly number[], precision = 9) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, i) => expect(value).toBeCloseTo(expected[i]!, precision));
}

function planar(...lengths: number[]): Robot {
  return { name: "Planar", joints: lengths.map(a => createJoint({ a })) };
}

describe("serial manipulator models and DH", () => {
  test("validates every built-in robot and rejects malformed physical properties", () => {
    for (const robot of builtInRobots) expect(validateRobot(robot).success).toBe(true);
    expect(validateRobot({ name: "Missing", joints: new Array(1) }).success).toBe(false);
    expect(validateRobot({ name: "Bad inertia", joints: [createJoint({ inertia: [1, 0, 0, 0, 1, 0, 0, 0, 3] })] }).success).toBe(false);
    expect(validateRobot({ name: "Bad limits", joints: [createJoint({ min: 2, max: -2 })] }).success).toBe(false);
  });

  test("applies standard DH in four cumulative steps", () => {
    const joint = createJoint({ a: 2, alpha: Math.PI / 2, d: 3, theta: Math.PI / 6 });
    const q = Math.PI / 3;
    const expected = [0, 0, 1, 0, 1, 0, 0, 2, 0, 1, 0, 3, 0, 0, 0, 1];
    expectVector(dhTransform(joint, q), expected);
    const steps = dhSteps(joint, q);
    expectVector(stagedDhTransform(Math.PI / 2, 3, 2, Math.PI / 2, 0), identity);
    expectVector(steps.map(frame => frame[7]), [0, 0, 2, 2]);
    expectVector(steps.map(frame => frame[11]), [0, 3, 3, 3]);
    expectVector(steps[3]!, expected);
  });

  test("matches hand-derived 1R, 2R, 3R spatial and single-slide positions", () => {
    const one = unwrap(forwardKinematics(planar(2), [Math.PI / 2]));
    expectVector(one.positions[1]!, [0, 2, 0]);
    const two = unwrap(forwardKinematics(planar(2, 1), [Math.PI / 2, -Math.PI / 2]));
    expectVector(two.positions[2]!, [1, 2, 0]);
    expectVector(orientationOf(two.endEffector), rotationFromRPY(0, 0, 0));
    const spatial = builtInRobots.find(robot => robot.name.includes("Spatial"))!;
    expectVector(unwrap(forwardKinematics(spatial, [0, Math.PI / 2, -Math.PI / 2])).positions[3]!, [0.48, 0, 0.93]);
    const slide: Robot = { name: "Slide", joints: [createJoint({ type: "prismatic", d: 0.1, theta: Math.PI / 2 })] };
    const result = unwrap(forwardKinematics(slide, [0.4]));
    expectVector(result.positions[1]!, [0, 0, 0.5]);
    expectVector(orientationOf(result.endEffector), rotationFromRPY(0, 0, Math.PI / 2));
    expect(forwardKinematics(slide, [1]).success).toBe(false);
    expect(forwardKinematics(slide, [NaN]).success).toBe(false);
  });

  test("Jacobian agrees with central position and orientation differences for mixed joints", () => {
    const robot: Robot = { name: "Mixed", joints: [createJoint({ a: 0.32, alpha: -0.6 }),
      createJoint({ type: "prismatic", a: 0.2, theta: 0.2, alpha: 0.8 }), createJoint({ a: 0.25 })] };
    const q = [0.24, 0.21, -0.3];
    const jacobian = unwrap(geometricJacobian(robot, q));
    const h = 1e-5;
    q.forEach((_, joint) => {
      const plus = computeFrames(robot, q.map((value, i) => value + (i === joint ? h : 0)));
      const minus = computeFrames(robot, q.map((value, i) => value - (i === joint ? h : 0)));
      const linear = plus.positions[3]!.map((value, i) => (value - minus.positions[3]![i]!) / (2 * h));
      const angular = rotationLog(multiplyRotation(orientationOf(plus.endEffector), transposeRotation(orientationOf(minus.endEffector))));
      expectVector(jacobian.map(row => row[joint]!), [...linear, ...angular.map(value => value / (2 * h))], 7);
    });
  });
});

describe("inverse kinematics", () => {
  test("returns both planar elbows, ordered by distance to the seed", () => {
    const robot = planar(1, 1);
    const target: Vec3 = [1, 1, 0];
    const { solutions, iterations } = unwrap(solveIK(robot, { position: target }, [Math.PI / 2, -Math.PI / 2]));
    expect(iterations).toBe(0);
    expect(solutions).toHaveLength(2);
    expectVector(solutions[0]!, [Math.PI / 2, -Math.PI / 2]);
    expectVector(solutions[1]!, [0, Math.PI / 2]);
    for (const solution of solutions) expectVector(unwrap(forwardKinematics(robot, solution)).positions[2]!, target);
  });

  test("limits solutions, flags straight-arm singularities and rejects impossible targets", () => {
    const robot = planar(1, 1);
    robot.joints[1] = createJoint({ a: 1, min: 0, max: Math.PI });
    expect(unwrap(solveIK(robot, { position: [1, 1, 0] }, [0, 0])).solutions).toHaveLength(1);
    expect(solveIK(robot, { position: [2, 0, 0] }, [0, 0]).warnings[0]).toMatch(/singular/i);
    expect(solveIK(robot, { position: [3, 0, 0] }, [0, 0]).errors[0]).toMatch(/reach/i);
    expect(solveIK(robot, { position: [1, 0, 0.1] }, [0, 0]).errors[0]).toMatch(/plane/i);
    expect(solveIK(robot, { position: [1, 0, 0], orientation: [-1, 0, 0, 0, 1, 0, 0, 0, 1] }, [0, 0]).errors[0]).toMatch(/rotation/i);
  });

  test("reconstructs a 3R planar pose including its yaw", () => {
    const robot = planar(0.6, 0.4, 0.2);
    const q = [0.3, -0.7, 0.9];
    const { positions, endEffector } = computeFrames(robot, q);
    const target = { position: positions[3]!, orientation: orientationOf(endEffector) };
    const { solutions } = unwrap(solveIK(robot, target, q));
    expect(solutions).toHaveLength(2);
    for (const solution of solutions) expectVector(computeFrames(robot, solution).endEffector, endEffector);
  });

  test("solves a mixed prismatic and revolute chain numerically", () => {
    const robot: Robot = { name: "Lift and turn", joints: [createJoint({ type: "prismatic" }), createJoint({ a: 0.5 })] };
    const target: Vec3 = [0, 0.5, 0.3];
    const solution = unwrap(solveIK(robot, { position: target }, [0, 0]));
    expect(solution.iterations).toBeGreaterThan(0);
    expectVector(solution.solutions[0]!, [0.3, Math.PI / 2], 5);
  });

  test("solves a six-axis pose numerically and verifies it through FK", () => {
    const robot = builtInRobots.find(robot => robot.name.includes("Atlas"))!;
    const q = [0.24, 0.78, -1.18, 0.15, 0.68, -0.26];
    const { positions, endEffector } = computeFrames(robot, q);
    const target = { position: positions[6]!, orientation: orientationOf(endEffector) };
    const solution = unwrap(solveIK(robot, target, robot.joints.map(joint => joint.initial)));
    const result = unwrap(forwardKinematics(robot, solution.solutions[0]!));
    expectVector(result.positions[6]!, target.position, 4);
    expectVector(orientationOf(result.endEffector), target.orientation, 3);
  });
});
