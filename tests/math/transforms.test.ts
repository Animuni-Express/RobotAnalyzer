import { describe, expect, test } from "bun:test";
import { solveLinearSystem } from "../../src/robotics/math";
import type { Mat4, Vec3 } from "../../src/robotics/math";
import { convertFrame, identity, inverseTransform, isRigidTransform, isRotation, multiply, rotationExp, rotationLog, rotationX, rotationY, rotationZ, transformPoint, translation } from "../../src/robotics/transforms";

function expectVector(actual: readonly number[], expected: readonly number[], precision = 10) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, index) => expect(value).toBeCloseTo(expected[index]!, precision));
}

describe("homogeneous transforms", () => {
  test("multiplies general 4 × 4 matrices", () => {
    const a: Mat4 = [1, 2, 3, 4, 0, -1, 0, 2, 3, 0, 1, 0, 0, 0, 0, 1];
    const b: Mat4 = [2, 0, 1, 3, 1, 0, -1, 2, 0, 4, 3, 0, 2, 3, 0, 1];
    expect(multiply(a, b)).toEqual([12, 24, 8, 11, 3, 6, 1, 0, 6, 4, 6, 9, 2, 3, 0, 1]);
  });

  test("composes rotations and translation with column-vector convention", () => {
    expectVector(transformPoint(rotationX(Math.PI / 2), [0, 1, 0]), [0, 0, 1]);
    expectVector(transformPoint(rotationY(Math.PI / 2), [0, 0, 1]), [1, 0, 0]);
    const worldFromLocal = multiply(translation(3, 4, 5), rotationZ(Math.PI / 2));
    expectVector(transformPoint(worldFromLocal, [1, 0, 0]), [3, 5, 5]);
    expectVector(multiply(inverseTransform(worldFromLocal), worldFromLocal), identity);
    expectVector(convertFrame([1, 0, 0], worldFromLocal, translation(0, 0, 2)), [3, 5, 3]);
    expect(isRigidTransform(worldFromLocal)).toBe(true);
  });

  test("detects malformed matrices and reflections", () => {
    expect(isRigidTransform([1, 2])).toBe(false);
    expect(isRigidTransform([...identity.slice(0, 15), NaN])).toBe(false);
    expect(isRotation([-1, 0, 0, 0, 1, 0, 0, 0, 1])).toBe(false);
  });

  test("retains the rotation at zero and near a half turn", () => {
    for (const angle of [0, 1e-10, 0.65, Math.PI - 1e-10, Math.PI]) {
      const axis: Vec3 = [-2 / 3, 2 / 3, -1 / 3];
      const rotation = rotationExp([angle * axis[0], angle * axis[1], angle * axis[2]]);
      expectVector(rotationExp(rotationLog(rotation)), rotation);
      expect(Math.hypot(...rotationLog(rotation))).toBeCloseTo(angle, 10);
    }
  });

  test("pivots a linear system and rejects a singular one", () => {
    expectVector(solveLinearSystem([[0, 2], [1, 3]], [4, 7])!, [1, 2]);
    expect(solveLinearSystem([[1, 2], [2, 4]], [1, 2])).toBeUndefined();
  });
});
