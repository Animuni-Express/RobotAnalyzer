import { determinant3, failure, finiteVector, success } from "../math";
import type { Mat3, Result, Vec3 } from "../math";

export type { Mat3, Vec3 } from "../math";
export type JointType = "revolute" | "prismatic";

export interface Joint {
  type: JointType;
  a: number;
  alpha: number;
  d: number;
  theta: number;
  min: number;
  max: number;
  initial: number;
  final: number;
  mass: number;
  centerOfMass: Vec3;
  inertia: Mat3;
}

export interface Robot {
  name: string;
  joints: Joint[];
}

export const maxJoints = 64;

export function createJoint(overrides: Partial<Joint> = {}): Joint {
  const prismatic = overrides.type === "prismatic";
  const a = overrides.a ?? (prismatic ? 0 : 0.45);
  const mass = overrides.mass ?? 1;
  const axial = mass * 0.05 ** 2 / 2;
  const radial = mass * (Math.max(Math.abs(a), 0.1) ** 2 / 12 + 0.05 ** 2 / 4);
  return {
    type: "revolute", a, alpha: 0, d: 0, theta: 0,
    min: prismatic ? 0 : -Math.PI, max: prismatic ? 0.65 : Math.PI,
    initial: 0, final: prismatic ? 0.35 : Math.PI / 3,
    mass, centerOfMass: [-a / 2, 0, 0],
    inertia: [axial, 0, 0, 0, radial, 0, 0, 0, radial],
    ...overrides,
  };
}

function validInertia(value: unknown): value is Mat3 {
  if (!finiteVector(value, 9)) return false;
  const scale = Math.max(...value.map(Math.abs));
  if (scale === 0) return false;
  const normalized = value.map(entry => entry / scale) as unknown as Mat3;
  const [xx, xy, xz, yx, yy, yz, zx, zy, zz] = normalized;
  const tolerance = 1e-10;
  if (Math.abs(xy - yx) > tolerance || Math.abs(xz - zx) > tolerance || Math.abs(yz - zy) > tolerance) return false;
  if (xx <= 0 || xx * yy - xy * xy <= 0 || determinant3(normalized) <= 0) return false;

  // The second-moment tensor must be positive semidefinite for a physical inertia.
  const traceHalf = (xx + yy + zz) / 2;
  const x = traceHalf - xx;
  const y = traceHalf - yy;
  const z = traceHalf - zz;
  return Math.min(x, y, z, x * y - xy * xy, x * z - xz * xz, y * z - yz * yz,
    determinant3([x, -xy, -xz, -xy, y, -yz, -xz, -yz, z])) >= -tolerance;
}

export function validateRobot(value: unknown): Result<Robot> {
  if (!value || typeof value !== "object" || Array.isArray(value)) return failure("Expected a robot object.");
  const { name, joints } = value as Record<string, unknown>;
  if (typeof name !== "string" || !name.trim() || name.length > 120) return failure("Robot name must contain 1–120 characters.");
  if (!Array.isArray(joints) || joints.length < 1 || joints.length > maxJoints) return failure(`Provide 1–${maxJoints} joints.`);

  for (let index = 0; index < joints.length; index++) {
    const value = joints[index];
    const label = `Joint ${index + 1}`;
    if (!value || typeof value !== "object" || Array.isArray(value)) return failure(`${label} is missing.`);
    const joint = value as Joint;
    const { type, a, alpha, d, theta, min, max, initial, final, mass, centerOfMass, inertia } = joint;
    if (type !== "revolute" && type !== "prismatic") return failure(`${label} must be revolute or prismatic.`);
    if (![a, alpha, d, theta, min, max, initial, final, mass].every(Number.isFinite)) return failure(`${label} contains a nonfinite number.`);
    if (min > max) return failure(`${label} has inverted limits.`);
    if (initial < min || initial > max || final < min || final > max) return failure(`${label} initial and final values must be within its limits.`);
    if (mass <= 0 || !finiteVector(centerOfMass, 3)) return failure(`${label} needs positive mass and a finite center of mass.`);
    if (!validInertia(inertia)) return failure(`${label} inertia must be symmetric, positive definite, and physically realizable.`);
  }
  return success({ name, joints: joints as Joint[] });
}

function robot(name: string, joints: Partial<Joint>[]): Robot {
  return { name, joints: joints.map(createJoint) };
}

export const builtInRobots: Robot[] = [
  robot("1R · Single link", [{ a: 0.7 }]),
  robot("2R · Planar elbow", [{ a: 0.55, final: Math.PI / 4 }, { a: 0.4, initial: Math.PI / 6, final: -Math.PI / 2 }]),
  robot("3R · Planar arm", [{ a: 0.45 }, { a: 0.35, initial: Math.PI / 6 }, { a: 0.25, initial: -Math.PI / 5 }]),
  robot("R–P · Rotary slide", [{ a: 0.25, alpha: -Math.PI / 2, d: 0.25 }, { type: "prismatic", initial: 0.2 }]),
  robot("P–R · Lift and turn", [{ type: "prismatic", initial: 0.25 }, { a: 0.48 }]),
  robot("3R · Spatial arm", [{ a: 0.1, d: 0.38, alpha: Math.PI / 2 }, { a: 0.55, initial: 0.6 }, { a: 0.38, initial: -0.75 }]),
  robot("6R · Atlas", [
    { a: 0.14, d: 0.45, alpha: Math.PI / 2, mass: 5 }, { a: 0.52, initial: 0.9, mass: 4 },
    { a: 0.13, initial: -1.25, alpha: Math.PI / 2, mass: 2.7 }, { d: 0.42, a: 0, alpha: -Math.PI / 2, mass: 1.8 },
    { a: 0, alpha: Math.PI / 2, initial: 0.55, mass: 0.8 }, { a: 0, d: 0.15, mass: 0.45 },
  ]),
  robot("6R · Meridian", [
    { d: 0.36, a: 0.09, alpha: Math.PI / 2, mass: 4.2 }, { a: 0.34, initial: 0.85, mass: 2.8 },
    { a: 0.12, initial: -1.35, alpha: Math.PI / 2, mass: 1.9 }, { a: 0, d: 0.31, alpha: -Math.PI / 2, mass: 1.3 },
    { a: 0, alpha: Math.PI / 2, initial: -0.5, mass: 0.6 }, { a: 0, d: 0.11, mass: 0.3 },
  ]),
  robot("6R · Vector", [
    { a: 0, d: 0.26, alpha: Math.PI / 2, mass: 2.5 }, { a: 0.33, initial: 0.8, mass: 2 },
    { a: 0.28, initial: -1.3, mass: 1.4 }, { a: 0, d: 0.14, alpha: Math.PI / 2, mass: 0.8 },
    { a: 0, d: 0.11, alpha: -Math.PI / 2, initial: 0.65, mass: 0.6 }, { a: 0, d: 0.1, mass: 0.35 },
  ]),
];
