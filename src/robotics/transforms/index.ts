import { clamp, determinant3, finiteVector } from "../math";
import type { Mat3, Mat4, Vec3 } from "../math";

export const identity: Mat4 = Object.freeze([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

export function multiply(left: Mat4, right: Mat4): Mat4 {
  const result: number[] = [];
  for (let row = 0; row < 16; row += 4) {
    for (let col = 0; col < 4; col++) {
      result.push(left[row]! * right[col]! + left[row + 1]! * right[col + 4]! +
        left[row + 2]! * right[col + 8]! + left[row + 3]! * right[col + 12]!);
    }
  }
  return result as unknown as Mat4;
}

export function translation(x: number, y: number, z: number): Mat4 {
  return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

export function rotationX(angle: number): Mat4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}

export function rotationY(angle: number): Mat4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

export function rotationZ(angle: number): Mat4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

export function positionOf(matrix: Mat4): Vec3 {
  return [matrix[3], matrix[7], matrix[11]];
}

export function orientationOf(matrix: Mat4): Mat3 {
  return [matrix[0], matrix[1], matrix[2], matrix[4], matrix[5], matrix[6], matrix[8], matrix[9], matrix[10]];
}

export function inverseTransform(matrix: Mat4): Mat4 {
  const [a, b, c, x, d, e, f, y, g, h, i, z] = matrix;
  return [a, d, g, -(a * x + d * y + g * z), b, e, h, -(b * x + e * y + h * z),
    c, f, i, -(c * x + f * y + i * z), 0, 0, 0, 1];
}

export function transformPoint(matrix: Mat4, [x, y, z]: Vec3): Vec3 {
  return [
    matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
    matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
    matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
  ];
}

export function convertFrame(point: Vec3, worldFromSource: Mat4, worldFromDestination: Mat4): Vec3 {
  return transformPoint(multiply(inverseTransform(worldFromDestination), worldFromSource), point);
}

export function transposeRotation([a, b, c, d, e, f, g, h, i]: Mat3): Mat3 {
  return [a, d, g, b, e, h, c, f, i];
}

export function multiplyRotation(left: Mat3, right: Mat3): Mat3 {
  const [a, b, c, d, e, f, g, h, i] = left;
  const [j, k, l, m, n, o, p, q, r] = right;
  return [a * j + b * m + c * p, a * k + b * n + c * q, a * l + b * o + c * r,
    d * j + e * m + f * p, d * k + e * n + f * q, d * l + e * o + f * r,
    g * j + h * m + i * p, g * k + h * n + i * q, g * l + h * o + i * r];
}

export function isRotation(value: unknown): value is Mat3 {
  if (!finiteVector(value, 9)) return false;
  const matrix = value as unknown as Mat3;
  const product = multiplyRotation(matrix, transposeRotation(matrix));
  return Math.abs(determinant3(matrix) - 1) <= 1e-8 &&
    product.every((value, index) => Math.abs(value - (index % 4 === 0 ? 1 : 0)) <= 1e-8);
}

export function isRigidTransform(value: unknown): value is Mat4 {
  if (!finiteVector(value, 16)) return false;
  const matrix = value as unknown as Mat4;
  return matrix[12] === 0 && matrix[13] === 0 && matrix[14] === 0 && matrix[15] === 1 && isRotation(orientationOf(matrix));
}

// Extrinsic roll, pitch, yaw about fixed X, Y, Z, respectively.
export function rotationFromRPY(roll: number, pitch: number, yaw: number): Mat3 {
  return orientationOf(multiply(multiply(rotationZ(yaw), rotationY(pitch)), rotationX(roll)));
}

export function rpyFromRotation(matrix: Mat3): Vec3 {
  const pitch = Math.atan2(-matrix[6], Math.hypot(matrix[0], matrix[3]));
  if (Math.abs(Math.cos(pitch)) < 1e-9) return [0, pitch, Math.atan2(-matrix[1], matrix[4])];
  return [Math.atan2(matrix[7], matrix[8]), pitch, Math.atan2(matrix[3], matrix[0])];
}

export function rotationLog(matrix: Mat3): Vec3 {
  const [a, b, c, d, e, f, g, h, i] = matrix;
  const skew: Vec3 = [h - f, c - g, d - b];
  const skewNorm = Math.hypot(...skew);
  const cosine = clamp((a + e + i - 1) / 2, -1, 1);
  const angle = Math.atan2(skewNorm / 2, cosine);
  if (angle < 1e-8) return [skew[0] / 2, skew[1] / 2, skew[2] / 2];
  if (cosine > -0.9999) {
    const scale = angle / skewNorm;
    return [skew[0] * scale, skew[1] * scale, skew[2] * scale];
  }

  // Near pi the skew vanishes; recover the axis from the largest diagonal entry.
  const major = a >= e && a >= i ? 0 : e >= i ? 1 : 2;
  const component = Math.sqrt(Math.max(0, (matrix[major * 4]! - cosine) / (1 - cosine)));
  const divisor = 2 * (1 - cosine) * component;
  const axis: Vec3 = major === 0 ? [component, (b + d) / divisor, (c + g) / divisor]
    : major === 1 ? [(b + d) / divisor, component, (f + h) / divisor]
      : [(c + g) / divisor, (f + h) / divisor, component];
  const direction = axis[0] * skew[0] + axis[1] * skew[1] + axis[2] * skew[2] < 0 ? -1 : 1;
  const scale = direction * angle / Math.hypot(...axis);
  return [axis[0] * scale, axis[1] * scale, axis[2] * scale];
}

export function rotationExp([x, y, z]: Vec3): Mat3 {
  const angle = Math.hypot(x, y, z);
  const s = angle < 1e-8 ? 1 : Math.sin(angle) / angle;
  const c = angle < 1e-8 ? 0.5 : (2 * Math.sin(angle / 2) ** 2) / (angle * angle);
  return [1 - c * (y * y + z * z), c * x * y - s * z, c * x * z + s * y,
    c * y * x + s * z, 1 - c * (x * x + z * z), c * y * z - s * x,
    c * z * x - s * y, c * z * y + s * x, 1 - c * (x * x + y * y)];
}
