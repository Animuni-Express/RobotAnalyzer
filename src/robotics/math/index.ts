export type Vec3 = readonly [number, number, number];
export type Mat3 = readonly [number, number, number, number, number, number, number, number, number];
export type Mat4 = readonly [
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
];

export type Result<T> =
  | { success: true; data: T; warnings: string[]; errors: [] }
  | { success: false; data?: undefined; warnings: string[]; errors: string[] };

export function success<T>(data: T, warnings: string[] = []): Result<T> {
  return { success: true, data, warnings, errors: [] };
}

export function failure<T = never>(error: string, warnings: string[] = []): Result<T> {
  return { success: false, warnings, errors: [error] };
}

export function finiteVector(value: unknown, length: number): value is number[] {
  if (!Array.isArray(value) || value.length !== length) return false;
  for (let i = 0; i < length; i++) {
    if (!Number.isFinite(value[i])) return false;
  }
  return true;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function subtract(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function determinant3([a, b, c, d, e, f, g, h, i]: Mat3): number {
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// Partial pivoting keeps the small task-space system stable with mixed joint types.
export function solveLinearSystem(matrix: readonly (readonly number[])[], rhs: readonly number[]): number[] | undefined {
  const size = rhs.length;
  if (!finiteVector(rhs, size) || matrix.length !== size || matrix.some(row => !finiteVector(row, size))) return;
  const rows = matrix.map((row, i) => [...row, rhs[i]!]);
  const scale = matrix.reduce((max, row) => Math.max(max, ...row.map(Math.abs)), 0);
  if (scale === 0) return;

  for (let col = 0; col < size; col++) {
    let pivot = col;
    for (let row = col + 1; row < size; row++) {
      if (Math.abs(rows[row]![col]!) > Math.abs(rows[pivot]![col]!)) pivot = row;
    }
    [rows[col], rows[pivot]] = [rows[pivot]!, rows[col]!];
    const pivotRow = rows[col]!;
    const divisor = pivotRow[col]!;
    if (Math.abs(divisor) <= Number.EPSILON * size * scale) return;
    for (let row = col + 1; row < size; row++) {
      const current = rows[row]!;
      const factor = current[col]! / divisor;
      for (let k = col + 1; k <= size; k++) current[k] = current[k]! - factor * pivotRow[k]!;
    }
  }

  const solution = new Array<number>(size);
  for (let i = size - 1; i >= 0; i--) {
    const row = rows[i]!;
    let value = row[size]!;
    for (let k = i + 1; k < size; k++) value -= row[k]! * solution[k]!;
    solution[i] = value / row[i]!;
  }
  return finiteVector(solution, size) ? solution : undefined;
}
