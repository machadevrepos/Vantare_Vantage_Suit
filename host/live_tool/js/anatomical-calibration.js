/** Pure right-arm sensor-neutral -> anatomical TRIAD calibration math. */

const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function finiteVector(value) {
  return Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
}

function norm(value) {
  return Math.hypot(value[0], value[1], value[2]);
}

function normalize(value) {
  const length = norm(value);
  return length > 1e-9 ? value.map((component) => component / length) : null;
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function orthogonalize(primary, secondary) {
  return normalize(secondary.map((value, index) => value - dot(secondary, primary) * primary[index]));
}

function columns(a, b, c) {
  return [
    [a[0], b[0], c[0]],
    [a[1], b[1], c[1]],
    [a[2], b[2], c[2]],
  ];
}

function transpose(matrix) {
  return matrix[0].map((_, column) => matrix.map((row) => row[column]));
}

function multiplyMatrices(a, b) {
  const bt = transpose(b);
  return a.map((row) => bt.map((column) => dot(row, column)));
}

function determinant(matrix) {
  const [a, b, c] = matrix;
  return a[0] * (b[1] * c[2] - b[2] * c[1])
    - a[1] * (b[0] * c[2] - b[2] * c[0])
    + a[2] * (b[0] * c[1] - b[1] * c[0]);
}

function orthogonalityError(matrix) {
  const product = multiplyMatrices(transpose(matrix), matrix);
  let worst = 0;
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      worst = Math.max(worst, Math.abs(product[row][column] - (row === column ? 1 : 0)));
    }
  }
  return worst;
}

function matrixToQuaternion(matrix) {
  const m = matrix;
  const trace = m[0][0] + m[1][1] + m[2][2];
  let w;
  let x;
  let y;
  let z;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2;
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2;
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2;
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }
  const length = Math.hypot(w, x, y, z);
  if (!(length > 1e-9)) return null;
  const sign = w < 0 ? -1 : 1;
  return [sign * w / length, sign * x / length, sign * y / length, sign * z / length];
}

function failure(reason) {
  return { ok: false, mount: null, quality: null, reason };
}

export function solveMountCorrection(sourceSide, sourceForward, {
  targetSide = [0, 0, -1],
  targetForward = [1, 0, 0],
  minSeparationDeg = 60,
  maxSeparationDeg = 120,
} = {}) {
  if (![sourceSide, sourceForward, targetSide, targetForward].every(finiteVector)) {
    return failure("non_finite_axis");
  }
  const s1 = normalize(sourceSide);
  const sourceForwardUnit = normalize(sourceForward);
  const t1 = normalize(targetSide);
  const targetForwardUnit = normalize(targetForward);
  if (!s1 || !sourceForwardUnit || !t1 || !targetForwardUnit) return failure("zero_axis");

  const separationDeg = Math.acos(clamp(dot(s1, sourceForwardUnit), -1, 1)) * 180 / Math.PI;
  if (separationDeg < minSeparationDeg || separationDeg > maxSeparationDeg) {
    return failure("axes_not_independent");
  }
  const s2 = orthogonalize(s1, sourceForwardUnit);
  const t2 = orthogonalize(t1, targetForwardUnit);
  if (!s2 || !t2) return failure("axes_not_independent");
  const s3 = normalize(cross(s1, s2));
  const t3 = normalize(cross(t1, t2));
  if (!s3 || !t3) return failure("axes_not_independent");

  const sourceBasis = columns(s1, s2, s3);
  const targetBasis = columns(t1, t2, t3);
  const mapMatrix = multiplyMatrices(targetBasis, transpose(sourceBasis));
  const det = determinant(mapMatrix);
  const error = orthogonalityError(mapMatrix);
  const mapQuaternion = matrixToQuaternion(mapMatrix);
  if (!mapQuaternion || !Number.isFinite(det) || Math.abs(det - 1) > 1e-6 || error > 1e-6) {
    return failure("invalid_rotation");
  }

  return {
    ok: true,
    mount: [mapQuaternion[0], -mapQuaternion[1], -mapQuaternion[2], -mapQuaternion[3]],
    quality: {
      axisSeparationDeg: separationDeg,
      determinant: det,
      orthogonalityError: error,
    },
    reason: null,
  };
}
