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
  // The TRIAD construction maps the captured directions onto the anatomical
  // targets exactly, so the calibrated poses are reproduced regardless of how
  // far apart the wearer's raises were. These bounds only refuse a pair too
  // close to a single direction (or its reverse) to define a rotation at all;
  // a soft quality hint is reported separately.
  minSeparationDeg = 25,
  maxSeparationDeg = 155,
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

// ------------------------------------------------------ pointing calibration

function degreesBetween(a, b) {
  return Math.acos(clamp(dot(normalize(a), normalize(b)), -1, 1)) * 180 / Math.PI;
}

function horizontal(vector, down) {
  return normalize(vector.map((value, index) => value - dot(vector, down) * down[index]));
}

/**
 * Solve a node's mount from WHERE THE ARM POINTED, not from how it rotated.
 *
 * Why this replaced the rotation-axis TRIAD: a raise that also twists the arm
 * about its own long axis (turning the palm) tilts the delta quaternion's
 * rotation axis, and the axis solver read that tilt as mount geometry. In the
 * 2026-09-11T09:10 field session the wearer held the palm down in both
 * directional poses; relative to their neutral palm that was +51 / -43 degrees
 * of twist, so the rotation axes measured only 60 degrees apart while the arm
 * itself pointed 80 degrees apart, and the straight-ahead hold rendered 25-29
 * degrees outward. The long axis goes to the same place whatever the palm
 * does, so this solve cannot be corrupted by twist at all.
 *
 * All three inputs are expressed in the node's own sensor-neutral frame, so
 * they are free of both the mount rotation and the per-sensor GRV heading:
 *   down            world vertical at neutral (the arm's long axis there)
 *   sidePointing    where that long axis pointed during the side hold
 *   forwardPointing where it pointed during the forward hold
 *
 * The vertical is exact (gravity). Only the heading of the frame has to come
 * from the wearer's raises, and real raises are rarely a perfect right angle
 * apart (field: 80 degrees). Rather than trusting one raise and pushing the
 * whole error onto the other, the forward direction is taken as the bisector
 * of the forward hold and the side hold turned 90 degrees, so each held pose
 * renders within half of the disagreement.
 *
 * Returns the mount M with anatomical axes as its columns: X right, Y down,
 * Z forward, expressed in the sensor-neutral frame. The engine applies it as
 * conj(M) * D * M.
 */
export function solvePointingMount(down, sidePointing, forwardPointing, {
  minElevationDeg = 45,
  maxElevationDeg = 135,
  maxDisagreementDeg = 30,
} = {}) {
  if (![down, sidePointing, forwardPointing].every(finiteVector)) {
    return failure("non_finite_axis");
  }
  const y = normalize(down);
  const side = normalize(sidePointing);
  const forward = normalize(forwardPointing);
  if (!y || !side || !forward) return failure("zero_axis");

  const sideElevationDeg = degreesBetween(y, side);
  const forwardElevationDeg = degreesBetween(y, forward);
  const measured = { sideElevationDeg, forwardElevationDeg };
  // A raise near vertical has no usable horizontal direction to read.
  if (sideElevationDeg < minElevationDeg || forwardElevationDeg < minElevationDeg) {
    return { ...failure("raise_too_low"), quality: measured };
  }
  if (sideElevationDeg > maxElevationDeg || forwardElevationDeg > maxElevationDeg) {
    return { ...failure("raise_too_high"), quality: measured };
  }

  const sideFlat = horizontal(side, y);
  const forwardFlat = horizontal(forward, y);
  if (!sideFlat || !forwardFlat) return { ...failure("raise_too_low"), quality: measured };

  // X = Y x Z for this right-handed frame, so a side (+X) direction implies
  // forward = X x Y.
  const forwardFromSide = normalize(cross(sideFlat, y));
  const pointingSeparationDeg = degreesBetween(sideFlat, forwardFlat);
  const disagreementDeg = degreesBetween(forwardFlat, forwardFromSide);
  const quality = { ...measured, pointingSeparationDeg, disagreementDeg };
  if (disagreementDeg > maxDisagreementDeg) {
    return { ...failure("raises_not_perpendicular"), quality };
  }

  const z = normalize(forwardFlat.map((value, index) => value + forwardFromSide[index]));
  if (!z) return { ...failure("raises_not_perpendicular"), quality };
  const x = normalize(cross(y, z));
  const mountMatrix = columns(x, y, z);
  const det = determinant(mountMatrix);
  const error = orthogonalityError(mountMatrix);
  const mount = matrixToQuaternion(mountMatrix);
  if (!mount || !Number.isFinite(det) || Math.abs(det - 1) > 1e-6 || error > 1e-6) {
    return { ...failure("invalid_rotation"), quality };
  }
  return {
    ok: true,
    mount,
    quality: { ...quality, determinant: det, orthogonalityError: error },
    reason: null,
  };
}

/**
 * The wearer's palm direction at neutral, in anatomical axes, taken from the
 * side hold where the palm is instructed to face the floor. The sensors cannot
 * see which way the palm faces relative to the strap, so one pose has to name
 * it; after that, every forearm twist (pronation, supination, shoulder
 * rotation) moves the rendered palm with the wearer's. Field check on the
 * 09:10 session: with the reference taken from the side hold alone, the palm
 * in the separate forward holds came out 12-13 degrees from facing down, where
 * the wearer held it.
 *
 * `sideDeltaAnatomical` is the side-hold rotation already mapped through the
 * mount. Returns a horizontal unit vector, or null if it is degenerate.
 */
export function palmRestNormal(sideDeltaAnatomical) {
  const q = sideDeltaAnatomical;
  if (!Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite)) return null;
  // Rotate world-down back through the side hold: conj(q) * (0,1,0) * q.
  const [w, x, y, z] = q;
  const v = [0, 1, 0];
  const cx = -x;
  const cy = -y;
  const cz = -z;
  const tx = 2 * (cy * v[2] - cz * v[1]);
  const ty = 2 * (cz * v[0] - cx * v[2]);
  const tz = 2 * (cx * v[1] - cy * v[0]);
  const palm = [
    v[0] + w * tx + cy * tz - cz * ty,
    v[1] + w * ty + cz * tx - cx * tz,
    v[2] + w * tz + cx * ty - cy * tx,
  ];
  return horizontal(palm, [0, 1, 0]);
}
