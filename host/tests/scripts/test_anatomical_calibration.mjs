import assert from "node:assert/strict";
import { solveMountCorrection } from "../../live_tool/js/anatomical-calibration.js";

const SIDE = [0, 0, -1];
const FORWARD = [1, 0, 0];

function normalize(values) {
  const n = Math.hypot(...values);
  return values.map((value) => value / n);
}

function axisAngle(axis, degrees) {
  const [x, y, z] = normalize(axis);
  const half = degrees * Math.PI / 360;
  const s = Math.sin(half);
  return [Math.cos(half), x * s, y * s, z * s];
}

// Independent reference implementation: Hamilton q * [0,v] * conjugate(q).
function rotate(q, v) {
  const [w, x, y, z] = q;
  const [vx, vy, vz] = v;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ];
}

function conjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

function assertVectorClose(actual, expected, tolerance = 1e-8) {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, index) => {
    assert.ok(Math.abs(value - expected[index]) <= tolerance,
      `axis ${index}: expected ${expected[index]}, got ${value}`);
  });
}

for (const mount of [
  [1, 0, 0, 0],
  axisAngle([0.31, -0.77, 0.55], 137),
  axisAngle([-0.62, 0.19, 0.76], 84),
]) {
  // source = M * target; the installed conjugate(M) mapping must recover target.
  const sourceSide = rotate(mount, SIDE);
  const sourceForward = rotate(mount, FORWARD);
  const solved = solveMountCorrection(sourceSide, sourceForward);
  assert.equal(solved.ok, true, solved.reason);
  const map = conjugate(solved.mount);
  assertVectorClose(rotate(map, sourceSide), SIDE);
  assertVectorClose(rotate(map, sourceForward), FORWARD);
  assert.ok(Math.abs(solved.quality.determinant - 1) < 1e-8);
  assert.ok(solved.quality.orthogonalityError < 1e-8);
  assert.ok(Math.abs(solved.quality.axisSeparationDeg - 90) < 1e-8);
}

assert.deepEqual(solveMountCorrection([0, 0, -1], [0, 0, -2]), {
  ok: false,
  mount: null,
  quality: null,
  reason: "axes_not_independent",
});
assert.equal(solveMountCorrection([0, 0, -1], [0.2, 0, -0.98]).reason, "axes_not_independent");

// Off-plane side raises are measured, not rejected: the mount frame is
// defined by the directions the wearer actually held, so the first captured
// axis maps exactly and the second maps to the same angle from it in the
// anatomical plane. Only pairs too close to a single direction (<25 or >155
// degrees apart) are refused.
const raisedAxis = (offPlaneDeg) => [
  Math.sin((offPlaneDeg * Math.PI) / 180), 0, -Math.cos((offPlaneDeg * Math.PI) / 180),
];
for (const offPlaneDeg of [10, 25, 60]) {
  const source = raisedAxis(offPlaneDeg);
  const solved = solveMountCorrection(source, FORWARD);
  assert.equal(solved.ok, true, `${offPlaneDeg} deg: ${solved.reason}`);
  const map = conjugate(solved.mount);
  assertVectorClose(rotate(map, source), SIDE);
  const mappedForward = rotate(map, FORWARD);
  const dotSide = mappedForward[0] * SIDE[0] + mappedForward[1] * SIDE[1] + mappedForward[2] * SIDE[2];
  const mappedSeparation = (Math.acos(Math.max(-1, Math.min(1, dotSide))) * 180) / Math.PI;
  assert.ok(
    Math.abs(mappedSeparation - solved.quality.axisSeparationDeg) < 1e-8,
    `second axis must land at the measured separation, got ${mappedSeparation}`
  );
}
// 10 degrees of separation and a 170-degree reversal remain degenerate.
assert.equal(solveMountCorrection(raisedAxis(80), FORWARD).reason, "axes_not_independent");
assert.equal(solveMountCorrection(raisedAxis(-80), FORWARD).reason, "axes_not_independent");
assert.equal(solveMountCorrection(raisedAxis(66), FORWARD).reason, "axes_not_independent");
assert.equal(solveMountCorrection([Number.NaN, 0, 1], [1, 0, 0]).reason, "non_finite_axis");
assert.equal(solveMountCorrection([0, 0, 0], [1, 0, 0]).reason, "zero_axis");

console.log("Anatomical calibration TRIAD tests passed");
