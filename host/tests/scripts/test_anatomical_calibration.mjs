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

// Off-plane side raises must be rejected, not absorbed into the mount. A
// raise 10 degrees forward of the coronal plane puts the observed axes 80
// degrees apart (accepted: the acceptance criterion allows 10 degrees of
// display error); 15 and 30 degrees land at 75 and 60 and would render
// exactly that far off the instructed plane if accepted.
const raisedAxis = (offPlaneDeg) => [
  Math.sin((offPlaneDeg * Math.PI) / 180), 0, -Math.cos((offPlaneDeg * Math.PI) / 180),
];
assert.equal(solveMountCorrection(raisedAxis(10), FORWARD).ok, true);
assert.equal(solveMountCorrection(raisedAxis(-10), FORWARD).ok, true);
assert.equal(solveMountCorrection(raisedAxis(15), FORWARD).reason, "axes_not_independent");
assert.equal(solveMountCorrection(raisedAxis(30), FORWARD).reason, "axes_not_independent");
assert.equal(solveMountCorrection([Number.NaN, 0, 1], [1, 0, 0]).reason, "non_finite_axis");
assert.equal(solveMountCorrection([0, 0, 0], [1, 0, 0]).reason, "zero_axis");

console.log("Anatomical calibration TRIAD tests passed");
