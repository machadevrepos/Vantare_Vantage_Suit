/**
 * Unit tests for the pointing-direction anatomical solve, the palm reference,
 * and the avatar's hand rest twist.
 *
 *   node host/tests/scripts/test_pointing_calibration.mjs
 *
 * The solve is fed directions expressed in an arbitrary sensor-neutral frame
 * (an unknown mount K applied to the anatomical truth), exactly as the engine
 * produces them, and must recover K: the anatomical axes in that frame.
 */
import assert from "node:assert/strict";
import { palmRestNormal, solvePointingMount } from "../../live_tool/js/anatomical-calibration.js";
import { handRestQuaternion } from "../../live_tool/js/arm-avatar.js";

const X = [1, 0, 0];
const Y = [0, 1, 0];
const Z = [0, 0, 1];

function axisAngle(axis, degrees) {
  const n = Math.hypot(...axis);
  const h = (degrees * Math.PI) / 360;
  const s = Math.sin(h);
  return [Math.cos(h), (axis[0] / n) * s, (axis[1] / n) * s, (axis[2] / n) * s];
}
function mul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
const conj = (q) => [q[0], -q[1], -q[2], -q[3]];
const rotate = (q, v) => mul(mul(q, [0, ...v]), conj(q)).slice(1);
const angleDeg = (a, b) =>
  (Math.acos(Math.max(-1, Math.min(1,
    (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / Math.hypot(...a) / Math.hypot(...b)))) * 180) / Math.PI;
const qAngleDeg = (q) => (2 * Math.acos(Math.min(1, Math.abs(q[0]) / Math.hypot(...q))) * 180) / Math.PI;

// An awkward unknown mount: anatomical axes as seen in the sensor-neutral frame.
const K = axisAngle([0.31, -0.77, 0.55], 137);
const inSensor = (v) => rotate(K, v);

// Perfect holds: the arm points right, then forward.
{
  const r = solvePointingMount(inSensor(Y), inSensor(X), inSensor(Z));
  assert.equal(r.ok, true, r.reason);
  assert.ok(qAngleDeg(mul(conj(r.mount), K)) < 1e-5, "must recover the mount exactly");
  assert.ok(Math.abs(r.quality.pointingSeparationDeg - 90) < 1e-5);
  assert.ok(r.quality.disagreementDeg < 1e-5);
}

// Holds that are not horizontal (a drooping side raise, a high forward raise)
// only change elevation; the heading of the frame must be unaffected.
{
  const droop = rotate(axisAngle([0, 0, 1], 20), X);    // side hold 20 deg below horizontal (+Y is down)
  const high = rotate(axisAngle([1, 0, 0], 25), Z);      // forward hold 25 deg above horizontal
  const r = solvePointingMount(inSensor(Y), inSensor(droop), inSensor(high));
  assert.equal(r.ok, true, r.reason);
  assert.ok(qAngleDeg(mul(conj(r.mount), K)) < 1e-5, "elevation must not skew the frame");
  assert.ok(Math.abs(r.quality.sideElevationDeg - 70) < 1e-5);
  assert.ok(Math.abs(r.quality.forwardElevationDeg - 115) < 1e-5);
}

// A disagreement is split evenly: a side hold 20 deg toward the front leaves
// the side and forward holds each exactly 10 deg from their rendered targets.
{
  const side = rotate(axisAngle(Y, -20), X);             // 20 deg forward of pure right
  const r = solvePointingMount(inSensor(Y), inSensor(side), inSensor(Z));
  assert.equal(r.ok, true, r.reason);
  assert.ok(Math.abs(r.quality.disagreementDeg - 20) < 1e-5);
  const toAnat = (v) => rotate(conj(r.mount), inSensor(v));
  assert.ok(Math.abs(angleDeg(toAnat(side), X) - 10) < 1e-5, "side renders within half");
  assert.ok(Math.abs(angleDeg(toAnat(Z), Z) - 10) < 1e-5, "forward renders within half");
}

// Refusals.
{
  const diag = rotate(axisAngle(Y, -45), X);             // side hold halfway to the front
  assert.equal(solvePointingMount(inSensor(Y), inSensor(diag), inSensor(Z)).reason, "raises_not_perpendicular");
  assert.equal(solvePointingMount(inSensor(Y), inSensor(X), inSensor(X)).reason, "raises_not_perpendicular");
  const low = rotate(axisAngle([0, 0, 1], 60), X);       // only 30 deg up from hanging
  const lowResult = solvePointingMount(inSensor(Y), inSensor(low), inSensor(Z));
  assert.equal(lowResult.reason, "raise_too_low");
  assert.ok(Math.abs(lowResult.quality.sideElevationDeg - 30) < 1e-5, "refusal carries the measured angle");
  assert.equal(solvePointingMount([Number.NaN, 0, 0], X, Z).reason, "non_finite_axis");
  assert.equal(solvePointingMount([0, 0, 0], X, Z).reason, "zero_axis");
}

// Palm reference: whatever the side hold's twist, the neutral palm is the
// direction that the side-hold rotation carries onto the floor.
for (const twist of [0, 51, -90, 170]) {
  const sideHold = mul(axisAngle([0, 0, -1], 90), axisAngle(Y, twist));
  const palm = palmRestNormal(sideHold);
  assert.ok(Math.abs(palm[1]) < 1e-12, "rest palm is horizontal");
  assert.ok(angleDeg(rotate(sideHold, palm), Y) < 1e-5, `twist ${twist}: palm must face down in the side hold`);
}
assert.ok(angleDeg(palmRestNormal(axisAngle([0, 0, -1], 90)), [-1, 0, 0]) < 1e-5,
  "an untwisted side raise means the neutral palm faced the thigh");
assert.equal(palmRestNormal([Number.NaN, 0, 0, 1]), null);

// Avatar hand rest twist: the drawn palm (+Z at rest) must turn onto the
// measured neutral palm, about the forearm's long axis only.
for (const palm of [[-1, 0, 0], [0, 0, -1], [-0.699, 0, -0.715], [0, 0, 1]]) {
  const h = handRestQuaternion(palm);
  const q = [h.qw, h.qx, h.qy, h.qz];
  assert.ok(angleDeg(rotate(q, Z), palm) < 1e-6, `hand rest must face ${palm}`);
  assert.ok(angleDeg(rotate(q, Y), Y) < 1e-5, "the twist must not tilt the hand off the forearm axis");
}
assert.deepEqual(handRestQuaternion(null), { qx: 0, qy: 0, qz: 0, qw: 1 });

console.log("Pointing calibration tests passed");
