import assert from "node:assert/strict";
import {
  ArmAvatar,
  ArmPoseSmoother,
  anatomicalArmMarkup,
  armPoseForMotion,
  rotationMatrixForQuaternion,
  slerpQuaternion,
} from "../../live_tool/js/arm-avatar.js";

// ---------------------------------------------------------------- helpers

const qIdentity = { qx: 0, qy: 0, qz: 0, qw: 1 };

function qAxisAngle([x, y, z], degrees) {
  const norm = Math.hypot(x, y, z);
  const half = (degrees * Math.PI) / 360;
  const s = Math.sin(half);
  return { qx: (x / norm) * s, qy: (y / norm) * s, qz: (z / norm) * s, qw: Math.cos(half) };
}

function qMul(a, b) {
  return {
    qw: a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz,
    qx: a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy,
    qy: a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx,
    qz: a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw,
  };
}

function qConj(q) {
  return { qx: -q.qx, qy: -q.qy, qz: -q.qz, qw: q.qw };
}

function separationDeg(a, b) {
  const dot = Math.abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw);
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

function assertQuatClose(actual, expected, tolDeg = 0.5, label = "quaternion") {
  assert.ok(
    separationDeg(actual, expected) <= tolDeg,
    `${label}: ${JSON.stringify(actual)} vs ${JSON.stringify(expected)}`
  );
}

const health = { calibrated: true, n2: true, n3: true, n4: true, synchronized: true };

function anatomicalFrame(upper, forearm) {
  return {
    health: { ...health },
    diagnostics: { axisFrame: "anatomical", flexionValid: true },
    upper_arm_orientation: upper,
    forearm_orientation: forearm,
  };
}

const sensorNeutralFrame = {
  health: { ...health },
  diagnostics: { axisFrame: "sensor_neutral", flexionValid: null },
  upper_arm_orientation: qIdentity,
  forearm_orientation: qIdentity,
};

// ------------------------------------------------- anatomical gating

// Directional rendering is only honest once both mount corrections are
// installed. A sensor-neutral frame must never drive the segment rig, even
// when it happens to carry calibrated-neutral quaternions.
const neutralPose = armPoseForMotion(sensorNeutralFrame);
assert.equal(neutralPose.state, "anatomical_calibration_required");
assertQuatClose(neutralPose.shoulder, qIdentity, 1e-6, "neutral shoulder");
assertQuatClose(neutralPose.forearmRelative, qIdentity, 1e-6, "neutral forearm");

// Uncalibrated engines report sensor-neutral, but a missing neutral must gate too.
assert.equal(
  armPoseForMotion({ health: { ...health, calibrated: false }, diagnostics: { axisFrame: "anatomical" } }).state,
  "anatomical_calibration_required"
);

// ---------------------------------------- relative forearm kinematics

// A rigid straight-arm raise must leave the forearm relative transform at
// identity: upper and forearm rotate as one body, so the elbow shows no bend.
const raise = qAxisAngle([0.31, -0.52, 0.79], 73);
const rigidPose = armPoseForMotion(anatomicalFrame(raise, raise));
assert.equal(rigidPose.state, "live");
assertQuatClose(rigidPose.shoulder, raise, 1e-6, "rigid upper");
assertQuatClose(rigidPose.forearmRelative, qIdentity, 1e-6, "rigid forearm relative");

// forearmRelative = conjugate(upper) * forearm must recover the same elbow
// bend regardless of where the upper arm points. The plan's literal case is a
// 90-degree bend about the anatomical side axis after a side raise.
const bend = qAxisAngle([0, 0, -1], 90);
const sideUpper = qAxisAngle([1, 0, 0], 90);
const sidePose = armPoseForMotion(anatomicalFrame(sideUpper, qMul(sideUpper, bend)));
assertQuatClose(sidePose.forearmRelative, bend, 1e-6, "side-raise elbow");

const oddUpper = qAxisAngle([0, 1, 0], 37);
const oddPose = armPoseForMotion(anatomicalFrame(oddUpper, qMul(oddUpper, bend)));
assertQuatClose(oddPose.forearmRelative, bend, 1e-6, "odd-pose elbow");

// A dropped node withholds live state; the smoother (not this function)
// decides how long to keep showing the last pose.
assert.equal(
  armPoseForMotion({ ...anatomicalFrame(raise, raise), health: { ...health, synchronized: false } }).state,
  "tracking_unavailable"
);

// ----------------------------------------------------- matrix + markup

// Applying three Euler angles in CSS order loses the quaternion's rotation
// semantics around combined axes. The renderer must consume the quaternion as
// one matrix so 360-degree sweeps do not hit Euler/gimbal artifacts.
assert.deepEqual(
  rotationMatrixForQuaternion({ qx: 0, qy: 0, qz: 0, qw: 1 }),
  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
);
assert.deepEqual(
  rotationMatrixForQuaternion({ qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 }),
  [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
);

// The live avatar must expose a complete recognizable arm rather than two
// generic cylinders. Removing a joint, palm, thumb, or finger is a visible
// anatomy regression even when the motion transforms continue to pass.
const anatomy = anatomicalArmMarkup();
for (const part of ["shoulder", "upper-arm", "elbow", "forearm", "wrist", "hand", "palm", "thumb"]) {
  assert.match(anatomy, new RegExp(`data-arm-part="${part}"`));
}
assert.equal((anatomy.match(/data-arm-part="finger"/g) || []).length, 4);
assert.equal((anatomy.match(/class="arm-phalange/g) || []).length, 12);

// --------------------------------------- smoothing and dropout behavior

// Quaternion interpolation must take the shortest hemisphere path. Treating q
// and -q as different rotations creates a full-spin flicker at sign changes.
assert.deepEqual(slerpQuaternion(qIdentity, { qx: 0, qy: 0, qz: 0, qw: -1 }, 0.5), qIdentity);
const qZ90 = { qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 };
const qZ45 = slerpQuaternion(qIdentity, qZ90, 0.5);
assert.ok(Math.abs(qZ45.qz - 0.382683432) < 1e-6);
assert.ok(Math.abs(qZ45.qw - 0.923879533) < 1e-6);

// A fresh anatomical pose feeds both segment targets; the sampled pose must
// blend toward a 90-degree elbow over a few display frames instead of jumping,
// while the upper arm stays at its own (independent) target.
const smoother = new ArmPoseSmoother();
smoother.pushFrame(anatomicalFrame(qIdentity, qIdentity), 0);
assert.deepEqual(smoother.sample(0), {
  shoulder: qIdentity,
  forearmRelative: qIdentity,
  state: "live",
});
smoother.pushFrame(anatomicalFrame(qIdentity, qMul(qIdentity, bend)), 40);
const firstStep = smoother.sample(56);
assert.ok(firstStep.state === "live");
const firstStepDeg = separationDeg(firstStep.forearmRelative, qIdentity);
assert.ok(firstStepDeg > 1 && firstStepDeg < 90, `elbow must blend, saw ${firstStepDeg} deg`);
let settled = firstStep;
for (let t = 72; t <= 240; t += 16) settled = smoother.sample(t);
assertQuatClose(settled.forearmRelative, bend, 1, "settled elbow");
assertQuatClose(settled.shoulder, qIdentity, 1, "settled upper");
// Fast enough for movement: after ~3 display frames most of the bend is shown.
assert.ok(firstStepDeg >= 30, `display lag too heavy at ${firstStepDeg} deg of 90`);

// A one-frame synchronization miss must hold the last valid pose for the
// 120 ms dropout window, then report tracking_unavailable while STILL
// returning the last transforms - never identity or zero.
smoother.pushFrame(anatomicalFrame(qIdentity, qMul(qIdentity, bend)), 244);
smoother.pushFrame(
  { ...anatomicalFrame(qIdentity, qMul(qIdentity, bend)), health: { ...health, synchronized: false } },
  260
);
assert.equal(smoother.sample(300).state, "live");
const held = smoother.sample(380);
assert.equal(held.state, "tracking_unavailable");
assertQuatClose(held.forearmRelative, bend, 1, "held elbow");
assert.ok(
  separationDeg(held.forearmRelative, qIdentity) > 45,
  "dropout must retain the pose, not collapse to identity"
);
assertQuatClose(held.shoulder, qIdentity, 0.05, "held upper");

// Recovery resumes blending from the held pose.
smoother.pushFrame(anatomicalFrame(qIdentity, qIdentity), 384);
assert.equal(smoother.sample(400).state, "live");
assert.ok(separationDeg(smoother.sample(400).forearmRelative, bend) < 90);

// Leaving the anatomical frame (new neutral) resets the display rather than
// freezing stale directional data on screen.
smoother.pushFrame(sensorNeutralFrame, 500);
const reset = smoother.sample(500);
assert.equal(reset.state, "anatomical_calibration_required");
assertQuatClose(reset.shoulder, qIdentity, 1e-6, "reset upper");
assertQuatClose(reset.forearmRelative, qIdentity, 1e-6, "reset forearm");

// -------------------------------------------------------- paint outputs

// The renderer must publish both nested matrices; the upper-arm fast path
// updates the upper matrix without touching the forearm transform.
const styleValues = new Map();
const avatar = new ArmAvatar({
  dataset: {},
  querySelector: () => null,
  style: { setProperty: (name, value) => styleValues.set(name, value) },
});
avatar.renderShoulder(qZ90);
assert.equal(
  styleValues.get("--upper-matrix"),
  "matrix3d(0,1,0,0,-1,0,0,0,0,0,1,0,0,0,0,1)",
);
const identityMatrix = "matrix3d(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)";
assert.equal(styleValues.get("--forearm-matrix"), identityMatrix);

avatar.render(anatomicalFrame(qIdentity, qMul(qIdentity, bend)), 1000);
avatar.paintAt(1000);
const forearmMatrix = styleValues.get("--forearm-matrix");
assert.ok(forearmMatrix && forearmMatrix !== identityMatrix, "forearm matrix must carry the elbow bend");
assert.equal(styleValues.get("--elbow-deg"), undefined, "scalar elbow transform must be gone");

console.log("Arm avatar nested-kinematics tests passed");
