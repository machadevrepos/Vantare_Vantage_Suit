import assert from "node:assert/strict";
import {
  ArmAvatar,
  ArmPoseSmoother,
  anatomicalArmMarkup,
  armPoseForMotion,
  rotationMatrixForQuaternion,
  slerpQuaternion,
} from "../../live_tool/js/arm-avatar.js";

const healthy = {
  health: { calibrated: true, n2: true, n3: true, n4: true, synchronized: true },
  diagnostics: { flexionValid: true },
  elbow_flexion_deg: 90,
  upper_arm_orientation: { qx: 0, qy: 0, qz: 0, qw: 1 },
};

// A wrong implementation that applies an uncalibrated or stale frame would
// make the avatar look live when the Motion Engine explicitly says it is not.
assert.deepEqual(armPoseForMotion({ ...healthy, health: { ...healthy.health, calibrated: false } }), {
  state: "calibration_required",
  elbowDeg: 0,
  shoulderDeg: { x: 0, y: 0, z: 0 },
});

assert.deepEqual(armPoseForMotion({ ...healthy, health: { ...healthy.health, synchronized: false } }), {
  state: "tracking_unavailable",
  elbowDeg: 0,
  shoulderDeg: { x: 0, y: 0, z: 0 },
});

assert.deepEqual(armPoseForMotion({ ...healthy, elbow_flexion_deg: null, diagnostics: { flexionValid: null } }), {
  state: "shoulder_live",
  elbowDeg: 0,
  shoulderDeg: { x: 0, y: 0, z: 0 },
});

assert.deepEqual(armPoseForMotion(healthy), {
  state: "live", elbowDeg: 90, shoulderDeg: { x: 0, y: 0, z: 0 },
});

// N4's calibrated delta must move the shoulder/upper-arm segment in the test
// visualizer. A regression that drops this quaternion recreates the bicep-only
// avatar even though the live Motion Engine is producing full segment motion.
assert.deepEqual(armPoseForMotion({
  ...healthy,
  upper_arm_orientation: { qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 },
}), {
  state: "live",
  elbowDeg: 90,
  shoulderDeg: { x: 0, y: 0, z: 90 },
});

// Hinge calibration is required only for elbow flexion. Upper-arm tracking
// remains useful while the wearer performs a shoulder-motion test.
assert.deepEqual(armPoseForMotion({
  ...healthy,
  elbow_flexion_deg: null,
  diagnostics: { flexionValid: null },
  upper_arm_orientation: { qx: 0, qy: 0, qz: 0, qw: 1 },
}), {
  state: "shoulder_live",
  elbowDeg: 0,
  shoulderDeg: { x: 0, y: 0, z: 0 },
});

// Applying three Euler angles in CSS order loses the quaternion's rotation
// semantics around combined axes. The renderer must consume the quaternion as
// one matrix so 360-degree shoulder sweeps do not hit Euler/gimbal artifacts.
assert.deepEqual(
  rotationMatrixForQuaternion({ qx: 0, qy: 0, qz: 0, qw: 1 }),
  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
);
assert.deepEqual(
  rotationMatrixForQuaternion({ qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 }),
  [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
);

// The visual fast path must apply a newly arrived calibrated N4 quaternion
// immediately instead of waiting up to one full 40 ms analytics tick.
const styleValues = new Map();
const avatar = new ArmAvatar({
  dataset: {},
  querySelector: () => null,
  style: { setProperty: (name, value) => styleValues.set(name, value) },
});
avatar.renderShoulder({ qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 });
assert.equal(
  styleValues.get("--upper-matrix"),
  "matrix3d(0,1,0,0,-1,0,0,0,0,0,1,0,0,0,0,1)",
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

// Quaternion interpolation must take the shortest hemisphere path. Treating q
// and -q as different rotations creates a full-spin flicker at sign changes.
const qIdentity = { qx: 0, qy: 0, qz: 0, qw: 1 };
const qZ90 = { qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 };
assert.deepEqual(slerpQuaternion(qIdentity, { qx: 0, qy: 0, qz: 0, qw: -1 }, 0.5), qIdentity);
const qZ45 = slerpQuaternion(qIdentity, qZ90, 0.5);
assert.ok(Math.abs(qZ45.qz - 0.382683432) < 1e-6);
assert.ok(Math.abs(qZ45.qw - 0.923879533) < 1e-6);

// A one-frame synchronization miss must hold the last valid pose. The old
// renderer returned elbowDeg=0 here, producing the recorded snap/flicker.
const smoother = new ArmPoseSmoother({ dropoutHoldMs: 250 });
smoother.pushFrame(healthy, 0);
assert.deepEqual(smoother.sample(0), { shoulder: qIdentity, elbowDeg: 90, state: "live" });
smoother.pushFrame({ ...healthy, health: { ...healthy.health, synchronized: false } }, 40);
assert.deepEqual(smoother.sample(120), { shoulder: qIdentity, elbowDeg: 90, state: "live" });
assert.deepEqual(smoother.sample(310), { shoulder: qIdentity, elbowDeg: 90, state: "tracking_unavailable" });

// Recovery and large intentional motion must blend across display frames,
// neither jumping to the target nor remaining visibly stuck.
smoother.pushFrame({ ...healthy, elbow_flexion_deg: 0, upper_arm_orientation: qZ90 }, 320);
const firstRecovery = smoother.sample(336);
assert.equal(firstRecovery.state, "live");
assert.ok(firstRecovery.elbowDeg > 0 && firstRecovery.elbowDeg < 90);
assert.ok(firstRecovery.shoulder.qz > 0 && firstRecovery.shoulder.qz < Math.SQRT1_2);
assert.ok(90 - firstRecovery.elbowDeg <= 15, "one display frame must not jump over 15 degrees");
const firstShoulderStepDeg = 2 * Math.acos(firstRecovery.shoulder.qw) * 180 / Math.PI;
assert.ok(firstShoulderStepDeg <= 15, "one display frame must not rotate over 15 degrees");
let converged = firstRecovery;
for (let t = 352; t <= 544; t += 16) converged = smoother.sample(t);
assert.ok(converged.elbowDeg < 5);
assert.ok(converged.shoulder.qz > 0.68);
console.log("Arm avatar frame-contract tests passed");
