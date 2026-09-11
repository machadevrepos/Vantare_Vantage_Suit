/**
 * Anatomical arm avatar for the live Coach Assist tool.
 *
 * Corrected upper-arm and forearm orientations from the Motion Engine render
 * as a nested transform: the forearm is drawn relative to the upper arm
 * (conjugate(upper) * forearm), so the elbow articulates in every shoulder
 * direction. Directional data is consumed only while the Motion Engine
 * reports axisFrame "anatomical"; anything else holds or resets the display
 * rather than implying anatomical accuracy.
 */

function cleanMatrixValue(value) {
  const rounded = Number(value.toFixed(9));
  return Math.abs(rounded) < 1e-9 ? 0 : rounded;
}

const IDENTITY_QUATERNION = Object.freeze({ qx: 0, qy: 0, qz: 0, qw: 1 });

function normalizeQuaternion(quaternion) {
  const { qx: x, qy: y, qz: z, qw: w } = quaternion || {};
  if (![x, y, z, w].every(Number.isFinite)) return null;
  const norm = Math.hypot(x, y, z, w);
  if (norm < 1e-9) return null;
  return { qx: x / norm, qy: y / norm, qz: z / norm, qw: w / norm };
}

function quatMultiply(a, b) {
  return {
    qw: a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz,
    qx: a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy,
    qy: a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx,
    qz: a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw,
  };
}

function quatConjugate(q) {
  return { qx: -q.qx, qy: -q.qy, qz: -q.qz, qw: q.qw };
}

/** Shortest-path spherical interpolation; q and -q are the same rotation. */
export function slerpQuaternion(from, to, amount) {
  const a = normalizeQuaternion(from) || { ...IDENTITY_QUATERNION };
  let b = normalizeQuaternion(to) || { ...a };
  const t = Math.max(0, Math.min(1, Number(amount) || 0));
  let dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
  if (dot < 0) {
    dot = -dot;
    b = { qx: -b.qx, qy: -b.qy, qz: -b.qz, qw: -b.qw };
  }
  if (dot > 0.9995) {
    return normalizeQuaternion({
      qx: a.qx + t * (b.qx - a.qx),
      qy: a.qy + t * (b.qy - a.qy),
      qz: a.qz + t * (b.qz - a.qz),
      qw: a.qw + t * (b.qw - a.qw),
    }) || { ...a };
  }
  const theta = Math.acos(Math.max(-1, Math.min(1, dot)));
  const sinTheta = Math.sin(theta);
  const fromWeight = Math.sin((1 - t) * theta) / sinTheta;
  const toWeight = Math.sin(t * theta) / sinTheta;
  return {
    qx: a.qx * fromWeight + b.qx * toWeight,
    qy: a.qy * fromWeight + b.qy * toWeight,
    qz: a.qz * fromWeight + b.qz * toWeight,
    qw: a.qw * fromWeight + b.qw * toWeight,
  };
}

/** Never extrapolate further than one 25 Hz sample interval past the last pose. */
const MAX_DISPLAY_LEAD_MS = 40;
/** A gyro glitch must not throw the rig across the room (15 rad/s ~ 860 deg/s). */
const MAX_OMEGA_RADPS = 15;

function clampRotationVector(v) {
  const magnitude = Math.hypot(v[0], v[1], v[2]);
  if (!(magnitude > MAX_OMEGA_RADPS)) return [v[0], v[1], v[2]];
  const scale = MAX_OMEGA_RADPS / magnitude;
  return [v[0] * scale, v[1] * scale, v[2] * scale];
}

/** Rotation vector (rad/s) integrated over `seconds` -> unit quaternion. */
function quatFromRotationVector(rotationVector, seconds) {
  const [x, y, z] = rotationVector || [0, 0, 0];
  const rate = Math.hypot(x, y, z);
  const angle = rate * seconds;
  if (!(angle > 1e-9)) return { ...IDENTITY_QUATERNION };
  const scale = Math.sin(angle / 2) / rate;
  return normalizeQuaternion({
    qx: x * scale,
    qy: y * scale,
    qz: z * scale,
    qw: Math.cos(angle / 2),
  }) || { ...IDENTITY_QUATERNION };
}

/** Angular velocity (rad/s) taking `from` to `to`, in `from`'s parent frame. */
function rotationVectorBetween(from, to, dtSeconds) {
  const a = normalizeQuaternion(from);
  let b = normalizeQuaternion(to);
  if (!a || !b || !(dtSeconds > 0)) return [0, 0, 0];
  const dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
  if (dot < 0) b = { qx: -b.qx, qy: -b.qy, qz: -b.qz, qw: -b.qw };
  const delta = quatMultiply(b, quatConjugate(a));
  let { qx: x, qy: y, qz: z, qw: w } = delta;
  if (w < 0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }
  const s = Math.hypot(x, y, z);
  if (s < 1e-9) return [0, 0, 0];
  const angle = 2 * Math.atan2(s, w);
  return [(x / s) * (angle / dtSeconds), (y / s) * (angle / dtSeconds), (z / s) * (angle / dtSeconds)];
}

/** Accept both packet ({qx..}) and MotionEngine array ([w,x,y,z]) shapes. */
function asQuaternion(value) {
  if (Array.isArray(value)) {
    return normalizeQuaternion({ qw: value[0], qx: value[1], qy: value[2], qz: value[3] });
  }
  return normalizeQuaternion(value);
}

/**
 * Display-only predictive pose tracker. It never changes the MotionEngine
 * packet used by logging, rep analysis, or qualification.
 *
 * The BLE grid is 25 Hz (40 ms), so a freshly received pose is already up to
 * 40 ms old by the time the next display frame runs. Instead of interpolating
 * toward it (which adds lag), the display extrapolates from the newest target
 * by its age using the BNO gyro's instantaneous angular rate: the rig shows
 * the arm's estimated CURRENT orientation at every 60 Hz paint. Finite
 * differences between targets are the fallback when no gyro rate is supplied
 * (session replay, tests).
 */
export class ArmPoseSmoother {
  constructor({ dropoutHoldMs = 120, maxLeadMs = MAX_DISPLAY_LEAD_MS } = {}) {
    this.dropoutHoldMs = dropoutHoldMs;
    this.maxLeadMs = maxLeadMs;
    this.currentUpper = { ...IDENTITY_QUATERNION };
    this.targetUpper = { ...IDENTITY_QUATERNION };
    this.currentForearmRelative = { ...IDENTITY_QUATERNION };
    this.targetForearmRelative = { ...IDENTITY_QUATERNION };
    this.omegaUpper = [0, 0, 0];
    this.omegaForearm = [0, 0, 0];
    this.targetAtMs = null;
    this.hasPose = false;
    this.inputState = "anatomical_calibration_required";
    this.lastLiveState = "live";
    this.lastValidAtMs = null;
  }

  reset() {
    this.currentUpper = { ...IDENTITY_QUATERNION };
    this.targetUpper = { ...IDENTITY_QUATERNION };
    this.currentForearmRelative = { ...IDENTITY_QUATERNION };
    this.targetForearmRelative = { ...IDENTITY_QUATERNION };
    this.omegaUpper = [0, 0, 0];
    this.omegaForearm = [0, 0, 0];
    this.targetAtMs = null;
    this.hasPose = false;
    this.lastValidAtMs = null;
  }

  /** Set targets, with explicit gyro rates when the caller has them. */
  setTargets(upper, forearmRelative, nowMs, omegaUpper = null, omegaForearm = null) {
    const dtS = this.targetAtMs === null ? 0 : (nowMs - this.targetAtMs) / 1000;
    if (omegaUpper) {
      this.omegaUpper = clampRotationVector(omegaUpper);
    } else if (dtS > 0.004 && dtS < 0.25) {
      this.omegaUpper = clampRotationVector(rotationVectorBetween(this.targetUpper, upper, dtS));
    }
    if (omegaForearm) {
      this.omegaForearm = clampRotationVector(omegaForearm);
    } else if (dtS > 0.004 && dtS < 0.25) {
      this.omegaForearm = clampRotationVector(
        rotationVectorBetween(this.targetForearmRelative, forearmRelative, dtS)
      );
    }
    this.targetUpper = upper;
    this.targetForearmRelative = forearmRelative;
    this.targetAtMs = nowMs;
    if (!this.hasPose) {
      this.currentUpper = { ...upper };
      this.currentForearmRelative = { ...forearmRelative };
      this.hasPose = true;
    }
  }

  pushFrame(frame, nowMs) {
    const pose = armPoseForMotion(frame);
    if (pose.state === "anatomical_calibration_required") {
      this.inputState = pose.state;
      this.reset();
      return;
    }
    if (pose.state === "tracking_unavailable") {
      this.inputState = pose.state;
      return;
    }
    this.setTargets(pose.shoulder, pose.forearmRelative, nowMs);
    this.inputState = pose.state;
    this.lastLiveState = pose.state;
    this.lastValidAtMs = nowMs;
  }

  /** Fast path: a freshly received pose plus gyro rates, no state change. */
  pushPose(pose, nowMs) {
    const upper = asQuaternion(pose.shoulder);
    const forearmRelative = asQuaternion(pose.forearmRelative);
    if (!upper || !forearmRelative) return;
    this.setTargets(upper, forearmRelative, nowMs, pose.omegaShoulder, pose.omegaForearm);
  }

  /** Display-only fast path for a freshly received calibrated N4 sample. */
  pushShoulder(quaternion, nowMs = performance.now()) {
    const shoulder = asQuaternion(quaternion);
    if (!shoulder) return;
    this.setTargets(shoulder, this.targetForearmRelative, nowMs);
  }

  sample(nowMs) {
    if (!this.hasPose) {
      return {
        shoulder: { ...IDENTITY_QUATERNION },
        forearmRelative: { ...IDENTITY_QUATERNION },
        state: this.inputState,
      };
    }
    const ageS = this.targetAtMs === null ? 0 : Math.max(0, nowMs - this.targetAtMs) / 1000;
    const leadS = Math.min(ageS, this.maxLeadMs / 1000);
    const upperAdvance = quatFromRotationVector(this.omegaUpper, leadS);
    const forearmAdvance = quatFromRotationVector(this.omegaForearm, leadS);
    this.currentUpper = quatMultiply(upperAdvance, this.targetUpper);
    this.currentForearmRelative = quatMultiply(forearmAdvance, this.targetForearmRelative);
    let state = this.inputState;
    if (state === "tracking_unavailable" && this.lastValidAtMs !== null &&
        nowMs - this.lastValidAtMs <= this.dropoutHoldMs) {
      state = this.lastLiveState;
    }
    return {
      shoulder: { ...this.currentUpper },
      forearmRelative: { ...this.currentForearmRelative },
      state,
    };
  }
}

/**
 * CSS column-major rotation matrix for an anatomical packet quaternion.
 * Packet axes: +X wearer-right, +Y down, +Z forward. In the front-facing
 * mannequin CSS +X is screen-right, +Y down, +Z toward the viewer, so the
 * coordinate basis is B = diag(-1, 1, 1). Apply B R(q) B^-1 to BOTH nested
 * segment rotations. A quaternion's axial vector transforms as det(B) B v:
 * (x, y, z, w) -> (x, -y, -z, w). Keep engine/smoother data anatomical.
 * Keeping the quaternion whole avoids Euler order ambiguity and gimbal lock.
 */
export function rotationMatrixForQuaternion(quaternion) {
  const { qx: px, qy: py, qz: pz, qw: pw } = quaternion || {};
  if (![px, py, pz, pw].every(Number.isFinite)) {
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  }
  const norm = Math.hypot(px, py, pz, pw);
  if (norm < 1e-9) return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const x = px / norm;
  const y = -py / norm;
  const z = -pz / norm;
  const w = pw / norm;
  const values = [
    1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0,
    2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0,
    2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0,
    0, 0, 0, 1,
  ];
  return values.map(cleanMatrixValue);
}

/**
 * Classify a MotionEngine frame for the rig. Directional transforms exist only
 * after both mount corrections are installed (axisFrame "anatomical").
 */
export function armPoseForMotion(frame) {
  const health = frame?.health;
  const axisFrame = frame?.diagnostics?.axisFrame;
  if (!health?.calibrated || axisFrame !== "anatomical") {
    return {
      state: "anatomical_calibration_required",
      shoulder: { ...IDENTITY_QUATERNION },
      forearmRelative: { ...IDENTITY_QUATERNION },
    };
  }
  const upper = normalizeQuaternion(frame.upper_arm_orientation);
  const forearm = normalizeQuaternion(frame.forearm_orientation);
  if (!health.n2 || !health.n4 || !health.synchronized || !upper || !forearm) {
    return {
      state: "tracking_unavailable",
      shoulder: upper || { ...IDENTITY_QUATERNION },
      forearmRelative: forearm || { ...IDENTITY_QUATERNION },
    };
  }
  return {
    state: "live",
    shoulder: upper,
    forearmRelative: quatMultiply(quatConjugate(upper), forearm),
  };
}

const LABELS = {
  anatomical_calibration_required: "Run the three-pose calibration",
  tracking_unavailable: "Live tracking unavailable",
  live: "Live arm tracking",
};

/** Human-readable anatomy used by the live CSS 3D rig. */
export function anatomicalArmMarkup() {
  const fingers = ["index", "middle", "ring", "little"]
    .map((name) => `
      <div class="arm-finger arm-finger-${name}" data-arm-part="finger" aria-label="${name} finger">
        <i class="arm-phalange proximal"></i><i class="arm-phalange middle"></i><i class="arm-phalange distal"></i>
      </div>`).join("");
  return `
    <div class="body-reference-note">Full body reference &middot; right arm tracked</div>
    <div class="body-rig" role="img" aria-label="Full human body with tracked right arm; torso, left arm and legs are an untracked neutral reference">
      <div class="body-head"><span class="body-face"></span></div>
      <div class="body-neck"></div>
      <div class="body-torso"><span class="body-chest-line"></span><span class="body-abdomen"></span></div>
      <div class="body-pelvis"></div>
      <div class="body-rest-arm"><div class="body-rest-upper"></div><div class="body-rest-elbow"></div><div class="body-rest-forearm"></div><div class="body-rest-hand"></div></div>
      <div class="body-leg body-leg-right"><div class="body-thigh"></div><div class="body-knee"></div><div class="body-shin"></div><div class="body-foot"></div></div>
      <div class="body-leg body-leg-left"><div class="body-thigh"></div><div class="body-knee"></div><div class="body-shin"></div><div class="body-foot"></div></div>
      <div class="arm-rig" aria-label="Live articulated human arm">
      <div class="arm-shoulder" data-arm-part="shoulder"><span class="anatomy-highlight"></span></div>
      <div class="arm-upper" data-arm-part="upper-arm">
        <div class="arm-limb arm-upper-surface"><span class="arm-muscle biceps"></span><span class="arm-muscle triceps"></span></div>
        <div class="arm-node-wrap shoulder"><span class="arm-node-dot"></span><span class="arm-node-label">N4 upper arm</span></div>
        <div class="arm-elbow" data-arm-part="elbow"><span class="elbow-point"></span></div>
        <div class="arm-node-wrap elbow"><span class="arm-node-dot"></span><span class="arm-node-label">N3 elbow</span></div>
        <div class="arm-forearm" data-arm-part="forearm">
          <div class="arm-limb arm-forearm-surface"><span class="arm-muscle forearm"></span></div>
          <div class="arm-wrist" data-arm-part="wrist"></div>
          <div class="arm-node-wrap wrist"><span class="arm-node-dot"></span><span class="arm-node-label">N2 wrist</span></div>
          <div class="arm-hand" data-arm-part="hand">
            <div class="arm-palm" data-arm-part="palm"><span class="palm-pad"></span></div>
            <div class="arm-thumb" data-arm-part="thumb"><i></i><i></i></div>
            <div class="arm-fingers">${fingers}</div>
          </div>
        </div>
      </div>
    </div>
    </div>
    <div class="body-reference-legend">Muted body = untracked reference &middot; gold markers = arm sensors</div>
    <div class="arm-avatar-label" data-arm-status>Run the three-pose calibration</div>`;
}

export class ArmAvatar {
  constructor(root) {
    this.root = root;
    if (root) root.innerHTML = anatomicalArmMarkup();
    this.status = root?.querySelector("[data-arm-status]") || null;
    this.smoother = new ArmPoseSmoother();
    this.animationFrame = null;
    if (typeof requestAnimationFrame === "function") {
      const animate = (nowMs) => {
        this.paintAt(nowMs);
        this.animationFrame = requestAnimationFrame(animate);
      };
      this.animationFrame = requestAnimationFrame(animate);
    }
  }

  render(frame, nowMs = performance.now()) {
    if (!this.root) return;
    this.smoother.pushFrame(frame, nowMs);
    if (typeof requestAnimationFrame !== "function") this.paintAt(nowMs);
  }

  /** Display-only fast path for a freshly received calibrated N4 sample. */
  renderShoulder(quaternion, nowMs = performance.now()) {
    if (!this.root || !quaternion) return;
    this.smoother.pushShoulder(quaternion, nowMs);
    if (typeof requestAnimationFrame !== "function") this.paintAt(nowMs);
  }

  /** Full-pose fast path: both segments plus gyro rates from one arrival. */
  renderPose(pose, nowMs = performance.now()) {
    if (!this.root || !pose) return;
    this.smoother.pushPose(pose, nowMs);
    if (typeof requestAnimationFrame !== "function") this.paintAt(nowMs);
  }

  paintAt(nowMs) {
    if (!this.root) return;
    const pose = this.smoother.sample(nowMs);
    const upperMatrix = rotationMatrixForQuaternion(pose.shoulder);
    const forearmMatrix = rotationMatrixForQuaternion(pose.forearmRelative);
    this.root.style.setProperty("--upper-matrix", `matrix3d(${upperMatrix.join(",")})`);
    this.root.style.setProperty("--forearm-matrix", `matrix3d(${forearmMatrix.join(",")})`);
    this.root.dataset.state = pose.state;
    if (this.status) this.status.textContent = LABELS[pose.state] ?? "";
  }
}
