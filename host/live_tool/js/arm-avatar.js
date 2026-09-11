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

function quaternionSeparationDeg(a, b) {
  const qa = normalizeQuaternion(a);
  const qb = normalizeQuaternion(b);
  if (!qa || !qb) return 0;
  const dot = Math.abs(qa.qx * qb.qx + qa.qy * qb.qy + qa.qz * qb.qz + qa.qw * qb.qw);
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

/**
 * Time-aware display smoothing: a ~15 ms time constant while the pose is
 * clearly moving, relaxing up to 60 ms so sub-degree jitter is damped without
 * adding visible lag to real movement.
 */
function smoothingAlpha(dtMs, errorDeg) {
  const activity = Math.max(0, Math.min(1, errorDeg / 10));
  const timeConstantMs = 15 + 45 * (1 - activity);
  return 1 - Math.exp(-Math.max(0, Math.min(50, dtMs)) / timeConstantMs);
}

/**
 * Display-only adaptive pose interpolation. It never changes the MotionEngine
 * packet used by logging, rep analysis, or qualification.
 */
export class ArmPoseSmoother {
  constructor({ dropoutHoldMs = 120 } = {}) {
    this.dropoutHoldMs = dropoutHoldMs;
    this.currentUpper = { ...IDENTITY_QUATERNION };
    this.targetUpper = { ...IDENTITY_QUATERNION };
    this.currentForearmRelative = { ...IDENTITY_QUATERNION };
    this.targetForearmRelative = { ...IDENTITY_QUATERNION };
    this.hasPose = false;
    this.inputState = "anatomical_calibration_required";
    this.lastLiveState = "live";
    this.lastValidAtMs = null;
    this.lastSampleAtMs = null;
  }

  pushFrame(frame, nowMs) {
    const pose = armPoseForMotion(frame);
    if (pose.state === "anatomical_calibration_required") {
      this.inputState = pose.state;
      this.hasPose = false;
      this.currentUpper = { ...IDENTITY_QUATERNION };
      this.targetUpper = { ...IDENTITY_QUATERNION };
      this.currentForearmRelative = { ...IDENTITY_QUATERNION };
      this.targetForearmRelative = { ...IDENTITY_QUATERNION };
      this.lastValidAtMs = null;
      this.lastSampleAtMs = null;
      return;
    }
    if (pose.state === "tracking_unavailable") {
      this.inputState = pose.state;
      return;
    }
    this.targetUpper = pose.shoulder;
    this.targetForearmRelative = pose.forearmRelative;
    this.inputState = pose.state;
    this.lastLiveState = pose.state;
    this.lastValidAtMs = nowMs;
    if (!this.hasPose) {
      this.currentUpper = { ...this.targetUpper };
      this.currentForearmRelative = { ...this.targetForearmRelative };
      this.hasPose = true;
    }
  }

  /** Display-only fast path for a freshly received calibrated N4 sample. */
  pushShoulder(quaternion) {
    // MotionEngine.segmentDelta() reports [w, x, y, z]; packet-shape objects
    // also arrive. Normalize both here so the fast path cannot silently no-op.
    const packet = Array.isArray(quaternion)
      ? { qw: quaternion[0], qx: quaternion[1], qy: quaternion[2], qz: quaternion[3] }
      : quaternion;
    const shoulder = normalizeQuaternion(packet);
    if (!shoulder) return;
    this.targetUpper = shoulder;
    if (!this.hasPose) {
      this.currentUpper = { ...shoulder };
      this.hasPose = true;
    }
  }

  sample(nowMs) {
    if (!this.hasPose) {
      return {
        shoulder: { ...IDENTITY_QUATERNION },
        forearmRelative: { ...IDENTITY_QUATERNION },
        state: this.inputState,
      };
    }
    if (this.lastSampleAtMs !== null) {
      const dtMs = nowMs - this.lastSampleAtMs;
      const upperError = quaternionSeparationDeg(this.currentUpper, this.targetUpper);
      const upperAlpha = smoothingAlpha(dtMs, upperError);
      this.currentUpper = slerpQuaternion(this.currentUpper, this.targetUpper, upperAlpha);
      const forearmError = quaternionSeparationDeg(
        this.currentForearmRelative, this.targetForearmRelative
      );
      const forearmAlpha = smoothingAlpha(dtMs, forearmError);
      this.currentForearmRelative = slerpQuaternion(
        this.currentForearmRelative, this.targetForearmRelative, forearmAlpha
      );
    }
    this.lastSampleAtMs = nowMs;
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
 * CSS column-major rotation matrix for a packet quaternion. Keeping the
 * quaternion whole avoids Euler order ambiguity and gimbal lock.
 */
export function rotationMatrixForQuaternion(quaternion) {
  const { qx: px, qy: py, qz: pz, qw: pw } = quaternion || {};
  if (![px, py, pz, pw].every(Number.isFinite)) {
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  }
  const norm = Math.hypot(px, py, pz, pw);
  if (norm < 1e-9) return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const x = px / norm;
  const y = py / norm;
  const z = pz / norm;
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
    <div class="arm-rig" role="img" aria-label="Live articulated human arm">
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
