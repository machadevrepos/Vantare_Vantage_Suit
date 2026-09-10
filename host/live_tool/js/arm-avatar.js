/**
 * Constrained arm avatar for the live Coach Assist tool.
 *
 * N4 drives a mount-relative shoulder/upper-arm transform and the Motion
 * Engine's hinge-calibrated N2-vs-N4 angle drives the elbow. The shoulder axes
 * remain sensor-neutral, so this is a test visualizer rather than an
 * anatomical claim.
 */

function cleanDegrees(value) {
  const rounded = Number(value.toFixed(3));
  return Math.abs(rounded) < 1e-9 ? 0 : rounded;
}

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

function smoothingAlpha(dtMs, error, fullSpeedError) {
  const activity = Math.max(0, Math.min(1, error / fullSpeedError));
  const timeConstantMs = 110 - 75 * activity;
  return 1 - Math.exp(-Math.max(0, Math.min(50, dtMs)) / timeConstantMs);
}

function rateLimitedAlpha(alpha, errorDeg, dtMs) {
  if (errorDeg <= 1e-9) return alpha;
  const maxStepDeg = 540 * Math.max(0, Math.min(50, dtMs)) / 1000;
  return Math.min(alpha, maxStepDeg / errorDeg);
}

/**
 * Display-only adaptive pose interpolation. It never changes the MotionEngine
 * packet used by logging, rep analysis, or qualification.
 */
export class ArmPoseSmoother {
  constructor({ dropoutHoldMs = 250 } = {}) {
    this.dropoutHoldMs = dropoutHoldMs;
    this.currentShoulder = { ...IDENTITY_QUATERNION };
    this.targetShoulder = { ...IDENTITY_QUATERNION };
    this.currentElbowDeg = 0;
    this.targetElbowDeg = 0;
    this.hasPose = false;
    this.inputState = "calibration_required";
    this.lastLiveState = "live";
    this.lastValidAtMs = null;
    this.lastSampleAtMs = null;
  }

  pushFrame(frame, nowMs) {
    const pose = armPoseForMotion(frame);
    if (pose.state === "calibration_required") {
      this.inputState = pose.state;
      this.hasPose = false;
      this.currentShoulder = { ...IDENTITY_QUATERNION };
      this.targetShoulder = { ...IDENTITY_QUATERNION };
      this.currentElbowDeg = 0;
      this.targetElbowDeg = 0;
      this.lastValidAtMs = null;
      this.lastSampleAtMs = null;
      return;
    }
    if (pose.state === "tracking_unavailable") {
      this.inputState = pose.state;
      return;
    }
    const shoulder = normalizeQuaternion(frame?.upper_arm_orientation);
    if (shoulder) this.targetShoulder = shoulder;
    if (pose.state === "live") this.targetElbowDeg = pose.elbowDeg;
    this.inputState = pose.state;
    this.lastLiveState = pose.state;
    this.lastValidAtMs = nowMs;
    if (!this.hasPose) {
      this.currentShoulder = { ...this.targetShoulder };
      this.currentElbowDeg = this.targetElbowDeg;
      this.hasPose = true;
    }
  }

  pushShoulder(quaternion) {
    const shoulder = normalizeQuaternion(quaternion);
    if (!shoulder) return;
    this.targetShoulder = shoulder;
    if (!this.hasPose) {
      this.currentShoulder = { ...shoulder };
      this.hasPose = true;
    }
  }

  sample(nowMs) {
    if (!this.hasPose) {
      return { shoulder: { ...IDENTITY_QUATERNION }, elbowDeg: 0, state: this.inputState };
    }
    if (this.lastSampleAtMs !== null) {
      const dtMs = nowMs - this.lastSampleAtMs;
      const shoulderError = quaternionSeparationDeg(this.currentShoulder, this.targetShoulder);
      const shoulderAlpha = rateLimitedAlpha(
        smoothingAlpha(dtMs, shoulderError, 12), shoulderError, dtMs
      );
      this.currentShoulder = slerpQuaternion(this.currentShoulder, this.targetShoulder, shoulderAlpha);
      const elbowError = Math.abs(this.targetElbowDeg - this.currentElbowDeg);
      const elbowAlpha = rateLimitedAlpha(
        smoothingAlpha(dtMs, elbowError, 10), elbowError, dtMs
      );
      this.currentElbowDeg += (this.targetElbowDeg - this.currentElbowDeg) * elbowAlpha;
    }
    this.lastSampleAtMs = nowMs;
    let state = this.inputState;
    if (state === "tracking_unavailable" && this.lastValidAtMs !== null &&
        nowMs - this.lastValidAtMs <= this.dropoutHoldMs) {
      state = this.lastLiveState;
    }
    return { shoulder: { ...this.currentShoulder }, elbowDeg: this.currentElbowDeg, state };
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

/** Convert a normalized packet quaternion to CSS rotation angles. */
function shoulderEulerDeg(quaternion) {
  const { qx: x, qy: y, qz: z, qw: w } = quaternion || {};
  if (![x, y, z, w].every(Number.isFinite)) return { x: 0, y: 0, z: 0 };
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const degrees = 180 / Math.PI;
  return {
    x: cleanDegrees(roll * degrees),
    y: cleanDegrees(pitch * degrees),
    z: cleanDegrees(yaw * degrees),
  };
}

export function armPoseForMotion(frame) {
  const shoulderDeg = shoulderEulerDeg(frame?.upper_arm_orientation);
  const health = frame?.health;
  if (!health?.calibrated) return { state: "calibration_required", elbowDeg: 0, shoulderDeg };
  if (!health.n2 || !health.n4 || !health.synchronized) {
    return { state: "tracking_unavailable", elbowDeg: 0, shoulderDeg };
  }
  if (frame.elbow_flexion_deg === null || frame.elbow_flexion_deg === undefined ||
      frame.diagnostics?.flexionValid === false) {
    return { state: "shoulder_live", elbowDeg: 0, shoulderDeg };
  }
  return {
    state: "live",
    elbowDeg: Math.max(-150, Math.min(150, frame.elbow_flexion_deg)),
    shoulderDeg,
  };
}

const LABELS = {
  calibration_required: "Calibrate neutral pose",
  tracking_unavailable: "Live tracking unavailable",
  shoulder_live: "Live shoulder, calibrate elbow hinge",
  live: "Live elbow tracking",
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
        <div class="arm-node-wrap shoulder"><span class="arm-node-dot"></span><span class="arm-node-label">N4 shoulder</span></div>
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
    <div class="arm-avatar-label" data-arm-status>Calibrate neutral pose</div>`;
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
    const matrix = rotationMatrixForQuaternion(pose.shoulder);
    this.root.style.setProperty("--upper-matrix", `matrix3d(${matrix.join(",")})`);
    this.root.style.setProperty("--elbow-deg", `${cleanDegrees(pose.elbowDeg)}deg`);
    this.root.dataset.state = pose.state;
    if (this.status) this.status.textContent = LABELS[pose.state];
  }
}
