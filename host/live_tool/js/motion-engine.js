import { palmRestNormal, solvePointingMount } from "./anatomical-calibration.js";

/**
 * Motion Engine: deterministic body-segment kinematics for Coach Assist.
 *
 * This module is the Milestone 4 deliverable described in
 * "Coach Assist Requirements, Milestone Alignment & Movement Tracking Plan"
 * (section 8, "Immediate Development Sprint"). It is deliberately independent
 * of the ML path: no model, no window assembler, no inference gate. Motion
 * output must keep running when the model path is degraded or switched off,
 * because the 3D/app team consumes it, not the classifier.
 *
 * WHAT IT DOES
 *   1. Keeps the latest BNO085 quaternion + gyro per node.
 *   2. Captures a neutral-pose calibration (hold still ~1.5 s) that maps each
 *      worn PCB orientation to its body segment reference.
 *   3. Emits calibrated segment orientations, elbow relative rotation and
 *      upper-arm deviation at the live rate, plus a health block.
 *
 * FRAME CONVENTION
 * The BNO085 reports the Game Rotation Vector as q = (i, j, k, real), a
 * rotation taking a vector from the SENSOR frame to the sensor's own reference
 * ("world") frame. Internally this module uses (w, x, y, z) ordering; the
 * output packet uses the {qx, qy, qz, qw} field names the plan specifies.
 *
 * THE MOUNT-INDEPENDENCE ARGUMENT (why this is the right math)
 * Let q_n(t) be node n's reported orientation and M_n the unknown, constant
 * rotation from the bone/segment frame into that sensor's frame. The true
 * segment orientation is q_n(t) * M_n.
 *
 * Calibration stores q_ref[n], the averaged quaternion at neutral pose. We
 * then report each segment relative to its own neutral pose:
 *
 *     D_n(t) = conj(q_ref[n]) * q_n(t)
 *
 * The corresponding segment-frame rotation is conj(M_n) * D_n(t) * M_n - a
 * conjugation. Rotation ANGLE is invariant under conjugation, so |D_n(t)| is
 * exactly the angle the segment has swept since neutral, regardless of how the
 * strap was rotated. That is what makes upper_arm_deviation_deg and
 * elbow_relative_rotation_deg trustworthy on the first wear with no anatomical
 * model.
 *
 * The rotation AXIS is not conjugation-invariant: it comes out expressed in
 * the sensor's neutral frame rather than the anatomical frame. So the exported
 * quaternions rotate the avatar by the right AMOUNT about an axis that is off
 * by the residual mount rotation. Resolving that needs a second calibration
 * pose (see mountCorrection below), which the plan schedules after the numeric
 * values are physically validated. The seam is left explicit rather than
 * hidden behind a guessed constant.
 *
 * KNOWN LIMITS - these belong in the handoff note to the 3D team
 *   - SH2_GAME_ROTATION_VECTOR is magnetometer-free (see
 *     firmware/common/inc/exo/sensors/bno85_stm32.h). Each sensor's yaw is
 *     arbitrary at power-up and drifts independently. Calibration cancels the
 *     inter-sensor yaw offset at t0; it does not stop it drifting afterwards.
 *     Elbow flexion largely lives on the swing axis and is robust; absolute
 *     upper-arm heading is not. yawDriftHintDeg reports the observable part.
 *   - There is no torso reference in the current arm-only rig, so
 *     upper_arm_deviation_deg is CALIBRATION-POSE-relative, not trunk-relative,
 *     and degrades if the wearer rotates their torso mid-rep. The final suit
 *     adds lower-back and neck nodes plus a chest Master; torso is the
 *     placeholder for that and is intentionally unused today.
 */

/** Node roles for the current arm-only rig (plan section 5). */
export const MOTION_ROLES = {
  upperArm: 4,
  forearm: 2,
  aux: 3,
  /** Reserved: lower-back / neck / chest-Master reference in the full suit. */
  torso: null,
};

export const MOTION_DEFAULTS = {
  /**
   * Neutral pose hold: a sliding window this long must be quiet. Every sample
   * joins the window and old samples age out, so a wobble no longer voids the
   * whole hold (07:45 field log: five "too much motion" restarts at 1.0 s).
   */
  holdSeconds: 3.0,
  /** Reject calibration if any node's gyro magnitude exceeds this (rad/s). */
  stillGyroMaxRadps: 0.2,
  /** Reject calibration if a node's orientation spread exceeds this (deg). */
  stillSpreadMaxDeg: 3.0,
  /** A node older than this is not fresh. */
  staleMaxMs: 250,
  /** Max pairwise device-time skew for a frame to count as synchronized. */
  syncSkewMaxMs: 60,
  /** Calibration aborts if stillness cannot be achieved within this. */
  calibrationTimeoutMs: 20000,

  // --- anatomical frame calibration: two held straight-arm directions ---
  /**
   * The mount is solved from where the arm POINTED during each hold (see
   * solvePointingMount), so palm orientation during the holds is free. The
   * angles below are elevations of the arm above hanging straight down.
   */
  anatomicalHoldSeconds: 3.0,
  anatomicalMinAngleDeg: 45,
  anatomicalMaxAngleDeg: 135,
  /**
   * Straight-elbow check: N4 and N2 elevations must agree. Elevation is free
   * of mount, heading AND twist, unlike rotation magnitude, which also grows
   * when the forearm alone turns the palm (a legitimate difference).
   */
  anatomicalMaxSegmentMismatchDeg: 15,
  anatomicalTimeoutMs: 25000,
  /**
   * Side and forward holds are ideally 90 degrees apart. The solve splits any
   * difference evenly, so each held direction renders within half of it.
   * Above the hint the wearer is told; above the max the forward hold is
   * refused and repeated (the side capture is kept).
   */
  anatomicalDisagreementHintDeg: 15,
  anatomicalMaxDisagreementDeg: 30,

  // --- hinge (range) calibration: a few slow reps to find the joint axis ---
  /** Only frames past this flexion contribute; small rotations have noisy axes. */
  hingeMinAngleDeg: 40,
  /** Samples needed before the axis is accepted (~2 reps at 25 Hz). */
  hingeMinSamples: 40,
  /** Reject the axis if the observed rotation axes scatter more than this. */
  hingeMaxSpreadDeg: 15,
  /** Hinge capture aborts if it cannot be satisfied within this. */
  hingeTimeoutMs: 30000,

  // --- validity gate and drift monitor ---
  /**
   * Withhold signed flexion once the motion departs this far from the hinge.
   * Swing-twist about a fixed axis is only meaningful while the movement is
   * roughly about that axis; past that the twist term becomes unstable. In the
   * 2026-09-10T06:02 random-movement session, 26% of frames reported
   * |flexion| > 150 deg — impossible for an elbow — at a median off-axis of
   * 74 deg. A withheld value is far better than a confident wrong one.
   */
  flexionMaxOffAxisDeg: 35,
  /**
   * Anatomical plausibility limit on flexion magnitude. The off-axis gate alone
   * is not sufficient: replaying the 06:02 session, 41 frames still reported
   * |flexion| > 150 deg while sitting at only 25-34 deg off-axis, because the
   * twist term can approach +/-180 whenever the quaternion's scalar part nears
   * zero. A human elbow tops out near 145-150 deg; the clean sets peaked at
   * 137-140 deg, so this rejects every impossible frame while keeping all real
   * ones.
   */
  flexionMaxPlausibleDeg: 150,
  /** Continuous stillness needed before a frame counts as a rest observation. */
  restStillMs: 400,
  /** A rest only informs drift when the arm is back near the calibration pose. */
  restMaxFlexionDeg: 20,
  /** Recommend recalibration once the drift estimate passes this. */
  driftWarnDeg: 15,
};

// ------------------------------------------------------------------ quaternion
// All helpers take and return [w, x, y, z] and assume nothing about handedness
// beyond the Hamilton product.

export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (!(n > 1e-9)) return null;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function quatConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatMultiply(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

/**
 * Rotate a 3-vector by a quaternion: v' = q v q^-1. Used to carry the BNO's
 * body-frame gyro into the delta frame the display extrapolates in.
 */
export function rotateVector(v, q) {
  if (!v || !q) return null;
  const [qw, qx, qy, qz] = q;
  const tx = 2 * (qy * v[2] - qz * v[1]);
  const ty = 2 * (qz * v[0] - qx * v[2]);
  const tz = 2 * (qx * v[1] - qy * v[0]);
  return [
    v[0] + qw * tx + qy * tz - qz * ty,
    v[1] + qw * ty + qz * tx - qx * tz,
    v[2] + qw * tz + qx * ty - qy * tx,
  ];
}

/**
 * Rotation angle of a unit quaternion, in degrees, folded into [0, 180].
 * q and -q are the same rotation, so the sign of w is normalized away first.
 */
export function quatAngleDeg(q) {
  if (!q) return null;
  const unit = quatNormalize(q);
  if (!unit) return null;
  const w = Math.min(Math.abs(unit[0]), 1);
  return (2 * Math.acos(w) * 180) / Math.PI;
}

/**
 * Unit rotation axis of a quaternion, or null when the rotation is too small
 * for the axis to be meaningful. Sign is taken with w >= 0 so the axis pairs
 * with the [0, 180] angle above.
 */
export function quatAxis(q) {
  if (!q) return null;
  const unit = quatNormalize(q);
  if (!unit) return null;
  const signed = unit[0] < 0 ? [-unit[0], -unit[1], -unit[2], -unit[3]] : unit;
  const s = Math.hypot(signed[1], signed[2], signed[3]);
  if (s < 1e-6) return null;
  return [signed[1] / s, signed[2] / s, signed[3] / s];
}

/**
 * Mean of a set of near-identical unit quaternions. Each sample is sign-aligned
 * to the first (q and -q are the same rotation, and an unaligned sum cancels),
 * then the arithmetic mean is renormalized. Exact enough for the few-degree
 * spread a stillness gate admits; the full eigenvector method buys nothing here.
 */
export function quatAverage(samples) {
  if (!samples || samples.length === 0) return null;
  const first = quatNormalize(samples[0]);
  if (!first) return null;
  const acc = [0, 0, 0, 0];
  for (const sample of samples) {
    const unit = quatNormalize(sample);
    if (!unit) return null;
    const dot = unit[0] * first[0] + unit[1] * first[1] + unit[2] * first[2] + unit[3] * first[3];
    const sign = dot < 0 ? -1 : 1;
    for (let i = 0; i < 4; i += 1) acc[i] += sign * unit[i];
  }
  for (let i = 0; i < 4; i += 1) acc[i] /= samples.length;
  return quatNormalize(acc);
}

/** Largest angular distance from `mean` across `samples`, in degrees. */
export function quatSpreadDeg(samples, mean) {
  let worst = 0;
  for (const sample of samples) {
    const unit = quatNormalize(sample);
    if (!unit) return Infinity;
    const angle = quatAngleDeg(quatMultiply(quatConjugate(mean), unit));
    if (angle === null) return Infinity;
    if (angle > worst) worst = angle;
  }
  return worst;
}

/** BNO085 (i, j, k, real) as delivered by the live decoder -> [w, x, y, z]. */
export function quatFromBnoValues(values) {
  return [values.quat_real, values.quat_i, values.quat_j, values.quat_k];
}

/** [w, x, y, z] -> the {qx, qy, qz, qw} shape the plan's data contract names. */
export function quatToPacket(q) {
  return { qx: q[1], qy: q[2], qz: q[3], qw: q[0] };
}

/**
 * Swing-twist decomposition of `q` about unit axis `h`.
 *
 * Splits a rotation into the part that happens ABOUT the hinge (twist, signed,
 * this is flexion) and everything left over (swing, unsigned, this is how far
 * the motion departed from being a clean hinge rotation).
 *
 * Why this matters here: the plain rotation angle folds flexion together with
 * forearm rotation and any off-axis motion into one unsigned number in
 * [0, 180]. Session 2026-09-10T04:31 hit 179.9 deg on a compound movement — at
 * the fold limit, past which readings wrap back DOWN and a worse movement reads
 * as a smaller angle. Projecting onto a measured hinge axis removes that
 * failure and yields a real joint angle plus an off-axis quality signal.
 *
 * Returns { twistDeg, swingDeg } with twistDeg signed in (-180, 180].
 */
export function swingTwistDeg(q, axis) {
  const unit = quatNormalize(q);
  if (!unit || !axis) return null;
  const [w, x, y, z] = unit;
  const projection = x * axis[0] + y * axis[1] + z * axis[2];
  const twistRaw = [w, projection * axis[0], projection * axis[1], projection * axis[2]];
  const twistNorm = Math.hypot(w, projection);
  // Degenerate when the rotation is a half turn perpendicular to the hinge:
  // the twist component vanishes and its angle is undefined.
  if (twistNorm < 1e-6) return { twistDeg: 0, swingDeg: 180 };
  const twist = quatNormalize(twistRaw);
  let twistDeg = (2 * Math.atan2(projection, w) * 180) / Math.PI;
  // atan2 gives (-360, 360] after doubling; fold to (-180, 180].
  if (twistDeg > 180) twistDeg -= 360;
  if (twistDeg <= -180) twistDeg += 360;
  const swing = quatMultiply(unit, quatConjugate(twist));
  return { twistDeg, swingDeg: quatAngleDeg(swing) };
}

/**
 * Mean of unit axes, sign-aligned to the first. A rotation and its inverse
 * report opposite axes, so an unaligned mean of a hinge sweep cancels toward
 * zero. Returns { axis, meanSpreadDeg, maxSpreadDeg } or null.
 */
export function averageAxis(axes) {
  if (!axes || axes.length === 0) return null;
  const first = axes[0];
  const acc = [0, 0, 0];
  for (const a of axes) {
    const sign = a[0] * first[0] + a[1] * first[1] + a[2] * first[2] < 0 ? -1 : 1;
    for (let i = 0; i < 3; i += 1) acc[i] += sign * a[i];
  }
  const norm = Math.hypot(acc[0], acc[1], acc[2]);
  if (norm < 1e-9) return null;
  const axis = [acc[0] / norm, acc[1] / norm, acc[2] / norm];
  let total = 0;
  let worst = 0;
  for (const a of axes) {
    const dot = Math.abs(a[0] * axis[0] + a[1] * axis[1] + a[2] * axis[2]);
    const deviation = (Math.acos(Math.min(Math.max(dot, -1), 1)) * 180) / Math.PI;
    total += deviation;
    if (deviation > worst) worst = deviation;
  }
  return { axis, meanSpreadDeg: total / axes.length, maxSpreadDeg: worst };
}

// --------------------------------------------------------------------- engine

export const CAL_STATE = {
  UNCALIBRATED: "uncalibrated",
  CAPTURING: "capturing",
  CALIBRATED: "calibrated",
  FAILED: "failed",
};

export const HINGE_STATE = {
  NONE: "none",
  CAPTURING: "capturing",
  READY: "ready",
  FAILED: "failed",
};

export const ANATOMICAL_STATE = {
  NONE: "none",
  SIDE_CAPTURING: "side_capturing",
  SIDE_READY: "side_ready",
  FORWARD_CAPTURING: "forward_capturing",
  CALIBRATED: "calibrated",
  FAILED: "failed",
};

export class MotionEngine {
  constructor(options = {}) {
    this.options = { ...MOTION_DEFAULTS, ...options };
    this.roles = { ...MOTION_ROLES, ...(options.roles || {}) };
    /** Nodes that must be healthy for a motion frame. Aux is advisory only. */
    this.requiredNodes = [this.roles.upperArm, this.roles.forearm];
    this.trackedNodes = [this.roles.upperArm, this.roles.forearm, this.roles.aux].filter(
      (id) => id !== null && id !== undefined
    );

    /** nodeId -> { quat, gyroMag, deviceS, receivedAtMs } */
    this.latest = new Map();
    /** nodeId -> averaged neutral-pose quaternion. */
    this.reference = new Map();
    /**
     * Seam for the two-pose calibration the plan schedules after numeric
     * validation: nodeId -> mount rotation M_n, applied as conj(M) * D * M so
     * exported axes land in the anatomical frame. Empty until then.
     */
    this.mountCorrection = new Map();
    this.anatomicalState = ANATOMICAL_STATE.NONE;
    this.anatomicalMessage = "Capture a neutral pose first.";
    this.anatomicalQuality = null;
    this.anatomicalCapture = null;
    this.anatomicalSide = null;
    this.anatomicalSideAnglesDeg = null;
    this.anatomicalWarning = null;

    this.state = CAL_STATE.UNCALIBRATED;
    this.calibrationMessage = "Not calibrated.";
    this.calibratedAtMs = null;
    this.capture = null;
    this.neutralElbow = null;
    this.lastFrame = null;

    /**
     * Measured elbow hinge axis, in the frame the relative rotation lives in.
     * Null until a range calibration runs; signed flexion is unavailable until
     * then and the unsigned angle is all the packet carries.
     */
    this.hingeAxis = null;
    /**
     * Per node: world vertical at the neutral pose, in that node's own sensor
     * frame. The GRV world frame is Z-up; on the 09:10 field log this agreed
     * with the BNO's separate gravity report to 0.01 degrees on both nodes.
     */
    this.neutralDown = new Map();
    /**
     * The wearer's neutral palm direction in anatomical axes (from the side
     * hold, palm down). Rendering-only: the avatar turns its hand to match.
     */
    this.handRestPalm = null;
    this.hingeState = HINGE_STATE.NONE;
    this.hingeMessage = "No hinge axis.";
    this.hingeQuality = null;
    this.hingeCapture = null;

    /**
     * Drift monitor. The BNO085's magnetometer-free heading means N2 and N4
     * drift apart over a session. Measured on 2026-09-10T05:59 (60 s of held
     * stillness): the relative rotation grew +9.9 deg/min, almost all of it
     * perpendicular to the hinge, while each segment's own deviation stayed
     * flat at +0.03 deg/min. So the hinge projection protects flexion
     * (-4.1 deg/min) but the off-axis term absorbs the drift, and any fixed
     * off-axis threshold would decay over a session if left uncorrected.
     */
    this.restElbow = null;
    this.driftDeg = 0;
    this.stillSinceMs = null;

    this.onEvent = options.onEvent || (() => {});
  }

  // -------------------------------------------------------------- ingestion

  /**
   * Feed one decoded BNO sample. ICM rows carry no orientation and are ignored
   * here; they stay useful to the charts and the model path.
   * `deviceS` is the Master-mapped timebase used for skew, and arrival time is
   * taken locally so a stalled node is detectable without a device clock.
   */
  pushSample(nodeId, values, deviceS, nowMs = performance.now()) {
    if (!this.trackedNodes.includes(nodeId)) return false;
    if (!values || values.quat_real === undefined) return false;
    const quat = quatNormalize(quatFromBnoValues(values));
    if (!quat) return false;
    const gyro = [
      values.gyro_x_radps || 0,
      values.gyro_y_radps || 0,
      values.gyro_z_radps || 0,
    ];
    const gyroMag = Math.hypot(gyro[0], gyro[1], gyro[2]);
    this.latest.set(nodeId, { quat, gyro, gyroMag, deviceS, receivedAtMs: nowMs });
    if (this.state === CAL_STATE.CAPTURING) this.accumulateCalibration(nodeId, quat, gyroMag, nowMs);
    if (this.anatomicalCapture && this.requiredNodes.includes(nodeId)) {
      this.accumulateAnatomicalCalibration(nodeId, quat, gyroMag, nowMs);
    }
    return true;
  }

  // ------------------------------------------------------------ calibration

  /**
   * Begin a neutral-pose capture. The wearer holds the calibration posture;
   * the gate below decides when enough still data has accumulated.
   */
  beginCalibration(nowMs = performance.now()) {
    this.clearAnatomicalCalibration();
    this.clearHinge();
    this.restElbow = null;
    this.driftDeg = 0;
    this.stillSinceMs = null;
    this.state = CAL_STATE.CAPTURING;
    this.calibrationMessage = "Stand relaxed - arms hanging, palms facing your thighs.";
    this.capture = {
      startedAtMs: nowMs,
      /**
       * Sliding window: every sample joins, and only samples older than
       * holdSeconds age out. The mean is always taken over the freshest
       * window, so one wobble does not invalidate the whole hold.
       */
      samples: new Map(this.trackedNodes.map((id) => [id, []])),
      lastReason: null,
      lastReasonText: null,
    };
    this.onEvent({ kind: "calibration_started" });
    return true;
  }

  cancelCalibration(reason = "cancelled") {
    if (this.state !== CAL_STATE.CAPTURING) return;
    this.capture = null;
    this.state = this.reference.size > 0 ? CAL_STATE.CALIBRATED : CAL_STATE.UNCALIBRATED;
    this.calibrationMessage = `Calibration ${reason}.`;
    this.onEvent({ kind: "calibration_cancelled", reason });
  }

  clearCalibration() {
    this.reference.clear();
    this.neutralDown.clear();
    this.neutralElbow = null;
    this.calibratedAtMs = null;
    this.capture = null;
    this.state = CAL_STATE.UNCALIBRATED;
    this.calibrationMessage = "Not calibrated.";
    this.clearAnatomicalCalibration();
    // The hinge axis is expressed relative to the neutral reference, so a new
    // neutral pose invalidates it. Keeping it would silently report flexion
    // about a stale axis.
    this.clearHinge();
    this.restElbow = null;
    this.driftDeg = 0;
    this.stillSinceMs = null;
    this.onEvent({ kind: "calibration_cleared" });
  }

  /** Update the hold instruction; the sliding window is never discarded. */
  holdMessage(message, reason = null) {
    if (!this.capture) return;
    if (reason) this.capture.lastReason = reason;
    this.capture.lastReasonText = message;
    this.calibrationMessage = message;
  }

  /**
   * Every still-or-not sample joins the sliding window. Motion control lives
   * in updateCalibration, which measures the orientation spread over the
   * freshest window and simply waits for it to settle.
   */
  accumulateCalibration(nodeId, quat, gyroMag, nowMs) {
    if (!this.capture) return;
    this.capture.samples.get(nodeId).push({ quat, gyroMag, nowMs });
  }

  /**
   * Called from the render tick. Evaluates the freshest holdSeconds of data:
   * when it is long enough, complete across the required nodes, and tight
   * enough, the mean becomes the neutral reference.
   */
  updateCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CAPTURING) return;
    const capture = this.capture;

    if (nowMs - capture.startedAtMs > this.options.calibrationTimeoutMs) {
      this.state = CAL_STATE.FAILED;
      this.capture = null;
      this.calibrationMessage = capture.lastReasonText
        ? `Timed out: ${capture.lastReasonText.replace(/\.$/, "")}.`
        : "Timed out waiting for a still neutral pose.";
      this.onEvent({ kind: "calibration_failed", reason: this.calibrationMessage });
      return;
    }

    const windowMs = this.options.holdSeconds * 1000;
    if (nowMs - capture.startedAtMs < windowMs) {
      this.holdMessage(`Hold still - keeping a ${(windowMs / 1000).toFixed(0)} s window.`);
      return;
    }
    const cutoff = nowMs - windowMs;
    for (const list of capture.samples.values()) {
      while (list.length > 0 && list[0].nowMs < cutoff) list.shift();
    }

    // Every required node must be fresh AND contributing to this window.
    for (const nodeId of this.requiredNodes) {
      const entry = this.latest.get(nodeId);
      if (!entry || nowMs - entry.receivedAtMs > this.options.staleMaxMs) {
        this.holdMessage(`Waiting for N${nodeId} data.`, "stale");
        return;
      }
      if (capture.samples.get(nodeId).length < 4) {
        this.calibrationMessage = `Waiting for N${nodeId} samples.`;
        return;
      }
    }

    const reference = new Map();
    for (const nodeId of this.trackedNodes) {
      const list = capture.samples.get(nodeId);
      // The aux node may legitimately lag; only required nodes gate above.
      if (!list || list.length < 4) continue;
      const quats = list.map((sample) => sample.quat);
      const mean = quatAverage(quats);
      if (!mean) {
        this.holdMessage(`Degenerate quaternion on N${nodeId}.`, "degenerate");
        return;
      }
      const spread = quatSpreadDeg(quats, mean);
      const restless = list.filter((sample) => sample.gyroMag > this.options.stillGyroMaxRadps).length;
      if (spread > this.options.stillSpreadMaxDeg || restless > list.length * 0.2) {
        this.holdMessage("Pose was not steady - too much motion, keep holding.", "spread");
        return;
      }
      reference.set(nodeId, mean);
    }

    this.reference = reference;
    this.neutralDown = new Map(
      [...reference].map(([id, q]) => [id, rotateVector([0, 0, -1], quatConjugate(q))])
    );
    // Neutral inter-segment rotation, subtracted out so a straight arm reads 0
    // no matter how the two PCBs happen to sit relative to each other.
    this.neutralElbow = this.rawElbowRelative(
      reference.get(this.roles.upperArm),
      reference.get(this.roles.forearm)
    );
    this.state = CAL_STATE.CALIBRATED;
    this.calibratedAtMs = nowMs;
    this.capture = null;
    this.calibrationMessage = `Neutral calibrated from ${(windowMs / 1000).toFixed(1)} s of still data.`;
    this.anatomicalMessage = "Next: hold a straight-arm raise to your right side.";
    this.onEvent({
      kind: "calibration_complete",
      nodes: [...reference.keys()],
      heldMs: Math.round(windowMs),
    });
  }

  // ------------------------------------------------ anatomical calibration

  clearAnatomicalCalibration() {
    this.mountCorrection.clear();
    this.anatomicalState = ANATOMICAL_STATE.NONE;
    this.anatomicalMessage =
      this.state === CAL_STATE.CALIBRATED
        ? "Next: hold a straight-arm raise to your right side."
        : "Capture a neutral pose first.";
    this.anatomicalQuality = null;
    this.anatomicalCapture = null;
    this.anatomicalSide = null;
    this.anatomicalSideAnglesDeg = null;
    this.anatomicalWarning = null;
    this.handRestPalm = null;
  }

  beginSideCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CALIBRATED) {
      this.anatomicalState = ANATOMICAL_STATE.FAILED;
      this.anatomicalMessage = "Capture a neutral pose first.";
      return false;
    }
    this.mountCorrection.clear();
    this.anatomicalSide = null;
    this.anatomicalSideAnglesDeg = null;
    this.anatomicalQuality = null;
    this.anatomicalWarning = null;
    this.handRestPalm = null;
    this.startAnatomicalCapture("side", nowMs);
    this.onEvent({ kind: "anatomical_side_started" });
    return true;
  }

  beginForwardCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CALIBRATED) {
      this.anatomicalState = ANATOMICAL_STATE.FAILED;
      this.anatomicalMessage = "Capture a neutral pose first.";
      return false;
    }
    if (!this.anatomicalSide) {
      this.anatomicalState = ANATOMICAL_STATE.FAILED;
      this.anatomicalMessage = "Capture the right-side raise first.";
      return false;
    }
    this.startAnatomicalCapture("forward", nowMs);
    this.onEvent({ kind: "anatomical_forward_started" });
    return true;
  }

  startAnatomicalCapture(kind, nowMs) {
    this.anatomicalState =
      kind === "side" ? ANATOMICAL_STATE.SIDE_CAPTURING : ANATOMICAL_STATE.FORWARD_CAPTURING;
    // The solve uses where the arm points, so the palm is free in both holds.
    // "Palm down" on the side hold is not a twist constraint: it is the one
    // pose that tells the avatar which way the wearer's palm faces.
    this.anatomicalMessage =
      kind === "side"
        ? "Point your straight arm out to your right side at shoulder height, palm facing the floor."
        : "Point your straight arm straight ahead at shoulder height.";
    this.anatomicalCapture = {
      kind,
      startedAtMs: nowMs,
      samples: new Map(this.requiredNodes.map((id) => [id, []])),
      lastRejectCode: null,
    };
  }

  /**
   * Update the capture instruction and audit the reason once per code. The
   * sliding window is never discarded: a wobble or one loose sample ages out
   * of the window instead of restarting the whole hold.
   */
  anatomicalWait(message, code = "unspecified") {
    const capture = this.anatomicalCapture;
    if (!capture) return;
    if (code !== capture.lastRejectCode) {
      this.onEvent({
        kind: "anatomical_hold_restarted",
        stage: capture.kind,
        code,
        reason: message,
      });
    }
    capture.lastRejectCode = code;
    this.anatomicalMessage = message;
  }

  accumulateAnatomicalCalibration(nodeId, quat, gyroMag, nowMs) {
    const capture = this.anatomicalCapture;
    const reference = this.reference.get(nodeId);
    if (!capture || !reference) return;
    capture.samples.get(nodeId).push({
      quat: quatMultiply(quatConjugate(reference), quat),
      gyroMag,
      nowMs,
    });
  }

  failAnatomicalCalibration(message, details = null) {
    this.anatomicalState = ANATOMICAL_STATE.FAILED;
    this.anatomicalMessage = message;
    this.anatomicalCapture = null;
    this.mountCorrection.clear();
    this.anatomicalQuality = null;
    this.onEvent({ kind: "anatomical_failed", reason: message, ...(details ?? {}) });
  }

  updateAnatomicalCalibration(nowMs = performance.now()) {
    const capture = this.anatomicalCapture;
    if (!capture) return;
    if (nowMs - capture.startedAtMs > this.options.anatomicalTimeoutMs) {
      this.failAnatomicalCalibration(
        capture.lastRejectCode
          ? `Timed out: ${this.anatomicalMessage.replace(/\.$/, "")}.`
          : `Timed out waiting for the ${capture.kind} pose.`
      );
      return;
    }

    const windowMs = this.options.anatomicalHoldSeconds * 1000;
    if (nowMs - capture.startedAtMs < windowMs) {
      this.anatomicalWait(
        `Hold the ${capture.kind} pose - measuring a ${(windowMs / 1000).toFixed(0)} s window.`,
        "holding"
      );
      return;
    }
    const cutoff = nowMs - windowMs;
    for (const list of capture.samples.values()) {
      while (list.length > 0 && list[0].nowMs < cutoff) list.shift();
    }

    const deviceTimes = [];
    for (const nodeId of this.requiredNodes) {
      const entry = this.latest.get(nodeId);
      if (!entry || nowMs - entry.receivedAtMs > this.options.staleMaxMs) {
        this.anatomicalWait(`Waiting for N${nodeId} data.`, "stale");
        return;
      }
      deviceTimes.push(entry.deviceS);
      if (capture.samples.get(nodeId).length < 4) {
        this.anatomicalMessage = `Waiting for N${nodeId} samples.`;
        return;
      }
    }
    const skewMs = (Math.max(...deviceTimes) - Math.min(...deviceTimes)) * 1000;
    if (Math.abs(skewMs) > this.options.syncSkewMaxMs) {
      this.anatomicalWait("Sensor clock skew - samples are not synchronized.", "sync");
      return;
    }

    const captured = new Map();
    for (const nodeId of this.requiredNodes) {
      const list = capture.samples.get(nodeId);
      const quats = list.map((sample) => sample.quat);
      const mean = quatAverage(quats);
      const spreadDeg = mean ? quatSpreadDeg(quats, mean) : Infinity;
      const restless = list.filter((sample) => sample.gyroMag > this.options.stillGyroMaxRadps).length;
      if (!mean || spreadDeg > this.options.stillSpreadMaxDeg || restless > list.length * 0.2) {
        this.anatomicalWait("Pose was not steady - too much motion, keep holding.", "spread");
        return;
      }
      const pointing = this.pointingOf(nodeId, mean);
      if (!pointing) {
        this.anatomicalWait(`N${nodeId} pointing direction unreadable.`, "degenerate");
        return;
      }
      captured.set(nodeId, { mean, spreadDeg, ...pointing });
    }

    // Straight elbow first: a bent side raise leaves N2 well below N4, and
    // answering that with "raise higher" coaches the wrong correction.
    const elevations = this.requiredNodes.map((id) => captured.get(id).elevationDeg);
    const mismatchDeg = Math.abs(elevations[0] - elevations[1]);
    if (mismatchDeg > this.options.anatomicalMaxSegmentMismatchDeg) {
      this.anatomicalWait(
        `Keep the elbow straight; upper arm and forearm point ${mismatchDeg.toFixed(0)} degrees apart.`,
        "mismatch"
      );
      return;
    }

    for (const nodeId of this.requiredNodes) {
      const { elevationDeg } = captured.get(nodeId);
      if (
        elevationDeg < this.options.anatomicalMinAngleDeg ||
        elevationDeg > this.options.anatomicalMaxAngleDeg
      ) {
        const message =
          `Arm is ${elevationDeg.toFixed(0)} degrees up from hanging - ` +
          `hold it between ${this.options.anatomicalMinAngleDeg} and ${this.options.anatomicalMaxAngleDeg} (shoulder height is 90).`;
        this.anatomicalWait(message, "raise_range");
        return;
      }
    }

    // NOTE: do NOT gate this capture on an inter-sensor quantity such as an
    // elbow-relative rotation conj(q_upper)*q_fore. Each BNO085 Game Rotation
    // Vector carries its own arbitrary power-up heading, and once the upper arm
    // rotates the N2-vs-N4 heading offset leaks straight into the product
    // (session 2026-09-11T04:44). Everything above is per node.

    const summarize = (map) => Object.fromEntries(
      [...map].map(([id, value]) => [id, {
        elevationDeg: value.elevationDeg,
        twistDeg: value.twistDeg,
        spreadDeg: value.spreadDeg,
      }])
    );

    if (capture.kind === "side") {
      this.anatomicalSide = captured;
      this.anatomicalCapture = null;
      this.anatomicalState = ANATOMICAL_STATE.SIDE_READY;
      this.anatomicalSideAnglesDeg = Object.fromEntries(
        [...captured].map(([id, value]) => [id, value.elevationDeg])
      );
      const summary = Object.entries(this.anatomicalSideAnglesDeg)
        .map(([id, deg]) => `N${id} ${deg.toFixed(0)}°`)
        .join(" · ");
      this.anatomicalMessage =
        `Side pose captured (${summary} up; 90° is shoulder height). Next: point straight ahead.`;
      this.onEvent({
        kind: "anatomical_side_complete",
        anglesDeg: { ...this.anatomicalSideAnglesDeg },
        nodes: summarize(captured),
      });
      return;
    }

    const solved = new Map();
    const quality = {};
    for (const nodeId of this.requiredNodes) {
      const side = this.anatomicalSide.get(nodeId);
      const forward = captured.get(nodeId);
      const result = solvePointingMount(
        this.neutralDown.get(nodeId),
        side.pointing,
        forward.pointing,
        {
          minElevationDeg: this.options.anatomicalMinAngleDeg,
          maxElevationDeg: this.options.anatomicalMaxAngleDeg,
          maxDisagreementDeg: this.options.anatomicalMaxDisagreementDeg,
        }
      );
      if (!result.ok) {
        if (result.reason !== "raises_not_perpendicular") {
          this.failAnatomicalCalibration(
            `Could not solve anatomical mapping for N${nodeId} (${result.reason}).`
          );
          return;
        }
        // Keep the accepted side capture and send the wearer back to the
        // forward step with the measured angle: a retry is one raise away.
        const separationsDeg = {};
        for (const id of this.requiredNodes) {
          separationsDeg[id] = this.pointingSeparationDeg(
            id, this.anatomicalSide.get(id).pointing, captured.get(id).pointing
          );
        }
        const measured = Math.min(...Object.values(separationsDeg)).toFixed(0);
        const message =
          `Side and forward holds pointed ${measured} degrees apart (90 is ideal). ` +
          "Point straight ahead of your shoulder, not diagonally, and hold again.";
        this.anatomicalCapture = null;
        this.mountCorrection.clear();
        this.anatomicalQuality = null;
        this.anatomicalState = ANATOMICAL_STATE.SIDE_READY;
        this.anatomicalMessage = message;
        this.onEvent({
          kind: "anatomical_forward_rejected",
          code: "raises_not_perpendicular",
          reason: message,
          separationsDeg,
        });
        return;
      }
      solved.set(nodeId, result.mount);
      quality[nodeId] = {
        ...result.quality,
        sideAngleDeg: side.elevationDeg,
        forwardAngleDeg: forward.elevationDeg,
        sideTwistDeg: side.twistDeg,
        forwardTwistDeg: forward.twistDeg,
        sideSpreadDeg: side.spreadDeg,
        forwardSpreadDeg: forward.spreadDeg,
      };
    }

    this.mountCorrection = solved;
    this.anatomicalQuality = { nodes: quality, segmentMismatchDeg: mismatchDeg, method: "pointing" };
    const forearmMount = solved.get(this.roles.forearm);
    const sideForearm = this.anatomicalSide.get(this.roles.forearm).mean;
    this.handRestPalm = palmRestNormal(
      quatMultiply(quatMultiply(quatConjugate(forearmMount), sideForearm), forearmMount)
    );
    const worstDisagreementDeg = Math.max(
      ...this.requiredNodes.map((id) => quality[id].disagreementDeg)
    );
    this.anatomicalWarning =
      worstDisagreementDeg > this.options.anatomicalDisagreementHintDeg
        ? `Side and forward holds were ${worstDisagreementDeg.toFixed(0)} degrees off a right angle; ` +
          `each renders within ${(worstDisagreementDeg / 2).toFixed(0)} degrees. Redo them for a tighter fit.`
        : null;
    this.anatomicalCapture = null;
    this.anatomicalState = ANATOMICAL_STATE.CALIBRATED;
    this.anatomicalMessage = this.anatomicalWarning
      ? `Anatomical axes calibrated. ${this.anatomicalWarning}`
      : "Anatomical axes calibrated.";
    this.onEvent({
      kind: "anatomical_complete",
      method: "pointing",
      mounts: Object.fromEntries(solved),
      quality: this.anatomicalQuality,
      handRestPalm: this.handRestPalm,
      warning: this.anatomicalWarning,
    });
    // Same-session continuation (2026-09-11 protocol): the wearer is already
    // in the forward pose, so the hinge curl phase starts straight away.
    this.beginHingeCalibration(nowMs);
  }

  /**
   * Where node's long axis points for a sensor-neutral delta, plus how far the
   * segment has twisted about its own axis. Both are in the node's own neutral
   * frame, so mount and heading cancel; twist is logged, never used to solve.
   */
  pointingOf(nodeId, delta) {
    const down = this.neutralDown.get(nodeId);
    if (!down || !delta) return null;
    const pointing = rotateVector(down, delta);
    const cos = Math.max(-1, Math.min(1,
      pointing[0] * down[0] + pointing[1] * down[1] + pointing[2] * down[2]));
    const twist = swingTwistDeg(delta, down);
    return {
      pointing,
      elevationDeg: (Math.acos(cos) * 180) / Math.PI,
      twistDeg: twist ? twist.twistDeg : null,
    };
  }

  /** Horizontal angle between two pointing directions of the same node. */
  pointingSeparationDeg(nodeId, a, b) {
    const down = this.neutralDown.get(nodeId);
    if (!down || !a || !b) return null;
    const flat = (v) => {
      const d = v[0] * down[0] + v[1] * down[1] + v[2] * down[2];
      const h = [v[0] - d * down[0], v[1] - d * down[1], v[2] - d * down[2]];
      const n = Math.hypot(h[0], h[1], h[2]);
      return n > 1e-6 ? [h[0] / n, h[1] / n, h[2] / n] : null;
    };
    const fa = flat(a);
    const fb = flat(b);
    if (!fa || !fb) return null;
    const cos = Math.max(-1, Math.min(1, fa[0] * fb[0] + fa[1] * fb[1] + fa[2] * fb[2]));
    return (Math.acos(cos) * 180) / Math.PI;
  }

  // ------------------------------------------------------- hinge calibration

  /**
   * Start a range calibration: the wearer performs a few slow, deliberate reps
   * and the engine measures the joint's actual rotation axis from them. This is
   * what turns the unsigned composite angle into a real signed flexion angle.
   *
   * Requires a neutral calibration first — the axis is measured in the frame
   * the neutral reference establishes, so clearing neutral invalidates it.
   */
  beginHingeCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CALIBRATED) {
      this.hingeState = HINGE_STATE.FAILED;
      this.hingeMessage = "Capture a neutral pose first.";
      return false;
    }
    this.hingeState = HINGE_STATE.CAPTURING;
    this.hingeMessage = "Palm up, keep the upper arm steady - do 3 slow full curls.";
    this.hingeCapture = { startedAtMs: nowMs, axes: [], quats: [], peakElbowDeg: 0 };
    this.onEvent({ kind: "hinge_started" });
    return true;
  }

  clearHinge() {
    this.hingeAxis = null;
    this.hingeQuality = null;
    this.hingeCapture = null;
    this.hingeState = HINGE_STATE.NONE;
    this.hingeMessage = "No hinge axis.";
  }

  /**
   * Feed one elbow rotation into the hinge capture and decide whether the axis
   * can be accepted yet. Only well-flexed frames count: near the neutral pose
   * the rotation is tiny and its axis is mostly noise.
   */
  accumulateHinge(elbowQuat, nowMs) {
    const capture = this.hingeCapture;
    if (!capture) return;
    const angle = quatAngleDeg(elbowQuat);
    if (angle !== null && angle > capture.peakElbowDeg) capture.peakElbowDeg = angle;
    if (angle !== null && angle >= this.options.hingeMinAngleDeg) {
      const axis = quatAxis(elbowQuat);
      if (axis) {
        capture.axes.push(axis);
        capture.quats.push(elbowQuat);
      }
    }

    if (capture.axes.length >= this.options.hingeMinSamples) {
      const result = averageAxis(capture.axes);
      if (result && result.meanSpreadDeg <= this.options.hingeMaxSpreadDeg) {
        // Orient the axis so flexion reads positive: the motion the wearer just
        // performed defines the positive direction, which keeps the sign
        // meaningful without assuming which arm or how the PCB is oriented.
        let axis = result.axis;
        let total = 0;
        for (const q of capture.quats) {
          const st = swingTwistDeg(q, axis);
          if (st) total += st.twistDeg;
        }
        if (total < 0) axis = [-axis[0], -axis[1], -axis[2]];

        this.hingeAxis = axis;
        this.hingeQuality = {
          samples: capture.axes.length,
          meanSpreadDeg: result.meanSpreadDeg,
          maxSpreadDeg: result.maxSpreadDeg,
          /** Measured curl range for this wearer, from the calibration reps. */
          peakElbowDeg: capture.peakElbowDeg,
        };
        this.hingeState = HINGE_STATE.READY;
        this.hingeMessage =
          `Hinge axis found (spread ${result.meanSpreadDeg.toFixed(1)} deg, ` +
          `measured curl peak ${capture.peakElbowDeg.toFixed(0)} deg).`;
        this.hingeCapture = null;
        this.onEvent({ kind: "hinge_complete", ...this.hingeQuality });
        return;
      }
    }

    if (nowMs - capture.startedAtMs > this.options.hingeTimeoutMs) {
      const result = averageAxis(capture.axes);
      this.hingeState = HINGE_STATE.FAILED;
      this.hingeMessage =
        capture.axes.length < this.options.hingeMinSamples
          ? `Not enough full reps (${capture.axes.length}/${this.options.hingeMinSamples} samples).`
          : `Motion was not a consistent hinge (spread ${result.meanSpreadDeg.toFixed(1)} deg).`;
      this.hingeCapture = null;
      this.onEvent({ kind: "hinge_failed", reason: this.hingeMessage });
    }
  }

  /** conj(q_upper) * q_fore - inter-segment rotation before neutral removal. */
  rawElbowRelative(upperQuat, foreQuat) {
    if (!upperQuat || !foreQuat) return null;
    return quatMultiply(quatConjugate(upperQuat), foreQuat);
  }

  /**
   * Segment rotation since neutral, with the mount correction applied when a
   * two-pose calibration has supplied one. With none, the angle is still
   * correct and the axis stays in the sensor's neutral frame.
   */
  segmentDelta(nodeId) {
    const entry = this.latest.get(nodeId);
    const ref = this.reference.get(nodeId);
    if (!entry || !ref) return null;
    const delta = quatMultiply(quatConjugate(ref), entry.quat);
    const mount = this.mountCorrection.get(nodeId);
    if (!mount) return delta;
    return quatMultiply(quatMultiply(quatConjugate(mount), delta), mount);
  }

  /**
   * Angular velocity of a segment's corrected delta, in the delta's own
   * parent frame, taken from the BNO gyro rather than differenced from 25 Hz
   * quaternions. The gyro is instantaneous, so it sees the zero crossing at a
   * rep turnaround that a finite difference misses by a whole sample.
   *
   * Frames: sensor body rate -> world (rotate by D) -> corrected delta frame
   * (rotate by conj M), matching segmentDelta's conj(M) * D * M.
   */
  segmentOmega(nodeId) {
    const entry = this.latest.get(nodeId);
    const ref = this.reference.get(nodeId);
    if (!entry || !ref || !entry.gyro) return null;
    const delta = quatMultiply(quatConjugate(ref), entry.quat);
    const worldRate = rotateVector(entry.gyro, delta);
    const mount = this.mountCorrection.get(nodeId);
    return mount ? rotateVector(worldRate, quatConjugate(mount)) : worldRate;
  }

  /**
   * Display-only pose with instantaneous angular velocities, for the avatar's
   * latency-compensating extrapolator. Returns null until the anatomical
   * mounts are installed: sensor-neutral axes must never drive the rig.
   *
   * The forearm rate is expressed in the upper arm's frame, which is the frame
   * the nested `forearmRelative` transform lives in: with
   * D_f = D_u * D_rel, differentiating gives
   * omega_rel = conj(D_u) * (omega_f - omega_u) * D_u.
   */
  displayPose(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CALIBRATED) return null;
    if (!this.requiredNodes.every((id) => this.mountCorrection.has(id))) return null;
    // Same health rules as the frame path: a fresh pair on one timebase, or
    // the rig must hold its last pose rather than present stale pair data as
    // live (the 25 Hz frame tick still owns the tracking_unavailable state).
    const entries = [];
    for (const nodeId of this.requiredNodes) {
      const entry = this.latest.get(nodeId);
      if (!entry || nowMs - entry.receivedAtMs > this.options.staleMaxMs) return null;
      entries.push(entry);
    }
    const skewMs = (Math.max(...entries.map((e) => e.deviceS)) - Math.min(...entries.map((e) => e.deviceS))) * 1000;
    if (Math.abs(skewMs) > this.options.syncSkewMaxMs) return null;
    const upper = this.segmentDelta(this.roles.upperArm);
    const fore = this.segmentDelta(this.roles.forearm);
    if (!upper || !fore) return null;
    const omegaUpper = this.segmentOmega(this.roles.upperArm);
    const omegaFore = this.segmentOmega(this.roles.forearm);
    let omegaForearm = null;
    if (omegaUpper && omegaFore) {
      const difference = [
        omegaFore[0] - omegaUpper[0],
        omegaFore[1] - omegaUpper[1],
        omegaFore[2] - omegaUpper[2],
      ];
      omegaForearm = rotateVector(difference, quatConjugate(upper));
    }
    return {
      shoulder: upper,
      forearmRelative: quatMultiply(quatConjugate(upper), fore),
      omegaShoulder: omegaUpper,
      omegaForearm,
      atMs: nowMs,
    };
  }

  // ------------------------------------------------------------ drift monitor

  /**
   * Update the drift estimate from rest observations.
   *
   * When the arm is held still AND back near the calibration pose, the elbow
   * relative rotation should be identity. Whatever it actually is, is the
   * accumulated inter-sensor drift. Both conditions matter: stillness alone
   * would let a rest in a flexed pose be mistaken for drift.
   *
   * This only measures. Applying the correction is `rezeroFromRest`, kept
   * manual because a wearer whose "rest" is not the calibration pose would
   * otherwise have a wrong reference silently written underneath them.
   */
  updateDrift(elbowQuat, nowMs) {
    const allStill = this.trackedNodes.every((id) => {
      const entry = this.latest.get(id);
      return entry && entry.gyroMag <= this.options.stillGyroMaxRadps;
    });
    if (!allStill) {
      this.stillSinceMs = null;
      return;
    }
    if (this.stillSinceMs === null) this.stillSinceMs = nowMs;
    if (nowMs - this.stillSinceMs < this.options.restStillMs) return;

    // Near-neutral must be judged on the TOTAL elbow rotation, not on the
    // flexion component. A large purely off-axis pose has a flexion of ~0 and
    // would otherwise be mistaken for the neutral pose, so the drift estimate
    // would absorb the whole off-axis rotation and disarm the validity gate.
    const totalDeg = quatAngleDeg(elbowQuat);
    if (totalDeg === null || totalDeg > this.options.restMaxFlexionDeg) return;

    this.restElbow = elbowQuat;
    this.driftDeg = quatAngleDeg(elbowQuat) || 0;
  }

  /**
   * Re-zero the elbow reference to the current pose, cancelling accumulated
   * drift without redoing the neutral pose or the hinge axis. Only valid from
   * a rest observation, which is what makes "the current pose is neutral" a
   * safe assumption.
   */
  rezeroFromRest() {
    if (this.state !== CAL_STATE.CALIBRATED) return false;
    if (!this.restElbow) return false;
    // Correct using the STORED rest observation, never the live pose. Reading
    // this.latest here would re-zero to whatever the arm happens to be doing at
    // the moment the button is pressed, which is exactly the silent
    // mis-referencing this method is supposed to avoid.
    //
    // With E_rest = conj(N) * raw_rest, the neutral that makes the rest pose
    // read as identity is N' = N * E_rest.
    this.neutralElbow = quatMultiply(this.neutralElbow, this.restElbow);
    const previous = this.driftDeg;
    this.restElbow = null;
    this.driftDeg = 0;
    this.onEvent({ kind: "drift_rezeroed", clearedDeg: previous });
    return true;
  }

  // ------------------------------------------------------------------ frame

  /**
   * Build the motion packet. Returns a packet whenever orientation data is
   * present; `health` - not a null return - carries whether it is trustworthy,
   * so the diagnostic panel and the 3D consumer can show a degraded state
   * instead of freezing on the last good value.
   */
  computeFrame(nowMs = performance.now()) {
    const fresh = new Map();
    for (const nodeId of this.trackedNodes) {
      const entry = this.latest.get(nodeId);
      fresh.set(nodeId, Boolean(entry) && nowMs - entry.receivedAtMs <= this.options.staleMaxMs);
    }

    const requiredFresh = this.requiredNodes.every((id) => fresh.get(id));
    const deviceTimes = this.requiredNodes
      .map((id) => this.latest.get(id))
      .filter(Boolean)
      .map((entry) => entry.deviceS);
    const skewMs =
      deviceTimes.length === this.requiredNodes.length
        ? (Math.max(...deviceTimes) - Math.min(...deviceTimes)) * 1000
        : null;
    const synchronized =
      requiredFresh && skewMs !== null && Math.abs(skewMs) <= this.options.syncSkewMaxMs;
    const calibrated = this.state === CAL_STATE.CALIBRATED;

    const upperDelta = calibrated ? this.segmentDelta(this.roles.upperArm) : null;
    const foreDelta = calibrated ? this.segmentDelta(this.roles.forearm) : null;

    let elbowDeg = null;
    let elbowAxis = null;
    let yawDriftHintDeg = null;
    let flexionDeg = null;
    let offAxisDeg = null;
    let offAxisExcessDeg = null;
    let flexionValid = true;
    if (calibrated && requiredFresh && this.neutralElbow) {
      const raw = this.rawElbowRelative(
        this.latest.get(this.roles.upperArm).quat,
        this.latest.get(this.roles.forearm).quat
      );
      // E = conj(neutral) * raw: zero at the calibration pose, and invariant to
      // both mount rotations because the same unknowns appear on each side.
      const elbow = quatMultiply(quatConjugate(this.neutralElbow), raw);
      elbowDeg = quatAngleDeg(elbow);
      elbowAxis = quatAxis(elbow);
      if (this.hingeState === HINGE_STATE.CAPTURING) this.accumulateHinge(elbow, nowMs);
      if (this.hingeAxis) {
        const decomposed = swingTwistDeg(elbow, this.hingeAxis);
        if (decomposed) {
          flexionDeg = decomposed.twistDeg;
          offAxisDeg = decomposed.swingDeg;
        }
      }
      this.updateDrift(elbow, nowMs);
      // Drift lands almost entirely in the off-axis term, so the validity gate
      // is applied to the EXCESS over the current drift estimate. Comparing the
      // raw value would make a normal rep fail the gate after a few minutes.
      offAxisExcessDeg = offAxisDeg === null ? null : Math.max(0, offAxisDeg - this.driftDeg);
      const tooFarOffAxis =
        offAxisExcessDeg !== null && offAxisExcessDeg > this.options.flexionMaxOffAxisDeg;
      const implausible =
        flexionDeg !== null && Math.abs(flexionDeg) > this.options.flexionMaxPlausibleDeg;
      if (tooFarOffAxis || implausible) {
        flexionValid = false;
        flexionDeg = null;
      }
      // With no magnetometer, a slow common-mode rotation of both segments that
      // is NOT accompanied by elbow change is the visible signature of yaw
      // drift. Reported as a hint, never silently corrected.
      const upperDeg = quatAngleDeg(upperDelta) || 0;
      const foreDeg = quatAngleDeg(foreDelta) || 0;
      yawDriftHintDeg = Math.max(0, Math.min(upperDeg, foreDeg) - elbowDeg);
    }

    // Live capture feedback: how high the arm is and, during the forward hold,
    // how far its pointing direction is from the stored side hold. These are
    // exactly the quantities the pointing solve will use, so the wearer can
    // steer to them before the window closes. Rotation axes are deliberately
    // not shown: a palm turn tilts them and would coach the wrong correction.
    let anatomicalLive = null;
    if (calibrated && this.anatomicalCapture) {
      const live = {};
      for (const nodeId of this.requiredNodes) {
        const entry = this.latest.get(nodeId);
        const ref = this.reference.get(nodeId);
        if (!entry || !ref) continue;
        const now = this.pointingOf(nodeId, quatMultiply(quatConjugate(ref), entry.quat));
        if (!now) continue;
        let separationDeg = null;
        // Below ~30 degrees up the horizontal direction is noise.
        if (this.anatomicalCapture.kind === "forward" && now.elevationDeg >= 30) {
          const side = this.anatomicalSide && this.anatomicalSide.get(nodeId);
          if (side) separationDeg = this.pointingSeparationDeg(nodeId, side.pointing, now.pointing);
        }
        live[nodeId] = { raiseDeg: now.elevationDeg, separationDeg };
      }
      if (Object.keys(live).length > 0) anatomicalLive = live;
    }

    const frame = {
      timestamp_ms: Math.round(nowMs),
      upper_arm_orientation: upperDelta ? quatToPacket(upperDelta) : null,
      forearm_orientation: foreDelta ? quatToPacket(foreDelta) : null,
      elbow_relative_rotation_deg: elbowDeg,
      /**
       * Signed rotation about the measured hinge axis - the real joint angle.
       * Null until a range calibration has run. Prefer this over the unsigned
       * composite above wherever it is available.
       */
      elbow_flexion_deg: flexionDeg,
      /**
       * How far the motion departed from a clean hinge rotation. Large values
       * mean the movement was compound (forearm rotation, shoulder involvement)
       * and that elbow_flexion_deg is describing only part of what happened.
       */
      elbow_off_axis_deg: offAxisDeg,
      /**
       * Off-axis with the drift estimate removed. This, not the raw value, is
       * what a consumer should threshold on to detect compound motion.
       */
      elbow_off_axis_excess_deg: offAxisExcessDeg,
      upper_arm_deviation_deg: calibrated ? quatAngleDeg(upperDelta) : null,
      health: {
        n2: Boolean(fresh.get(2)),
        n3: Boolean(fresh.get(3)),
        n4: Boolean(fresh.get(4)),
        synchronized,
        calibrated,
      },
      diagnostics: {
        calibrationState: this.state,
        calibrationMessage: this.calibrationMessage,
        hingeState: this.hingeState,
        hingeMessage: this.hingeMessage,
        hingeQuality: this.hingeQuality,
        /**
         * Why elbow_flexion_deg is null: false here means the motion was too
         * far off the hinge for a signed angle to mean anything, as opposed to
         * no hinge calibration having been run at all.
         */
        flexionValid,
        driftDeg: this.driftDeg,
        recalibrationRecommended: this.driftDeg > this.options.driftWarnDeg,
        restObserved: this.restElbow !== null,
        forearmDeviationDeg: calibrated ? quatAngleDeg(foreDelta) : null,
        elbowAxisNeutralFrame: elbowAxis,
        skewMs,
        yawDriftHintDeg,
        // The plan's stated scope limits, carried in-band so a consumer cannot
        // mistake this for trunk-relative or anatomically-aligned data.
        trunkReferenced: false,
        anatomicalState: this.anatomicalState,
        anatomicalMessage: this.anatomicalMessage,
        anatomicalQuality: this.anatomicalQuality,
        anatomicalSideAnglesDeg: this.anatomicalSideAnglesDeg,
        anatomicalLive,
        anatomicalWarning: this.anatomicalWarning,
        handRestPalm: this.handRestPalm,
        axisFrame: this.requiredNodes.every((id) => this.mountCorrection.has(id))
          ? "anatomical"
          : "sensor_neutral",
      },
    };
    this.lastFrame = frame;
    return frame;
  }

  /**
   * Flat row for the session log, so recorded runs can be replayed.
   *
   * Missing values are NaN, not a sentinel number: NaN survives the Float32
   * ring and serializes to JSON `null`, whereas the -1 this used to write is a
   * legitimate value for the signed flexion angle.
   */
  toLogRow(frame) {
    const f = frame || this.lastFrame;
    if (!f) return null;
    const u = f.upper_arm_orientation;
    const a = f.forearm_orientation;
    const num = (value) => (value === null || value === undefined ? NaN : value);
    return [
      f.timestamp_ms / 1000,
      u ? u.qx : NaN, u ? u.qy : NaN, u ? u.qz : NaN, u ? u.qw : NaN,
      a ? a.qx : NaN, a ? a.qy : NaN, a ? a.qz : NaN, a ? a.qw : NaN,
      num(f.elbow_relative_rotation_deg),
      num(f.elbow_flexion_deg),
      num(f.elbow_off_axis_deg),
      num(f.elbow_off_axis_excess_deg),
      num(f.upper_arm_deviation_deg),
      num(f.diagnostics.driftDeg),
      f.health.calibrated ? 1 : 0,
      f.health.synchronized ? 1 : 0,
      num(f.diagnostics.skewMs),
    ];
  }

  static get LOG_COLUMNS() {
    return [
      "t_s",
      "upper_qx", "upper_qy", "upper_qz", "upper_qw",
      "fore_qx", "fore_qy", "fore_qz", "fore_qw",
      "elbow_deg", "elbow_flexion_deg", "elbow_off_axis_deg",
      "elbow_off_axis_excess_deg", "upper_dev_deg", "drift_deg",
      "calibrated", "synchronized", "skew_ms",
    ];
  }

  reset() {
    this.latest.clear();
    this.clearCalibration();
    this.lastFrame = null;
  }
}
