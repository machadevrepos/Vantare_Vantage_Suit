import { solveMountCorrection } from "./anatomical-calibration.js";

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
  /** Neutral pose hold, seconds of usable stillness required. */
  holdSeconds: 1.5,
  /** Reject calibration if any node's gyro magnitude exceeds this (rad/s). */
  stillGyroMaxRadps: 0.2,
  /** Reject calibration if a node's orientation spread exceeds this (deg). */
  stillSpreadMaxDeg: 3.0,
  /** A node older than this is not fresh. */
  staleMaxMs: 250,
  /** Max pairwise device-time skew for a frame to count as synchronized. */
  syncSkewMaxMs: 60,
  /** Calibration aborts if stillness cannot be achieved within this. */
  calibrationTimeoutMs: 15000,

  // --- anatomical frame calibration: two held straight-arm directions ---
  anatomicalHoldSeconds: 1.0,
  anatomicalMinAngleDeg: 60,
  anatomicalMaxAngleDeg: 120,
  anatomicalMaxSegmentMismatchDeg: 15,
  anatomicalTimeoutMs: 15000,

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
    const gyroMag = Math.hypot(
      values.gyro_x_radps || 0,
      values.gyro_y_radps || 0,
      values.gyro_z_radps || 0
    );
    this.latest.set(nodeId, { quat, gyroMag, deviceS, receivedAtMs: nowMs });
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
      /** Restarted whenever motion is seen, so the hold must be contiguous. */
      windowStartedAtMs: nowMs,
      samples: new Map(this.trackedNodes.map((id) => [id, []])),
      rejections: 0,
      lastRejectReason: null,
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

  /** Drop the accumulated hold and start the stillness window again. */
  restartHold(nowMs, message, reason = null) {
    for (const list of this.capture.samples.values()) list.length = 0;
    this.capture.windowStartedAtMs = nowMs;
    if (reason) {
      this.capture.rejections += 1;
      this.capture.lastRejectReason = reason;
    }
    this.calibrationMessage = message;
  }

  /**
   * Stillness gate. A sample above the gyro threshold discards the whole
   * accumulated window rather than just that sample: a neutral reference built
   * from the still halves either side of a twitch would be a blend of two
   * poses, which is worse than asking the wearer to hold again.
   */
  accumulateCalibration(nodeId, quat, gyroMag, nowMs) {
    if (!this.capture) return;
    if (gyroMag > this.options.stillGyroMaxRadps) {
      this.restartHold(
        nowMs,
        "Too much motion - keep holding.",
        `motion on N${nodeId} (${gyroMag.toFixed(2)} rad/s)`
      );
      return;
    }
    this.capture.samples.get(nodeId).push(quat);
  }

  /**
   * Called from the render tick. Decides whether the held window is now long
   * enough, complete across the required nodes, and tight enough to accept.
   */
  updateCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CAPTURING) return;
    const capture = this.capture;
    const heldMs = nowMs - capture.windowStartedAtMs;

    if (nowMs - capture.startedAtMs > this.options.calibrationTimeoutMs) {
      this.state = CAL_STATE.FAILED;
      this.capture = null;
      this.calibrationMessage = capture.lastRejectReason
        ? `Timed out: ${capture.lastRejectReason}.`
        : "Timed out waiting for a still neutral pose.";
      this.onEvent({ kind: "calibration_failed", reason: this.calibrationMessage });
      return;
    }
    if (heldMs < this.options.holdSeconds * 1000) return;

    // Every required node must be fresh AND have contributed to this window.
    for (const nodeId of this.requiredNodes) {
      const entry = this.latest.get(nodeId);
      if (!entry || nowMs - entry.receivedAtMs > this.options.staleMaxMs) {
        this.restartHold(nowMs, `Waiting for N${nodeId} data.`);
        return;
      }
      if (capture.samples.get(nodeId).length < 4) {
        this.calibrationMessage = `Waiting for N${nodeId} samples.`;
        return;
      }
    }

    const reference = new Map();
    for (const nodeId of this.trackedNodes) {
      const samples = capture.samples.get(nodeId);
      // The aux node may legitimately lag; only required nodes gate above.
      if (!samples || samples.length < 4) continue;
      const mean = quatAverage(samples);
      if (!mean) {
        this.restartHold(nowMs, `Degenerate quaternion on N${nodeId}.`);
        return;
      }
      const spread = quatSpreadDeg(samples, mean);
      if (spread > this.options.stillSpreadMaxDeg) {
        this.restartHold(
          nowMs,
          "Pose was not steady - keep holding.",
          `N${nodeId} drifted ${spread.toFixed(1)} deg during the hold`
        );
        return;
      }
      reference.set(nodeId, mean);
    }

    this.reference = reference;
    // Neutral inter-segment rotation, subtracted out so a straight arm reads 0
    // no matter how the two PCBs happen to sit relative to each other.
    this.neutralElbow = this.rawElbowRelative(
      reference.get(this.roles.upperArm),
      reference.get(this.roles.forearm)
    );
    this.state = CAL_STATE.CALIBRATED;
    this.calibratedAtMs = nowMs;
    this.capture = null;
    this.calibrationMessage = "Calibrated.";
    this.anatomicalMessage = "Next: hold a straight-arm raise to your right side.";
    this.onEvent({
      kind: "calibration_complete",
      nodes: [...reference.keys()],
      heldMs: Math.round(heldMs),
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
  }

  beginSideCalibration(nowMs = performance.now()) {
    if (this.state !== CAL_STATE.CALIBRATED) {
      this.anatomicalState = ANATOMICAL_STATE.FAILED;
      this.anatomicalMessage = "Capture a neutral pose first.";
      return false;
    }
    this.mountCorrection.clear();
    this.anatomicalSide = null;
    this.anatomicalQuality = null;
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
    // The palm/thumb cues are the no-twist end states: from a neutral with
    // the palms facing the thighs, a pure side raise lands palm down and a
    // pure forward raise lands thumb up. Asking for any other orientation
    // forces a forearm rotation into the capture, which trips the straight-
    // elbow gates (2026-09-11 field session).
    this.anatomicalMessage =
      kind === "side"
        ? "Hold your straight arm 90 degrees to your right side, palm down."
        : "Hold your straight arm 90 degrees forward, thumb up.";
    this.anatomicalCapture = {
      kind,
      startedAtMs: nowMs,
      windowStartedAtMs: nowMs,
      samples: new Map(this.requiredNodes.map((id) => [id, []])),
      lastRejectReason: null,
    };
  }

  restartAnatomicalHold(nowMs, message, reason = null) {
    const capture = this.anatomicalCapture;
    if (!capture) return;
    // Audit every rejection (spec section 9), but only when the reason
    // changes: a continuously-moving pose restarts on every sample and would
    // flood the Tier-1 event log with duplicates.
    if (reason !== capture.lastRejectReason) {
      this.onEvent({
        kind: "anatomical_hold_restarted",
        stage: capture.kind,
        reason: reason ?? message,
      });
    }
    for (const samples of capture.samples.values()) samples.length = 0;
    capture.windowStartedAtMs = nowMs;
    capture.lastRejectReason = reason;
    this.anatomicalMessage = message;
  }

  accumulateAnatomicalCalibration(nodeId, quat, gyroMag, nowMs) {
    const capture = this.anatomicalCapture;
    const reference = this.reference.get(nodeId);
    if (!capture || !reference) return;
    if (gyroMag > this.options.stillGyroMaxRadps) {
      this.restartAnatomicalHold(
        nowMs,
        "Settle into the pose and hold still.",
        `motion on N${nodeId} (${gyroMag.toFixed(2)} rad/s)`
      );
      return;
    }
    capture.samples.get(nodeId).push(quatMultiply(quatConjugate(reference), quat));
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
        capture.lastRejectReason
          ? `Timed out: ${capture.lastRejectReason.replace(/\.$/, "")}.`
          : `Timed out waiting for the ${capture.kind} pose.`
      );
      return;
    }
    if (nowMs - capture.windowStartedAtMs < this.options.anatomicalHoldSeconds * 1000) return;

    const deviceTimes = [];
    for (const nodeId of this.requiredNodes) {
      const entry = this.latest.get(nodeId);
      if (!entry || nowMs - entry.receivedAtMs > this.options.staleMaxMs) {
        this.restartAnatomicalHold(nowMs, `Waiting for N${nodeId} data.`);
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
      this.restartAnatomicalHold(nowMs, "Sensor samples are not synchronized.", "sample skew");
      return;
    }

    const captured = new Map();
    for (const nodeId of this.requiredNodes) {
      const samples = capture.samples.get(nodeId);
      const mean = quatAverage(samples);
      const spreadDeg = mean ? quatSpreadDeg(samples, mean) : Infinity;
      const angleDeg = mean ? quatAngleDeg(mean) : null;
      const axis = mean ? quatAxis(mean) : null;
      if (!mean || spreadDeg > this.options.stillSpreadMaxDeg) {
        this.restartAnatomicalHold(
          nowMs,
          "Pose was not steady - keep holding.",
          `N${nodeId} spread ${spreadDeg.toFixed(1)} deg`
        );
        return;
      }
      if (
        angleDeg === null ||
        angleDeg < this.options.anatomicalMinAngleDeg ||
        angleDeg > this.options.anatomicalMaxAngleDeg ||
        !axis
      ) {
        // A transient pass-through (e.g. pausing low on the way up) must not
        // kill the attempt: discard the window and let the wearer keep
        // holding. The timeout carries the reason if it never settles.
        this.restartAnatomicalHold(
          nowMs,
          `N${nodeId} raise must be ${this.options.anatomicalMinAngleDeg}-${this.options.anatomicalMaxAngleDeg} degrees.`,
          `N${nodeId} raise must be ${this.options.anatomicalMinAngleDeg}-${this.options.anatomicalMaxAngleDeg} degrees`
        );
        return;
      }
      captured.set(nodeId, { mean, axis, angleDeg, spreadDeg });
    }

    const angles = this.requiredNodes.map((id) => captured.get(id).angleDeg);
    const mismatchDeg = Math.abs(angles[0] - angles[1]);
    if (mismatchDeg > this.options.anatomicalMaxSegmentMismatchDeg) {
      const message = `Keep the elbow straight; segment raises differed by ${mismatchDeg.toFixed(1)} degrees.`;
      this.restartAnatomicalHold(nowMs, message, message);
      return;
    }

    // NOTE: do NOT gate this capture on an inter-sensor quantity such as an
    // elbow-relative rotation conj(q_upper)*q_fore. Mount rotations cancel
    // out of it, but each BNO085 Game Rotation Vector carries its own
    // arbitrary power-up heading, and once the upper arm rotates the N2-vs-N4
    // heading offset leaks straight into the product (~5 degrees of error per
    // 5 degrees of heading offset on a perfectly straight arm). That gate
    // rejected real straight-arm raises in the field and could never pass
    // (session 2026-09-11T04:44). The magnitude and axis-separation gates
    // above are conjugation-invariant and carry the straight-elbow coverage;
    // twist detection needs the gravity vector instead.

    if (capture.kind === "side") {
      this.anatomicalSide = captured;
      this.anatomicalCapture = null;
      this.anatomicalState = ANATOMICAL_STATE.SIDE_READY;
      this.anatomicalMessage = "Side pose captured. Next: hold a straight-arm forward raise.";
      this.onEvent({
        kind: "anatomical_side_complete",
        anglesDeg: Object.fromEntries([...captured].map(([id, value]) => [id, value.angleDeg])),
      });
      return;
    }

    const solved = new Map();
    const quality = {};
    for (const nodeId of this.requiredNodes) {
      const result = solveMountCorrection(this.anatomicalSide.get(nodeId).axis, captured.get(nodeId).axis);
      if (!result.ok) {
        if (result.reason !== "axes_not_independent") {
          this.failAnatomicalCalibration(
            `Could not solve anatomical mapping for N${nodeId} (${result.reason}).`
          );
          return;
        }
        // Report the MEASURED separation: without it a field log shows a
        // rejection with no way to tell a scapular-plane raise from two
        // raises in the same direction (05:14 session had no number).
        const separationsDeg = {};
        for (const id of this.requiredNodes) {
          const a = this.anatomicalSide.get(id).axis;
          const b = captured.get(id).axis;
          const dot = Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
          separationsDeg[id] = Math.acos(dot) * 180 / Math.PI;
        }
        const measured = separationsDeg[this.roles.upperArm].toFixed(0);
        this.failAnatomicalCalibration(
          `The raises measured ${measured} degrees apart - they were not independent enough ` +
            "(need 60-120). Raise straight out to the side, then straight ahead.",
          { separationsDeg }
        );
        return;
      }
      solved.set(nodeId, result.mount);
      quality[nodeId] = {
        ...result.quality,
        sideAngleDeg: this.anatomicalSide.get(nodeId).angleDeg,
        forwardAngleDeg: captured.get(nodeId).angleDeg,
        sideSpreadDeg: this.anatomicalSide.get(nodeId).spreadDeg,
        forwardSpreadDeg: captured.get(nodeId).spreadDeg,
      };
    }

    this.mountCorrection = solved;
    this.anatomicalQuality = { nodes: quality, segmentMismatchDeg: mismatchDeg };
    this.anatomicalCapture = null;
    this.anatomicalState = ANATOMICAL_STATE.CALIBRATED;
    this.anatomicalMessage = "Anatomical axes calibrated.";
    this.onEvent({
      kind: "anatomical_complete",
      mounts: Object.fromEntries(solved),
      quality: this.anatomicalQuality,
    });
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
    this.hingeMessage = "Perform a few slow full reps.";
    this.hingeCapture = { startedAtMs: nowMs, axes: [], quats: [] };
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
        };
        this.hingeState = HINGE_STATE.READY;
        this.hingeMessage = `Hinge axis found (spread ${result.meanSpreadDeg.toFixed(1)} deg).`;
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
