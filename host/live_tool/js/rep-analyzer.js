/**
 * Rep analyzer: turns the Motion Engine's continuous output into per-rep
 * verdicts a coach can act on.
 *
 * This is the Coach Assist correction loop (movement-tracking plan, steps 6-7):
 * the coach defines what correct means, the system measures the gap. There is
 * no classifier here and none is needed -- the two faults the V1 model was
 * trying to learn are both direct measurements.
 *
 * WHICH METRICS ACTUALLY DISCRIMINATE
 * Thresholds below are not guesses. They come from replaying a deliberately
 * bad set (2026-09-10T06:43, 3 clean reps then 10 with swinging, elbow drift
 * and partial range) against clean sets from the same rig:
 *
 *   upper-arm deviation  clean 6.4-9.8 deg | bad 18.9-103.5 deg  CLEAN SPLIT
 *   off-axis excess      clean 6.1-23.8    | bad 17.2-91.7       OVERLAPS
 *   peak flexion         clean 132.8-137.6 | bad 96.2-149.3      OVERLAPS
 *
 * So upper-arm deviation carries the form judgement, with a 15 deg threshold
 * sitting in the middle of a wide empty gap. Peak flexion catches short reps
 * but cannot stand alone -- a swung rep often reaches FULL range precisely
 * because it was swung. Off-axis excess is deliberately NOT used as a form
 * grade despite being the more obvious candidate: the clean and bad ranges
 * overlap, so thresholding it would both false-alarm on good reps and miss real
 * faults. It earns its place as a validity signal instead, via the off-hinge
 * fraction, which was 0% on every clean rep and 25-90% during gross swinging.
 */

/** Segmentation parameters. Not coach-facing; these define what a rep IS. */
export const REP_DEFAULTS = {
  /** Flexion rising past this opens a rep. */
  openDeg: 30,
  /** Falling back under this closes it. Hysteresis stops jitter double-counting. */
  closeDeg: 20,
  /** Shorter than this is a twitch, not a rep. */
  minDurationS: 0.4,
  /**
   * Longer than this is not a rep either. In the bad set, one stretch of
   * continuous swinging stayed above the close threshold for 11.4 s and would
   * otherwise have been scored as a single enormous rep.
   */
  maxDurationS: 6.0,
  /** A rep that never reaches this never really happened. */
  minPeakDeg: 40,
};

/** Coach-facing target. Every value here is meant to be tuned per athlete. */
export const TARGET_DEFAULTS = {
  /** Peak flexion a rep must reach. Clean sets peaked at 131.6 +/- 4.0 deg. */
  romTargetDeg: 120,
  /** Upper-arm movement allowed before it counts as swinging or elbow drift. */
  upperArmToleranceDeg: 15,
  /**
   * Fraction of frames allowed to be off-hinge before the movement is called
   * compound. Clean reps sit at 0; gross swinging ran 25-90%.
   */
  maxOffHingeFraction: 0.1,
};

export const FAULTS = {
  INCOMPLETE_ROM: "incomplete_rom",
  UPPER_ARM_MOVEMENT: "upper_arm_movement",
  COMPOUND_MOTION: "compound_motion",
  MALFORMED: "malformed",
};

export const FAULT_LABELS = {
  [FAULTS.INCOMPLETE_ROM]: "Incomplete range of motion",
  [FAULTS.UPPER_ARM_MOVEMENT]: "Upper arm moving (swing / elbow drift)",
  [FAULTS.COMPOUND_MOTION]: "Compound motion, not a clean elbow movement",
  [FAULTS.MALFORMED]: "Not a countable rep",
};

export class RepAnalyzer {
  constructor(options = {}) {
    this.params = { ...REP_DEFAULTS, ...(options.params || {}) };
    this.target = { ...TARGET_DEFAULTS, ...(options.target || {}) };
    this.onRep = options.onRep || (() => {});
    this.reps = [];
    this.current = null;
    this.wasCalibrated = false;
    /**
     * Set after a rep is force-closed on duration. Without it, a long hold
     * above the open threshold immediately re-opens and keeps emitting
     * pseudo-reps for as long as the arm stays up. Cleared once flexion drops
     * back through the close threshold, which is a real end of movement.
     */
    this.needsRearm = false;
  }

  setTarget(partial) {
    this.target = { ...this.target, ...partial };
  }

  reset() {
    this.reps = [];
    this.current = null;
    this.wasCalibrated = false;
    this.needsRearm = false;
  }

  /**
   * Feed one motion packet. Returns the rep that just completed, or null.
   *
   * Frames whose flexion is withheld (off-hinge, or no hinge calibration) are
   * still consumed: they cannot open or close a rep, but inside one they count
   * toward the off-hinge fraction, which is how gross compound motion gets
   * flagged rather than silently ignored.
   */
  pushFrame(frame) {
    if (!frame) return null;
    const calibrated = Boolean(frame.health && frame.health.calibrated);
    if (!calibrated) {
      // Losing calibration mid-rep invalidates everything measured against the
      // old reference, so the in-progress rep is dropped rather than scored.
      this.current = null;
      this.wasCalibrated = false;
      this.needsRearm = false;
      return null;
    }
    this.wasCalibrated = true;

    const flexion = frame.elbow_flexion_deg;
    const tMs = frame.timestamp_ms;
    const upperDev = frame.upper_arm_deviation_deg;
    const offAxis = frame.elbow_off_axis_excess_deg;

    if (this.current === null) {
      if (this.needsRearm) {
        if (flexion !== null && flexion !== undefined && flexion < this.params.closeDeg) {
          this.needsRearm = false;
        }
        return null;
      }
      if (flexion !== null && flexion !== undefined && flexion > this.params.openDeg) {
        this.current = {
          index: this.reps.length + 1,
          startMs: tMs,
          endMs: tMs,
          peakFlexionDeg: flexion,
          minFlexionDeg: flexion,
          maxUpperArmDevDeg: upperDev === null ? 0 : upperDev,
          maxOffAxisExcessDeg: offAxis === null ? 0 : offAxis,
          frames: 1,
          offHingeFrames: 0,
        };
      }
      return null;
    }

    const rep = this.current;
    rep.endMs = tMs;
    rep.frames += 1;
    if (upperDev !== null && upperDev !== undefined) {
      rep.maxUpperArmDevDeg = Math.max(rep.maxUpperArmDevDeg, upperDev);
    }
    if (offAxis !== null && offAxis !== undefined) {
      rep.maxOffAxisExcessDeg = Math.max(rep.maxOffAxisExcessDeg, offAxis);
    }

    if (flexion === null || flexion === undefined) {
      rep.offHingeFrames += 1;
      // A rep that runs long while off-hinge would never close on its own.
      if ((tMs - rep.startMs) / 1000 > this.params.maxDurationS) return this.close(tMs, true);
      return null;
    }

    rep.peakFlexionDeg = Math.max(rep.peakFlexionDeg, flexion);
    rep.minFlexionDeg = Math.min(rep.minFlexionDeg, flexion);
    if (flexion < this.params.closeDeg) return this.close(tMs);
    if ((tMs - rep.startMs) / 1000 > this.params.maxDurationS) return this.close(tMs, true);
    return null;
  }

  close(tMs, forced = false) {
    const rep = this.current;
    this.current = null;
    if (forced) this.needsRearm = true;
    if (!rep) return null;
    rep.endMs = tMs;
    rep.durationS = (rep.endMs - rep.startMs) / 1000;
    rep.offHingeFraction = rep.frames > 0 ? rep.offHingeFrames / rep.frames : 0;

    const faults = [];
    const malformed =
      rep.durationS < this.params.minDurationS ||
      rep.durationS > this.params.maxDurationS ||
      rep.peakFlexionDeg < this.params.minPeakDeg;
    if (malformed) {
      faults.push(FAULTS.MALFORMED);
    } else {
      if (rep.peakFlexionDeg < this.target.romTargetDeg) faults.push(FAULTS.INCOMPLETE_ROM);
      if (rep.maxUpperArmDevDeg > this.target.upperArmToleranceDeg) {
        faults.push(FAULTS.UPPER_ARM_MOVEMENT);
      }
      if (rep.offHingeFraction > this.target.maxOffHingeFraction) {
        faults.push(FAULTS.COMPOUND_MOTION);
      }
    }
    rep.faults = faults;
    rep.verdict = faults.length === 0 ? "correct" : "fault";

    // A malformed segment is not a rep, so it must not consume an index or
    // appear in the count the coach sees.
    if (malformed) {
      rep.index = null;
      this.onRep(rep);
      return rep;
    }
    rep.index = this.reps.length + 1;
    this.reps.push(rep);
    this.onRep(rep);
    return rep;
  }

  /** Session tallies for the panel and the end-of-session summary. */
  get summary() {
    const total = this.reps.length;
    const correct = this.reps.filter((rep) => rep.verdict === "correct").length;
    const tally = {};
    for (const rep of this.reps) {
      for (const fault of rep.faults) tally[fault] = (tally[fault] || 0) + 1;
    }
    const peaks = this.reps.map((rep) => rep.peakFlexionDeg);
    const mean = peaks.length ? peaks.reduce((a, b) => a + b, 0) / peaks.length : null;
    const sd =
      peaks.length > 1
        ? Math.sqrt(peaks.reduce((a, b) => a + (b - mean) ** 2, 0) / peaks.length)
        : null;
    return {
      total,
      correct,
      faults: tally,
      meanPeakDeg: mean,
      sdPeakDeg: sd,
      // Consistency is what the plan's "movement consistency" chart shows, and
      // it is only meaningful once there are a few reps to compare.
      consistent: sd !== null ? sd <= 10 : null,
    };
  }
}
