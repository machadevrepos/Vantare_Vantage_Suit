/**
 * Replay audit for the anatomical arm avatar (three-pose calibration plan).
 *
 * Reads a session NDJSON log (session-log.js buildDownload format), replays
 * the "motion" stream through the PRODUCTION avatar classification and
 * smoothing path (arm-avatar.js) at 60 Hz, and prints JSON metrics on stdout:
 *
 *   replayedSamples        motion rows consumed
 *   cadenceHz              motion sample rate observed in the log
 *   invalidFrames          rows the rig refuses to render live
 *   dropoutFrames          calibrated rows with a missing/unsynced node
 *   visualStepDistribution per-tick displayed rotation steps (p50/p95/max deg)
 *   medianAddedLatencyMs   added display lag: the time shift at which the
 *                          displayed signal best matches the input target
 *                          signal (cross-correlation argmin over 0-12 ticks)
 *   axisFrame              "anatomical" | "sensor_neutral" (from events)
 *   directionValidation    "available" | "unavailable"
 *   sideDirectionErrorDeg  best calibrated upper-arm axis error vs [0,0,-1]
 *   forwardDirectionErrorDeg  ... vs [1,0,0]
 *
 * Missing calibration information is reported, never repaired: a legacy log
 * without motion_anatomical_complete events replays as axisFrame
 * "sensor_neutral" with directionValidation "unavailable". Rows logged before
 * the neutral calibration (calibrated=0) are always gated as sensor-neutral.
 * Note a real log timestamps events on the performance.now() clock while
 * motion rows carry device seconds, so the two cannot be aligned; frame gating
 * therefore keys on each row's calibrated flag and the presence of the
 * directional events, which only affects replay visuals, never the direction
 * validation math.
 *
 *   node host/tests/scripts/replay_arm_avatar.mjs <session.ndjson>
 */
import { readFileSync } from "node:fs";
import { ArmPoseSmoother, armPoseForMotion } from "../../live_tool/js/arm-avatar.js";

const TICK_MS = 1000 / 60;
const SIDE_AXIS = [0, 0, -1];
const FORWARD_AXIS = [1, 0, 0];
/** A calibrated pose counts as a directional candidate within this window. */
const CAPTURE_ANGLE_TOLERANCE_DEG = 15;
/** Log row layout: MotionEngine.LOG_COLUMNS. */
const COL = { T: 0, UQ: [1, 2, 3, 4], FQ: [5, 6, 7, 8], CAL: 15, SYNC: 16 };

function parseLog(text) {
  const meta = {};
  const events = [];
  const rows = [];
  for (const line of text.split("\n")) {
    const trimmed = line.trim();
    if (!trimmed) continue;
    const entry = JSON.parse(trimmed);
    if (entry.type === "meta") Object.assign(meta, entry);
    else if (entry.type === "event") events.push(entry);
    else if (entry.type === "sample" && entry.stream === "motion") rows.push(entry.data);
  }
  return { meta, events, rows };
}

function normalizeQuat(components) {
  const [x, y, z, w] = components;
  if (![x, y, z, w].every((v) => Number.isFinite(v))) return null;
  const norm = Math.hypot(x, y, z, w);
  if (norm < 1e-9) return null;
  return { qx: x / norm, qy: y / norm, qz: z / norm, qw: w / norm };
}

function separationDeg(a, b) {
  if (!a || !b) return NaN;
  const dot = Math.abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw);
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

/** Rotation angle and axis with the w >= 0 sign convention (0-180 degrees). */
function angleAxisDeg(q) {
  let { qx: x, qy: y, qz: z, qw: w } = q;
  if (w < 0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }
  const angleDeg = 2 * Math.acos(Math.min(1, w)) * 180 / Math.PI;
  const s = Math.sqrt(Math.max(0, 1 - w * w));
  const axis = s < 1e-9 ? [1, 0, 0] : [x / s, y / s, z / s];
  return { angleDeg, axis };
}

function angleBetweenVectorsDeg(a, b) {
  let dot = 0;
  for (let i = 0; i < 3; i += 1) dot += a[i] * b[i];
  return Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

function percentile(sorted, fraction) {
  if (sorted.length === 0) return null;
  const index = Math.min(sorted.length - 1, Math.round(fraction * (sorted.length - 1)));
  return Number(sorted[index].toFixed(2));
}

/** Directional capture angles the events recorded (N4 = upper arm). */
function captureAnglesFromEvents(events) {
  const sideEvent = events.find((e) => e.kind === "motion_anatomical_side_complete");
  const complete = events.find((e) => e.kind === "motion_anatomical_complete");
  if (!sideEvent || !complete) return null;
  const sideAngleDeg = sideEvent.anglesDeg?.["4"];
  const forwardAngleDeg = complete.quality?.nodes?.["4"]?.forwardAngleDeg;
  if (!Number.isFinite(sideAngleDeg) || !Number.isFinite(forwardAngleDeg)) return null;
  return { sideAngleDeg, forwardAngleDeg };
}

/**
 * Smallest angle between any calibrated row's corrected upper-arm axis and the
 * anatomical target, among rows whose rotation angle matches the recorded
 * capture. No candidate rows means the log cannot validate this direction.
 */
function directionErrorDeg(rows, captureAngleDeg, expectedAxis) {
  let best = null;
  for (const row of rows) {
    if (row[COL.CAL] !== 1) continue;
    const q = normalizeQuat(COL.UQ.map((i) => row[i]));
    if (!q) continue;
    const { angleDeg, axis } = angleAxisDeg(q);
    if (Math.abs(angleDeg - captureAngleDeg) > CAPTURE_ANGLE_TOLERANCE_DEG) continue;
    const error = angleBetweenVectorsDeg(axis, expectedAxis);
    if (best === null || error < best) best = error;
  }
  return best;
}

function frameForRow(row, axisFrame) {
  const calibrated = row[COL.CAL] === 1;
  const synchronized = calibrated && row[COL.SYNC] === 1;
  return {
    health: {
      calibrated,
      n2: synchronized,
      n4: synchronized,
      synchronized,
    },
    diagnostics: { axisFrame: calibrated ? axisFrame : "sensor_neutral" },
    upper_arm_orientation: normalizeQuat(COL.UQ.map((i) => row[i])),
    forearm_orientation: normalizeQuat(COL.FQ.map((i) => row[i])),
  };
}

function replay(rows, axisFrame) {
  const smoother = new ArmPoseSmoother();
  if (rows.length === 0) {
    return { replayedSamples: 0, ticks: [], targets: [], invalidFrames: 0, dropoutFrames: 0, cadenceHz: 0 };
  }
  const firstS = rows[0][COL.T];
  const lastS = rows[rows.length - 1][COL.T];
  const durationS = Math.max(lastS - firstS, 1 / 25);

  let rowIdx = 0;
  let invalidFrames = 0;
  let dropoutFrames = 0;
  const ticks = [];
  const targets = []; // per-tick latest motion-frame pose the display chases
  let currentTarget = null;

  for (let tMs = 0; tMs <= (lastS - firstS) * 1000 + TICK_MS; tMs += TICK_MS) {
    while (rowIdx < rows.length && (rows[rowIdx][COL.T] - firstS) * 1000 <= tMs) {
      const row = rows[rowIdx];
      if (row[COL.CAL] === 1 && row[COL.SYNC] !== 1) dropoutFrames += 1;
      const frame = frameForRow(row, axisFrame);
      const pose = armPoseForMotion(frame);
      if (pose.state !== "live") invalidFrames += 1;
      smoother.pushFrame(frame, tMs);
      currentTarget = { shoulder: pose.shoulder, forearmRelative: pose.forearmRelative };
      rowIdx += 1;
    }
    targets.push(currentTarget);
    ticks.push(smoother.sample(tMs));
  }

  return {
    replayedSamples: rows.length,
    cadenceHz: Number((rows.length / durationS).toFixed(2)),
    ticks,
    targets,
    invalidFrames,
    dropoutFrames,
  };
}

/** Added display lag: displayed-vs-input mismatch minimized over a time shift. */
function medianAddedLatencyMs(ticks, targets) {
  const maxLag = 12;
  let bestCost = Infinity;
  let bestLag = 0;
  for (let lag = 0; lag <= maxLag; lag += 1) {
    let cost = 0;
    let count = 0;
    for (let t = lag; t < ticks.length; t += 1) {
      const target = targets[t - lag];
      if (!target) continue;
      const error = Math.max(
        separationDeg(ticks[t].shoulder, target.shoulder),
        separationDeg(ticks[t].forearmRelative, target.forearmRelative),
      );
      if (Number.isFinite(error)) {
        cost += error;
        count += 1;
      }
    }
    if (count > 0 && cost / count < bestCost) {
      bestCost = cost / count;
      bestLag = lag;
    }
  }
  return Number((bestLag * TICK_MS).toFixed(2));
}

function main() {
  const logPath = process.argv[2];
  if (!logPath) {
    process.stderr.write("usage: replay_arm_avatar.mjs <session.ndjson>\n");
    process.exit(2);
  }

  const { events, rows } = parseLog(readFileSync(logPath, "utf8"));
  const axisFrame = events.some((e) => e.kind === "motion_anatomical_complete")
    ? "anatomical"
    : "sensor_neutral";

  const { ticks, targets, ...replayStats } = replay(rows, axisFrame);
  const steps = [];
  for (let i = 1; i < ticks.length; i += 1) {
    const step =
      separationDeg(ticks[i].shoulder, ticks[i - 1].shoulder) +
      separationDeg(ticks[i].forearmRelative, ticks[i - 1].forearmRelative);
    if (Number.isFinite(step)) steps.push(step);
  }
  steps.sort((a, b) => a - b);

  const angles = captureAnglesFromEvents(events);
  const metrics = {
    replayedSamples: replayStats.replayedSamples,
    cadenceHz: replayStats.cadenceHz,
    invalidFrames: replayStats.invalidFrames,
    dropoutFrames: replayStats.dropoutFrames,
    displayTicks: ticks.length,
    visualStepDistribution: {
      p50: percentile(steps, 0.5),
      p95: percentile(steps, 0.95),
      max: percentile(steps, 1),
    },
    medianAddedLatencyMs: medianAddedLatencyMs(ticks, targets),
    axisFrame,
    directionValidation: angles ? "available" : "unavailable",
  };
  if (angles) {
    metrics.sideDirectionErrorDeg = Number(
      (directionErrorDeg(rows, angles.sideAngleDeg, SIDE_AXIS) ?? NaN).toFixed(2)
    );
    metrics.forwardDirectionErrorDeg = Number(
      (directionErrorDeg(rows, angles.forwardAngleDeg, FORWARD_AXIS) ?? NaN).toFixed(2)
    );
  }

  process.stdout.write(JSON.stringify(metrics, (_key, value) => (Number.isNaN(value) ? null : value)));
  process.stdout.write("\n");
}

main();
