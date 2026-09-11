/**
 * Replay a recorded session's RAW BNO streams through the MotionEngine,
 * re-running the anatomical calibration at the times the wearer performed it,
 * then report what the calibrated pose renders as during later holds.
 *
 *   node host/tests/scripts/replay_pointing_calibration.mjs <session.ndjson> <plan.json>
 *
 * plan.json (device-clock seconds from the first N4 sample):
 *   { "neutral": 6.5, "side": 14.6, "forward": 22.6,
 *     "holds": [ { "label": "forward test", "from": 61, "to": 66, "expect": "forward" } ] }
 *
 * Output per hold: arm elevation and heading (0 = straight ahead, +90 = wearer's
 * right) for N4 and N2, and the angle between the rendered palm and the floor.
 * This is the check that the avatar will show what the wearer actually did.
 */
import { readFileSync } from "node:fs";
import { MotionEngine, rotateVector } from "../../live_tool/js/motion-engine.js";

const BNO_COLUMNS = [
  "quat_i", "quat_j", "quat_k", "quat_real",
  "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
  "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
  "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
];
const NODE_OF_STREAM = { n2s1: 2, n3s1: 3, n4s1: 4 };

const [sessionPath, planPath] = process.argv.slice(2);
const plan = JSON.parse(readFileSync(planPath, "utf8"));
const rows = [];
for (const line of readFileSync(sessionPath, "utf8").split("\n")) {
  if (!line.trim()) continue;
  const r = JSON.parse(line);
  if (r.type !== "sample" || !(r.stream in NODE_OF_STREAM)) continue;
  const values = Object.fromEntries(BNO_COLUMNS.map((c, i) => [c, r.data[1 + i]]));
  rows.push({ node: NODE_OF_STREAM[r.stream], t: r.data[0], values });
}
rows.sort((a, b) => a.t - b.t);
const t0 = rows.find((r) => r.node === 4).t;

const events = [];
const engine = new MotionEngine({ onEvent: (e) => events.push(e) });
const steps = [
  ["neutral", () => engine.beginCalibration((plan.neutral + t0) * 1000)],
  ["side", () => engine.beginSideCalibration((plan.side + t0) * 1000)],
  ["forward", () => engine.beginForwardCalibration((plan.forward + t0) * 1000)],
];
let next = 0;
const holdSamples = plan.holds.map(() => []);

for (const row of rows) {
  const rel = row.t - t0;
  while (next < steps.length && rel >= plan[steps[next][0]]) {
    steps[next][1]();
    next += 1;
  }
  const nowMs = row.t * 1000;
  engine.pushSample(row.node, row.values, row.t, nowMs);
  engine.updateCalibration(nowMs);
  engine.updateAnatomicalCalibration(nowMs);
  if (row.node === 4) {
    const frame = engine.computeFrame(nowMs);
    plan.holds.forEach((hold, i) => {
      if (rel >= hold.from && rel <= hold.to && frame.diagnostics.axisFrame === "anatomical") {
        holdSamples[i].push(frame);
      }
    });
  }
}

const q = (p) => [p.qw, p.qx, p.qy, p.qz];
const DOWN = [0, 1, 0];
const describe = (dir) => {
  const [x, y, z] = dir;
  const elev = (Math.asin(Math.max(-1, Math.min(1, -y))) * 180) / Math.PI;
  const heading = (Math.atan2(x, z) * 180) / Math.PI;
  return { elevationDeg: elev, headingDeg: heading };
};
const angleDeg = (a, b) =>
  (Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) * 180) / Math.PI;

const complete = events.find((e) => e.kind === "anatomical_complete");
const report = {
  calibrated: engine.anatomicalState,
  message: engine.anatomicalMessage,
  quality: complete ? complete.quality : null,
  handRestPalm: engine.handRestPalm,
  holds: plan.holds.map((hold, i) => {
    const frames = holdSamples[i];
    if (frames.length === 0) return { label: hold.label, frames: 0 };
    const mid = frames[Math.floor(frames.length / 2)];
    const upper = q(mid.upper_arm_orientation);
    const fore = q(mid.forearm_orientation);
    const palm = engine.handRestPalm ? rotateVector(engine.handRestPalm, fore) : null;
    const upperDir = rotateVector(DOWN, upper);
    const foreDir = rotateVector(DOWN, fore);
    return {
      label: hold.label,
      frames: frames.length,
      upper: describe(upperDir),
      forearm: describe(foreDir),
      // What the avatar draws at the elbow for this hold.
      elbowBendDeg: angleDeg(upperDir, foreDir),
      palmFromFloorDeg: palm ? angleDeg(palm, DOWN) : null,
    };
  }),
};
process.stdout.write(JSON.stringify(report, null, 2));
