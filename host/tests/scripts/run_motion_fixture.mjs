/**
 * Motion Engine harness runner.
 *
 * Reads a fixture JSON produced by
 * host/tests/python/test_motion_engine.py, replays each scenario's samples
 * through the browser MotionEngine with a fully controlled clock, and prints
 * the resulting motion frames as JSON on stdout.
 *
 * The clock is driven by the fixture (every sample carries nowMs) rather than
 * performance.now(), so stillness holds, staleness and calibration timeouts are
 * deterministic and the test never depends on wall-clock timing.
 *
 *   node host/tests/scripts/run_motion_fixture.mjs <fixture.json>
 */
import { readFileSync } from "node:fs";
import { MotionEngine } from "../../live_tool/js/motion-engine.js";

const fixturePath = process.argv[2];
if (!fixturePath) {
  process.stderr.write("usage: run_motion_fixture.mjs <fixture.json>\n");
  process.exit(2);
}

const fixture = JSON.parse(readFileSync(fixturePath, "utf8"));
const results = [];

for (const scenario of fixture.scenarios) {
  const events = [];
  const engine = new MotionEngine({
    ...(scenario.options || {}),
    onEvent: (event) => events.push(event),
  });
  const frames = [];

  for (const step of scenario.steps) {
    if (step.op === "beginCalibration") {
      engine.beginCalibration(step.nowMs);
    } else if (step.op === "clearCalibration") {
      engine.clearCalibration();
    } else if (step.op === "beginHinge") {
      engine.beginHingeCalibration(step.nowMs);
    } else if (step.op === "sample") {
      engine.pushSample(step.node, step.values, step.deviceS, step.nowMs);
      // main.js drives updateCalibration from the render tick; the harness
      // pumps it after every sample so a hold can complete mid-stream.
      engine.updateCalibration(step.nowMs);
    } else if (step.op === "frame") {
      const frame = engine.computeFrame(step.nowMs);
      frames.push({ label: step.label, frame, logRow: engine.toLogRow(frame) });
    } else {
      process.stderr.write(`unknown op: ${step.op}\n`);
      process.exit(3);
    }
  }

  results.push({
    name: scenario.name,
    frames,
    events,
    calibrationState: engine.state,
    calibrationMessage: engine.calibrationMessage,
    hingeState: engine.hingeState,
    hingeMessage: engine.hingeMessage,
    hingeQuality: engine.hingeQuality,
  });
}

process.stdout.write(JSON.stringify({ scenarios: results, logColumns: MotionEngine.LOG_COLUMNS }));
