import assert from "node:assert/strict";
import fs from "node:fs";
import vm from "node:vm";
import { SessionLog } from "../../live_tool/js/session-log.js";
import { MotionEngine } from "../../live_tool/js/motion-engine.js";

// Exercise the real App methods without constructing browser/BLE UI objects.
const source = fs.readFileSync(new URL("../../live_tool/js/main.js", import.meta.url), "utf8");
const App = vm.runInNewContext(source.slice(source.indexOf("class App {"), source.indexOf("const app = new App();")) + "\nApp;", {
  performance, MotionEngine, MOTION_LOG_STREAM: "motion",
  SENSOR: { BNO: 1, ICM: 2 }, BNO_COLUMNS: ["qx", "qy", "qz", "qw"], ICM_COLUMNS: ["ax", "ay", "az"],
});
const app = Object.create(App.prototype);
app.sessionActive = false;
app.preprocessor = null;
app.transport = { connected: true };
app.sessionLog = new SessionLog({ byteBudget: 8192, streamShares: 7 });
app.motion = new MotionEngine();
app.armAvatar = { render() {}, renderPose() {} };
app.repAnalyzer = { pushFrame() {} };
app.ui = { renderMotion() {}, pushChartSample() {} };
app.onSample({ nodeId: 4, sensorId: 1, mappedMs: 1000, isModelStream: true,
  values: { qx: 0, qy: 0, qz: 0, qw: 1, gx: 0, gy: 0, gz: 0 } });
app.motionTick();
assert.equal(app.sessionLog.sampleCounts.get("n4s1"), 1, "raw sensor sample must log without a model/session");
assert.equal(app.sessionLog.sampleCounts.get("motion"), 1, "raw-stream motion must be replayable");
app.transport.connected = false;
app.motionTick();
assert.equal(app.sessionLog.sampleCounts.get("motion"), 1, "disconnected idle ticks must not fill the log");
console.log("Raw-stream motion logging tests passed");
