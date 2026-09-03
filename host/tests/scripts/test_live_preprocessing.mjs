/**
 * Functional fixtures for the live preprocessing port (design Section 12,
 * "Browser unit tests"). Runs with plain node, no framework:
 *
 *   node host/tests/scripts/test_live_preprocessing.mjs
 *
 * The fixtures use analytic signals with closed-form statistics, so a failure
 * pins the defect to a specific stage (statistics, grid anchoring,
 * interpolation, channel assembly, or a Section 10 gate).
 */
import assert from "node:assert/strict";
import {
  Preprocessor,
  buildChannelNames,
  buildFeatureNames,
  columnStatistics,
  percentileLinear,
} from "../../live_tool/js/ml-preprocessing.js";

const CHANNELS = buildChannelNames();
const FEATURES = buildFeatureNames(CHANNELS);
const NODES = [2, 3, 4];
const PERIOD = 0.04; // 25 Hz contract

function featureIndex(name) {
  const index = FEATURES.indexOf(name);
  assert.notEqual(index, -1, `expected feature ${name} to exist`);
  return index;
}

function contract() {
  return { target_hz: 25, window_samples: 50, stride_samples: 12 };
}

/** Push one sample time across all six streams; per-stream overrides allowed. */
function pushAll(pre, i, overrides = {}, timeOverride = null) {
  const t = timeOverride === null ? i * PERIOD : timeOverride;
  for (const node of NODES) {
    for (const sensor of [1, 2]) {
      const options = overrides[`${node}:${sensor}`] || {};
      if (options.skip && options.skip(i)) continue;
      const values =
        sensor === 1
          ? {
              quat_i: options.quat ? options.quat[0] : 1,
              quat_j: options.quat ? options.quat[1] : 0,
              quat_k: options.quat ? options.quat[2] : 0,
              quat_real: options.quat ? options.quat[3] : 0,
              linear_accel_x_mps2: options.linX ? options.linX(t) : 1,
              linear_accel_y_mps2: 0,
              linear_accel_z_mps2: 0,
              gravity_x_mps2: 0,
              gravity_y_mps2: 0,
              gravity_z_mps2: 0,
              gyro_x_radps: 0,
              gyro_y_radps: 0,
              gyro_z_radps: 0,
            }
          : {
              accel_x_g: 4,
              accel_y_g: 0,
              accel_z_g: 0,
              gyro_x_dps: 0,
              gyro_y_dps: 0,
              gyro_z_dps: 0,
            };
      assert.equal(pre.pushSample(node, sensor, t, i, values), true);
    }
  }
}

function testColumnStatistics() {
  const stats = columnStatistics([1, 2, 3, 4]);
  assert.equal(stats.length, 8);
  assert.equal(stats[0], 2.5); // mean
  assert.ok(Math.abs(stats[1] - Math.sqrt(1.25)) < 1e-12); // population std
  assert.equal(stats[2], 1);
  assert.equal(stats[3], 4);
  assert.equal(stats[4], 3); // range
  assert.ok(Math.abs(stats[5] - 1.5) < 1e-12); // iqr = 3.25 - 1.75
  assert.ok(Math.abs(stats[6] - Math.sqrt(7.5)) < 1e-12); // rms
  assert.ok(Math.abs(stats[7] - 1) < 1e-12); // mean abs diff
  assert.ok(Math.abs(percentileLinear([1, 2, 3, 4], 25) - 1.75) < 1e-12);
  assert.ok(Math.abs(percentileLinear([1, 2, 3, 4], 75) - 3.25) < 1e-12);
}

function testFeatureOrdering() {
  assert.equal(CHANNELS.length, 72);
  assert.equal(FEATURES.length, 576);
  // Notebook order: per node BNO(13), ICM(6), four magnitudes; then angles.
  assert.equal(CHANNELS[0], "n2_bno_quat_i");
  assert.equal(CHANNELS[13], "n2_icm_accel_x_g");
  assert.equal(CHANNELS[19], "n2_bno_linear_accel_mag");
  assert.equal(CHANNELS[20], "n2_bno_gyro_mag");
  assert.equal(CHANNELS[21], "n2_icm_accel_mag");
  assert.equal(CHANNELS[22], "n2_icm_gyro_mag");
  assert.equal(CHANNELS[23], "n3_bno_quat_i");
  assert.equal(CHANNELS[69], "relative_angle_n2_n3_deg");
  assert.equal(CHANNELS[70], "relative_angle_n3_n4_deg");
  assert.equal(CHANNELS[71], "relative_angle_n2_n4_deg");
  assert.equal(FEATURES[0], "n2_bno_quat_i__mean");
  assert.equal(FEATURES[8], "n2_bno_quat_j__mean");
  assert.equal(FEATURES[575], "relative_angle_n2_n4_deg__mean_abs_diff");
}

function testConstantSession() {
  const pre = new Preprocessor(contract());
  let result = { status: "waiting" };
  for (let i = 0; i < 51 && result.status !== "emitted"; i += 1) {
    pushAll(pre, i);
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "emitted", `expected first window at t=2.0, got ${result.status}`);
  // Grid end is common_end = 2.0 s (50 samples: 0.00 .. 1.96).
  assert.ok(Math.abs(result.window.endS - 2.0) < 1e-9);
  assert.ok(Math.abs(result.window.startS - 0.04) < 1e-9);
  const features = result.window.features;
  assert.equal(features.length, 576);
  for (const [name, expected] of [
    ["n2_bno_quat_i__mean", 1],
    ["n2_bno_quat_i__std", 0],
    ["n3_bno_quat_real__max", 0],
    ["n2_bno_linear_accel_x_mps2__mean", 1],
    ["n2_bno_linear_accel_mag__rms", 1],
    ["n2_icm_accel_x_g__mean", 4],
    ["n2_icm_accel_mag__mean", 4],
    ["n4_icm_gyro_mag__mean_abs_diff", 0],
    ["relative_angle_n2_n3_deg__mean", 0],
    ["relative_angle_n3_n4_deg__max", 0],
    ["relative_angle_n2_n4_deg__mean_abs_diff", 0],
  ]) {
    assert.ok(Math.abs(features[featureIndex(name)] - expected) < 1e-6, `${name}=${features[featureIndex(name)]} != ${expected}`);
  }
  // Stride gate: the next window comes only a full stride later (t = 2.48).
  result = pre.maybeEmit();
  assert.equal(result.status, "waiting");
  for (let i = 51; i <= 61; i += 1) {
    pushAll(pre, i);
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "waiting", "window at t=2.44 must not emit (stride 0.48 s)");
  pushAll(pre, 62);
  result = pre.maybeEmit();
  assert.equal(result.status, "emitted", "window at t=2.48 must emit");
  assert.ok(Math.abs(result.window.endS - 2.48) < 1e-9);
}

function testGridAnchoredInterpolation() {
  // N2 BNO linear acceleration is a ramp linX(t) = 100 t. Samples sit exactly
  // on the 25 Hz grid, so interpolated grid values equal 100 * grid time and
  // the window mean pins the grid anchor: mean = 100 * mean(startS..endS).
  const pre = new Preprocessor(contract());
  let result = { status: "waiting" };
  for (let i = 0; i < 51 && result.status !== "emitted"; i += 1) {
    pushAll(pre, i, { "2:1": { linX: (t) => 100 * t } });
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "emitted");
  const expectedMean = 100 * ((0.04 + 2.0) / 2);
  const actual = result.window.features[featureIndex("n2_bno_linear_accel_x_mps2__mean")];
  assert.ok(Math.abs(actual - expectedMean) < 1e-3, `ramp mean ${actual} != ${expectedMean}`);
  // The y channel stays constant: interpolation must not smear it.
  assert.ok(Math.abs(result.window.features[featureIndex("n2_bno_linear_accel_y_mps2__mean")]) < 1e-12);
}

function testJitteredSamplesUseNearestSample() {
  // Samples offset by +/- 10 ms around the grid carry a two-level signal.
  // Nearest-sample decimation emits only values that a sensor actually
  // reported, so min and max land exactly on the two levels. Linear
  // interpolation would blend them and pull both toward the middle, so this
  // fixture fails if the pipeline ever goes back to interpolating.
  const jitter = (i) => (i % 2 === 0 ? -0.01 : 0.01);
  const pre = new Preprocessor(contract());
  let result = { status: "waiting" };
  for (let i = 0; i < 53 && result.status !== "emitted"; i += 1) {
    pushAll(
      pre,
      i,
      { "2:1": { linX: (t) => (Math.round(t / PERIOD) % 2 === 0 ? 0 : 10) } },
      i * PERIOD + jitter(i)
    );
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "emitted");
  const min = result.window.features[featureIndex("n2_bno_linear_accel_x_mps2__min")];
  const max = result.window.features[featureIndex("n2_bno_linear_accel_x_mps2__max")];
  assert.ok(Math.abs(min - 0) < 1e-6, `nearest-sample min ${min} should be exactly 0`);
  assert.ok(Math.abs(max - 10) < 1e-6, `nearest-sample max ${max} should be exactly 10`);
}

function testJitteredRampStaysWithinJitterBound() {
  // A ramp reconstructed from real samples is off by at most slope * jitter.
  const jitter = (i) => (i % 2 === 0 ? -0.01 : 0.01);
  const pre = new Preprocessor(contract());
  let result = { status: "waiting" };
  for (let i = 0; i < 53 && result.status !== "emitted"; i += 1) {
    pushAll(pre, i, { "2:1": { linX: (t) => 100 * t } }, i * PERIOD + jitter(i));
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "emitted");
  const actual = result.window.features[featureIndex("n2_bno_linear_accel_x_mps2__mean")];
  const expectedMean = 100 * ((result.window.startS + result.window.endS) / 2);
  assert.ok(
    Math.abs(actual - expectedMean) <= 100 * 0.01 + 1e-6,
    `jittered ramp mean ${actual} vs ${expectedMean}`
  );
}

function testInterpSpanGateOnLoss() {
  // Drop three consecutive ICM samples on N3: the 160 ms gap exceeds the
  // 1.5T (60 ms) interpolation span and 3 missing > 2% of 50, so the window
  // must be invalidated with the stream named, not silently produced.
  const pre = new Preprocessor(contract());
  const overrides = { "3:2": { skip: (i) => i === 25 || i === 26 || i === 27 } };
  let result = { status: "waiting" };
  for (let i = 0; i < 60 && result.status === "waiting"; i += 1) {
    pushAll(pre, i, overrides);
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "invalid", "window with a 3-sample gap must be invalid");
  assert.equal(result.reason, "interp_span_n3s2");
}

function testHapticBlankingGate() {
  const pre = new Preprocessor(contract());
  pre.registerBlanking(3, 0.5, 0.3); // N3 pulse, 250 ms + 250 ms margin
  let result = { status: "waiting" };
  for (let i = 0; i < 51 && result.status === "waiting"; i += 1) {
    pushAll(pre, i);
    result = pre.maybeEmit();
  }
  // The first window (0.04..2.00) intersects the blanking interval.
  assert.equal(result.status, "invalid");
  assert.equal(result.reason, "haptic_blanking_n3");
}

function testQuaternionNormalization() {
  const pre = new Preprocessor(contract());
  const quat = [2, 0, 0, 0]; // non-unit; must normalize to (1, 0, 0, 0)
  let result = { status: "waiting" };
  for (let i = 0; i < 51 && result.status !== "emitted"; i += 1) {
    pushAll(pre, i, {
      "2:1": { quat },
      "3:1": { quat },
      "4:1": { quat },
    });
    result = pre.maybeEmit();
  }
  assert.equal(result.status, "emitted");
  const features = result.window.features;
  assert.ok(Math.abs(features[featureIndex("n2_bno_quat_i__mean")] - 1) < 1e-9);
  assert.ok(Math.abs(features[featureIndex("relative_angle_n2_n3_deg__mean")]) < 1e-9);
}

function testNonMonotonicRejected() {
  const pre = new Preprocessor(contract());
  pushAll(pre, 0);
  const stream = { quat_i: 1, quat_j: 0, quat_k: 0, quat_real: 0, linear_accel_x_mps2: 1, linear_accel_y_mps2: 0, linear_accel_z_mps2: 0, gravity_x_mps2: 0, gravity_y_mps2: 0, gravity_z_mps2: 0, gyro_x_radps: 0, gyro_y_radps: 0, gyro_z_radps: 0 };
  assert.equal(pre.pushSample(2, 1, 0.04, 1, stream), true);
  assert.equal(pre.pushSample(2, 1, 0.04, 2, stream), false, "duplicate timestamp must be rejected");
  assert.equal(pre.pushSample(2, 1, 0.02, 3, stream), false, "backwards timestamp must be rejected");
}

const tests = {
  testColumnStatistics,
  testFeatureOrdering,
  testConstantSession,
  testGridAnchoredInterpolation,
  testJitteredSamplesUseNearestSample,
  testJitteredRampStaysWithinJitterBound,
  testInterpSpanGateOnLoss,
  testHapticBlankingGate,
  testQuaternionNormalization,
  testNonMonotonicRejected,
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    fn();
    console.log(`PASS ${name}`);
  } catch (err) {
    failed += 1;
    console.error(`FAIL ${name}: ${err.message}`);
  }
}
if (failed > 0) {
  console.error(`${failed} test(s) failed`);
  process.exit(1);
}
console.log(`All ${Object.keys(tests).length} preprocessing fixture tests passed.`);
