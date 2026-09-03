/**
 * ONNX Runtime Web integration for the live bicep-curl classifier.
 *
 * The runtime is vendored and version-pinned because the WASM execution
 * provider's SIMD/threading flags change floating-point accumulation order and
 * therefore the parity tolerance in design Section 12. Single-threaded WASM is
 * used so the page needs no cross-origin isolation (no SharedArrayBuffer).
 */

export const ORT_VERSION_PINNED = "1.20.1";
// Absolute URL derived from this module's location. ORT 1.20 resolves the
// dynamic .mjs loader import relative to ort.min.js, so a relative wasmPaths
// doubles the path (that is how /vendor/ort/vendor/ort/... happened).
export const ORT_WASM_BASE = new URL("../vendor/ort/", import.meta.url).href;

/** Bumped on every behavioral change so a stale cached page is detectable. */
export const LIVE_TOOL_BUILD = "2026-09-03.12";

/** The qualified live contract rate (design Section 11). */
export const APP_TARGET_HZ = 25;

export class ContractError extends Error {}

function requireCondition(condition, message) {
  if (!condition) throw new ContractError(message);
}

/**
 * Validate the model contract against the qualified live rate before any
 * session can start (design Section 3 "Contract status": refuse rather than
 * resample a lower-rate stream onto a higher-rate grid).
 */
export function validateContract(contract, featureNames, channelNames) {
  requireCondition(contract, "model contract not found");
  requireCondition(
    contract.preprocessing && typeof contract.preprocessing.target_hz === "number",
    "contract.preprocessing.target_hz missing"
  );
  const prep = contract.preprocessing;
  // The notebook emits "raw_synchronized_channels"/"feature_count"; the
  // curated V1 contract used "synchronized_channels"/"features". Accept both.
  const channels = prep.synchronized_channels ?? prep.raw_synchronized_channels;
  const featureCount = prep.features ?? prep.feature_count;
  requireCondition(
    prep.target_hz === APP_TARGET_HZ,
    `contract target_hz=${prep.target_hz} but the qualified live rate is ${APP_TARGET_HZ} Hz. ` +
      "Retrain at the live contract rate (design Section 11.3) and install the " +
      "v1_1 artifacts in live_tool/model/ — running a higher-rate model on a " +
      "lower-rate stream is refused, never resampled."
  );
  requireCondition(channels === 72, `expected 72 channels, got ${channels} (preprocessing keys: ${Object.keys(prep).join(", ")})`);
  requireCondition(featureCount === 576, `expected 576 features, got ${featureCount}`);
  const windowSamples = Math.round(prep.target_hz * prep.window_seconds);
  requireCondition(
    prep.window_samples === windowSamples,
    `window_samples=${prep.window_samples} is not target_hz * window_seconds (${windowSamples})`
  );
  requireCondition(Number.isInteger(prep.stride_samples), "stride_samples must be an integer (Section 3)");
  const strideSeconds = prep.stride_samples / prep.target_hz;
  if (typeof prep.stride_seconds === "number") {
    requireCondition(
      Math.abs(strideSeconds - prep.stride_seconds) < 1e-9,
      `stride_samples/${prep.target_hz}=${strideSeconds}s disagrees with stride_seconds=${prep.stride_seconds}s`
    );
  }
  requireCondition(Array.isArray(featureNames) && featureNames.length === 576, "feature_names.json must list 576 features");
  const expected = buildExpectedFeatureNames(channelNames);
  for (let i = 0; i < 576; i += 1) {
    requireCondition(
      featureNames[i] === expected[i],
      `feature order mismatch at index ${i}: contract=${featureNames[i]} constructed=${expected[i]}`
    );
  }
  return { windowSamples, strideSeconds };
}

function buildExpectedFeatureNames(channelNames) {
  const statistics = ["mean", "std", "min", "max", "range", "iqr", "rms", "mean_abs_diff"];
  const names = [];
  for (const channel of channelNames) {
    for (const statistic of statistics) names.push(`${channel}__${statistic}`);
  }
  return names;
}

export class InferenceEngine {
  constructor(ort, session, contract) {
    this.ort = ort;
    this.session = session;
    this.contract = contract;
    this.inputName = "features";
    this.labelName = "label";
    this.probabilitiesName = "probabilities";
    this.classNames = ["correct", "incomplete_range", "elbow_movement"];
    this.lastLatencyMs = null;
  }

  static async load({ contract, featureNames, channelNames, log }) {
    // ort is the vendored UMD bundle loaded by index.html.
    const ort = window.ort;
    if (!ort) throw new ContractError("ONNX Runtime Web failed to load (vendor/ort/ort.min.js)");
    ort.env.wasm.wasmPaths = ORT_WASM_BASE;
    ort.env.wasm.numThreads = 1;
    ort.env.logLevel = "error";

    validateContract(contract, featureNames, channelNames);

    const modelUrl = "./model/model.onnx";
    const session = await ort.InferenceSession.create(modelUrl, {
      executionProviders: ["wasm"],
      graphOptimizationLevel: "all",
    });

    const input = session.inputNames[0];
    requireCondition(session.inputNames.length === 1, `expected one model input, got ${session.inputNames.length}`);
    requireCondition(input === "features", `model input name is "${input}", expected "features"`);
    requireCondition(
      session.outputNames.includes("label") && session.outputNames.includes("probabilities"),
      `model outputs [${session.outputNames}] must include "label" and "probabilities"`
    );

    // Width check with a probe tensor; catches a stale ONNX artifact cheaply.
    const probe = new ort.Tensor("float32", new Float32Array(576), [1, 576]);
    const outputs = await session.run({ [input]: probe });
    const label = outputs.label;
    const probabilities = outputs.probabilities;
    requireCondition(label && label.data instanceof BigInt64Array, "label output must be int64 (BigInt64Array)");
    requireCondition(
      probabilities && probabilities.data.length === 3,
      `probabilities output must have 3 columns, got ${probabilities?.data.length}`
    );

    if (log) {
      log(`Model loaded: ${contract.model_name} @ ${contract.preprocessing.target_hz} Hz, ` +
          `window ${contract.preprocessing.window_samples} samples, stride ${contract.preprocessing.stride_samples} samples.`);
      log(`ONNX Runtime Web ${ORT_VERSION_PINNED}, WASM EP, numThreads=1 (pinned for parity, Section 12).`);
    }
    return new InferenceEngine(ort, session, contract);
  }

  /** Run one inference. Returns { classId, label, probabilities, latencyMs }. */
  async run(features) {
    if (features.length !== 576) throw new ContractError(`feature vector length ${features.length} != 576`);
    const tensor = new this.ort.Tensor("float32", features, [1, 576]);
    const startedAt = performance.now();
    const outputs = await this.session.run({ [this.inputName]: tensor });
    this.lastLatencyMs = performance.now() - startedAt;
    const labelValue = Number(outputs.label.data[0]);
    if (!Number.isInteger(labelValue) || labelValue < 0 || labelValue > 2) {
      throw new Error(`model returned invalid label ${labelValue}`);
    }
    const probabilities = new Float32Array(3);
    for (let i = 0; i < 3; i += 1) {
      const value = outputs.probabilities.data[i];
      if (!Number.isFinite(value)) throw new Error("model returned non-finite probability");
      probabilities[i] = value;
    }
    return {
      classId: labelValue,
      label: this.classNames[labelValue],
      probabilities,
      latencyMs: this.lastLatencyMs,
    };
  }
}

/** Fetch + JSON helper with useful errors. */
export async function fetchJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new ContractError(`${url}: HTTP ${response.status}`);
  return response.json();
}

export async function fetchModelBytes(url) {
  const response = await fetch(url);
  if (!response.ok) throw new ContractError(`${url}: HTTP ${response.status}`);
  return response.arrayBuffer();
}
