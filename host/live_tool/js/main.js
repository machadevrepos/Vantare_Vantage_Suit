/**
 * Application controller: state machine, health gates, inference loop,
 * qualification harness (design Sections 8, 10, 11.2).
 *
 * State machine:
 *   Disconnected -> Connecting -> Ready -> Warming Up -> Inferring
 *                                    |           ^             |
 *                                    |           |             v
 *                                    Stopped     +--------- Degraded
 *                                   (recovery re-enters Warming Up with an
 *                                    empty window buffer)
 */

import {
  BleTransport,
  BNO_COLUMNS,
  DISPLAY_SOURCE_IDS,
  ICM_COLUMNS,
  MASTER_ID,
  NODE_IDS,
  SENSOR,
  sourceLabel,
} from "./ble-protocol.js";
import { Preprocessor, buildChannelNames } from "./ml-preprocessing.js";
import {
  ContractError,
  InferenceEngine,
  LIVE_TOOL_BUILD,
  ORT_VERSION_PINNED,
  fetchJson,
  validateContract,
} from "./live-inference.js";
import { HapticController, HAPTIC_DEFAULTS } from "./haptic-controller.js";
import { SessionLog } from "./session-log.js";
import { Ui } from "./ui.js";

const CONTRACT_URL = "./model/model_contract.json";
const FEATURE_NAMES_URL = "./model/feature_names.json";

const LIVE_INTERVAL_MS = 40; // qualified live contract (Section 11)
const HEALTH_TICK_MS = 200;
const RENDER_TICK_MS = 250;
const QUALIFICATION_SECONDS = 60;

function sensorLabel(sensorId) {
  return sensorId === SENSOR.BNO ? "BNO85" : "ICM45686";
}

class App {
  constructor() {
    this.ui = new Ui();
    this.state = "disconnected";
    this.degradeReason = null;
    this.inferenceUnavailableReason = "Not connected.";

    this.contract = null;
    this.featureNames = null;
    this.engine = null;
    this.modelError = null;

    this.transport = new BleTransport({
      onSample: (sample) => this.onSample(sample),
      onLog: (line, level) => this.ui.log(line, level),
    });
    this.transport.onDisconnect = () => {
      this.stopSessionLocal("Master disconnected");
      this.setState("disconnected");
    };

    this.preprocessor = null; // constructed once the contract validates
    this.haptics = null;
    this.sessionLog = new SessionLog();
    this.channelNames = buildChannelNames();

    this.inferBusy = false;
    this.lastPrediction = null;
    this.predictionTimes = [];
    this.sessionActive = false;
    this.sessionStartedAtMs = 0;
    this.firstSampleAtMs = null;
    this.dropTimes = [];
    this.lastValidWindowEndS = null;

    this.qualification = null; // { startedAtMs, emitted, invalid, maxStaleMs }
    this.hiddenAtMs = null;

    this.bindUi();
    setInterval(() => this.healthTick(), HEALTH_TICK_MS);
    setInterval(() => this.renderTick(), RENDER_TICK_MS);
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) {
        this.hiddenAtMs = performance.now();
      } else if (this.hiddenAtMs !== null) {
        if (performance.now() - this.hiddenAtMs > 2000 && this.sessionActive) {
          this.degrade("page was suspended while streaming");
        }
        this.hiddenAtMs = null;
      }
    });
  }

  bindUi() {
    this.ui.connectBtn.addEventListener("click", () => this.onConnect());
    this.ui.scanAllBtn.addEventListener("click", () => this.onConnect({ acceptAll: true }));
    this.ui.disconnectBtn.addEventListener("click", () => this.onDisconnect());
    this.ui.startBtn.addEventListener("click", () => this.onStartSession());
    this.ui.stopBtn.addEventListener("click", () => this.onStopSession());
    this.ui.qualifyBtn.addEventListener("click", () => this.onQualify());
    this.ui.armBtn.addEventListener("click", () => {
      if (!this.haptics.arm()) this.ui.log("Arm refused: conditions not met.", "warn");
    });
    this.ui.disarmBtn.addEventListener("click", () => this.haptics.disarm("user disarmed"));
    this.ui.downloadBtn.addEventListener("click", () => this.onDownload());
    this.bindFirmwareControls();
    this.refreshButtons();
  }

  /**
   * Direct firmware control. Every handler routes through runCommand so a
   * rejected write surfaces in the log instead of an unhandled rejection, and
   * so nothing here can throw into a DOM event handler.
   */
  bindFirmwareControls() {
    const ui = this.ui;
    const target = () => ui.actuatorTargetId;

    ui.applyIntervalBtn.addEventListener("click", () =>
      this.runCommand(`set interval ${ui.intervalInput.value} ms`, () =>
        this.transport.setStreamInterval(Number(ui.intervalInput.value))
      )
    );
    ui.rawStartBtn.addEventListener("click", () =>
      this.runCommand("start stream", () => this.transport.startStream(Number(ui.intervalInput.value)))
    );
    ui.rawStopBtn.addEventListener("click", () =>
      this.runCommand("stop stream", () => this.transport.stopStream())
    );

    ui.ermApplyBtn.addEventListener("click", () =>
      this.runCommand(`ERM ${ui.ermRange.value}% -> ${sourceLabel(target())}`, () =>
        this.transport.motorPercent(target(), Number(ui.ermRange.value))
      )
    );
    ui.buzzerApplyBtn.addEventListener("click", () =>
      this.runCommand(`buzzer ${ui.buzzerRange.value}% -> ${sourceLabel(target())}`, () =>
        this.transport.buzzerPercent(target(), Number(ui.buzzerRange.value))
      )
    );
    const rgb = (mask, name) =>
      this.runCommand(`RGB ${name} -> ${sourceLabel(target())}`, () =>
        this.transport.rgbMask(target(), mask)
      );
    ui.rgbRBtn.addEventListener("click", () => rgb(0x01, "R"));
    ui.rgbGBtn.addEventListener("click", () => rgb(0x02, "G"));
    ui.rgbBBtn.addEventListener("click", () => rgb(0x04, "B"));
    ui.rgbOffBtn.addEventListener("click", () => rgb(0x00, "off"));
    ui.touchTestBtn.addEventListener("click", () =>
      this.runCommand(`touch test -> ${sourceLabel(target())}`, () => this.transport.touchTest(target()))
    );
    ui.overrideOffBtn.addEventListener("click", () =>
      this.runCommand(`release override -> ${sourceLabel(target())}`, () =>
        this.transport.clearActuatorOverride(target())
      )
    );
    ui.pulseTestBtn.addEventListener("click", () => {
      const node = target();
      if (node === MASTER_ID) {
        this.ui.log("Bounded pulse is a Node command; the Master has no pulse engine.", "warn");
        return;
      }
      this.pulseTestEventId = (this.pulseTestEventId || 0) + 1;
      // Manual test ids start high so they cannot collide with the haptic
      // controller's session counter and get suppressed as stale.
      const eventId = 0x40000000 + this.pulseTestEventId;
      this.runCommand(
        `pulse ${ui.pulseIntensity.value}% ${ui.pulseDuration.value}ms -> ${sourceLabel(node)}`,
        () =>
          this.transport.hapticPulse(
            node,
            Number(ui.pulseIntensity.value),
            Number(ui.pulseDuration.value),
            eventId
          )
      );
    });

    ui.rediscoverBtn.addEventListener("click", () =>
      this.runCommand("rediscover nodes", () => this.transport.rediscoverNodes())
    );
    ui.nodeReportBtn.addEventListener("click", () =>
      this.runCommand("node report", () => this.transport.requestDiscoveredNodes())
    );
    ui.resetRetainedBtn.addEventListener("click", () => {
      if (this.sessionActive) {
        this.ui.log("Stop the live session before resetting retained sessions.", "warn");
        return;
      }
      const confirmed = window.confirm(
        "Erase retained un-uploaded session data on every node?\n\n" +
          "This permanently discards any recording that was never collected. " +
          "Use it only when a transfer failed and a node is stuck holding data."
      );
      if (!confirmed) return;
      this.runCommand("reset retained sessions (0xB3)", () =>
        this.transport.resetRetainedSessions()
      );
    });
  }

  /** Await a control write and report the outcome; never throws. */
  async runCommand(description, action) {
    if (!this.transport.connected) {
      this.ui.log(`${description}: not connected.`, "warn");
      return;
    }
    try {
      await action();
      this.ui.log(`${description}: sent.`);
    } catch (err) {
      this.ui.log(`${description}: FAILED — ${err.message}`, "error");
    }
  }

  async init() {
    this.ui.log(`Vantare live inference tool — build ${LIVE_TOOL_BUILD}, model V1, experimental (Section 15). ORT pinned ${ORT_VERSION_PINNED}.`);
    try {
      const [contract, featureNames] = await Promise.all([
        fetchJson(CONTRACT_URL),
        fetchJson(FEATURE_NAMES_URL),
      ]);
      this.contract = contract;
      this.featureNames = featureNames;
      // Validate the contract against the qualified live rate before anything
      // else; a 50 Hz artifact must refuse to start (Section 3 Contract status).
      const { windowSamples, strideSeconds } = validateContract(contract, featureNames, this.channelNames);
      this.preprocessor = new Preprocessor({
        target_hz: contract.preprocessing.target_hz,
        window_samples: windowSamples,
        stride_samples: contract.preprocessing.stride_samples,
      });
      this.sessionLog.registerStream("n2s1", BNO_COLUMNS.length + 1);
      this.sessionLog.registerStream("n2s2", ICM_COLUMNS.length + 1);
      this.sessionLog.registerStream("n3s1", BNO_COLUMNS.length + 1);
      this.sessionLog.registerStream("n3s2", ICM_COLUMNS.length + 1);
      this.sessionLog.registerStream("n4s1", BNO_COLUMNS.length + 1);
      this.sessionLog.registerStream("n4s2", ICM_COLUMNS.length + 1);
      this.haptics = new HapticController(this.transport, this.preprocessor, {
        onEvent: (event) => this.onHapticEvent(event),
        onStateChange: () => this.renderTick(),
      });
      this.engine = await InferenceEngine.load({ contract, featureNames, channelNames: this.channelNames, log: (l) => this.ui.log(l) });
      this.ui.setModelPanel([
        `model: ${contract.model_name} (${contract.class_mapping && Object.values(contract.class_mapping).join(" / ")})`,
        `contract: ${contract.preprocessing.target_hz} Hz · ${windowSamples} samples/window · stride ${contract.preprocessing.stride_samples} samples (${strideSeconds.toFixed(2)} s)`,
        `channels: ${contract.preprocessing.synchronized_channels ?? contract.preprocessing.raw_synchronized_channels} · features: ${contract.preprocessing.features ?? contract.preprocessing.feature_count}`,
        `runtime: ONNX Runtime Web ${ORT_VERSION_PINNED}, WASM EP, numThreads=1 (pinned, Section 12)`,
        "V1 experimental personalized model — no rest/transition class. Not a general accuracy claim (Section 15).",
      ]);
      this.inferenceUnavailableReason = "Connect the Master and start a session.";
      this.ui.log(`Contract gate passed: ${contract.preprocessing.target_hz} Hz live contract (Section 11).`);
      this.refreshButtons();
    } catch (err) {
      this.modelError = err;
      const gateMessage = err instanceof ContractError ? err.message : `${err.message}`;
      this.ui.setModelPanel([
        `MODEL NOT READY: ${gateMessage}`,
        err instanceof ContractError && /target_hz/.test(err.message)
          ? `Fix: run host/notebooks/Vantare_Bicep_Curl_Training_ONNX_v1_1.ipynb in Colab (design Section 11.3), then copy vantare_bicep_curl_v1_1.onnx, model_contract_v1_1.json (renamed to model_contract.json), and feature_names_v1_1.json (renamed to feature_names.json) into host/live_tool/model/.`
          : `Fix: serve this directory over localhost and place the model artifacts in host/live_tool/model/ (see README.md).`,
      ]);
      this.inferenceUnavailableReason = gateMessage;
      this.ui.log(`Model load failed: ${gateMessage}`, "error");
      this.refreshButtons();
    }
  }

  setState(state, detail) {
    this.state = state;
    this.ui.setAppState(state, detail);
    this.refreshButtons();
  }

  refreshButtons() {
    const modelReady = this.engine !== null;
    const connected = this.transport.connected;
    const streaming = this.sessionActive;
    this.ui.setEnabled(this.ui.connectBtn, !connected);
    this.ui.setEnabled(this.ui.scanAllBtn, !connected);
    this.ui.setEnabled(this.ui.disconnectBtn, connected && !streaming);
    this.ui.setEnabled(this.ui.startBtn, modelReady && connected && !streaming);
    this.ui.setEnabled(this.ui.stopBtn, streaming);
    this.ui.setEnabled(this.ui.qualifyBtn, streaming && this.qualification === null);
    this.ui.setEnabled(this.ui.downloadBtn, this.sessionLog.events.length > 0 || this.sessionLog.rings.size > 0);
    this.ui.setControlsEnabled(connected);
    if (!modelReady) {
      this.ui.setEnabled(this.ui.armBtn, false);
      this.ui.setEnabled(this.ui.disarmBtn, false);
    }
  }

  async onConnect(options = {}) {
    this.setState("connecting");
    if (options.acceptAll) {
      this.ui.log("Scanning all nearby BLE devices (no name filter).", "warn");
    }
    try {
      await this.transport.connect(options);
      this.setState("ready", this.modelError ? `Model not ready: ${this.modelError.message}` : "Connected. Start a session.");
    } catch (err) {
      this.ui.log(`Connect failed: ${err.message}`, "error");
      this.setState("disconnected", err.message);
    }
  }

  onDisconnect() {
    this.transport.disconnect();
    this.stopSessionLocal("disconnected by user");
    this.setState("disconnected");
  }

  async onStartSession() {
    if (!this.engine || !this.preprocessor) return;
    this.transport.resetSessionState();
    this.preprocessor.resetSession();
    this.haptics.resetSession();
    this.sessionLog.startSession();
    this.predictionTimes = [];
    this.lastPrediction = null;
    this.lastValidWindowEndS = null;
    this.firstSampleAtMs = null;
    this.sessionActive = true;
    this.sessionStartedAtMs = performance.now();
    this.dropTimes = [];
    this.inferenceUnavailableReason = "Warming up: need two seconds of healthy data on all six streams.";
    try {
      await this.transport.startStream(LIVE_INTERVAL_MS);
      this.setState("warming_up", "Waiting for the first complete healthy window.");
    } catch (err) {
      this.sessionActive = false;
      this.ui.log(`Stream start failed: ${err.message}`, "error");
      this.setState("ready", `Stream start failed: ${err.message}`);
    }
    this.refreshButtons();
  }

  async onStopSession() {
    try {
      await this.transport.stopStream();
    } catch (err) {
      this.ui.log(`Stream stop failed: ${err.message}`, "warn");
    }
    this.stopSessionLocal("session stopped");
  }

  stopSessionLocal(reason) {
    if (this.haptics) this.haptics.resetSession();
    const wasActive = this.sessionActive;
    this.sessionActive = false;
    this.qualification = null;
    this.inferenceUnavailableReason = reason;
    if (wasActive) this.sessionLog.event({ kind: "session_stop", reason });
    if (this.transport.connected) {
      this.setState("ready", `${reason}. Log remains available for download.`);
    }
    this.refreshButtons();
  }

  onSample(sample) {
    const values = sample.values;
    // Charting happens for every decoded source, MASTER included, and BEFORE
    // the session gate: raw streaming can be driven from the Stream Control
    // panel without starting an inference session, and the signals should be
    // visible then. Only the model path below needs an active session.
    this.ui.pushChartSample(
      sample.nodeId,
      sample.sensorId === SENSOR.BNO ? "bno" : "icm",
      sample.mappedMs / 1000,
      values
    );
    if (!sample.isModelStream) return;
    if (!this.sessionActive || !this.preprocessor) return;
    if (this.firstSampleAtMs === null) this.firstSampleAtMs = performance.now();
    const key = `n${sample.nodeId}s${sample.sensorId}`;
    this.sessionLog.logSample(
      key,
      sample.mappedMs / 1000,
      Object.values(
        sample.sensorId === SENSOR.BNO
          ? Object.fromEntries(BNO_COLUMNS.map((c) => [c, values[c]]))
          : Object.fromEntries(ICM_COLUMNS.map((c) => [c, values[c]]))
      )
    );
    const accepted = this.preprocessor.pushSample(
      sample.nodeId,
      sample.sensorId,
      sample.mappedMs / 1000,
      sample.sequence,
      values
    );
    if (!accepted) {
      // A single non-monotonic sample (for example at time-base handoff) is
      // dropped; only repeated backwards timestamps degrade the session
      // (Section 10: "timestamps move backward or cannot be aligned").
      const now = performance.now();
      this.dropTimes.push(now);
      while (this.dropTimes.length > 0 && now - this.dropTimes[0] > 2000) this.dropTimes.shift();
      if (this.dropTimes.length >= 3) {
        this.degrade(`repeated backwards timestamps on N${sample.nodeId} ${sensorLabel(sample.sensorId)}`);
      }
      return;
    }
    if (this.inferBusy) return;
    const result = this.preprocessor.maybeEmit();
    if (result.status === "emitted") {
      this.onValidWindow(result.window);
    } else if (result.status === "invalid") {
      this.sessionLog.event({ kind: "window_invalid", reason: result.reason, endS: result.endS });
      if (this.qualification) this.qualification.invalid += 1;
    }
  }

  async onValidWindow(windowData) {
    if (this.qualification) this.qualification.emitted += 1;
    if (this.state === "warming_up") {
      this.setState("inferring");
      this.haptics.setEnabled(true);
      this.inferenceUnavailableReason = null;
      this.ui.log("Warm-up complete: inference active.");
    }
    this.inferBusy = true;
    try {
      const prediction = await this.engine.run(windowData.features);
      this.lastPrediction = prediction;
      this.predictionTimes.push(performance.now());
      while (this.predictionTimes.length > 0 && performance.now() - this.predictionTimes[0] > 5000) {
        this.predictionTimes.shift();
      }
      const decision = this.haptics.evaluate(prediction, windowData.endS);
      this.sessionLog.event({
        kind: "prediction",
        windowStartS: windowData.startS,
        windowEndS: windowData.endS,
        classId: prediction.classId,
        label: prediction.label,
        probabilities: Array.from(prediction.probabilities),
        latencyMs: prediction.latencyMs,
        hapticTrigger: decision.fired,
        consecutiveIncorrect: decision.consecutiveIncorrect,
      });
      this.lastValidWindowEndS = windowData.endS;
      this.inferenceUnavailableReason = null;
    } catch (err) {
      this.ui.log(`Inference failed: ${err.message}`, "error");
      this.degrade(`inference error: ${err.message}`);
    } finally {
      this.inferBusy = false;
    }
  }

  /**
   * Keys of the six streams the model consumes. MASTER and N1 are charted but
   * must never gate inference: their absence or staleness is not a fault.
   */
  modelStreamKeys() {
    const keys = [];
    for (const node of NODE_IDS) {
      for (const sensor of [SENSOR.BNO, SENSOR.ICM]) keys.push(`n${node}s${sensor}`);
    }
    return keys;
  }

  /** Section 10 health gates, evaluated against T = 1000 / target_hz. */
  healthTick() {
    if (!this.preprocessor || !this.transport.connected || !this.sessionActive) return;
    const periodMs = 1000 / this.preprocessor.targetHz;
    const staleLimitMs = 3 * periodMs;
    const skewLimitMs = 0.5 * periodMs;
    const now = performance.now();

    if (this.state === "degraded") {
      // Recovery requires fresh healthy data and a new complete window with an
      // empty buffer (Section 10).
      const allFresh = NODE_IDS.every((node) =>
        [SENSOR.BNO, SENSOR.ICM].every((sensor) => {
          const health = this.transport.health.get(`n${node}s${sensor}`);
          return health && health.staleMs(now) < staleLimitMs;
        })
      );
      if (allFresh) {
        this.preprocessor.resetSession();
        this.lastPrediction = null;
        this.inferenceUnavailableReason = "Recovering: warming up again after degradation.";
        this.setState("warming_up", "Recovered from degradation; warming up with an empty buffer.");
        this.ui.log("Health recovered: re-entering warm-up with an empty window buffer.");
      }
      return;
    }

    for (const key of this.modelStreamKeys()) {
      const health = this.transport.health.get(key);
      if (!health) continue;
      if (health.staleMs(now) > staleLimitMs) {
        this.degrade(`stream ${key} stale for ${health.staleMs(now).toFixed(0)} ms (limit ${staleLimitMs.toFixed(0)} ms)`);
        return;
      }
    }
    for (const node of NODE_IDS) {
      const skew = this.transport.timeBase.skewMs(node);
      if (skew !== null && skew > skewLimitMs) {
        this.degrade(`N${node} clock skew ${skew.toFixed(1)} ms exceeds ${skewLimitMs.toFixed(0)} ms`);
        return;
      }
    }
    // Congestion demotion detection (Section 6.2) requires the firmware to
    // report decimated/dropped counters; the current firmware does not, so the
    // browser infers rate loss from stream rate instead: a sustained effective
    // rate below the contract is a rate-contract violation. Require ten
    // seconds of data so the session-start ramp cannot false-trigger.
    for (const key of this.modelStreamKeys()) {
      const health = this.transport.health.get(key);
      if (!health) continue;
      const elapsedMs = now - (health.firstRecvMs || now);
      const rate = health.averageRate();
      if (health.received > 250 && elapsedMs > 10000 && rate < 0.9 * this.preprocessor.targetHz) {
        this.degrade(`stream ${key} effective rate ${rate.toFixed(1)} Hz below contract (${this.preprocessor.targetHz} Hz) — congestion demotion?`);
        return;
      }
    }
    this.haptics.setHealthy(true);
  }

  degrade(reason) {
    if (!this.sessionActive) return;
    if (this.state === "degraded") return;
    this.degradeReason = reason;
    this.inferenceUnavailableReason = `Degraded: ${reason}`;
    this.haptics.setHealthy(false); // disarms if armed
    this.haptics.setEnabled(false);
    this.sessionLog.event({ kind: "degraded", reason });
    this.ui.log(`DEGRADED: ${reason}`, "warn");
    this.setState("degraded", reason);
  }

  onHapticEvent(event) {
    this.sessionLog.event({ kind: "haptic", ...event });
    if (event.kind === "pulse_start") {
      this.ui.pushHapticHistory(
        `#${event.eventId} N${event.targetNode} ${event.classId === 1 ? "incomplete_range" : "elbow_movement"} ` +
          `${event.intensityPercent}% / ${event.durationMs} ms`
      );
    } else if (event.kind === "pulse_failed" || event.kind === "pulse_off_failed") {
      this.ui.pushHapticHistory(`#${event.eventId} FAILED: ${event.reason}`);
    } else if (event.kind === "arm") {
      this.ui.pushHapticHistory("armed by user");
    } else if (event.kind === "disarm") {
      this.ui.pushHapticHistory(`disarmed (${event.reason})`);
    }
  }

  onDownload() {
    const blob = this.sessionLog.buildDownload({
      contract: this.contract && this.contract.model_name,
      ortVersion: ORT_VERSION_PINNED,
      targetHz: this.preprocessor ? this.preprocessor.targetHz : null,
    });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `vantage_live_${new Date().toISOString().replace(/[:.]/g, "-")}.ndjson`;
    anchor.click();
    URL.revokeObjectURL(url);
    this.ui.log("Session log download started.");
  }

  onQualify() {
    if (!this.sessionActive || this.qualification) return;
    this.qualification = { startedAtMs: performance.now(), emitted: 0, invalid: 0 };
    for (const health of this.transport.health.values()) health.maxGapMs = 0;
    this.ui.log(`Qualification run started: ${QUALIFICATION_SECONDS} s at the ${LIVE_INTERVAL_MS} ms contract interval.`);
    setTimeout(() => this.finishQualification(), QUALIFICATION_SECONDS * 1000);
  }

  finishQualification() {
    if (!this.qualification) return;
    const qual = this.qualification;
    this.qualification = null;
    const periodMs = this.preprocessor ? 1000 / this.preprocessor.targetHz : LIVE_INTERVAL_MS;
    const lines = [`Qualification run (${QUALIFICATION_SECONDS} s @ ${LIVE_INTERVAL_MS} ms interval, Section 11.2):`];
    let pass = true;
    const scheduled = QUALIFICATION_SECONDS / (this.preprocessor ? this.preprocessor.strideSeconds : 0.48);
    const windowsConsidered = qual.emitted + qual.invalid;
    const validPct = windowsConsidered > 0 ? (100 * qual.emitted) / windowsConsidered : 0;
    const validLine = `valid inference windows: ${validPct.toFixed(1)}% of ${windowsConsidered} (scheduled ≈ ${scheduled.toFixed(0)})`;
    if (validPct < 95) {
      pass = false;
      lines.push(`FAIL ${validLine} — below 95%`);
    } else {
      lines.push(`PASS ${validLine}`);
    }
    for (const node of NODE_IDS) {
      for (const sensor of [SENSOR.BNO, SENSOR.ICM]) {
        const key = `n${node}s${sensor}`;
        const health = this.transport.health.get(key);
        if (!health || health.received < 2) {
          pass = false;
          lines.push(`FAIL ${key}: no data`);
          continue;
        }
        const rate = health.averageRate();
        const lossPct = (100 * health.lost) / Math.max(1, health.received + health.lost);
        const rateLine = `${key}: ${rate.toFixed(2)} Hz avg, ${lossPct.toFixed(2)}% seq loss, max inter-packet ${health.maxGapMs.toFixed(0)} ms`;
        if (rate < 23.75 || lossPct > 1.0 || health.maxGapMs > 3 * periodMs) {
          pass = false;
          lines.push(`FAIL ${rateLine}`);
        } else {
          lines.push(`PASS ${rateLine}`);
        }
      }
    }
    let maxSkew = 0;
    for (const node of NODE_IDS) {
      const skew = this.transport.timeBase.skewMs(node);
      if (skew !== null) maxSkew = Math.max(maxSkew, skew);
    }
    const skewLine = `max cross-node skew estimate: ${maxSkew.toFixed(1)} ms (limit ${0.5 * periodMs} ms)`;
    if (maxSkew > 0.5 * periodMs) {
      pass = false;
      lines.push(`FAIL ${skewLine}`);
    } else {
      lines.push(`PASS ${skewLine}`);
    }
    lines.push(`transport throughput: ${(this.transport.rxBytes / 1000 / QUALIFICATION_SECONDS).toFixed(1)} KB/s average`);
    if (pass) lines.unshift("PASS — qualification criteria met. Proceed per Section 14 step 5.");
    else lines.unshift("FAIL — do not proceed to model integration on this rate (Section 11.2).");
    this.ui.renderQualification({ lines });
    for (const line of lines) this.ui.log(line, pass ? "info" : "warn");
    this.sessionLog.event({ kind: "qualification", pass, lines });
    this.refreshButtons();
  }

  renderTick() {
    const now = performance.now();
    const periodMs = this.preprocessor ? 1000 / this.preprocessor.targetHz : LIVE_INTERVAL_MS;
    const streamSnapshots = [];
    const healthBySource = new Map();
    for (const source of DISPLAY_SOURCE_IDS) {
      const perSensor = {};
      let seen = false;
      for (const sensor of [SENSOR.BNO, SENSOR.ICM]) {
        const key = `n${source}s${sensor}`;
        const health = this.transport.health.get(key);
        if (!health) continue;
        seen = true;
        const rate = health.received >= 2 ? health.averageRate() : null;
        perSensor[sensor] = { rate, staleMs: health.staleMs(now), lost: health.lost };
        streamSnapshots.push({
          key,
          label: `${sourceLabel(source)} ${sensorLabel(sensor)}`,
          isModelStream: NODE_IDS.includes(source),
          rate,
          lost: health.lost,
          maxGapMs: health.maxGapMs,
          staleMs: this.transport.connected ? Math.min(health.staleMs(now), 9999) : null,
        });
      }
      if (!seen) continue;
      const bno = perSensor[SENSOR.BNO];
      const icm = perSensor[SENSOR.ICM];
      const ages = [bno, icm].filter(Boolean).map((entry) => entry.staleMs);
      healthBySource.set(source, {
        bnoRate: bno ? bno.rate : null,
        icmRate: icm ? icm.rate : null,
        lost: (bno ? bno.lost : 0) + (icm ? icm.lost : 0),
        staleMs: ages.length ? Math.min(...ages) : null,
      });
    }
    this.ui.renderStreams(streamSnapshots);
    this.ui.drawCharts(now / 1000, healthBySource);
    this.ui.renderSkew(
      Object.fromEntries(
        NODE_IDS.map((node) => [node, this.transport.timeBase.skewMs(node)])
      )
    );
    this.ui.renderThroughput(this.transport.rxBytes / 1000 / Math.max(1, (now - (this.sessionStartedAtMs || now)) / 1000));
    if (this.preprocessor) {
      const warmupFraction = this.firstSampleAtMs
        ? (now - this.firstSampleAtMs) / (this.preprocessor.windowSeconds * 1000)
        : 0;
      this.ui.renderWarmup(warmupFraction);
    }
    let unavailable = null;
    if (this.state !== "inferring") {
      unavailable = this.inferenceUnavailableReason || `State: ${this.state}`;
    } else if (!this.lastPrediction) {
      unavailable = "Waiting for the first inference result.";
    }
    this.ui.renderPrediction(this.lastPrediction, unavailable);
    const cadencePerSecond = this.predictionTimes.length / 5;
    const expected = this.preprocessor ? 1 / this.preprocessor.strideSeconds : 1 / 0.48;
    this.ui.renderCadence(cadencePerSecond, expected);
    if (this.haptics) {
      this.ui.renderHaptics(this.haptics.snapshot(), HAPTIC_DEFAULTS);
    }
    this.ui.renderRetention(this.sessionLog);
  }
}

const app = new App();
app.init();

/* Debug handle for bench work: lets you replay decoded samples into the page
 * without hardware, e.g.
 *   vantare.onSample({nodeId:0, sensorId:2, isModelStream:false,
 *                     mappedMs: performance.now(), sequence:1, values:{...}})
 * It is a read/write handle to live state; nothing in the app depends on it. */
window.vantare = app;
