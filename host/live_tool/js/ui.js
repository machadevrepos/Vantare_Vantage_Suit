/**
 * UI rendering for the live inference page. All DOM writes live here; main.js
 * owns state and calls render with plain snapshots.
 *
 * Charts follow the four-node preview rule: BLE callbacks only append samples,
 * and a fixed-rate render loop draws. A source going stale keeps its history.
 */

import { SourceChart, SIGNAL_GROUPS } from "./charts.js";
import { FAULT_LABELS } from "./rep-analyzer.js";
import { MASTER_ID, sourceLabel } from "./ble-protocol.js";

const STATE_LABELS = {
  disconnected: { text: "Disconnected", cls: "idle" },
  connecting: { text: "Connecting", cls: "busy" },
  ready: { text: "Ready", cls: "ok" },
  warming_up: { text: "Warming Up", cls: "busy" },
  inferring: { text: "Inferring", cls: "ok" },
  degraded: { text: "Degraded", cls: "bad" },
  stopped: { text: "Stopped", cls: "idle" },
};

const CLASS_LABELS = {
  0: "Correct curl",
  1: "Incomplete range of motion",
  2: "Elbow moving forward/backward",
};

const SOURCE_ROLES = {
  0: "hub · display only",
  1: "unused by V1",
  2: "wrist / distal forearm",
  3: "elbow region",
  4: "upper arm / shoulder",
};

/** Default signal per source so the first paint is informative. */
const DEFAULT_GROUP = {
  0: "icm_accel",
  1: "icm_accel",
  2: "bno_gyro",
  3: "bno_gyro",
  4: "bno_gyro",
};

function el(id) {
  return document.getElementById(id);
}

export class Ui {
  constructor() {
    // session
    this.stateBadge = el("stateBadge");
    this.connectBtn = el("connectBtn");
    this.scanAllBtn = el("scanAllBtn");
    this.disconnectBtn = el("disconnectBtn");
    this.startBtn = el("startBtn");
    this.stopBtn = el("stopBtn");
    this.qualifyBtn = el("qualifyBtn");
    this.downloadBtn = el("downloadBtn");
    this.armBtn = el("armBtn");
    this.disarmBtn = el("disarmBtn");
    this.modelPanel = el("modelPanel");
    this.reasonLine = el("reasonLine");

    // stream control
    this.intervalInput = el("intervalInput");
    this.applyIntervalBtn = el("applyIntervalBtn");
    this.rawStartBtn = el("rawStartBtn");
    this.rawStopBtn = el("rawStopBtn");

    // actuators
    this.actuatorTarget = el("actuatorTarget");
    this.ermRange = el("ermRange");
    this.ermValue = el("ermValue");
    this.ermApplyBtn = el("ermApplyBtn");
    this.buzzerRange = el("buzzerRange");
    this.buzzerValue = el("buzzerValue");
    this.buzzerApplyBtn = el("buzzerApplyBtn");
    this.rgbRBtn = el("rgbRBtn");
    this.rgbGBtn = el("rgbGBtn");
    this.rgbBBtn = el("rgbBBtn");
    this.rgbOffBtn = el("rgbOffBtn");
    this.touchTestBtn = el("touchTestBtn");
    this.overrideOffBtn = el("overrideOffBtn");
    this.pulseIntensity = el("pulseIntensity");
    this.pulseDuration = el("pulseDuration");
    this.pulseTestBtn = el("pulseTestBtn");

    // maintenance
    this.rediscoverBtn = el("rediscoverBtn");
    this.nodeReportBtn = el("nodeReportBtn");
    this.resetRetainedBtn = el("resetRetainedBtn");

    // results
    this.classLine = el("classLine");
    this.classTag = el("classTag");
    this.probBars = [0, 1, 2].map((i) => ({ fill: el(`probFill${i}`), value: el(`probValue${i}`) }));
    this.latencyLine = el("latencyLine");
    this.cadenceLine = el("cadenceLine");
    this.skewLine = el("skewLine");
    this.warmupBar = el("warmupFill");
    this.throughputLine = el("throughputLine");

    // motion engine
    this.calibrateBtn = el("calibrateBtn");
    this.sideCalibrateBtn = el("sideCalibrateBtn");
    this.forwardCalibrateBtn = el("forwardCalibrateBtn");
    this.hingeBtn = el("hingeBtn");
    this.clearCalibrationBtn = el("clearCalibrationBtn");
    this.anatomicalNote = el("anatomicalNote");
    this.hingeNote = el("hingeNote");
    this.axisFrameValue = el("axisFrameValue");
    this.flexionValue = el("flexionValue");
    this.offAxisValue = el("offAxisValue");
    this.driftValue = el("driftValue");
    this.calibrationBadge = el("calibrationBadge");
    this.calibrationNote = el("calibrationNote");
    this.elbowValue = el("elbowValue");
    this.upperDevValue = el("upperDevValue");
    this.foreDevValue = el("foreDevValue");
    this.motionHealthValue = el("motionHealthValue");
    this.motionSkewValue = el("motionSkewValue");
    this.yawDriftValue = el("yawDriftValue");

    // reps / coach target
    this.romTargetInput = el("romTargetInput");
    this.upperArmTolInput = el("upperArmTolInput");
    this.resetRepsBtn = el("resetRepsBtn");
    this.repTally = el("repTally");
    this.repCurrent = el("repCurrent");
    this.repTableBody = el("repTable").getElementsByTagName("tbody")[0];
    this.repSummary = el("repSummary");

    // streams / log
    this.streamTable = el("streamTable").getElementsByTagName("tbody")[0];
    this.hapticStatus = el("hapticStatus");
    this.hapticHistory = el("hapticHistory");
    this.retentionLine = el("retentionLine");
    this.qualReport = el("qualReport");
    this.logBox = el("logBox");
    this.streamRows = new Map();

    // charts
    this.chartsHost = el("charts");
    this.chartWindow = el("chartWindow");
    this.chartGroupAll = el("chartGroupAll");
    this.clearChartsBtn = el("clearChartsBtn");
    this.charts = new Map(); // sourceId -> { chart, dot, foot, card }

    this.bindLocalControls();
  }

  /** Wiring that needs no application state stays here. */
  bindLocalControls() {
    this.ermRange.addEventListener("input", () => {
      this.ermValue.textContent = `${this.ermRange.value}%`;
    });
    this.buzzerRange.addEventListener("input", () => {
      this.buzzerValue.textContent = `${this.buzzerRange.value}%`;
    });
    this.chartWindow.addEventListener("change", () => {
      const seconds = Number(this.chartWindow.value);
      for (const entry of this.charts.values()) entry.chart.windowSeconds = seconds;
    });
    this.chartGroupAll.addEventListener("change", () => {
      const group = this.chartGroupAll.value;
      if (!group) return;
      for (const entry of this.charts.values()) {
        entry.chart.setGroup(group);
        entry.select.value = group;
      }
    });
    this.clearChartsBtn.addEventListener("click", () => {
      for (const entry of this.charts.values()) entry.chart.clear();
    });
  }

  get actuatorTargetId() {
    return Number(this.actuatorTarget.value);
  }

  setAppState(state, detail) {
    const def = STATE_LABELS[state] || { text: state, cls: "idle" };
    this.stateBadge.textContent = def.text;
    this.stateBadge.className = `badge ${def.cls}`;
    this.reasonLine.textContent = detail || "";
  }

  setModelPanel(lines) {
    this.modelPanel.innerHTML = "";
    for (const line of lines) {
      const div = document.createElement("div");
      div.textContent = line;
      this.modelPanel.appendChild(div);
    }
  }

  setEnabled(button, enabled) {
    button.disabled = !enabled;
  }

  /** Enable or disable the whole firmware-control surface at once. */
  setControlsEnabled(connected) {
    const controls = [
      this.applyIntervalBtn, this.rawStartBtn, this.rawStopBtn,
      this.ermApplyBtn, this.buzzerApplyBtn,
      this.rgbRBtn, this.rgbGBtn, this.rgbBBtn, this.rgbOffBtn,
      this.touchTestBtn, this.overrideOffBtn, this.pulseTestBtn,
      this.rediscoverBtn, this.nodeReportBtn, this.resetRetainedBtn,
    ];
    for (const control of controls) this.setEnabled(control, connected);
  }

  // --------------------------------------------------------- motion engine

  /**
   * Render one motion frame. Values are shown as em-dash rather than a stale
   * number whenever the frame says they are untrustworthy: a frozen angle that
   * looks live is the failure mode this panel exists to make impossible.
   */
  renderMotion(frame) {
    if (!frame) return;
    const deg = (value) => (value === null || value === undefined ? "—" : `${value.toFixed(1)}°`);
    const diagnostics = frame.diagnostics;

    const badges = {
      uncalibrated: { text: "Not calibrated", cls: "idle" },
      capturing: { text: "Hold still…", cls: "busy" },
      calibrated: { text: "Calibrated", cls: "ok" },
      failed: { text: "Calibration failed", cls: "bad" },
    };
    const badge = badges[diagnostics.calibrationState] || badges.uncalibrated;
    this.calibrationBadge.textContent = badge.text;
    this.calibrationBadge.className = `badge ${badge.cls}`;
    this.calibrationNote.textContent = diagnostics.calibrationMessage || "";

    // Ordered workflow: step 2 needs a neutral, step 3 needs the side capture.
    // Step 4 (hinge) needs only the neutral pose - its math is independent of
    // the anatomical solve, so it stays available from step 1's completion.
    this.setEnabled(this.sideCalibrateBtn, diagnostics.calibrationState === "calibrated");
    this.setEnabled(this.forwardCalibrateBtn, diagnostics.anatomicalState === "side_ready");
    this.setEnabled(this.hingeBtn, diagnostics.calibrationState === "calibrated");
    this.anatomicalNote.textContent = this.anatomicalSummary(diagnostics);
    this.axisFrameValue.textContent =
      diagnostics.axisFrame === "anatomical" ? "anatomical" : "sensor-neutral";

    const hingeBadges = {
      none: "Hinge axis not calibrated — flexion unavailable.",
      capturing: "Measuring the hinge axis: perform a few slow full reps.",
      ready: diagnostics.hingeMessage,
      failed: diagnostics.hingeMessage,
    };
    this.hingeNote.textContent = hingeBadges[diagnostics.hingeState] || "";

    // Signed flexion carries a sign, so it gets an explicit + for positive
    // values: "-25" next to "25" must not be readable as a typo.
    this.flexionValue.textContent = (() => {
      const value = frame.elbow_flexion_deg;
      if (value === null || value === undefined) {
        // Distinguish "no hinge axis yet" from "this motion is not a hinge".
        return diagnostics.flexionValid === false ? "off-hinge" : "—";
      }
      // toFixed keeps the sign of a negative zero, which renders as "-0.0".
      const fixed = value.toFixed(1);
      if (fixed === "-0.0") return "0.0°";
      return `${value > 0 ? "+" : ""}${fixed}°`;
    })();
    this.offAxisValue.textContent = deg(frame.elbow_off_axis_excess_deg);
    // Drift is a session-health number, so it is flagged rather than just shown.
    this.driftValue.textContent = diagnostics.recalibrationRecommended
      ? `${diagnostics.driftDeg.toFixed(1)}° — recalibrate`
      : deg(diagnostics.driftDeg);
    this.elbowValue.textContent = deg(frame.elbow_relative_rotation_deg);
    this.upperDevValue.textContent = deg(frame.upper_arm_deviation_deg);
    this.foreDevValue.textContent = deg(diagnostics.forearmDeviationDeg);

    const health = frame.health;
    const flag = (name, ok) => `${name}${ok ? "✓" : "✗"}`;
    this.motionHealthValue.textContent = [
      flag("N2", health.n2),
      flag("N3", health.n3),
      flag("N4", health.n4),
      flag(" sync", health.synchronized),
    ].join(" ");

    this.motionSkewValue.textContent =
      diagnostics.skewMs === null ? "—" : `${diagnostics.skewMs.toFixed(0)} ms`;
    this.yawDriftValue.textContent = deg(diagnostics.yawDriftHintDeg);
  }

  /** Instruction line plus the measured three-pose quality, when solved. */
  anatomicalSummary(diagnostics) {
    const message = diagnostics.anatomicalMessage || "";
    const quality = diagnostics.anatomicalQuality;
    if (!quality || !quality.nodes) return message;
    const parts = Object.entries(quality.nodes)
      .map(
        ([id, q]) =>
          `N${id} side ${q.sideAngleDeg.toFixed(0)}° / fwd ${q.forwardAngleDeg.toFixed(0)}°` +
          ` (sep ${q.axisSeparationDeg.toFixed(0)}°)`
      );
    if (typeof quality.segmentMismatchDeg === "number") {
      parts.push(`mismatch ${quality.segmentMismatchDeg.toFixed(1)}°`);
    }
    return `${message} (${parts.join(" · ")})`;
  }

  // ------------------------------------------------------------------ reps

  /** Append one completed rep to the table. Uncountable segments are skipped. */
  addRep(rep) {
    if (rep.index === null) return;
    const tr = this.repTableBody.insertRow(0);
    const cells = [
      String(rep.index),
      `${rep.peakFlexionDeg.toFixed(0)}°`,
      `${rep.maxUpperArmDevDeg.toFixed(0)}°`,
      `${rep.durationS.toFixed(1)}s`,
    ];
    for (const text of cells) {
      const td = tr.insertCell();
      td.textContent = text;
      td.style.textAlign = cells.indexOf(text) === 0 ? "left" : "right";
    }
    const verdict = tr.insertCell();
    verdict.style.paddingLeft = "10px";
    if (rep.faults.length === 0) {
      verdict.textContent = "correct";
      verdict.style.color = "var(--ok)";
    } else {
      // Faults are listed rather than reduced to one label: a rep can be both
      // short and swung, and the coach needs to correct both.
      verdict.textContent = rep.faults.map((f) => FAULT_LABELS[f] || f).join(" + ");
      verdict.style.color = "var(--warn)";
    }
    while (this.repTableBody.rows.length > 12) {
      this.repTableBody.deleteRow(this.repTableBody.rows.length - 1);
    }
  }

  clearReps() {
    this.repTableBody.innerHTML = "";
    this.repTally.textContent = "—";
    this.repCurrent.textContent = "";
    this.repSummary.textContent = "";
  }

  renderReps(summary, current) {
    this.repTally.textContent =
      summary.total === 0 ? "no reps yet" : `${summary.correct} / ${summary.total} correct`;
    this.repCurrent.textContent = current
      ? `rep in progress — peak so far ${current.peakFlexionDeg.toFixed(0)}°`
      : "";
    if (summary.total < 2) {
      this.repSummary.textContent = "";
      return;
    }
    const parts = [
      `mean peak ${summary.meanPeakDeg.toFixed(0)}° (sd ${summary.sdPeakDeg.toFixed(1)}°)`,
      summary.consistent ? "consistent" : "inconsistent",
    ];
    for (const [fault, count] of Object.entries(summary.faults)) {
      parts.push(`${FAULT_LABELS[fault] || fault}: ${count}`);
    }
    this.repSummary.textContent = parts.join(" · ");
  }

  // ---------------------------------------------------------------- charts

  /**
   * Create the card for a source the first time a sample arrives from it.
   * Cards are ordered MASTER first, then ascending node id.
   */
  ensureChart(sourceId) {
    let entry = this.charts.get(sourceId);
    if (entry) return entry;

    const card = document.createElement("div");
    card.className = "chart-card";
    card.dataset.sourceId = String(sourceId);

    const head = document.createElement("div");
    head.className = "chart-head";
    const dot = document.createElement("span");
    dot.className = "dot";
    const name = document.createElement("span");
    name.className = `name ${sourceId === MASTER_ID ? "master" : "node"}`;
    name.textContent = sourceLabel(sourceId);
    const role = document.createElement("span");
    role.className = "role";
    role.textContent = SOURCE_ROLES[sourceId] || "";
    const spacer = document.createElement("span");
    spacer.className = "spacer";
    const select = document.createElement("select");
    for (const [key, group] of Object.entries(SIGNAL_GROUPS)) {
      const option = document.createElement("option");
      option.value = key;
      option.textContent = group.label;
      select.appendChild(option);
    }
    head.append(dot, name, role, spacer, select);

    const canvas = document.createElement("canvas");
    const foot = document.createElement("div");
    foot.className = "chart-foot";

    card.append(head, canvas, foot);
    this.insertCardOrdered(card, sourceId);

    const chart = new SourceChart(canvas, { windowSeconds: Number(this.chartWindow.value) });
    const initialGroup = DEFAULT_GROUP[sourceId] || "bno_gyro";
    chart.setGroup(initialGroup);
    select.value = initialGroup;
    select.addEventListener("change", () => chart.setGroup(select.value));

    entry = { chart, dot, foot, card, select };
    this.charts.set(sourceId, entry);
    return entry;
  }

  insertCardOrdered(card, sourceId) {
    const existing = [...this.chartsHost.children];
    const next = existing.find((node) => Number(node.dataset.sourceId) > sourceId);
    if (next) this.chartsHost.insertBefore(card, next);
    else this.chartsHost.appendChild(card);
  }

  /** Append one decoded sample to its source chart. */
  pushChartSample(sourceId, sensorName, timeS, values) {
    this.ensureChart(sourceId).chart.push(sensorName, timeS, values);
  }

  /** Draw every chart; called from the render loop, not from BLE callbacks. */
  drawCharts(nowS, healthBySource) {
    for (const [sourceId, entry] of this.charts) {
      entry.chart.draw(nowS);
      const health = healthBySource.get(sourceId);
      if (!health) {
        entry.dot.className = "dot";
        entry.foot.textContent = "";
        continue;
      }
      const stale = health.staleMs;
      entry.dot.className = `dot ${stale === null ? "" : stale > 1000 ? "dead" : stale > 250 ? "stale" : "live"}`;
      entry.foot.innerHTML = "";
      const parts = [
        ["BNO", health.bnoRate === null ? "—" : `${health.bnoRate.toFixed(1)} Hz`],
        ["ICM", health.icmRate === null ? "—" : `${health.icmRate.toFixed(1)} Hz`],
        ["loss", health.lossPct === null ? "—" : `${(health.lossPct * 100).toFixed(0)}%`],
        ["age", stale === null ? "—" : `${stale.toFixed(0)} ms`],
      ];
      for (const [key, value] of parts) {
        const span = document.createElement("span");
        const bold = document.createElement("b");
        bold.textContent = `${key} `;
        span.append(bold, document.createTextNode(value));
        entry.foot.appendChild(span);
      }
    }
  }

  clearCharts() {
    for (const entry of this.charts.values()) entry.chart.clear();
  }

  // --------------------------------------------------------------- streams

  ensureStreamRow(key, label, isModelStream) {
    let row = this.streamRows.get(key);
    if (row) return row;
    const tr = this.streamTable.insertRow();
    tr.className = isModelStream ? "model" : "display";
    const nameCell = tr.insertCell();
    nameCell.textContent = label;
    const cells = [tr.insertCell(), tr.insertCell(), tr.insertCell(), tr.insertCell()];
    row = { cells };
    this.streamRows.set(key, row);
    return row;
  }

  renderStreams(streamSnapshots) {
    for (const snap of streamSnapshots) {
      const row = this.ensureStreamRow(snap.key, snap.label, snap.isModelStream);
      row.cells[0].textContent = snap.rate === null ? "—" : `${snap.rate.toFixed(1)} Hz`;
      row.cells[1].textContent = snap.lossPct === null ? "—" : `${(snap.lossPct * 100).toFixed(0)}%`;
      row.cells[1].className =
        snap.lossPct === null ? "" : snap.lossPct > 0.1 ? "bad" : snap.lossPct > 0.02 ? "warn" : "";
      row.cells[2].textContent = `${snap.maxGapMs.toFixed(0)} ms`;
      row.cells[3].textContent = snap.staleMs === null ? "—" : `${snap.staleMs.toFixed(0)} ms`;
      row.cells[3].className =
        snap.staleMs === null ? "" : snap.staleMs > 1000 ? "bad" : snap.staleMs > 250 ? "warn" : "";
    }
  }

  // --------------------------------------------------------------- results

  renderPrediction(prediction, inferenceUnavailableReason) {
    if (inferenceUnavailableReason) {
      this.classTag.textContent = "Result unavailable";
      this.classTag.className = "class-tag idle";
      this.classLine.textContent = inferenceUnavailableReason;
      for (const bar of this.probBars) {
        bar.fill.style.width = "0%";
        bar.value.textContent = "—";
      }
      this.latencyLine.textContent = "—";
      this.cadenceLine.textContent = "—";
      return;
    }
    const label = CLASS_LABELS[prediction.classId] || prediction.label;
    this.classTag.textContent = prediction.label;
    this.classTag.className = `class-tag c${prediction.classId}`;
    this.classLine.textContent = label;
    for (let i = 0; i < 3; i += 1) {
      const probability = prediction.probabilities[i];
      this.probBars[i].fill.style.width = `${(probability * 100).toFixed(1)}%`;
      this.probBars[i].value.textContent = `${(probability * 100).toFixed(1)}%`;
    }
    this.latencyLine.textContent = `${prediction.latencyMs.toFixed(1)} ms`;
  }

  renderCadence(perSecond, expectedPerSecond) {
    this.cadenceLine.textContent = `${perSecond.toFixed(2)}/s (exp ${expectedPerSecond.toFixed(2)})`;
  }

  renderWarmup(fraction) {
    this.warmupBar.style.width = `${Math.min(100, fraction * 100).toFixed(0)}%`;
  }

  renderSkew(skewByNode) {
    const parts = [];
    for (const [node, skew] of Object.entries(skewByNode)) {
      parts.push(`N${node} ${skew === null ? "—" : `${skew.toFixed(1)}`}`);
    }
    this.skewLine.textContent = parts.length ? `${parts.join(" · ")} ms` : "—";
  }

  renderHaptics(snapshot, options) {
    let status;
    if (snapshot.armed) status = "ARMED";
    else if (snapshot.canArm) status = "ready to arm";
    else status = "disarmed";
    this.hapticStatus.textContent =
      `${status} · streak ${snapshot.consecutiveIncorrect}/${options.requiredConsecutive}` +
      ` · thr ${(options.probabilityThreshold * 100).toFixed(0)}%` +
      ` · cooldown ${options.cooldownSeconds.toFixed(0)} s`;
    this.setEnabled(this.armBtn, !snapshot.armed && snapshot.canArm);
    this.setEnabled(this.disarmBtn, snapshot.armed);
  }

  pushHapticHistory(line) {
    const div = document.createElement("div");
    div.textContent = line;
    this.hapticHistory.prepend(div);
    while (this.hapticHistory.childElementCount > 12) {
      this.hapticHistory.removeChild(this.hapticHistory.lastChild);
    }
  }

  renderRetention(ring) {
    const seconds = ring.retentionSeconds();
    const wrapped = ring.wrappedStreams();
    this.retentionLine.textContent =
      `raw retention ≈ ${seconds > 3600 ? `${(seconds / 3600).toFixed(1)} h` : `${(seconds / 60).toFixed(0)} min`} ` +
      (wrapped.length ? `· OVERWRITING oldest — download now (${wrapped.join(", ")})` : "· no wrap yet");
  }

  renderQualification(report) {
    this.qualReport.innerHTML = "";
    for (const line of report.lines) {
      const div = document.createElement("div");
      div.textContent = line;
      if (line.startsWith("PASS")) div.className = "pass";
      if (line.startsWith("FAIL")) div.className = "fail";
      this.qualReport.appendChild(div);
    }
  }

  log(line, level = "info") {
    const div = document.createElement("div");
    div.textContent = line;
    div.className = level;
    this.logBox.appendChild(div);
    while (this.logBox.childElementCount > 400) {
      this.logBox.removeChild(this.logBox.firstChild);
    }
    this.logBox.scrollTop = this.logBox.scrollHeight;
  }

  renderThroughput(bytesPerSecond) {
    this.throughputLine.textContent = `${(bytesPerSecond / 1024).toFixed(1)} KB/s`;
  }
}
