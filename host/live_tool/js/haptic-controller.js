/**
 * Manually armed haptic feedback controller (design Sections 9, 9.1, 15).
 *
 * Gate rules mirror the notebook's HapticGate: an incorrect class triggers
 * only when the same class reaches >= 0.70 probability in two consecutive
 * windows, with a 2 s cooldown; `correct`, low confidence, invalid data, or a
 * degraded state resets the count.
 *
 * The pulse itself is bounded by the NODE (0xA7, design Section 6.4): the
 * Node validates intensity and duration, owns the stop deadline, and ignores
 * an event id at or below the last one it executed. A dropped packet or a
 * closed tab therefore cannot leave a motor running. The browser still keeps
 * a local watchdog that sends an explicit stop shortly after the expected
 * end, but that is belt-and-braces, not the safety mechanism.
 */

import {} from "./ble-protocol.js";

/** Watchdog stop is sent after the Node's own deadline should have fired. */
const WATCHDOG_GRACE_MS = 100;

export const HAPTIC_DEFAULTS = {
  probabilityThreshold: 0.7,
  requiredConsecutive: 2,
  cooldownSeconds: 2.0,
  intensityPercent: 50,
  durationMs: 250,
  // Set from the bench ring-down measurement (Section 12); 250 ms is the
  // pre-measurement default and must not be treated as settled.
  ringDownMarginMs: 250,
};

// Class id -> haptic target node (Section 9).
export const CLASS_TARGET_NODE = {
  1: 2, // incomplete_range -> N2 wrist
  2: 3, // elbow_movement -> N3 elbow
};

export class HapticController {
  constructor(transport, preprocessor, { onEvent, onStateChange, ...options } = {}) {
    this.transport = transport;
    this.preprocessor = preprocessor;
    this.options = { ...HAPTIC_DEFAULTS, ...options };
    this.armed = false;
    this.healthy = false;
    this.enabled = false; // model valid + inference running
    this.consecutiveIncorrect = 0;
    this.lastTriggerS = null;
    this.lastTriggerPerfMs = null;
    this.eventCounter = 0;
    this.pendingOffByNode = new Map(); // nodeId -> timerId
    this.onEvent = onEvent || (() => {});
    this.onStateChange = onStateChange || (() => {});
  }

  canArm() {
    return this.enabled && this.healthy;
  }

  setHealthy(healthy) {
    if (!healthy && this.armed) {
      this.disarm("health degraded");
    }
    this.healthy = healthy;
    this.onStateChange(this.snapshot());
  }

  setEnabled(enabled) {
    if (!enabled && this.armed) {
      this.disarm("inference unavailable");
    }
    this.enabled = enabled;
    this.onStateChange(this.snapshot());
  }

  arm() {
    if (!this.canArm()) return false;
    this.armed = true;
    this.onStateChange(this.snapshot());
    this.onEvent({ kind: "arm" });
    return true;
  }

  disarm(reason) {
    const wasArmed = this.armed;
    this.armed = false;
    // Fail silent (Section 15): a disarm mid-pulse must still stop the motor,
    // so send the off command for any node with a pending off timer. The
    // timers are cleared regardless; each send is best-effort and a failure
    // is logged through the pulse path.
    for (const [nodeId, timer] of this.pendingOffByNode) {
      clearTimeout(timer);
      this.pendingOffByNode.delete(nodeId);
      this.transport
        .motorPercent(nodeId, 0)
        .catch((err) => this.onEvent({ kind: "pulse_off_failed", targetNode: nodeId, reason: err.message }));
    }
    this.consecutiveIncorrect = 0;
    if (wasArmed) {
      this.onEvent({ kind: "disarm", reason });
    }
    this.onStateChange(this.snapshot());
  }

  resetSession() {
    this.disarm("session stopped");
    this.consecutiveIncorrect = 0;
    this.lastTriggerS = null;
    this.eventCounter = 0;
  }

  /**
   * Feed one valid prediction. Returns decision info for the caller/log even
   * when disarmed (the gate state is still displayed).
   */
  evaluate(prediction, windowEndS) {
    const confidence = prediction.probabilities[prediction.classId];
    const confidentIncorrect = prediction.classId !== 0 && confidence >= this.options.probabilityThreshold;
    this.consecutiveIncorrect = confidentIncorrect ? this.consecutiveIncorrect + 1 : 0;
    const cooldownReady =
      this.lastTriggerS === null || windowEndS - this.lastTriggerS >= this.options.cooldownSeconds;
    const gateTrigger = this.consecutiveIncorrect >= this.requiredConsecutiveCount() && cooldownReady;
    const decision = {
      classId: prediction.classId,
      label: prediction.label,
      confidence,
      consecutiveIncorrect: this.consecutiveIncorrect,
      gateTrigger,
      fired: false,
    };
    if (!gateTrigger) return decision;
    this.lastTriggerS = windowEndS;
    this.lastTriggerPerfMs = performance.now();
    if (!this.armed || !this.healthy) return decision;

    const targetNode = CLASS_TARGET_NODE[prediction.classId];
    if (!targetNode) return decision;
    this.fire(targetNode, prediction.classId, windowEndS);
    decision.fired = true;
    return decision;
  }

  requiredConsecutiveCount() {
    return this.options.requiredConsecutive;
  }

  async fire(targetNode, classId, windowEndS) {
    const eventId = ++this.eventCounter;
    const { intensityPercent, durationMs, ringDownMarginMs } = this.options;
    const startedAtMs = performance.now();
    this.onEvent({
      kind: "pulse_start", eventId, targetNode, classId,
      intensityPercent, durationMs, windowEndS,
    });
    try {
      await this.transport.hapticPulse(targetNode, intensityPercent, durationMs, eventId);
    } catch (err) {
      this.disarm(`pulse send failed: ${err.message}`);
      this.onEvent({ kind: "pulse_failed", eventId, targetNode, reason: err.message });
      return;
    }
    // Blanking covers the pulse plus ring-down on the pulsed node; any window
    // intersecting it is invalid (Section 9.1).
    this.preprocessor.registerBlanking(
      targetNode,
      windowEndS,
      (durationMs + ringDownMarginMs) / 1000
    );
    // Watchdog only: the Node has already stopped itself by now. A failure
    // here is logged but does not disarm, because the motor is not stuck.
    const offTimer = setTimeout(async () => {
      this.pendingOffByNode.delete(targetNode);
      try {
        await this.transport.motorPercent(targetNode, 0);
        this.onEvent({ kind: "pulse_end", eventId, targetNode, elapsedMs: performance.now() - startedAtMs });
      } catch (err) {
        this.onEvent({ kind: "pulse_watchdog_failed", eventId, targetNode, reason: err.message });
      }
    }, durationMs + WATCHDOG_GRACE_MS);
    this.pendingOffByNode.set(targetNode, offTimer);
  }

  snapshot() {
    return {
      armed: this.armed,
      canArm: this.canArm(),
      healthy: this.healthy,
      enabled: this.enabled,
      consecutiveIncorrect: this.consecutiveIncorrect,
      cooldownRemainingS:
        this.lastTriggerPerfMs === null
          ? 0
          : Math.max(0, this.options.cooldownSeconds - (performance.now() - this.lastTriggerPerfMs) / 1000),
    };
  }
}
