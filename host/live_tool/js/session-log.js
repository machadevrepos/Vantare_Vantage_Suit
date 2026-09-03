/**
 * Two-tier session log (design Section 7, session-log.js).
 *
 * Tier 1 (full retention): predictions, probabilities, latency, health
 * transitions, decisions, skew estimates, and haptic events. Low rate; these
 * are the primary diagnostic record and are kept for the whole session.
 *
 * Tier 2 (byte-budgeted): raw decoded samples in preallocated typed-array ring
 * buffers. The budget (default 64 MB) — not wall-clock time — is the retention
 * limit; the equivalent duration is computed and displayed, never assumed.
 * When the ring wraps, the oldest samples are being overwritten and the UI
 * warns the user to download the current log.
 */

const TIER1_CAP = 100_000;

export class SessionLog {
  constructor({ byteBudget = 64 * 1024 * 1024 } = {}) {
    this.byteBudget = byteBudget;
    this.events = [];
    this.sessionStartedAtMs = null;
    this.rings = new Map(); // key -> { width, capacity, cursor, wrapped, buffer: Float32Array }
    this.sampleCounts = new Map();
  }

  startSession() {
    this.events.length = 0;
    this.rings.clear();
    this.sampleCounts.clear();
    this.sessionStartedAtMs = performance.now();
    this.event({ kind: "session_start" });
  }

  event(payload) {
    if (this.events.length < TIER1_CAP) {
      this.events.push({ tMs: performance.now(), ...payload });
    }
  }

  /** Register a stream's ring before the session starts. */
  registerStream(key, floatsPerRow) {
    const rowBytes = floatsPerRow * 4;
    const capacity = Math.max(16, Math.floor(this.byteBudget / 6 / rowBytes)); // 6 streams share the budget
    this.rings.set(key, {
      width: floatsPerRow,
      capacity,
      cursor: 0,
      wrapped: false,
      buffer: new Float32Array(capacity * floatsPerRow),
    });
    this.sampleCounts.set(key, 0);
  }

  logSample(key, mappedS, values) {
    const ring = this.rings.get(key);
    if (!ring) return;
    const offset = ring.cursor * ring.width;
    ring.buffer[offset] = mappedS;
    for (let i = 0; i < values.length; i += 1) ring.buffer[offset + 1 + i] = values[i];
    ring.cursor += 1;
    if (ring.cursor >= ring.capacity) {
      ring.cursor = 0;
      if (!ring.wrapped) {
        ring.wrapped = true;
        this.event({ kind: "log_ring_wrap", stream: key });
      }
    }
    this.sampleCounts.set(key, this.sampleCounts.get(key) + 1);
  }

  /** Retention duration in seconds implied by the byte budget and the rate. */
  retentionSeconds() {
    let totalCapacity = 0;
    for (const ring of this.rings.values()) totalCapacity += ring.capacity;
    // Samples arrive across six streams; capacity above is per stream.
    return totalCapacity / (6 * 25);
  }

  wrappedStreams() {
    return [...this.rings.entries()].filter(([, ring]) => ring.wrapped).map(([key]) => key);
  }

  buildDownload(meta = {}) {
    const lines = [];
    lines.push(JSON.stringify({
      type: "meta",
      recordedAt: new Date().toISOString(),
      ortVersion: window.ort && window.ort.env && window.ort.env.versionString,
      ...meta,
    }));
    for (const event of this.events) lines.push(JSON.stringify({ type: "event", ...event }));
    for (const [key, ring] of this.rings) {
      lines.push(JSON.stringify({ type: "stream_decl", stream: key, width: ring.width, capacity: ring.capacity }));
      const count = ring.wrapped ? ring.capacity : ring.cursor;
      for (let row = 0; row < count; row += 1) {
        const offset = row * ring.width;
        lines.push(JSON.stringify({
          type: "sample", stream: key,
          data: Array.from(ring.buffer.subarray(offset, offset + ring.width)),
        }));
      }
    }
    return new Blob([lines.join("\n")], { type: "application/x-ndjson" });
  }
}
