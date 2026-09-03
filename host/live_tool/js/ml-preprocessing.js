/**
 * Live preprocessing: window assembly, 72-channel synchronization, and 576
 * feature extraction. This is a faithful port of the training notebook's
 * pipeline (vantare_live_pipeline.py):
 *
 * - decimate_stream_to_grid (NEAREST sample per grid point)
 * - assemble_synchronized_frame (quaternion normalize, magnitudes, angles)
 * - summarize_window (mean, std, min, max, range, iqr, rms, mean_abs_diff)
 * - LiveWindowAssembler emission rules (common_end anchored grid, stride gate,
 *   buffer trim at common_end - window - stride)
 *
 * Grid sampling is NEAREST-SAMPLE, not linear interpolation. The training
 * features were built by synchronize_session, which decimates each stream onto
 * the grid with decimate_stream_to_grid before assembling channels, so every
 * training value is a real sensor sample. Linear interpolation would blend two
 * samples 40 ms apart and systematically lower std, range, rms, and
 * mean_abs_diff relative to anything the model was trained on.
 *
 * Any change here must be mirrored in the notebook (or vice versa) and pass
 * the Python-to-browser parity test (design Section 12).
 */

import { BNO_COLUMNS, ICM_COLUMNS, NODE_IDS, SENSOR } from "./ble-protocol.js";

const QUAT_COLS = ["quat_i", "quat_j", "quat_k", "quat_real"];

/** The 72 synchronized channels in the exact training order. */
export function buildChannelNames() {
  const channels = [];
  for (const node of NODE_IDS) {
    for (const column of BNO_COLUMNS) channels.push(`n${node}_bno_${column}`);
    for (const column of ICM_COLUMNS) channels.push(`n${node}_icm_${column}`);
    channels.push(`n${node}_bno_linear_accel_mag`);
    channels.push(`n${node}_bno_gyro_mag`);
    channels.push(`n${node}_icm_accel_mag`);
    channels.push(`n${node}_icm_gyro_mag`);
  }
  channels.push("relative_angle_n2_n3_deg");
  channels.push("relative_angle_n3_n4_deg");
  channels.push("relative_angle_n2_n4_deg");
  return channels;
}

/** 8 statistics per channel, in the exact training order. */
export function buildFeatureNames(channelNames) {
  const statistics = ["mean", "std", "min", "max", "range", "iqr", "rms", "mean_abs_diff"];
  const names = [];
  for (const channel of channelNames) {
    for (const statistic of statistics) names.push(`${channel}__${statistic}`);
  }
  return names;
}

function streamKey(nodeId, sensorId) {
  return `n${nodeId}s${sensorId}`;
}

/** numpy.percentile linear interpolation on an ascending-sorted array. */
export function percentileLinear(sorted, q) {
  if (sorted.length === 1) return sorted[0];
  const position = ((sorted.length - 1) * q) / 100;
  const lower = Math.floor(position);
  const upper = Math.ceil(position);
  if (lower === upper) return sorted[lower];
  return sorted[lower] + (sorted[upper] - sorted[lower]) * (position - lower);
}

/** The eight window statistics for one column, float64 internally. */
export function columnStatistics(signal) {
  const n = signal.length;
  let sum = 0;
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < n; i += 1) {
    const value = signal[i];
    sum += value;
    if (value < min) min = value;
    if (value > max) max = value;
  }
  const mean = sum / n;
  let variance = 0;
  let squaredSum = 0;
  for (let i = 0; i < n; i += 1) {
    const centered = signal[i] - mean;
    variance += centered * centered;
    squaredSum += signal[i] * signal[i];
  }
  variance /= n;
  let absDiffSum = 0;
  for (let i = 1; i < n; i += 1) absDiffSum += Math.abs(signal[i] - signal[i - 1]);
  const sorted = Float64Array.from(signal).sort();
  return [
    mean,
    Math.sqrt(variance),
    min,
    max,
    max - min,
    percentileLinear(sorted, 75) - percentileLinear(sorted, 25),
    Math.sqrt(squaredSum / n),
    absDiffSum / (n - 1),
  ];
}

/**
 * Index of the sample nearest `target`, mirroring decimate_stream_to_grid:
 *
 *   right = clip(searchsorted(times, target), 1, n - 1)      # side="left"
 *   choose right when |times[right] - target| < |target - times[right - 1]|
 *
 * Ties resolve to the left sample, exactly as the strict `<` does in numpy.
 */
export function nearestIndex(times, target) {
  const n = times.length;
  if (n === 1) return 0;
  let low = 0;
  let high = n;
  while (low < high) {
    const mid = (low + high) >> 1;
    if (times[mid] < target) low = mid + 1;
    else high = mid;
  }
  let right = low;
  if (right < 1) right = 1;
  if (right > n - 1) right = n - 1;
  const leftDistance = target - times[right - 1];
  const rightDistance = times[right] - target;
  return Math.abs(rightDistance) < Math.abs(leftDistance) ? right : right - 1;
}

/**
 * Window assembler and feature extractor for one live session.
 *
 * contract: { target_hz, window_samples, stride_samples, ... } — loaded from
 * model_contract.json and validated against the qualified live rate by the
 * caller. All health gates scale with T = 1000 / target_hz (Section 10).
 */
export class Preprocessor {
  constructor(contract, { maxInterpPeriods = 1.5, maxMissingFraction = 0.02, bufferCap = 512 } = {}) {
    this.targetHz = contract.target_hz;
    this.windowSamples = contract.window_samples;
    this.strideSamples = contract.stride_samples;
    this.windowSeconds = this.windowSamples / this.targetHz;
    this.strideSeconds = this.strideSamples / this.targetHz;
    this.periodS = 1 / this.targetHz;
    this.maxInterpSpanS = maxInterpPeriods * this.periodS;
    this.maxMissing = Math.floor(maxMissingFraction * this.windowSamples); // >2% of expected is invalid
    this.bufferCap = bufferCap;
    this.channels = buildChannelNames();
    this.streams = new Map(); // key -> { times: [], rows: [Float64Array], seqs: [], lastSeq }
    for (const node of NODE_IDS) {
      this.streams.set(streamKey(node, SENSOR.BNO), { times: [], rows: [], seqs: [], lastSeq: null });
      this.streams.set(streamKey(node, SENSOR.ICM), { times: [], rows: [], seqs: [], lastSeq: null });
    }
    this.blankingIntervals = []; // { nodeId, startS, endS }
    this.lastEmittedEndS = null;
    this.stats = { emitted: 0, invalid: 0, droppedSamples: 0, byReason: {} };
  }

  expectedColumns(sensorId) {
    return sensorId === SENSOR.BNO ? BNO_COLUMNS.length : ICM_COLUMNS.length;
  }

  /**
   * Push one decoded sample. times are session-mapped seconds (TimeBase).
   * Returns true if the sample was accepted; a non-monotonic timestamp is
   * dropped and reported (Section 10: "timestamps move backward").
   */
  pushSample(nodeId, sensorId, mappedS, sequence, values) {
    const stream = this.streams.get(streamKey(nodeId, sensorId));
    if (!stream) return false;
    if (stream.times.length > 0 && mappedS <= stream.times[stream.times.length - 1]) {
      this.stats.droppedSamples += 1;
      return false;
    }
    const width = this.expectedColumns(sensorId);
    const row = new Float64Array(width);
    const keys = sensorId === SENSOR.BNO ? BNO_COLUMNS : ICM_COLUMNS;
    for (let i = 0; i < width; i += 1) row[i] = values[keys[i]];
    stream.times.push(mappedS);
    stream.rows.push(row);
    stream.seqs.push(sequence);
    stream.lastSeq = sequence;
    if (stream.times.length > this.bufferCap) {
      stream.times.shift();
      stream.rows.shift();
      stream.seqs.shift();
    }
    return true;
  }

  registerBlanking(nodeId, startS, durationS) {
    this.blankingIntervals.push({ nodeId, startS, endS: startS + durationS });
    const horizon = startS - 60;
    this.blankingIntervals = this.blankingIntervals.filter((entry) => entry.endS > horizon);
  }

  isBlanked(startS, endS) {
    for (const entry of this.blankingIntervals) {
      if (entry.startS < endS && startS < entry.endS) return entry;
    }
    return null;
  }

  /**
   * Try to emit one window. Notebook emission rules, then Section 10 gates.
   * Returns { status: "waiting" } | { status: "invalid", reason }
   *       | { status: "emitted", window: { startS, endS, features } }.
   */
  maybeEmit() {
    for (const stream of this.streams.values()) {
      if (stream.times.length < 2) return { status: "waiting" };
    }
    let commonEnd = Infinity;
    let commonStart = -Infinity;
    for (const stream of this.streams.values()) {
      commonEnd = Math.min(commonEnd, stream.times[stream.times.length - 1]);
      commonStart = Math.max(commonStart, stream.times[0]);
    }
    if (commonEnd - commonStart + 1e-9 < this.windowSeconds) return { status: "waiting" };
    if (
      this.lastEmittedEndS !== null &&
      commonEnd - this.lastEmittedEndS + 1e-9 < this.strideSeconds
    ) {
      return { status: "waiting" };
    }
    const firstGridS = commonEnd - (this.windowSamples - 1) * this.periodS;
    if (firstGridS < commonStart - 1e-9) return { status: "waiting" };

    const invalid = this.validateWindow(firstGridS, commonEnd);
    if (invalid) {
      this.stats.invalid += 1;
      this.stats.byReason[invalid] = (this.stats.byReason[invalid] || 0) + 1;
      this.lastEmittedEndS = commonEnd;
      return { status: "invalid", reason: invalid };
    }

    const grid = new Float64Array(this.windowSamples);
    for (let i = 0; i < this.windowSamples; i += 1) grid[i] = firstGridS + i * this.periodS;

    const synchronizedFrame = this.assembleSynchronizedFrame(grid);
    if (!synchronizedFrame) {
      this.stats.invalid += 1;
      this.stats.byReason.quaternion = (this.stats.byReason.quaternion || 0) + 1;
      this.lastEmittedEndS = commonEnd;
      return { status: "invalid", reason: "quaternion" };
    }

    const features = new Float32Array(this.channels.length * 8);
    let offset = 0;
    for (let column = 0; column < this.channels.length; column += 1) {
      const stats = columnStatistics(synchronizedFrame[column]);
      for (let s = 0; s < 8; s += 1) features[offset + s] = stats[s];
      offset += 8;
    }
    for (let i = 0; i < features.length; i += 1) {
      if (!Number.isFinite(features[i])) {
        this.stats.invalid += 1;
        this.stats.byReason.nonfinite = (this.stats.byReason.nonfinite || 0) + 1;
        this.lastEmittedEndS = commonEnd;
        return { status: "invalid", reason: "nonfinite" };
      }
    }

    this.lastEmittedEndS = commonEnd;
    this.stats.emitted += 1;
    this.trimBuffers(commonEnd);
    return { status: "emitted", window: { startS: firstGridS, endS: commonEnd, features } };
  }

  /** Section 10 window gates: interpolation span, packet loss, haptic blanking. */
  validateWindow(firstGridS, commonEndS) {
    const blanked = this.isBlanked(firstGridS, commonEndS);
    if (blanked) return `haptic_blanking_n${blanked.nodeId}`;

    for (const [key, stream] of this.streams) {
      const times = stream.times;
      // Sample-gap gate: the samples bracketing every grid point must be
      // within 1.5T, so the nearest sample chosen by decimateStream is never
      // more than a half-span away. Grid points at or beyond the newest sample
      // clamp to it, and the emission invariant guarantees the grid never
      // precedes a stream's oldest sample, so only interior gaps are
      // violations.
      for (let g = 0; g < this.windowSamples; g += 1) {
        const target = firstGridS + g * this.periodS;
        const right = this.upperBound(times, target);
        if (right === 0 || right >= times.length) continue;
        if (times[right] - times[right - 1] > this.maxInterpSpanS + 1e-9) {
          return `interp_span_${key}`;
        }
      }
      // Packet-loss gate: sequence continuity across the window span.
      const firstIndex = this.lowerBound(times, firstGridS - this.periodS);
      const lastIndex = this.upperBound(times, commonEndS + this.periodS) - 1;
      if (lastIndex <= firstIndex) return `loss_${key}`;
      const inWindow = lastIndex - firstIndex + 1;
      const missing = (stream.seqs[lastIndex] - stream.seqs[firstIndex] + 1 + 65536) % 65536 - inWindow;
      if (missing > this.maxMissing) return `loss_${key}`;
    }
    return null;
  }

  upperBound(times, target) {
    let low = 0;
    let high = times.length;
    while (low < high) {
      const mid = (low + high) >> 1;
      if (times[mid] <= target) low = mid + 1;
      else high = mid;
    }
    return low;
  }

  lowerBound(times, target) {
    let low = 0;
    let high = times.length;
    while (low < high) {
      const mid = (low + high) >> 1;
      if (times[mid] < target) low = mid + 1;
      else high = mid;
    }
    return low;
  }

  /** decimate_stream_to_grid equivalent: nearest real sample per grid point. */
  decimateStream(stream, grid) {
    const times = stream.times;
    const width = stream.rows[0].length;
    const out = new Array(width);
    for (let c = 0; c < width; c += 1) out[c] = new Float64Array(grid.length);
    for (let g = 0; g < grid.length; g += 1) {
      const source = stream.rows[nearestIndex(times, grid[g])];
      for (let c = 0; c < width; c += 1) out[c][g] = source[c];
    }
    return out;
  }

  /**
   * Notebook assemble_synchronized_frame: decimate the six streams onto the
   * grid, unit-normalize quaternions, derive magnitudes and angles. Returns
   * channels.length arrays of grid.length values, or null on a degenerate
   * quaternion.
   */
  assembleSynchronizedFrame(grid) {
    const frame = [];
    const quats = new Map(); // nodeId -> [qi[], qj[], qk[], qr[]]
    for (const node of NODE_IDS) {
      const bno = this.decimateStream(this.streams.get(streamKey(node, SENSOR.BNO)), grid);
      for (let g = 0; g < grid.length; g += 1) {
        const qi = bno[0][g];
        const qj = bno[1][g];
        const qk = bno[2][g];
        const qr = bno[3][g];
        const norm = Math.sqrt(qi * qi + qj * qj + qk * qk + qr * qr);
        if (norm < 1e-9) return null;
        bno[0][g] = qi / norm;
        bno[1][g] = qj / norm;
        bno[2][g] = qk / norm;
        bno[3][g] = qr / norm;
      }
      quats.set(node, bno);
      for (let c = 0; c < BNO_COLUMNS.length; c += 1) frame.push(bno[c]);
      const icm = this.decimateStream(this.streams.get(streamKey(node, SENSOR.ICM)), grid);
      for (let c = 0; c < ICM_COLUMNS.length; c += 1) frame.push(icm[c]);
      // Magnitude groups in training order.
      const magnitudeGroups = [
        [bno[4], bno[5], bno[6]], // linear accel xyz
        [bno[10], bno[11], bno[12]], // gyro xyz
        [icm[0], icm[1], icm[2]], // accel xyz
        [icm[3], icm[4], icm[5]], // gyro xyz
      ];
      for (const [x, y, z] of magnitudeGroups) {
        const mag = new Float64Array(grid.length);
        for (let g = 0; g < grid.length; g += 1) mag[g] = Math.sqrt(x[g] * x[g] + y[g] * y[g] + z[g] * z[g]);
        frame.push(mag);
      }
    }
    for (const [left, right] of [[2, 3], [3, 4], [2, 4]]) {
      const lq = quats.get(left);
      const rq = quats.get(right);
      const angle = new Float64Array(grid.length);
      for (let g = 0; g < grid.length; g += 1) {
        // quaternion_angle_degrees normalizes again after assembly; unit
        // vectors make that a no-op, but re-normalize to mirror it exactly.
        let ln = Math.sqrt(lq[0][g] ** 2 + lq[1][g] ** 2 + lq[2][g] ** 2 + lq[3][g] ** 2);
        let rn = Math.sqrt(rq[0][g] ** 2 + rq[1][g] ** 2 + rq[2][g] ** 2 + rq[3][g] ** 2);
        ln = Math.max(ln, 1e-9);
        rn = Math.max(rn, 1e-9);
        const dot =
          Math.abs(
            (lq[0][g] / ln) * (rq[0][g] / rn) +
              (lq[1][g] / ln) * (rq[1][g] / rn) +
              (lq[2][g] / ln) * (rq[2][g] / rn) +
              (lq[3][g] / ln) * (rq[3][g] / rn)
          );
        const clipped = Math.min(Math.max(dot, 0), 1);
        angle[g] = (2 * Math.acos(clipped) * 180) / Math.PI;
      }
      frame.push(angle);
    }
    return frame;
  }

  /** Notebook trim: cutoff = common_end - window - stride. */
  trimBuffers(commonEndS) {
    const cutoff = commonEndS - this.windowSeconds - this.strideSeconds;
    for (const stream of this.streams.values()) {
      while (stream.times.length > 2 && stream.times[1] < cutoff) {
        stream.times.shift();
        stream.rows.shift();
        stream.seqs.shift();
      }
    }
  }

  resetSession() {
    for (const stream of this.streams.values()) {
      stream.times.length = 0;
      stream.rows.length = 0;
      stream.seqs.length = 0;
      stream.lastSeq = null;
    }
    this.blankingIntervals.length = 0;
    this.lastEmittedEndS = null;
    this.stats = { emitted: 0, invalid: 0, droppedSamples: 0, byReason: {} };
  }
}
