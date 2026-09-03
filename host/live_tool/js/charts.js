/**
 * Real-time strip charts for the live session.
 *
 * Rendering is decoupled from BLE callbacks exactly as the four-node preview
 * design requires: notifications only append to a ring, and a fixed-rate
 * animation loop draws. A stalled or bursty link changes the trace, never the
 * frame rate, and a source going stale never erases the history already drawn.
 *
 * No charting library: the page is served locally and vendored, so a canvas
 * implementation avoids another pinned dependency.
 */

/** Signal groups selectable per source. Each maps to decoded sample fields. */
export const SIGNAL_GROUPS = {
  bno_gyro: {
    label: "BNO gyro (rad/s)",
    sensor: "bno",
    series: [
      { key: "gyro_x_radps", name: "x" },
      { key: "gyro_y_radps", name: "y" },
      { key: "gyro_z_radps", name: "z" },
    ],
  },
  bno_accel: {
    label: "BNO linear accel (m/s²)",
    sensor: "bno",
    series: [
      { key: "linear_accel_x_mps2", name: "x" },
      { key: "linear_accel_y_mps2", name: "y" },
      { key: "linear_accel_z_mps2", name: "z" },
    ],
  },
  bno_quat: {
    label: "BNO quaternion",
    sensor: "bno",
    series: [
      { key: "quat_i", name: "i" },
      { key: "quat_j", name: "j" },
      { key: "quat_k", name: "k" },
      { key: "quat_real", name: "w" },
    ],
  },
  icm_accel: {
    label: "ICM accel (g)",
    sensor: "icm",
    series: [
      { key: "accel_x_g", name: "x" },
      { key: "accel_y_g", name: "y" },
      { key: "accel_z_g", name: "z" },
    ],
  },
  icm_gyro: {
    label: "ICM gyro (dps)",
    sensor: "icm",
    series: [
      { key: "gyro_x_dps", name: "x" },
      { key: "gyro_y_dps", name: "y" },
      { key: "gyro_z_dps", name: "z" },
    ],
  },
};

export const SERIES_COLORS = ["#4da3ff", "#3fb96f", "#e0a63f", "#c98bff"];

/**
 * Fixed-capacity ring of (timeS, value[]) rows for one source+sensor.
 * Preallocated typed arrays: no per-sample allocation at 25 Hz x 10 streams.
 */
class SignalRing {
  constructor(width, capacity) {
    this.width = width;
    this.capacity = capacity;
    this.times = new Float64Array(capacity);
    this.values = new Float32Array(capacity * width);
    this.count = 0;
    this.head = 0;
  }

  push(timeS, row) {
    const index = this.head;
    this.times[index] = timeS;
    this.values.set(row, index * this.width);
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) this.count += 1;
  }

  /** Oldest-to-newest index walk without copying. */
  forEach(callback) {
    const start = (this.head - this.count + this.capacity) % this.capacity;
    for (let i = 0; i < this.count; i += 1) {
      const index = (start + i) % this.capacity;
      callback(this.times[index], index * this.width, i);
    }
  }

  clear() {
    this.count = 0;
    this.head = 0;
  }
}

/**
 * One canvas per source. Draws the selected signal group over a rolling
 * window, auto-scaling to the data actually present.
 */
export class SourceChart {
  constructor(canvas, { windowSeconds = 10, capacity = 1200 } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.windowSeconds = windowSeconds;
    this.groupKey = "bno_gyro";
    this.rings = new Map(); // groupKey -> SignalRing
    this.capacity = capacity;
    this.lastSampleS = null;
    for (const [key, group] of Object.entries(SIGNAL_GROUPS)) {
      this.rings.set(key, new SignalRing(group.series.length, capacity));
    }
  }

  setGroup(groupKey) {
    if (SIGNAL_GROUPS[groupKey]) this.groupKey = groupKey;
  }

  /** Feed one decoded sample; it lands in every group for that sensor. */
  push(sensorName, timeS, values) {
    for (const [key, group] of Object.entries(SIGNAL_GROUPS)) {
      if (group.sensor !== sensorName) continue;
      const row = group.series.map((series) => {
        const value = values[series.key];
        return Number.isFinite(value) ? value : 0;
      });
      this.rings.get(key).push(timeS, row);
    }
    this.lastSampleS = timeS;
  }

  clear() {
    for (const ring of this.rings.values()) ring.clear();
    this.lastSampleS = null;
  }

  /** Resize the backing store to the CSS box; cheap no-op when unchanged. */
  syncSize() {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(this.canvas.clientWidth * ratio));
    const height = Math.max(1, Math.round(this.canvas.clientHeight * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    return ratio;
  }

  draw(nowS) {
    const ratio = this.syncSize();
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;
    ctx.save();
    ctx.clearRect(0, 0, width, height);

    const style = getComputedStyle(this.canvas);
    const gridColor = style.getPropertyValue("--chart-grid").trim() || "#1e2733";
    const axisColor = style.getPropertyValue("--chart-axis").trim() || "#37465a";
    const mutedColor = style.getPropertyValue("--chart-muted").trim() || "#7d8da1";

    const padLeft = 46 * ratio;
    const padRight = 8 * ratio;
    const padTop = 8 * ratio;
    const padBottom = 16 * ratio;
    const plotWidth = Math.max(1, width - padLeft - padRight);
    const plotHeight = Math.max(1, height - padTop - padBottom);

    const group = SIGNAL_GROUPS[this.groupKey];
    const ring = this.rings.get(this.groupKey);
    const endS = nowS;
    const startS = endS - this.windowSeconds;

    // Auto-scale over the visible window only, so an old spike does not
    // permanently flatten the current signal.
    let min = Infinity;
    let max = -Infinity;
    ring.forEach((timeS, offset) => {
      if (timeS < startS) return;
      for (let c = 0; c < ring.width; c += 1) {
        const value = ring.values[offset + c];
        if (value < min) min = value;
        if (value > max) max = value;
      }
    });
    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      min = -1;
      max = 1;
    }
    if (max - min < 1e-6) {
      const mid = (max + min) / 2;
      min = mid - 0.5;
      max = mid + 0.5;
    }
    const span = max - min;
    min -= span * 0.08;
    max += span * 0.08;

    const yFor = (value) => padTop + plotHeight * (1 - (value - min) / (max - min));
    const xFor = (timeS) => padLeft + plotWidth * ((timeS - startS) / this.windowSeconds);

    // Grid and value axis.
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1 * ratio;
    ctx.font = `${10 * ratio}px ui-monospace, "JetBrains Mono", "Cascadia Mono", monospace`;
    ctx.fillStyle = mutedColor;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let i = 0; i <= 4; i += 1) {
      const value = min + ((max - min) * i) / 4;
      const y = yFor(value);
      ctx.beginPath();
      ctx.moveTo(padLeft, y);
      ctx.lineTo(padLeft + plotWidth, y);
      ctx.stroke();
      ctx.fillText(formatTick(value), padLeft - 6 * ratio, y);
    }
    ctx.strokeStyle = axisColor;
    ctx.beginPath();
    ctx.moveTo(padLeft, padTop);
    ctx.lineTo(padLeft, padTop + plotHeight);
    ctx.stroke();

    // Traces.
    ctx.lineWidth = 1.4 * ratio;
    ctx.lineJoin = "round";
    for (let c = 0; c < ring.width; c += 1) {
      ctx.strokeStyle = SERIES_COLORS[c % SERIES_COLORS.length];
      ctx.beginPath();
      let started = false;
      ring.forEach((timeS, offset) => {
        if (timeS < startS || timeS > endS) return;
        const x = xFor(timeS);
        const y = yFor(ring.values[offset + c]);
        if (started) ctx.lineTo(x, y);
        else {
          ctx.moveTo(x, y);
          started = true;
        }
      });
      if (started) ctx.stroke();
    }

    // Legend.
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    let legendX = padLeft + 4 * ratio;
    for (let c = 0; c < group.series.length; c += 1) {
      ctx.fillStyle = SERIES_COLORS[c % SERIES_COLORS.length];
      ctx.fillRect(legendX, padTop + 2 * ratio, 8 * ratio, 3 * ratio);
      legendX += 11 * ratio;
      ctx.fillStyle = mutedColor;
      ctx.fillText(group.series[c].name, legendX, padTop);
      legendX += ctx.measureText(group.series[c].name).width + 10 * ratio;
    }

    if (ring.count === 0) {
      ctx.fillStyle = mutedColor;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("no data", padLeft + plotWidth / 2, padTop + plotHeight / 2);
    }
    ctx.restore();
  }
}

function formatTick(value) {
  const magnitude = Math.abs(value);
  if (magnitude >= 1000) return value.toFixed(0);
  if (magnitude >= 10) return value.toFixed(1);
  return value.toFixed(2);
}
