"""Dependency-free reference implementation of the live preprocessing pipeline.

This is an INDEPENDENT re-implementation of
host/live_tool/pipeline/vantare_live_pipeline.py, written without numpy or
pandas so it runs on a stock Python install. Its purpose is the Section 12
Python-to-browser parity test: two implementations written separately that
agree to 1e-4 on the same fixture are real evidence the browser port is
faithful, in a way a single shared implementation can never be.

Where numpy semantics matter, they are reproduced exactly:
  - std is the population standard deviation (np.std, ddof=0)
  - percentile uses linear interpolation (np.percentile default)
  - nearest-sample decimation resolves ties to the left sample
"""

from __future__ import annotations

import bisect
import math

NODES = (2, 3, 4)
BNO = 1
ICM = 2
BNO_COLUMNS = (
    "quat_i", "quat_j", "quat_k", "quat_real",
    "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
    "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
    "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
)
ICM_COLUMNS = (
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
)
SUMMARY_STATISTICS = ("mean", "std", "min", "max", "range", "iqr", "rms", "mean_abs_diff")


def columns_for(sensor_id: int) -> tuple[str, ...]:
    return BNO_COLUMNS if sensor_id == BNO else ICM_COLUMNS


def build_channel_names() -> list[str]:
    channels: list[str] = []
    for node in NODES:
        channels.extend(f"n{node}_bno_{column}" for column in BNO_COLUMNS)
        channels.extend(f"n{node}_icm_{column}" for column in ICM_COLUMNS)
        channels.append(f"n{node}_bno_linear_accel_mag")
        channels.append(f"n{node}_bno_gyro_mag")
        channels.append(f"n{node}_icm_accel_mag")
        channels.append(f"n{node}_icm_gyro_mag")
    channels.append("relative_angle_n2_n3_deg")
    channels.append("relative_angle_n3_n4_deg")
    channels.append("relative_angle_n2_n4_deg")
    return channels


def build_feature_names() -> list[str]:
    return [
        f"{channel}__{statistic}"
        for channel in build_channel_names()
        for statistic in SUMMARY_STATISTICS
    ]


def percentile_linear(sorted_values: list[float], q: float) -> float:
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = (len(sorted_values) - 1) * q / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[int(position)]
    return sorted_values[lower] + (sorted_values[upper] - sorted_values[lower]) * (position - lower)


def column_statistics(signal: list[float]) -> list[float]:
    n = len(signal)
    mean = sum(signal) / n
    variance = sum((value - mean) ** 2 for value in signal) / n
    rms = math.sqrt(sum(value * value for value in signal) / n)
    abs_diff = sum(abs(signal[i] - signal[i - 1]) for i in range(1, n)) / (n - 1)
    ordered = sorted(signal)
    return [
        mean,
        math.sqrt(variance),
        min(signal),
        max(signal),
        max(signal) - min(signal),
        percentile_linear(ordered, 75) - percentile_linear(ordered, 25),
        rms,
        abs_diff,
    ]


def nearest_index(times: list[float], target: float) -> int:
    """Mirror decimate_stream_to_grid: clip(searchsorted(times, target), 1, n-1)
    then take the closer neighbour, resolving ties to the left."""
    n = len(times)
    if n == 1:
        return 0
    right = bisect.bisect_left(times, target)
    right = max(1, min(right, n - 1))
    left_distance = target - times[right - 1]
    right_distance = times[right] - target
    return right if abs(right_distance) < abs(left_distance) else right - 1


class ReferencePreprocessor:
    """Independent implementation of the browser Preprocessor emission rules."""

    def __init__(self, target_hz: int, window_samples: int, stride_samples: int):
        self.target_hz = target_hz
        self.window_samples = window_samples
        self.stride_samples = stride_samples
        self.period_s = 1.0 / target_hz
        self.window_seconds = window_samples / target_hz
        self.stride_seconds = stride_samples / target_hz
        self.streams: dict[tuple[int, int], dict] = {
            (node, sensor): {"times": [], "rows": []}
            for node in NODES
            for sensor in (BNO, ICM)
        }
        self.last_emitted_end_s: float | None = None

    def push_sample(self, node_id: int, sensor_id: int, time_s: float, values: dict) -> None:
        stream = self.streams[(node_id, sensor_id)]
        if stream["times"] and time_s <= stream["times"][-1]:
            return
        stream["times"].append(time_s)
        stream["rows"].append([float(values[column]) for column in columns_for(sensor_id)])

    def maybe_emit(self):
        if any(len(stream["times"]) < 2 for stream in self.streams.values()):
            return None
        common_end = min(stream["times"][-1] for stream in self.streams.values())
        common_start = max(stream["times"][0] for stream in self.streams.values())
        if common_end - common_start + 1e-9 < self.window_seconds:
            return None
        if (
            self.last_emitted_end_s is not None
            and common_end - self.last_emitted_end_s + 1e-9 < self.stride_seconds
        ):
            return None
        first_grid_s = common_end - (self.window_samples - 1) * self.period_s
        if first_grid_s < common_start - 1e-9:
            return None

        grid = [first_grid_s + i * self.period_s for i in range(self.window_samples)]
        frame = self._assemble(grid)
        features: list[float] = []
        for channel in frame:
            features.extend(column_statistics(channel))
        self.last_emitted_end_s = common_end
        self._trim(common_end)
        return {"startS": first_grid_s, "endS": common_end, "features": features}

    def _decimate(self, stream: dict, grid: list[float]) -> list[list[float]]:
        width = len(stream["rows"][0])
        out = [[0.0] * len(grid) for _ in range(width)]
        for g, target in enumerate(grid):
            source = stream["rows"][nearest_index(stream["times"], target)]
            for c in range(width):
                out[c][g] = source[c]
        return out

    def _assemble(self, grid: list[float]) -> list[list[float]]:
        frame: list[list[float]] = []
        quaternions: dict[int, list[list[float]]] = {}
        for node in NODES:
            bno = self._decimate(self.streams[(node, BNO)], grid)
            for g in range(len(grid)):
                norm = math.sqrt(sum(bno[c][g] ** 2 for c in range(4)))
                if norm < 1e-9:
                    raise ValueError(f"Zero-length quaternion for N{node}")
                for c in range(4):
                    bno[c][g] /= norm
            quaternions[node] = bno
            frame.extend(bno[c] for c in range(len(BNO_COLUMNS)))
            icm = self._decimate(self.streams[(node, ICM)], grid)
            frame.extend(icm[c] for c in range(len(ICM_COLUMNS)))
            for x, y, z in (
                (bno[4], bno[5], bno[6]),
                (bno[10], bno[11], bno[12]),
                (icm[0], icm[1], icm[2]),
                (icm[3], icm[4], icm[5]),
            ):
                frame.append(
                    [math.sqrt(x[g] ** 2 + y[g] ** 2 + z[g] ** 2) for g in range(len(grid))]
                )
        for left, right in ((2, 3), (3, 4), (2, 4)):
            lq = quaternions[left]
            rq = quaternions[right]
            angles = []
            for g in range(len(grid)):
                ln = max(math.sqrt(sum(lq[c][g] ** 2 for c in range(4))), 1e-9)
                rn = max(math.sqrt(sum(rq[c][g] ** 2 for c in range(4))), 1e-9)
                dot = abs(sum((lq[c][g] / ln) * (rq[c][g] / rn) for c in range(4)))
                angles.append(math.degrees(2 * math.acos(min(max(dot, 0.0), 1.0))))
            frame.append(angles)
        return frame

    def _trim(self, common_end_s: float) -> None:
        cutoff = common_end_s - self.window_seconds - self.stride_seconds
        for stream in self.streams.values():
            while len(stream["times"]) > 2 and stream["times"][1] < cutoff:
                stream["times"].pop(0)
                stream["rows"].pop(0)
