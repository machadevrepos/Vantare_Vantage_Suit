"""Shared offline and live preprocessing for the Vantare bicep-curl model.

Authoritative Python source of truth for the pipeline described in
docs/superpowers/specs/2026-09-01-live-bicep-curl-inference-design.md.
The notebook exports a copy of this file, and the browser port in
host/live_tool/js/ml-preprocessing.js mirrors it channel for channel.

Grid sampling is NEAREST-SAMPLE decimation, never linear interpolation, in
both the offline and the live path. Interpolating between two samples 40 ms
apart blends real readings and systematically lowers std, range, rms and
mean_abs_diff relative to the values the model was trained on.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
import pandas as pd


NODES = (2, 3, 4)
SENSORS = ("BNO85", "ICM45686")
CLASS_NAMES = ("correct", "incomplete_range", "elbow_movement")
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

# Live contract (design Section 11.1): 25 Hz, 2 s windows, 12-sample stride.
TARGET_HZ = 25
WINDOW_SECONDS = 2.0
STRIDE_SECONDS = 0.48


def expected_sensor_columns(sensor: str) -> tuple[str, ...]:
    if sensor == "BNO85":
        return BNO_COLUMNS
    if sensor == "ICM45686":
        return ICM_COLUMNS
    raise ValueError(f"Unsupported sensor: {sensor}")


def session_file(dataset_root: Path | str, entry: Mapping, node: int, sensor: str) -> Path:
    session = int(entry["session"])
    return Path(dataset_root) / str(entry["folder"]) / f"R{session:04d}N{node}_{sensor}.csv"


def load_numeric_stream(path: Path, columns: Iterable[str]) -> pd.DataFrame:
    columns = list(columns)
    frame = pd.read_csv(path, usecols=["timestamp_us", *columns])
    for column in ["timestamp_us", *columns]:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    if frame.isna().any().any():
        raise ValueError(f"Non-numeric or missing values in {path}")
    frame["time_s"] = frame.pop("timestamp_us") / 1_000_000.0
    frame = frame.drop_duplicates("time_s", keep="last").sort_values("time_s")
    if len(frame) < 2 or not frame["time_s"].is_monotonic_increasing:
        raise ValueError(f"Invalid timestamp series in {path}")
    return frame.set_index("time_s")


def decimate_stream_to_grid(frame: pd.DataFrame, grid: np.ndarray) -> pd.DataFrame:
    """Nearest-sample decimation onto the grid (design Section 11.3 step 3).

    Every emitted value is a real sensor sample. Ties resolve to the left
    sample, which the strict `<` below expresses.
    """
    index = frame.index.to_numpy()
    if len(index) == 0:
        raise ValueError("Cannot decimate an empty stream")
    if len(index) == 1:
        decimated = frame.iloc[[0] * len(grid)].copy()
        decimated.index = grid
        return decimated
    right_positions = np.clip(np.searchsorted(index, grid), 1, len(index) - 1)
    left_times = index[right_positions - 1]
    right_times = index[right_positions]
    choose_right = np.abs(right_times - grid) < np.abs(grid - left_times)
    chosen = np.where(choose_right, right_positions, right_positions - 1)
    decimated = frame.iloc[chosen].copy()
    decimated.index = grid
    return decimated


def quaternion_angle_degrees(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left = left / np.clip(np.linalg.norm(left, axis=1, keepdims=True), 1e-9, None)
    right = right / np.clip(np.linalg.norm(right, axis=1, keepdims=True), 1e-9, None)
    dots = np.abs(np.sum(left * right, axis=1))
    return np.degrees(2 * np.arccos(np.clip(dots, 0, 1)))


def assemble_synchronized_frame(
    raw_streams: Mapping[tuple[int, str], pd.DataFrame], grid: np.ndarray
) -> pd.DataFrame:
    """Decimate six streams onto the grid and derive the 72 model channels."""
    merged = pd.DataFrame(index=grid)
    for node in NODES:
        for sensor in SENSORS:
            sampled = decimate_stream_to_grid(raw_streams[(node, sensor)], grid)
            if sensor == "BNO85":
                quaternion = sampled[list(BNO_COLUMNS[:4])].to_numpy(copy=True)
                norms = np.linalg.norm(quaternion, axis=1, keepdims=True)
                if np.any(norms < 1e-9):
                    raise ValueError(f"Zero-length quaternion detected for N{node}")
                sampled.loc[:, list(BNO_COLUMNS[:4])] = quaternion / norms
                prefix = f"n{node}_bno_"
            else:
                prefix = f"n{node}_icm_"
            sampled.columns = [f"{prefix}{column}" for column in sampled.columns]
            merged = merged.join(sampled)

        magnitude_groups = {
            f"n{node}_bno_linear_accel_mag": [
                f"n{node}_bno_linear_accel_x_mps2", f"n{node}_bno_linear_accel_y_mps2",
                f"n{node}_bno_linear_accel_z_mps2",
            ],
            f"n{node}_bno_gyro_mag": [
                f"n{node}_bno_gyro_x_radps", f"n{node}_bno_gyro_y_radps", f"n{node}_bno_gyro_z_radps",
            ],
            f"n{node}_icm_accel_mag": [
                f"n{node}_icm_accel_x_g", f"n{node}_icm_accel_y_g", f"n{node}_icm_accel_z_g",
            ],
            f"n{node}_icm_gyro_mag": [
                f"n{node}_icm_gyro_x_dps", f"n{node}_icm_gyro_y_dps", f"n{node}_icm_gyro_z_dps",
            ],
        }
        for name, columns in magnitude_groups.items():
            merged[name] = np.linalg.norm(merged[columns].to_numpy(), axis=1)

    quaternion_suffixes = BNO_COLUMNS[:4]
    for left, right in ((2, 3), (3, 4), (2, 4)):
        left_q = merged[[f"n{left}_bno_{suffix}" for suffix in quaternion_suffixes]].to_numpy()
        right_q = merged[[f"n{right}_bno_{suffix}" for suffix in quaternion_suffixes]].to_numpy()
        merged[f"relative_angle_n{left}_n{right}_deg"] = quaternion_angle_degrees(left_q, right_q)
    merged.index.name = "time_s"
    return merged


def synchronize_session(
    dataset_root: Path | str, entry: Mapping, target_hz: int = TARGET_HZ
) -> pd.DataFrame:
    raw_streams = {}
    for node in NODES:
        for sensor in SENSORS:
            raw_streams[(node, sensor)] = load_numeric_stream(
                session_file(dataset_root, entry, node, sensor), expected_sensor_columns(sensor)
            )
    common_start = max(frame.index.min() for frame in raw_streams.values())
    common_end = min(frame.index.max() for frame in raw_streams.values())
    grid = np.arange(common_start, common_end, 1.0 / target_hz)
    if len(grid) < target_hz * 2:
        raise ValueError("Session has less than two seconds of common data")
    return assemble_synchronized_frame(raw_streams, grid)


def build_feature_names(signal_columns: Iterable[str]) -> list[str]:
    return [f"{column}__{statistic}" for column in signal_columns for statistic in SUMMARY_STATISTICS]


def summarize_window(window: pd.DataFrame) -> dict[str, float]:
    values = window.to_numpy(dtype=float)
    features: dict[str, float] = {}
    for column_index, column in enumerate(window.columns):
        signal = values[:, column_index]
        statistics = (
            float(np.mean(signal)),
            float(np.std(signal)),
            float(np.min(signal)),
            float(np.max(signal)),
            float(np.ptp(signal)),
            float(np.percentile(signal, 75) - np.percentile(signal, 25)),
            float(np.sqrt(np.mean(signal ** 2))),
            float(np.mean(np.abs(np.diff(signal)))),
        )
        for statistic_name, value in zip(SUMMARY_STATISTICS, statistics):
            features[f"{column}__{statistic_name}"] = value
    return features


def window_session(
    frame: pd.DataFrame,
    session: int,
    class_id: int,
    target_hz: int = TARGET_HZ,
    window_seconds: float = WINDOW_SECONDS,
    stride_seconds: float = STRIDE_SECONDS,
) -> pd.DataFrame:
    window_samples = int(round(window_seconds * target_hz))
    stride_samples = int(round(stride_seconds * target_hz))
    rows = []
    for start in range(0, len(frame) - window_samples + 1, stride_samples):
        window = frame.iloc[start : start + window_samples]
        row = summarize_window(window)
        row.update(
            session=int(session),
            class_id=int(class_id),
            window_start_s=float(window.index[0]),
            window_end_s=float(window.index[-1]),
        )
        rows.append(row)
    return pd.DataFrame(rows)


@dataclass(frozen=True)
class LiveFeatureWindow:
    end_timestamp_us: int
    features: np.ndarray


class LiveWindowAssembler:
    """Turn asynchronous live sensor samples into offline-equivalent windows.

    Uses the same nearest-sample decimation as synchronize_session, so live
    feature vectors are drawn from the same distribution as the training set.
    """

    def __init__(
        self,
        target_hz: int = TARGET_HZ,
        window_seconds: float = WINDOW_SECONDS,
        stride_seconds: float = STRIDE_SECONDS,
    ):
        self.target_hz = int(target_hz)
        self.window_seconds = float(window_seconds)
        self.stride_seconds = float(stride_seconds)
        self.window_samples = int(round(self.target_hz * self.window_seconds))
        self.streams = {(node, sensor): deque() for node in NODES for sensor in SENSORS}
        self.last_emitted_end_s: float | None = None

    def push_sample(
        self, node: int, sensor: str, timestamp_us: int, values: Mapping[str, float]
    ) -> list[LiveFeatureWindow]:
        key = (int(node), sensor)
        if key not in self.streams:
            raise ValueError(f"Unsupported live stream: N{node} {sensor}")
        expected = expected_sensor_columns(sensor)
        missing = [column for column in expected if column not in values]
        if missing:
            raise ValueError(f"Missing {sensor} fields: {missing}")
        timestamp_s = int(timestamp_us) / 1_000_000.0
        stream = self.streams[key]
        if stream and timestamp_s <= stream[-1][0]:
            raise ValueError(f"Timestamps must increase for N{node} {sensor}")
        stream.append((timestamp_s, tuple(float(values[column]) for column in expected)))
        return self._emit_ready_windows()

    def _emit_ready_windows(self) -> list[LiveFeatureWindow]:
        if any(len(stream) < 2 for stream in self.streams.values()):
            return []
        common_end = min(stream[-1][0] for stream in self.streams.values())
        common_start = max(stream[0][0] for stream in self.streams.values())
        if common_end - common_start + 1e-9 < self.window_seconds:
            return []
        if (
            self.last_emitted_end_s is not None
            and common_end - self.last_emitted_end_s + 1e-9 < self.stride_seconds
        ):
            return []

        first_grid_time = common_end - (self.window_samples - 1) / self.target_hz
        if first_grid_time < common_start - 1e-9:
            return []
        grid = first_grid_time + np.arange(self.window_samples) / self.target_hz
        raw_frames = {}
        for (node, sensor), stream in self.streams.items():
            columns = expected_sensor_columns(sensor)
            data = list(stream)
            raw_frames[(node, sensor)] = pd.DataFrame(
                [row[1] for row in data], index=[row[0] for row in data], columns=columns
            )
        synchronized = assemble_synchronized_frame(raw_frames, grid)
        feature_values = np.fromiter(summarize_window(synchronized).values(), dtype=np.float32)
        self.last_emitted_end_s = common_end
        cutoff = common_end - self.window_seconds - self.stride_seconds
        for stream in self.streams.values():
            while len(stream) > 2 and stream[1][0] < cutoff:
                stream.popleft()
        return [LiveFeatureWindow(int(round(common_end * 1_000_000)), feature_values)]
