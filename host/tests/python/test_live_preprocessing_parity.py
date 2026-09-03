"""Python-to-browser preprocessing parity test (design Section 12).

Generates a deterministic six-stream fixture, runs it through two independent
implementations of the pipeline -- the pure-Python reference in
reference_preprocessing.py and the browser port in
host/live_tool/js/ml-preprocessing.js -- and requires every one of the 576
features to agree within 1e-4.

The fixture deliberately gives each stream a different phase offset and a
different signal frequency, so nearest-sample decimation and linear
interpolation produce measurably different features. test_fixture_is_
discriminating asserts that separation, which is what keeps this test able to
catch a regression back to interpolation.
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from reference_preprocessing import (
    BNO_COLUMNS,
    ICM_COLUMNS,
    NODES,
    BNO,
    ICM,
    ReferencePreprocessor,
    build_feature_names,
    nearest_index,
)

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "host" / "tests" / "scripts" / "run_preprocessing_fixture.mjs"

TARGET_HZ = 25
WINDOW_SAMPLES = 50
STRIDE_SAMPLES = 12
PERIOD = 1.0 / TARGET_HZ
TOLERANCE = 1e-4


class _Random:
    """Deterministic LCG so Python and the fixture file never disagree."""

    def __init__(self, seed: int = 20260902):
        self.state = seed

    def next_float(self) -> float:
        self.state = (1103515245 * self.state + 12345) % (1 << 31)
        return self.state / float(1 << 31)


def build_fixture(sample_count: int = 160) -> dict:
    """Six streams at ~25 Hz with per-stream phase offsets and timing jitter."""
    rng = _Random()
    samples = []
    for index in range(sample_count):
        for node_position, node in enumerate(NODES):
            for sensor in (BNO, ICM):
                # Distinct phase offset per stream, plus bounded jitter. The
                # offset is what makes grid points fall between real samples.
                offset = 0.004 * (node_position * 2 + (0 if sensor == BNO else 1))
                jitter = (rng.next_float() - 0.5) * 0.004
                t = index * PERIOD + offset + jitter
                phase = 2.0 * math.pi * (0.7 + 0.35 * node_position) * t
                swing = math.sin(phase)
                if sensor == BNO:
                    half = 0.35 * swing
                    values = {
                        "quat_i": math.sin(half),
                        "quat_j": 0.15 * math.cos(phase * 1.3),
                        "quat_k": 0.10 * math.sin(phase * 0.6),
                        "quat_real": math.cos(half),
                        "linear_accel_x_mps2": 3.5 * swing,
                        "linear_accel_y_mps2": 1.5 * math.cos(phase * 1.7),
                        "linear_accel_z_mps2": 0.8 * math.sin(phase * 2.3),
                        "gravity_x_mps2": 9.81 * math.cos(half),
                        "gravity_y_mps2": 0.4 * swing,
                        "gravity_z_mps2": 0.2 * math.cos(phase),
                        "gyro_x_radps": 2.2 * math.cos(phase),
                        "gyro_y_radps": 0.9 * math.sin(phase * 1.9),
                        "gyro_z_radps": 0.5 * math.cos(phase * 2.7),
                    }
                else:
                    values = {
                        "accel_x_g": 0.45 * swing,
                        "accel_y_g": 0.20 * math.cos(phase * 2.1),
                        "accel_z_g": 1.0 + 0.05 * math.sin(phase * 3.1),
                        "gyro_x_dps": 120.0 * math.cos(phase),
                        "gyro_y_dps": 45.0 * math.sin(phase * 1.4),
                        "gyro_z_dps": 20.0 * math.cos(phase * 2.9),
                    }
                samples.append(
                    {
                        "node": node,
                        "sensor": sensor,
                        "t": t,
                        "seq": index & 0xFFFF,
                        "values": values,
                    }
                )
    samples.sort(key=lambda entry: entry["t"])
    return {
        "contract": {
            "target_hz": TARGET_HZ,
            "window_samples": WINDOW_SAMPLES,
            "stride_samples": STRIDE_SAMPLES,
        },
        "samples": samples,
    }


def reference_windows(fixture: dict) -> list[dict]:
    preprocessor = ReferencePreprocessor(TARGET_HZ, WINDOW_SAMPLES, STRIDE_SAMPLES)
    windows = []
    for sample in fixture["samples"]:
        preprocessor.push_sample(sample["node"], sample["sensor"], sample["t"], sample["values"])
        while True:
            window = preprocessor.maybe_emit()
            if window is None:
                break
            windows.append(window)
    return windows


@unittest.skipUnless(shutil.which("node"), "node is required for the parity test")
class PreprocessingParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = build_fixture()
        cls.reference = reference_windows(cls.fixture)
        with tempfile.TemporaryDirectory() as directory:
            fixture_path = Path(directory) / "fixture.json"
            fixture_path.write_text(json.dumps(cls.fixture), encoding="utf-8")
            completed = subprocess.run(
                ["node", str(RUNNER), str(fixture_path)],
                capture_output=True,
                text=True,
                check=False,
            )
        if completed.returncode != 0:
            raise AssertionError(f"browser runner failed:\n{completed.stderr}")
        cls.browser = json.loads(completed.stdout)["windows"]

    def test_fixture_produces_windows(self):
        self.assertGreaterEqual(len(self.reference), 3, "fixture should emit several windows")

    def test_window_bounds_agree(self):
        self.assertEqual(
            len(self.browser), len(self.reference), "window counts differ between implementations"
        )
        for index, (mine, theirs) in enumerate(zip(self.reference, self.browser)):
            self.assertNotIn("invalid", theirs, f"window {index} unexpectedly invalid: {theirs}")
            self.assertAlmostEqual(mine["startS"], theirs["startS"], places=9, msg=f"window {index}")
            self.assertAlmostEqual(mine["endS"], theirs["endS"], places=9, msg=f"window {index}")

    def test_feature_vectors_agree_within_tolerance(self):
        names = build_feature_names()
        self.assertEqual(len(names), 576)
        worst = 0.0
        worst_name = ""
        for index, (mine, theirs) in enumerate(zip(self.reference, self.browser)):
            self.assertEqual(len(theirs["features"]), 576, f"window {index} width")
            for position, (expected, actual) in enumerate(zip(mine["features"], theirs["features"])):
                self.assertTrue(
                    math.isfinite(actual), f"non-finite {names[position]} in window {index}"
                )
                difference = abs(expected - actual)
                relative = difference / max(1.0, abs(expected))
                if relative > worst:
                    worst = relative
                    worst_name = f"{names[position]} (window {index})"
        self.assertLess(
            worst, TOLERANCE, f"largest disagreement {worst:.3e} at {worst_name}"
        )

    def test_fixture_is_discriminating(self):
        """Nearest-sample and linear interpolation must differ on this fixture.

        Without this, the parity test would still pass if both sides silently
        went back to interpolating, since they would agree with each other.
        """
        stream = [
            (sample["t"], sample["values"]["gyro_x_dps"])
            for sample in self.fixture["samples"]
            if sample["node"] == 3 and sample["sensor"] == ICM
        ]
        times = [entry[0] for entry in stream]
        values = [entry[1] for entry in stream]
        grid_start = times[10]
        largest = 0.0
        for step in range(WINDOW_SAMPLES):
            target = grid_start + step * PERIOD + PERIOD / 2.0
            if target >= times[-1]:
                break
            nearest = values[nearest_index(times, target)]
            position = next(i for i in range(1, len(times)) if times[i] >= target)
            span = times[position] - times[position - 1]
            fraction = (target - times[position - 1]) / span if span > 0 else 0.0
            linear = values[position - 1] + (values[position] - values[position - 1]) * fraction
            largest = max(largest, abs(nearest - linear))
        self.assertGreater(
            largest,
            1.0,
            "fixture does not separate nearest-sample from interpolation; "
            "the parity test would not catch a regression",
        )


if __name__ == "__main__":
    unittest.main()
