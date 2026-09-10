"""Rep analyzer tests (Coach Assist correction loop, plan steps 6-7).

The thresholds under test were derived from a deliberately-bad hardware set
(2026-09-10T06:43): three clean reps followed by swinging, elbow drift and
partial-range reps. The discrimination measured there was:

    upper-arm deviation  clean 6.4-9.8 deg | bad 18.9-103.5 deg  CLEAN SPLIT
    off-axis excess      clean 6.1-23.8    | bad 17.2-91.7       OVERLAPS
    peak flexion         clean 132.8-137.6 | bad 96.2-149.3      OVERLAPS

test_recorded_bad_set_is_scored_correctly replays those measured reps and
requires the analyzer to reach the same verdicts, so a future threshold change
cannot silently undo the one thing the hardware session established.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "host" / "tests" / "scripts" / "run_rep_fixture.mjs"

PERIOD_MS = 40.0  # the 25 Hz motion tick


def frame(t_ms, flexion, upper_dev=3.0, off_axis=4.0, calibrated=True):
    """One motion packet, carrying only the fields the analyzer reads."""
    return {
        "timestamp_ms": t_ms,
        "elbow_flexion_deg": flexion,
        "upper_arm_deviation_deg": upper_dev,
        "elbow_off_axis_excess_deg": off_axis,
        "health": {"calibrated": calibrated},
    }


class FrameBuilder:
    def __init__(self):
        self.frames = []
        self.t = 0.0

    def add(self, flexion, upper_dev=3.0, off_axis=4.0, calibrated=True, count=1):
        for _ in range(count):
            self.frames.append(frame(self.t, flexion, upper_dev, off_axis, calibrated))
            self.t += PERIOD_MS
        return self

    #: Frames held per waypoint. Ten waypoints x 5 frames at 25 Hz is a 2.0 s
    #: rep, matching the 1.7-2.2 s measured on hardware. Shorter fixtures are
    #: rejected as twitches by minDurationS, which is the correct behaviour.
    HOLD = 5

    def rep(self, peak=132.0, upper_dev=6.0, off_axis=5.0, off_hinge=0, bottom=5.0):
        """A curl: up to `peak`, back down, with `off_hinge` withheld frames."""
        # Waypoints scale with `peak` so a low-ROM rep never passes through a
        # higher fixed value on the way up.
        ramp = [0.30, 0.60, 0.85]
        rising = [bottom] + [peak * f for f in ramp] + [peak]
        falling = [peak] + [peak * f for f in reversed(ramp)] + [bottom]
        for value in rising:
            self.add(value, upper_dev, off_axis, count=self.HOLD)
        # Withheld frames belong INSIDE the rep, at the top of the movement.
        for _ in range(off_hinge):
            self.add(None, upper_dev, off_axis)
        for value in falling:
            self.add(value, upper_dev, off_axis, count=self.HOLD)
        # Settle below the close threshold so the rep is definitely closed.
        self.add(bottom, upper_dev, off_axis, count=3)
        return self

    #: In-range frames a `rep()` contributes between opening and closing, used
    #: to convert a measured off-hinge FRACTION into a whole frame count.
    IN_RANGE_FRAMES = 8 * HOLD


def run(scenarios):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "rep_fixture.json"
        path.write_text(json.dumps({"scenarios": scenarios}), encoding="utf-8")
        done = subprocess.run(
            ["node", str(RUNNER), str(path)], capture_output=True, text=True, check=False
        )
    if done.returncode != 0:
        raise AssertionError(f"runner failed ({done.returncode}): {done.stderr}")
    parsed = json.loads(done.stdout)
    return {s["name"]: s for s in parsed["scenarios"]}


# Reps measured on hardware, 2026-09-10T06:43. (peak, maxUpperDev, maxOffAxis,
# offHingeFraction, expected verdict).
#
# "uncountable" is the segment that ran 11.4 s at 90% off-hinge -- continuous
# flailing that happened to stay above the close threshold. It is not a rep and
# must not reach the coach's rep count at all.
RECORDED_BAD_SET = [
    (137.6, 6.4, 6.1, 0.00, "correct"),
    (132.8, 9.8, 8.5, 0.00, "correct"),
    (134.3, 8.8, 23.8, 0.00, "correct"),
    (96.2, 95.3, 91.7, 0.73, "fault"),
    (123.4, 103.5, 84.6, 0.90, "uncountable"),
    (121.2, 27.7, 20.6, 0.00, "fault"),
    (120.2, 21.8, 17.2, 0.00, "fault"),
    (114.8, 19.9, 17.9, 0.00, "fault"),
    (140.8, 18.9, 23.8, 0.00, "fault"),
    (121.0, 38.1, 62.8, 0.25, "fault"),
    (149.3, 43.8, 73.1, 0.54, "fault"),
    (108.0, 38.4, 29.7, 0.00, "fault"),
    (141.8, 29.5, 22.1, 0.00, "fault"),
]


@unittest.skipUnless(shutil.which("node"), "node is required for the rep analyzer tests")
class RepAnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = run(cls.build_all())

    @staticmethod
    def build_all():
        scenarios = []

        clean = FrameBuilder()
        for _ in range(4):
            clean.rep(peak=132.0, upper_dev=7.0)
        scenarios.append({"name": "clean", "frames": clean.frames})

        mixed = FrameBuilder()
        mixed.rep(peak=133.0, upper_dev=7.0)          # correct
        mixed.rep(peak=104.0, upper_dev=7.0)          # short range
        mixed.rep(peak=131.0, upper_dev=28.0)         # swinging
        mixed.rep(peak=130.0, upper_dev=7.0, off_hinge=20)  # compound
        scenarios.append({"name": "mixed", "frames": mixed.frames})

        # A twitch and a very long hold: neither is a rep, and neither may
        # consume a rep number.
        junk = FrameBuilder()
        junk.add(5.0, count=3).add(60.0, count=2).add(5.0, count=5)
        junk.add(80.0, count=200).add(5.0, count=5)  # 8 s hold: over maxDurationS
        junk.rep(peak=133.0, upper_dev=6.0)
        scenarios.append({"name": "junk", "frames": junk.frames})

        # Calibration lost mid-rep: the partial rep must be discarded, not
        # scored against a reference that no longer applies.
        lost = FrameBuilder()
        lost.add(5.0, count=3).add(60.0, count=10).add(120.0, count=10)
        lost.add(120.0, calibrated=False, count=3)
        lost.add(5.0, count=3)
        lost.rep(peak=134.0, upper_dev=6.0)
        scenarios.append({"name": "lost_calibration", "frames": lost.frames})

        # Replay of the measured hardware reps.
        recorded = FrameBuilder()
        for peak, dev, off_axis, off_hinge_fraction, _ in RECORDED_BAD_SET:
            n = FrameBuilder.IN_RANGE_FRAMES
            off_hinge = round(off_hinge_fraction * n / max(1e-9, 1 - off_hinge_fraction))
            recorded.rep(peak=peak, upper_dev=dev, off_axis=off_axis, off_hinge=off_hinge)
        scenarios.append({"name": "recorded", "frames": recorded.frames})

        # Coach raises the bar: the same clean reps should now fail ROM.
        strict = FrameBuilder()
        for _ in range(3):
            strict.rep(peak=132.0, upper_dev=7.0)
        scenarios.append(
            {"name": "strict", "frames": strict.frames, "target": {"romTargetDeg": 140}}
        )
        return scenarios

    # ---------------------------------------------------------------- tests

    def test_clean_set_counts_every_rep_as_correct(self):
        s = self.results["clean"]
        self.assertEqual(s["summary"]["total"], 4)
        self.assertEqual(s["summary"]["correct"], 4)
        self.assertEqual(s["summary"]["faults"], {})

    def test_reps_are_numbered_consecutively(self):
        s = self.results["clean"]
        self.assertEqual([rep["index"] for rep in s["reps"]], [1, 2, 3, 4])

    def test_each_fault_type_is_detected(self):
        s = self.results["mixed"]
        self.assertEqual(s["summary"]["total"], 4)
        self.assertEqual(s["summary"]["correct"], 1)
        faults = [rep["faults"] for rep in s["reps"]]
        self.assertEqual(faults[0], [])
        self.assertEqual(faults[1], ["incomplete_rom"])
        self.assertEqual(faults[2], ["upper_arm_movement"])
        self.assertEqual(faults[3], ["compound_motion"])

    def test_twitches_and_long_holds_are_not_reps(self):
        s = self.results["junk"]
        self.assertEqual(s["summary"]["total"], 1)
        self.assertEqual([rep["index"] for rep in s["reps"]], [1])
        malformed = [rep for rep in s["emitted"] if "malformed" in rep["faults"]]
        self.assertGreaterEqual(len(malformed), 2)
        for rep in malformed:
            self.assertIsNone(rep["index"])

    def test_losing_calibration_discards_the_rep_in_progress(self):
        s = self.results["lost_calibration"]
        self.assertEqual(s["summary"]["total"], 1)
        self.assertEqual(s["reps"][0]["faults"], [])

    def test_recorded_bad_set_is_scored_correctly(self):
        """Every rep measured on hardware must get the verdict a coach would."""
        s = self.results["recorded"]
        countable = [row for row in RECORDED_BAD_SET if row[4] != "uncountable"]
        self.assertEqual(s["summary"]["total"], len(countable))
        for rep, expected in zip(s["reps"], countable):
            peak, dev, _, _, verdict = expected
            with self.subTest(peak=peak, dev=dev):
                self.assertEqual(rep["verdict"], verdict)
                self.assertAlmostEqual(rep["peakFlexionDeg"], peak, delta=0.5)

    def test_prolonged_flailing_never_reaches_the_rep_count(self):
        """The 11.4 s / 90% off-hinge segment is not a rep at any verdict."""
        s = self.results["recorded"]
        self.assertNotIn(123.4, [round(rep["peakFlexionDeg"], 1) for rep in s["reps"]])
        malformed = [rep for rep in s["emitted"] if "malformed" in rep["faults"]]
        self.assertEqual(len(malformed), 1)
        self.assertIsNone(malformed[0]["index"])

    def test_recorded_clean_reps_pass_and_the_rest_do_not(self):
        s = self.results["recorded"]
        self.assertEqual(s["summary"]["correct"], 3)
        self.assertEqual(s["summary"]["total"] - s["summary"]["correct"], 9)

    def test_swinging_is_the_dominant_recorded_fault(self):
        """Upper-arm deviation is the metric that carries the judgement."""
        tally = self.results["recorded"]["summary"]["faults"]
        self.assertEqual(tally["upper_arm_movement"], 9)
        # Every faulted rep was caught by deviation; ROM and compound motion
        # only ever add detail on top of it.
        self.assertLess(tally.get("incomplete_rom", 0), tally["upper_arm_movement"])

    def test_target_is_coach_adjustable(self):
        s = self.results["strict"]
        self.assertEqual(s["summary"]["correct"], 0)
        for rep in s["reps"]:
            self.assertEqual(rep["faults"], ["incomplete_rom"])

    def test_summary_reports_consistency(self):
        summary = self.results["clean"]["summary"]
        self.assertAlmostEqual(summary["meanPeakDeg"], 132.0, delta=0.5)
        self.assertLess(summary["sdPeakDeg"], 1.0)
        self.assertTrue(summary["consistent"])


if __name__ == "__main__":
    unittest.main()
