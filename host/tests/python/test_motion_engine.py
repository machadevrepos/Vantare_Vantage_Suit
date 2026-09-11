"""Motion Engine correctness tests (Coach Assist Milestone 4).

These tests exist to prove the one property the whole calibration approach
rests on: the reported joint and segment angles must NOT depend on how the PCBs
happen to be rotated on their straps. Every scenario below builds the sensor
readings from a known-true body pose composed with an arbitrary, deliberately
awkward mount rotation the engine is never told about, then requires the engine
to recover the true angle.

Construction (mirrors the argument in motion-engine.js):
    true segment orientation  S_n(t)          (world <- segment)
    constant mount rotation   M_n             (segment <- sensor), unknown
    reported quaternion       q_n(t) = S_n(t) * conj(M_n)

so that q_n * M_n == S_n. The engine only ever sees q_n.

Scenarios cover the plan's "First Acceptance Checks" table: calibration
stability and motion rejection, elbow separation at straight / 45 / 90 / 120,
remount repeatability, and exercise independence (a shoulder-press-like motion
still moves the segment state with no classifier involved).
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "host" / "tests" / "scripts" / "run_motion_fixture.mjs"

UPPER_ARM = 4
FOREARM = 2
AUX = 3

SAMPLE_PERIOD_MS = 40.0  # the qualified 25 Hz live grid
ANGLE_TOLERANCE_DEG = 1e-6


# --------------------------------------------------------------- quaternion ref
# Independent of the JS implementation on purpose: if both were the same code a
# sign error would cancel out and the test would pass on wrong math.


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def q_axis_angle(axis, degrees):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / n, ay / n, az / n
    half = math.radians(degrees) / 2.0
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


def q_angle_deg(q):
    q = q_norm(q)
    return 2.0 * math.degrees(math.acos(min(abs(q[0]), 1.0)))


def packet_to_q(packet):
    return (packet["qw"], packet["qx"], packet["qy"], packet["qz"])


def to_values(q, gyro=(0.0, 0.0, 0.0)):
    """The subset of BNO columns the Motion Engine reads."""
    w, x, y, z = q
    return {
        "quat_i": x,
        "quat_j": y,
        "quat_k": z,
        "quat_real": w,
        "gyro_x_radps": gyro[0],
        "gyro_y_radps": gyro[1],
        "gyro_z_radps": gyro[2],
    }


# Deliberately ugly mount rotations: no axis-aligned, no small-angle case, and
# a different one per node. Nothing in the engine is told about these.
MOUNTS = {
    UPPER_ARM: q_axis_angle((0.31, -0.77, 0.55), 137.0),
    FOREARM: q_axis_angle((-0.62, 0.19, 0.76), 84.0),
    AUX: q_axis_angle((0.44, 0.44, -0.78), 21.0),
}

# Each Game Rotation Vector powers up with an arbitrary heading. These are
# explicit, large, per-sensor reference-frame offsets applied on the LEFT of
# the reported quaternion (the reported frame is the true world frame rotated
# by the sensor's own heading). They are NOT part of the default scenarios:
# an engine quantity that cancels them (single shared world frame) but not
# the headings is exactly the bug class the anatomical scenarios must never
# reintroduce - see scenario_anatomical_with_heading_offsets.
HEADINGS = {
    UPPER_ARM: q_axis_angle((0.0, 0.0, 1.0), 37.0),
    FOREARM: q_axis_angle((0.0, 0.0, 1.0), -112.0),
    AUX: q_axis_angle((0.0, 0.0, 1.0), 71.0),
}

# Arbitrary neutral body pose, and an arbitrary world yaw offset per sensor to
# stand in for the Game Rotation Vector's unreferenced heading.
NEUTRAL = {
    UPPER_ARM: q_axis_angle((0.0, 0.0, 1.0), 23.0),
    FOREARM: q_axis_angle((0.1, 0.2, 0.97), 41.0),
    AUX: q_axis_angle((0.3, 0.5, 0.81), 12.0),
}


def reported(node, segment_orientation):
    """q_n = S_n * conj(M_n) - what the BNO would report for this true pose."""
    return q_norm(q_mul(segment_orientation, q_conj(MOUNTS[node])))


class MotionFixtureBuilder:
    """Accumulates fixture steps with a monotonically advancing fake clock."""

    def __init__(self, name, options=None):
        self.name = name
        self.options = options or {}
        self.steps = []
        self.now_ms = 1000.0
        self.auto = 0

    def advance(self, ms):
        self.now_ms += ms

    def begin_calibration(self):
        self.steps.append({"op": "beginCalibration", "nowMs": self.now_ms})

    def begin_hinge(self):
        self.steps.append({"op": "beginHinge", "nowMs": self.now_ms})

    def begin_side(self):
        self.steps.append({"op": "beginSide", "nowMs": self.now_ms})

    def begin_forward(self):
        self.steps.append({"op": "beginForward", "nowMs": self.now_ms})

    def rezero(self):
        self.steps.append({"op": "rezero"})

    def rep(self, axis=(0.0, 1.0, 0.0), angles=(50, 80, 110, 130, 110, 80, 50), holds=2):
        """One flexion rep about `axis`.

        A frame is emitted after every pose because the hinge capture consumes
        elbow rotations from computeFrame, exactly as the app's motion tick does.
        """
        for degrees in angles:
            for _ in range(holds):
                self.push_pose(MotionEngineTest.flexed_poses(degrees, axis))
                self.auto_frame()

    def auto_frame(self):
        self.auto += 1
        self.frame(f"_tick{self.auto}")

    def clear_calibration(self):
        self.steps.append({"op": "clearCalibration"})

    def push_pose(self, poses, gyro=(0.0, 0.0, 0.0), skew_ms=0.0):
        """One 25 Hz tick: every node reports its pose, then the clock advances.

        `skew_ms` offsets the forearm's device timestamp only, which is what the
        engine's synchronization check looks at.
        """
        for node, segment in poses.items():
            device_s = self.now_ms / 1000.0
            if node == FOREARM:
                device_s += skew_ms / 1000.0
            self.steps.append(
                {
                    "op": "sample",
                    "node": node,
                    "values": to_values(reported(node, segment), gyro),
                    "deviceS": device_s,
                    "nowMs": self.now_ms,
                }
            )
        self.advance(SAMPLE_PERIOD_MS)

    def hold(self, poses, seconds, gyro=(0.0, 0.0, 0.0)):
        for _ in range(int(seconds * 1000 / SAMPLE_PERIOD_MS)):
            self.push_pose(poses, gyro)

    def frame(self, label):
        self.steps.append({"op": "frame", "label": label, "nowMs": self.now_ms})

    def build(self):
        return {"name": self.name, "options": self.options, "steps": self.steps}


def run_scenarios(scenarios):
    fixture = {"scenarios": scenarios}
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "motion_fixture.json"
        path.write_text(json.dumps(fixture), encoding="utf-8")
        completed = subprocess.run(
            ["node", str(RUNNER), str(path)],
            capture_output=True,
            text=True,
            check=False,
        )
    if completed.returncode != 0:
        raise AssertionError(f"runner failed ({completed.returncode}): {completed.stderr}")
    parsed = json.loads(completed.stdout)
    return {scenario["name"]: scenario for scenario in parsed["scenarios"]}, parsed


@unittest.skipUnless(shutil.which("node"), "node is required for the Motion Engine tests")
class MotionEngineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results, cls.raw = run_scenarios(cls.build_all())

    # ------------------------------------------------------------- scenarios

    @staticmethod
    def build_all():
        return [
            MotionEngineTest.scenario_elbow_sweep(),
            MotionEngineTest.scenario_remount(),
            MotionEngineTest.scenario_upper_arm_only(),
            MotionEngineTest.scenario_motion_rejects_calibration(),
            MotionEngineTest.scenario_common_mode_yaw_drift(),
            MotionEngineTest.scenario_uncalibrated_and_stale(),
            MotionEngineTest.scenario_hinge_calibration(),
            MotionEngineTest.scenario_hinge_rejects_compound_motion(),
            MotionEngineTest.scenario_flexion_gate_and_drift(),
            MotionEngineTest.scenario_anatomical_calibration(),
            MotionEngineTest.scenario_anatomical_rejects_collinear_poses(),
            MotionEngineTest.scenario_anatomical_rejects_motion(),
            MotionEngineTest.scenario_anatomical_rejects_under_raise(),
            MotionEngineTest.scenario_anatomical_rejects_bent_elbow(),
            MotionEngineTest.scenario_anatomical_rejects_bent_elbow_hidden(),
            MotionEngineTest.scenario_anatomical_rejects_offplane_side_raise(),
            MotionEngineTest.scenario_anatomical_with_heading_offsets(),
            MotionEngineTest.scenario_anatomical_rejects_sync_loss(),
            MotionEngineTest.scenario_anatomical_reset(),
        ]

    @staticmethod
    def calibrated_builder(name, options=None):
        """Common opening: neutral pose held still long enough to calibrate."""
        builder = MotionFixtureBuilder(name, options)
        builder.hold(NEUTRAL, 0.4)
        builder.begin_calibration()
        builder.hold(NEUTRAL, 2.0)
        return builder

    @staticmethod
    def flexed_poses(degrees, axis=(0.0, 1.0, 0.0)):
        """Neutral upper arm; forearm flexed by `degrees` about a segment axis."""
        return {
            UPPER_ARM: NEUTRAL[UPPER_ARM],
            FOREARM: q_mul(NEUTRAL[FOREARM], q_axis_angle(axis, degrees)),
            AUX: NEUTRAL[AUX],
        }

    @staticmethod
    def rigid_arm_pose(axis, degrees):
        """Whole rigid arm about a world-fixed axis (physical locked elbow).

        The raise composes on the LEFT of each segment's neutral world
        orientation, exactly like scenario_upper_arm_only. Composing per
        segment on the right would rotate each sensor differently in the
        world, which no locked elbow can do - the mount-independent
        elbow-relative capture gate rightly rejects it as a bend."""
        lift = q_axis_angle(axis, degrees)
        return {
            UPPER_ARM: q_mul(lift, NEUTRAL[UPPER_ARM]),
            FOREARM: q_mul(lift, NEUTRAL[FOREARM]),
            AUX: NEUTRAL[AUX],
        }

    @staticmethod
    def capture_anatomical(builder):
        builder.begin_side()
        builder.hold(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0), 1.2)
        builder.begin_forward()
        builder.hold(MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0), 1.2)

    @staticmethod
    def scenario_elbow_sweep():
        # Plan acceptance check: straight / ~45 / ~90 / ~120 must be clearly
        # separated, and the engine must read them back exactly.
        builder = MotionEngineTest.calibrated_builder("elbow_sweep")
        for degrees in (0.0, 45.0, 90.0, 120.0, 150.0):
            builder.hold(MotionEngineTest.flexed_poses(degrees), 0.2)
            builder.frame(f"elbow_{int(degrees)}")
        return builder.build()

    @staticmethod
    def scenario_remount():
        # Remount repeatability: same physical pose, recalibrated, must give the
        # same answer. Handled here by recalibrating at a DIFFERENT neutral and
        # then reproducing the same relative flexion.
        builder = MotionFixtureBuilder("remount")
        shifted = {
            UPPER_ARM: q_mul(NEUTRAL[UPPER_ARM], q_axis_angle((0.2, 0.9, 0.3), 65.0)),
            FOREARM: q_mul(NEUTRAL[FOREARM], q_axis_angle((0.7, -0.4, 0.6), 110.0)),
            AUX: NEUTRAL[AUX],
        }
        builder.hold(shifted, 0.4)
        builder.begin_calibration()
        builder.hold(shifted, 2.0)
        for degrees in (0.0, 90.0):
            poses = {
                UPPER_ARM: shifted[UPPER_ARM],
                FOREARM: q_mul(shifted[FOREARM], q_axis_angle((0.0, 1.0, 0.0), degrees)),
                AUX: shifted[AUX],
            }
            builder.hold(poses, 0.2)
            builder.frame(f"remount_elbow_{int(degrees)}")
        return builder.build()

    @staticmethod
    def scenario_upper_arm_only():
        # Exercise independence: a shoulder-press-like motion rotates the upper
        # arm with the elbow locked. upper_arm_deviation must track it and the
        # elbow must stay at zero - no classifier anywhere in the path.
        builder = MotionEngineTest.calibrated_builder("upper_arm_only")
        for degrees in (0.0, 30.0, 75.0):
            lift = q_axis_angle((1.0, 0.0, 0.0), degrees)
            poses = {
                # Elbow locked: the whole arm turns as one rigid body about a
                # world-fixed axis, so `lift` is applied on the LEFT. Composing
                # on the right would instead rotate each segment about its own
                # body axis, which is a different (and non-rigid) motion.
                UPPER_ARM: q_mul(lift, NEUTRAL[UPPER_ARM]),
                FOREARM: q_mul(lift, NEUTRAL[FOREARM]),
                AUX: NEUTRAL[AUX],
            }
            builder.hold(poses, 0.2)
            builder.frame(f"press_{int(degrees)}")
        return builder.build()

    @staticmethod
    def scenario_motion_rejects_calibration():
        # Calibration must reject a pose captured while the wearer is moving.
        builder = MotionFixtureBuilder("motion_rejects", {"calibrationTimeoutMs": 4000})
        builder.hold(NEUTRAL, 0.4)
        builder.begin_calibration()
        builder.hold(NEUTRAL, 5.0, gyro=(0.5, 0.0, 0.0))
        builder.frame("after_motion")
        return builder.build()

    @staticmethod
    def scenario_common_mode_yaw_drift():
        # Game Rotation Vector drift that is common to both sensors must cancel
        # out of the elbow angle. This locks in the property that protects the
        # joint measurement even though absolute heading is unreliable.
        builder = MotionEngineTest.calibrated_builder("common_mode_drift")
        drift = q_axis_angle((0.0, 0.0, 1.0), 12.0)
        poses = MotionEngineTest.flexed_poses(90.0)
        drifted = {node: q_mul(drift, q) for node, q in poses.items()}
        builder.hold(drifted, 0.2)
        builder.frame("drifted_elbow_90")
        return builder.build()

    @staticmethod
    def scenario_uncalibrated_and_stale():
        # Before calibration the packet must still be emitted, with health
        # telling the consumer why the values are null. Then a large device-time
        # skew must clear `synchronized` without killing the frame.
        builder = MotionFixtureBuilder("uncalibrated_and_stale")
        builder.hold(NEUTRAL, 0.4)
        builder.frame("uncalibrated")
        builder.begin_calibration()
        builder.hold(NEUTRAL, 2.0)
        builder.push_pose(NEUTRAL, skew_ms=250.0)
        builder.frame("skewed")
        return builder.build()

    @staticmethod
    def scenario_hinge_calibration():
        """Range calibration: a few reps must recover the true hinge axis.

        The whole point is that the unsigned composite angle saturates on
        compound motion (session 2026-09-10T04:31 read 179.9 deg, at the fold
        limit). Projecting onto a measured axis must give a signed joint angle
        instead, and must flag off-axis motion separately.
        """
        builder = MotionEngineTest.calibrated_builder("hinge")
        builder.begin_hinge()
        for _ in range(4):
            builder.rep()
        # Signed readback at known flexions, plus a negative (hyperextension).
        for degrees in (45.0, 90.0, 130.0, -25.0):
            for _ in range(3):
                builder.push_pose(MotionEngineTest.flexed_poses(degrees))
            builder.frame(f"flex_{int(degrees)}")
        # Motion about an axis perpendicular to the hinge, kept below the
        # validity gate: flexion stays near zero while the off-axis term carries
        # the whole rotation. The gate's own behaviour is covered separately by
        # scenario_flexion_gate_and_drift.
        for _ in range(3):
            builder.push_pose(MotionEngineTest.flexed_poses(25.0, axis=(1.0, 0.0, 0.0)))
        builder.frame("off_axis_25")
        return builder.build()

    @staticmethod
    def scenario_hinge_rejects_compound_motion():
        """Incoherent motion must NOT yield a hinge axis."""
        # Timeout sized so the capture runs past the sample minimum first: the
        # rejection must come from axis spread, not from too little data.
        builder = MotionEngineTest.calibrated_builder(
            "hinge_incoherent", {"hingeTimeoutMs": 2000}
        )
        builder.begin_hinge()
        # Each rep rotates about a different axis, so no consistent hinge exists.
        for axis in ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 1.0, 0.0)):
            builder.rep(axis=axis)
        builder.frame("after_incoherent")
        return builder.build()

    @staticmethod
    def scenario_flexion_gate_and_drift():
        """Validity gate on off-axis motion, plus the drift monitor.

        Both come straight from hardware sessions on 2026-09-10: the random
        movement set showed 26% of frames reporting an impossible |flexion| >
        150 deg, and the 60 s stillness hold showed the relative rotation
        growing +9.9 deg/min almost entirely off-hinge.
        """
        builder = MotionEngineTest.calibrated_builder("gate_and_drift")
        builder.begin_hinge()
        for _ in range(4):
            builder.rep()

        # A large rotation about an axis perpendicular to the hinge: the signed
        # angle must be withheld rather than reported as a confident number.
        for _ in range(3):
            builder.push_pose(MotionEngineTest.flexed_poses(120.0, axis=(1.0, 0.0, 0.0)))
        builder.frame("far_off_axis")

        # Back near neutral and held still: this is a rest observation, and with
        # no simulated drift the estimate must stay at zero.
        builder.hold(MotionEngineTest.flexed_poses(0.0), 1.0)
        builder.frame("rest_no_drift")

        # Now simulate drift: rotate only the FOREARM sensor's reference, which
        # is what independent yaw drift between two sensors looks like.
        drift = q_axis_angle((0.2, 0.3, 0.93), 12.0)
        drifted = {
            UPPER_ARM: NEUTRAL[UPPER_ARM],
            FOREARM: q_mul(drift, NEUTRAL[FOREARM]),
            AUX: NEUTRAL[AUX],
        }
        builder.hold(drifted, 1.2)
        builder.frame("rest_with_drift")
        # An implausible flexion at modest off-axis: the twist term can run to
        # +/-180 when the quaternion scalar nears zero, which the off-axis gate
        # alone does not catch. 41 real frames did exactly this on 2026-09-10.
        for _ in range(3):
            builder.push_pose(MotionEngineTest.flexed_poses(172.0))
        builder.frame("implausible_flexion")

        builder.rezero()
        for _ in range(3):
            builder.push_pose(drifted)
        builder.frame("after_rezero")
        return builder.build()

    @staticmethod
    def scenario_anatomical_calibration():
        builder = MotionEngineTest.calibrated_builder("anatomical")
        MotionEngineTest.capture_anatomical(builder)
        # Repeat the calibrated directions physically, then move on: the
        # corrected segments must read back the anatomical targets exactly
        # and the elbow must read straight throughout.
        builder.hold(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0), 0.2)
        builder.frame("side_again")
        builder.hold(MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0), 0.2)
        builder.frame("forward_again")
        builder.hold(MotionEngineTest.rigid_arm_pose((0.31, -0.52, 0.79), 73.0), 0.2)
        builder.frame("combined")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_collinear_poses():
        builder = MotionEngineTest.calibrated_builder("anatomical_collinear")
        side = MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0)
        builder.begin_side()
        builder.hold(side, 1.2)
        builder.begin_forward()
        builder.hold(side, 1.2)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_motion():
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_motion", {"anatomicalTimeoutMs": 1800}
        )
        builder.begin_side()
        builder.hold(
            MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0),
            2.2,
            gyro=(0.4, 0.0, 0.0),
        )
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_under_raise():
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_under_raise", {"anatomicalTimeoutMs": 1800}
        )
        builder.begin_side()
        builder.hold(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 45.0), 2.2)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_bent_elbow():
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_bent", {"anatomicalTimeoutMs": 1800}
        )
        poses = {
            UPPER_ARM: q_mul(NEUTRAL[UPPER_ARM], q_axis_angle((0.0, 0.0, -1.0), 90.0)),
            FOREARM: q_mul(NEUTRAL[FOREARM], q_axis_angle((0.0, 0.0, -1.0), 65.0)),
            AUX: NEUTRAL[AUX],
        }
        builder.begin_side()
        builder.hold(poses, 2.2)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def headed(node, world):
        """Report a world pose through node's own GRV heading offset."""
        return q_mul(HEADINGS[node], world)

    @staticmethod
    def hold_headed(builder, pose, seconds):
        builder.hold(
            {n: MotionEngineTest.headed(n, pose[n]) for n in (UPPER_ARM, FOREARM, AUX)},
            seconds,
        )

    @staticmethod
    def scenario_anatomical_rejects_bent_elbow_hidden():
        """A 45-degree elbow bend during the side raise is invisible to the
        magnitude gates (both segments still read ~90 degrees of raise), but
        it contaminates the observed side axis, so the forward solve sees
        axes far from a right angle and must reject. An inter-sensor
        elbow-relative gate cannot provide this coverage: GRV heading
        offsets leak into conj(qu)*qf once the upper arm rotates."""
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_bent_hidden", {"anatomicalTimeoutMs": 8000}
        )
        upper_world = q_mul(q_axis_angle((0.2, -0.3, 0.93), 90.0), NEUTRAL[UPPER_ARM])
        elbow_offset = q_mul(q_conj(NEUTRAL[UPPER_ARM]), NEUTRAL[FOREARM])
        fore_world = q_mul(
            q_mul(upper_world, elbow_offset), q_axis_angle((0.0, 1.0, 0.0), 45.0)
        )
        builder.begin_side()
        builder.hold({UPPER_ARM: upper_world, FOREARM: fore_world, AUX: NEUTRAL[AUX]}, 1.2)
        builder.begin_forward()
        builder.hold(MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0), 1.2)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_with_heading_offsets():
        """The 2026-09-11 field failure: every real GRV carries its own
        power-up heading, and a straight side raise would not calibrate.
        Headings of +37/-112/+71 degrees must not block the capture, the
        solve must map each node's observed axes onto the anatomical targets
        exactly, and the axis separation must stay near a right angle."""
        builder = MotionFixtureBuilder("anatomical_headings")
        neutral = {n: MotionEngineTest.headed(n, NEUTRAL[n]) for n in NEUTRAL}
        builder.hold(neutral, 0.4)
        builder.begin_calibration()
        builder.hold(neutral, 2.0)
        side = MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0)
        forward = MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0)
        builder.begin_side()
        MotionEngineTest.hold_headed(builder, side, 1.2)
        builder.begin_forward()
        MotionEngineTest.hold_headed(builder, forward, 1.2)
        MotionEngineTest.hold_headed(builder, side, 0.3)
        builder.frame("side_again")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_offplane_side_raise():
        """A side raise 30 degrees forward of the coronal plane puts the two
        observed axes 60 degrees apart: inside the old 60-120 window, outside
        the tightened 80-100 one. Acceptance (spec 11.3) allows 10 degrees of
        display error; accepting this fault would render 30 degrees off."""
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_offplane", {"anatomicalTimeoutMs": 4000}
        )
        offplane = MotionEngineTest.rigid_arm_pose((0.5, 0.0, -0.866), 90.0)
        builder.begin_side()
        builder.hold(offplane, 1.2)
        builder.begin_forward()
        builder.hold(MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0), 1.2)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_rejects_sync_loss():
        builder = MotionEngineTest.calibrated_builder(
            "anatomical_sync", {"anatomicalTimeoutMs": 1800}
        )
        builder.begin_side()
        pose = MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0)
        for _ in range(55):
            builder.push_pose(pose, skew_ms=150.0)
        builder.frame("after_rejection")
        return builder.build()

    @staticmethod
    def scenario_anatomical_reset():
        builder = MotionEngineTest.calibrated_builder("anatomical_reset")
        MotionEngineTest.capture_anatomical(builder)
        builder.clear_calibration()
        builder.frame("after_clear")
        return builder.build()

    # ----------------------------------------------------------------- tests

    def frames(self, scenario):
        return {entry["label"]: entry for entry in self.results[scenario]["frames"]}

    def test_calibration_completes_on_a_still_neutral_pose(self):
        self.assertEqual(self.results["elbow_sweep"]["calibrationState"], "calibrated")
        kinds = [event["kind"] for event in self.results["elbow_sweep"]["events"]]
        self.assertIn("calibration_complete", kinds)

    def test_neutral_pose_reads_zero(self):
        frame = self.frames("elbow_sweep")["elbow_0"]["frame"]
        self.assertAlmostEqual(frame["elbow_relative_rotation_deg"], 0.0, places=6)
        self.assertAlmostEqual(frame["upper_arm_deviation_deg"], 0.0, places=6)
        self.assertTrue(frame["health"]["calibrated"])
        self.assertTrue(frame["health"]["synchronized"])

    def test_elbow_angles_recovered_exactly_despite_unknown_mounts(self):
        """The core claim: mount rotations cancel, so the true angle comes back."""
        frames = self.frames("elbow_sweep")
        for degrees in (45.0, 90.0, 120.0, 150.0):
            with self.subTest(degrees=degrees):
                measured = frames[f"elbow_{int(degrees)}"]["frame"][
                    "elbow_relative_rotation_deg"
                ]
                self.assertAlmostEqual(measured, degrees, delta=1e-4)

    def test_elbow_positions_are_clearly_separated(self):
        """Plan acceptance check: straight / 45 / 90 / 120 must not overlap."""
        frames = self.frames("elbow_sweep")
        values = [
            frames[f"elbow_{d}"]["frame"]["elbow_relative_rotation_deg"]
            for d in (0, 45, 90, 120)
        ]
        gaps = [b - a for a, b in zip(values, values[1:])]
        # The tightest true gap in this sweep is 90 -> 120, so the bar is set
        # below that: what is being asserted is that no two coaching-relevant
        # positions collapse together, not the sweep's own spacing.
        self.assertTrue(all(gap > 20.0 for gap in gaps), f"gaps too small: {gaps}")

    def test_flexion_does_not_leak_into_upper_arm_deviation(self):
        frame = self.frames("elbow_sweep")["elbow_90"]["frame"]
        self.assertAlmostEqual(frame["upper_arm_deviation_deg"], 0.0, places=6)

    def test_remount_reproduces_the_same_angle(self):
        """A different strap position + fresh calibration -> same 90 deg."""
        frames = self.frames("remount")
        self.assertAlmostEqual(
            frames["remount_elbow_0"]["frame"]["elbow_relative_rotation_deg"], 0.0, places=6
        )
        self.assertAlmostEqual(
            frames["remount_elbow_90"]["frame"]["elbow_relative_rotation_deg"], 90.0, delta=1e-4
        )

    def test_upper_arm_motion_tracks_with_elbow_locked(self):
        """Exercise independence: no classifier, the segment state just moves."""
        frames = self.frames("upper_arm_only")
        for degrees in (30.0, 75.0):
            with self.subTest(degrees=degrees):
                frame = frames[f"press_{int(degrees)}"]["frame"]
                self.assertAlmostEqual(
                    frame["upper_arm_deviation_deg"], degrees, delta=1e-4
                )
                self.assertAlmostEqual(
                    frame["elbow_relative_rotation_deg"], 0.0, delta=1e-4
                )

    def test_calibration_rejects_a_moving_pose(self):
        scenario = self.results["motion_rejects"]
        self.assertEqual(scenario["calibrationState"], "failed")
        frame = self.frames("motion_rejects")["after_motion"]["frame"]
        self.assertFalse(frame["health"]["calibrated"])
        self.assertIsNone(frame["elbow_relative_rotation_deg"])

    def test_common_mode_yaw_drift_cancels_from_the_elbow_angle(self):
        frame = self.frames("common_mode_drift")["drifted_elbow_90"]["frame"]
        self.assertAlmostEqual(frame["elbow_relative_rotation_deg"], 90.0, delta=1e-4)

    def test_uncalibrated_frame_is_emitted_with_honest_health(self):
        frame = self.frames("uncalibrated_and_stale")["uncalibrated"]["frame"]
        self.assertFalse(frame["health"]["calibrated"])
        self.assertIsNone(frame["upper_arm_orientation"])
        self.assertIsNone(frame["elbow_relative_rotation_deg"])
        # Nodes are streaming even though calibration has not happened.
        self.assertTrue(frame["health"]["n2"])
        self.assertTrue(frame["health"]["n4"])

    def test_device_time_skew_clears_synchronized_without_dropping_the_frame(self):
        frame = self.frames("uncalibrated_and_stale")["skewed"]["frame"]
        self.assertFalse(frame["health"]["synchronized"])
        self.assertGreater(abs(frame["diagnostics"]["skewMs"]), 60.0)
        # The packet is still produced; degraded, not frozen.
        self.assertIsNotNone(frame["elbow_relative_rotation_deg"])

    def test_scope_limits_are_carried_in_band(self):
        """The 3D team must not be able to mistake this for trunk-relative data."""
        frame = self.frames("elbow_sweep")["elbow_90"]["frame"]
        self.assertFalse(frame["diagnostics"]["trunkReferenced"])
        self.assertEqual(frame["diagnostics"]["axisFrame"], "sensor_neutral")

    def test_directional_poses_recover_anatomical_segment_axes(self):
        """The solver's exact promise, checked on physical poses: after the
        three-pose capture, a repeated side raise reads back the anatomical
        side target on BOTH corrected segments with a straight elbow, the
        same holds for the forward target, and a rigid arbitrary raise keeps
        the elbow at identity."""
        scenario = self.results["anatomical"]
        self.assertEqual(scenario["anatomicalState"], "calibrated", scenario["anatomicalMessage"])
        frames = self.frames("anatomical")
        targets = {
            "side_again": q_axis_angle((0.0, 0.0, -1.0), 90.0),
            "forward_again": q_axis_angle((1.0, 0.0, 0.0), 90.0),
        }
        for label, target in targets.items():
            frame = frames[label]["frame"]
            for field in ("upper_arm_orientation", "forearm_orientation"):
                measured = packet_to_q(frame[field])
                error = q_angle_deg(q_mul(q_conj(target), measured))
                self.assertLess(error, 1e-3, f"{label} {field} anatomical error {error}")
            self.assertAlmostEqual(frame["elbow_relative_rotation_deg"], 0.0, delta=1e-3)
        combined = frames["combined"]["frame"]
        for field in ("upper_arm_orientation", "forearm_orientation"):
            self.assertAlmostEqual(
                q_angle_deg(packet_to_q(combined[field])), 73.0, delta=1e-3,
                msg=f"{field} must keep the raise magnitude",
            )
        self.assertAlmostEqual(combined["elbow_relative_rotation_deg"], 0.0, delta=1e-3)
        self.assertEqual(frames["side_again"]["frame"]["diagnostics"]["axisFrame"], "anatomical")
        self.assertEqual(set(scenario["mountCorrections"]), {"2", "4"})

    def test_collinear_directional_poses_are_rejected_atomically(self):
        scenario = self.results["anatomical_collinear"]
        self.assertEqual(scenario["anatomicalState"], "failed")
        self.assertEqual(scenario["mountCorrections"], {})
        frame = self.frames("anatomical_collinear")["after_rejection"]["frame"]
        self.assertEqual(frame["diagnostics"]["axisFrame"], "sensor_neutral")
        self.assertIn("independent", scenario["anatomicalMessage"].lower())

    def test_offplane_side_raise_is_rejected_at_solve_time(self):
        scenario = self.results["anatomical_offplane"]
        self.assertEqual(scenario["anatomicalState"], "failed", scenario["anatomicalMessage"])
        self.assertIn("independent", scenario["anatomicalMessage"].lower())
        self.assertEqual(scenario["mountCorrections"], {})
        frame = self.frames("anatomical_offplane")["after_rejection"]["frame"]
        self.assertEqual(frame["diagnostics"]["axisFrame"], "sensor_neutral")

    def test_clear_calibration_removes_anatomical_mounts(self):
        scenario = self.results["anatomical_reset"]
        self.assertEqual(scenario["mountCorrections"], {})
        self.assertEqual(scenario["anatomicalState"], "none")
        frame = self.frames("anatomical_reset")["after_clear"]["frame"]
        self.assertEqual(frame["diagnostics"]["axisFrame"], "sensor_neutral")

    def test_directional_capture_rejects_continuous_motion(self):
        scenario = self.results["anatomical_motion"]
        self.assertEqual(scenario["anatomicalState"], "failed")
        self.assertIn("motion", scenario["anatomicalMessage"].lower())

    def test_directional_capture_rejects_under_raise(self):
        scenario = self.results["anatomical_under_raise"]
        self.assertEqual(scenario["anatomicalState"], "failed")
        self.assertIn("60-120", scenario["anatomicalMessage"])
        self.assertNotIn("..", scenario["anatomicalMessage"], "doubled period")
        # Rejections must be auditable (spec section 9): the restarts carry
        # the reason, not just the final timeout.
        restarted = [
            e for e in scenario["events"] if e["kind"] == "anatomical_hold_restarted"
        ]
        self.assertTrue(restarted, "capture restarts must be logged")
        self.assertTrue(any("60-120" in (e.get("reason") or "") for e in restarted))

    def test_directional_capture_rejects_bent_elbow(self):
        scenario = self.results["anatomical_bent"]
        self.assertEqual(scenario["anatomicalState"], "failed")
        self.assertIn("elbow straight", scenario["anatomicalMessage"].lower())

    def test_directional_capture_catches_bent_elbow_the_magnitude_gate_misses(self):
        """45 degrees of bend with both raise magnitudes still ~90 passes the
        magnitude gates, but the contaminated side axis fails the 80-100
        degree separation check at the forward solve."""
        scenario = self.results["anatomical_bent_hidden"]
        self.assertEqual(scenario["anatomicalState"], "failed", scenario["anatomicalMessage"])
        self.assertIn("independent", scenario["anatomicalMessage"].lower())
        self.assertEqual(scenario["mountCorrections"], {})

    def test_anatomical_capture_completes_despite_heading_offsets(self):
        """GRV headings do NOT cancel in inter-sensor products, so every
        capture gate must be conjugation-invariant. Headings of
        +37/-112/+71 degrees must not block calibration, the solve must map
        the observed axes onto the anatomical targets exactly, and the
        reported axis separation must stay near 90 degrees."""
        scenario = self.results["anatomical_headings"]
        self.assertEqual(scenario["anatomicalState"], "calibrated", scenario["anatomicalMessage"])
        complete = next(
            e for e in scenario["events"] if e["kind"] == "anatomical_complete"
        )
        for node_id, quality in complete["quality"]["nodes"].items():
            self.assertGreater(
                quality["axisSeparationDeg"], 85.0, f"N{node_id} separation"
            )
            self.assertLess(quality["axisSeparationDeg"], 95.0, f"N{node_id} separation")
        frame = self.frames("anatomical_headings")["side_again"]["frame"]
        target = q_axis_angle((0.0, 0.0, -1.0), 90.0)
        for field in ("upper_arm_orientation", "forearm_orientation"):
            measured = packet_to_q(frame[field])
            error = q_angle_deg(q_mul(q_conj(target), measured))
            self.assertLess(error, 1e-3, f"{field} error under heading offsets {error}")
        self.assertEqual(frame["diagnostics"]["axisFrame"], "anatomical")

    def test_directional_capture_rejects_unsynchronized_nodes(self):
        scenario = self.results["anatomical_sync"]
        self.assertEqual(scenario["anatomicalState"], "failed")
        self.assertIn("skew", scenario["anatomicalMessage"].lower())

    def test_hinge_calibration_finds_a_consistent_axis(self):
        scenario = self.results["hinge"]
        self.assertEqual(scenario["hingeState"], "ready", scenario["hingeMessage"])
        quality = scenario["hingeQuality"]
        self.assertLess(quality["meanSpreadDeg"], 1.0)
        self.assertIn("hinge_complete", [e["kind"] for e in scenario["events"]])

    def test_signed_flexion_recovers_the_true_angle(self):
        frames = self.frames("hinge")
        for degrees in (45.0, 90.0, 130.0, -25.0):
            with self.subTest(degrees=degrees):
                frame = frames[f"flex_{int(degrees)}"]["frame"]
                self.assertAlmostEqual(frame["elbow_flexion_deg"], degrees, delta=1e-3)
                # A pure hinge rotation has nothing left over.
                self.assertLess(frame["elbow_off_axis_deg"], 1e-3)

    def test_signed_flexion_distinguishes_direction(self):
        """The unsigned angle cannot tell -25 from +25; the signed one must."""
        frames = self.frames("hinge")
        negative = frames["flex_-25"]["frame"]
        self.assertLess(negative["elbow_flexion_deg"], 0)
        self.assertAlmostEqual(negative["elbow_relative_rotation_deg"], 25.0, delta=1e-3)

    def test_off_axis_motion_is_reported_separately(self):
        """Compound motion must not be laundered into the flexion number."""
        frame = self.frames("hinge")["off_axis_25"]["frame"]
        self.assertLess(abs(frame["elbow_flexion_deg"]), 1.0)
        self.assertAlmostEqual(frame["elbow_off_axis_deg"], 25.0, delta=1e-3)
        self.assertTrue(frame["diagnostics"]["flexionValid"])

    def test_incoherent_motion_yields_no_hinge_axis(self):
        scenario = self.results["hinge_incoherent"]
        self.assertEqual(scenario["hingeState"], "failed")
        frame = self.frames("hinge_incoherent")["after_incoherent"]["frame"]
        self.assertIsNone(frame["elbow_flexion_deg"])
        # The unsigned angle still works; only the signed one is withheld.
        self.assertIsNotNone(frame["elbow_relative_rotation_deg"])

    def test_flexion_is_unavailable_before_a_hinge_calibration(self):
        frame = self.frames("elbow_sweep")["elbow_90"]["frame"]
        self.assertIsNone(frame["elbow_flexion_deg"])
        self.assertIsNone(frame["elbow_off_axis_deg"])
        self.assertEqual(frame["diagnostics"]["hingeState"], "none")

    def test_flexion_is_withheld_when_motion_leaves_the_hinge(self):
        frame = self.frames("gate_and_drift")["far_off_axis"]["frame"]
        self.assertIsNone(frame["elbow_flexion_deg"])
        self.assertFalse(frame["diagnostics"]["flexionValid"])
        # The unsigned angle and the off-axis term are still reported.
        self.assertIsNotNone(frame["elbow_relative_rotation_deg"])
        self.assertGreater(frame["elbow_off_axis_deg"], 35.0)

    def test_implausible_flexion_is_withheld(self):
        frame = self.frames("gate_and_drift")["implausible_flexion"]["frame"]
        self.assertIsNone(frame["elbow_flexion_deg"])
        self.assertFalse(frame["diagnostics"]["flexionValid"])

    def test_rest_without_drift_reports_no_drift(self):
        frame = self.frames("gate_and_drift")["rest_no_drift"]["frame"]
        self.assertTrue(frame["diagnostics"]["restObserved"])
        self.assertLess(frame["diagnostics"]["driftDeg"], 1e-3)
        self.assertFalse(frame["diagnostics"]["recalibrationRecommended"])

    def test_drift_is_measured_from_a_rest_observation(self):
        frame = self.frames("gate_and_drift")["rest_with_drift"]["frame"]
        self.assertAlmostEqual(frame["diagnostics"]["driftDeg"], 12.0, delta=0.5)

    def test_rezero_clears_the_drift(self):
        frame = self.frames("gate_and_drift")["after_rezero"]["frame"]
        self.assertLess(frame["diagnostics"]["driftDeg"], 1e-3)
        self.assertAlmostEqual(frame["elbow_flexion_deg"], 0.0, delta=0.5)
        kinds = [e["kind"] for e in self.results["gate_and_drift"]["events"]]
        self.assertIn("drift_rezeroed", kinds)

    def test_off_axis_excess_removes_the_drift_estimate(self):
        """The gate must threshold on excess, or drift would gate normal reps."""
        frame = self.frames("gate_and_drift")["rest_with_drift"]["frame"]
        raw = frame["elbow_off_axis_deg"]
        excess = frame["elbow_off_axis_excess_deg"]
        self.assertLess(excess, raw)

    def test_log_row_matches_the_declared_columns(self):
        entry = self.frames("elbow_sweep")["elbow_90"]
        self.assertEqual(len(entry["logRow"]), len(self.raw["logColumns"]))
        columns = self.raw["logColumns"]
        self.assertAlmostEqual(
            entry["logRow"][columns.index("elbow_deg")], 90.0, delta=1e-4
        )


if __name__ == "__main__":
    unittest.main()
