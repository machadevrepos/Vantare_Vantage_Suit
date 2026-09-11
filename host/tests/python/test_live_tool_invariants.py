"""Invariants for the live inference tool (host/live_tool).

Source-level contracts are checked directly; the node fixture suite
(host/tests/scripts/test_live_preprocessing.mjs) runs when node is available.
"""
import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / "host" / "live_tool"
BLE = (TOOL / "js" / "ble-protocol.js").read_text(encoding="utf-8")
PREPROC = (TOOL / "js" / "ml-preprocessing.js").read_text(encoding="utf-8")
INFERENCE = (TOOL / "js" / "live-inference.js").read_text(encoding="utf-8")
HAPTIC = (TOOL / "js" / "haptic-controller.js").read_text(encoding="utf-8")
MAIN = (TOOL / "js" / "main.js").read_text(encoding="utf-8")
UI = (TOOL / "js" / "ui.js").read_text(encoding="utf-8")
INDEX = (TOOL / "index.html").read_text(encoding="utf-8")
CONVERTER = (ROOT / "host" / "desktop_tool" / "vantage_bin_to_csv.py").read_text(encoding="utf-8")


class LiveToolInvariants(unittest.TestCase):
    def test_icm_scale_factors_match_training_converter(self):
        """The live decoder must scale raw ICM int16 exactly like the converter
        that produced the training CSVs (accel g, gyro dps)."""
        self.assertIn("4.0 / 32768.0", CONVERTER)
        self.assertIn("2000.0 / 32768.0", CONVERTER)
        self.assertIn("4.0 / 32768.0", BLE)
        self.assertIn("2000.0 / 32768.0", BLE)

    def test_index_loads_ort_before_the_app_module(self):
        """live-inference.js reads window.ort, so the vendored UMD bundle must
        be script-tagged ahead of the module entry point. Dropping the tag
        leaves the page loading with MODEL NOT READY and no inference."""
        index = (ROOT / "host" / "live_tool" / "index.html").read_text(encoding="utf-8")
        ort_at = index.find("vendor/ort/ort.min.js")
        app_at = index.find("./js/main.js")
        self.assertNotEqual(ort_at, -1, "vendored ort.min.js script tag is missing")
        self.assertNotEqual(app_at, -1, "app module script tag is missing")
        self.assertLess(ort_at, app_at, "ort.min.js must load before js/main.js")
        self.assertNotIn("cdn.jsdelivr", index)
        self.assertNotIn("unpkg.com", index)

    def test_master_source_is_charted_not_dropped(self):
        """The Master streams its own BNO/ICM as source 0. Filtering the decode
        to NODE_IDS made its graph permanently dead; display sources must be
        decoded, and must never gate inference."""
        self.assertIn("DISPLAY_SOURCE_IDS", BLE)
        self.assertIn("DISPLAY_SOURCE_IDS.includes(parsed.nodeId)", BLE)
        self.assertIn("isModelStream", BLE)
        # Health gates iterate the model streams only.
        self.assertIn("modelStreamKeys()", MAIN)
        self.assertNotIn("for (const [key, health] of this.transport.health)", MAIN)

    def test_gatt_uuids_match_master_firmware(self):
        """The Master exposes one service (3f881000) with every characteristic
        inside it. Resolving 3f882000/3f883000 as services fails with
        "No Services matching UUID" and silently drops status notifications."""
        custom_stm = (
            ROOT / "firmware" / "Master" / "Core" / "Src" / "ble" / "custom_stm.cpp"
        ).read_text(encoding="utf-8")
        for name in (
            "BLEPIPESERVICE",
            "PIPEDATATX",
            "PIPECONTROLRX",
            "PIPECONTROLTX",
            "PIPESTATUSTX",
        ):
            self.assertIn(name, custom_stm, f"firmware no longer defines {name}")
        # The vestigial service entries must not come back as config values.
        self.assertNotIn("statusServiceUuid", BLE)
        self.assertNotIn("controlServiceUuid", BLE)
        # Exactly one service is resolved, and status comes from inside it.
        self.assertEqual(
            BLE.count("getPrimaryService"), 1, "only the pipe service may be resolved"
        )
        self.assertIn("service.getCharacteristic(BLE_CFG.statusCharUuid)", BLE)

    def test_grid_sampling_is_nearest_not_interpolation(self):
        """Training features come from decimate_stream_to_grid, so the browser
        must select real samples. Linear interpolation between samples 40 ms
        apart lowers std/range/rms/mean_abs_diff (design Section 12)."""
        self.assertIn("nearestIndex", PREPROC)
        self.assertIn("decimateStream", PREPROC)
        self.assertNotIn("interpolateStream", PREPROC)
        self.assertNotIn("lerp(", PREPROC)

    def test_notebook_pipeline_matches_repository_module(self):
        """The notebook exports host/live_tool/pipeline/vantare_live_pipeline.py
        verbatim; drift between them is what produced the interpolation bug."""
        import json

        notebook = json.loads(
            (ROOT / "host" / "notebooks" / "Vantare_Bicep_Curl_Training_ONNX_v1_1.ipynb")
            .read_text(encoding="utf-8")
        )
        sources = [
            "".join(cell["source"])
            for cell in notebook["cells"]
            if cell["cell_type"] == "code" and "PIPELINE_SOURCE" in "".join(cell["source"])
        ]
        self.assertEqual(len(sources), 1, "expected exactly one PIPELINE_SOURCE cell")
        namespace: dict = {}
        exec(sources[0].split("runtime_module_path")[0], namespace)
        module = (
            ROOT / "host" / "live_tool" / "pipeline" / "vantare_live_pipeline.py"
        ).read_text(encoding="utf-8")
        self.assertEqual(namespace["PIPELINE_SOURCE"], module)

    def test_pulse_is_bounded_by_the_node(self):
        """Section 6.4: the Node owns the stop deadline. The browser sends a
        duration and an event id and must not rely on its own timer."""
        self.assertIn("HAPTIC_PULSE: 0xa7", BLE)
        self.assertIn("hapticPulse", BLE)
        self.assertIn("transport.hapticPulse", HAPTIC)
        header = (
            ROOT / "firmware" / "common" / "inc" / "exo" / "actuator" / "node_haptic_pulse.h"
        ).read_text(encoding="utf-8")
        self.assertIn("kMinDurationMs = 50U", header)
        self.assertIn("kMaxDurationMs = 500U", header)
        node_main = (ROOT / "firmware" / "Node" / "Core" / "Src" / "main.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("g_node_haptic_pulse.service(HAL_GetTick())", node_main)
        self.assertIn("case 0xA7U:", node_main)

    def test_contract_gate_requires_qualified_live_rate(self):
        """A contract whose rate differs from the qualified live rate must be
        refused, never resampled (design Sections 3, 11)."""
        self.assertIn("APP_TARGET_HZ = 25", INFERENCE)
        self.assertIn("Retrain at the live contract rate", INFERENCE)
        self.assertIn("never resampled", INFERENCE)

    def test_health_gates_follow_section_10_table(self):
        """Staleness 4T (arrival, absorbs Chrome notification coalescing),
        interpolation span 2.5T soft (12% budget) / 4T hard, window-loss
        deficit 2%, skew 0.5T."""
        self.assertIn("staleLimitMs = 4 * periodMs", MAIN)
        self.assertIn("skewLimitMs = 0.5 * periodMs", MAIN)
        self.assertIn("maxInterpPeriods = 2.5", PREPROC)
        self.assertIn("hardInterpPeriods = 4", PREPROC)
        self.assertIn("maxInterpViolationFraction = 0.12", PREPROC)
        self.assertIn("maxMissingFraction = 0.02", PREPROC)

    def test_haptic_rules_follow_section_9(self):
        """0.70 threshold, 2 consecutive windows, 2 s cooldown, bounded pulse,
        class 1 -> N2, class 2 -> N3, ring-down blanking margin."""
        self.assertIn("probabilityThreshold: 0.7", HAPTIC)
        self.assertIn("requiredConsecutive: 2", HAPTIC)
        self.assertIn("cooldownSeconds: 2.0", HAPTIC)
        self.assertIn("intensityPercent: 50", HAPTIC)
        self.assertIn("durationMs: 250", HAPTIC)
        self.assertIn("1: 2, // incomplete_range -> N2 wrist", HAPTIC)
        self.assertIn("2: 3, // elbow_movement -> N3 elbow", HAPTIC)
        self.assertIn("ringDownMarginMs: 250", HAPTIC)

    def test_haptics_fail_silent(self):
        """Disarm must stop any motor with a pending off timer (Section 15)."""
        self.assertIn("pendingOffByNode", HAPTIC)
        self.assertIn("motorPercent(nodeId, 0)", HAPTIC)

    def test_ort_pinned_and_local(self):
        """Pinned runtime version, local vendor path, single-threaded WASM."""
        self.assertIn('ORT_VERSION_PINNED = "1.20.1"', INFERENCE)
        # wasmPaths must be an absolute URL derived from the module location:
        # ORT 1.20 resolves the dynamic .mjs loader import relative to
        # ort.min.js, so a relative prefix doubles the path.
        self.assertIn('new URL("../vendor/ort/", import.meta.url).href', INFERENCE)
        self.assertIn("numThreads = 1", INFERENCE)
        vendor = TOOL / "vendor" / "ort"
        for name in ("ort.min.js", "ort-wasm-simd-threaded.wasm", "ort-wasm-simd-threaded.mjs"):
            self.assertTrue((vendor / name).is_file(), f"missing vendored file: {name}")
        for name, source in [("live-inference", INFERENCE), ("main", MAIN), ("index", (TOOL / "index.html").read_text(encoding="utf-8"))]:
            self.assertNotIn("cdn.jsdelivr.net", source, f"{name} must not reference a CDN")

    def test_stream_start_independent_of_recording(self):
        """The live tool drives the stream-control commands that the firmware
        implements independently of SD recording (0xA0/0xA1/0xA2)."""
        for byte in ("0xa0", "0xa1", "0xa2", "0xa3"):
            self.assertIn(byte, BLE)
        self.assertIn("LIVE_INTERVAL_MS = 40", MAIN)

    def test_contract_gate_accepts_installed_artifacts(self):
        """The installed live_tool model artifacts must pass the contract gate
        (regression: the notebook emits raw_synchronized_channels/feature_count,
        the curated V1 contract used synchronized_channels/features)."""
        model_dir = TOOL / "model"
        if not (model_dir / "model_contract.json").is_file():
            self.skipTest("no model artifacts installed")
        if shutil.which("node") is None:
            self.skipTest("node not available")
        script = (
            "import { validateContract } from './js/live-inference.js';"
            "import { buildChannelNames } from './js/ml-preprocessing.js';"
            "import { readFileSync } from 'node:fs';"
            "const contract = JSON.parse(readFileSync('./model/model_contract.json', 'utf-8'));"
            "const featureNames = JSON.parse(readFileSync('./model/feature_names.json', 'utf-8'));"
            "const r = validateContract(contract, featureNames, buildChannelNames());"
            "console.log(JSON.stringify(r));"
        )
        result = subprocess.run(
            ["node", "--input-type=module", "-e", script],
            capture_output=True,
            text=True,
            cwd=TOOL,
            timeout=120,
        )
        self.assertEqual(result.returncode, 0, f"contract gate failed:\n{result.stdout}\n{result.stderr}")

    def test_node_fixture_suite_passes(self):
        """Runs the analytic preprocessing fixtures with node when available."""
        if shutil.which("node") is None:
            self.skipTest("node not available")
        script = ROOT / "host" / "tests" / "scripts" / "test_live_preprocessing.mjs"
        result = subprocess.run(
            ["node", str(script)],
            capture_output=True,
            text=True,
            cwd=ROOT,
            timeout=120,
        )
        self.assertEqual(
            result.returncode, 0,
            f"fixture suite failed:\n{result.stdout}\n{result.stderr}",
        )
        self.assertRegex(result.stdout, r"All \d+ preprocessing fixture tests passed")


    def test_anatomical_workflow_controls_are_ordered_and_bound(self):
        """The three-pose workflow must read as numbered steps in DOM order:
        neutral, side raise, forward raise, elbow hinge. Each step binds to its
        MotionEngine method, the app pumps the directional capture, and the UI
        enables a step only after its prerequisite."""
        for button in ("calibrateBtn", "sideCalibrateBtn", "forwardCalibrateBtn", "hingeBtn"):
            self.assertIn(f'id="{button}"', INDEX)
        self.assertIn(">1. Calibrate Neutral</button>", INDEX)
        self.assertIn(">2. Capture Right-Side Raise</button>", INDEX)
        self.assertIn(">3. Capture Forward Raise</button>", INDEX)
        self.assertIn(">4. Calibrate Elbow Hinge</button>", INDEX)
        side_at = INDEX.find('id="sideCalibrateBtn"')
        forward_at = INDEX.find('id="forwardCalibrateBtn"')
        hinge_at = INDEX.find('id="hingeBtn"')
        self.assertLess(side_at, forward_at, "side capture must precede forward capture")
        self.assertLess(forward_at, hinge_at, "forward capture must precede the hinge")

        # App bindings: each button drives the matching MotionEngine entry point.
        self.assertIn(
            'this.ui.sideCalibrateBtn.addEventListener("click", () => this.motion.beginSideCalibration());',
            MAIN,
        )
        self.assertIn(
            'this.ui.forwardCalibrateBtn.addEventListener("click", () => this.motion.beginForwardCalibration());',
            MAIN,
        )
        self.assertIn("this.motion.updateAnatomicalCalibration(now);", MAIN)

        # Anatomical transitions stay audible in the NDJSON audit trail.
        self.assertIn('event.kind === "anatomical_complete"', MAIN)
        self.assertIn('event.kind === "anatomical_failed"', MAIN)

        # UI renders the ordered enablement and the honest axis frame. The
        # hinge needs only the neutral pose - its math is independent of the
        # anatomical solve, so gating it behind step 3 was pure friction
        # (review finding, 2026-09-11).
        self.assertIn('anatomicalState === "side_ready"', UI)
        self.assertIn(
            'this.setEnabled(this.hingeBtn, diagnostics.calibrationState === "calibrated");',
            UI,
        )
        self.assertIn("axisFrame", UI)
        self.assertIn("anatomicalMessage", UI)

    def test_avatar_fast_path_is_anatomically_gated(self):
        """The display fast path consumes displayPose(), which refuses
        sensor-neutral axes; the call must be gated so an uncorrected
        quaternion never drives the directional rig, and it must run for both
        instrumented segments on every arrival so prediction has fresh gyro
        rates (the old N4-only shoulder path left the forearm 40 ms behind)."""
        self.assertIn('this.motion.anatomicalState === "calibrated"', MAIN)
        self.assertIn("this.motion.displayPose(arrivalMs)", MAIN)
        self.assertIn("this.armAvatar.renderPose(pose, arrivalMs);", MAIN)
        self.assertIn("sample.nodeId === this.motion.roles.forearm", MAIN)

    def test_avatar_note_requires_the_three_pose_workflow(self):
        """Directional tracking claims must be gated on the workflow in the
        panel the user actually reads, not just in code comments."""
        self.assertIn("three-pose", INDEX)

    def test_live_tool_build_bumped_for_anatomical_workflow(self):
        """A stale cached module graph silently runs the old two-pose UI; the
        visible build string must move with this workflow change."""
        self.assertIn('LIVE_TOOL_BUILD = "2026-09-11.25"', INFERENCE)

    # --------------------------------------------------- anatomical replay

    @staticmethod
    def _replay_anatomical(builder):
        """Run a fixture scenario through the real MotionEngine, serialize the
        result as a session NDJSON log, and replay it through the avatar
        replay CLI. Returns the CLI's JSON metrics."""
        sys.path.insert(0, str(ROOT / "host" / "tests" / "python"))
        try:
            from test_motion_engine import MotionEngineTest  # noqa: E402
        finally:
            sys.path.pop(0)

        with tempfile.TemporaryDirectory() as tmp:
            scenario_path = Path(tmp) / "scenario.json"
            scenario_path.write_text(
                json.dumps({"scenarios": [builder.build()]}), encoding="utf-8"
            )
            runner = ROOT / "host" / "tests" / "scripts" / "run_motion_fixture.mjs"
            completed = subprocess.run(
                ["node", str(runner), str(scenario_path)],
                check=True, capture_output=True, text=True, timeout=120,
            )
            result = json.loads(completed.stdout)["scenarios"][0]

            lines = [json.dumps({"type": "meta", "recordedAt": "fixture", "generator": "test"})]
            for event in result["events"]:
                lines.append(json.dumps(
                    {"type": "event", "tMs": 0, **event, "kind": f"motion_{event['kind']}"}
                ))
            frames = result["frames"]
            lines.append(json.dumps({
                "type": "stream_decl", "stream": "motion",
                "width": len(frames[0]["logRow"]), "capacity": len(frames),
            }))
            for frame in frames:
                lines.append(json.dumps({"type": "sample", "stream": "motion", "data": frame["logRow"]}))
            ndjson_path = Path(tmp) / "session.ndjson"
            ndjson_path.write_text("\n".join(lines), encoding="utf-8")

            replay = ROOT / "host" / "tests" / "scripts" / "replay_arm_avatar.mjs"
            completed = subprocess.run(
                ["node", str(replay), str(ndjson_path)],
                check=True, capture_output=True, text=True, timeout=120,
            )
            return json.loads(completed.stdout)

    def test_anatomical_avatar_replay_reports_direction_and_latency(self):
        """The replay CLI must audit a session through the production avatar
        path: anatomical frames drive the rig, calibrated direction errors are
        small for a correct three-pose capture, and display smoothing adds
        well under two motion ticks of latency."""
        if shutil.which("node") is None:
            self.skipTest("node not available")
        sys.path.insert(0, str(ROOT / "host" / "tests" / "python"))
        try:
            import test_motion_engine as motion_fixtures
        finally:
            sys.path.pop(0)
        MotionEngineTest = motion_fixtures.MotionEngineTest
        builder = MotionEngineTest.calibrated_builder("replay_anatomical")
        MotionEngineTest.capture_anatomical(builder)

        # Post-calibration validation segment, mirroring the acceptance run:
        # repeat the two calibrated directions, then move freely.
        def hold_frames(pose, ticks):
            for _ in range(ticks):
                builder.push_pose(pose)
                builder.auto_frame()

        hold_frames(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0), 10)
        hold_frames(MotionEngineTest.rigid_arm_pose((1.0, 0.0, 0.0), 90.0), 10)
        combined = MotionEngineTest.rigid_arm_pose((0.31, -0.52, 0.79), 73.0)
        hold_frames(combined, 10)
        builder.frame("combined")
        # Dropout: only N4 keeps reporting, so the pair reads unsynchronized.
        hold_frames({motion_fixtures.UPPER_ARM: combined[motion_fixtures.UPPER_ARM]}, 5)
        builder.frame("dropout")
        hold_frames(combined, 10)
        builder.frame("recovery")

        metrics = self._replay_anatomical(builder)

        self.assertEqual(metrics["axisFrame"], "anatomical")
        self.assertEqual(metrics["directionValidation"], "available")
        self.assertLessEqual(metrics["sideDirectionErrorDeg"], 10.0)
        self.assertLessEqual(metrics["forwardDirectionErrorDeg"], 10.0)
        self.assertLessEqual(metrics["medianAddedLatencyMs"], 40.0)
        self.assertGreater(metrics["replayedSamples"], 40)
        self.assertGreater(metrics["dropoutFrames"], 0, "dropout segment must be visible")
        self.assertIn("p50", metrics["visualStepDistribution"])

    def test_anatomical_replay_keeps_rejected_geometry_unavailable(self):
        """Rejected geometry must never replay as an anatomical calibration."""
        if shutil.which("node") is None:
            self.skipTest("node not available")
        sys.path.insert(0, str(ROOT / "host" / "tests" / "python"))
        try:
            import test_motion_engine as motion_fixtures
        finally:
            sys.path.pop(0)
        MotionEngineTest = motion_fixtures.MotionEngineTest
        builder = MotionEngineTest.calibrated_builder("replay_warning")
        off_axis = (math.cos(math.radians(20)), 0.0, -math.sin(math.radians(20)))
        builder.begin_side()
        builder.hold(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0), 1.2)
        builder.begin_forward()
        builder.hold(MotionEngineTest.rigid_arm_pose(off_axis, 90.0), 1.2)
        builder.hold(MotionEngineTest.rigid_arm_pose((0.0, 0.0, -1.0), 90.0), 0.3)
        builder.frame("after")

        metrics = self._replay_anatomical(builder)

        self.assertEqual(metrics["axisFrame"], "sensor_neutral")
        self.assertEqual(metrics["directionValidation"], "unavailable")
        self.assertNotIn("calibrationWarning", metrics)

    def test_replay_reports_legacy_sensor_neutral_logs_honestly(self):
        """A log from before the three-pose workflow has no directional
        calibration events; the CLI must report sensor_neutral and decline
        direction validation instead of inventing numbers."""
        if shutil.which("node") is None:
            self.skipTest("node not available")
        sys.path.insert(0, str(ROOT / "host" / "tests" / "python"))
        try:
            import test_motion_engine as motion_fixtures
        finally:
            sys.path.pop(0)
        builder = motion_fixtures.MotionEngineTest.calibrated_builder("replay_legacy")
        combined = motion_fixtures.MotionEngineTest.rigid_arm_pose((0.31, -0.52, 0.79), 73.0)
        for _ in range(15):
            builder.push_pose(combined)
            builder.auto_frame()
        builder.frame("pose")

        metrics = self._replay_anatomical(builder)

        self.assertEqual(metrics["axisFrame"], "sensor_neutral")
        self.assertEqual(metrics["directionValidation"], "unavailable")


if __name__ == "__main__":
    unittest.main()
