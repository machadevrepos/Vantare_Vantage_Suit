"""Invariants for the live inference tool (host/live_tool).

Source-level contracts are checked directly; the node fixture suite
(host/tests/scripts/test_live_preprocessing.mjs) runs when node is available.
"""
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / "host" / "live_tool"
BLE = (TOOL / "js" / "ble-protocol.js").read_text(encoding="utf-8")
PREPROC = (TOOL / "js" / "ml-preprocessing.js").read_text(encoding="utf-8")
INFERENCE = (TOOL / "js" / "live-inference.js").read_text(encoding="utf-8")
HAPTIC = (TOOL / "js" / "haptic-controller.js").read_text(encoding="utf-8")
MAIN = (TOOL / "js" / "main.js").read_text(encoding="utf-8")
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
        """Staleness 3T, interpolation span 1.5T, window loss 2%, skew 0.5T."""
        self.assertIn("staleLimitMs = 3 * periodMs", MAIN)
        self.assertIn("skewLimitMs = 0.5 * periodMs", MAIN)
        self.assertIn("maxInterpPeriods = 1.5", PREPROC)
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


if __name__ == "__main__":
    unittest.main()
