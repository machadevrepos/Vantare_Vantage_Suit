import json
import shutil
import subprocess
import unittest
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
HTML_PATH = ROOT / "host" / "desktop_tool" / "Exoskeleton.html"
CONTRACT_SCRIPT_ID = "exoskeleton-telemetry-contract"


class _ContractScriptParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self._capturing = False
        self.parts = []

    def handle_starttag(self, tag, attrs):
        if tag == "script" and dict(attrs).get("id") == CONTRACT_SCRIPT_ID:
            self._capturing = True

    def handle_endtag(self, tag):
        if tag == "script" and self._capturing:
            self._capturing = False

    def handle_data(self, data):
        if self._capturing:
            self.parts.append(data)


class _SelectOptionsParser(HTMLParser):
    def __init__(self, select_id):
        super().__init__()
        self.select_id = select_id
        self.in_select = False
        self.current_value = None
        self.current_text = []
        self.options = []

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if tag == "select" and attributes.get("id") == self.select_id:
            self.in_select = True
        elif tag == "option" and self.in_select:
            self.current_value = attributes.get("value")
            self.current_text = []

    def handle_endtag(self, tag):
        if tag == "option" and self.in_select and self.current_value is not None:
            self.options.append((self.current_value, "".join(self.current_text).strip()))
            self.current_value = None
            self.current_text = []
        elif tag == "select" and self.in_select:
            self.in_select = False

    def handle_data(self, data):
        if self.current_value is not None:
            self.current_text.append(data)


def _contract_script():
    parser = _ContractScriptParser()
    parser.feed(HTML_PATH.read_text(encoding="utf-8-sig"))
    script = "".join(parser.parts)
    if not script.strip():
        raise AssertionError(f"missing <script id={CONTRACT_SCRIPT_ID!r}>")
    return script


def _run_contract(expression):
    node = shutil.which("node")
    if not node:
        raise AssertionError("Node.js is required to execute the browser telemetry contract")
    program = _contract_script() + f"\nconsole.log(JSON.stringify({expression}));\n"
    result = subprocess.run(
        [node, "-"], input=program, text=True, capture_output=True, check=False
    )
    if result.returncode != 0:
        raise AssertionError(f"browser telemetry contract failed:\n{result.stderr}")
    return json.loads(result.stdout)


class ExoskeletonTelemetryContractTests(unittest.TestCase):
    def test_node_link_stats_preserve_all_five_evidence_states(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const payload = new Uint8Array(49);
              const view = new DataView(payload.buffer);
              payload[0] = 1;
              const render = (outcome, requestStatus, attempts, tx, rx) => {
                payload[1] = outcome;
                payload[2] = requestStatus;
                payload[4] = attempts;
                view.setUint16(7, tx, true);
                view.setUint16(9, rx, true);
                return api.formatNodeUploadLinkStats(api.decodeNodeUploadLinkStats(payload, 2));
              };
              return {
                labels: [0, 1, 2, 3, 4].map(api.dleOutcomeLabel),
                unknown: render(0, 0xff, 0, 0, 0),
                requested: render(1, 0x00, 1, 0, 0),
                confirmed: render(2, 0x00, 1, 247, 247),
                degraded: render(3, 0x3a, 2, 0, 0),
                failed: render(4, 0x12, 3, 0, 0)
              };
            })()"""
        )

        self.assertEqual(
            result["labels"], ["unknown", "requested", "confirmed", "degraded", "failed"]
        )
        self.assertIn("DLE unknown", result["unknown"])
        self.assertIn("tx=unknown", result["unknown"])
        self.assertNotIn("27-byte default", result["unknown"])
        self.assertIn("DLE requested", result["requested"])
        self.assertIn("awaiting completion", result["requested"])
        self.assertIn("DLE confirmed", result["confirmed"])
        self.assertIn("tx=247", result["confirmed"])
        self.assertIn("rx=247", result["confirmed"])
        self.assertIn("DLE degraded", result["degraded"])
        self.assertIn("request_status=0x3a", result["degraded"])
        self.assertIn("DLE failed", result["failed"])
        self.assertIn("request_status=0x12", result["failed"])

    def test_master_link_events_distinguish_evidence_from_request_acceptance(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              return {
                unknownDle: api.formatMasterLinkEvent(0x10, 0, 0),
                requestedDle: api.formatMasterLinkEvent(0x13, 0, 0),
                unknownState: api.formatMasterLinkEvent(0x16, 0, 0x00ff),
                requestedState: api.formatMasterLinkEvent(0x16, 1, 0x0000),
                nextProcedureNotRequested: api.formatMasterLinkEvent(0x16, 2, 0x0000),
                confirmedState: api.formatMasterLinkEvent(0x16, 6, 0x0000),
                degradedState: api.formatMasterLinkEvent(0x16, 7, 0x02fe),
                failedState: api.formatMasterLinkEvent(0x16, 8, 0x0112)
              };
            })()"""
        )

        self.assertIn("DLE unknown", result["unknownDle"])
        self.assertIn("no completion evidence", result["unknownDle"])
        self.assertNotIn("27-byte default", result["unknownDle"])
        self.assertIn("DLE requested", result["requestedDle"])
        self.assertIn("command accepted", result["requestedDle"])
        self.assertIn("link unknown", result["unknownState"])
        self.assertIn("link requested", result["requestedState"])
        self.assertIn("link unknown", result["nextProcedureNotRequested"])
        self.assertIn("phase=need_phy", result["nextProcedureNotRequested"])
        self.assertIn("link confirmed", result["confirmedState"])
        self.assertIn("link degraded", result["degradedState"])
        self.assertIn("status=0xfe", result["degradedState"])
        self.assertIn("retries=2", result["degradedState"])
        self.assertIn("link failed", result["failedState"])
        self.assertIn("status=0x12", result["failedState"])

    def test_v2_transfer_report_displays_effective_flow_control_and_compact_relay_progress(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const payload = new Uint8Array(59);
              const view = new DataView(payload.buffer);
              payload[0] = 5;
              payload[1] = 0x1f;
              payload[2] = 0x03;
              payload[19] = 2;
              payload[20] = 24;
              payload[21] = 8;
              view.setUint16(22, 350, true);
              payload[24] = 3;
              payload[25] = 17;
              view.setUint32(26, 1000, true);
              view.setUint32(30, 0, true);
              view.setUint32(34, 13, true);
              view.setUint32(38, 12, true);
              view.setUint32(42, 1, true);
              payload[46] = 1;
              view.setUint32(47, 999, true);
              view.setUint32(51, 20, true);
              view.setUint32(55, 7, true);
              const status = api.decodeTrainingStatusPayload(payload);
              return { status, display: api.formatEffectiveTransfer(status) };
            })()"""
        )

        status = result["status"]
        self.assertEqual(status["flowVersion"], 2)
        self.assertEqual(status["effectiveCredit"], 24)
        self.assertEqual(status["ackChunkThreshold"], 8)
        self.assertEqual(status["ackTimeoutMs"], 350)
        self.assertEqual(status["pendingQueue"], 3)
        self.assertEqual(status["queueHighWater"], 17)
        self.assertEqual(status["suppressedRelay"], 999)
        display = result["display"]
        self.assertIn("effective credit=24", display)
        self.assertIn("ACK=8 chunks/350 ms", display)
        self.assertIn("queue pending=3 high-water=17 overflow=0", display)
        self.assertIn("ACK attempts=13 ok=12 failed=1 last=ok", display)
        self.assertIn("raw relay suppressed=999", display)
        self.assertIn("compact progress remains active", display)
        self.assertIn("SD flushes=20 max=7 ms", display)

    def test_b5_v2_selects_only_supported_fast_intervals_and_keeps_v1_default(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const legacy = new Uint8Array(12);
              legacy[0] = 0xb5;
              legacy[1] = 1;
              const base = {
                credit: 24,
                ackEveryChunks: 8,
                ackEveryMs: 750,
                controlHeartbeatMs: 900,
                nackBurstChunks: 8,
                masterBurstLimit: 6,
                masterChunkGapMs: 1,
                flags: 0
              };
              const encoded = [12, 9, 6, 7].map(fastIntervalUnits =>
                Array.from(api.buildTransferTuningPayload(0xb5, { ...base, fastIntervalUnits })));
              return {
                legacy: api.decodeTransferTuningPayload(legacy),
                decoded: encoded.map(bytes =>
                  api.decodeTransferTuningPayload(Uint8Array.from(bytes))),
                encoded
              };
            })()"""
        )

        self.assertEqual(result["legacy"]["version"], 1)
        self.assertEqual(result["legacy"]["fastIntervalUnits"], 12)
        self.assertEqual(result["legacy"]["fastIntervalMs"], 15)
        self.assertEqual([item["fastIntervalUnits"] for item in result["decoded"]], [12, 9, 6, 12])
        for payload in result["encoded"]:
            self.assertEqual(len(payload), 13)
            self.assertEqual(payload[:2], [0xB5, 2])
        self.assertEqual([payload[12] for payload in result["encoded"]], [12, 9, 6, 12])

    def test_fast_interval_selector_has_exact_executable_matrix_choices(self):
        parser = _SelectOptionsParser("transferFastIntervalSelect")
        parser.feed(HTML_PATH.read_text(encoding="utf-8-sig"))

        self.assertEqual(
            parser.options,
            [("12", "15 ms"), ("9", "11.25 ms"), ("6", "7.5 ms")],
        )

    def test_configured_request_and_disc_completion_use_separate_ui_outputs(self):
        html = HTML_PATH.read_text(encoding="utf-8-sig")

        self.assertIn('id="effectiveTransferTelemetry"', html)
        self.assertIn('id="confirmedIntervalTelemetry"', html)
        self.assertIn("Effective Transfer / Configured Request", html)
        self.assertIn("DISC Confirmed Interval", html)
        self.assertIn("renderConfirmedIntervalTelemetry(nodeId, extra)", html)

    def test_master_raw_relay_debug_never_invokes_browser_control_path(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const rawMode = api.artifactRelayMode(true, 0x02);
              const compactMode = api.artifactRelayMode(true, 0x00);
              const legacyMode = api.artifactRelayMode(false, 0x00);
              const rawObserved = [];
              const rawControl = [];
              const compactControl = [];
              const legacyControl = [];
              const emitBrowserControls = sink => frame => {
                if (frame.kind === "manifest") sink.push("Manifest ACK");
                if (frame.kind === "chunk") sink.push("ACK", "NACK");
                if (frame.kind === "complete") sink.push("Verify");
              };
              for (const kind of ["manifest", "chunk", "complete"]) {
                api.dispatchArtifactRelay(rawMode, { kind }, {
                  onReadOnly: frame => rawObserved.push(frame.kind),
                  onBrowserControl: emitBrowserControls(rawControl)
                });
                api.dispatchArtifactRelay(compactMode, { kind }, {
                  onBrowserControl: emitBrowserControls(compactControl)
                });
                api.dispatchArtifactRelay(legacyMode, { kind }, {
                  onBrowserControl: emitBrowserControls(legacyControl)
                });
              }
              return {
                rawMode,
                compactMode,
                legacyMode,
                rawObserved,
                rawControl,
                compactControl,
                legacyControl
              };
            })()"""
        )

        self.assertEqual(result["rawMode"], "master_raw_read_only")
        self.assertEqual(result["compactMode"], "compact_progress")
        self.assertEqual(result["legacyMode"], "browser_manual")
        self.assertEqual(result["rawObserved"], ["manifest", "chunk", "complete"])
        self.assertEqual(result["rawControl"], [])
        self.assertEqual(result["compactControl"], [])
        self.assertEqual(
            result["legacyControl"], ["Manifest ACK", "ACK", "NACK", "Verify"]
        )

    def test_master_owned_node_artifacts_have_defense_in_depth_control_guards(self):
        html = HTML_PATH.read_text(encoding="utf-8-sig")

        for function_name in ("queueChunkAck", "queueReliableControl"):
            start = html.index(f"function {function_name}(")
            end = html.index("\n  function ", start + 1)
            body = html[start:end]
            self.assertIn(
                "if (masterStagedSource(transfer))",
                body,
                f"{function_name} must reject control for Master-owned Node artifacts",
            )

        self.assertGreaterEqual(
            html.count("ExoskeletonTelemetry.dispatchArtifactRelay("),
            3,
            "reliable, legacy, and V3 artifact paths must all use the ownership dispatcher",
        )

    def test_zero_ack_attempts_render_last_status_as_not_attempted(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const payload = new Uint8Array(59);
              payload[0] = 5;
              payload[19] = 2;
              payload[20] = 24;
              payload[21] = 8;
              const status = api.decodeTrainingStatusPayload(payload);
              return api.formatEffectiveTransfer(status);
            })()"""
        )

        self.assertIn("ACK attempts=0 ok=0 failed=0 last=unknown (not attempted)", result)
        self.assertNotIn("last=failed", result)

    def test_b6_v3_decodes_configured_interval_and_retransmission_ratio(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const payload = new Uint8Array(69);
              const view = new DataView(payload.buffer);
              payload[0] = 5;
              payload[19] = 3;
              payload[20] = 24;
              payload[21] = 8;
              payload[59] = 6;
              payload[60] = 4;
              view.setUint32(61, 20000, true);
              view.setUint32(65, 19, true);
              const status = api.decodeTrainingStatusPayload(payload);
              const zeroPayload = payload.slice();
              new DataView(zeroPayload.buffer).setUint32(61, 0, true);
              new DataView(zeroPayload.buffer).setUint32(65, 0, true);
              const zero = api.decodeTrainingStatusPayload(zeroPayload);
              const truncatedPayload = payload.slice(0, 65);
              const truncated = api.decodeTrainingStatusPayload(truncatedPayload);
              return {
                status,
                display: api.formatEffectiveTransfer(status),
                zero,
                zeroDisplay: api.formatEffectiveTransfer(zero),
                truncated,
                truncatedDisplay: api.formatEffectiveTransfer(truncated),
                confirmed: api.formatMasterLinkEvent(0x12, 0, 6)
              };
            })()"""
        )

        status = result["status"]
        self.assertEqual(status["flowVersion"], 3)
        self.assertEqual(status["configuredFastIntervalUnits"], 6)
        self.assertEqual(status["configuredFastIntervalMs"], 7.5)
        self.assertEqual(status["counterNodeId"], 4)
        self.assertEqual(status["uniqueAcceptedChunks"], 20000)
        self.assertEqual(status["retransmittedFrames"], 19)
        self.assertAlmostEqual(status["retransmissionRatioPct"], 0.095)
        self.assertIn("configured/requested fast CI=7.5 ms (units=6; not confirmation)", result["display"])
        self.assertIn("NODE4 chunks unique=20000 retransmitted_frames=19 ratio=0.095%", result["display"])
        self.assertIsNone(result["zero"]["retransmissionRatioPct"])
        self.assertIn("ratio=unknown (unique=0)", result["zeroDisplay"])
        self.assertNotIn("Infinity", result["zeroDisplay"])
        self.assertNotIn("NaN", result["zeroDisplay"])
        self.assertFalse(result["truncated"]["flowV3Available"])
        self.assertIn("requires complete 0xB6 v3", result["truncatedDisplay"])
        self.assertNotIn("undefined", result["truncatedDisplay"])
        self.assertIn("interval confirmed units=6 (7.50 ms)", result["confirmed"])
        self.assertNotIn("confirmed", result["display"])

    def test_legacy_status_and_raw_debug_mode_remain_wire_compatible(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              const legacy = new Uint8Array(19);
              legacy[0] = 5;
              legacy[1] = 0x1f;
              legacy[2] = 0x03;
              const decoded = api.decodeTrainingStatusPayload(legacy);
              return {
                flowVersion: decoded.flowVersion,
                display: api.formatEffectiveTransfer(decoded),
                rawOff: api.rawRelayDebugEnabled(0),
                rawOn: api.rawRelayDebugEnabled(0x02),
                rawOnWithOtherFlags: api.rawRelayDebugEnabled(0x82)
              };
            })()"""
        )

        self.assertIsNone(result["flowVersion"])
        self.assertIn("legacy 0xB6 status", result["display"])
        self.assertFalse(result["rawOff"])
        self.assertTrue(result["rawOn"])
        self.assertTrue(result["rawOnWithOtherFlags"])

    def test_requested_ack_threshold_is_kept_below_credit(self):
        result = _run_contract(
            """(() => {
              const api = globalThis.ExoskeletonTelemetry;
              return {
                matrix: api.normalizeFlowControl(8, 4),
                clamped: api.normalizeFlowControl(24, 48),
                defaults: api.normalizeFlowControl(0, 0)
              };
            })()"""
        )

        self.assertEqual(result["matrix"], {"credit": 8, "ackChunkThreshold": 4})
        self.assertEqual(result["clamped"], {"credit": 24, "ackChunkThreshold": 23})
        self.assertEqual(result["defaults"], {"credit": 24, "ackChunkThreshold": 8})


if __name__ == "__main__":
    unittest.main()
