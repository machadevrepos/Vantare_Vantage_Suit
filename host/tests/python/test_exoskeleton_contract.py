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
