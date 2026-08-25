#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
HTML = (ROOT / "host/desktop_tool/Exoskeleton.html").read_text(encoding="utf-8-sig")

checks = [
    ("Vantare Vantage | Session Console" in HTML, "professional console title missing"),
    ("MASTER_SD_STATUS: 0xB6" in HTML, "desktop must name report 0xB6 as Master SD status"),
    ("const BINARY_FIRST_SD_MODE = true" in HTML, "binary-first desktop mode must be explicit"),
    ("BinaryFinalizeNode" in HTML, "desktop must understand coordinator state 12"),
    ("R####M.BIN" in HTML and "R####N#.BIN" in HTML, "desktop must explain authoritative SD filenames"),
    ("Browser is control/status/live preview only" in HTML, "desktop must explain browser role"),
    (HTML.count("ExoskeletonTelemetry.dispatchArtifactRelay(relayMode") >= 3,
     "reliable, legacy, and v3 bulk paths must use the ownership dispatcher"),
    ("if (!binaryFirstMode) return \"browser_manual\";" in HTML and
     "return rawRelayDebugEnabled(flags) ? \"master_raw_read_only\" : \"compact_progress\";" in HTML,
     "browser-owned manual download must remain separate from binary-first raw trace"),
    ("Master raw-relay trace (read-only)" in HTML and
     "the browser never downloads, ACKs, NACKs, or verifies them" in HTML,
     "raw trace UI must state its read-only control contract"),
    ("Master SD collector owns the authoritative transfer" in HTML,
     "RecordDone must not arm a browser-owned bulk transfer"),
    ("\"Collecting\", \"Finalizing\", \"Incomplete\", \"Error\"" in HTML,
     "collection/error states must block starting a new recording"),
    ("Do not start a new session" in HTML,
     "incomplete-session UI must protect retained Node data from accidental reset"),
    ("failedSourceMask" in HTML and "cleanupPendingMask" in HTML,
     "Master SD status UI must expose failed/cleanup source masks"),
    ("encodeRetrySource" in HTML and "retryFailedBtn" in HTML and
     "dv.setUint8(0, 0x10);" in HTML,
     "failed sources must be re-pullable from the UI (RetrySource 0x10)"),
    ("(completed & expected) !== expected" in HTML,
     "Master SD Complete state must not hide an incomplete expected-source mask"),
    ("liveExpectedCadenceMs" in HTML,
     "live graph gap detection must use preview delivery cadence"),
    ("TRAIN CSV:" not in HTML, "legacy TRAIN CSV user-facing status must be removed"),
    ("Master CSV failure" not in HTML, "legacy Master CSV failure wording must be removed"),
    ("Exoskeleton BLE Plotter Android" not in HTML, "legacy Android title must be removed"),
]

failures = [message for ok, message in checks if not ok]
if failures:
    for failure in failures:
        print(f"ERROR: {failure}", file=sys.stderr)
    raise SystemExit(1)
print("desktop binary-first guards passed")
