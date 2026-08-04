#!/usr/bin/env python3
"""Guard active firmware against reintroducing removed RS485-era code."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DELETED_PATHS = (
    "Firmware/LIBRARY/CUSTOM/RECORDING_BRIDGE.h",
    "Firmware/LIBRARY/CUSTOM/RECORDING_INTEGRATION_NOTES.md",
    "Firmware/LIBRARY/CUSTOM/RS485.h",
    "Firmware/LIBRARY/CUSTOM/RS485_FRAME_PROTOCOL.h",
    "Firmware/LIBRARY/CUSTOM/RS485_RECORD_MASTER_APP.h",
    "Firmware/LIBRARY/CUSTOM/RS485_RECORD_NODE_APP.h",
)

ACTIVE_ROOTS = (
    REPO_ROOT / "Firmware/Master/Core",
    REPO_ROOT / "Firmware/Master/STM32_WPAN",
    REPO_ROOT / "Firmware/Node/Core",
    REPO_ROOT / "Firmware/Node/STM32_WPAN",
    REPO_ROOT / "Firmware/LIBRARY/CUSTOM",
)

TEXT_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp", ".inc"}

FORBIDDEN_PATTERNS = {
    "MasterRs485_": re.compile(r"\bMasterRs485_"),
    "HubRs485": re.compile(r"\bHubRs485"),
    "master_rs485_recording": re.compile(r"\bmaster_rs485_recording\b"),
    "node_rs485_recording": re.compile(r"\bnode_rs485_recording\b"),
    "comms_rs485": re.compile(r"\bcomms_rs485\b"),
    "RS485_RECORD_": re.compile(r"\bRS485_RECORD_"),
    "EXO_NODE_UART_RS485_MODE": re.compile(r"\bEXO_NODE_UART_RS485_MODE\b"),
    "removed RS485 include": re.compile(
        r"#\s*include\s*[<\"](?:RS485(?:_FRAME_PROTOCOL|_RECORD_MASTER_APP|_RECORD_NODE_APP)?|RECORDING_BRIDGE)\.h[>\"]"
    ),
}


def active_files() -> list[Path]:
    files: list[Path] = []
    for root in ACTIVE_ROOTS:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in TEXT_SUFFIXES:
                files.append(path)
    return sorted(files)


def main() -> int:
    failures: list[str] = []

    for relative_path in DELETED_PATHS:
        if (REPO_ROOT / relative_path).exists():
            failures.append(f"deleted compatibility file still exists: {relative_path}")

    for path in active_files():
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        relative = path.relative_to(REPO_ROOT)
        for label, pattern in FORBIDDEN_PATTERNS.items():
            for match in pattern.finditer(text):
                line = text.count("\n", 0, match.start()) + 1
                failures.append(f"{relative}:{line}: stale {label}")

    master_main = (REPO_ROOT / "Firmware/Master/Core/Src/main.c").read_text(encoding="utf-8")
    diagnostics = (
        REPO_ROOT / "Firmware/LIBRARY/CUSTOM/ACQUISITION_DIAGNOSTICS.h"
    ).read_text(encoding="utf-8")
    node_main = (REPO_ROOT / "Firmware/Node/Core/Src/main.c").read_text(encoding="utf-8")

    if "leaf_ble_manager" not in master_main:
        failures.append("Master main.c does not expose leaf_ble_manager")
    if "comms_leaf" not in diagnostics:
        failures.append("Acquisition diagnostics does not expose comms_leaf")
    if "node_stream_enabled" not in node_main:
        failures.append("Node main.c does not expose direct BLE stream state")

    record_done_guards = (
        "message.command != exo::RecordCommand::RecordDone",
        "message.session_id == 0U",
        "message.total_size < sizeof(exo::SessionHeader)",
    )
    for guard in record_done_guards:
        if guard not in master_main:
            failures.append(f"Master record-done ingest lacks guard: {guard}")

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1

    print(f"BLE-only cleanup guard passed ({len(active_files())} active source files scanned)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
