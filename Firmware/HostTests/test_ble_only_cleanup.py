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


def function_body(text: str, signature: str, next_signature: str) -> str:
    start = text.find(signature)
    end = text.find(next_signature, start + len(signature))
    if start < 0 or end < 0:
        return ""
    return text[start:end]


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
    manager_header = (
        REPO_ROOT / "Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h"
    ).read_text(encoding="utf-8")
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

    manager_lifecycle_symbols = (
        "on_ble_reliable_resume",
        "on_ble_reliable_cancel",
        "on_ble_session_complete",
    )
    for symbol in manager_lifecycle_symbols:
        if symbol not in manager_header:
            failures.append(f"HubLeafBleManager lacks remote lifecycle operation: {symbol}")

    main_lifecycle_calls = (
        "leaf_ble_manager.on_ble_reliable_pause",
        "leaf_ble_manager.on_ble_reliable_resume",
        "leaf_ble_manager.on_ble_reliable_cancel",
        "leaf_ble_manager.on_ble_session_complete",
    )
    for call in main_lifecycle_calls:
        if call not in master_main:
            failures.append(f"Master remote lifecycle dispatch lacks call: {call}")

    held_verify = function_body(
        master_main,
        "static void training_pending_verify_store(",
        "static void release_remote_node_verify_ok(",
    )
    released_verify = function_body(
        master_main,
        "static void release_remote_node_verify_ok(",
        "static void master_training_csv_release_completed_verify_ok(",
    )
    if not held_verify or not released_verify:
        failures.append("Master VerifyOk lifecycle functions could not be located")
    else:
        for label, body in (
            ("held VerifyOk", held_verify),
            ("released VerifyOk", released_verify),
        ):
            if "g_remote_transfer_active = false" in body:
                failures.append(f"{label} clears ownership before SessionCompleteAck")
            if "start_next_pending_node_manifest_now();" in body:
                failures.append(f"{label} starts queued work before SessionCompleteAck")

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1

    print(f"BLE-only cleanup guard passed ({len(active_files())} active source files scanned)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
