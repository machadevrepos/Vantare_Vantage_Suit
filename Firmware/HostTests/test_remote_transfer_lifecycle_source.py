#!/usr/bin/env python3
"""Guard the Master remote Pause/Resume/Cancel runtime ownership lifecycle."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MASTER_MAIN = ROOT / "Firmware" / "Master" / "Core" / "Src" / "main.c"


def main() -> int:
    text = MASTER_MAIN.read_text(encoding="utf-8")
    write_start = text.find('extern "C" uint8_t exo_hub_ble_write')
    dispatch_start = text.find(
        "case exo::RecordReliableType::Pause:", write_start
    )
    dispatch_end = text.find(
        "case exo::RecordReliableType::VerifyOk:", dispatch_start
    )
    if write_start < 0 or dispatch_start < 0 or dispatch_end < 0:
        raise SystemExit("remote Pause/Resume/Cancel dispatch was not found")

    dispatch = text[dispatch_start:dispatch_end]
    required = (
        "bool remote_state_accepted",
        "remote_state_accepted = leaf_ble_manager.on_ble_reliable_pause",
        "remote_state_accepted = leaf_ble_manager.on_ble_reliable_resume",
        "remote_state_accepted = leaf_ble_manager.on_ble_reliable_cancel",
        "type == exo::RecordReliableType::Cancel",
        "g_remote_transfer_active",
        "g_remote_transfer_source_id == hdr.source_id",
        "g_remote_transfer_session_id == hdr.session_id",
        "g_remote_transfer_active = false;",
        "g_remote_transfer_source_id = 0U;",
        "g_remote_transfer_session_id = 0U;",
        "start_next_pending_node_manifest_now();",
    )
    missing = [token for token in required if token not in dispatch]
    if missing:
        details = "\n".join(f"  - {token}" for token in missing)
        raise SystemExit(
            "remote Cancel does not synchronously release the parallel runtime owner:\n"
            + details
        )

    forward_pos = dispatch.find("forward_remote_record_control")
    clear_pos = dispatch.find("g_remote_transfer_active = false;")
    if forward_pos < 0 or clear_pos < 0 or clear_pos < forward_pos:
        raise SystemExit(
            "remote Cancel runtime ownership must be released after forwarding control"
        )

    print("remote transfer lifecycle source checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
