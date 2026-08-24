#!/usr/bin/env python3
"""Guard remote lifecycle and scheduler/coordinator ownership alignment."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MASTER_MAIN = ROOT / "Firmware" / "Master" / "Core" / "Src" / "main.cpp"


def main() -> int:
    text = MASTER_MAIN.read_text(encoding="utf-8")

    ingest_start = text.find(
        'extern "C" uint8_t exo_hub_leaf_record_done_ingest'
    )
    ingest_end = text.find(
        'extern "C" void exo_hub_leaf_record_frame_ingest', ingest_start
    )
    if ingest_start < 0 or ingest_end < 0:
        raise SystemExit("leaf RecordDone ingest block was not found")

    ingest = text[ingest_start:ingest_end]
    if "master_training_csv_coordinator.on_node_record_done(message);" in ingest:
        raise SystemExit(
            "RecordDone ingest starts CSV before scheduler ownership is selected"
        )

    queue_pos = ingest.find("leaf_ble_manager.queue_record_done(message)")
    cache_pos = ingest.find("g_training_node_done[training_index] = message;")
    if queue_pos < 0 or cache_pos < 0 or cache_pos < queue_pos:
        raise SystemExit(
            "RecordDone metadata must be cached only after scheduler queue acceptance"
        )

    replay_token = (
        "master_training_csv_coordinator.on_node_record_done("
        "g_training_node_done[index]);"
    )
    if replay_token not in text:
        raise SystemExit(
            "selected scheduler owner is not replayed into CSV coordinator"
        )

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

    print("remote transfer lifecycle and owner alignment source checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
