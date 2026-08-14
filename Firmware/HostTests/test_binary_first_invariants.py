#!/usr/bin/env python3
"""Source-level guards for the binary-first Master SD collection path."""
from __future__ import annotations
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")

def require(ok: bool, message: str, failures: list[str]) -> None:
    if not ok:
        failures.append(message)

def main() -> int:
    failures: list[str] = []
    allocator_path = ROOT / "Firmware/Master/Core/Inc/MASTER_BINARY_SESSION_INDEX.h"
    require(allocator_path.exists(), "binary run-index allocator must exist", failures)
    if allocator_path.exists():
        allocator = allocator_path.read_text(encoding="utf-8")
        for suffix in ("M.BIN", "N1.BIN", "N2.BIN", "N3.BIN", "N4.BIN"):
            require(suffix in allocator, f"allocator must reserve {suffix} names", failures)
        require("FA_CREATE_ALWAYS" not in allocator,
                "allocator must never create/truncate session payload files", failures)
        require("USERFatFs.fs_type == 0U" in allocator,
                "allocator must not remount an already-mounted FatFs volume", failures)

    coordinator = read("Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h")
    require("EXO_MASTER_BINARY_ONLY_BUILD" in coordinator and
            "static constexpr bool binary_only_ = true" in coordinator,
            "production build must support compile-time binary-only specialization", failures)
    require("BinaryFinalizeNode" in coordinator,
            "coordinator needs an explicit post-validation binary state", failures)
    require("begin_binary_session" in coordinator and "binary_only_" in coordinator,
            "coordinator must expose binary-only collection mode", failures)
    require("binary_only_ ? file_index_ : logger_.file_index()" in coordinator,
            "node staging index must not depend on the CSV logger in binary mode", failures)
    require("cleanup_pending_mask_" in coordinator,
            "binary collector must record nodes whose erase/VerifyOk could not be delivered", failures)
    binary_finalize = coordinator.find("void begin_binary_node_finalize()")
    binary_complete = coordinator.find("void complete_binary_node", binary_finalize)
    binary_block = coordinator[binary_finalize:binary_complete]
    require(binary_finalize >= 0 and
            binary_block.find("stager_.discard_after_success()") >= 0 and
            binary_block.find("reliable_control_.verify_ok") > binary_block.find("stager_.discard_after_success()"),
            "node VerifyOk/erase permission must only be queued after the validated SD stage is closed", failures)

    main_src = read("Firmware/Master/Core/Src/main.c")
    require("#define EXO_MASTER_BINARY_ONLY_BUILD 1" in main_src,
            "Master must select the compile-time binary-only specialization", failures)
    require("#include <MASTER_BINARY_SESSION_INDEX.h>" in main_src,
            "Master must include the binary session index allocator", failures)
    require("begin_binary_session" in main_src,
            "record start must select binary-only collection", failures)
    helper = main_src.find("record_sync_begin_binary_collection")
    prepare = main_src.find("static uint8_t record_sync_send_prepare", helper)
    require(helper >= 0 and prepare > helper and
            "record_sync_begin_binary_collection()" in main_src[prepare:prepare + 320],
            "binary run allocation must complete before PrepareRecord is sent", failures)
    require("g_active_session_file_index" in main_src,
            "Master must persist the allocated run index across record-sync teardown", failures)
    archive = main_src.find("archive_to_index(g_active_session_file_index)")
    finalized = main_src.find("master_training_csv_coordinator.on_master_finalized", archive)
    require(archive >= 0 and finalized > archive,
            "Master binary must be archived before the coordinator accepts it as durable", failures)
    require("g_local_record_phase = LocalRecordPhase::Finished;" in main_src,
            "binary-first mode must not enter the Master-to-browser bulk transfer phase", failures)
    require("!master_training_csv_coordinator.binary_only() &&" in main_src,
            "node manifest forwarding to the browser must be disabled in binary-only mode", failures)
    require("leaf_ble_manager.on_ble_session_complete" in main_src and
            "master_training_csv_coordinator.binary_only()" in main_src,
            "Master must release each leaf after validated binary completion", failures)
    require("leaf_ble_manager.on_ble_reliable_cancel" in main_src and
            "retained_on_node=1" in main_src,
            "a stalled node must release the local scheduler slot without erasing node flash", failures)

    recorder = read("Firmware/LIBRARY/CUSTOM/MASTER_SD_SESSION_RECORDER.h")
    require("/SESSIONS/R0000T.BIN" in recorder,
            "Master archive must use a temporary compact file", failures)
    require("read(logical_offset, copy_buffer_, chunk)" in recorder,
            "Master archive must copy the recorder's canonical logical stream", failures)
    require("validate_compact_archive" in recorder and "f_size(&file)" in recorder,
            "Master archive must validate the exact compact file size", failures)
    require("session_header_crc(compact_header)" in recorder and
            "compact_header.payload_crc32" in recorder and
            "crc32_update(payload_crc" in recorder,
            "Master archive must validate both header and payload CRCs", failures)
    require("f_rename(temp_path, archive_path)" in recorder and
            "f_rename(EXO_MASTER_REC_FINAL_PATH, archive_path)" not in recorder,
            "Master archive must install the validated compact file, not rename the sparse source", failures)
    archive_install = recorder.find("f_rename(temp_path, archive_path)")
    live_remove = recorder.find("f_unlink(EXO_MASTER_REC_FINAL_PATH)", archive_install)
    require(archive_install >= 0 and live_remove > archive_install,
            "sparse MREC.BIN must survive until the compact archive is installed", failures)
    require(recorder.count("f_unlink(temp_path)") >= 2,
            "failed compact/archive attempts must clean temporary files", failures)
    installed_tail = recorder[archive_install:] if archive_install >= 0 else ""
    require(installed_tail.count("(void)f_unlink(archive_path);") >= 2,
            "failed post-install verification must remove the blocking archive so the same index can be retried", failures)

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1
    print("binary-first invariant guards passed")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
