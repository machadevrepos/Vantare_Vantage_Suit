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
    allocator_path = ROOT / "Firmware/common/inc/exo/storage/master_binary_session_index.h"
    require(allocator_path.exists(), "binary run-index allocator must exist", failures)
    if allocator_path.exists():
        allocator = allocator_path.read_text(encoding="utf-8")
        for suffix in ("M.BIN", "N1.BIN", "N2.BIN", "N3.BIN", "N4.BIN"):
            require(suffix in allocator, f"allocator must reserve {suffix} names", failures)
        # The allocator may create/truncate only the 8-byte RUNIDX.BIN marker
        # (crash-safe cached last index); it must never touch payload files.
        create_always = allocator.count("FA_CREATE_ALWAYS")
        require(create_always == allocator.count("FA_CREATE_ALWAYS | FA_WRITE") and
                allocator.count("kMarkerPath, FA_CREATE_ALWAYS | FA_WRITE") == create_always and
                "RUNIDX.BIN" in allocator,
                "allocator must never create/truncate session payload files "
                "(only the RUNIDX.BIN marker)", failures)
        require("USERFatFs.fs_type == 0U" in allocator,
                "allocator must not remount an already-mounted FatFs volume", failures)

    coordinator = read("Firmware/common/inc/exo/protocol/master_training_csv_coordinator.h")
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

    main_src = read("Firmware/Master/Core/Src/main.cpp")
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
    archive = main_src.find("begin_archive(g_active_session_file_index)")
    finalized = main_src.find("master_training_csv_coordinator.on_master_finalized", archive)
    require(archive >= 0 and finalized > archive,
            "Master binary must be archived before the coordinator accepts it as durable", failures)
    require("g_local_archive_attempt >= kLocalArchiveMaxAttempts" in main_src and
            "master archive attempt=" in main_src,
            "Master must retry transient canonical archive failures before declaring durability failure", failures)
    require("begin_finalize(g_local_finalize_duration_ms)" in main_src and
            "service_finalize(kLocalRecordArchiveChunkBytes)" in main_src and
            "service_archive(kLocalRecordArchiveChunkBytes)" in main_src and
            "LocalRecordPhase::Finalizing" in main_src and
            "LocalRecordPhase::Archiving" in main_src and
            "master_archive_ok" not in main_src,
            "Master finalize/archive must advance as bounded superloop phases so BLE dispatch keeps running", failures)
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

    recorder = read("Firmware/common/inc/exo/storage/master_sd_session_recorder.h")
    require("/SESSIONS/R0000T.BIN" in recorder,
            "Master archive must use a temporary compact file", failures)
    require("read(archive_offset_, copy_buffer_, chunk)" in recorder,
            "Master archive must copy the recorder's canonical logical stream", failures)
    require("service_archive_validate" in recorder and "f_size(&archive_file_)" in recorder,
            "Master archive must validate the exact compact file size", failures)
    require("session_header_crc(compact_header)" in recorder and
            "archive_crc_ != header_.payload_crc32" in recorder and
            "crc32_update(archive_crc_" in recorder,
            "Master archive must validate both header and payload CRCs", failures)
    require("f_rename(temp_path_, archive_path_)" in recorder and
            "f_rename(EXO_MASTER_REC_FINAL_PATH, archive_path)" not in recorder,
            "Master archive must install the validated compact file, not rename the sparse source", failures)
    archive_install = recorder.find("f_rename(temp_path_, archive_path_)")
    live_remove = recorder.find("f_unlink(EXO_MASTER_REC_FINAL_PATH)", archive_install)
    require(archive_install >= 0 and live_remove > archive_install,
            "sparse MREC.BIN must survive until the compact archive is installed", failures)
    require(recorder.count("f_unlink(temp_path_)") >= 2,
            "failed compact/archive attempts must clean temporary files", failures)
    installed_tail = recorder[archive_install:] if archive_install >= 0 else ""
    require("set_archive_cleanup_path(archive_path_)" in installed_tail and
            "service_archive_cleanup()" in recorder,
            "failed post-install validation must remain retry-cleanable without losing a live FatFs handle", failures)
    require("FIL archive_file_{};" in recorder and "bool archive_file_open_ = false;" in recorder,
            "archive temp/verify handles must remain recorder-owned across close failures", failures)
    require("bool close_tracked_file(FIL &file, bool &open_flag)" in recorder and
            "open_flag = false;" in recorder,
            "FatFs open-state flags must only clear through a successful tracked close", failures)
    require("if (archive_required_)" in recorder and "last_error_ = FR_LOCKED;" in recorder,
            "a finalized unarchived Master recording must block overwrite by a new session", failures)
    require("if (!close_all_files())" in recorder,
            "a new Master recording must not start while a prior FatFs handle cannot be closed", failures)
    require("set_archive_cleanup_path(archive_phase_ == ArchivePhase::InstallValidate ?" in recorder and
            "archive_path_ : temp_path_)" in recorder and
            "set_archive_cleanup_path(archive_path_)" in recorder,
            "failed temp/final archive closes must remain retry-cleanable", failures)
    durable = recorder.find("archive_required_ = false;", archive_install)
    durable_tail = recorder[durable:] if durable >= 0 else ""
    require(durable >= 0 and "sparse_cleanup_pending_ = true;" in durable_tail and
            "return ArchiveStep::Complete;" in durable_tail,
            "once R####M.BIN is validated, cleanup-only failures must not revoke durability", failures)
    require("service_archive_recovery" in recorder and "kArchiveRecoveryRetryMs" in recorder and
            "service_archive_recovery(HAL_GetTick())" in main_src,
            "a latched archive (SD full/rename failure) must self-heal with backoff instead of refusing new sessions forever", failures)

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1
    print("binary-first invariant guards passed")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
