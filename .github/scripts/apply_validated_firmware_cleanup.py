#!/usr/bin/env python3
from __future__ import annotations

import base64
import gzip
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import textwrap

REPO = Path.cwd().resolve()
WORK = Path('/tmp/vantage-validated-firmware-cleanup')
APPLY_REF = 'origin/ble-only-firmware-cleanup-apply'
BASE_REF = 'origin/2nd_Branch'
PATCH_SHA256 = '539c0486217bfdacd2cfb1adc857350ce58dbda2dde3797bd94e5d60fcdd8116'
MASTER_BLOB = '8daf733b87ca7af9d4e61adca4dab8a71ef6abd5'
NODE_BLOB = 'fa66d023c600f1e9dab30a8982f5a9c94db27a28'

PARTS = [
    'part-01.b64', 'part-02.b64', 'part-03.b64', 'part-04.b64',
    'part-05.b64', 'part-06a.b64', 'part-06b.b64', 'part-07.b64',
    'part-08.b64', 'part-09.b64',
]
PART_BLOBS = {
    'part-01.b64': '0cffd530b67683b40796fc5d3d8a8ce1ed2aaa51',
    'part-02.b64': 'f6f61ce45f035b4802461e71bab6a2910cbd2de7',
    'part-03.b64': 'a6183b4f7b09865db94d6ba4fad0ed653eab6db3',
    'part-04.b64': 'bbc6921e0604320420c665822ff4c2fed0a7ae77',
    'part-05.b64': '776131caf22fbbb5e8b6095fe6c8cccf1ca29625',
    'part-06a.b64': '3779523ba418e3066d09eee0521eb6db7264ce69',
    'part-06b.b64': '11d59b3b3649896bc73c45961f3d49b1ee1ff95a',
    'part-07.b64': '16b3febfdf6ce733c88046c2630a1e1a7256c736',
    'part-08.b64': 'b579c21b18744a6d7c199f1e981dc33210c6eee8',
    'part-09.b64': 'b7a3748d74f88099ee2fa490b85d3fccd45a066d',
}
PRESERVE = [
    'Docs/Superpowers/Plans/2026-08-03-ble-only-firmware-cleanup.md',
    'Docs/Superpowers/Specs/2026-08-03-ble-only-firmware-cleanup-design.md',
    'Firmware/HostTests/test_ble_only_cleanup.py',
]


def run(*args: str, cwd: Path = REPO, check: bool = True,
        capture: bool = False) -> subprocess.CompletedProcess[str]:
    print('+', ' '.join(args), flush=True)
    return subprocess.run(
        args,
        cwd=cwd,
        check=check,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )


def output(*args: str, cwd: Path = REPO) -> str:
    result = run(*args, cwd=cwd, capture=True)
    assert result.stdout is not None
    return result.stdout.strip()


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(f'blob {len(data)}\0'.encode('ascii') + data).hexdigest()


def replace_once(path: str, old: str, new: str) -> None:
    target = WORK / path
    text = target.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected one occurrence, found {count}')
    target.write_text(text.replace(old, new, 1))


def reconstruct_cleanup_patch() -> Path:
    pieces: dict[str, bytes] = {}
    for name in PARTS:
        raw = subprocess.check_output(
            ['git', 'show', f'{APPLY_REF}:.cleanup-patch/{name}'],
            cwd=REPO,
        )
        wanted = PART_BLOBS[name]
        actual = git_blob_sha(raw)
        if name != 'part-06b.b64' and actual != wanted:
            raise RuntimeError(f'{name}: expected blob {wanted}, got {actual}')
        if name == 'part-06b.b64' and actual != wanted:
            alphabet = b'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/='
            repaired: bytes | None = None
            for position in range(len(raw) + 1):
                prefix, suffix = raw[:position], raw[position:]
                for value in alphabet:
                    candidate = prefix + bytes((value,)) + suffix
                    if git_blob_sha(candidate) == wanted:
                        repaired = candidate
                        print(f'repaired {name} at byte {position}', flush=True)
                        break
                if repaired is not None:
                    break
            if repaired is None:
                raise RuntimeError(f'unable to repair {name} to blob {wanted}')
            raw = repaired
        if git_blob_sha(raw) != wanted:
            raise RuntimeError(f'{name}: repaired blob mismatch')
        pieces[name] = raw

    encoded = b''.join(pieces[name] for name in PARTS)
    patch = gzip.decompress(base64.b64decode(encoded, validate=False))
    actual_sha = hashlib.sha256(patch).hexdigest()
    if actual_sha != PATCH_SHA256:
        raise RuntimeError(f'cleanup patch SHA-256 mismatch: {actual_sha}')
    patch_path = Path('/tmp/ble-cleanup-validated.patch')
    patch_path.write_bytes(patch)
    return patch_path


def prepare_worktree(current_head: str) -> None:
    if WORK.exists():
        run('git', 'worktree', 'remove', '--force', str(WORK), check=False)
        shutil.rmtree(WORK, ignore_errors=True)
    run('git', 'fetch', 'origin',
        '2nd_Branch:refs/remotes/origin/2nd_Branch',
        'ble-only-firmware-cleanup-apply:refs/remotes/origin/ble-only-firmware-cleanup-apply')
    run('git', 'worktree', 'add', '--detach', str(WORK), BASE_REF)
    if output('git', 'rev-parse', current_head, cwd=WORK) != current_head:
        raise RuntimeError('current cleanup head is not available in worktree repository')


def apply_cleanup_patch(patch_path: Path) -> None:
    run('git', 'apply', '--check', str(patch_path), cwd=WORK)
    run('git', 'apply', str(patch_path), cwd=WORK)
    master = output('git', 'hash-object', 'Firmware/Master/Core/Src/main.c', cwd=WORK)
    node = output('git', 'hash-object', 'Firmware/Node/Core/Src/main.c', cwd=WORK)
    if master != MASTER_BLOB or node != NODE_BLOB:
        raise RuntimeError(f'cleanup target blob mismatch: master={master}, node={node}')
    for relative in PRESERVE:
        source = REPO / relative
        if source.exists():
            destination = WORK / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)


def add_regression_tests() -> None:
    firmware_test = r'''#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

#include "HUB_LEAF_BLE_MANAGER.h"
#include "MASTER_NODE_RELIABLE_CONTROL.h"
#include "MASTER_NODE_TRANSFER_WINDOW.h"
#include "MASTER_TRAINING_CSV_FORMATTER.h"

namespace {
int failures = 0;
#define EXPECT_TRUE(expr) do { if (!(expr)) { std::cerr << __FILE__ << ':' << __LINE__ << ": expected " #expr "\n"; ++failures; } } while (0)

struct Transport {
    std::vector<uint8_t> frame;
    static bool send(void *context, uint8_t, const uint8_t *data, uint16_t length)
    {
        auto &self = *static_cast<Transport *>(context);
        self.frame.assign(data, data + length);
        return true;
    }
};

exo::RecordReliableFrameHeader frame_header(const std::vector<uint8_t> &frame)
{
    exo::RecordReliableFrameHeader header{};
    if (frame.size() >= sizeof(header)) std::memcpy(&header, frame.data(), sizeof(header));
    return header;
}

template<typename Manager, typename = void>
struct CongestionProbe {
    static bool run() { return false; }
};

template<typename Manager>
struct CongestionProbe<Manager, std::void_t<
    decltype(std::declval<Manager &>().peek_next_live_sample(
        std::declval<typename Manager::LiveSample &>(), uint32_t{})),
    decltype(std::declval<Manager &>().on_live_sample_send_result(bool{}, uint32_t{})),
    decltype(std::declval<const Manager &>().live_preview_congested())>> {
    static bool run()
    {
        Manager manager;
        uint8_t value = 1U;
        if (!manager.push_leaf_sample(1U, 1U, &value, 1U)) return false;
        value = 2U;
        if (!manager.push_leaf_sample(2U, 1U, &value, 1U)) return false;
        typename Manager::LiveSample sample{};
        if (!manager.peek_next_live_sample(sample, 0U) || sample.node_id != 1U) return false;
        manager.on_live_sample_send_result(false, 0U);
        if (!manager.live_preview_congested()) return false;
        if (manager.peek_next_live_sample(sample, 79U)) return false;
        if (!manager.peek_next_live_sample(sample, 80U) || sample.node_id != 2U) return false;
        manager.on_live_sample_send_result(true, 80U);
        if (manager.peek_next_live_sample(sample, 159U)) return false;
        if (!manager.peek_next_live_sample(sample, 160U) || sample.node_id != 1U) return false;
        manager.on_live_sample_send_result(true, 160U);
        value = 3U;
        if (!manager.push_leaf_sample(3U, 1U, &value, 1U)) return false;
        if (!manager.peek_next_live_sample(sample, 5080U) || sample.node_id != 3U) return false;
        manager.on_live_sample_send_result(true, 5080U);
        if (manager.live_preview_congested()) return false;
        value = 4U;
        if (!manager.push_leaf_sample(4U, 1U, &value, 1U)) return false;
        if (manager.peek_next_live_sample(sample, 5119U)) return false;
        return manager.peek_next_live_sample(sample, 5120U) && sample.node_id == 4U;
    }
};
}

int main()
{
    exo::MasterNodeTransferWindow window;
    EXPECT_TRUE(window.begin(1U, 42U, 400U, 180U));
    const auto first = window.inspect(1U, 42U, 0U, 0U, 180U, true, false);
    EXPECT_TRUE(window.commit(first));
    const auto corrupt = window.inspect(1U, 42U, 0xFFFFFFFFU, 180U, 180U, false, false);
    EXPECT_TRUE(corrupt.decision == exo::NodeTransferDecision::NackCorrupt);
    EXPECT_TRUE(corrupt.request_chunk == 1U);

    Transport transport;
    exo::MasterNodeReliableControl control(&Transport::send, &transport);
    exo::RecordDoneMessage done{};
    done.node_id = 1U;
    done.session_id = 42U;
    done.total_size = 1000U;
    done.payload_crc32 = 0x12345678U;
    EXPECT_TRUE(control.begin(done, 180U));
    EXPECT_TRUE(control.service(1U));
    EXPECT_TRUE(control.nack_range(0xFFFFFFFFU));
    EXPECT_TRUE(control.service(2U));
    EXPECT_TRUE(frame_header(transport.frame).byte_offset == 0xFFFFFFFFU);
    EXPECT_TRUE(control.ack_window(0xFFFFFFFFU));
    EXPECT_TRUE(control.service(3U));
    EXPECT_TRUE(frame_header(transport.frame).byte_offset == 0xFFFFFFFFU);

    exo::ble_hub::HubLeafBleManager manager;
    exo::RecordDoneMessage done1{};
    done1.node_id = 1U; done1.session_id = 77U; done1.total_size = 100U; done1.payload_crc32 = 1U;
    exo::RecordDoneMessage done2 = done1;
    done2.node_id = 2U; done2.payload_crc32 = 2U;
    EXPECT_TRUE(manager.queue_record_done(done1));
    EXPECT_TRUE(manager.queue_record_done(done2));
    exo::RecordDoneMessage selected{};
    EXPECT_TRUE(manager.pop_next_record_done(selected));
    EXPECT_TRUE(selected.node_id == 1U);
    EXPECT_TRUE(!manager.pop_next_record_done(selected));
    EXPECT_TRUE(manager.active_source_id() == 1U && manager.active_session_id() == 77U);

    char numeric[32]{};
    exo::training_csv::CsvRowWriter writer(numeric, sizeof(numeric));
    EXPECT_TRUE(exo::training_csv::append_double_or_blank(writer, 1.0e30));
    EXPECT_TRUE(writer.length() == 0U);
    exo::training_csv::TrainingCsvRowContext row_context{};
    row_context.source.target_rate_hz = 100U;
    row_context.source.attempted_count = 1U;
    row_context.source.captured_count = 1U;
    exo::Bno85Sample extreme{};
    extreme.quat_real = 1.0F;
    extreme.linear_accel_x = 1.0e30F;
    char row[2048]{};
    size_t written = 0U;
    EXPECT_TRUE(exo::training_csv::format_bno_row(row, sizeof(row), written,
        1U, 0U, 0U, 0U, 1000U, row_context, false, 0U, extreme));

    exo::Bno85Sample sample{};
    constexpr double pi = 3.14159265358979323846;
    const double roll = 30.0 * pi / 180.0;
    const double pitch = 20.0 * pi / 180.0;
    const double yaw = 10.0 * pi / 180.0;
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    sample.quat_real = static_cast<float>(cr * cp * cy + sr * sp * sy);
    sample.quat_i = static_cast<float>(sr * cp * cy - cr * sp * sy);
    sample.quat_j = static_cast<float>(cr * sp * cy + sr * cp * sy);
    sample.quat_k = static_cast<float>(cr * cp * sy - sr * sp * cy);
    const auto derived = exo::training_csv::derive_bno_features(sample);
    EXPECT_TRUE(std::fabs(derived.roll_deg - 30.0) < 0.001);
    EXPECT_TRUE(std::fabs(derived.pitch_deg - 20.0) < 0.001);
    EXPECT_TRUE(std::fabs(derived.yaw_deg - 10.0) < 0.001);

    EXPECT_TRUE(CongestionProbe<exo::ble_hub::HubLeafBleManager>::run());
    return failures == 0 ? 0 : 1;
}
'''

    logger_test = r'''#include <cstdint>
#include <cstring>
#include <iostream>

#include "MASTER_TRAINING_CSV_LOGGER.h"

namespace {
int failures = 0;
int marker_open_failures = 0;
int marker_open_calls = 0;
int rename_calls = 0;
#define EXPECT_TRUE(expr) do { if (!(expr)) { std::cerr << __FILE__ << ':' << __LINE__ << ": expected " #expr "\n"; ++failures; } } while (0)

FRESULT mkdir_ok(const TCHAR*) { return FR_OK; }
FRESULT stat_missing(const TCHAR*, FILINFO*) { return FR_NO_FILE; }
FRESULT open_controlled(FIL*, const TCHAR *path, BYTE)
{
    const char *text = reinterpret_cast<const char *>(path);
    if (text != nullptr && std::strstr(text, ".OK") != nullptr) {
        ++marker_open_calls;
        if (marker_open_failures > 0) {
            --marker_open_failures;
            return FR_DISK_ERR;
        }
    }
    return FR_OK;
}
FRESULT write_ok(FIL*, const void*, UINT bytes, UINT *written) { *written = bytes; return FR_OK; }
FRESULT sync_ok(FIL*) { return FR_OK; }
FRESULT close_ok(FIL*) { return FR_OK; }
FRESULT rename_ok(const TCHAR*, const TCHAR*) { ++rename_calls; return FR_OK; }

exo::SessionHeader metadata(uint8_t source, uint32_t session)
{
    exo::SessionHeader header{};
    header.node_id = source;
    header.session_id = session;
    header.bno85_target_rate_hz = 100U;
    header.bno85_attempted_count = 2U;
    header.bno85_captured_count = 2U;
    header.icm45686_target_rate_hz = 100U;
    header.icm45686_attempted_count = 2U;
    header.icm45686_captured_count = 2U;
    header.payload_crc32 = 0x12345678U + source;
    return header;
}
}

int main()
{
    const exo::training_csv::TrainingCsvFatFsOps ops = {
        mkdir_ok, stat_missing, open_controlled, write_ok, sync_ok, close_ok, rename_ok
    };

    exo::MasterTrainingCsvLogger metadata_logger(&ops);
    EXPECT_TRUE(metadata_logger.begin(9U, 0x03U, 0U));
    exo::Bno85Sample sample{};
    sample.quat_real = 1.0F;
    EXPECT_TRUE(!metadata_logger.append_bno(1U, 1000U, sample, false, 0U, 1U));
    EXPECT_TRUE(metadata_logger.last_operation() ==
        exo::training_csv::TrainingCsvLogOperation::InvalidSourceMetadata);
    EXPECT_TRUE(metadata_logger.set_source_metadata(1U, metadata(1U, 9U)));
    EXPECT_TRUE(metadata_logger.append_bno(1U, 2000U, sample, false, 0U, 2U));

    marker_open_failures = 1;
    marker_open_calls = 0;
    rename_calls = 0;
    exo::MasterTrainingCsvLogger publish_logger(&ops);
    EXPECT_TRUE(publish_logger.begin(10U, 0x01U, 0U));
    EXPECT_TRUE(publish_logger.set_source_metadata(0U, metadata(0U, 10U)));
    EXPECT_TRUE(publish_logger.append_bno(0U, 1000U, sample, false, 0U, 1U));
    EXPECT_TRUE(publish_logger.mark_source_complete(0U, 2U));
    EXPECT_TRUE(!publish_logger.shutdown(3U));
    EXPECT_TRUE(!publish_logger.published());
    EXPECT_TRUE(rename_calls == 1);
    EXPECT_TRUE(publish_logger.publish());
    EXPECT_TRUE(publish_logger.published());
    EXPECT_TRUE(rename_calls == 1);
    EXPECT_TRUE(marker_open_calls == 2);
    return failures == 0 ? 0 : 1;
}
'''
    host_tests = WORK / 'Firmware/HostTests'
    host_tests.mkdir(parents=True, exist_ok=True)
    (host_tests / 'test_coderabbit_firmware_regressions.cpp').write_text(firmware_test)
    (host_tests / 'test_coderabbit_csv_logger_regressions.cpp').write_text(logger_test)


def ff_include_dir() -> Path:
    preferred = sorted((WORK / 'Firmware/HostTests').rglob('ff.h'))
    candidates = preferred or sorted((WORK / 'Firmware').rglob('ff.h'))
    if not candidates:
        raise RuntimeError('ff.h not found')
    return candidates[0].parent


def compile_test(source: str, extra: list[str] | None = None,
                 suffix: str = '') -> Path:
    executable = Path('/tmp') / (Path(source).stem + suffix)
    command = [
        'g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
        '-I', 'Firmware/LIBRARY/CUSTOM', '-I', 'Firmware/Master/Core/Inc',
    ]
    if extra:
        command.extend(extra)
    command.extend([source, '-o', str(executable)])
    run(*command, cwd=WORK)
    return executable


def verify_red() -> None:
    firmware = compile_test(
        'Firmware/HostTests/test_coderabbit_firmware_regressions.cpp',
        suffix='_red')
    result = run(str(firmware), cwd=WORK, check=False)
    if result.returncode == 0:
        raise RuntimeError('firmware regression test unexpectedly passed before fixes')

    ff_dir = ff_include_dir()
    logger = compile_test(
        'Firmware/HostTests/test_coderabbit_csv_logger_regressions.cpp',
        extra=['-I', str(ff_dir)], suffix='_red')
    result = run(str(logger), cwd=WORK, check=False)
    if result.returncode == 0:
        raise RuntimeError('logger regression test unexpectedly passed before fixes')


def apply_confirmed_fixes() -> None:
    replace_once(
        'Firmware/LIBRARY/CUSTOM/MASTER_NODE_TRANSFER_WINDOW.h',
        '''            result.decision = NodeTransferDecision::NackCorrupt;\n            result.request_chunk = chunk_index;\n            return result;''',
        '''            result.decision = NodeTransferDecision::NackCorrupt;\n            return result;''')

    control_path = WORK / 'Firmware/Master/Core/Inc/MASTER_NODE_RELIABLE_CONTROL.h'
    control = control_path.read_text()
    if control.count('next_chunk * static_cast<uint32_t>(chunk_size_)') != 1:
        raise RuntimeError('unexpected ACK offset expression')
    if control.count('first_chunk * static_cast<uint32_t>(chunk_size_)') != 1:
        raise RuntimeError('unexpected NACK offset expression')
    control = control.replace(
        'next_chunk * static_cast<uint32_t>(chunk_size_)',
        'chunk_byte_offset_(next_chunk, chunk_size_)', 1)
    control = control.replace(
        'first_chunk * static_cast<uint32_t>(chunk_size_)',
        'chunk_byte_offset_(first_chunk, chunk_size_)', 1)
    anchor = '    bool queue_frame(RecordReliableType type, uint32_t chunk_index,\n'
    helper = '''    static uint32_t chunk_byte_offset_(uint32_t chunk_index, uint16_t chunk_size)\n    {\n        const uint64_t offset = static_cast<uint64_t>(chunk_index) * chunk_size;\n        return offset > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(offset);\n    }\n\n'''
    if control.count(anchor) != 1:
        raise RuntimeError('reliable-control insertion anchor not found')
    control_path.write_text(control.replace(anchor, helper + anchor, 1))

    hub_path = WORK / 'Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h'
    hub = hub_path.read_text()
    old_owner = '''  bool pop_next_record_done(exo::RecordDoneMessage &out) {\n    if (queued_done_count_ == 0U) {'''
    new_owner = '''  bool pop_next_record_done(exo::RecordDoneMessage &out) {\n    if (active_source_id_ != 0U) {\n      return false;\n    }\n    if (queued_done_count_ == 0U) {'''
    if hub.count(old_owner) != 1:
        raise RuntimeError('active-owner guard anchor not found')
    hub = hub.replace(old_owner, new_owner, 1)

    peek = '''  bool peek_next_live_sample(LiveSample &out) const {\n    const int8_t selected = select_next_live_index_();\n    if (selected < 0) return false;\n    out = live_slots_[static_cast<uint8_t>(selected)].sample;\n    return true;\n  }\n'''
    timed = peek + '''\n  bool peek_next_live_sample(LiveSample &out, uint32_t now_ms) const {\n    return live_preview_due_(now_ms) && peek_next_live_sample(out);\n  }\n\n  void on_live_sample_send_result(bool sent, uint32_t now_ms) {\n    if (sent) {\n      (void)discard_next_live_sample();\n      if (live_preview_congested_ &&\n          static_cast<uint32_t>(now_ms - last_live_failure_ms_) >= kCongestionRecoveryMs) {\n        live_preview_congested_ = false;\n      }\n      next_live_attempt_ms_ = now_ms +\n          (live_preview_congested_ ? kCongestedPreviewIntervalMs : kNormalPreviewIntervalMs);\n      return;\n    }\n    const int8_t selected = select_next_live_index_();\n    if (selected >= 0) {\n      const uint8_t source = live_slots_[static_cast<uint8_t>(selected)].sample.node_id;\n      if (source >= 1U && source <= kMaxLeaves) {\n        next_preview_source_ = source == kMaxLeaves ? 1U : static_cast<uint8_t>(source + 1U);\n      }\n    }\n    selected_live_index_ = kNoLiveSelection;\n    live_preview_congested_ = true;\n    last_live_failure_ms_ = now_ms;\n    next_live_attempt_ms_ = now_ms + kCongestedPreviewIntervalMs;\n  }\n\n  bool live_preview_congested() const { return live_preview_congested_; }\n'''
    if hub.count(peek) != 1:
        raise RuntimeError('live preview method anchor not found')
    hub = hub.replace(peek, timed, 1)
    hub = hub.replace(
        '  static constexpr int8_t kNoLiveSelection = -1;\n',
        '''  static constexpr int8_t kNoLiveSelection = -1;\n  static constexpr uint32_t kNormalPreviewIntervalMs = 40U;\n  static constexpr uint32_t kCongestedPreviewIntervalMs = 80U;\n  static constexpr uint32_t kCongestionRecoveryMs = 5000U;\n''', 1)
    hub = hub.replace(
        '  bool owns_transfer_(uint32_t session_id, uint16_t source_id) const {\n',
        '''  bool live_preview_due_(uint32_t now_ms) const {\n    return next_live_attempt_ms_ == 0U ||\n        static_cast<int32_t>(now_ms - next_live_attempt_ms_) >= 0;\n  }\n\n  bool owns_transfer_(uint32_t session_id, uint16_t source_id) const {\n''', 1)
    reset_old = '''    pending_live_count_ = 0U;\n    next_preview_source_ = 1U;\n    selected_live_index_ = kNoLiveSelection;\n    active_source_id_ = 0U;'''
    reset_new = '''    pending_live_count_ = 0U;\n    next_preview_source_ = 1U;\n    selected_live_index_ = kNoLiveSelection;\n    next_live_attempt_ms_ = 0U;\n    last_live_failure_ms_ = 0U;\n    live_preview_congested_ = false;\n    active_source_id_ = 0U;'''
    if hub.count(reset_old) != 1:
        raise RuntimeError('live-preview reset anchor not found')
    hub = hub.replace(reset_old, reset_new, 1)
    fields_old = '''  mutable int8_t selected_live_index_ = kNoLiveSelection;\n  bool start_or_record_active_ = false;'''
    fields_new = '''  mutable int8_t selected_live_index_ = kNoLiveSelection;\n  uint32_t next_live_attempt_ms_ = 0U;\n  uint32_t last_live_failure_ms_ = 0U;\n  bool live_preview_congested_ = false;\n  bool start_or_record_active_ = false;'''
    if hub.count(fields_old) != 1:
        raise RuntimeError('live-preview field anchor not found')
    hub_path.write_text(hub.replace(fields_old, fields_new, 1))

    main_path = WORK / 'Firmware/Master/Core/Src/main.c'
    main = main_path.read_text()
    pattern = re.compile(
        r'(\tstatic void drain_leaf_stream_passthrough\(\)\n\t\{\n)(.*?)(\n\t\}\n\n\tstatic bool is_duplicate_start_record)',
        re.S)
    match = pattern.search(main)
    if match is None:
        raise RuntimeError('drain_leaf_stream_passthrough not found')
    manager_match = re.search(
        r'if \(!([A-Za-z_][A-Za-z0-9_]*)\.peek_next_live_sample',
        match.group(2))
    if manager_match is None:
        raise RuntimeError('leaf manager variable not found')
    manager = manager_match.group(1)
    replacement = f'''\t\texo::ble_hub::HubLeafBleManager::LiveSample sample {{ }};\n\t\tconst uint32_t now_ms = HAL_GetTick();\n\t\tif (!{manager}.peek_next_live_sample(sample, now_ms)) {{\n\t\t\treturn;\n\t\t}}\n\t\tif (!g_ble_stream_enabled || g_ble_record_transfer_mode) {{\n\t\t\t(void){manager}.discard_next_live_sample();\n\t\t\treturn;\n\t\t}}\n\t\tif (!send_ble_v2_sample(sample.node_id,\n\t\t\t\tsample.sensor_id,\n\t\t\t\tsample.payload,\n\t\t\t\tsample.payload_len)) {{\n\t\t\t{manager}.on_live_sample_send_result(false, now_ms);\n\t\t\treturn;\n\t\t}}\n\t\t{manager}.on_live_sample_send_result(true, now_ms);'''
    main_path.write_text(main[:match.start(2)] + replacement + main[match.end(2):])

    replace_once(
        'Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_FORMATTER.h',
        '''inline bool append_double_or_blank(CsvRowWriter &writer, double value)\n{\n    return !finite_value(value) ? true : append_double(writer, value);\n}''',
        '''inline bool append_double_or_blank(CsvRowWriter &writer, double value)\n{\n    if (!finite_value(value) || value > 1000000.0 || value < -1000000.0) return true;\n    return append_double(writer, value);\n}''')

    replace_once(
        'Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h',
        '''        for (uint8_t source = 0U; source < kSourceCount; ++source) {\n            if ((expected_source_mask_ & static_cast<uint8_t>(1U << source)) != 0U) {\n                metadata_valid_[source][0] = true;\n                metadata_valid_[source][1] = true;\n            }\n        }\n''',
        '''        // Every source, including the Master, must register its validated\n        // SessionHeader before rows are accepted. reset_runtime() leaves all\n        // metadata validity flags clear.\n''')

    replace_once(
        'Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h',
        '''        const FRESULT rename_result = ops_->rename_fn(path_, csv_path_);\n        if (rename_result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::Rename, rename_result);\n        memcpy(path_, csv_path_, sizeof(path_));\n        published_ = true;\n        FIL marker{};\n        FRESULT result = ops_->open_fn(&marker, ok_path_, static_cast<BYTE>(FA_CREATE_NEW | FA_WRITE));\n        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerOpen, result);\n        result = ops_->sync_fn(&marker);\n        if (result != FR_OK) { (void)ops_->close_fn(&marker); return set_terminal(training_csv::TrainingCsvLogOperation::MarkerSync, result); }\n        result = ops_->close_fn(&marker);\n        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, result);\n        return true;''',
        '''        const bool already_renamed = strcmp(path_, csv_path_) == 0;\n        if (!already_renamed) {\n            const FRESULT rename_result = ops_->rename_fn(path_, csv_path_);\n            if (rename_result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::Rename, rename_result);\n            memcpy(path_, csv_path_, sizeof(path_));\n        }\n        FIL marker{};\n        const BYTE marker_mode = static_cast<BYTE>((already_renamed ? FA_OPEN_ALWAYS : FA_CREATE_NEW) | FA_WRITE);\n        FRESULT result = ops_->open_fn(&marker, ok_path_, marker_mode);\n        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerOpen, result);\n        result = ops_->sync_fn(&marker);\n        if (result != FR_OK) { (void)ops_->close_fn(&marker); return set_terminal(training_csv::TrainingCsvLogOperation::MarkerSync, result); }\n        result = ops_->close_fn(&marker);\n        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, result);\n        published_ = true;\n        return true;''')

    replace_once(
        'Docs/FourNode_Live_Csv_Validation.md',
        '''1. Wait until one Node upload is active.\n2. Disconnect or close the browser.\n3. Leave the Master and Nodes powered.\n4. Reconnect the browser after several seconds.\n5. Confirm the Master resumes from its next expected chunk rather than restarting the complete session or discarding validated data.''',
        '''1. Wait until one Node upload is active, then record the active Node ID and next expected chunk.\n2. Disconnect or close the browser.\n3. Leave the Master and Nodes powered while the browser remains disconnected.\n4. Before reconnecting, confirm from the Master diagnostics that the next expected chunk advanced or that the active Node moved into the validated state.\n5. Reconnect the browser after several seconds.\n6. Confirm the Master continues from the advanced transfer state rather than restarting the complete session or discarding validated data.''')

    replace_once(
        'Docs/Superpowers/Plans/2026-08-03-four-node-live-preview-training-csv-implementation.md',
        'Use normalized quaternion `(x,y,z,w)` with right-handed intrinsic XYZ / roll-pitch-yaw formulas. Add Euler degrees and BNO/ICM vector magnitudes.',
        'Use normalized quaternion `(x,y,z,w)` with a right-handed intrinsic Z-Y-X yaw-pitch-roll convention, reported as roll, pitch, and yaw. Add Euler degrees and BNO/ICM vector magnitudes.')

    replace_once(
        'Docs/Superpowers/Specs/2026-08-03-four-node-live-preview-training-csv-design.md',
        '- `source_payload_crc32`\n\n### BNO85 raw and calibrated columns',
        '- `source_payload_crc32`\n- `timestamp_quality_flags`\n\n### BNO85 raw and calibrated columns')


def run_green_suite() -> None:
    tests = [
        'Firmware/HostTests/test_coderabbit_firmware_regressions.cpp',
        'Firmware/HostTests/test_master_node_transfer_window.cpp',
        'Firmware/HostTests/test_master_node_reliable_control.cpp',
        'Firmware/HostTests/test_hub_leaf_ble_manager.cpp',
        'Firmware/HostTests/test_master_training_csv_formatter_v2.cpp',
        'Firmware/HostTests/test_node_live_sample_queue.cpp',
    ]
    for source in tests:
        executable = compile_test(source)
        run(str(executable), cwd=WORK)

    ff_dir = ff_include_dir()
    for source in [
        'Firmware/HostTests/test_coderabbit_csv_logger_regressions.cpp',
        'Firmware/HostTests/test_master_training_csv_logger_v2.cpp',
    ]:
        executable = compile_test(source, extra=['-I', str(ff_dir)])
        run(str(executable), cwd=WORK)

    python_files = [str(path.relative_to(WORK)) for path in
                    sorted((WORK / 'Firmware/HostTests').glob('*.py'))]
    if python_files:
        run('python3', '-m', 'py_compile', *python_files, cwd=WORK)
    run('python3', 'Firmware/HostTests/test_ble_only_cleanup.py', cwd=WORK)
    run(
        'g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
        '-DEXO_ACQ_DIAG_HOST_TEST=1', '-I', 'Firmware/LIBRARY/CUSTOM',
        'Firmware/Master/tests/acquisition_diagnostics_test.cpp',
        '-o', '/tmp/acquisition_diagnostics_test', cwd=WORK)
    run('/tmp/acquisition_diagnostics_test', cwd=WORK)
    run('git', 'diff', '--check', cwd=WORK)


def remove_transport_machinery() -> None:
    shutil.rmtree(WORK / '.cleanup-patch', ignore_errors=True)
    for relative in [
        '.github/workflows/apply-ble-cleanup.yml',
        '.github/workflows/apply-ble-cleanup-repair.yml',
        '.github/workflows/apply-coderabbit-firmware-fixes.yml',
        '.github/workflows/run-validated-firmware-cleanup.yml',
        '.github/scripts/apply_validated_firmware_cleanup.py',
    ]:
        target = WORK / relative
        if target.exists():
            target.unlink()
    scripts = WORK / '.github/scripts'
    if scripts.exists() and not any(scripts.iterdir()):
        scripts.rmdir()
    workflows = WORK / '.github/workflows'
    if workflows.exists() and not any(workflows.iterdir()):
        workflows.rmdir()


def commit_and_push(current_head: str) -> str:
    run('git', 'add', '-A', cwd=WORK)
    tree = output('git', 'write-tree', cwd=WORK)
    env = os.environ.copy()
    env.update({
        'GIT_AUTHOR_NAME': 'github-actions[bot]',
        'GIT_AUTHOR_EMAIL': '41898282+github-actions[bot]@users.noreply.github.com',
        'GIT_COMMITTER_NAME': 'github-actions[bot]',
        'GIT_COMMITTER_EMAIL': '41898282+github-actions[bot]@users.noreply.github.com',
    })
    result = subprocess.run(
        ['git', 'commit-tree', tree, '-p', current_head],
        cwd=WORK,
        env=env,
        input='fix: harden BLE transfer and CSV cleanup\n',
        text=True,
        check=True,
        stdout=subprocess.PIPE,
    )
    commit = result.stdout.strip()
    run('git', 'push', 'origin', f'{commit}:refs/heads/ble-only-firmware-cleanup', cwd=WORK)
    return commit


def main() -> int:
    current_head = output('git', 'rev-parse', 'HEAD')
    if output('git', 'branch', '--show-current') != 'ble-only-firmware-cleanup':
        raise RuntimeError('executor must run on ble-only-firmware-cleanup')
    prepare_worktree(current_head)
    patch_path = reconstruct_cleanup_patch()
    apply_cleanup_patch(patch_path)
    add_regression_tests()
    verify_red()
    apply_confirmed_fixes()
    run_green_suite()
    remove_transport_machinery()
    run('git', 'diff', '--check', cwd=WORK)
    commit = commit_and_push(current_head)
    print(f'published validated cleanup commit {commit}', flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr, flush=True)
        raise
