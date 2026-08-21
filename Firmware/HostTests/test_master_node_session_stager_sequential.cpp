#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "MASTER_NODE_SESSION_STAGER.h"

namespace {
std::vector<uint8_t> g_file;
uint32_t g_cursor = 0U;
uint32_t g_lseek_count = 0U;
uint32_t g_unlink_count = 0U;

FRESULT seq_open(FIL *, const TCHAR *, BYTE mode)
{
    g_cursor = 0U;
    if ((mode & FA_CREATE_ALWAYS) != 0U) {
        g_file.clear();
    }
    return FR_OK;
}

FRESULT seq_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    if (g_file.size() < static_cast<size_t>(g_cursor) + bytes) {
        g_file.resize(static_cast<size_t>(g_cursor) + bytes);
    }
    memcpy(g_file.data() + g_cursor, data, bytes);
    g_cursor += bytes;
    *written = bytes;
    return FR_OK;
}

FRESULT seq_read(FIL *, void *data, UINT bytes, UINT *received)
{
    if (static_cast<size_t>(g_cursor) + bytes > g_file.size()) {
        *received = 0U;
        return FR_INVALID_OBJECT;
    }
    memcpy(data, g_file.data() + g_cursor, bytes);
    g_cursor += bytes;
    *received = bytes;
    return FR_OK;
}

FRESULT seq_lseek(FIL *, FSIZE_t offset)
{
    ++g_lseek_count;
    if (offset > g_file.size()) {
        return FR_INVALID_PARAMETER;
    }
    g_cursor = static_cast<uint32_t>(offset);
    return FR_OK;
}

FRESULT seq_close(FIL *)
{
    return FR_OK;
}

FRESULT seq_unlink(const TCHAR *)
{
    ++g_unlink_count;
    return FR_OK;
}

const exo::node_session_staging::NodeSessionFatFsOps kSeqOps = {
    seq_open, seq_write, seq_read, seq_lseek, seq_close, seq_unlink
};
} // namespace

int main()
{
    constexpr uint32_t bno_count = 3U;
    constexpr uint32_t icm_count = 4U;

    exo::SessionHeader header{};
    header.magic = exo::kSessionMagic;
    header.version = exo::kSessionFormatVersion;
    header.node_id = 1U;
    header.session_id = 42U;
    header.completion_flag = exo::kSessionComplete;
    header.bno85_sample_count = bno_count;
    header.icm45686_sample_count = icm_count;
    header.bno85_payload_size = bno_count * sizeof(exo::Bno85Sample);
    header.icm45686_payload_size = icm_count * sizeof(exo::Icm45686Sample);

    std::vector<uint8_t> payload(header.bno85_payload_size + header.icm45686_payload_size);
    for (size_t i = 0U; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i + 1U);
    }
    header.payload_crc32 = exo::crc32(payload.data(), payload.size());
    header.header_crc32 = exo::session_header_crc(header);

    std::vector<uint8_t> session(sizeof(header) + payload.size());
    memcpy(session.data(), &header, sizeof(header));
    memcpy(session.data() + sizeof(header), payload.data(), payload.size());

    exo::RecordDoneMessage done{};
    done.command = exo::RecordCommand::RecordDone;
    done.node_id = 1U;
    done.session_id = 42U;
    done.total_size = static_cast<uint32_t>(session.size());
    done.payload_crc32 = header.payload_crc32;

    exo::MasterNodeSessionStager stager(&kSeqOps);
    assert(stager.begin(done, 1U));
    assert(stager.accept_chunk(1U, 42U, 0U, session.data(),
            static_cast<uint16_t>(session.size())));
    assert(stager.begin_validation());
    while (stager.validation_status() ==
            exo::node_session_staging::NodeSessionValidationStatus::InProgress) {
        assert(stager.step_validation(256U));
    }
    assert(stager.finalize_validation());

    g_lseek_count = 0U;
    exo::Bno85Sample bno{};
    exo::Icm45686Sample icm{};
    for (uint32_t i = 0U; i < bno_count; ++i) {
        assert(stager.read_bno(i, bno));
    }
    for (uint32_t i = 0U; i < icm_count; ++i) {
        assert(stager.read_icm(i, icm));
    }

    // One seek positions the first sample. BNO and ICM records are contiguous,
    // so every later row is read without another FAT filesystem seek.
    assert(g_lseek_count == 1U);

    // Buffered-write scenario: many small appends must cross the internal
    // write-block boundary, a duplicate may straddle flushed prefix vs RAM
    // tail, gaps stay rejected, and shutdown must flush any pending tail.
    constexpr uint32_t big_bno_count = 100U;
    constexpr uint32_t big_icm_count = 170U;
    exo::SessionHeader big_header{};
    big_header.magic = exo::kSessionMagic;
    big_header.version = exo::kSessionFormatVersion;
    big_header.node_id = 2U;
    big_header.session_id = 77U;
    big_header.completion_flag = exo::kSessionComplete;
    big_header.bno85_sample_count = big_bno_count;
    big_header.icm45686_sample_count = big_icm_count;
    big_header.bno85_payload_size = big_bno_count * sizeof(exo::Bno85Sample);
    big_header.icm45686_payload_size = big_icm_count * sizeof(exo::Icm45686Sample);

    std::vector<uint8_t> big_payload(
            big_header.bno85_payload_size + big_header.icm45686_payload_size);
    for (size_t i = 0U; i < big_payload.size(); ++i) {
        big_payload[i] = static_cast<uint8_t>(i * 7U + 3U);
    }
    big_header.payload_crc32 = exo::crc32(big_payload.data(), big_payload.size());
    big_header.header_crc32 = exo::session_header_crc(big_header);

    std::vector<uint8_t> big_session(sizeof(big_header) + big_payload.size());
    memcpy(big_session.data(), &big_header, sizeof(big_header));
    memcpy(big_session.data() + sizeof(big_header), big_payload.data(), big_payload.size());

    exo::RecordDoneMessage big_done{};
    big_done.command = exo::RecordCommand::RecordDone;
    big_done.node_id = 2U;
    big_done.session_id = 77U;
    big_done.total_size = static_cast<uint32_t>(big_session.size());
    big_done.payload_crc32 = big_header.payload_crc32;

    g_file.clear();
    g_cursor = 0U;
    exo::MasterNodeSessionStager buffered(&kSeqOps);
    assert(buffered.begin(big_done, 2U));
    uint32_t append_offset = 0U;
    while (append_offset < big_session.size()) {
        const size_t left = big_session.size() - append_offset;
        const uint16_t take = left < 180U ? static_cast<uint16_t>(left) : 180U;
        assert(buffered.accept_chunk(2U, 77U, append_offset,
                big_session.data() + append_offset, take));
        append_offset += take;
    }
    assert(buffered.staged_size() == big_session.size());
    // At least one full write block reached the card before validation, i.e.
    // appends were amortized instead of written per chunk.
    assert(g_file.size() >= 4096U);

    // Duplicate whose range starts in the flushed prefix and ends inside the
    // RAM tail buffer.
    const uint32_t straddle_offset =
            static_cast<uint32_t>(big_session.size()) - 100U;
    assert(buffered.accept_chunk(2U, 77U, straddle_offset,
            big_session.data() + straddle_offset, 100U));

    // Duplicate entirely inside the flushed prefix while the RAM tail holds
    // data: read-back must compare exactly the caller's length, no more.
    assert(buffered.accept_chunk(2U, 77U, 4000U,
            big_session.data() + 4000U, 180U));

    // A gap beyond the staged end must still be rejected.
    assert(!buffered.accept_chunk(2U, 77U,
            static_cast<uint32_t>(big_session.size()) + 1U,
            big_session.data(), 1U));

    assert(buffered.begin_validation());
    while (buffered.validation_status() ==
            exo::node_session_staging::NodeSessionValidationStatus::InProgress) {
        assert(buffered.step_validation(256U));
    }
    assert(buffered.finalize_validation());

    // Abandon path: shutdown must flush the short tail so the staged file
    // keeps every byte that was acknowledged.
    g_file.clear();
    g_cursor = 0U;
    exo::MasterNodeSessionStager abandoned(&kSeqOps);
    assert(abandoned.begin(big_done, 3U));
    assert(abandoned.accept_chunk(2U, 77U, 0U, big_session.data(), 200U));
    assert(abandoned.shutdown());
    assert(g_file.size() == 200U);
    assert(memcmp(g_file.data(), big_session.data(), 200U) == 0);

    // Abandon-and-unlink path: a truncated stage must be removed so the run
    // index stays reusable, while the node's flash copy stays retained.
    g_unlink_count = 0U;
    g_file.clear();
    g_cursor = 0U;
    exo::MasterNodeSessionStager discarded_early(&kSeqOps);
    assert(discarded_early.begin(big_done, 4U));
    assert(discarded_early.accept_chunk(2U, 77U, 0U, big_session.data(), 200U));
    assert(discarded_early.abandon_and_unlink());
    assert(g_unlink_count == 1U);
    assert(!discarded_early.node_flash_may_be_erased());

    // A validated stage is the durable archive: even after the coordinator
    // shuts down (e.g. session reset), abandon_and_unlink must not remove it.
    g_unlink_count = 0U;
    g_file.clear();
    g_cursor = 0U;
    exo::MasterNodeSessionStager validated_stage(&kSeqOps);
    assert(validated_stage.begin(done, 5U));
    assert(validated_stage.accept_chunk(1U, 42U, 0U, session.data(),
            static_cast<uint16_t>(session.size())));
    assert(validated_stage.begin_validation());
    while (validated_stage.validation_status() ==
            exo::node_session_staging::NodeSessionValidationStatus::InProgress) {
        assert(validated_stage.step_validation(256U));
    }
    assert(validated_stage.finalize_validation());
    assert(validated_stage.discard_after_success());
    assert(validated_stage.node_flash_may_be_erased());
    assert(validated_stage.abandon_and_unlink());
    assert(g_unlink_count == 0U);

    // A stager that never staged anything must not unlink stray files.
    g_unlink_count = 0U;
    exo::MasterNodeSessionStager idle(&kSeqOps);
    assert(idle.abandon_and_unlink());
    assert(g_unlink_count == 0U);

    std::cout << "master node stager sequential read tests passed\n";
    return 0;
}
