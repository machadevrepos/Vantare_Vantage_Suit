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
}

extern "C" FRESULT f_open(FIL *, const TCHAR *, BYTE mode)
{
    g_cursor = 0U;
    if ((mode & FA_CREATE_ALWAYS) != 0U) {
        g_file.clear();
    }
    return FR_OK;
}

extern "C" FRESULT f_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    if (g_file.size() < static_cast<size_t>(g_cursor) + bytes) {
        g_file.resize(static_cast<size_t>(g_cursor) + bytes);
    }
    memcpy(g_file.data() + g_cursor, data, bytes);
    g_cursor += bytes;
    *written = bytes;
    return FR_OK;
}

extern "C" FRESULT f_read(FIL *, void *data, UINT bytes, UINT *received)
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

extern "C" FRESULT f_lseek(FIL *, FSIZE_t offset)
{
    ++g_lseek_count;
    if (offset > g_file.size()) {
        return FR_INVALID_PARAMETER;
    }
    g_cursor = static_cast<uint32_t>(offset);
    return FR_OK;
}

extern "C" FRESULT f_close(FIL *)
{
    return FR_OK;
}

extern "C" FRESULT f_unlink(const TCHAR *)
{
    return FR_OK;
}

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

    exo::MasterNodeSessionStager stager;
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

    std::cout << "master node stager sequential read tests passed\n";
    return 0;
}
