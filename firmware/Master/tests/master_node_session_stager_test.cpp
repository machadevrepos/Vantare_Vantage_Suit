#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <exo/storage/master_node_session_stager.h>

namespace {

struct FakeFsState {
    uint8_t bytes[1024]{};
    uint32_t size = 0U;
    uint32_t cursor = 0U;
    BYTE open_mode = 0U;
    bool open = false;
    bool unlinked = false;
    bool fail_close = false;
    unsigned open_calls = 0U;
    unsigned close_calls = 0U;
    unsigned unlink_calls = 0U;
    unsigned read_calls = 0U;
    UINT largest_read = 0U;
    char opened_path[32]{};
} g_fs;

FRESULT fake_open(FIL *, const TCHAR *path, BYTE mode)
{
    ++g_fs.open_calls;
    g_fs.open_mode = mode;
    g_fs.open = true;
    g_fs.cursor = 0U;
    strncpy(g_fs.opened_path, path, sizeof(g_fs.opened_path) - 1U);
    if ((mode & FA_CREATE_ALWAYS) != 0U) {
        g_fs.size = 0U;
        g_fs.unlinked = false;
    }
    return FR_OK;
}

FRESULT fake_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    if (!g_fs.open || g_fs.cursor + bytes > sizeof(g_fs.bytes)) {
        *written = 0U;
        return FR_DISK_ERR;
    }
    memcpy(&g_fs.bytes[g_fs.cursor], data, bytes);
    g_fs.cursor += bytes;
    if (g_fs.cursor > g_fs.size) {
        g_fs.size = g_fs.cursor;
    }
    *written = bytes;
    return FR_OK;
}

FRESULT fake_read(FIL *, void *data, UINT bytes, UINT *read)
{
	++g_fs.read_calls;
    if (bytes > g_fs.largest_read) {
        g_fs.largest_read = bytes;
    }
    if (!g_fs.open || g_fs.cursor > g_fs.size) {
        *read = 0U;
        return FR_DISK_ERR;
    }
    const uint32_t available = g_fs.size - g_fs.cursor;
    const UINT copied = bytes < available ? bytes : static_cast<UINT>(available);
    memcpy(data, &g_fs.bytes[g_fs.cursor], copied);
    g_fs.cursor += copied;
    *read = copied;
    return FR_OK;
}

FRESULT fake_lseek(FIL *, FSIZE_t offset)
{
    if (!g_fs.open || offset > sizeof(g_fs.bytes)) {
        return FR_DISK_ERR;
    }
    g_fs.cursor = static_cast<uint32_t>(offset);
    return FR_OK;
}

FRESULT fake_close(FIL *)
{
    ++g_fs.close_calls;
    if (g_fs.fail_close) {
        return FR_DISK_ERR;
    }
    g_fs.open = false;
    return FR_OK;
}

FRESULT fake_unlink(const TCHAR *)
{
    ++g_fs.unlink_calls;
    g_fs.unlinked = true;
    g_fs.size = 0U;
    return FR_OK;
}

const exo::node_session_staging::NodeSessionFatFsOps kFakeOps = {
    fake_open, fake_write, fake_read, fake_lseek, fake_close, fake_unlink
};

void reset_fs()
{
    g_fs = FakeFsState{};
}

struct SessionImage {
    uint8_t bytes[sizeof(exo::SessionHeader) + sizeof(exo::Bno85Sample) +
            (2U * sizeof(exo::Icm45686Sample))]{};
    exo::RecordDoneMessage done{};
    exo::Bno85Sample bno{};
    exo::Icm45686Sample icm[2]{};
};

SessionImage make_session()
{
    SessionImage image{};
    image.bno.offset_us = 12345U;
    image.bno.quat_real = 0.75F;
    image.bno.linear_accel_x = -1.25F;
    image.icm[0].accel_x = 101;
    image.icm[0].gyro_z = -202;
    image.icm[1].accel_y = 303;
    image.icm[1].gyro_x = -404;

    exo::SessionHeader header{};
    header.magic = exo::kSessionMagic;
    header.version = exo::kSessionFormatVersion;
    header.node_id = 2U;
    header.session_id = 77U;
    header.completion_flag = exo::kSessionComplete;
    header.bno85_sample_count = 1U;
    header.icm45686_sample_count = 2U;
    header.bno85_attempted_count = 1U;
    header.icm45686_attempted_count = 2U;
    header.bno85_captured_count = 1U;
    header.icm45686_captured_count = 2U;
    header.bno85_payload_size = sizeof(exo::Bno85Sample);
    header.icm45686_payload_size = 2U * sizeof(exo::Icm45686Sample);
    memcpy(image.bytes, &header, sizeof(header));
    memcpy(&image.bytes[sizeof(header)], &image.bno, sizeof(image.bno));
    memcpy(&image.bytes[sizeof(header) + sizeof(image.bno)], image.icm, sizeof(image.icm));
    header.payload_crc32 = exo::crc32(&image.bytes[sizeof(header)],
            sizeof(image.bytes) - sizeof(header));
    header.header_crc32 = exo::session_header_crc(header);
    memcpy(image.bytes, &header, sizeof(header));

    image.done.command = exo::RecordCommand::RecordDone;
    image.done.node_id = 2U;
    image.done.session_id = 77U;
    image.done.total_size = sizeof(image.bytes);
    image.done.payload_crc32 = header.payload_crc32;
    return image;
}

bool stage_all(exo::MasterNodeSessionStager &stager, const SessionImage &image)
{
    const uint16_t first = 37U;
    return stager.begin(image.done) &&
            stager.accept_chunk(2U, 77U, 0U, image.bytes, first) &&
            stager.accept_chunk(2U, 77U, first, &image.bytes[first],
                    static_cast<uint16_t>(image.done.total_size - first));
}

bool validate_incrementally(exo::MasterNodeSessionStager &stager, uint32_t byte_budget)
{
    if (!stager.begin_validation() || stager.validation_remaining() == 0U ||
            stager.validated()) {
        return false;
    }
    while (stager.validation_remaining() > 0U) {
        const uint32_t before = stager.validation_remaining();
        const unsigned reads_before = g_fs.read_calls;
        g_fs.largest_read = 0U;
        if (!stager.step_validation(byte_budget) ||
                before - stager.validation_remaining() > byte_budget ||
                g_fs.read_calls != reads_before + 1U ||
                g_fs.largest_read > byte_budget || stager.validated()) {
            return false;
        }
    }
    return stager.finalize_validation() && stager.validated() &&
            stager.validation_status() ==
                    exo::node_session_staging::NodeSessionValidationStatus::Complete;
}

bool validation_rejects(const SessionImage &image)
{
    reset_fs();
    exo::MasterNodeSessionStager stager(&kFakeOps);
    if (!stage_all(stager, image)) {
        return false;
    }
    if (stager.begin_validation()) {
        while (stager.validation_remaining() > 0U && stager.step_validation(64U)) {
        }
        if (stager.validation_remaining() == 0U && stager.finalize_validation()) {
            return false;
        }
    }
    return !stager.validated() && !stager.node_flash_may_be_erased() && !g_fs.open &&
            g_fs.close_calls == 2U && stager.begin(make_session().done);
}

} // namespace

int main()
{
    SessionImage image = make_session();

    /* A complete, CRC-valid capture with missed scheduled BNO ticks is degraded
     * quality, not a transfer-integrity failure. Removing this acceptance would
     * discard usable data such as R0001N1.BIN. */
    SessionImage degraded = image;
    exo::SessionHeader degraded_header{};
    memcpy(&degraded_header, degraded.bytes, sizeof(degraded_header));
    degraded_header.loss_flags = exo::kSessionLossBnoWrite;
    degraded_header.bno85_attempted_count = 1385U;
    degraded_header.bno85_captured_count = 269U;
    degraded_header.bno85_dropped_count = 1116U;
    degraded_header.icm45686_captured_count = 17U;
    degraded_header.icm45686_dropped_count = 3U;
    degraded_header.header_crc32 = exo::session_header_crc(degraded_header);
    memcpy(degraded.bytes, &degraded_header, sizeof(degraded_header));
    reset_fs();
    exo::MasterNodeSessionStager degraded_stager(&kFakeOps);
    if (!stage_all(degraded_stager, degraded) ||
            !validate_incrementally(degraded_stager, 64U)) {
        return 26;
    }

    reset_fs();
    exo::MasterNodeSessionStager stager(&kFakeOps);
    if (!stager.begin(image.done) || strcmp(g_fs.opened_path, "/SESSIONS/R0001N2.BIN") != 0 ||
            g_fs.open_mode != static_cast<BYTE>(FA_CREATE_ALWAYS | FA_WRITE | FA_READ) ||
            stager.staged_size() != 0U) {
        return 1;
    }
    if (!stager.accept_chunk(2U, 77U, 0U, image.bytes, 37U) ||
            !stager.accept_chunk(2U, 77U, 37U, &image.bytes[37], 20U) ||
            stager.staged_size() != 57U) {
        return 2;
    }
    if (!stager.accept_chunk(2U, 77U, 0U, image.bytes, 37U) || stager.staged_size() != 57U) {
        return 3;
    }
    uint8_t different[37]{};
    memcpy(different, image.bytes, sizeof(different));
    different[3] ^= 0x55U;
    if (stager.accept_chunk(2U, 77U, 0U, different, sizeof(different)) ||
            stager.accept_chunk(2U, 77U, 56U, &image.bytes[56], 2U) ||
            stager.accept_chunk(2U, 77U, 60U, &image.bytes[60], 2U) ||
            stager.accept_chunk(3U, 77U, 57U, &image.bytes[57], 1U) ||
            stager.accept_chunk(2U, 78U, 57U, &image.bytes[57], 1U)) {
        return 4;
    }
    if (!stager.accept_chunk(2U, 77U, 57U, &image.bytes[57],
                static_cast<uint16_t>(sizeof(image.bytes) - 57U)) ||
            !validate_incrementally(stager, 64U) || !stager.validated() ||
            stager.node_flash_may_be_erased() || g_fs.largest_read > 256U) {
        return 5;
    }
    exo::Bno85Sample bno{};
    exo::Icm45686Sample icm{};
    if (!stager.read_bno(0U, bno) || bno.offset_us != image.bno.offset_us ||
            bno.quat_real != image.bno.quat_real || stager.read_bno(1U, bno) ||
            !stager.read_icm(1U, icm) || icm.accel_y != image.icm[1].accel_y ||
            icm.gyro_x != image.icm[1].gyro_x || stager.read_icm(2U, icm)) {
        return 6;
    }
    if (!stager.discard_after_success() || g_fs.unlinked || g_fs.open ||
            !stager.node_flash_may_be_erased() || g_fs.close_calls != 2U ||
            g_fs.unlink_calls != 0U) {
        return 7;
    }

    reset_fs();
    exo::MasterNodeSessionStager wrong_node(&kFakeOps);
    exo::RecordDoneMessage wrong_done = image.done;
    wrong_done.node_id = 0U;
    if (wrong_node.begin(wrong_done)) {
        return 8;
    }
    wrong_done.node_id = 5U;
    if (wrong_node.begin(wrong_done)) {
        return 9;
    }

    reset_fs();
    exo::MasterNodeSessionStager interrupted(&kFakeOps);
    if (!interrupted.begin(image.done) ||
            !interrupted.accept_chunk(2U, 77U, 0U, image.bytes, 37U)) {
        return 17;
    }
    g_fs.fail_close = true;
    if (interrupted.shutdown() || !g_fs.open || !interrupted.active() ||
            interrupted.node_flash_may_be_erased() || g_fs.unlink_calls != 0U) {
        return 18;
    }
    g_fs.fail_close = false;
    if (!interrupted.shutdown() || g_fs.open || interrupted.active() ||
            interrupted.validated() || interrupted.node_flash_may_be_erased() ||
            g_fs.unlink_calls != 0U || !interrupted.begin(image.done)) {
        return 19;
    }
    if (!interrupted.shutdown()) {
        return 20;
    }

    SessionImage corrupt = make_session();
    exo::SessionHeader header{};
    memcpy(&header, corrupt.bytes, sizeof(header));
    header.header_crc32 ^= 1U;
    memcpy(corrupt.bytes, &header, sizeof(header));
    if (!validation_rejects(corrupt)) {
        return 10;
    }

    corrupt = make_session();
    corrupt.bytes[sizeof(exo::SessionHeader)] ^= 1U;
    if (!validation_rejects(corrupt)) {
        return 11;
    }

    corrupt = make_session();
    --corrupt.done.total_size;
    if (!validation_rejects(corrupt)) {
        return 12;
    }

    corrupt = make_session();
    memcpy(&header, corrupt.bytes, sizeof(header));
    ++header.icm45686_sample_count;
    header.header_crc32 = exo::session_header_crc(header);
    memcpy(corrupt.bytes, &header, sizeof(header));
    if (!validation_rejects(corrupt)) {
        return 13;
    }

    reset_fs();
    exo::MasterNodeSessionStager cleanup_failure(&kFakeOps);
    if (!stage_all(cleanup_failure, image) || !validate_incrementally(cleanup_failure, 64U)) {
        return 14;
    }
    g_fs.fail_close = true;
    if (cleanup_failure.discard_after_success() || cleanup_failure.node_flash_may_be_erased() ||
            g_fs.unlink_calls != 0U) {
        return 15;
    }
    g_fs.fail_close = false;
    if (!cleanup_failure.discard_after_success() || !cleanup_failure.node_flash_may_be_erased()) {
        return 16;
    }

    return 0;
}
