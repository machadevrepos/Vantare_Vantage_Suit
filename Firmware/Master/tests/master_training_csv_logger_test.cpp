#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "MASTER_TRAINING_CSV_LOGGER.h"

namespace {

struct FakeFsState {
    uint16_t highest_existing = 1U;
    bool short_write = false;
    bool fail_sync = false;
    FRESULT write_result = FR_OK;
    FRESULT close_result = FR_OK;
    unsigned write_calls = 0U;
    unsigned sync_calls = 0U;
    unsigned close_calls = 0U;
    UINT largest_write = 0U;
    bool inspect_completion_on_sync = false;
    uint8_t completion_mask_during_sync = 0xFFU;
    size_t captured_bytes = 0U;
    char captured[4096]{};
    char opened_path[32]{};
    char renamed_from[32]{};
    char renamed_to[32]{};
    unsigned rename_calls = 0U;
    unsigned marker_open_calls = 0U;
} g_fake;

const exo::MasterTrainingCsvLogger *g_observed_logger = nullptr;

FRESULT fake_mkdir(const TCHAR *) { return FR_EXIST; }

FRESULT fake_stat(const TCHAR *path, FILINFO *)
{
    unsigned index = 0U;
    for (const char *cursor = path; *cursor != '\0'; ++cursor) {
        if (*cursor >= '0' && *cursor <= '9') {
            index = (index * 10U) + static_cast<unsigned>(*cursor - '0');
        }
    }
    return index <= g_fake.highest_existing ? FR_OK : FR_NO_FILE;
}

FRESULT fake_open(FIL *, const TCHAR *path, BYTE mode)
{
    if (mode != static_cast<BYTE>(FA_CREATE_NEW | FA_WRITE)) {
        return FR_INVALID_PARAMETER;
    }
    if (strstr(path, ".OK") != nullptr) {
        ++g_fake.marker_open_calls;
    }
    strncpy(g_fake.opened_path, path, sizeof(g_fake.opened_path) - 1U);
    return FR_OK;
}

FRESULT fake_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    ++g_fake.write_calls;
    if (bytes > g_fake.largest_write) {
        g_fake.largest_write = bytes;
    }
    if (g_fake.write_result != FR_OK) {
        *written = 0U;
        return g_fake.write_result;
    }
    *written = g_fake.short_write && bytes > 0U ? bytes - 1U : bytes;
    const size_t available = sizeof(g_fake.captured) - g_fake.captured_bytes - 1U;
    const size_t copy_bytes = *written < available ? *written : available;
    if (copy_bytes > 0U) {
        memcpy(&g_fake.captured[g_fake.captured_bytes], data, copy_bytes);
        g_fake.captured_bytes += copy_bytes;
        g_fake.captured[g_fake.captured_bytes] = '\0';
    }
    return FR_OK;
}

FRESULT fake_sync(FIL *)
{
    ++g_fake.sync_calls;
    if (g_fake.inspect_completion_on_sync && g_observed_logger != nullptr) {
        g_fake.completion_mask_during_sync = g_observed_logger->completed_source_mask();
    }
    return g_fake.fail_sync ? FR_DISK_ERR : FR_OK;
}

FRESULT fake_close(FIL *)
{
    ++g_fake.close_calls;
    return g_fake.close_result;
}

FRESULT fake_rename(const TCHAR *from, const TCHAR *to)
{
    ++g_fake.rename_calls;
    strncpy(g_fake.renamed_from, from, sizeof(g_fake.renamed_from) - 1U);
    strncpy(g_fake.renamed_to, to, sizeof(g_fake.renamed_to) - 1U);
    return FR_OK;
}

const exo::training_csv::TrainingCsvFatFsOps kFakeOps = {
    fake_mkdir, fake_stat, fake_open, fake_write, fake_sync, fake_close, fake_rename
};

void reset_fake(uint16_t highest_existing = 0U)
{
    g_fake = FakeFsState{};
    g_fake.highest_existing = highest_existing;
}

} // namespace

int main()
{
    reset_fake(1U);
    exo::MasterTrainingCsvLogger logger(&kFakeOps);
    if (!logger.begin(77U, 0x1FU, 10U) ||
            strcmp(logger.path(), "/SESSIONS/TRN0002.TMP") != 0 ||
            logger.session_id() != 77U || logger.expected_source_mask() != 0x1FU ||
            g_fake.write_calls != 1U || g_fake.sync_calls != 1U) {
        return 1;
    }
    const unsigned writes_before_rebegin = g_fake.write_calls;
    const unsigned syncs_before_rebegin = g_fake.sync_calls;
    const unsigned closes_before_rebegin = g_fake.close_calls;
    if (logger.begin(88U, 0x01U, 10U) ||
            logger.last_operation() != exo::training_csv::TrainingCsvLogOperation::AlreadyStarted ||
            logger.session_id() != 77U || strcmp(logger.path(), "/SESSIONS/TRN0002.TMP") != 0 ||
            !logger.ready() || g_fake.write_calls != writes_before_rebegin ||
            g_fake.sync_calls != syncs_before_rebegin ||
            g_fake.close_calls != closes_before_rebegin) {
        return 21;
    }

    exo::Bno85Sample bno{};
    bno.quat_real = 1.0f;
    exo::Icm45686Sample icm{};
    icm.accel_z = 8192;
    for (uint8_t source = 0U; source < 5U; ++source) {
        if (!logger.append_bno(source, 10000ULL + source, bno,
                    source == 0U, 0x0FU, 11U) ||
                !logger.append_icm(source, 11000ULL + source, icm, 12U)) {
            return 2;
        }
        if (logger.bno_count(source) != 1U || logger.icm_count(source) != 1U) {
            return 3;
        }
    }
    if (logger.row_count() != 10U) {
        return 4;
    }
    if (!logger.append_bno(0U, 20000ULL, bno, true, 0x0FU, 13U) ||
            logger.bno_count(0U) != 2U || logger.icm_count(0U) != 1U ||
            logger.row_count() != 11U) {
        return 5;
    }

    const unsigned writes_before_complete = g_fake.write_calls;
    const unsigned syncs_before_complete = g_fake.sync_calls;
    g_observed_logger = &logger;
    g_fake.inspect_completion_on_sync = true;
    if (!logger.mark_source_complete(2U, 14U) ||
            g_fake.write_calls != writes_before_complete + 1U ||
            g_fake.sync_calls != syncs_before_complete + 1U ||
            g_fake.completion_mask_during_sync != 0U ||
            logger.completed_source_mask() != 0x04U) {
        return 6;
    }
    const char *const expected_rows[] = {
        "1,77,0,0,MASTER,BNO85,0,10000,",
        "1,77,1,0,MASTER,ICM45686,0,11000,",
        "1,77,2,1,NODE1,BNO85,0,10001,",
        "1,77,3,1,NODE1,ICM45686,0,11001,",
        "1,77,4,2,NODE2,BNO85,0,10002,",
        "1,77,5,2,NODE2,ICM45686,0,11002,",
        "1,77,6,3,NODE3,BNO85,0,10003,",
        "1,77,7,3,NODE3,ICM45686,0,11003,",
        "1,77,8,4,NODE4,BNO85,0,10004,",
        "1,77,9,4,NODE4,ICM45686,0,11004,",
        "1,77,10,0,MASTER,BNO85,1,20000,"
    };
    for (size_t index = 0U; index < sizeof(expected_rows) / sizeof(expected_rows[0]); ++index) {
        if (strstr(g_fake.captured, expected_rows[index]) == nullptr) {
            return 20;
        }
    }
    g_fake.inspect_completion_on_sync = false;
    g_observed_logger = nullptr;
    if (logger.mark_source_complete(2U, 15U) ||
            logger.last_operation() != exo::training_csv::TrainingCsvLogOperation::DuplicateSource) {
        return 7;
    }
    const uint32_t rows_before_completed_append = logger.row_count();
    if (logger.append_icm(2U, 20000ULL, icm, 15U) ||
            logger.last_operation() != exo::training_csv::TrainingCsvLogOperation::SourceComplete ||
            logger.row_count() != rows_before_completed_append || logger.icm_count(2U) != 1U) {
        return 22;
    }
    if (logger.append_icm(5U, 20000ULL, icm, 16U) ||
            logger.last_operation() != exo::training_csv::TrainingCsvLogOperation::InvalidSource) {
        return 8;
    }
    if (logger.shutdown(17U) || g_fake.close_calls != 1U || logger.ready() ||
            logger.expected_source_mask() != 0x1FU || logger.completed_source_mask() != 0x04U) {
        return 9;
    }

    reset_fake(9999U);
    exo::MasterTrainingCsvLogger exhausted(&kFakeOps);
    if (exhausted.begin(1U, 0x01U, 0U) ||
            exhausted.last_operation() != exo::training_csv::TrainingCsvLogOperation::NameExhausted) {
        return 10;
    }

    reset_fake();
    g_fake.short_write = true;
    exo::MasterTrainingCsvLogger header_short(&kFakeOps);
    if (header_short.begin(1U, 0x01U, 0U) ||
            header_short.last_operation() != exo::training_csv::TrainingCsvLogOperation::HeaderShortWrite ||
            !header_short.terminal_error() || g_fake.close_calls != 1U) {
        return 11;
    }

    reset_fake();
    g_fake.short_write = true;
    g_fake.close_result = FR_DISK_ERR;
    exo::MasterTrainingCsvLogger begin_close_failure(&kFakeOps);
    if (begin_close_failure.begin(2U, 0x01U, 0U) ||
            begin_close_failure.last_operation() != exo::training_csv::TrainingCsvLogOperation::Close ||
            begin_close_failure.last_result() != FR_DISK_ERR || g_fake.close_calls != 1U ||
            begin_close_failure.begin(3U, 0x01U, 0U) ||
            begin_close_failure.last_operation() !=
                    exo::training_csv::TrainingCsvLogOperation::AlreadyStarted) {
        return 23;
    }
    g_fake.short_write = false;
    g_fake.close_result = FR_OK;
    if (begin_close_failure.shutdown(1U) || g_fake.close_calls != 2U ||
            !begin_close_failure.begin(3U, 0x01U, 2U)) {
        return 24;
    }
    if (begin_close_failure.shutdown(3U)) {
        return 25;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger data_short(&kFakeOps);
    if (!data_short.begin(1U, 0x01U, 0U) ||
            !data_short.append_icm(0U, 1000ULL, icm, 0U)) {
        return 12;
    }
    g_fake.short_write = true;
    if (data_short.service(1000U) ||
            data_short.last_operation() != exo::training_csv::TrainingCsvLogOperation::DataShortWrite ||
            !data_short.terminal_error() || data_short.ready()) {
        return 13;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger incomplete(&kFakeOps);
    if (!incomplete.begin(5U, 0x1FU, 0U) ||
            !incomplete.append_icm(0U, 1ULL, icm, 0U) ||
            incomplete.shutdown(1U) || g_fake.write_calls != 2U ||
            g_fake.sync_calls != 2U || g_fake.close_calls != 1U ||
            incomplete.completed_source_mask() != 0U || g_fake.rename_calls != 0U ||
            incomplete.published()) {
        return 14;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger shutdown_sync_failure(&kFakeOps);
    if (!shutdown_sync_failure.begin(6U, 0x01U, 0U) ||
            !shutdown_sync_failure.append_icm(0U, 1ULL, icm, 0U)) {
        return 15;
    }
    g_fake.fail_sync = true;
    if (shutdown_sync_failure.shutdown(1U) || g_fake.close_calls != 1U ||
            shutdown_sync_failure.last_operation() !=
                    exo::training_csv::TrainingCsvLogOperation::DataSync ||
            g_fake.rename_calls != 0U || g_fake.marker_open_calls != 0U ||
            shutdown_sync_failure.published()) {
        return 16;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger complete_then_sync_failure(&kFakeOps);
    if (!complete_then_sync_failure.begin(8U, 0x01U, 0U) ||
            !complete_then_sync_failure.mark_source_complete(0U, 1U)) {
        return 29;
    }
    g_fake.fail_sync = true;
    if (complete_then_sync_failure.shutdown(2U) ||
            g_fake.rename_calls != 0U || g_fake.marker_open_calls != 0U ||
            complete_then_sync_failure.published() ||
            strcmp(complete_then_sync_failure.path(), "/SESSIONS/TRN0001.TMP") != 0) {
        return 30;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger shutdown_close_failure(&kFakeOps);
    if (!shutdown_close_failure.begin(7U, 0x01U, 0U)) {
        return 26;
    }
    g_fake.close_result = FR_DISK_ERR;
    if (shutdown_close_failure.shutdown(1U) ||
            shutdown_close_failure.last_operation() !=
                    exo::training_csv::TrainingCsvLogOperation::Close ||
            shutdown_close_failure.last_result() != FR_DISK_ERR ||
            shutdown_close_failure.begin(8U, 0x01U, 2U)) {
        return 27;
    }
    g_fake.close_result = FR_OK;
    if (shutdown_close_failure.shutdown(3U) ||
            !shutdown_close_failure.begin(8U, 0x01U, 4U) ||
            !shutdown_close_failure.mark_source_complete(0U, 5U) ||
            !shutdown_close_failure.shutdown(5U)) {
        return 28;
    }

    reset_fake();
    exo::MasterTrainingCsvLogger bounded(&kFakeOps);
    if (!bounded.begin(6U, 0x01U, 0U)) {
        return 17;
    }
    for (uint32_t index = 0U; index < 200U; ++index) {
        if (!bounded.append_icm(0U, static_cast<uint64_t>(index) * 5000ULL, icm, 0U)) {
            return 18;
        }
    }
    if (!bounded.mark_source_complete(0U, 1U) || !bounded.shutdown(1U) ||
            g_fake.largest_write > 8192U ||
            bounded.row_count() != 200U || bounded.icm_count(0U) != 200U) {
        return 19;
    }

    return 0;
}
