#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <exo/sensors/master_imu_csv_logger.h>

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
    char opened_path[32]{};
} g_fake;

FRESULT fake_mkdir(const TCHAR *)
{
    return FR_EXIST;
}

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

FRESULT fake_open(FIL *, const TCHAR *path, BYTE)
{
    strncpy(g_fake.opened_path, path, sizeof(g_fake.opened_path) - 1U);
    return FR_OK;
}

FRESULT fake_write(FIL *, const void *, UINT bytes, UINT *written)
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
    return FR_OK;
}

FRESULT fake_sync(FIL *)
{
    ++g_fake.sync_calls;
    return g_fake.fail_sync ? FR_DISK_ERR : FR_OK;
}

FRESULT fake_close(FIL *)
{
    ++g_fake.close_calls;
    return g_fake.close_result;
}

const exo::imu_csv::CsvFatFsOps kFakeOps = {
    fake_mkdir,
    fake_stat,
    fake_open,
    fake_write,
    fake_sync,
    fake_close
};

} // namespace

int main()
{
    exo::MasterImuCsvLogger logger(&kFakeOps);
    if (!logger.begin(10U) || strcmp(logger.path(), "/SESSIONS/IMU0002.CSV") != 0) {
        return 1;
    }
    if (g_fake.write_calls != 1U || g_fake.sync_calls != 1U || !logger.ready()) {
        return 2;
    }

    exo::Bno85Sample bno{};
    bno.quat_real = 1.0f;
    exo::Icm45686Sample icm{};
    icm.accel_z = 8192;
    if (!logger.append_bno(bno, 0x0FU, 10000ULL, 10U) ||
            !logger.append_icm(icm, 11000ULL, 11U)) {
        return 3;
    }
    if (logger.row_count() != 2U || logger.bno_count() != 1U || logger.icm_count() != 1U) {
        return 4;
    }
    if (!logger.shutdown() || g_fake.write_calls < 2U || g_fake.sync_calls < 2U || g_fake.close_calls != 1U) {
        return 5;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    g_fake.short_write = true;
    exo::MasterImuCsvLogger short_write_logger(&kFakeOps);
    if (short_write_logger.begin(0U) ||
            short_write_logger.last_operation() != exo::imu_csv::CsvLogOperation::HeaderShortWrite ||
            short_write_logger.last_result() != FR_OK) {
        return 6;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 9999U;
    exo::MasterImuCsvLogger exhausted_logger(&kFakeOps);
    if (exhausted_logger.begin(0U) ||
            exhausted_logger.last_operation() != exo::imu_csv::CsvLogOperation::NameExhausted) {
        return 7;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    g_fake.fail_sync = true;
    exo::MasterImuCsvLogger sync_logger(&kFakeOps);
    if (sync_logger.begin(0U) ||
            sync_logger.last_operation() != exo::imu_csv::CsvLogOperation::HeaderSync ||
            sync_logger.last_result() != FR_DISK_ERR) {
        return 8;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger data_write_logger(&kFakeOps);
    if (!data_write_logger.begin(0U)) {
        return 9;
    }
    g_fake.short_write = true;
    if (data_write_logger.append_icm(icm, 1000000ULL, 1000U) ||
            data_write_logger.last_operation() != exo::imu_csv::CsvLogOperation::DataShortWrite) {
        return 10;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger data_sync_logger(&kFakeOps);
    if (!data_sync_logger.begin(0U)) {
        return 11;
    }
    g_fake.fail_sync = true;
    if (data_sync_logger.append_bno(bno, 0x0FU, 1000000ULL, 1000U) ||
            data_sync_logger.last_operation() != exo::imu_csv::CsvLogOperation::DataSync ||
            data_sync_logger.last_result() != FR_DISK_ERR ||
            data_sync_logger.row_count() != 1U || data_sync_logger.bno_count() != 1U) {
        return 12;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger data_error_logger(&kFakeOps);
    if (!data_error_logger.begin(0U)) {
        return 13;
    }
    g_fake.write_result = FR_DISK_ERR;
    if (data_error_logger.append_icm(icm, 1000000ULL, 1000U) ||
            data_error_logger.last_operation() != exo::imu_csv::CsvLogOperation::DataWrite ||
            data_error_logger.last_result() != FR_DISK_ERR) {
        return 14;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger shutdown_sync_logger(&kFakeOps);
    if (!shutdown_sync_logger.begin(0U) ||
            !shutdown_sync_logger.append_icm(icm, 1000ULL, 0U)) {
        return 15;
    }
    g_fake.fail_sync = true;
    if (shutdown_sync_logger.shutdown() ||
            shutdown_sync_logger.last_operation() != exo::imu_csv::CsvLogOperation::DataSync) {
        return 16;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger close_error_logger(&kFakeOps);
    if (!close_error_logger.begin(0U)) {
        return 17;
    }
    g_fake.close_result = FR_DISK_ERR;
    if (close_error_logger.shutdown() ||
            close_error_logger.last_operation() != exo::imu_csv::CsvLogOperation::Close ||
            close_error_logger.last_result() != FR_DISK_ERR) {
        return 18;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger boundary_logger(&kFakeOps);
    if (!boundary_logger.begin(0U)) {
        return 19;
    }
    for (uint32_t index = 0U; index < 200U; ++index) {
        if (!boundary_logger.append_icm(icm, static_cast<uint64_t>(index) * 5000ULL, 0U)) {
            return 20;
        }
    }
    if (!boundary_logger.shutdown() || g_fake.write_calls < 3U || g_fake.largest_write > 8192U ||
            boundary_logger.row_count() != 200U || boundary_logger.icm_count() != 200U) {
        return 21;
    }

    g_fake = FakeFsState{};
    g_fake.highest_existing = 0U;
    exo::MasterImuCsvLogger idle_sync_logger(&kFakeOps);
    if (!idle_sync_logger.begin(0U) ||
            !idle_sync_logger.append_icm(icm, 1000ULL, 0U)) {
        return 22;
    }
    const unsigned writes_before_idle_service = g_fake.write_calls;
    if (!idle_sync_logger.service(1000U) ||
            g_fake.write_calls != (writes_before_idle_service + 1U) ||
            g_fake.sync_calls != 2U) {
        return 23;
    }

    return 0;
}
