#include <cstdint>
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

    exo::MasterTrainingCsvLogger zero_row_logger(&ops);
    EXPECT_TRUE(zero_row_logger.begin(11U, 0x03U, 0U));
    EXPECT_TRUE(!zero_row_logger.mark_source_complete(1U, 1U));
    EXPECT_TRUE(zero_row_logger.last_operation() ==
        exo::training_csv::TrainingCsvLogOperation::InvalidSourceMetadata);
    EXPECT_TRUE(zero_row_logger.set_source_metadata(1U, metadata(1U, 11U)));
    EXPECT_TRUE(zero_row_logger.mark_source_complete(1U, 2U));

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
    EXPECT_TRUE(!publish_logger.terminal_error());
    EXPECT_TRUE(publish_logger.last_operation() ==
        exo::training_csv::TrainingCsvLogOperation::None);
    EXPECT_TRUE(publish_logger.last_result() == FR_OK);
    EXPECT_TRUE(rename_calls == 1);
    EXPECT_TRUE(marker_open_calls == 2);
    return failures == 0 ? 0 : 1;
}
