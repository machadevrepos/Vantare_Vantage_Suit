#ifndef MASTER_IMU_CSV_LOGGER_H_
#define MASTER_IMU_CSV_LOGGER_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ff.h"
#include <MASTER_IMU_CSV_FORMATTER.h>

namespace exo {
namespace imu_csv {

enum class CsvLogOperation : uint8_t {
    None,
    Mkdir,
    Stat,
    NameExhausted,
    Open,
    HeaderWrite,
    HeaderShortWrite,
    HeaderSync,
    FormatBno,
    FormatIcm,
    DataWrite,
    DataShortWrite,
    DataSync,
    Close
};

inline const char *csv_log_operation_name(CsvLogOperation operation)
{
    switch (operation) {
        case CsvLogOperation::None: return "none";
        case CsvLogOperation::Mkdir: return "mkdir";
        case CsvLogOperation::Stat: return "stat";
        case CsvLogOperation::NameExhausted: return "name_exhausted";
        case CsvLogOperation::Open: return "open";
        case CsvLogOperation::HeaderWrite: return "header_write";
        case CsvLogOperation::HeaderShortWrite: return "header_short_write";
        case CsvLogOperation::HeaderSync: return "header_sync";
        case CsvLogOperation::FormatBno: return "format_bno";
        case CsvLogOperation::FormatIcm: return "format_icm";
        case CsvLogOperation::DataWrite: return "data_write";
        case CsvLogOperation::DataShortWrite: return "data_short_write";
        case CsvLogOperation::DataSync: return "data_sync";
        case CsvLogOperation::Close: return "close";
    }
    return "unknown";
}

struct CsvFatFsOps {
    FRESULT (*mkdir_fn)(const TCHAR *path);
    FRESULT (*stat_fn)(const TCHAR *path, FILINFO *info);
    FRESULT (*open_fn)(FIL *file, const TCHAR *path, BYTE mode);
    FRESULT (*write_fn)(FIL *file, const void *data, UINT bytes, UINT *written);
    FRESULT (*sync_fn)(FIL *file);
    FRESULT (*close_fn)(FIL *file);
};

static const CsvFatFsOps kDefaultFatFsOps = {
    f_mkdir,
    f_stat,
    f_open,
    f_write,
    f_sync,
    f_close
};

} // namespace imu_csv

class MasterImuCsvLogger {
public:
    explicit MasterImuCsvLogger(const imu_csv::CsvFatFsOps *ops = nullptr)
        : ops_(ops != nullptr ? ops : &imu_csv::kDefaultFatFsOps)
    {
    }

    bool begin(uint32_t now_ms)
    {
        reset_runtime();

        FRESULT result = ops_->mkdir_fn("/SESSIONS");
        if (result != FR_OK && result != FR_EXIST) {
            return fail(imu_csv::CsvLogOperation::Mkdir, result, false);
        }

        bool found = false;
        for (uint16_t index = 1U; index <= kMaxFileIndex; ++index) {
            make_path(index);
            FILINFO info{};
            result = ops_->stat_fn(path_, &info);
            if (result == FR_OK) {
                continue;
            }
            if (result == FR_NO_FILE) {
                found = true;
                break;
            }
            return fail(imu_csv::CsvLogOperation::Stat, result, false);
        }
        if (!found) {
            path_[0] = '\0';
            return fail(imu_csv::CsvLogOperation::NameExhausted, FR_EXIST, false);
        }

        result = ops_->open_fn(&file_, path_, static_cast<BYTE>(FA_CREATE_NEW | FA_WRITE));
        if (result != FR_OK) {
            return fail(imu_csv::CsvLogOperation::Open, result, false);
        }
        file_open_ = true;

        const UINT header_bytes = static_cast<UINT>(sizeof(imu_csv::kCsvHeader) - 1U);
        UINT written = 0U;
        result = ops_->write_fn(&file_, imu_csv::kCsvHeader, header_bytes, &written);
        if (result != FR_OK) {
            return fail(imu_csv::CsvLogOperation::HeaderWrite, result, true);
        }
        if (written != header_bytes) {
            return fail(imu_csv::CsvLogOperation::HeaderShortWrite, result, true);
        }
        result = ops_->sync_fn(&file_);
        if (result != FR_OK) {
            return fail(imu_csv::CsvLogOperation::HeaderSync, result, true);
        }

        last_sync_ms_ = now_ms;
        ready_ = true;
        return true;
    }

    bool append_bno(const Bno85Sample &sample,
            uint8_t available_mask,
            uint64_t timestamp_us,
            uint32_t now_ms)
    {
        if (!ready_) {
            return false;
        }
        size_t row_bytes = 0U;
        if (!imu_csv::format_bno_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                row_count_, bno_count_, timestamp_us, available_mask, sample)) {
            return fail(imu_csv::CsvLogOperation::FormatBno, FR_INVALID_PARAMETER, true);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) {
            return false;
        }
        ++row_count_;
        ++bno_count_;
        return synchronize_if_due(now_ms);
    }

    bool append_icm(const Icm45686Sample &sample,
            uint64_t timestamp_us,
            uint32_t now_ms)
    {
        if (!ready_) {
            return false;
        }
        size_t row_bytes = 0U;
        if (!imu_csv::format_icm_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                row_count_, icm_count_, timestamp_us, sample)) {
            return fail(imu_csv::CsvLogOperation::FormatIcm, FR_INVALID_PARAMETER, true);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) {
            return false;
        }
        ++row_count_;
        ++icm_count_;
        return synchronize_if_due(now_ms);
    }

    bool service(uint32_t now_ms)
    {
        return ready_ && synchronize_if_due(now_ms);
    }

    bool shutdown()
    {
        if (!file_open_) {
            ready_ = false;
            return last_operation_ == imu_csv::CsvLogOperation::None;
        }
        if (!flush_buffer(true, last_sync_ms_)) {
            return false;
        }
        const FRESULT result = ops_->close_fn(&file_);
        file_open_ = false;
        ready_ = false;
        if (result != FR_OK) {
            last_operation_ = imu_csv::CsvLogOperation::Close;
            last_result_ = result;
            return false;
        }
        return true;
    }

    bool ready() const { return ready_; }
    const char *path() const { return path_; }
    imu_csv::CsvLogOperation last_operation() const { return last_operation_; }
    FRESULT last_result() const { return last_result_; }
    uint32_t row_count() const { return row_count_; }
    uint32_t bno_count() const { return bno_count_; }
    uint32_t icm_count() const { return icm_count_; }

private:
    static constexpr uint16_t kMaxFileIndex = 9999U;
    static constexpr uint32_t kSyncPeriodMs = 1000U;
    static constexpr size_t kRowBufferBytes = 640U;
    static constexpr size_t kWriteBufferBytes = 8192U;

    void reset_runtime()
    {
        memset(&file_, 0, sizeof(file_));
        path_[0] = '\0';
        buffered_bytes_ = 0U;
        last_sync_ms_ = 0U;
        row_count_ = 0U;
        bno_count_ = 0U;
        icm_count_ = 0U;
        file_open_ = false;
        ready_ = false;
        last_operation_ = imu_csv::CsvLogOperation::None;
        last_result_ = FR_OK;
    }

    void make_path(uint16_t index)
    {
        static const char prefix[] = "/SESSIONS/IMU";
        static const char suffix[] = ".CSV";
        size_t cursor = 0U;
        for (size_t i = 0U; i < sizeof(prefix) - 1U; ++i) {
            path_[cursor++] = prefix[i];
        }
        path_[cursor++] = static_cast<char>('0' + ((index / 1000U) % 10U));
        path_[cursor++] = static_cast<char>('0' + ((index / 100U) % 10U));
        path_[cursor++] = static_cast<char>('0' + ((index / 10U) % 10U));
        path_[cursor++] = static_cast<char>('0' + (index % 10U));
        for (size_t i = 0U; i < sizeof(suffix) - 1U; ++i) {
            path_[cursor++] = suffix[i];
        }
        path_[cursor] = '\0';
    }

    bool append_complete_row(const char *row, size_t row_bytes)
    {
        if (row == nullptr || row_bytes == 0U || row_bytes > sizeof(write_buffer_)) {
            return fail(imu_csv::CsvLogOperation::DataWrite, FR_INVALID_PARAMETER, true);
        }
        if ((buffered_bytes_ + row_bytes) > sizeof(write_buffer_) && !flush_buffer(false, last_sync_ms_)) {
            return false;
        }
        memcpy(&write_buffer_[buffered_bytes_], row, row_bytes);
        buffered_bytes_ += row_bytes;

        return true;
    }

    bool synchronize_if_due(uint32_t now_ms)
    {
        return (now_ms - last_sync_ms_) < kSyncPeriodMs || flush_buffer(true, now_ms);
    }

    bool flush_buffer(bool synchronize, uint32_t now_ms)
    {
        if (!file_open_) {
            return false;
        }
        if (buffered_bytes_ > 0U) {
            UINT written = 0U;
            const UINT requested = static_cast<UINT>(buffered_bytes_);
            const FRESULT result = ops_->write_fn(&file_, write_buffer_, requested, &written);
            if (result != FR_OK) {
                return fail(imu_csv::CsvLogOperation::DataWrite, result, true);
            }
            if (written != requested) {
                return fail(imu_csv::CsvLogOperation::DataShortWrite, result, true);
            }
            buffered_bytes_ = 0U;
        }
        if (synchronize) {
            const FRESULT result = ops_->sync_fn(&file_);
            if (result != FR_OK) {
                return fail(imu_csv::CsvLogOperation::DataSync, result, true);
            }
            last_sync_ms_ = now_ms;
        }
        return true;
    }

    bool fail(imu_csv::CsvLogOperation operation, FRESULT result, bool close_file)
    {
        last_operation_ = operation;
        last_result_ = result;
        ready_ = false;
        if (close_file && file_open_) {
            (void)ops_->close_fn(&file_);
            file_open_ = false;
        }
        return false;
    }

    const imu_csv::CsvFatFsOps *ops_;
    FIL file_{};
    char path_[32]{};
    char row_buffer_[kRowBufferBytes]{};
    uint8_t write_buffer_[kWriteBufferBytes]{};
    size_t buffered_bytes_ = 0U;
    uint32_t last_sync_ms_ = 0U;
    uint32_t row_count_ = 0U;
    uint32_t bno_count_ = 0U;
    uint32_t icm_count_ = 0U;
    bool file_open_ = false;
    bool ready_ = false;
    imu_csv::CsvLogOperation last_operation_ = imu_csv::CsvLogOperation::None;
    FRESULT last_result_ = FR_OK;
};

} // namespace exo

#endif
