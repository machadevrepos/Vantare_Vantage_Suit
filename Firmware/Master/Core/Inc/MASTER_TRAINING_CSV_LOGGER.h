#ifndef MASTER_TRAINING_CSV_LOGGER_H_
#define MASTER_TRAINING_CSV_LOGGER_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ff.h"
#include <MASTER_TRAINING_CSV_FORMATTER.h>

namespace exo {
namespace training_csv {

enum class TrainingCsvLogOperation : uint8_t {
    None,
    AlreadyStarted,
    Mkdir,
    Stat,
    NameExhausted,
    Open,
    HeaderWrite,
    HeaderShortWrite,
    HeaderSync,
    InvalidExpectedMask,
    InvalidSource,
    SourceNotExpected,
    DuplicateSource,
    SourceComplete,
    FormatBno,
    FormatIcm,
    DataWrite,
    DataShortWrite,
    DataSync,
    Close,
    Rename,
    MarkerOpen,
    MarkerSync,
    MarkerClose
};

inline const char *training_csv_log_operation_name(TrainingCsvLogOperation operation)
{
    switch (operation) {
        case TrainingCsvLogOperation::None: return "none";
        case TrainingCsvLogOperation::AlreadyStarted: return "already_started";
        case TrainingCsvLogOperation::Mkdir: return "mkdir";
        case TrainingCsvLogOperation::Stat: return "stat";
        case TrainingCsvLogOperation::NameExhausted: return "name_exhausted";
        case TrainingCsvLogOperation::Open: return "open";
        case TrainingCsvLogOperation::HeaderWrite: return "header_write";
        case TrainingCsvLogOperation::HeaderShortWrite: return "header_short_write";
        case TrainingCsvLogOperation::HeaderSync: return "header_sync";
        case TrainingCsvLogOperation::InvalidExpectedMask: return "invalid_expected_mask";
        case TrainingCsvLogOperation::InvalidSource: return "invalid_source";
        case TrainingCsvLogOperation::SourceNotExpected: return "source_not_expected";
        case TrainingCsvLogOperation::DuplicateSource: return "duplicate_source";
        case TrainingCsvLogOperation::SourceComplete: return "source_complete";
        case TrainingCsvLogOperation::FormatBno: return "format_bno";
        case TrainingCsvLogOperation::FormatIcm: return "format_icm";
        case TrainingCsvLogOperation::DataWrite: return "data_write";
        case TrainingCsvLogOperation::DataShortWrite: return "data_short_write";
        case TrainingCsvLogOperation::DataSync: return "data_sync";
        case TrainingCsvLogOperation::Close: return "close";
        case TrainingCsvLogOperation::Rename: return "rename";
        case TrainingCsvLogOperation::MarkerOpen: return "marker_open";
        case TrainingCsvLogOperation::MarkerSync: return "marker_sync";
        case TrainingCsvLogOperation::MarkerClose: return "marker_close";
    }
    return "unknown";
}

struct TrainingCsvFatFsOps {
    FRESULT (*mkdir_fn)(const TCHAR *path);
    FRESULT (*stat_fn)(const TCHAR *path, FILINFO *info);
    FRESULT (*open_fn)(FIL *file, const TCHAR *path, BYTE mode);
    FRESULT (*write_fn)(FIL *file, const void *data, UINT bytes, UINT *written);
    FRESULT (*sync_fn)(FIL *file);
    FRESULT (*close_fn)(FIL *file);
    FRESULT (*rename_fn)(const TCHAR *old_path, const TCHAR *new_path);
};

static const TrainingCsvFatFsOps kDefaultTrainingCsvFatFsOps = {
    f_mkdir, f_stat, f_open, f_write, f_sync, f_close, f_rename
};

} // namespace training_csv

class MasterTrainingCsvLogger {
public:
    explicit MasterTrainingCsvLogger(const training_csv::TrainingCsvFatFsOps *ops = nullptr)
        : ops_(ops != nullptr ? ops : &training_csv::kDefaultTrainingCsvFatFsOps)
    {
    }

    bool begin(uint32_t session_id, uint8_t expected_source_mask, uint32_t now_ms)
    {
        if (file_open_) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::AlreadyStarted,
                    FR_INVALID_OBJECT);
        }
        reset_runtime();
        session_id_ = session_id;
        expected_source_mask_ = expected_source_mask;
        if ((expected_source_mask & kMasterSourceBit) == 0U ||
                (expected_source_mask & static_cast<uint8_t>(~kAllSourceBits)) != 0U) {
            return set_terminal(training_csv::TrainingCsvLogOperation::InvalidExpectedMask,
                    FR_INVALID_PARAMETER);
        }

        FRESULT result = ops_->mkdir_fn("/SESSIONS");
        if (result != FR_OK && result != FR_EXIST) {
            return set_terminal(training_csv::TrainingCsvLogOperation::Mkdir, result);
        }

        bool found = false;
        for (uint16_t index = 1U; index <= kMaxFileIndex; ++index) {
            make_paths(index);
            FILINFO info{};
            result = ops_->stat_fn(csv_path_, &info);
            if (result == FR_OK) {
                continue;
            }
            if (result != FR_NO_FILE) {
                return set_terminal(training_csv::TrainingCsvLogOperation::Stat, result);
            }
            result = ops_->stat_fn(path_, &info);
            if (result == FR_NO_FILE) {
                found = true;
                break;
            }
            if (result != FR_OK) {
                return set_terminal(training_csv::TrainingCsvLogOperation::Stat, result);
            }
        }
        if (!found) {
            path_[0] = '\0';
            return set_terminal(training_csv::TrainingCsvLogOperation::NameExhausted, FR_EXIST);
        }

        result = ops_->open_fn(&file_, path_, static_cast<BYTE>(FA_CREATE_NEW | FA_WRITE));
        if (result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::Open, result);
        }
        file_open_ = true;

        const UINT header_bytes = static_cast<UINT>(sizeof(training_csv::kCsvHeader) - 1U);
        UINT written = 0U;
        result = ops_->write_fn(&file_, training_csv::kCsvHeader, header_bytes, &written);
        if (result != FR_OK) {
            return fail_begin(training_csv::TrainingCsvLogOperation::HeaderWrite, result);
        }
        if (written != header_bytes) {
            return fail_begin(training_csv::TrainingCsvLogOperation::HeaderShortWrite, FR_OK);
        }
        result = ops_->sync_fn(&file_);
        if (result != FR_OK) {
            return fail_begin(training_csv::TrainingCsvLogOperation::HeaderSync, result);
        }

        last_sync_ms_ = now_ms;
        ready_ = true;
        return true;
    }

    bool append_bno(uint8_t source_id, uint64_t session_time_us,
            const Bno85Sample &sample, bool has_available_mask, uint8_t available_mask,
            uint32_t now_ms)
    {
        if (!validate_append_source(source_id)) {
            return false;
        }
        size_t row_bytes = 0U;
        const uint32_t sensor_sequence = sensor_sequence_[source_id][kBnoSensorIndex];
        if (!training_csv::format_bno_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                session_id_, row_sequence_, source_id, sensor_sequence, session_time_us,
                has_available_mask, available_mask, sample)) {
            return set_terminal(training_csv::TrainingCsvLogOperation::FormatBno,
                    FR_INVALID_PARAMETER);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) {
            return false;
        }
        ++row_sequence_;
        ++sensor_sequence_[source_id][kBnoSensorIndex];
        ++accepted_rows_[source_id][kBnoSensorIndex];
        return synchronize_if_due(now_ms);
    }

    bool append_icm(uint8_t source_id, uint64_t session_time_us,
            const Icm45686Sample &sample, uint32_t now_ms)
    {
        if (!validate_append_source(source_id)) {
            return false;
        }
        size_t row_bytes = 0U;
        const uint32_t sensor_sequence = sensor_sequence_[source_id][kIcmSensorIndex];
        if (!training_csv::format_icm_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                session_id_, row_sequence_, source_id, sensor_sequence, session_time_us, sample)) {
            return set_terminal(training_csv::TrainingCsvLogOperation::FormatIcm,
                    FR_INVALID_PARAMETER);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) {
            return false;
        }
        ++row_sequence_;
        ++sensor_sequence_[source_id][kIcmSensorIndex];
        ++accepted_rows_[source_id][kIcmSensorIndex];
        return synchronize_if_due(now_ms);
    }

    bool mark_source_complete(uint8_t source_id, uint32_t now_ms)
    {
        if (!ready_ || !validate_source_id(source_id)) {
            return false;
        }
        const uint8_t source_bit = static_cast<uint8_t>(1U << source_id);
        if ((expected_source_mask_ & source_bit) == 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceNotExpected,
                    FR_INVALID_PARAMETER);
        }
        if ((completed_source_mask_ & source_bit) != 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::DuplicateSource,
                    FR_INVALID_PARAMETER);
        }
        if (!flush_buffer() || !synchronize(now_ms)) {
            return false;
        }
        completed_source_mask_ = static_cast<uint8_t>(completed_source_mask_ | source_bit);
        return true;
    }

    bool service(uint32_t now_ms)
    {
        return ready_ && synchronize_if_due(now_ms);
    }

    bool shutdown(uint32_t now_ms)
    {
        ready_ = false;
        if (!file_open_) {
            return !terminal_error_;
        }

        bool success = !terminal_error_;
        if (!flush_buffer()) {
            success = false;
        }
        const FRESULT sync_result = ops_->sync_fn(&file_);
        if (sync_result != FR_OK) {
            if (!terminal_error_) {
                set_terminal(training_csv::TrainingCsvLogOperation::DataSync, sync_result);
            }
            success = false;
        } else {
            last_sync_ms_ = now_ms;
        }

        const FRESULT close_result = ops_->close_fn(&file_);
        if (close_result != FR_OK) {
            set_terminal(training_csv::TrainingCsvLogOperation::Close, close_result);
            success = false;
        } else {
            file_open_ = false;
        }
        if (file_open_) {
            /* Retain the live handle so controlled shutdown can retry the close. */
            return false;
        }
        if (!success) {
            return false;
        }
        /* A partial file is a recovery artifact, not a published training dataset. Keep the
         * .TMP name unless every expected source completed; this lets a later controlled
         * session recover it without consumers mistaking it for a complete CSV. */
        if (completed_source_mask_ != expected_source_mask_) {
            return false;
        }
        return publish();
    }

    /* Renames TRNxxxx.TMP to TRNxxxx.CSV and writes TRNxxxx.OK only after every expected
     * source completed. Safe to call more than once after successful publication. */
    bool publish()
    {
        if (completed_source_mask_ != expected_source_mask_) {
            return false;
        }
        if (published_) {
            return true;
        }
        if (file_open_ || path_[0] == '\0') {
            return false;
        }
        const FRESULT rename_result = ops_->rename_fn(path_, csv_path_);
        if (rename_result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::Rename, rename_result);
        }
        memcpy(path_, csv_path_, sizeof(path_));
        published_ = true;
        FIL marker{};
        FRESULT marker_result = ops_->open_fn(&marker, ok_path_,
                static_cast<BYTE>(FA_CREATE_NEW | FA_WRITE));
        if (marker_result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::MarkerOpen, marker_result);
        }
        marker_result = ops_->sync_fn(&marker);
        if (marker_result != FR_OK) {
            (void)ops_->close_fn(&marker);
            return set_terminal(training_csv::TrainingCsvLogOperation::MarkerSync, marker_result);
        }
        marker_result = ops_->close_fn(&marker);
        if (marker_result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, marker_result);
        }
        return true;
    }

    bool ready() const { return ready_; }
    bool terminal_error() const { return terminal_error_; }
    bool has_open_file() const { return file_open_; }
    const char *path() const { return path_; }
    uint32_t session_id() const { return session_id_; }
    uint16_t file_index() const { return file_index_; }
    bool published() const { return published_; }
    uint8_t expected_source_mask() const { return expected_source_mask_; }
    uint8_t completed_source_mask() const { return completed_source_mask_; }
    uint32_t row_count() const { return row_sequence_; }
    uint32_t bno_count(uint8_t source_id) const
    {
        return training_csv::source_id_valid(source_id) ?
                accepted_rows_[source_id][kBnoSensorIndex] : 0U;
    }
    uint32_t icm_count(uint8_t source_id) const
    {
        return training_csv::source_id_valid(source_id) ?
                accepted_rows_[source_id][kIcmSensorIndex] : 0U;
    }
    training_csv::TrainingCsvLogOperation last_operation() const { return last_operation_; }
    FRESULT last_result() const { return last_result_; }

private:
    static constexpr uint16_t kMaxFileIndex = 9999U;
    static constexpr uint32_t kSyncPeriodMs = 1000U;
    static constexpr size_t kRowBufferBytes = 640U;
    static constexpr size_t kWriteBufferBytes = 8192U;
    static constexpr uint8_t kMasterSourceBit = 0x01U;
    static constexpr uint8_t kAllSourceBits = 0x1FU;
    static constexpr uint8_t kBnoSensorIndex = 0U;
    static constexpr uint8_t kIcmSensorIndex = 1U;

    void reset_runtime()
    {
        memset(&file_, 0, sizeof(file_));
        memset(sensor_sequence_, 0, sizeof(sensor_sequence_));
        memset(accepted_rows_, 0, sizeof(accepted_rows_));
        path_[0] = '\0';
        csv_path_[0] = '\0';
        ok_path_[0] = '\0';
        buffered_bytes_ = 0U;
        last_sync_ms_ = 0U;
        session_id_ = 0U;
        row_sequence_ = 0U;
        expected_source_mask_ = 0U;
        completed_source_mask_ = 0U;
        file_open_ = false;
        ready_ = false;
        terminal_error_ = false;
        published_ = false;
        file_index_ = 0U;
        last_operation_ = training_csv::TrainingCsvLogOperation::None;
        last_result_ = FR_OK;
    }

    void make_one_path(char *out, uint16_t index, const char *suffix)
    {
        static const char prefix[] = "/SESSIONS/TRN";
        size_t cursor = 0U;
        for (size_t i = 0U; i < sizeof(prefix) - 1U; ++i) {
            out[cursor++] = prefix[i];
        }
        out[cursor++] = static_cast<char>('0' + ((index / 1000U) % 10U));
        out[cursor++] = static_cast<char>('0' + ((index / 100U) % 10U));
        out[cursor++] = static_cast<char>('0' + ((index / 10U) % 10U));
        out[cursor++] = static_cast<char>('0' + (index % 10U));
        for (size_t i = 0U; suffix[i] != '\0'; ++i) {
            out[cursor++] = suffix[i];
        }
        out[cursor] = '\0';
    }

    void make_paths(uint16_t index)
    {
        file_index_ = index;
        make_one_path(path_, index, ".TMP");
        make_one_path(csv_path_, index, ".CSV");
        make_one_path(ok_path_, index, ".OK");
    }

    bool validate_source_id(uint8_t source_id)
    {
        if (!training_csv::source_id_valid(source_id)) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::InvalidSource,
                    FR_INVALID_PARAMETER);
        }
        return true;
    }

    bool validate_append_source(uint8_t source_id)
    {
        if (!ready_ || !validate_source_id(source_id)) {
            return false;
        }
        if ((expected_source_mask_ & static_cast<uint8_t>(1U << source_id)) == 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceNotExpected,
                    FR_INVALID_PARAMETER);
        }
        if ((completed_source_mask_ & static_cast<uint8_t>(1U << source_id)) != 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceComplete,
                    FR_INVALID_PARAMETER);
        }
        return true;
    }

    bool append_complete_row(const char *row, size_t row_bytes)
    {
        if (row == nullptr || row_bytes == 0U || row_bytes > sizeof(write_buffer_)) {
            return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite,
                    FR_INVALID_PARAMETER);
        }
        if ((buffered_bytes_ + row_bytes) > sizeof(write_buffer_) && !flush_buffer()) {
            return false;
        }
        memcpy(&write_buffer_[buffered_bytes_], row, row_bytes);
        buffered_bytes_ += row_bytes;
        return true;
    }

    bool synchronize_if_due(uint32_t now_ms)
    {
        if ((now_ms - last_sync_ms_) < kSyncPeriodMs) {
            return true;
        }
        return flush_buffer() && synchronize(now_ms);
    }

    bool flush_buffer()
    {
        if (!file_open_) {
            return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite,
                    FR_INVALID_OBJECT);
        }
        if (buffered_bytes_ == 0U) {
            return true;
        }
        UINT written = 0U;
        const UINT requested = static_cast<UINT>(buffered_bytes_);
        const FRESULT result = ops_->write_fn(&file_, write_buffer_, requested, &written);
        buffered_bytes_ = 0U;
        if (result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite, result);
        }
        if (written != requested) {
            return set_terminal(training_csv::TrainingCsvLogOperation::DataShortWrite, FR_OK);
        }
        return true;
    }

    bool synchronize(uint32_t now_ms)
    {
        const FRESULT result = ops_->sync_fn(&file_);
        if (result != FR_OK) {
            return set_terminal(training_csv::TrainingCsvLogOperation::DataSync, result);
        }
        last_sync_ms_ = now_ms;
        return true;
    }

    bool set_nonterminal(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    {
        last_operation_ = operation;
        last_result_ = result;
        return false;
    }

    bool set_terminal(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    {
        last_operation_ = operation;
        last_result_ = result;
        terminal_error_ = true;
        ready_ = false;
        return false;
    }

    bool fail_begin(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    {
        set_terminal(operation, result);
        if (file_open_) {
            const FRESULT close_result = ops_->close_fn(&file_);
            if (close_result == FR_OK) {
                file_open_ = false;
            } else {
                set_terminal(training_csv::TrainingCsvLogOperation::Close, close_result);
            }
        }
        return false;
    }

    const training_csv::TrainingCsvFatFsOps *ops_;
    FIL file_{};
    char path_[32]{};
    char csv_path_[32]{};
    char ok_path_[32]{};
    char row_buffer_[kRowBufferBytes]{};
    uint8_t write_buffer_[kWriteBufferBytes]{};
    size_t buffered_bytes_ = 0U;
    uint32_t sensor_sequence_[5][2]{};
    uint32_t accepted_rows_[5][2]{};
    uint32_t last_sync_ms_ = 0U;
    uint32_t session_id_ = 0U;
    uint32_t row_sequence_ = 0U;
    uint8_t expected_source_mask_ = 0U;
    uint8_t completed_source_mask_ = 0U;
    bool file_open_ = false;
    bool ready_ = false;
    bool terminal_error_ = false;
    bool published_ = false;
    uint16_t file_index_ = 0U;
    training_csv::TrainingCsvLogOperation last_operation_ =
            training_csv::TrainingCsvLogOperation::None;
    FRESULT last_result_ = FR_OK;
};

} // namespace exo

#endif /* MASTER_TRAINING_CSV_LOGGER_H_ */
