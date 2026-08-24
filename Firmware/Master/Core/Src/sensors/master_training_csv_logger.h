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
    MarkerClose,
    InvalidSourceMetadata
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
        case TrainingCsvLogOperation::InvalidSourceMetadata: return "invalid_source_metadata";
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
        // Every source, including the Master, must register its validated
        // SessionHeader before rows are accepted. reset_runtime() leaves all
        // metadata validity flags clear.

        FRESULT result = ops_->mkdir_fn("/SESSIONS");
        if (result != FR_OK && result != FR_EXIST) {
            return set_terminal(training_csv::TrainingCsvLogOperation::Mkdir, result);
        }

        bool found = false;
        for (uint16_t index = 1U; index <= kMaxFileIndex; ++index) {
            make_paths(index);
            FILINFO info{};
            result = ops_->stat_fn(csv_path_, &info);
            if (result == FR_OK) continue;
            if (result != FR_NO_FILE) {
                return set_terminal(training_csv::TrainingCsvLogOperation::Stat, result);
            }
            result = ops_->stat_fn(path_, &info);
            if (result == FR_NO_FILE) { found = true; break; }
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
        if (result != FR_OK) return fail_begin(training_csv::TrainingCsvLogOperation::HeaderWrite, result);
        if (written != header_bytes) return fail_begin(training_csv::TrainingCsvLogOperation::HeaderShortWrite, FR_OK);
        result = ops_->sync_fn(&file_);
        if (result != FR_OK) return fail_begin(training_csv::TrainingCsvLogOperation::HeaderSync, result);
        last_sync_ms_ = now_ms;
        ready_ = true;
        return true;
    }

    bool set_source_metadata(uint8_t source_id, const SessionHeader &header)
    {
        if (!ready_ || !validate_source_id(source_id) ||
                header.session_id != session_id_ || header.node_id != source_id) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::InvalidSourceMetadata,
                    FR_INVALID_PARAMETER);
        }
        metadata_[source_id][kBnoSensorIndex] = make_metadata(header, kBnoSensorIndex);
        metadata_[source_id][kIcmSensorIndex] = make_metadata(header, kIcmSensorIndex);
        metadata_valid_[source_id][kBnoSensorIndex] = true;
        metadata_valid_[source_id][kIcmSensorIndex] = true;
        return true;
    }

    bool append_bno(uint8_t source_id, uint64_t session_time_us,
            const Bno85Sample &sample, bool has_available_mask, uint8_t available_mask,
            uint32_t now_ms)
    {
        if (!validate_append_source(source_id)) return false;
        training_csv::TrainingCsvRowContext context{};
        if (!make_row_context(source_id, kBnoSensorIndex, session_time_us, context)) return false;
        size_t row_bytes = 0U;
        const uint32_t sensor_sequence = sensor_sequence_[source_id][kBnoSensorIndex];
        if (!training_csv::format_bno_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                session_id_, row_sequence_, source_id, sensor_sequence, session_time_us,
                context, has_available_mask, available_mask, sample)) {
            return set_terminal(training_csv::TrainingCsvLogOperation::FormatBno,
                    FR_INVALID_PARAMETER);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) return false;
        commit_timestamp(source_id, kBnoSensorIndex, session_time_us, context);
        ++row_sequence_;
        ++sensor_sequence_[source_id][kBnoSensorIndex];
        ++accepted_rows_[source_id][kBnoSensorIndex];
        return synchronize_if_due(now_ms);
    }

    bool append_icm(uint8_t source_id, uint64_t session_time_us,
            const Icm45686Sample &sample, uint32_t now_ms)
    {
        if (!validate_append_source(source_id)) return false;
        training_csv::TrainingCsvRowContext context{};
        if (!make_row_context(source_id, kIcmSensorIndex, session_time_us, context)) return false;
        size_t row_bytes = 0U;
        const uint32_t sensor_sequence = sensor_sequence_[source_id][kIcmSensorIndex];
        if (!training_csv::format_icm_row(row_buffer_, sizeof(row_buffer_), row_bytes,
                session_id_, row_sequence_, source_id, sensor_sequence, session_time_us,
                context, sample)) {
            return set_terminal(training_csv::TrainingCsvLogOperation::FormatIcm,
                    FR_INVALID_PARAMETER);
        }
        if (!append_complete_row(row_buffer_, row_bytes)) return false;
        commit_timestamp(source_id, kIcmSensorIndex, session_time_us, context);
        ++row_sequence_;
        ++sensor_sequence_[source_id][kIcmSensorIndex];
        ++accepted_rows_[source_id][kIcmSensorIndex];
        return synchronize_if_due(now_ms);
    }

    bool mark_source_complete(uint8_t source_id, uint32_t now_ms)
    {
        if (!ready_ || !validate_source_id(source_id)) return false;
        const uint8_t source_bit = static_cast<uint8_t>(1U << source_id);
        if ((expected_source_mask_ & source_bit) == 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceNotExpected,
                    FR_INVALID_PARAMETER);
        }
        if ((completed_source_mask_ & source_bit) != 0U) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::DuplicateSource,
                    FR_INVALID_PARAMETER);
        }
        if (!metadata_valid_[source_id][kBnoSensorIndex] ||
                !metadata_valid_[source_id][kIcmSensorIndex]) {
            return set_nonterminal(
                    training_csv::TrainingCsvLogOperation::InvalidSourceMetadata,
                    FR_INVALID_PARAMETER);
        }
        if (!flush_buffer() || !synchronize(now_ms)) return false;
        completed_source_mask_ = static_cast<uint8_t>(completed_source_mask_ | source_bit);
        return true;
    }

    bool service(uint32_t now_ms) { return ready_ && synchronize_if_due(now_ms); }

    bool shutdown(uint32_t now_ms)
    {
        ready_ = false;
        if (!file_open_) return !terminal_error_;
        bool success = !terminal_error_;
        if (!flush_buffer()) success = false;
        const FRESULT sync_result = ops_->sync_fn(&file_);
        if (sync_result != FR_OK) {
            if (!terminal_error_) set_terminal(training_csv::TrainingCsvLogOperation::DataSync, sync_result);
            success = false;
        } else last_sync_ms_ = now_ms;
        const FRESULT close_result = ops_->close_fn(&file_);
        if (close_result != FR_OK) {
            set_terminal(training_csv::TrainingCsvLogOperation::Close, close_result);
            success = false;
        } else file_open_ = false;
        if (file_open_ || !success) return false;
        if (completed_source_mask_ != expected_source_mask_) return false;
        return publish();
    }

    bool publish()
    {
        if (completed_source_mask_ != expected_source_mask_) return false;
        if (published_) return true;
        if (file_open_ || path_[0] == '\0') return false;
        const bool already_renamed = strcmp(path_, csv_path_) == 0;
        if (!already_renamed) {
            const FRESULT rename_result = ops_->rename_fn(path_, csv_path_);
            if (rename_result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::Rename, rename_result);
            memcpy(path_, csv_path_, sizeof(path_));
        }
        FIL marker{};
        const BYTE marker_mode = static_cast<BYTE>((already_renamed ? FA_OPEN_ALWAYS : FA_CREATE_NEW) | FA_WRITE);
        FRESULT result = ops_->open_fn(&marker, ok_path_, marker_mode);
        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerOpen, result);
        result = ops_->sync_fn(&marker);
        if (result != FR_OK) { (void)ops_->close_fn(&marker); return set_terminal(training_csv::TrainingCsvLogOperation::MarkerSync, result); }
        result = ops_->close_fn(&marker);
        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, result);
        terminal_error_ = false;
        last_operation_ = training_csv::TrainingCsvLogOperation::None;
        last_result_ = FR_OK;
        published_ = true;
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
        return training_csv::source_id_valid(source_id) ? accepted_rows_[source_id][kBnoSensorIndex] : 0U;
    }
    uint32_t icm_count(uint8_t source_id) const
    {
        return training_csv::source_id_valid(source_id) ? accepted_rows_[source_id][kIcmSensorIndex] : 0U;
    }
    training_csv::TrainingCsvLogOperation last_operation() const { return last_operation_; }
    FRESULT last_result() const { return last_result_; }

private:
    static constexpr uint16_t kMaxFileIndex = 9999U;
    static constexpr uint32_t kSyncPeriodMs = 1000U;
    static constexpr size_t kRowBufferBytes = 1024U;
    static constexpr size_t kWriteBufferBytes = 8192U;
    static constexpr uint8_t kMasterSourceBit = 0x01U;
    static constexpr uint8_t kAllSourceBits = 0x1FU;
    static constexpr uint8_t kSourceCount = 5U;
    static constexpr uint8_t kBnoSensorIndex = 0U;
    static constexpr uint8_t kIcmSensorIndex = 1U;

    static training_csv::TrainingCsvSourceMetadata make_metadata(
            const SessionHeader &header, uint8_t sensor_index)
    {
        training_csv::TrainingCsvSourceMetadata metadata{};
        if (sensor_index == kBnoSensorIndex) {
            metadata.target_rate_hz = header.bno85_target_rate_hz;
            metadata.attempted_count = header.bno85_attempted_count;
            metadata.captured_count = header.bno85_captured_count;
            metadata.dropped_count = header.bno85_dropped_count;
        } else {
            metadata.target_rate_hz = header.icm45686_target_rate_hz;
            metadata.attempted_count = header.icm45686_attempted_count;
            metadata.captured_count = header.icm45686_captured_count;
            metadata.dropped_count = header.icm45686_dropped_count;
        }
        metadata.loss_flags = header.loss_flags;
        metadata.payload_crc32 = header.payload_crc32;
        return metadata;
    }

    bool make_row_context(uint8_t source_id, uint8_t sensor_index,
            uint64_t session_time_us, training_csv::TrainingCsvRowContext &context)
    {
        if (!metadata_valid_[source_id][sensor_index]) {
            return set_nonterminal(training_csv::TrainingCsvLogOperation::InvalidSourceMetadata,
                    FR_INVALID_PARAMETER);
        }
        context = training_csv::TrainingCsvRowContext{};
        context.source = metadata_[source_id][sensor_index];
        if (!previous_timestamp_valid_[source_id][sensor_index]) return true;
        const uint64_t previous = previous_timestamp_us_[source_id][sensor_index];
        if (session_time_us <= previous) {
            context.timestamp_quality_flags = training_csv::kTimestampQualityNonMonotonic;
            return true;
        }
        context.sample_delta_us = session_time_us - previous;
        context.has_sample_delta = true;
        context.effective_sample_rate_hz = 1000000.0 / static_cast<double>(context.sample_delta_us);
        context.has_effective_sample_rate = true;
        return true;
    }

    void commit_timestamp(uint8_t source_id, uint8_t sensor_index,
            uint64_t session_time_us, const training_csv::TrainingCsvRowContext &context)
    {
        if (!previous_timestamp_valid_[source_id][sensor_index] ||
                context.timestamp_quality_flags == 0U) {
            previous_timestamp_us_[source_id][sensor_index] = session_time_us;
            previous_timestamp_valid_[source_id][sensor_index] = true;
        }
    }

    void reset_runtime()
    {
        memset(&file_, 0, sizeof(file_));
        memset(sensor_sequence_, 0, sizeof(sensor_sequence_));
        memset(accepted_rows_, 0, sizeof(accepted_rows_));
        memset(metadata_, 0, sizeof(metadata_));
        memset(metadata_valid_, 0, sizeof(metadata_valid_));
        memset(previous_timestamp_us_, 0, sizeof(previous_timestamp_us_));
        memset(previous_timestamp_valid_, 0, sizeof(previous_timestamp_valid_));
        path_[0] = '\0'; csv_path_[0] = '\0'; ok_path_[0] = '\0';
        buffered_bytes_ = 0U; last_sync_ms_ = 0U; session_id_ = 0U; row_sequence_ = 0U;
        expected_source_mask_ = 0U; completed_source_mask_ = 0U;
        file_open_ = false; ready_ = false; terminal_error_ = false; published_ = false;
        file_index_ = 0U;
        last_operation_ = training_csv::TrainingCsvLogOperation::None;
        last_result_ = FR_OK;
    }

    void make_one_path(char *out, uint16_t index, const char *suffix)
    {
        static const char prefix[] = "/SESSIONS/TRN"; size_t cursor = 0U;
        for (size_t i = 0U; i < sizeof(prefix) - 1U; ++i) out[cursor++] = prefix[i];
        out[cursor++] = static_cast<char>('0' + ((index / 1000U) % 10U));
        out[cursor++] = static_cast<char>('0' + ((index / 100U) % 10U));
        out[cursor++] = static_cast<char>('0' + ((index / 10U) % 10U));
        out[cursor++] = static_cast<char>('0' + (index % 10U));
        for (size_t i = 0U; suffix[i] != '\0'; ++i) out[cursor++] = suffix[i];
        out[cursor] = '\0';
    }
    void make_paths(uint16_t index)
    {
        file_index_ = index;
        make_one_path(path_, index, ".TMP"); make_one_path(csv_path_, index, ".CSV"); make_one_path(ok_path_, index, ".OK");
    }
    bool validate_source_id(uint8_t source_id)
    {
        if (!training_csv::source_id_valid(source_id)) return set_nonterminal(training_csv::TrainingCsvLogOperation::InvalidSource, FR_INVALID_PARAMETER);
        return true;
    }
    bool validate_append_source(uint8_t source_id)
    {
        if (!ready_ || !validate_source_id(source_id)) return false;
        if ((expected_source_mask_ & static_cast<uint8_t>(1U << source_id)) == 0U)
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceNotExpected, FR_INVALID_PARAMETER);
        if ((completed_source_mask_ & static_cast<uint8_t>(1U << source_id)) != 0U)
            return set_nonterminal(training_csv::TrainingCsvLogOperation::SourceComplete, FR_INVALID_PARAMETER);
        return true;
    }
    bool append_complete_row(const char *row, size_t row_bytes)
    {
        if (row == nullptr || row_bytes == 0U || row_bytes > sizeof(write_buffer_))
            return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite, FR_INVALID_PARAMETER);
        if ((buffered_bytes_ + row_bytes) > sizeof(write_buffer_) && !flush_buffer()) return false;
        memcpy(&write_buffer_[buffered_bytes_], row, row_bytes); buffered_bytes_ += row_bytes; return true;
    }
    bool synchronize_if_due(uint32_t now_ms)
    {
        if ((now_ms - last_sync_ms_) < kSyncPeriodMs) return true;
        return flush_buffer() && synchronize(now_ms);
    }
    bool flush_buffer()
    {
        if (!file_open_) return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite, FR_INVALID_OBJECT);
        if (buffered_bytes_ == 0U) return true;
        UINT written = 0U; const UINT requested = static_cast<UINT>(buffered_bytes_);
        const FRESULT result = ops_->write_fn(&file_, write_buffer_, requested, &written);
        buffered_bytes_ = 0U;
        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::DataWrite, result);
        if (written != requested) return set_terminal(training_csv::TrainingCsvLogOperation::DataShortWrite, FR_OK);
        return true;
    }
    bool synchronize(uint32_t now_ms)
    {
        const FRESULT result = ops_->sync_fn(&file_);
        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::DataSync, result);
        last_sync_ms_ = now_ms; return true;
    }
    bool set_nonterminal(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    { last_operation_ = operation; last_result_ = result; return false; }
    bool set_terminal(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    { last_operation_ = operation; last_result_ = result; terminal_error_ = true; ready_ = false; return false; }
    bool fail_begin(training_csv::TrainingCsvLogOperation operation, FRESULT result)
    {
        set_terminal(operation, result);
        if (file_open_) {
            const FRESULT close_result = ops_->close_fn(&file_);
            if (close_result == FR_OK) file_open_ = false;
            else set_terminal(training_csv::TrainingCsvLogOperation::Close, close_result);
        }
        return false;
    }

    const training_csv::TrainingCsvFatFsOps *ops_;
    FIL file_{};
    char path_[32]{}; char csv_path_[32]{}; char ok_path_[32]{};
    char row_buffer_[kRowBufferBytes]{}; uint8_t write_buffer_[kWriteBufferBytes]{};
    size_t buffered_bytes_ = 0U;
    uint32_t sensor_sequence_[kSourceCount][2]{};
    uint32_t accepted_rows_[kSourceCount][2]{};
    training_csv::TrainingCsvSourceMetadata metadata_[kSourceCount][2]{};
    bool metadata_valid_[kSourceCount][2]{};
    uint64_t previous_timestamp_us_[kSourceCount][2]{};
    bool previous_timestamp_valid_[kSourceCount][2]{};
    uint32_t last_sync_ms_ = 0U; uint32_t session_id_ = 0U; uint32_t row_sequence_ = 0U;
    uint8_t expected_source_mask_ = 0U; uint8_t completed_source_mask_ = 0U;
    bool file_open_ = false; bool ready_ = false; bool terminal_error_ = false; bool published_ = false;
    uint16_t file_index_ = 0U;
    training_csv::TrainingCsvLogOperation last_operation_ = training_csv::TrainingCsvLogOperation::None;
    FRESULT last_result_ = FR_OK;
};

} // namespace exo
#endif
