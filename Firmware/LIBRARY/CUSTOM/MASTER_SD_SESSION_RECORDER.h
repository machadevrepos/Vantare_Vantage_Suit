#ifndef MASTER_SD_SESSION_RECORDER_H_
#define MASTER_SD_SESSION_RECORDER_H_

#include <stdint.h>
#include <string.h>

#include "ACQUISITION_DIAGNOSTICS.h"
#include "HUB_SESSION_STORE.h"
#include "RECORDING_TYPES.h"

#define EXO_MASTER_REC_ROOT_DIR "/SESSIONS"
#define EXO_MASTER_REC_FINAL_PATH "/SESSIONS/MREC.BIN"

namespace exo {

class MasterSdSessionRecorder {
public:
    /* Optional; when set, sample-path f_write() calls are timed. Storage
     * behavior is unchanged either way. */
    void set_diagnostics(diag::AcquisitionDiagnostics *diagnostics) { diagnostics_ = diagnostics; }

    bool start(uint16_t node_id, uint32_t session_id, uint64_t start_timestamp_us, uint32_t duration_ms) {
#if EXO_HAS_FATFS
        /* A finalized-but-not-archived MREC.BIN is the last durable copy of the
         * Master recording. Never silently overwrite it with a new session. */
        if (archive_required_) {
            (void)service_archive_cleanup();
            last_error_ = FR_LOCKED;
            return false;
        }
        if (!close_all_files()) {
            return false;
        }
        reset_runtime();

        FRESULT result = FR_OK;
        if (::USERFatFs.fs_type == 0U) {
            result = f_mount(&::USERFatFs, ::USERPath, 1U);
            if (result != FR_OK) {
                last_error_ = result;
                return false;
            }
        }
        result = f_mkdir(EXO_MASTER_REC_ROOT_DIR);
        if (result != FR_OK && result != FR_EXIST) {
            last_error_ = result;
            return false;
        }

        result = f_open(&session_file_, EXO_MASTER_REC_FINAL_PATH, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            return false;
        }
        session_open_ = true;

        header_.magic = kSessionMagic;
        header_.version = kSessionFormatVersion;
        header_.node_id = node_id;
        header_.session_id = session_id;
        header_.start_timestamp_us = start_timestamp_us;
        header_.requested_duration_ms = duration_ms;
        header_.bno85_target_rate_hz = 100U;
        header_.icm45686_target_rate_hz = 200U;

        UINT written = 0U;
        result = f_write(&session_file_, &header_, sizeof(header_), &written);
        if (result != FR_OK || written != sizeof(header_)) {
            last_error_ = result;
            close_session();
            return false;
        }
        if (f_lseek(&session_file_, icm_region_start()) != FR_OK ||
            f_lseek(&session_file_, bno_region_start()) != FR_OK) {
            close_session();
            return false;
        }
        recording_ = true;
        return true;
#else
        (void)node_id;
        (void)session_id;
        (void)start_timestamp_us;
        (void)duration_ms;
        return false;
#endif
    }

    bool append_bno85(const Bno85Sample &sample) {
#if EXO_HAS_FATFS
        if (!recording_ || write_failed_ || bno_buffer_count_ >= kBnoBufferSamples ||
                (header_.bno85_sample_count + bno_buffer_count_) >= kMaxBnoSamples) {
            return false;
        }
        bno_buffer_[bno_buffer_count_++] = sample;
        if (bno_buffer_count_ >= kBnoBufferSamples) {
            if (!flush_bno()) {
                write_failed_ = true;
                return false;
            }
        }
        return true;
#else
        (void)sample;
        return false;
#endif
    }

    bool append_icm45686(const Icm45686Sample &sample) {
#if EXO_HAS_FATFS
        if (!recording_ || write_failed_ || icm_buffer_count_ >= kIcmBufferSamples ||
                (header_.icm45686_sample_count + icm_buffer_count_) >= kMaxIcmSamples) {
            return false;
        }
        Icm45686Sample sequenced = sample;
        /* The committed count does not advance until a 32-sample batch flushes. Assign at
         * acceptance time so every buffered sample receives its own record-local sequence. */
        sequenced.sequence = header_.icm45686_sample_count + icm_buffer_count_;
        icm_buffer_[icm_buffer_count_++] = sequenced;
        if (icm_buffer_count_ >= kIcmBufferSamples) {
            if (!flush_icm()) {
                write_failed_ = true;
                return false;
            }
        }
        return true;
#else
        (void)sample;
        return false;
#endif
    }

    bool finalize(uint32_t actual_duration_ms) {
#if EXO_HAS_FATFS
        if (!recording_ || !session_open_) {
            return false;
        }
        if (!flush_bno() || !flush_icm()) {
            recording_ = false;
            return false;
        }

        uint32_t payload_crc = 0U;
        if (!crc_file_region(bno_region_start(), header_.bno85_payload_size, payload_crc) ||
            !crc_file_region(icm_region_start(), header_.icm45686_payload_size, payload_crc)) {
            recording_ = false;
            return false;
        }

        header_.actual_duration_ms = actual_duration_ms;
        const uint32_t expected_bno = static_cast<uint32_t>(
                (static_cast<uint64_t>(actual_duration_ms) * header_.bno85_target_rate_hz + 999ULL) / 1000ULL);
        const uint32_t expected_icm = static_cast<uint32_t>(
                (static_cast<uint64_t>(actual_duration_ms) * header_.icm45686_target_rate_hz + 999ULL) / 1000ULL);
        const uint32_t observed_bno_attempts = header_.bno85_sample_count + bno_failed_count_;
        const uint32_t observed_icm_attempts = header_.icm45686_sample_count + icm_failed_count_;
        header_.bno85_attempted_count = expected_bno > observed_bno_attempts ? expected_bno : observed_bno_attempts;
        header_.icm45686_attempted_count = expected_icm > observed_icm_attempts ? expected_icm : observed_icm_attempts;
        header_.bno85_captured_count = header_.bno85_sample_count;
        header_.icm45686_captured_count = header_.icm45686_sample_count;
        header_.bno85_dropped_count = header_.bno85_attempted_count - header_.bno85_captured_count;
        header_.icm45686_dropped_count = header_.icm45686_attempted_count - header_.icm45686_captured_count;
        header_.loss_flags =
                (header_.bno85_dropped_count > bno_failed_count_ ? kSessionLossBnoRead : 0U) |
                (header_.icm45686_dropped_count > icm_failed_count_ ? kSessionLossIcmRead : 0U) |
                (bno_failed_count_ != 0U ? kSessionLossBnoWrite : 0U) |
                (icm_failed_count_ != 0U ? kSessionLossIcmWrite : 0U);
        header_.sensor_mask = 0U;
        if (header_.bno85_sample_count > 0U) {
            header_.sensor_mask |= kSensorBno85;
        }
        if (header_.icm45686_sample_count > 0U) {
            header_.sensor_mask |= kSensorIcm45686;
        }
        header_.completion_flag = kSessionComplete;
        header_.reserved = 0U;
        header_.payload_crc32 = payload_crc;
        header_.header_crc32 = session_header_crc(header_);

        if (f_lseek(&session_file_, 0U) != FR_OK) {
            recording_ = false;
            return false;
        }
        UINT written = 0U;
        FRESULT result = f_write(&session_file_, &header_, sizeof(header_), &written);
        if (result != FR_OK || written != sizeof(header_) || f_sync(&session_file_) != FR_OK) {
            last_error_ = result;
            recording_ = false;
            return false;
        }
        recording_ = false;
        ready_ = true;
        archive_required_ = true;
        return true;
#else
        (void)actual_duration_ms;
        return false;
#endif
    }

    void set_capture_failures(uint32_t bno_failures, uint32_t icm_failures) {
        bno_failed_count_ = bno_failures;
        icm_failed_count_ = icm_failures;
    }

    bool read(uint32_t offset, void *data, uint32_t size) {
#if EXO_HAS_FATFS
        if (!ready_ || !session_open_ || data == nullptr || (offset + size) > total_size()) {
            return false;
        }
        uint8_t *out = static_cast<uint8_t *>(data);
        uint32_t remaining = size;
        uint32_t logical = offset;
        while (remaining > 0U) {
            uint32_t physical = 0U;
            uint32_t span = 0U;
            if (!map_logical_span(logical, remaining, physical, span)) {
                return false;
            }
            if (f_lseek(&session_file_, physical) != FR_OK) {
                return false;
            }
            UINT read_bytes = 0U;
            if (f_read(&session_file_, out, span, &read_bytes) != FR_OK || read_bytes != span) {
                return false;
            }
            logical += span;
            out += span;
            remaining -= span;
        }
        return true;
#else
        (void)offset;
        (void)data;
        (void)size;
        return false;
#endif
    }

    bool archive_to_index(uint16_t index) {
#if EXO_HAS_FATFS
        if (!ready_ || !session_open_ || index == 0U || index > 9999U) {
            return false;
        }
        if (!service_archive_cleanup()) {
            return false;
        }

        char archive_path[32] = "/SESSIONS/R0000M.BIN";
        char temp_path[32] = "/SESSIONS/R0000T.BIN";
        archive_path[11] = temp_path[11] = static_cast<char>('0' + ((index / 1000U) % 10U));
        archive_path[12] = temp_path[12] = static_cast<char>('0' + ((index / 100U) % 10U));
        archive_path[13] = temp_path[13] = static_cast<char>('0' + ((index / 10U) % 10U));
        archive_path[14] = temp_path[14] = static_cast<char>('0' + (index % 10U));

        FILINFO info{};
        FRESULT result = f_stat(archive_path, &info);
        if (result == FR_OK) {
            return false;
        }
        if (result != FR_NO_FILE) {
            last_error_ = result;
            return false;
        }

        result = f_unlink(temp_path);
        if (result != FR_OK && result != FR_NO_FILE) {
            last_error_ = result;
            return false;
        }
        result = f_sync(&session_file_);
        if (result != FR_OK) {
            last_error_ = result;
            return false;
        }

        memset(&archive_file_, 0, sizeof(archive_file_));
        result = f_open(&archive_file_, temp_path, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            return false;
        }
        archive_file_open_ = true;

        bool compact_ok = true;
        uint32_t logical_offset = 0U;
        while (logical_offset < total_size()) {
            const uint32_t remaining = total_size() - logical_offset;
            const uint32_t chunk = min_u32(remaining, static_cast<uint32_t>(sizeof(copy_buffer_)));
            if (!read(logical_offset, copy_buffer_, chunk)) {
                compact_ok = false;
                last_error_ = FR_DISK_ERR;
                break;
            }
            UINT written = 0U;
            result = f_write(&archive_file_, copy_buffer_, chunk, &written);
            if (result != FR_OK || written != chunk) {
                compact_ok = false;
                last_error_ = (result == FR_OK) ? FR_DISK_ERR : result;
                break;
            }
            logical_offset += chunk;
        }

        if (compact_ok) {
            result = f_sync(&archive_file_);
            if (result != FR_OK) {
                compact_ok = false;
                last_error_ = result;
            }
        }
        if (compact_ok && !validate_compact_archive(archive_file_)) {
            compact_ok = false;
            last_error_ = FR_INT_ERR;
        }
        if (!compact_ok) {
            set_archive_cleanup_path(temp_path);
            (void)service_archive_cleanup();
            return false;
        }
        if (!close_tracked_file(archive_file_, archive_file_open_)) {
            set_archive_cleanup_path(temp_path);
            return false;
        }

        /* Install the already-validated compact file while the sparse live file
         * still exists. A reset during copy/install therefore cannot destroy the
         * only durable copy of the recording. */
        result = f_rename(temp_path, archive_path);
        if (result != FR_OK) {
            last_error_ = result;
            (void)f_unlink(temp_path);
            return false;
        }

        memset(&archive_file_, 0, sizeof(archive_file_));
        result = f_open(&archive_file_, archive_path, FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            set_archive_cleanup_path(archive_path);
            return false;
        }
        archive_file_open_ = true;
        const bool installed_ok = validate_compact_archive(archive_file_);
        if (!installed_ok) {
            last_error_ = FR_INT_ERR;
            set_archive_cleanup_path(archive_path);
            (void)service_archive_cleanup();
            return false;
        }

        /* The compact archive is now independently validated and was synced
         * before rename. From this point onward failures are cleanup failures;
         * they must not revoke the durable recording or lose ownership of an
         * open FIL when FatFs f_close() reports an error. */
        archive_required_ = false;
        if (!close_tracked_file(archive_file_, archive_file_open_)) {
            sparse_cleanup_pending_ = true;
            return true;
        }
        if (!close_tracked_file(session_file_, session_open_)) {
            sparse_cleanup_pending_ = true;
            return true;
        }

        result = f_open(&session_file_, archive_path, FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            sparse_cleanup_pending_ = true;
            /* R####M.BIN is already validated and durable. Reopen MREC only to
             * preserve logical reads until the next cleanup opportunity. */
            if (f_open(&session_file_, EXO_MASTER_REC_FINAL_PATH, FA_READ) == FR_OK) {
                session_open_ = true;
            }
            return true;
        }
        session_open_ = true;

        /* Failure to remove the now-redundant sparse source is non-fatal: the
         * validated R####M.BIN is already durable and authoritative. */
        result = f_unlink(EXO_MASTER_REC_FINAL_PATH);
        if (result != FR_OK && result != FR_NO_FILE) {
            last_error_ = result;
            sparse_cleanup_pending_ = true;
        } else {
            sparse_cleanup_pending_ = false;
        }
        if (!sparse_cleanup_pending_) last_error_ = FR_OK;
        return true;
#else
        (void)index;
        return false;
#endif
    }

    void reset() {
#if EXO_HAS_FATFS
        /* Preserve a finalized sparse recording until archive_to_index() has
         * produced and validated its canonical R####M.BIN. */
        if (archive_required_) {
            (void)service_archive_cleanup();
            recording_ = false;
            return;
        }
#endif
        if (close_all_files()) {
            reset_runtime();
        } else {
            recording_ = false;
            ready_ = false;
        }
    }

    bool ready() const { return ready_; }
    bool recording() const { return recording_; }
    const SessionHeader &header() const { return header_; }
    uint32_t total_size() const {
        return static_cast<uint32_t>(sizeof(SessionHeader)) + header_.bno85_payload_size + header_.icm45686_payload_size;
    }
    FRESULT last_error() const { return last_error_; }

private:
    static constexpr uint32_t kMaxBnoSamples = 360000U;  /* 15 min at 400 Hz */
    static constexpr uint32_t kMaxIcmSamples = 720000U;  /* 60 min at 200 Hz */
    static constexpr uint32_t kBnoBufferSamples = 8U;
    static constexpr uint32_t kIcmBufferSamples = 32U;
    static constexpr uint32_t kCopyBufferBytes = 256U;

    static constexpr uint32_t bno_region_start() {
        return static_cast<uint32_t>(sizeof(SessionHeader));
    }

    static constexpr uint32_t icm_region_start() {
        return bno_region_start() + (kMaxBnoSamples * static_cast<uint32_t>(sizeof(Bno85Sample)));
    }

    void reset_runtime() {
        memset(&header_, 0, sizeof(header_));
        bno_buffer_count_ = 0U;
        icm_buffer_count_ = 0U;
        recording_ = false;
        ready_ = false;
        archive_required_ = false;
        last_error_ = FR_OK;
        bno_failed_count_ = 0U;
        icm_failed_count_ = 0U;
        write_failed_ = false;
    }

#if EXO_HAS_FATFS
    /* Sample-batch writes only. The header, CRC and finalize writes are left
     * untimed because they happen outside the capture deadlines. */
    FRESULT timed_write(const void *data, uint32_t bytes, UINT &written) {
#if EXO_ACQ_DIAG_ENABLE
        diag::AcquisitionDiagnostics *const d = diagnostics_;
        const uint64_t start_us = (d != nullptr) ? EXO_ACQ_DIAG_NOW_US() : 0ULL;
        const FRESULT result = f_write(&session_file_, data, bytes, &written);
        if (d != nullptr) {
            d->sd_write.note(static_cast<uint32_t>(EXO_ACQ_DIAG_NOW_US() - start_us));
            ++d->sd_flushes;
        }
        return result;
#else
        return f_write(&session_file_, data, bytes, &written);
#endif
    }

    bool flush_bno() {
        if (!session_open_ || bno_buffer_count_ == 0U) {
            return true;
        }
        const uint32_t bytes = bno_buffer_count_ * static_cast<uint32_t>(sizeof(Bno85Sample));
        if ((header_.bno85_sample_count + bno_buffer_count_) > kMaxBnoSamples) {
            return false;
        }
        if (f_lseek(&session_file_, bno_region_start() + header_.bno85_payload_size) != FR_OK) {
            return false;
        }
        UINT written = 0U;
        const FRESULT result = timed_write(bno_buffer_, bytes, written);
        if (result != FR_OK || written != bytes) {
            last_error_ = result;
            return false;
        }
        header_.bno85_sample_count += bno_buffer_count_;
        header_.bno85_payload_size += bytes;
        bno_buffer_count_ = 0U;
        return true;
    }

    bool flush_icm() {
        if (!session_open_ || icm_buffer_count_ == 0U) {
            return true;
        }
        const uint32_t bytes = icm_buffer_count_ * static_cast<uint32_t>(sizeof(Icm45686Sample));
        if ((header_.icm45686_sample_count + icm_buffer_count_) > kMaxIcmSamples) {
            return false;
        }
        if (f_lseek(&session_file_, icm_region_start() + header_.icm45686_payload_size) != FR_OK) {
            return false;
        }
        UINT written = 0U;
        const FRESULT result = timed_write(icm_buffer_, bytes, written);
        if (result != FR_OK || written != bytes) {
            last_error_ = result;
            return false;
        }
        header_.icm45686_sample_count += icm_buffer_count_;
        header_.icm45686_payload_size += bytes;
        icm_buffer_count_ = 0U;
        return true;
    }

    bool validate_compact_archive(FIL &file) {
        const uint32_t expected_size = total_size();
        if (expected_size < static_cast<uint32_t>(sizeof(SessionHeader)) ||
            static_cast<uint32_t>(f_size(&file)) != expected_size) {
            return false;
        }

        SessionHeader compact_header{};
        if (f_lseek(&file, 0U) != FR_OK) {
            return false;
        }
        UINT read_bytes = 0U;
        if (f_read(&file, &compact_header, sizeof(compact_header), &read_bytes) != FR_OK ||
            read_bytes != sizeof(compact_header)) {
            return false;
        }
        if (memcmp(&compact_header, &header_, sizeof(header_)) != 0 ||
            compact_header.magic != kSessionMagic ||
            compact_header.version != kSessionFormatVersion ||
            compact_header.completion_flag != kSessionComplete ||
            session_header_crc(compact_header) != compact_header.header_crc32) {
            return false;
        }

        uint32_t payload_crc = 0U;
        uint32_t consumed = static_cast<uint32_t>(sizeof(SessionHeader));
        while (consumed < expected_size) {
            const uint32_t remaining = expected_size - consumed;
            const uint32_t chunk = min_u32(remaining, static_cast<uint32_t>(sizeof(copy_buffer_)));
            if (f_lseek(&file, consumed) != FR_OK) {
                return false;
            }
            read_bytes = 0U;
            if (f_read(&file, copy_buffer_, chunk, &read_bytes) != FR_OK || read_bytes != chunk) {
                return false;
            }
            payload_crc = crc32_update(payload_crc, copy_buffer_, chunk);
            consumed += chunk;
        }
        return payload_crc == compact_header.payload_crc32;
    }

    bool crc_file_region(uint32_t physical_offset, uint32_t size, uint32_t &crc) {
        uint32_t consumed = 0U;
        while (consumed < size) {
            const uint32_t remaining = size - consumed;
            const uint32_t chunk = remaining > sizeof(copy_buffer_) ? static_cast<uint32_t>(sizeof(copy_buffer_)) : remaining;
            if (f_lseek(&session_file_, physical_offset + consumed) != FR_OK) {
                return false;
            }
            UINT read_bytes = 0U;
            if (f_read(&session_file_, copy_buffer_, chunk, &read_bytes) != FR_OK || read_bytes != chunk) {
                return false;
            }
            crc = crc32_update(crc, copy_buffer_, chunk);
            consumed += chunk;
        }
        return true;
    }

    bool map_logical_span(uint32_t logical, uint32_t request, uint32_t &physical, uint32_t &span) const {
        const uint32_t header_end = static_cast<uint32_t>(sizeof(SessionHeader));
        const uint32_t bno_logical_start = header_end;
        const uint32_t icm_logical_start = bno_logical_start + header_.bno85_payload_size;
        const uint32_t total = total_size();

        if (logical < header_end) {
            physical = logical;
            span = min_u32(request, header_end - logical);
            return true;
        }
        if (logical < icm_logical_start) {
            const uint32_t offset_in_bno = logical - bno_logical_start;
            physical = bno_region_start() + offset_in_bno;
            span = min_u32(request, header_.bno85_payload_size - offset_in_bno);
            return true;
        }
        if (logical < total) {
            const uint32_t offset_in_icm = logical - icm_logical_start;
            physical = icm_region_start() + offset_in_icm;
            span = min_u32(request, header_.icm45686_payload_size - offset_in_icm);
            return true;
        }
        return false;
    }

    bool close_tracked_file(FIL &file, bool &open_flag) {
        if (!open_flag) {
            return true;
        }
        const FRESULT result = f_close(&file);
        if (result != FR_OK) {
            last_error_ = result;
            return false;
        }
        open_flag = false;
        memset(&file, 0, sizeof(file));
        return true;
    }

    void set_archive_cleanup_path(const char *path) {
        if (path == nullptr) {
            archive_cleanup_path_[0] = '\0';
            return;
        }
        strncpy(archive_cleanup_path_, path, sizeof(archive_cleanup_path_) - 1U);
        archive_cleanup_path_[sizeof(archive_cleanup_path_) - 1U] = '\0';
    }

    bool service_archive_cleanup() {
        if (!close_tracked_file(archive_file_, archive_file_open_)) {
            return false;
        }
        if (archive_cleanup_path_[0] == '\0') {
            return true;
        }
        const FRESULT result = f_unlink(archive_cleanup_path_);
        if (result != FR_OK && result != FR_NO_FILE) {
            last_error_ = result;
            return false;
        }
        archive_cleanup_path_[0] = '\0';
        return true;
    }

    bool service_sparse_cleanup() {
        if (!sparse_cleanup_pending_ || session_open_) {
            return true;
        }
        const FRESULT result = f_unlink(EXO_MASTER_REC_FINAL_PATH);
        if (result != FR_OK && result != FR_NO_FILE) {
            last_error_ = result;
            return false;
        }
        sparse_cleanup_pending_ = false;
        return true;
    }

    bool close_all_files() {
        bool ok = service_archive_cleanup();
        if (!close_tracked_file(session_file_, session_open_)) {
            ok = false;
        }
        if (!session_open_ && !service_sparse_cleanup()) {
            ok = false;
        }
        return ok;
    }

    void close_session() {
        (void)close_all_files();
    }

    static uint32_t min_u32(uint32_t a, uint32_t b) {
        return a < b ? a : b;
    }

    FIL session_file_{};
    FIL archive_file_{};
    bool session_open_ = false;
    bool archive_file_open_ = false;
    bool sparse_cleanup_pending_ = false;
    char archive_cleanup_path_[32]{};
    uint8_t copy_buffer_[kCopyBufferBytes]{};
#else
    bool close_all_files() { return true; }
    void close_session() {}
#endif

    SessionHeader header_{};
    Bno85Sample bno_buffer_[kBnoBufferSamples]{};
    Icm45686Sample icm_buffer_[kIcmBufferSamples]{};
    uint32_t bno_buffer_count_ = 0U;
    uint32_t icm_buffer_count_ = 0U;
    bool recording_ = false;
    bool ready_ = false;
    bool archive_required_ = false;
    FRESULT last_error_ = FR_OK;
    uint32_t bno_failed_count_ = 0U;
    uint32_t icm_failed_count_ = 0U;
    bool write_failed_ = false;
    diag::AcquisitionDiagnostics *diagnostics_ = nullptr;
};

} // namespace exo

#endif
