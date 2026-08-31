#ifndef MASTER_SD_SESSION_RECORDER_H_
#define MASTER_SD_SESSION_RECORDER_H_

#include <stdint.h>
#include <string.h>

#include "app_fatfs.h"

#include <exo/utils/acquisition_diagnostics.h>
#include <exo/storage/hub_session_store.h>
#include <exo/types/recording_types.h>

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

    enum class FinalizeStep : uint8_t { InProgress, Complete, Failed };

    bool finalize(uint32_t actual_duration_ms) {
        if (!begin_finalize(actual_duration_ms)) {
            return false;
        }
        for (;;) {
            const FinalizeStep step = service_finalize(UINT32_MAX);
            if (step == FinalizeStep::Complete) return true;
            if (step == FinalizeStep::Failed) return false;
        }
    }

    /* Chunked finalize, entry: stop appends, flush the RAM sample buffers and
     * compute the pure-RAM header fields. The long blocking pass (payload CRC
     * over both regions) is then advanced by service_finalize() in bounded
     * chunks so the superloop keeps servicing BLE links between steps. */
    bool begin_finalize(uint32_t actual_duration_ms) {
#if EXO_HAS_FATFS
        if (!recording_ || !session_open_) {
            return false;
        }
        if (!flush_bno() || !flush_icm()) {
            recording_ = false;
            finalize_phase_ = FinalizePhase::Idle;
            return false;
        }
        finalize_crc_ = 0U;
        finalize_crc_offset_ = 0U;
        finalize_region_ = 0U; /* 0 = BNO payload, 1 = ICM payload */
        finalize_phase_ = FinalizePhase::PayloadCrc;
        /* Appends must stop before the header fields are computed so they
         * describe exactly the flushed payload. */
        recording_ = false;
        compute_finalize_header(actual_duration_ms);
        return true;
#else
        (void)actual_duration_ms;
        return false;
#endif
    }

    FinalizeStep service_finalize(uint32_t byte_budget) {
#if EXO_HAS_FATFS
        if (finalize_phase_ == FinalizePhase::Idle) {
            return FinalizeStep::Failed;
        }
        uint32_t spent = 0U;
        while (spent < byte_budget) {
            if (finalize_phase_ == FinalizePhase::PayloadCrc) {
                const uint32_t region_size = finalize_region_ == 0U ?
                        header_.bno85_payload_size : header_.icm45686_payload_size;
                const uint32_t region_start = finalize_region_ == 0U ?
                        bno_region_start() : icm_region_start();
                if (finalize_crc_offset_ >= region_size) {
                    ++finalize_region_;
                    finalize_crc_offset_ = 0U;
                    if (finalize_region_ > 1U) {
                        finalize_phase_ = FinalizePhase::HeaderWrite;
                    }
                    continue;
                }
                const uint32_t remaining = region_size - finalize_crc_offset_;
                const uint32_t chunk = min_u32(remaining,
                        min_u32(static_cast<uint32_t>(sizeof(copy_buffer_)),
                                byte_budget - spent));
                UINT read_bytes = 0U;
                if (f_lseek(&session_file_, region_start + finalize_crc_offset_) != FR_OK ||
                        f_read(&session_file_, copy_buffer_, chunk, &read_bytes) != FR_OK ||
                        read_bytes != chunk) {
                    last_error_ = FR_DISK_ERR;
                    finalize_phase_ = FinalizePhase::Idle;
                    return FinalizeStep::Failed;
                }
                finalize_crc_ = crc32_update(finalize_crc_, copy_buffer_, chunk);
                finalize_crc_offset_ += chunk;
                spent += chunk;
                continue;
            }
            /* HeaderWrite: the payload CRC is complete, so seal the header. */
            header_.payload_crc32 = finalize_crc_;
            header_.header_crc32 = session_header_crc(header_);
            if (f_lseek(&session_file_, 0U) != FR_OK) {
                last_error_ = FR_DISK_ERR;
                finalize_phase_ = FinalizePhase::Idle;
                return FinalizeStep::Failed;
            }
            UINT written = 0U;
            const FRESULT result = f_write(&session_file_, &header_, sizeof(header_), &written);
            if (result != FR_OK || written != sizeof(header_) ||
                    f_sync(&session_file_) != FR_OK) {
                last_error_ = result == FR_OK ? FR_DISK_ERR : result;
                finalize_phase_ = FinalizePhase::Idle;
                return FinalizeStep::Failed;
            }
            finalize_phase_ = FinalizePhase::Idle;
            ready_ = true;
            archive_required_ = true;
            return FinalizeStep::Complete;
        }
        return FinalizeStep::InProgress;
#else
        (void)byte_budget;
        return FinalizeStep::Failed;
#endif
    }

    bool finalize_in_progress() const { return finalize_phase_ != FinalizePhase::Idle; }

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

    enum class ArchiveStep : uint8_t { InProgress, Complete, Failed };

    bool archive_to_index(uint16_t index) {
        if (!begin_archive(index)) {
            return false;
        }
        for (;;) {
            const ArchiveStep step = service_archive(UINT32_MAX);
            if (step == ArchiveStep::Complete) return true;
            if (step == ArchiveStep::Failed) return false;
        }
    }

    /* Chunked archive, entry: same guards and staging as the historical
     * synchronous archive (cleanup, target-name check, temp file), but the
     * copy/validate passes advance via service_archive() in bounded chunks so
     * the superloop keeps servicing BLE links while the archive runs. */
    bool begin_archive(uint16_t index) {
#if EXO_HAS_FATFS
        if (!ready_ || !session_open_ || index == 0U || index > 9999U) {
            return false;
        }
        if (archive_phase_ != ArchivePhase::Idle) {
            return false;
        }
        if (!service_archive_cleanup()) {
            return false;
        }

        make_archive_paths(index);
        last_archive_index_ = index;

        FILINFO info{};
        FRESULT result = f_stat(archive_path_, &info);
        if (result == FR_OK) {
            return false;
        }
        if (result != FR_NO_FILE) {
            last_error_ = result;
            return false;
        }

        result = f_unlink(temp_path_);
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
        result = f_open(&archive_file_, temp_path_, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            return false;
        }
        archive_file_open_ = true;
        archive_phase_ = ArchivePhase::Copying;
        archive_offset_ = 0U;
        archive_crc_ = 0U;
        return true;
#else
        (void)index;
        return false;
#endif
    }

    ArchiveStep service_archive(uint32_t byte_budget) {
#if EXO_HAS_FATFS
        if (archive_phase_ == ArchivePhase::Idle) {
            return ArchiveStep::Failed;
        }
        uint32_t spent = 0U;
        while (spent < byte_budget) {
            if (archive_phase_ == ArchivePhase::Copying) {
                const uint32_t total = total_size();
                if (archive_offset_ >= total) {
                    const FRESULT sync = f_sync(&archive_file_);
                    if (sync != FR_OK) {
                        last_error_ = sync;
                        return abort_archive();
                    }
                    archive_phase_ = ArchivePhase::CopyValidate;
                    archive_offset_ = 0U;
                    archive_crc_ = 0U;
                    continue;
                }
                const uint32_t remaining = total - archive_offset_;
                const uint32_t chunk = min_u32(remaining,
                        min_u32(static_cast<uint32_t>(sizeof(copy_buffer_)),
                                byte_budget - spent));
                if (!read(archive_offset_, copy_buffer_, chunk)) {
                    last_error_ = FR_DISK_ERR;
                    return abort_archive();
                }
                UINT written = 0U;
                const FRESULT result = f_write(&archive_file_, copy_buffer_, chunk, &written);
                if (result != FR_OK || written != chunk) {
                    last_error_ = result == FR_OK ? FR_DISK_ERR : result;
                    return abort_archive();
                }
                archive_offset_ += chunk;
                spent += chunk;
                continue;
            }
            if (archive_phase_ == ArchivePhase::CopyValidate) {
                return service_archive_validate(byte_budget, spent, false);
            }
            /* InstallValidate: same validation pass over the renamed file, then
             * the historical end state (session_file_ becomes the archive). */
            return service_archive_validate(byte_budget, spent, true);
        }
        return ArchiveStep::InProgress;
#else
        (void)byte_budget;
        return ArchiveStep::Failed;
#endif
    }

    bool archive_in_progress() const { return archive_phase_ != ArchivePhase::Idle; }
    bool archive_blocked() const { return archive_required_; }
    uint16_t last_archive_index() const { return last_archive_index_; }

    /* A failed archive (SD full, rename failure) must not refuse new sessions
     * forever: while idle, re-arm the latched archive with backoff. The caller
     * then keeps servicing service_archive() in bounded chunks. */
    bool service_archive_recovery(uint32_t now_ms) {
#if EXO_HAS_FATFS
        if (!archive_required_ || recording_ ||
                archive_phase_ != ArchivePhase::Idle ||
                finalize_phase_ != FinalizePhase::Idle) {
            return !archive_required_;
        }
        if (archive_recovery_last_ms_ != 0U &&
                static_cast<uint32_t>(now_ms - archive_recovery_last_ms_) < kArchiveRecoveryRetryMs) {
            return false;
        }
        archive_recovery_last_ms_ = now_ms;
        if (last_archive_index_ == 0U) {
            return service_archive_cleanup();
        }
        return begin_archive(last_archive_index_);
#else
        (void)now_ms;
        return true;
#endif
    }

    void reset() {
#if EXO_HAS_FATFS
        if (archive_phase_ != ArchivePhase::Idle || finalize_phase_ != FinalizePhase::Idle) {
            /* A reset during a chunked finalize/archive must cancel the machine
             * before the cleanup path closes its files, otherwise later service
             * calls would run against closed FILs. The latched archive is then
             * re-attempted by service_archive_recovery(). */
            if (archive_file_open_) {
                (void)f_close(&archive_file_);
                archive_file_open_ = false;
            }
            set_archive_cleanup_path(archive_phase_ == ArchivePhase::InstallValidate ?
                    archive_path_ : temp_path_);
            archive_phase_ = ArchivePhase::Idle;
            finalize_phase_ = FinalizePhase::Idle;
            recording_ = false;
        }
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
    /* Buffers sized so a flush is a whole number of 512 B sectors: every
     * flush then writes full sectors directly. A partial-sector flush would
     * force FatFS through a read-modify-write for every flush, which on the
     * SPI bit-banged card path cost tens of ms per superloop iteration and
     * throttled the capture loop below the sensor rates. */
    static constexpr uint32_t kBnoBufferSamples = 64U;   /* 3584 B = 7 sectors */
    static constexpr uint32_t kIcmBufferSamples = 128U;  /* 2560 B = 5 sectors */
    static_assert((kBnoBufferSamples * sizeof(Bno85Sample)) % 512U == 0U,
            "BNO flush size must be sector-aligned to avoid read-modify-write");
    static_assert((kIcmBufferSamples * sizeof(Icm45686Sample)) % 512U == 0U,
            "ICM flush size must be sector-aligned to avoid read-modify-write");
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

    void compute_finalize_header(uint32_t actual_duration_ms) {
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
    }

    void make_archive_paths(uint16_t index) {
        memcpy(archive_path_, "/SESSIONS/R0000M.BIN", sizeof("/SESSIONS/R0000M.BIN"));
        memcpy(temp_path_, "/SESSIONS/R0000T.BIN", sizeof("/SESSIONS/R0000T.BIN"));
        archive_path_[11] = temp_path_[11] = static_cast<char>('0' + ((index / 1000U) % 10U));
        archive_path_[12] = temp_path_[12] = static_cast<char>('0' + ((index / 100U) % 10U));
        archive_path_[13] = temp_path_[13] = static_cast<char>('0' + ((index / 10U) % 10U));
        archive_path_[14] = temp_path_[14] = static_cast<char>('0' + (index % 10U));
    }

    ArchiveStep abort_archive() {
        if (archive_file_open_) {
            (void)f_close(&archive_file_);
            archive_file_open_ = false;
        }
        /* During InstallValidate the temp file is already renamed; the bad
         * artifact to remove is the installed archive itself. */
        set_archive_cleanup_path(archive_phase_ == ArchivePhase::InstallValidate ?
                archive_path_ : temp_path_);
        (void)service_archive_cleanup();
        archive_phase_ = ArchivePhase::Idle;
        return ArchiveStep::Failed;
    }

    /* Chunked validation of the archive file for both the pre-rename and the
     * post-rename pass. On pre-rename success the temp file is installed
     * (rename + reopen) and validation restarts over the renamed file. */
    ArchiveStep service_archive_validate(uint32_t byte_budget, uint32_t spent, bool install) {
        const uint32_t expected_size = total_size();
        while (spent < byte_budget) {
            if (archive_offset_ == 0U) {
                if (expected_size < static_cast<uint32_t>(sizeof(SessionHeader)) ||
                        static_cast<uint32_t>(f_size(&archive_file_)) != expected_size) {
                    last_error_ = FR_INT_ERR;
                    return abort_archive();
                }
                SessionHeader compact_header{};
                if (f_lseek(&archive_file_, 0U) != FR_OK) {
                    last_error_ = FR_DISK_ERR;
                    return abort_archive();
                }
                UINT read_bytes = 0U;
                if (f_read(&archive_file_, &compact_header, sizeof(compact_header), &read_bytes) != FR_OK ||
                        read_bytes != sizeof(compact_header)) {
                    last_error_ = FR_DISK_ERR;
                    return abort_archive();
                }
                if (memcmp(&compact_header, &header_, sizeof(header_)) != 0 ||
                        compact_header.magic != kSessionMagic ||
                        compact_header.version != kSessionFormatVersion ||
                        compact_header.completion_flag != kSessionComplete ||
                        session_header_crc(compact_header) != compact_header.header_crc32) {
                    last_error_ = FR_INT_ERR;
                    return abort_archive();
                }
                archive_offset_ = static_cast<uint32_t>(sizeof(SessionHeader));
                archive_crc_ = 0U;
                spent += static_cast<uint32_t>(sizeof(SessionHeader));
                continue;
            }
            if (archive_offset_ >= expected_size) {
                if (archive_crc_ != header_.payload_crc32) {
                    last_error_ = FR_INT_ERR;
                    return abort_archive();
                }
                if (!install) {
                    /* Install the already-validated compact file while the
                     * sparse live file still exists, then re-validate the
                     * renamed file before declaring durability. */
                    if (!close_tracked_file(archive_file_, archive_file_open_)) {
                        return abort_archive();
                    }
                    const FRESULT rename = f_rename(temp_path_, archive_path_);
                    if (rename != FR_OK) {
                        last_error_ = rename;
                        (void)f_unlink(temp_path_);
                        archive_phase_ = ArchivePhase::Idle;
                        return ArchiveStep::Failed;
                    }
                    memset(&archive_file_, 0, sizeof(archive_file_));
                    const FRESULT open = f_open(&archive_file_, archive_path_, FA_READ);
                    if (open != FR_OK) {
                        last_error_ = open;
                        archive_phase_ = ArchivePhase::Idle;
                        set_archive_cleanup_path(archive_path_);
                        (void)service_archive_cleanup();
                        return ArchiveStep::Failed;
                    }
                    archive_file_open_ = true;
                    archive_phase_ = ArchivePhase::InstallValidate;
                    archive_offset_ = 0U;
                    archive_crc_ = 0U;
                    continue;
                }
                return finish_archive_install();
            }
            const uint32_t remaining = expected_size - archive_offset_;
            const uint32_t chunk = min_u32(remaining,
                    min_u32(static_cast<uint32_t>(sizeof(copy_buffer_)),
                            byte_budget - spent));
            UINT read_bytes = 0U;
            if (f_lseek(&archive_file_, archive_offset_) != FR_OK ||
                    f_read(&archive_file_, copy_buffer_, chunk, &read_bytes) != FR_OK ||
                    read_bytes != chunk) {
                last_error_ = FR_DISK_ERR;
                return abort_archive();
            }
            archive_crc_ = crc32_update(archive_crc_, copy_buffer_, chunk);
            archive_offset_ += chunk;
            spent += chunk;
        }
        return ArchiveStep::InProgress;
    }

    ArchiveStep finish_archive_install() {
        /* The compact archive is now independently validated and was synced
         * before rename. From this point onward failures are cleanup failures;
         * they must not revoke the durable recording or lose ownership of an
         * open FIL when FatFs f_close() reports an error. */
        archive_required_ = false;
        archive_phase_ = ArchivePhase::Idle;
        if (!close_tracked_file(archive_file_, archive_file_open_)) {
            sparse_cleanup_pending_ = true;
            return ArchiveStep::Complete;
        }
        if (!close_tracked_file(session_file_, session_open_)) {
            sparse_cleanup_pending_ = true;
            return ArchiveStep::Complete;
        }
        FRESULT result = f_open(&session_file_, archive_path_, FA_READ);
        if (result != FR_OK) {
            last_error_ = result;
            sparse_cleanup_pending_ = true;
            /* R####M.BIN is already validated and durable. Reopen MREC only to
             * preserve logical reads until the next cleanup opportunity. */
            if (f_open(&session_file_, EXO_MASTER_REC_FINAL_PATH, FA_READ) == FR_OK) {
                session_open_ = true;
            }
            return ArchiveStep::Complete;
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
        return ArchiveStep::Complete;
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

    enum class FinalizePhase : uint8_t { Idle, PayloadCrc, HeaderWrite };
    enum class ArchivePhase : uint8_t { Idle, Copying, CopyValidate, InstallValidate };
    static constexpr uint32_t kArchiveRecoveryRetryMs = 5000U;
    FinalizePhase finalize_phase_ = FinalizePhase::Idle;
    uint32_t finalize_crc_ = 0U;
    uint32_t finalize_crc_offset_ = 0U;
    uint8_t finalize_region_ = 0U;
    ArchivePhase archive_phase_ = ArchivePhase::Idle;
    uint32_t archive_offset_ = 0U;
    uint32_t archive_crc_ = 0U;
    uint16_t last_archive_index_ = 0U;
    uint32_t archive_recovery_last_ms_ = 0U;
    char archive_path_[32] {};
    char temp_path_[32] {};
};

} // namespace exo

#endif
