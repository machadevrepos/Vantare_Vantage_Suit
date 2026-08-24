#ifndef HUB_SENSOR_TEST_APP_H_
#define HUB_SENSOR_TEST_APP_H_

#include <stdio.h>
#include <string.h>

#include "ACQUISITION_DIAGNOSTICS.h"
#include "BNO85_STM32.h"
#include "HUB_SESSION_STORE.h"
#include "ICM45686_STM32.h"
#include "app_fatfs.h"

#ifndef EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE
#define EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE 0
#endif

namespace exo {

struct HubSensorSnapshot {
    bool has_bno85;
    bool has_icm45686;
    Bno85Sample bno85;
    Icm45686Sample icm45686;
};

class HubSensorTestApp {
public:
    HubSensorTestApp(I2C_HandleTypeDef &icm_bus, I2C_HandleTypeDef &bno_bus,
                     uint8_t bno85_address = 0x4AU, uint8_t icm45686_address = 0x68U)
        : bno85_(bno_bus, bno85_address), icm45686_(icm_bus, icm45686_address) {}

    bool begin() {
#if defined(BNO_INT_GPIO_Port) && defined(BNO_INT_Pin)
        bno85_.set_interrupt_pin(BNO_INT_GPIO_Port, BNO_INT_Pin);
#endif
        bno_ready_ = bno85_.begin();
        icm_ready_ = icm45686_.begin();
#if EXO_HAS_FATFS
        sd_ready_ = open_log_file_bin();
#endif
        return bno_ready_ || icm_ready_;
    }

    /* diagnostics may be null; when set, every call accumulates counters only. */
    void set_diagnostics(diag::AcquisitionDiagnostics *diagnostics) { diagnostics_ = diagnostics; }

    HubSensorSnapshot process() {
        HubSensorSnapshot snapshot{};
        const uint32_t now_us = HAL_GetTick() * 1000U;

#if EXO_ACQ_DIAG_ENABLE
        diag::AcquisitionDiagnostics *const d = diagnostics_;
#endif

#if !EXO_ACQ_DIAG_ICM_ONLY
        if (bno_ready_) {
#if EXO_ACQ_DIAG_ENABLE
            const uint64_t bno_start_us = (d != nullptr) ? EXO_ACQ_DIAG_NOW_US() : 0ULL;
#endif
            bno85_.service();
            /* While recording, the snapshot consumes the oldest queued
             * sample so it is still recorded via the snapshot append path;
             * remaining queued samples are drained by
             * drain_record_bno_samples() from the capture loop. Outside a
             * recording the live preview keeps the latest-value snapshot. */
            snapshot.has_bno85 = record_bno_queue_active_
                    ? bno85_.pop_sample(snapshot.bno85)
                    : bno85_.take_latest(now_us, snapshot.bno85);
#if EXO_ACQ_DIAG_ENABLE
            if (d != nullptr) {
                const uint64_t bno_end_us = EXO_ACQ_DIAG_NOW_US();
                ++d->bno_service_calls;
                d->bno_service.note(static_cast<uint32_t>(bno_end_us - bno_start_us));
                if (snapshot.has_bno85) {
                    ++d->bno_take_latest_ok;
                    d->bno_gap.note(bno_end_us);
                }
            }
#endif
        }
#endif
#if !EXO_ACQ_DIAG_BNO_ONLY
        if (icm_ready_) {
#if EXO_ACQ_DIAG_ENABLE
            const uint64_t icm_start_us = (d != nullptr) ? EXO_ACQ_DIAG_NOW_US() : 0ULL;
#endif
            if (record_icm_fifo_active_) {
                snapshot.has_icm45686 =
                        icm45686_.read_fifo_samples(&snapshot.icm45686, 1U) == 1U;
            } else {
                snapshot.has_icm45686 = icm45686_.read_sample(now_us, snapshot.icm45686);
            }
#if EXO_ACQ_DIAG_ENABLE
            if (d != nullptr) {
                const uint64_t icm_end_us = EXO_ACQ_DIAG_NOW_US();
                ++d->icm_attempts;
                d->icm_read.note(static_cast<uint32_t>(icm_end_us - icm_start_us));
                if (snapshot.has_icm45686) {
                    ++d->icm_ok;
                    d->icm_gap.note(icm_end_us);
                } else {
                    ++d->icm_fail;
                }
            }
#endif
        }
#endif
#if EXO_ACQ_DIAG_ENABLE
        if (d != nullptr) {
            /* Sensor-event and per-report totals live in the driver; mirror the
             * running totals so the summary needs no extra plumbing. */
            d->bno_sensor_events = bno85_.sensor_event_count();
            for (uint8_t slot = 0U; slot < diag::kBnoSlotCount; ++slot) {
                d->bno_reports[slot] = bno85_.report_count(slot);
            }
            if (bno85_.last_i2c_error() != 0U) {
                ++d->bno_i2c_errors;
            }
        }
#endif
#if EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE
        if (snapshot.has_bno85 || snapshot.has_icm45686) {
            append_bin_record(snapshot);
        }
#endif
        return snapshot;
    }

    /* Bounded BNO drain for extra superloop service points placed between
     * long-blocking regions (BLE dispatch, SD collect). Applies the same
     * capture/idle packet-budget policy as process(); INT-gated, so it is
     * cheap when no report is pending. */
    void service_bno() {
#if !EXO_ACQ_DIAG_ICM_ONLY
        if (!bno_ready_) return;
        bno85_.service();
#endif
    }

    bool begin_record_capture() {
        /* The BNO capture queue is enabled for every recording regardless of
         * ICM FIFO availability so the orientation stream is lossless even
         * when the register-polling fallback runs. */
        bno85_.set_capture_queue_enabled(true);
        record_bno_queue_active_ = true;
        if (!icm_ready_) return false;
        record_icm_fifo_active_ = icm45686_.begin_fifo_capture_200hz();
        return record_icm_fifo_active_;
    }

    uint8_t drain_record_icm_samples(Icm45686Sample *samples, uint8_t capacity) {
        if (!record_icm_fifo_active_) return 0U;
        return icm45686_.read_fifo_samples(samples, capacity);
    }

    uint8_t drain_record_bno_samples(Bno85Sample *samples, uint8_t capacity) {
        return bno85_.pop_samples(samples, capacity);
    }

    void freeze_record_bno_capture() {
        bno85_.set_capture_queue_enabled(false);
        record_bno_queue_active_ = false;
    }

    void clear_record_bno_samples() {
        bno85_.clear_capture_queue();
    }

    void end_record_capture() {
        freeze_record_bno_capture();
        if (!record_icm_fifo_active_) return;
        (void)icm45686_.end_fifo_capture();
        record_icm_fifo_active_ = false;
    }

    bool record_icm_fifo_active() const {
        return record_icm_fifo_active_;
    }

    bool record_bno_queue_active() const {
        return record_bno_queue_active_;
    }

    bool bno_ready() const { return bno_ready_; }
    bool icm_ready() const { return icm_ready_; }
    bool sd_ready() const { return sd_ready_; }
    uint8_t bno_available_mask() const { return bno85_.available_mask(); }
    int bno_begin_status() const { return bno85_.last_begin_status(); }
    int bno_begin_open_status() const { return bno85_.begin_open_status(); }
    int bno_begin_callback_status() const { return bno85_.begin_callback_status(); }
    int bno_begin_config_status() const { return bno85_.begin_config_status(); }
    HAL_StatusTypeDef bno_open_probe_hal_status() const { return bno85_.last_open_probe_hal_status(); }
    HAL_StatusTypeDef bno_read_header_hal_status() const { return bno85_.last_read_header_hal_status(); }
    HAL_StatusTypeDef bno_read_packet_hal_status() const { return bno85_.last_read_packet_hal_status(); }
    HAL_StatusTypeDef bno_write_hal_status() const { return bno85_.last_write_hal_status(); }
    uint32_t bno_last_i2c_error() const { return bno85_.last_i2c_error(); }
    uint32_t bno_sensor_event_count() const { return bno85_.sensor_event_count(); }
#if EXO_ACQ_DIAG_ENABLE
    void reset_bno_report_counts() { bno85_.reset_report_counts(); }
#endif
    uint8_t bno_last_report_id() const {
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        return bno85_.last_report_id();
#else
        return 0U;
#endif
    }
    int bno_last_decode_status() const { return bno85_.last_decode_status(); }
    int sd_last_mount_result() const { return static_cast<int>(sd_last_mount_result_); }
    int sd_last_mkdir_result() const { return static_cast<int>(sd_last_mkdir_result_); }
    int sd_last_open_result() const { return static_cast<int>(sd_last_open_result_); }
    int sd_last_write_result() const { return static_cast<int>(sd_last_write_result_); }
    int sd_last_sync_result() const { return static_cast<int>(sd_last_sync_result_); }
    unsigned sd_last_written_bytes() const { return static_cast<unsigned>(sd_last_written_bytes_); }

private:
#if EXO_HAS_FATFS
    static constexpr uint32_t kLogMagic = 0x31474F4CU;  // "LOG1"
    static constexpr uint32_t kLogVersion = 1U;
    static constexpr uint32_t kSyncPeriodMs = 250U;
    static constexpr UINT kBufferBytes = 16384U;
    static constexpr UINT kFlushChunkBytes = 4096U;

#pragma pack(push, 1)
    struct LogFileHeader {
        uint32_t magic;
        uint32_t version;
        uint32_t record_size;
        uint32_t created_ms;
    };

    struct BinRecord {
        uint32_t time_us;
        uint8_t flags;
        uint8_t bno_accuracy;
        uint8_t bno_status;
        uint8_t bno_report_id;
        uint8_t bno_sequence;
        uint8_t bno_available_mask;
        int8_t icm_read_status;
        uint8_t icm_data_valid;
        uint8_t reserved0;

        float bno_quat_i;
        float bno_quat_j;
        float bno_quat_k;
        float bno_quat_real;
        float bno_roll_rad;
        float bno_pitch_rad;
        float bno_yaw_rad;
        float bno_linear_accel_x;
        float bno_linear_accel_y;
        float bno_linear_accel_z;
        float bno_gravity_x;
        float bno_gravity_y;
        float bno_gravity_z;
        float bno_gyro_x;
        float bno_gyro_y;
        float bno_gyro_z;

        int16_t icm_accel_x;
        int16_t icm_accel_y;
        int16_t icm_accel_z;
        int16_t icm_gyro_x;
        int16_t icm_gyro_y;
        int16_t icm_gyro_z;
        int16_t icm_temp;
        uint16_t reserved1;
    };
#pragma pack(pop)

    bool open_log_file_bin() {
        sd_last_mount_result_ = FR_INT_ERR;
        sd_last_mkdir_result_ = FR_INT_ERR;
        sd_last_open_result_ = FR_INT_ERR;
        sd_last_write_result_ = FR_OK;
        sd_last_sync_result_ = FR_OK;
        sd_last_written_bytes_ = 0U;

        FRESULT result = f_mount(&::USERFatFs, ::USERPath, 1U);
        sd_last_mount_result_ = result;
        if (result != FR_OK) {
            return false;
        }

        result = f_mkdir("/SESSIONS");
        sd_last_mkdir_result_ = result;
        if (result != FR_OK && result != FR_EXIST) {
            return false;
        }
        result = f_open(&log_file_, "/SESSIONS/HUBTEST.BIN", FA_OPEN_APPEND | FA_WRITE);
        sd_last_open_result_ = result;
        if (result != FR_OK) {
            return false;
        }
        buffered_count_ = 0U;
        last_sync_ms_ = HAL_GetTick();

        if (f_size(&log_file_) == 0U) {
            LogFileHeader header{};
            header.magic = kLogMagic;
            header.version = kLogVersion;
            header.record_size = static_cast<uint32_t>(sizeof(BinRecord));
            header.created_ms = HAL_GetTick();
            UINT written = 0U;
            result = f_write(&log_file_, &header, sizeof(header), &written);
            sd_last_write_result_ = result;
            sd_last_written_bytes_ = written;
            if (result != FR_OK ||
                written != sizeof(header)) {
                f_close(&log_file_);
                return false;
            }
            result = f_sync(&log_file_);
            sd_last_sync_result_ = result;
            if (result != FR_OK) {
                f_close(&log_file_);
                return false;
            }
        }
        return true;
    }
#endif

    bool flush_log_buffer(bool force_sync) {
#if EXO_HAS_FATFS
        if (!sd_ready_ || buffered_count_ == 0U) {
            return true;
        }

        UINT pending = buffered_count_;
        UINT offset = 0U;
        while (pending > 0U) {
            const UINT chunk = (pending > kFlushChunkBytes) ? kFlushChunkBytes : pending;
            UINT written = 0U;
            FRESULT result = f_write(&log_file_, &log_buffer_[offset], chunk, &written);
            sd_last_write_result_ = result;
            sd_last_written_bytes_ = written;
            if (result != FR_OK || written != chunk) {
                return false;
            }
            pending -= chunk;
            offset += chunk;
        }

        buffered_count_ = 0U;

        const uint32_t now_ms = HAL_GetTick();
        if (force_sync || ((now_ms - last_sync_ms_) >= kSyncPeriodMs)) {
            FRESULT result = f_sync(&log_file_);
            sd_last_sync_result_ = result;
            if (result != FR_OK) {
                return false;
            }
            last_sync_ms_ = now_ms;
        }
        return true;
#else
        (void)force_sync;
        return false;
#endif
    }

    void append_bin_record(const HubSensorSnapshot &snapshot) {
#if EXO_HAS_FATFS
        if (!sd_ready_) {
            return;
        }
        BinRecord record{};
        record.time_us = snapshot.has_bno85 ? snapshot.bno85.offset_us : 0U;
        record.flags = static_cast<uint8_t>((snapshot.has_bno85 ? 0x01U : 0x00U) | (snapshot.has_icm45686 ? 0x02U : 0x00U));
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        record.bno_accuracy = snapshot.has_bno85 ? snapshot.bno85.accuracy : 0U;
        record.bno_status = snapshot.has_bno85 ? snapshot.bno85.status : 0U;
        record.bno_report_id = snapshot.has_bno85 ? snapshot.bno85.report_id : 0U;
        record.bno_sequence = snapshot.has_bno85 ? snapshot.bno85.sequence : 0U;
        record.bno_available_mask = snapshot.has_bno85 ? snapshot.bno85.available_mask : 0U;
        record.icm_read_status = snapshot.has_icm45686 ? static_cast<int8_t>(snapshot.icm45686.read_status) : 0;
        record.icm_data_valid = snapshot.has_icm45686 ? snapshot.icm45686.data_valid : 0U;
#else
        record.bno_accuracy = 0U;
        record.bno_status = 0U;
        record.bno_report_id = 0U;
        record.bno_sequence = 0U;
        record.bno_available_mask = 0U;
        record.icm_read_status = 0;
        record.icm_data_valid = 0U;
#endif
        record.bno_quat_i = snapshot.has_bno85 ? snapshot.bno85.quat_i : 0.0f;
        record.bno_quat_j = snapshot.has_bno85 ? snapshot.bno85.quat_j : 0.0f;
        record.bno_quat_k = snapshot.has_bno85 ? snapshot.bno85.quat_k : 0.0f;
        record.bno_quat_real = snapshot.has_bno85 ? snapshot.bno85.quat_real : 0.0f;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        record.bno_roll_rad = snapshot.has_bno85 ? snapshot.bno85.roll_rad : 0.0f;
        record.bno_pitch_rad = snapshot.has_bno85 ? snapshot.bno85.pitch_rad : 0.0f;
        record.bno_yaw_rad = snapshot.has_bno85 ? snapshot.bno85.yaw_rad : 0.0f;
#else
        record.bno_roll_rad = 0.0f;
        record.bno_pitch_rad = 0.0f;
        record.bno_yaw_rad = 0.0f;
#endif
        record.bno_linear_accel_x = snapshot.has_bno85 ? snapshot.bno85.linear_accel_x : 0.0f;
        record.bno_linear_accel_y = snapshot.has_bno85 ? snapshot.bno85.linear_accel_y : 0.0f;
        record.bno_linear_accel_z = snapshot.has_bno85 ? snapshot.bno85.linear_accel_z : 0.0f;
        record.bno_gravity_x = snapshot.has_bno85 ? snapshot.bno85.gravity_x : 0.0f;
        record.bno_gravity_y = snapshot.has_bno85 ? snapshot.bno85.gravity_y : 0.0f;
        record.bno_gravity_z = snapshot.has_bno85 ? snapshot.bno85.gravity_z : 0.0f;
        record.bno_gyro_x = snapshot.has_bno85 ? snapshot.bno85.gyro_x : 0.0f;
        record.bno_gyro_y = snapshot.has_bno85 ? snapshot.bno85.gyro_y : 0.0f;
        record.bno_gyro_z = snapshot.has_bno85 ? snapshot.bno85.gyro_z : 0.0f;
        record.icm_accel_x = snapshot.has_icm45686 ? snapshot.icm45686.accel_x : 0;
        record.icm_accel_y = snapshot.has_icm45686 ? snapshot.icm45686.accel_y : 0;
        record.icm_accel_z = snapshot.has_icm45686 ? snapshot.icm45686.accel_z : 0;
        record.icm_gyro_x = snapshot.has_icm45686 ? snapshot.icm45686.gyro_x : 0;
        record.icm_gyro_y = snapshot.has_icm45686 ? snapshot.icm45686.gyro_y : 0;
        record.icm_gyro_z = snapshot.has_icm45686 ? snapshot.icm45686.gyro_z : 0;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        record.icm_temp = snapshot.has_icm45686 ? snapshot.icm45686.temperature : 0;
#else
        record.icm_temp = 0;
#endif

        if ((buffered_count_ + sizeof(record)) > sizeof(log_buffer_)) {
            if (!flush_log_buffer(false)) {
                sd_ready_ = false;
                return;
            }
        }

        memcpy(&log_buffer_[buffered_count_], &record, sizeof(record));
        buffered_count_ += sizeof(record);

        if (buffered_count_ >= kFlushChunkBytes) {
            if (!flush_log_buffer(false)) {
                sd_ready_ = false;
                return;
            }
        }

        if ((HAL_GetTick() - last_sync_ms_) >= kSyncPeriodMs) {
            if (!flush_log_buffer(true)) {
                sd_ready_ = false;
                return;
            }
        }
#else
        (void)snapshot;
#endif
    }

    Bno85Stm32 bno85_;
    Icm45686Stm32 icm45686_;
    diag::AcquisitionDiagnostics *diagnostics_ = nullptr;
    bool bno_ready_ = false;
    bool icm_ready_ = false;
    bool sd_ready_ = false;
    bool record_icm_fifo_active_ = false;
    bool record_bno_queue_active_ = false;
#if EXO_HAS_FATFS
    FIL log_file_{};
    uint8_t log_buffer_[kBufferBytes]{};
    UINT buffered_count_ = 0U;
    uint32_t last_sync_ms_ = 0U;
    FRESULT sd_last_mount_result_ = FR_INT_ERR;
    FRESULT sd_last_mkdir_result_ = FR_INT_ERR;
    FRESULT sd_last_open_result_ = FR_INT_ERR;
    FRESULT sd_last_write_result_ = FR_OK;
    FRESULT sd_last_sync_result_ = FR_OK;
    UINT sd_last_written_bytes_ = 0U;
#endif
};

} // namespace exo

#endif
