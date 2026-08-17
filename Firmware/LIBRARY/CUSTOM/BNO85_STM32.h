#ifndef BNO85_STM32_H_
#define BNO85_STM32_H_

#include <string.h>

extern "C" {
#include "i2c.h"
#include "sh2/sh2.h"
#include "sh2/sh2_err.h"
#include "sh2/sh2_SensorValue.h"
}
#include "INCLUDER.h"

#include "ACQUISITION_DIAGNOSTICS.h"
#include "RECORDING_TYPES.h"

namespace exo {

#ifndef EXO_BNO85_I2C_TRACE
#define EXO_BNO85_I2C_TRACE 0
#endif

#ifndef EXO_BNO85_REPORT_INTERVAL_US
#define EXO_BNO85_REPORT_INTERVAL_US 10000U
#endif

/* The BNO08x caps every fused report (game rotation vector, linear
 * acceleration, gravity, calibrated gyro) at 400 Hz = 2500 us. The game
 * rotation vector anchors one recorded sample per report, so it runs at the
 * fusion maximum. The auxiliary reports run slower: every report costs the
 * superloop ~0.7 ms of blocking I2C, so 400 + 3x100 reports/s saturated the
 * Node loop (measured: GRV delivered at ~125 Hz, ICM sagged to ~125 Hz).
 * 400 + 3x50 = 550 reports/s leaves the loop headroom for flash flushes,
 * the ICM schedule and BLE. */
#ifndef EXO_BNO85_RV_REPORT_INTERVAL_US
#define EXO_BNO85_RV_REPORT_INTERVAL_US 2500U
#endif

#ifndef EXO_BNO85_AUX_REPORT_INTERVAL_US
#define EXO_BNO85_AUX_REPORT_INTERVAL_US 5000U
#endif

/* SHTP packets drained per service() call while recording. At 400 Hz +
 * 3x50 Hz the bus produces ~0.55 packets/ms, so the budget must cover the
 * backlog left by a worst-case superloop iteration (BLE + storage bursts). */
#ifndef EXO_BNO85_SERVICE_BUDGET
#define EXO_BNO85_SERVICE_BUDGET 16U
#endif

/* Packets drained per service() call while NOT recording. The sensor keeps
 * producing reports at the configured rate forever; draining them at the
 * full budget made idle loops (live preview, and especially the flash
 * upload phase after a recording) spend half their time in I2C, which
 * starved BLE and contributed to node upload stalls. Preview needs ~25 Hz,
 * which a small budget delivers while keeping the loop responsive. */
#ifndef EXO_BNO85_IDLE_SERVICE_BUDGET
#define EXO_BNO85_IDLE_SERVICE_BUDGET 4U
#endif

/* Capture queue depth (power of two). While enabled, every game rotation
 * vector report is queued instead of only latching the latest value, so a
 * slow superloop iteration stalls the recorder instead of discarding
 * reports. At 400 Hz the queue spans depth * 2.5 ms of stall. */
#ifndef EXO_BNO85_SAMPLE_QUEUE_DEPTH
#define EXO_BNO85_SAMPLE_QUEUE_DEPTH 64U
#endif
static_assert(((EXO_BNO85_SAMPLE_QUEUE_DEPTH != 0U) &&
               ((EXO_BNO85_SAMPLE_QUEUE_DEPTH & (EXO_BNO85_SAMPLE_QUEUE_DEPTH - 1U)) == 0U)),
              "EXO_BNO85_SAMPLE_QUEUE_DEPTH must be a power of two");
static_assert(EXO_BNO85_SAMPLE_QUEUE_DEPTH <= 255U,
              "EXO_BNO85_SAMPLE_QUEUE_DEPTH must fit the queue counters");

class Bno85Stm32 {
public:
    Bno85Stm32(I2C_HandleTypeDef &bus, uint8_t address_7bit)
        : bus_(bus), address_(static_cast<uint16_t>(address_7bit << 1U)) {
        active_ = this;
        hal_.open = &Bno85Stm32::open;
        hal_.close = &Bno85Stm32::close;
        hal_.read = &Bno85Stm32::read;
        hal_.write = &Bno85Stm32::write;
        hal_.getTimeUs = &Bno85Stm32::get_time_us;
    }

    void set_interrupt_pin(GPIO_TypeDef *port, uint16_t pin) {
        interrupt_port_ = port;
        interrupt_pin_ = pin;
    }

    bool begin() {
        begin_open_status_ = SH2_ERR;
        begin_callback_status_ = SH2_ERR;
        begin_config_status_ = SH2_ERR;
        next_read_len_ = kShtpHeaderLen;
        sh2_SensorConfig_t rv_config = {};
        rv_config.reportInterval_us = EXO_BNO85_RV_REPORT_INTERVAL_US;
        sh2_SensorConfig_t aux_config = {};
        aux_config.reportInterval_us = EXO_BNO85_AUX_REPORT_INTERVAL_US;
        int open_status = SH2_ERR_IO;
        for (uint8_t attempt = 0U; attempt < 5U; ++attempt) {
            open_status = sh2_open(&hal_, &Bno85Stm32::async_event_callback, nullptr);
            if (open_status == SH2_OK) {
                break;
            }
            HAL_Delay(50U);
        }
        if (open_status != SH2_OK) {
            begin_open_status_ = open_status;
            last_begin_status_ = open_status;
            return false;
        }
        begin_open_status_ = open_status;
        const int callback_status = sh2_setSensorCallback(&Bno85Stm32::sensor_event_callback, nullptr);
        if (callback_status != SH2_OK) {
            begin_callback_status_ = callback_status;
            last_begin_status_ = callback_status;
            return false;
        }
        begin_callback_status_ = callback_status;
        const uint32_t wait_start = HAL_GetTick();
        while ((HAL_GetTick() - wait_start) < 50U) {
            sh2_service();
        }
        const int grv_status = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &rv_config);
#if EXO_ACQ_DIAG_BNO_RV_ONLY
        /* Experiment D2: leave the auxiliary reports disabled so the rotation
         * vector owns the SHTP bandwidth. Reported as SH2_OK so begin() still
         * succeeds on the strength of the rotation vector alone. */
        const int la_status = SH2_OK;
        const int grav_status = SH2_OK;
        const int gyro_status = SH2_OK;
#else
        const int la_status = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &aux_config);
        const int grav_status = sh2_setSensorConfig(SH2_GRAVITY, &aux_config);
        const int gyro_status = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &aux_config);
#endif
        const int config_status =
            (grv_status == SH2_OK || la_status == SH2_OK || grav_status == SH2_OK || gyro_status == SH2_OK)
                ? SH2_OK
                : grv_status;
        begin_config_status_ = config_status;
        last_begin_status_ = config_status;
        return config_status == SH2_OK;
    }

    void service() {
        service_pending(capture_queue_enabled_
                ? EXO_BNO85_SERVICE_BUDGET
                : EXO_BNO85_IDLE_SERVICE_BUDGET);
    }

    /* BNO08x INT is level-low while SHTP data is pending. Polling the level in
     * foreground avoids a 20 ms HAL receive timeout when no packet exists and
     * lets one superloop iteration drain a bounded backlog. I2C is never done
     * from the GPIO ISR. A partial SHTP transfer is always finished even if INT
     * has returned high between chunks. */
    void service_pending(uint8_t max_packets) {
        if (max_packets == 0U) return;
        if (interrupt_port_ == nullptr || interrupt_pin_ == 0U) {
            sh2_service();
            return;
        }
        for (uint8_t serviced = 0U; serviced < max_packets; ++serviced) {
            const bool continuation = next_read_len_ != kShtpHeaderLen;
            const bool pending = HAL_GPIO_ReadPin(interrupt_port_, interrupt_pin_) == GPIO_PIN_RESET;
            if (!continuation && !pending) break;
            sh2_service();
        }
    }

    bool take_latest(uint32_t offset_us, Bno85Sample &sample) {
        if (!has_rotation_ || !has_new_data_) {
            return false;
        }
        sample.offset_us = offset_us;
        sample.quat_i = latest_quat_i_;
        sample.quat_j = latest_quat_j_;
        sample.quat_k = latest_quat_k_;
        sample.quat_real = latest_quat_real_;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        sample.roll_rad = 0.0f;
        sample.pitch_rad = 0.0f;
        sample.yaw_rad = 0.0f;
#endif

        sample.linear_accel_x = latest_linear_accel_x_;
        sample.linear_accel_y = latest_linear_accel_y_;
        sample.linear_accel_z = latest_linear_accel_z_;
        sample.gravity_x = latest_gravity_x_;
        sample.gravity_y = latest_gravity_y_;
        sample.gravity_z = latest_gravity_z_;
        sample.gyro_x = latest_gyro_x_;
        sample.gyro_y = latest_gyro_y_;
        sample.gyro_z = latest_gyro_z_;

#if EXO_SAMPLE_FORMAT_VERSION == 2U
        sample.accuracy = latest_status_;
        sample.status = latest_sensor_id_;
        sample.report_id = last_report_id_;
        sample.sequence = latest_sequence_;
        sample.available_mask = latest_available_mask_;
        sample.reserved0 = 0U;
        sample.sensor_timestamp_us = latest_sensor_timestamp_us_;
        sample.delay_us = latest_delay_us_;
#endif

        has_new_data_ = false;
        return true;
    }

    /* ---- Lossless capture queue (recording path) ----
     *
     * take_latest() coalesces reports into one snapshot, so the recorded BNO
     * rate could never exceed the superloop rate. While the capture queue is
     * enabled, every game rotation vector report instead pushes one merged
     * sample here; the recorder drains them in bulk exactly like the
     * ICM45686 hardware FIFO. */
    void set_capture_queue_enabled(bool enabled) {
        capture_queue_enabled_ = enabled;
        q_head_ = 0U;
        q_count_ = 0U;
        pop_offset_valid_ = false;
        pop_offset_us_ = 0U;
        if (enabled) {
            /* Counters start clean per session; on disable they survive so
             * the finalizer can still report the session's queue drops. */
            queue_drops_ = 0U;
        }
    }

    bool capture_queue_enabled() const { return capture_queue_enabled_; }

    uint8_t queued_sample_count() const { return q_count_; }

    uint32_t queue_drops() const { return queue_drops_; }

    uint8_t pop_samples(Bno85Sample *out, uint8_t capacity) {
        if (out == nullptr || capacity == 0U || q_count_ == 0U) {
            return 0U;
        }
        const uint8_t n = (q_count_ < capacity) ? q_count_ : capacity;
        /* The sensor samples on a uniform 1/rate grid; arrival times only
         * reflect when the superloop drained the bus. Rebuild that grid:
         * continue from the last assigned offset, or re-anchor to now when
         * the grid fell far behind (the device dropped reports during a
         * stall) so the timeline jumps forward instead of compressing. */
        uint32_t newest = pop_offset_valid_
                ? pop_offset_us_ + (static_cast<uint32_t>(n) * EXO_BNO85_RV_REPORT_INTERVAL_US)
                : micros32();
        if ((static_cast<int32_t>(micros32() - newest)) >
                static_cast<int32_t>(2U * EXO_BNO85_RV_REPORT_INTERVAL_US)) {
            newest = micros32();
        }
        for (uint8_t i = 0U; i < n; ++i) {
            out[i] = queue_[(q_head_ + i) & kQueueMask];
            out[i].offset_us = newest - (static_cast<uint32_t>(n - 1U - i) * EXO_BNO85_RV_REPORT_INTERVAL_US);
        }
        q_head_ = static_cast<uint8_t>((q_head_ + n) & kQueueMask);
        q_count_ = static_cast<uint8_t>(q_count_ - n);
        pop_offset_us_ = newest;
        pop_offset_valid_ = true;
        return n;
    }

    bool pop_sample(Bno85Sample &out) {
        return pop_samples(&out, 1U) == 1U;
    }

    int last_begin_status() const { return last_begin_status_; }
    int begin_open_status() const { return begin_open_status_; }
    int begin_callback_status() const { return begin_callback_status_; }
    int begin_config_status() const { return begin_config_status_; }
    uint8_t available_mask() const { return latest_available_mask_; }
    HAL_StatusTypeDef last_open_probe_hal_status() const { return last_open_probe_hal_status_; }
    HAL_StatusTypeDef last_read_header_hal_status() const { return last_read_header_hal_status_; }
    HAL_StatusTypeDef last_read_packet_hal_status() const { return last_read_packet_hal_status_; }
    HAL_StatusTypeDef last_write_hal_status() const { return last_write_hal_status_; }
    uint32_t last_i2c_error() const { return last_i2c_error_; }
    uint32_t sensor_event_count() const { return sensor_event_count_; }
    uint8_t last_report_id() const { return last_report_id_; }
#if EXO_ACQ_DIAG_ENABLE
    uint32_t report_count(uint8_t slot) const {
        return (slot < ::exo::diag::kBnoSlotCount) ? report_counts_[slot] : 0U;
    }
    void reset_report_counts() {
        for (uint8_t i = 0U; i < ::exo::diag::kBnoSlotCount; ++i) {
            report_counts_[i] = 0U;
        }
        sensor_event_count_ = 0U;
    }
#endif
    int last_decode_status() const { return last_decode_status_; }

private:
    static constexpr uint16_t kShtpHeaderLen = 4U;
    static constexpr uint16_t kMaxI2cReadLen = 128U;
    static constexpr float kPi = 3.14159265f;
    static constexpr float kHalfPi = 1.57079633f;
    static constexpr uint8_t kQueueMask =
            static_cast<uint8_t>(EXO_BNO85_SAMPLE_QUEUE_DEPTH - 1U);

    /* Microsecond clock for report timestamping. HAL_GetTick() is 1 ms
     * quantized and cannot order 2.5 ms reports, so the DWT cycle counter
     * is accumulated into microseconds instead (deltas are wrap-safe). */
    static uint32_t micros32() {
        static bool started = false;
        static uint32_t last_cycles = 0U;
        static uint32_t carry_cycles = 0U;
        static uint32_t accumulated_us = 0U;
        if (!started) {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            last_cycles = DWT->CYCCNT;
            started = true;
        }
        const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
        if (cycles_per_us == 0U) {
            return accumulated_us;
        }
        const uint32_t now_cycles = DWT->CYCCNT;
        const uint32_t delta_cycles = now_cycles - last_cycles;
        last_cycles = now_cycles;
        accumulated_us += delta_cycles / cycles_per_us;
        carry_cycles += delta_cycles % cycles_per_us;
        if (carry_cycles >= cycles_per_us) {
            accumulated_us += carry_cycles / cycles_per_us;
            carry_cycles %= cycles_per_us;
        }
        return accumulated_us;
    }

    static float fast_abs(float v) {
        return (v < 0.0f) ? -v : v;
    }

    static float fast_atan(float x) {
        const float ax = fast_abs(x);
        if (ax <= 1.0f) {
            return x / (1.0f + (0.28f * x * x));
        }
        const float inv = 1.0f / x;
        return (x > 0.0f ? kHalfPi : -kHalfPi) - (inv / (1.0f + (0.28f * inv * inv)));
    }

    static float fast_atan2(float y, float x) {
        if (x > 0.0f) {
            return fast_atan(y / x);
        }
        if (x < 0.0f) {
            if (y >= 0.0f) {
                return fast_atan(y / x) + kPi;
            }
            return fast_atan(y / x) - kPi;
        }
        if (y > 0.0f) {
            return kHalfPi;
        }
        if (y < 0.0f) {
            return -kHalfPi;
        }
        return 0.0f;
    }

    static float fast_asin(float x) {
        float clamped = x;
        if (clamped > 1.0f) {
            clamped = 1.0f;
        } else if (clamped < -1.0f) {
            clamped = -1.0f;
        }
        const float x2 = clamped * clamped;
        return clamped * (1.0f + (0.165f * x2) + (0.00761f * x2 * x2));
    }
    static bool is_finite_f32(float v) {
        union {
            float f;
            uint32_t u;
        } bits = { v };
        return (bits.u & 0x7F800000UL) != 0x7F800000UL;
    }

    static bool quat_is_valid(float w, float x, float y, float z) {
        if (!is_finite_f32(w) || !is_finite_f32(x) || !is_finite_f32(y) || !is_finite_f32(z)) {
            return false;
        }
        const float norm2 = (w * w) + (x * x) + (y * y) + (z * z);
        if (!is_finite_f32(norm2)) {
            return false;
        }
        return (norm2 > 0.01f) && (norm2 < 4.0f);
    }
    static void quat_to_rpy(float w, float x, float y, float z, float &roll_rad, float &pitch_rad, float &yaw_rad) {
        if (!quat_is_valid(w, x, y, z)) {
            roll_rad = 0.0f;
            pitch_rad = 0.0f;
            yaw_rad = 0.0f;
            return;
        }

        const float sinr_cosp = 2.0f * ((w * x) + (y * z));
        const float cosr_cosp = 1.0f - (2.0f * ((x * x) + (y * y)));
        roll_rad = fast_atan2(sinr_cosp, cosr_cosp);

        const float sinp = 2.0f * ((w * y) - (z * x));
        pitch_rad = fast_asin(sinp);

        const float siny_cosp = 2.0f * ((w * z) + (x * y));
        const float cosy_cosp = 1.0f - (2.0f * ((y * y) + (z * z)));
        yaw_rad = fast_atan2(siny_cosp, cosy_cosp);
    }

    static int open(sh2_Hal_t *) {
        if (active_ == nullptr) {
            return SH2_ERR_IO;
        }
        const HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&active_->bus_, active_->address_, 2U, 20U);
        active_->last_open_probe_hal_status_ = status;
        if (status == HAL_OK) {
            active_->last_i2c_error_ = 0U;
            return SH2_OK;
        }
        active_->last_i2c_error_ = HAL_I2C_GetError(&active_->bus_);
        return SH2_ERR_IO;
    }
    static void close(sh2_Hal_t *) {}

    static int read(sh2_Hal_t *, uint8_t *buffer, unsigned len, uint32_t *timestamp_us) {
        if (active_ == nullptr || len < kShtpHeaderLen) {
            return 0;
        }
        uint16_t read_len =
            (active_->next_read_len_ > len) ? static_cast<uint16_t>(len) : active_->next_read_len_;
        if (read_len > kMaxI2cReadLen) {
            read_len = kMaxI2cReadLen;
        }
        const HAL_StatusTypeDef read_status =
            HAL_I2C_Master_Receive(&active_->bus_, active_->address_, buffer, read_len, 20U);
        if (read_len == kShtpHeaderLen) {
            active_->last_read_header_hal_status_ = read_status;
        } else {
            active_->last_read_packet_hal_status_ = read_status;
        }
#if EXO_BNO85_I2C_TRACE
        debug.snprint("BNO I2C RX st=%d err=0x%08lX len=%u raw=[%02X %02X %02X %02X]\r\n",
                      static_cast<int>(read_status),
                      static_cast<unsigned long>(HAL_I2C_GetError(&active_->bus_)),
                      static_cast<unsigned>(read_len),
                      buffer[0], buffer[1], buffer[2], buffer[3]);
#endif
        if (read_status != HAL_OK) {
            active_->last_i2c_error_ = HAL_I2C_GetError(&active_->bus_);
            active_->next_read_len_ = kShtpHeaderLen;
            return 0;
        }
        active_->last_i2c_error_ = 0U;
        const uint16_t transfer_len = static_cast<uint16_t>((buffer[1] << 8U) | buffer[0]) & 0x7FFFU;
#if EXO_BNO85_I2C_TRACE
        debug.snprint("BNO I2C RX parsed transfer_len=%u chan=%u seq=%u\r\n",
                      static_cast<unsigned>(transfer_len),
                      static_cast<unsigned>(buffer[2]),
                      static_cast<unsigned>(buffer[3]));
#endif
        if (transfer_len < kShtpHeaderLen || transfer_len > len) {
#if EXO_BNO85_I2C_TRACE
            debug.snprint("BNO I2C RX invalid transfer_len=%u (max=%u)\r\n",
                          static_cast<unsigned>(transfer_len),
                          static_cast<unsigned>(len));
#endif
            active_->next_read_len_ = kShtpHeaderLen;
            return 0;
        }

        // Every BNO08x I2C read begins with a fresh SHTP header. If this
        // transaction only returned part of the cargo, the next read must ask
        // for the remainder plus the next transfer header.
        if (transfer_len > read_len) {
            active_->next_read_len_ =
                static_cast<uint16_t>((transfer_len - read_len) + kShtpHeaderLen);
        } else {
            active_->next_read_len_ = kShtpHeaderLen;
        }
#if EXO_BNO85_I2C_TRACE
        debug.snprint("BNO I2C RX-DONE got=%u next=%u first=[%02X %02X %02X %02X]\r\n",
                      static_cast<unsigned>(read_len),
                      static_cast<unsigned>(active_->next_read_len_),
                      buffer[0], buffer[1], buffer[2], buffer[3]);
#endif
        *timestamp_us = get_time_us(nullptr);
        return static_cast<int>(read_len);
    }

    static int write(sh2_Hal_t *, uint8_t *buffer, unsigned len) {
        if (active_ == nullptr) {
            return SH2_ERR_IO;
        }
#if EXO_BNO85_I2C_TRACE
        debug.snprint("BNO I2C TX len=%u hdr=[%02X %02X %02X %02X]\r\n",
                      static_cast<unsigned>(len),
                      len > 0U ? buffer[0] : 0U,
                      len > 1U ? buffer[1] : 0U,
                      len > 2U ? buffer[2] : 0U,
                      len > 3U ? buffer[3] : 0U);
#endif
        // Avoid infinite retry loops in shtp.c when the bus stays busy.
        for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
            const HAL_StatusTypeDef status =
                HAL_I2C_Master_Transmit(&active_->bus_, active_->address_, buffer, len, 20U);
            active_->last_write_hal_status_ = status;
#if EXO_BNO85_I2C_TRACE
            debug.snprint("BNO I2C TX attempt=%u st=%d err=0x%08lX\r\n",
                          static_cast<unsigned>(attempt),
                          static_cast<int>(status),
                          static_cast<unsigned long>(HAL_I2C_GetError(&active_->bus_)));
#endif
            if (status == HAL_OK) {
                active_->last_i2c_error_ = 0U;
                return static_cast<int>(len);
            }
            if (status != HAL_BUSY) {
                active_->last_i2c_error_ = HAL_I2C_GetError(&active_->bus_);
                return SH2_ERR_IO;
            }
            HAL_Delay(1U);
        }
        active_->last_i2c_error_ = HAL_I2C_GetError(&active_->bus_);
        return SH2_ERR_IO;
    }

    static uint32_t get_time_us(sh2_Hal_t *) {
        return micros32();
    }

    static void async_event_callback(void *, sh2_AsyncEvent_t *) {}

    static void sensor_event_callback(void *, sh2_SensorEvent_t *event) {
        if (active_ == nullptr) {
            return;
        }
        active_->sensor_event_count_++;
        active_->last_report_id_ = event->reportId;
#if EXO_ACQ_DIAG_ENABLE
        /* Counted before decode so undecodable traffic still shows up as bus
         * load. Runs in the sh2_service() foreground call, never in an ISR. */
        active_->report_counts_[::exo::diag::bno_slot_for_report(event->reportId)]++;
#endif
        active_->last_decode_status_ = sh2_decodeSensorEvent(&active_->latest_, event);
        if (active_->last_decode_status_ == SH2_OK) {
            const uint8_t id = active_->latest_.sensorId;
            switch (id) {
            case SH2_GAME_ROTATION_VECTOR:
                active_->latest_quat_i_ = active_->latest_.un.gameRotationVector.i;
                active_->latest_quat_j_ = active_->latest_.un.gameRotationVector.j;
                active_->latest_quat_k_ = active_->latest_.un.gameRotationVector.k;
                active_->latest_quat_real_ = active_->latest_.un.gameRotationVector.real;
                active_->latest_available_mask_ |= 0x01U;
                active_->has_rotation_ = true;
                break;
            case SH2_LINEAR_ACCELERATION:
                active_->latest_linear_accel_x_ = active_->latest_.un.linearAcceleration.x;
                active_->latest_linear_accel_y_ = active_->latest_.un.linearAcceleration.y;
                active_->latest_linear_accel_z_ = active_->latest_.un.linearAcceleration.z;
                active_->latest_available_mask_ |= 0x02U;
                break;
            case SH2_GRAVITY:
                active_->latest_gravity_x_ = active_->latest_.un.gravity.x;
                active_->latest_gravity_y_ = active_->latest_.un.gravity.y;
                active_->latest_gravity_z_ = active_->latest_.un.gravity.z;
                active_->latest_available_mask_ |= 0x04U;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                active_->latest_gyro_x_ = active_->latest_.un.gyroscope.x;
                active_->latest_gyro_y_ = active_->latest_.un.gyroscope.y;
                active_->latest_gyro_z_ = active_->latest_.un.gyroscope.z;
                active_->latest_available_mask_ |= 0x08U;
                break;
            default:
                break;
            }

            active_->latest_sensor_id_ = id;
            active_->latest_status_ = active_->latest_.status;
            active_->latest_sequence_ = active_->latest_.sequence;
            active_->latest_sensor_timestamp_us_ = static_cast<uint32_t>(active_->latest_.timestamp);
            active_->latest_delay_us_ = active_->latest_.delay;
            active_->has_new_data_ = true;
            /* The anchor report seals one queued sample from the merged
             * latest values; auxiliary reports only refresh the merge. */
            if (id == SH2_GAME_ROTATION_VECTOR && active_->capture_queue_enabled_) {
                active_->push_capture_sample_();
            }
        }
    }

    void push_capture_sample_() {
        if (q_count_ >= EXO_BNO85_SAMPLE_QUEUE_DEPTH) {
            ++queue_drops_;
            return;
        }
        Bno85Sample sample { };
        /* offset_us is assigned on pop from the uniform report grid. */
        sample.quat_i = latest_quat_i_;
        sample.quat_j = latest_quat_j_;
        sample.quat_k = latest_quat_k_;
        sample.quat_real = latest_quat_real_;
        sample.linear_accel_x = latest_linear_accel_x_;
        sample.linear_accel_y = latest_linear_accel_y_;
        sample.linear_accel_z = latest_linear_accel_z_;
        sample.gravity_x = latest_gravity_x_;
        sample.gravity_y = latest_gravity_y_;
        sample.gravity_z = latest_gravity_z_;
        sample.gyro_x = latest_gyro_x_;
        sample.gyro_y = latest_gyro_y_;
        sample.gyro_z = latest_gyro_z_;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        sample.roll_rad = 0.0f;
        sample.pitch_rad = 0.0f;
        sample.yaw_rad = 0.0f;
        sample.accuracy = latest_status_;
        sample.status = latest_sensor_id_;
        sample.report_id = last_report_id_;
        sample.sequence = latest_sequence_;
        sample.available_mask = latest_available_mask_;
        sample.reserved0 = 0U;
        sample.sensor_timestamp_us = latest_sensor_timestamp_us_;
        sample.delay_us = latest_delay_us_;
#endif
        queue_[(q_head_ + q_count_) & kQueueMask] = sample;
        ++q_count_;
    }

    I2C_HandleTypeDef &bus_;
    uint16_t address_;
    sh2_Hal_t hal_{};
    sh2_SensorValue_t latest_{};
    bool has_rotation_ = false;
    bool has_new_data_ = false;
    float latest_quat_i_ = 0.0f;
    float latest_quat_j_ = 0.0f;
    float latest_quat_k_ = 0.0f;
    float latest_quat_real_ = 1.0f;
    float latest_linear_accel_x_ = 0.0f;
    float latest_linear_accel_y_ = 0.0f;
    float latest_linear_accel_z_ = 0.0f;
    float latest_gravity_x_ = 0.0f;
    float latest_gravity_y_ = 0.0f;
    float latest_gravity_z_ = 0.0f;
    float latest_gyro_x_ = 0.0f;
    float latest_gyro_y_ = 0.0f;
    float latest_gyro_z_ = 0.0f;
    uint8_t latest_sensor_id_ = 0U;
    uint8_t latest_status_ = 0U;
    uint8_t latest_sequence_ = 0U;
    uint8_t latest_available_mask_ = 0U;
    uint32_t latest_sensor_timestamp_us_ = 0U;
    uint32_t latest_delay_us_ = 0U;
    int last_begin_status_ = SH2_ERR;
    int begin_open_status_ = SH2_ERR;
    int begin_callback_status_ = SH2_ERR;
    int begin_config_status_ = SH2_ERR;
    HAL_StatusTypeDef last_open_probe_hal_status_ = HAL_ERROR;
    HAL_StatusTypeDef last_read_header_hal_status_ = HAL_ERROR;
    HAL_StatusTypeDef last_read_packet_hal_status_ = HAL_ERROR;
    HAL_StatusTypeDef last_write_hal_status_ = HAL_ERROR;
    uint32_t last_i2c_error_ = 0U;
    uint32_t sensor_event_count_ = 0U;
#if EXO_ACQ_DIAG_ENABLE
    uint32_t report_counts_[::exo::diag::kBnoSlotCount] = {0U, 0U, 0U, 0U, 0U};
#endif
    uint8_t last_report_id_ = 0U;
    int last_decode_status_ = SH2_ERR;
    uint16_t next_read_len_ = kShtpHeaderLen;
    GPIO_TypeDef *interrupt_port_ = nullptr;
    uint16_t interrupt_pin_ = 0U;
    Bno85Sample queue_[EXO_BNO85_SAMPLE_QUEUE_DEPTH] { };
    uint8_t q_head_ = 0U;
    uint8_t q_count_ = 0U;
    uint32_t queue_drops_ = 0U;
    bool capture_queue_enabled_ = false;
    bool pop_offset_valid_ = false;
    uint32_t pop_offset_us_ = 0U;
    inline static Bno85Stm32 *active_ = nullptr;
};

} // namespace exo

#endif
