#ifndef ICM45686_STM32_H_
#define ICM45686_STM32_H_

#include <string.h>

extern "C" {
#include "i2c.h"
#include "motion.mcu.icm45686.driver/icm45686/imu/inv_imu_driver.h"
}

#include "RECORDING_TYPES.h"

namespace exo {

#ifndef EXO_ICM45686_ACCEL_ODR
#define EXO_ICM45686_ACCEL_ODR ACCEL_CONFIG0_ACCEL_ODR_400_HZ
#endif

#ifndef EXO_ICM45686_GYRO_ODR
#define EXO_ICM45686_GYRO_ODR GYRO_CONFIG0_GYRO_ODR_400_HZ
#endif

class Icm45686Stm32 {
public:
    Icm45686Stm32(I2C_HandleTypeDef &bus, uint8_t address_7bit)
        : bus_(bus), address_(static_cast<uint16_t>(address_7bit << 1U)) {
        memset(&device_, 0, sizeof(device_));
        device_.transport.context = this;
        device_.transport.read_reg = &Icm45686Stm32::read_reg;
        device_.transport.write_reg = &Icm45686Stm32::write_reg;
        device_.transport.serif_type = UI_I2C;
        device_.transport.sleep_us = &Icm45686Stm32::sleep_us;
    }

    bool begin() {
        recover_i2c_bus();

        uint8_t warmup_frame[20] = {0};
        for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
            const HAL_StatusTypeDef warmup_status =
                    HAL_I2C_Master_Receive(&bus_, address_, warmup_frame, sizeof(warmup_frame), 20U);
            if (warmup_status == HAL_BUSY) {
                recover_i2c_bus();
            }
            HAL_Delay(2U);
        }

        uint8_t whoami = 0U;
        if (inv_imu_get_who_am_i(&device_, &whoami) != 0 || whoami != INV_IMU_WHOAMI) {
            last_read_status_ = -1;
            return false;
        }
        if (inv_imu_soft_reset(&device_) != 0) {
            last_read_status_ = -2;
            return false;
        }
        const bool configured = inv_imu_set_accel_fsr(&device_, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G) == 0 &&
               inv_imu_set_gyro_fsr(&device_, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) == 0 &&
               inv_imu_set_accel_frequency(&device_, EXO_ICM45686_ACCEL_ODR) == 0 &&
               inv_imu_set_gyro_frequency(&device_, EXO_ICM45686_GYRO_ODR) == 0 &&
               inv_imu_set_accel_mode(&device_, PWR_MGMT0_ACCEL_MODE_LN) == 0 &&
               inv_imu_set_gyro_mode(&device_, PWR_MGMT0_GYRO_MODE_LN) == 0;
        last_read_status_ = configured ? 0 : -3;
        return configured;
    }

    bool read_sample(uint32_t offset_us, Icm45686Sample &sample) {
        if (fifo_active_) {
            return false;
        }
        (void)offset_us;
        inv_imu_sensor_data_t raw;
        const int status = inv_imu_get_register_data(&device_, &raw);
        if (status != 0) {
            last_read_status_ = static_cast<int8_t>(status);
#if EXO_SAMPLE_FORMAT_VERSION == 2U || EXO_SAMPLE_FORMAT_VERSION == 4U
            sample.offset_us = offset_us;
#if EXO_SAMPLE_FORMAT_VERSION == 4U
            sample.sequence = 0U;
#else
            sample.data_valid = 0U;
            sample.read_status = last_read_status_;
            sample.sample_timestamp_us = offset_us;
#endif
#endif
            return false;
        }
        last_read_status_ = 0;
#if EXO_SAMPLE_FORMAT_VERSION == 2U || EXO_SAMPLE_FORMAT_VERSION == 4U
        sample.offset_us = offset_us;
#endif
#if EXO_SAMPLE_FORMAT_VERSION == 4U
        sample.sequence = next_sequence_++;
#endif
        sample.accel_x = raw.accel_data[0];
        sample.accel_y = raw.accel_data[1];
        sample.accel_z = raw.accel_data[2];
        sample.gyro_x = raw.gyro_data[0];
        sample.gyro_y = raw.gyro_data[1];
        sample.gyro_z = raw.gyro_data[2];
#if EXO_SAMPLE_FORMAT_VERSION == 2U
        sample.temperature = raw.temp_data;
        sample.data_valid = 1U;
        sample.read_status = last_read_status_;
        sample.reserved0 = 0U;
        sample.sample_timestamp_us = offset_us;
#endif
        return true;
    }

    /* Recording path shared by Master and Nodes. The FIFO captures real
     * 200 Hz accel+gyro frames independently of the foreground superloop,
     * so draining several frames later
     * never fabricates repeated register snapshots. */
    bool begin_fifo_capture_200hz() {
        if (fifo_active_) return true;
#if EXO_SAMPLE_FORMAT_VERSION != 4U
        return false;
#else
        next_sequence_ = 0U;
        fifo_elapsed_us_ = 0U;
        fifo_timestamp_valid_ = false;

        tmst_wom_config_t timestamp_config{};
        int status = inv_imu_read_reg(&device_, TMST_WOM_CONFIG, 1U,
                reinterpret_cast<uint8_t *>(&timestamp_config));
        if (status == 0) {
            timestamp_config.tmst_resol = TMST_WOM_CONFIG_TMST_RESOL_16_US;
            status = inv_imu_write_reg(&device_, TMST_WOM_CONFIG, 1U,
                    reinterpret_cast<const uint8_t *>(&timestamp_config));
        }
        if (status == 0) status = inv_imu_set_accel_frequency(&device_, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
        if (status == 0) status = inv_imu_set_gyro_frequency(&device_, GYRO_CONFIG0_GYRO_ODR_200_HZ);
        /* Low-noise anti-alias bandwidth = ODR/4, matching the InvenSense
         * basic_read_fifo reference for a clean 200 Hz capture. */
        if (status == 0) status = inv_imu_set_accel_ln_bw(&device_, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
        if (status == 0) status = inv_imu_set_gyro_ln_bw(&device_, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);

        /* Documented read-modify-write: start from the device's current FIFO
         * configuration so driver-managed reserved fields are preserved, then
         * enable the combined accel+gyro stream. */
        inv_imu_fifo_config_t fifo{};
        if (status == 0) status = inv_imu_get_fifo_config(&device_, &fifo);
        fifo.accel_en = INV_IMU_ENABLE;
        fifo.gyro_en = INV_IMU_ENABLE;
        fifo.hires_en = INV_IMU_DISABLE;
        fifo.fifo_wm_th = 1U;
        fifo.fifo_mode = FIFO_CONFIG0_FIFO_MODE_STREAM;
        fifo.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX;
        if (status == 0) status = inv_imu_set_fifo_config(&device_, &fifo);
        if (status == 0) status = inv_imu_flush_fifo(&device_);
        if (status != 0) {
            fifo.fifo_mode = FIFO_CONFIG0_FIFO_MODE_BYPASS;
            (void)inv_imu_set_fifo_config(&device_, &fifo);
            (void)inv_imu_set_accel_frequency(&device_, EXO_ICM45686_ACCEL_ODR);
            (void)inv_imu_set_gyro_frequency(&device_, EXO_ICM45686_GYRO_ODR);
            last_read_status_ = static_cast<int8_t>(status);
            return false;
        }
        fifo_active_ = true;
        last_read_status_ = 0;
        return true;
#endif
    }

    uint8_t read_fifo_samples(Icm45686Sample *samples, uint8_t capacity) {
#if EXO_SAMPLE_FORMAT_VERSION != 4U
        (void)samples;
        (void)capacity;
        return 0U;
#else
        if (!fifo_active_ || samples == nullptr || capacity == 0U) return 0U;
        uint16_t frame_count = 0U;
        int status = inv_imu_get_frame_count(&device_, &frame_count);
        if (status != 0) {
            last_read_status_ = static_cast<int8_t>(status);
            return 0U;
        }
        const uint16_t requested = frame_count < capacity ? frame_count : capacity;
        uint8_t produced = 0U;
        for (uint16_t i = 0U; i < requested; ++i) {
            inv_imu_fifo_data_t frame{};
            status = inv_imu_get_fifo_frame(&device_, &frame);
            if (status != 0) {
                last_read_status_ = static_cast<int8_t>(status);
                break;
            }
            if (frame.header.bits.accel_bit == 0U || frame.header.bits.gyro_bit == 0U ||
                    frame.header.bits.timestamp_bit == 0U) {
                continue;
            }
            /* Documented invalid-sample sentinel (0x8000): a frame carrying it on
             * any accel/gyro axis is a settling/unavailable artefact, not real
             * sensor data, so it must not enter the recorded stream. */
            if (frame.byte_16.accel_data[0] == INVALID_VALUE_FIFO ||
                    frame.byte_16.accel_data[1] == INVALID_VALUE_FIFO ||
                    frame.byte_16.accel_data[2] == INVALID_VALUE_FIFO ||
                    frame.byte_16.gyro_data[0] == INVALID_VALUE_FIFO ||
                    frame.byte_16.gyro_data[1] == INVALID_VALUE_FIFO ||
                    frame.byte_16.gyro_data[2] == INVALID_VALUE_FIFO) {
                continue;
            }

            const uint16_t timestamp = frame.byte_16.timestamp;
            if (!fifo_timestamp_valid_) {
                fifo_timestamp_valid_ = true;
                fifo_last_timestamp_ = timestamp;
                fifo_elapsed_us_ = 0U;
            } else {
                uint32_t delta_us = static_cast<uint32_t>(
                        static_cast<uint16_t>(timestamp - fifo_last_timestamp_)) * 16U;
                fifo_last_timestamp_ = timestamp;
                /* At 200 Hz a valid delta is close to 5000 us. A zero timestamp
                 * cannot provide ordering, so reject that frame rather than
                 * inventing a duplicate time. */
                if (delta_us == 0U) continue;
                /* The 16-bit TMST wraps every 1.049 s: a stall at or beyond one
                 * wrap yields a garbage (small) delta that would silently
                 * compress offset_us. Substitute the nominal period so the
                 * timeline stays monotonic; the sequence gap already exposes
                 * the lost frames. */
                if (delta_us > kFifoMaxPlausibleDeltaUs) {
                    delta_us = kFifoNominalPeriodUs;
                }
                fifo_elapsed_us_ += delta_us;
            }

            Icm45686Sample sample{};
            sample.offset_us = fifo_elapsed_us_;
            sample.sequence = next_sequence_++;
            sample.accel_x = frame.byte_16.accel_data[0];
            sample.accel_y = frame.byte_16.accel_data[1];
            sample.accel_z = frame.byte_16.accel_data[2];
            sample.gyro_x = frame.byte_16.gyro_data[0];
            sample.gyro_y = frame.byte_16.gyro_data[1];
            sample.gyro_z = frame.byte_16.gyro_data[2];
            samples[produced++] = sample;
        }
        if (produced > 0U) last_read_status_ = 0;
        return produced;
#endif
    }

    bool end_fifo_capture() {
        if (!fifo_active_) return true;
        inv_imu_fifo_config_t fifo{};
        fifo.fifo_mode = FIFO_CONFIG0_FIFO_MODE_BYPASS;
        fifo.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX;
        int status = inv_imu_set_fifo_config(&device_, &fifo);
        status |= inv_imu_set_accel_frequency(&device_, EXO_ICM45686_ACCEL_ODR);
        status |= inv_imu_set_gyro_frequency(&device_, EXO_ICM45686_GYRO_ODR);
        fifo_active_ = false;
        if (status != 0) last_read_status_ = static_cast<int8_t>(status);
        return status == 0;
    }

    bool fifo_active() const { return fifo_active_; }
    /* Status of the most recent FIFO/capture operation: 0 = ok, else the
     * transport/config error code. Lets consumers distinguish an I2C error
     * from a legitimately empty FIFO when a read returns zero frames. */
    int8_t last_read_status() const { return last_read_status_; }

private:
    /* TMST wrap guard: at 200 Hz a frame delta is ~5000 us; anything beyond
     * 100 ms means the 16-bit timer wrapped during a stall. */
    static constexpr uint32_t kFifoNominalPeriodUs = 5000U;
    static constexpr uint32_t kFifoMaxPlausibleDeltaUs = 100000U;
    void recover_i2c_bus() {
        if (HAL_I2C_GetState(&bus_) == HAL_I2C_STATE_READY) {
            return;
        }
        (void)HAL_I2C_DeInit(&bus_);
        HAL_Delay(2U);
        (void)HAL_I2C_Init(&bus_);
        (void)HAL_I2CEx_ConfigAnalogFilter(&bus_, I2C_ANALOGFILTER_DISABLE);
        (void)HAL_I2CEx_ConfigDigitalFilter(&bus_, 0U);
        HAL_Delay(2U);
    }

    static int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len) {
        auto *self = static_cast<Icm45686Stm32 *>(context);
        HAL_StatusTypeDef status =
                HAL_I2C_Mem_Read(&self->bus_, self->address_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100U);
        if (status == HAL_BUSY) {
            self->recover_i2c_bus();
            status = HAL_I2C_Mem_Read(&self->bus_, self->address_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100U);
        }
        return status == HAL_OK ? 0 : -1;
    }

    static int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len) {
        auto *self = static_cast<Icm45686Stm32 *>(context);
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&self->bus_, self->address_, reg, I2C_MEMADD_SIZE_8BIT,
                                                     const_cast<uint8_t *>(buf), len, 100U);
        if (status == HAL_BUSY) {
            self->recover_i2c_bus();
            status = HAL_I2C_Mem_Write(&self->bus_, self->address_, reg, I2C_MEMADD_SIZE_8BIT,
                                       const_cast<uint8_t *>(buf), len, 100U);
        }
        return status == HAL_OK ? 0 : -1;
    }

    static void sleep_us(uint32_t us) {
        const uint32_t start = HAL_GetTick();
        const uint32_t wait_ms = (us + 999U) / 1000U;
        while ((HAL_GetTick() - start) < wait_ms) {
        }
    }

    I2C_HandleTypeDef &bus_;
    uint16_t address_;
    inv_imu_device_t device_;
    int8_t last_read_status_ = 0;
#if EXO_SAMPLE_FORMAT_VERSION == 4U
    uint32_t next_sequence_ = 0U;
    bool fifo_active_ = false;
    bool fifo_timestamp_valid_ = false;
    uint16_t fifo_last_timestamp_ = 0U;
    uint32_t fifo_elapsed_us_ = 0U;
#endif
};

} // namespace exo

#endif
