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

private:
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
#endif
};

} // namespace exo

#endif
