#ifndef W25Q256_FLASH_H_
#define W25Q256_FLASH_H_

#include <stdint.h>

extern "C" {
#include "spi.h"
#include "w25qxx/src/driver_w25qxx.h"
}

#include <exo/storage/node_recorder.h>

namespace exo {

#if defined(STM32H7xx)
#define EXO_FLASH_PLATFORM_STM32H7 1
#else
#define EXO_FLASH_PLATFORM_STM32H7 0
#endif

class W25Q256Flash final : public SessionFlash {
public:
    enum class InitError : uint8_t {
        None = 0,
        LinkSetupFailed,
        SetTypeFailed,
        SetInterfaceFailed,
        SetModeFailed,
        InitFailed,
        JedecReadFailed,
        ManufacturerMismatch,
        AddressModeFailed,
    };

    struct DebugInfo {
        InitError init_error;
        uint8_t jedec_manufacturer;
        uint8_t jedec_device_hi;
        uint8_t jedec_device_lo;
        uint8_t last_driver_result;
        uint8_t last_spi_instruction;
        uint8_t last_hal_status;
        uint8_t last_spi_stage;
        uint32_t last_header_len;
        uint32_t last_in_len;
        uint32_t last_out_len;
    };

    W25Q256Flash(SPI_HandleTypeDef &bus, GPIO_TypeDef *cs_port, uint16_t cs_pin)
        : bus_(bus), cs_port_(cs_port), cs_pin_(cs_pin) {}

    bool begin() {
        last_error_ = InitError::None;
        last_driver_result_ = 0U;
        active_ = this;
        GPIO_InitTypeDef cs_gpio = {0};
        cs_gpio.Pin = cs_pin_;
        cs_gpio.Mode = GPIO_MODE_OUTPUT_PP;
        cs_gpio.Pull = GPIO_PULLUP;
        cs_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(cs_port_, &cs_gpio);
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        DRIVER_W25QXX_LINK_INIT(&handle_, w25qxx_handle_t);
        DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&handle_, &W25Q256Flash::spi_init);
        DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&handle_, &W25Q256Flash::spi_deinit);
        DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&handle_, &W25Q256Flash::spi_write_read);
        DRIVER_W25QXX_LINK_DELAY_MS(&handle_, &W25Q256Flash::delay_ms);
        DRIVER_W25QXX_LINK_DELAY_US(&handle_, &W25Q256Flash::delay_us);
        DRIVER_W25QXX_LINK_DEBUG_PRINT(&handle_, &W25Q256Flash::debug_print);
        if (handle_.spi_qspi_write_read == nullptr) {
            last_error_ = InitError::LinkSetupFailed;
            return false;
        }

        last_driver_result_ = w25qxx_set_type(&handle_, W25Q256);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::SetTypeFailed;
            return false;
        }
        last_driver_result_ = w25qxx_set_interface(&handle_, W25QXX_INTERFACE_SPI);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::SetInterfaceFailed;
            return false;
        }
        last_driver_result_ = w25qxx_set_dual_quad_spi(&handle_, W25QXX_BOOL_FALSE);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::SetModeFailed;
            return false;
        }
        last_driver_result_ = w25qxx_init(&handle_);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::InitFailed;
            return false;
        }

        uint8_t manufacturer = 0U;
        uint8_t device_id[2] = {0U, 0U};
        last_driver_result_ = w25qxx_get_jedec_id(&handle_, &manufacturer, device_id);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::JedecReadFailed;
            return false;
        }
        jedec_manufacturer_ = manufacturer;
        jedec_device_hi_ = device_id[0];
        jedec_device_lo_ = device_id[1];
        if (manufacturer != kWinbondManufacturerId) {
            last_error_ = InitError::ManufacturerMismatch;
            return false;
        }
        last_driver_result_ = w25qxx_set_address_mode(&handle_, W25QXX_ADDRESS_MODE_4_BYTE);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::AddressModeFailed;
            return false;
        }
        last_driver_result_ = w25qxx_set_address_mode(&handle_, W25QXX_ADDRESS_MODE_4_BYTE);
        if (last_driver_result_ != 0U) {
            last_error_ = InitError::AddressModeFailed;
            return false;
        }

        last_error_ = InitError::None;
        return true;
    }

    bool erase_region(uint32_t address, uint32_t size) override {
        if (size == 0U || (address + size) < address) {
            return false;
        }
        const uint32_t end = address + size;
        for (uint32_t cursor = address; cursor < end; cursor += kSectorSize) {
            if (w25qxx_sector_erase_4k(&handle_, cursor) != 0) {
                return false;
            }
        }
        return true;
    }

    bool erase_4k(uint32_t address) override {
        return w25qxx_sector_erase_4k(&handle_, address) == 0;
    }

    bool write(uint32_t address, const void *data, uint32_t size) override {
        if (data == nullptr || size == 0U) {
            return false;
        }
        return w25qxx_write(&handle_, address, const_cast<uint8_t *>(static_cast<const uint8_t *>(data)), size) == 0;
    }

    /* Pre-erased fast path: verify the target range is still 0xFF (a read of
     * just the written span, not a 4 KB sector RMW), then page-program it
     * directly. Falls back to the safe write if the region is not blank. */
    bool write_pre_erased(uint32_t address, const void *data, uint32_t size) override {
        const uint8_t *bytes = static_cast<const uint8_t *>(data);
        if (bytes == nullptr || size == 0U) {
            return false;
        }
        uint8_t verify[64] = {0};
        uint32_t offset = 0U;
        while (offset < size) {
            const uint32_t chunk = (size - offset) > sizeof(verify) ? sizeof(verify) : (size - offset);
            if (w25qxx_read(&handle_, address + offset, verify, chunk) != 0) {
                return false;
            }
            for (uint32_t i = 0U; i < chunk; ++i) {
                if (verify[i] != 0xFFU) {
                    return write(address, data, size);
                }
            }
            offset += chunk;
        }
        return w25qxx_write_no_check(&handle_, address, const_cast<uint8_t *>(bytes), size) == 0;
    }

    bool read(uint32_t address, void *data, uint32_t size) override {
        if (data == nullptr || size == 0U) {
            return false;
        }
        return w25qxx_read(&handle_, address, static_cast<uint8_t *>(data), size) == 0;
    }

    bool get_jedec_id(uint8_t &manufacturer, uint8_t &device_id_hi, uint8_t &device_id_lo) {
        uint8_t device_id[2] = {0U, 0U};
        if (w25qxx_get_jedec_id(&handle_, &manufacturer, device_id) != 0) {
            return false;
        }
        device_id_hi = device_id[0];
        device_id_lo = device_id[1];
        return true;
    }

    uint32_t detected_capacity_bytes() const {
        switch (jedec_device_lo_) {
            case 0x15U: return 2UL * 1024UL * 1024UL;
            case 0x16U: return 4UL * 1024UL * 1024UL;
            case 0x17U: return 8UL * 1024UL * 1024UL;
            case 0x18U: return 16UL * 1024UL * 1024UL;
            case 0x19U: return 32UL * 1024UL * 1024UL;
            default: return 0U;
        }
    }

    bool random_write_read_test_128(uint32_t address, uint32_t seed, uint16_t &mismatch_count, uint16_t &first_mismatch_index,
                                    uint8_t *written_out = nullptr, uint8_t *read_out = nullptr) {
        mismatch_count = 0U;
        first_mismatch_index = 0xFFFFU;

        if ((address % kSectorSize) != 0U) {
            return false;
        }

        uint8_t tx[kTestSize] = {0};
        uint8_t rx[kTestSize] = {0};
        uint32_t state = seed ^ 0xA5A55A5AU;
        for (uint32_t i = 0U; i < kTestSize; ++i) {
            state = (state * 1664525UL) + 1013904223UL;
            tx[i] = static_cast<uint8_t>((state >> 24U) & 0xFFU);
        }

        if (!erase_region(address, kSectorSize)) {
            return false;
        }
        if (!write(address, tx, kTestSize)) {
            return false;
        }
        if (!read(address, rx, kTestSize)) {
            return false;
        }

        for (uint16_t i = 0U; i < kTestSize; ++i) {
            if (tx[i] != rx[i]) {
                if (first_mismatch_index == 0xFFFFU) {
                    first_mismatch_index = i;
                }
                ++mismatch_count;
            }
        }
        if (written_out != nullptr) {
            for (uint32_t i = 0U; i < kTestSize; ++i) {
                written_out[i] = tx[i];
            }
        }
        if (read_out != nullptr) {
            for (uint32_t i = 0U; i < kTestSize; ++i) {
                read_out[i] = rx[i];
            }
        }

        return mismatch_count == 0U;
    }

    const char *last_error_string() const {
        switch (last_error_) {
            case InitError::None: return "none";
            case InitError::LinkSetupFailed: return "link_setup_failed";
            case InitError::SetTypeFailed: return "set_type_failed";
            case InitError::SetInterfaceFailed: return "set_interface_failed";
            case InitError::SetModeFailed: return "set_mode_failed";
            case InitError::InitFailed: return "driver_init_failed";
            case InitError::JedecReadFailed: return "jedec_read_failed";
            case InitError::ManufacturerMismatch: return "manufacturer_mismatch";
            case InitError::AddressModeFailed: return "address_mode_4byte_failed";
            default: return "unknown";
        }
    }

    DebugInfo debug_info() const {
        DebugInfo info{};
        info.init_error = last_error_;
        info.jedec_manufacturer = jedec_manufacturer_;
        info.jedec_device_hi = jedec_device_hi_;
        info.jedec_device_lo = jedec_device_lo_;
        info.last_driver_result = last_driver_result_;
        info.last_spi_instruction = last_spi_instruction_;
        info.last_hal_status = last_hal_status_;
        info.last_spi_stage = last_spi_stage_;
        info.last_header_len = last_header_len_;
        info.last_in_len = last_in_len_;
        info.last_out_len = last_out_len_;
        return info;
    }

private:
    static constexpr uint32_t kSectorSize = 4096U;
    static constexpr uint32_t kTestSize = 128U;
    static constexpr uint8_t kWinbondManufacturerId = 0xEFU;

    static uint8_t spi_init() { return 0U; }
    static uint8_t spi_deinit() { return 0U; }

    static uint8_t spi_write_read(uint8_t instruction, uint8_t instruction_line,
                                  uint32_t address, uint8_t address_line, uint8_t address_len,
                                  uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                  uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                  uint8_t *out_buf, uint32_t out_len, uint8_t data_line) {
        (void)alternate;
        (void)alternate_line;
        (void)alternate_len;
        if (active_ == nullptr || instruction_line > 1U || address_line > 1U || data_line > 1U) {
            return 1U;
        }

        uint8_t header[16] = {0};
        uint32_t header_len = 0U;
        if (instruction_line != 0U) {
            header[header_len++] = instruction;
        }
        if (address_line != 0U && address_len != 0U) {
            for (int8_t shift = static_cast<int8_t>((address_len - 1U) * 8U); shift >= 0; shift -= 8) {
                header[header_len++] = static_cast<uint8_t>((address >> shift) & 0xFFU);
            }
        }
        for (uint8_t i = 0U; i < dummy; ++i) {
            if (header_len >= sizeof(header)) {
                active_->last_spi_stage_ = 0xE1U;
                return 1U;
            }
            header[header_len++] = 0x00U;
        }
        active_->last_spi_instruction_ = instruction;
        active_->last_header_len_ = header_len;
        active_->last_in_len_ = in_len;
        active_->last_out_len_ = out_len;
        active_->last_spi_stage_ = 0U;

        HAL_GPIO_WritePin(active_->cs_port_, active_->cs_pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_OK;
        if (header_len > 0U) {
            status = HAL_SPI_Transmit(&active_->bus_, header, header_len, 100U);
            active_->last_spi_stage_ = 1U;
        } else {
            active_->last_spi_stage_ = 0x10U;
        }
        if (status == HAL_OK && in_len > 0U) {
            status = HAL_SPI_Transmit(&active_->bus_, in_buf, in_len, 100U);
            active_->last_spi_stage_ = 3U;
        }
        if (status == HAL_OK && out_len > 0U) {
            status = HAL_SPI_Receive(&active_->bus_, out_buf, out_len, 100U);
            active_->last_spi_stage_ = 4U;
        }
        HAL_GPIO_WritePin(active_->cs_port_, active_->cs_pin_, GPIO_PIN_SET);
        active_->last_hal_status_ = static_cast<uint8_t>(status);
        if (status != HAL_OK) {
            active_->last_spi_stage_ = 0xF0U + active_->last_spi_stage_;
        }
        return status == HAL_OK ? 0U : 1U;
    }

    static void delay_ms(uint32_t ms) { HAL_Delay(ms); }

    static void delay_us(uint32_t us) {
        const uint32_t start = HAL_GetTick();
        const uint32_t wait_ms = (us + 999U) / 1000U;
        while ((HAL_GetTick() - start) < wait_ms) {
        }
    }

    static void debug_print(const char *const, ...) {}

    SPI_HandleTypeDef &bus_;
    GPIO_TypeDef *cs_port_;
    uint16_t cs_pin_;
    w25qxx_handle_t handle_{};
    InitError last_error_ = InitError::None;
    uint8_t last_driver_result_ = 0U;
    uint8_t jedec_manufacturer_ = 0U;
    uint8_t jedec_device_hi_ = 0U;
    uint8_t jedec_device_lo_ = 0U;
    uint8_t last_spi_instruction_ = 0U;
    uint8_t last_hal_status_ = static_cast<uint8_t>(HAL_OK);
    uint8_t last_spi_stage_ = 0U;
    uint32_t last_header_len_ = 0U;
    uint32_t last_in_len_ = 0U;
    uint32_t last_out_len_ = 0U;
    inline static W25Q256Flash *active_ = nullptr;
};

} // namespace exo

#endif
