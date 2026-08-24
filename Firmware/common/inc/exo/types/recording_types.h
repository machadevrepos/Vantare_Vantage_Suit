#ifndef RECORDING_TYPES_H_
#define RECORDING_TYPES_H_

#include <stddef.h>
#include <stdint.h>

namespace exo {

static constexpr uint32_t kSessionMagic = 0x584F5345UL; // "ESOX"
#ifndef EXO_SAMPLE_FORMAT_VERSION
#define EXO_SAMPLE_FORMAT_VERSION 4U
#endif

#if (EXO_SAMPLE_FORMAT_VERSION != 2U) && (EXO_SAMPLE_FORMAT_VERSION != 3U) && \
    (EXO_SAMPLE_FORMAT_VERSION != 4U)
#error "EXO_SAMPLE_FORMAT_VERSION must be 2, 3, or 4"
#endif

static constexpr uint16_t kSessionFormatVersion = static_cast<uint16_t>(EXO_SAMPLE_FORMAT_VERSION);
static constexpr uint8_t kSessionComplete = 0xA5U;
static constexpr uint8_t kSensorBno85 = 0x01U;
static constexpr uint8_t kSensorIcm45686 = 0x02U;
static constexpr uint32_t kSessionLossBnoRead = 0x00000001UL;
static constexpr uint32_t kSessionLossIcmRead = 0x00000002UL;
static constexpr uint32_t kSessionLossBnoBuffer = 0x00000004UL;
static constexpr uint32_t kSessionLossIcmBuffer = 0x00000008UL;
static constexpr uint32_t kSessionLossBnoWrite = 0x00000010UL;
static constexpr uint32_t kSessionLossIcmWrite = 0x00000020UL;

enum class RecorderState : uint8_t {
    Idle,
    Armed,
    Recording,
    Finalizing,
    ReadyForUpload,
    Uploading,
    AwaitingAck,
    EraseAfterAck
};

#pragma pack(push, 1)
struct SessionHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t node_id;
    uint32_t session_id;
    uint64_t start_timestamp_us;
    uint32_t requested_duration_ms;
    uint32_t actual_duration_ms;
    uint8_t sensor_mask;
    uint8_t completion_flag;
    uint16_t reserved;
    uint32_t bno85_sample_count;
    uint32_t icm45686_sample_count;
    uint32_t bno85_payload_size;
    uint32_t icm45686_payload_size;
    uint16_t bno85_target_rate_hz;
    uint16_t icm45686_target_rate_hz;
    uint32_t bno85_attempted_count;
    uint32_t icm45686_attempted_count;
    uint32_t bno85_captured_count;
    uint32_t icm45686_captured_count;
    uint32_t bno85_dropped_count;
    uint32_t icm45686_dropped_count;
    uint32_t loss_flags;
    uint32_t payload_crc32;
    uint32_t header_crc32;
};

struct Bno85SampleV2 {
    uint32_t offset_us;
    float quat_i;
    float quat_j;
    float quat_k;
    float quat_real;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t accuracy;
    uint8_t status;
    uint8_t report_id;
    uint8_t sequence;
    uint8_t available_mask;
    uint8_t reserved0;
    uint32_t sensor_timestamp_us;
    uint32_t delay_us;
};

struct Icm45686SampleV2 {
    uint32_t offset_us;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
    uint8_t data_valid;
    int8_t read_status;
    uint16_t reserved0;
    uint32_t sample_timestamp_us;
};

struct Bno85SampleV3 {
    uint32_t offset_us;
    float quat_i;
    float quat_j;
    float quat_k;
    float quat_real;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

struct Icm45686SampleV3 {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct Icm45686SampleV4 {
    uint32_t offset_us;
    uint32_t sequence;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};
#pragma pack(pop)

static_assert(sizeof(SessionHeader) == 88U, "Unexpected SessionHeader layout");
static_assert(sizeof(Bno85SampleV2) == 82U, "Unexpected Bno85SampleV2 layout");
static_assert(sizeof(Icm45686SampleV2) == 26U, "Unexpected Icm45686SampleV2 layout");
static_assert(sizeof(Bno85SampleV3) == 56U, "Unexpected Bno85SampleV3 layout");
static_assert(sizeof(Icm45686SampleV3) == 12U, "Unexpected Icm45686SampleV3 layout");
static_assert(sizeof(Icm45686SampleV4) == 20U, "Unexpected Icm45686SampleV4 layout");

#if EXO_SAMPLE_FORMAT_VERSION == 2U
using Bno85Sample = Bno85SampleV2;
using Icm45686Sample = Icm45686SampleV2;
#elif EXO_SAMPLE_FORMAT_VERSION == 3U
using Bno85Sample = Bno85SampleV3;
using Icm45686Sample = Icm45686SampleV3;
#else
using Bno85Sample = Bno85SampleV3;
using Icm45686Sample = Icm45686SampleV4;
#endif

inline uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t size) {
    crc = ~crc;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1U)));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

inline uint32_t crc32(const void *data, size_t size) {
    return crc32_update(0U, static_cast<const uint8_t *>(data), size);
}

inline uint32_t session_header_crc(const SessionHeader &header) {
    SessionHeader copy = header;
    copy.header_crc32 = 0U;
    return crc32(&copy, sizeof(copy));
}

} // namespace exo

#endif
