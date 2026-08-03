#ifndef MASTER_IMU_CSV_FORMATTER_H_
#define MASTER_IMU_CSV_FORMATTER_H_

#include <stddef.h>
#include <stdint.h>

#include <RECORDING_TYPES.h>

namespace exo {
namespace imu_csv {

static constexpr uint32_t kSchemaVersion = 1U;
static constexpr char kCsvHeader[] =
        "schema_version,row_sequence,sensor_sequence,timestamp_us,sensor_id,bno_available_mask,"
        "bno_qx,bno_qy,bno_qz,bno_qw,bno_linear_x_mps2,bno_linear_y_mps2,bno_linear_z_mps2,"
        "bno_gravity_x_mps2,bno_gravity_y_mps2,bno_gravity_z_mps2,bno_gyro_x_radps,"
        "bno_gyro_y_radps,bno_gyro_z_radps,icm_accel_x_raw,icm_accel_y_raw,icm_accel_z_raw,"
        "icm_gyro_x_raw,icm_gyro_y_raw,icm_gyro_z_raw,icm_accel_x_g,icm_accel_y_g,"
        "icm_accel_z_g,icm_gyro_x_dps,icm_gyro_y_dps,icm_gyro_z_dps\r\n";

constexpr uint32_t csv_header_column_count()
{
    uint32_t columns = 1U;
    for (size_t index = 0U; kCsvHeader[index] != '\0'; ++index) {
        if (kCsvHeader[index] == ',') {
            ++columns;
        }
    }
    return columns;
}

constexpr int64_t divide_round_nearest(int64_t numerator, int64_t denominator)
{
    return numerator >= 0 ?
            (numerator + (denominator / 2)) / denominator :
            (numerator - (denominator / 2)) / denominator;
}

constexpr int64_t scale_icm_accel_1e6(int16_t raw)
{
    return divide_round_nearest(static_cast<int64_t>(raw) * 4000000LL, 32768LL);
}

constexpr int64_t scale_icm_gyro_1e6(int16_t raw)
{
    return divide_round_nearest(static_cast<int64_t>(raw) * 2000000000LL, 32768LL);
}

constexpr int64_t scale_bno_value_1e6(float value)
{
    return static_cast<int64_t>(static_cast<double>(value) * 1000000.0 +
            (value > 0.0f ? 0.5 : (value < 0.0f ? -0.5 : 0.0)));
}

class CsvRowWriter {
public:
    CsvRowWriter(char *output, size_t capacity)
        : output_(output), capacity_(capacity)
    {
        if (output_ != nullptr && capacity_ > 0U) {
            output_[0] = '\0';
        }
    }

    bool append_char(char value)
    {
        if (!reserve(1U)) {
            return false;
        }
        output_[length_++] = value;
        output_[length_] = '\0';
        return true;
    }

    bool append_text(const char *value)
    {
        if (value == nullptr) {
            return false;
        }
        for (const char *cursor = value; *cursor != '\0'; ++cursor) {
            if (!append_char(*cursor)) {
                return false;
            }
        }
        return true;
    }

    bool append_u64(uint64_t value)
    {
        char digits[20];
        size_t count = 0U;
        do {
            digits[count++] = static_cast<char>('0' + (value % 10U));
            value /= 10U;
        } while (value != 0U);

        if (!reserve(count)) {
            return false;
        }
        while (count > 0U) {
            output_[length_++] = digits[--count];
        }
        output_[length_] = '\0';
        return true;
    }

    bool append_i64(int64_t value)
    {
        if (value < 0) {
            if (!append_char('-')) {
                return false;
            }
            const uint64_t magnitude = static_cast<uint64_t>(-(value + 1)) + 1U;
            return append_u64(magnitude);
        }
        return append_u64(static_cast<uint64_t>(value));
    }

    bool append_fixed_6(int64_t scaled_1e6)
    {
        uint64_t magnitude = 0U;
        if (scaled_1e6 < 0) {
            if (!append_char('-')) {
                return false;
            }
            magnitude = static_cast<uint64_t>(-(scaled_1e6 + 1)) + 1U;
        } else {
            magnitude = static_cast<uint64_t>(scaled_1e6);
        }

        if (!append_u64(magnitude / 1000000U) || !append_char('.')) {
            return false;
        }
        uint64_t divisor = 100000U;
        const uint64_t fraction = magnitude % 1000000U;
        for (uint8_t digit = 0U; digit < 6U; ++digit) {
            if (!append_char(static_cast<char>('0' + ((fraction / divisor) % 10U)))) {
                return false;
            }
            divisor /= 10U;
        }
        return true;
    }

    size_t length() const { return length_; }

private:
    bool reserve(size_t additional) const
    {
        return output_ != nullptr && capacity_ > 0U && additional < capacity_ &&
                length_ <= (capacity_ - additional - 1U);
    }

    char *output_ = nullptr;
    size_t capacity_ = 0U;
    size_t length_ = 0U;
};

inline bool scale_float_1e6(float value, int64_t &scaled)
{
    if (!(value == value) || value > 1000000.0f || value < -1000000.0f) {
        return false;
    }
    scaled = scale_bno_value_1e6(value);
    return true;
}

inline bool append_prefix(CsvRowWriter &writer,
        uint32_t row_sequence,
        uint32_t sensor_sequence,
        uint64_t timestamp_us,
        const char *sensor_id)
{
    return writer.append_u64(kSchemaVersion) && writer.append_char(',') &&
            writer.append_u64(row_sequence) && writer.append_char(',') &&
            writer.append_u64(sensor_sequence) && writer.append_char(',') &&
            writer.append_u64(timestamp_us) && writer.append_char(',') &&
            writer.append_text(sensor_id);
}

inline bool append_scaled_float(CsvRowWriter &writer, float value)
{
    int64_t scaled = 0;
    return scale_float_1e6(value, scaled) && writer.append_fixed_6(scaled);
}

inline bool finish_row(CsvRowWriter &writer, size_t &written)
{
    if (!writer.append_char('\r') || !writer.append_char('\n')) {
        written = 0U;
        return false;
    }
    written = writer.length();
    return true;
}

inline bool format_bno_row(char *output,
        size_t capacity,
        size_t &written,
        uint32_t row_sequence,
        uint32_t sensor_sequence,
        uint64_t timestamp_us,
        uint8_t available_mask,
        const Bno85Sample &sample)
{
    written = 0U;
    CsvRowWriter writer(output, capacity);
    if (!append_prefix(writer, row_sequence, sensor_sequence, timestamp_us, "BNO85") ||
            !writer.append_char(',') || !writer.append_u64(available_mask)) {
        return false;
    }

    const float values[] = {
        sample.quat_i, sample.quat_j, sample.quat_k, sample.quat_real,
        sample.linear_accel_x, sample.linear_accel_y, sample.linear_accel_z,
        sample.gravity_x, sample.gravity_y, sample.gravity_z,
        sample.gyro_x, sample.gyro_y, sample.gyro_z
    };
    for (size_t index = 0U; index < (sizeof(values) / sizeof(values[0])); ++index) {
        if (!writer.append_char(',') || !append_scaled_float(writer, values[index])) {
            return false;
        }
    }
    for (uint8_t blank = 0U; blank < 12U; ++blank) {
        if (!writer.append_char(',')) {
            return false;
        }
    }
    return finish_row(writer, written);
}

inline bool format_icm_row(char *output,
        size_t capacity,
        size_t &written,
        uint32_t row_sequence,
        uint32_t sensor_sequence,
        uint64_t timestamp_us,
        const Icm45686Sample &sample)
{
    written = 0U;
    CsvRowWriter writer(output, capacity);
    if (!append_prefix(writer, row_sequence, sensor_sequence, timestamp_us, "ICM45686")) {
        return false;
    }
    for (uint8_t blank = 0U; blank < 14U; ++blank) {
        if (!writer.append_char(',')) {
            return false;
        }
    }

    const int16_t raw_values[] = {
        sample.accel_x, sample.accel_y, sample.accel_z,
        sample.gyro_x, sample.gyro_y, sample.gyro_z
    };
    for (size_t index = 0U; index < (sizeof(raw_values) / sizeof(raw_values[0])); ++index) {
        if (!writer.append_char(',') || !writer.append_i64(raw_values[index])) {
            return false;
        }
    }

    const int64_t scaled_values[] = {
        scale_icm_accel_1e6(sample.accel_x),
        scale_icm_accel_1e6(sample.accel_y),
        scale_icm_accel_1e6(sample.accel_z),
        scale_icm_gyro_1e6(sample.gyro_x),
        scale_icm_gyro_1e6(sample.gyro_y),
        scale_icm_gyro_1e6(sample.gyro_z)
    };
    for (size_t index = 0U; index < (sizeof(scaled_values) / sizeof(scaled_values[0])); ++index) {
        if (!writer.append_char(',') || !writer.append_fixed_6(scaled_values[index])) {
            return false;
        }
    }
    return finish_row(writer, written);
}

} // namespace imu_csv
} // namespace exo

#endif
