#ifndef MASTER_TRAINING_CSV_FORMATTER_H_
#define MASTER_TRAINING_CSV_FORMATTER_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <RECORDING_TYPES.h>

namespace exo {
namespace training_csv {

static constexpr uint32_t kSchemaVersion = 2U;
static constexpr uint32_t kTimestampQualityNonMonotonic = 0x00000001UL;

struct TrainingCsvSourceMetadata {
    uint16_t target_rate_hz = 0U;
    uint32_t attempted_count = 0U;
    uint32_t captured_count = 0U;
    uint32_t dropped_count = 0U;
    uint32_t loss_flags = 0U;
    uint32_t payload_crc32 = 0U;
};

struct TrainingCsvRowContext {
    TrainingCsvSourceMetadata source{};
    uint64_t sample_delta_us = 0U;
    double effective_sample_rate_hz = 0.0;
    uint32_t timestamp_quality_flags = 0U;
    bool has_sample_delta = false;
    bool has_effective_sample_rate = false;
};

static constexpr char kCsvHeader[] =
        "schema_version,session_id,row_sequence,source_node_id,source_label,sensor_id,sensor_sequence,session_time_us,"
        "sample_delta_us,effective_sample_rate_hz,source_target_rate_hz,source_attempted_count,source_captured_count,"
        "source_dropped_count,source_loss_flags,source_payload_crc32,timestamp_quality_flags,"
        "bno_available_mask,bno_qx,bno_qy,bno_qz,bno_qw,bno_linear_x_mps2,bno_linear_y_mps2,bno_linear_z_mps2,"
        "bno_gravity_x_mps2,bno_gravity_y_mps2,bno_gravity_z_mps2,bno_gyro_x_radps,bno_gyro_y_radps,bno_gyro_z_radps,"
        "icm_accel_x_raw,icm_accel_y_raw,icm_accel_z_raw,icm_gyro_x_raw,icm_gyro_y_raw,icm_gyro_z_raw,"
        "icm_accel_x_g,icm_accel_y_g,icm_accel_z_g,icm_gyro_x_dps,icm_gyro_y_dps,icm_gyro_z_dps,"
        "bno_roll_deg,bno_pitch_deg,bno_yaw_deg,bno_linear_accel_magnitude_mps2,bno_gravity_magnitude_mps2,"
        "bno_gyro_magnitude_radps,icm_accel_magnitude_g,icm_gyro_magnitude_dps\r\n";

constexpr uint32_t csv_header_column_count()
{
    uint32_t columns = 1U;
    for (size_t index = 0U; kCsvHeader[index] != '\0'; ++index) {
        if (kCsvHeader[index] == ',') ++columns;
    }
    return columns;
}

constexpr bool source_id_valid(uint8_t source_id) { return source_id <= 4U; }
inline const char *source_label(uint8_t source_id)
{
    static const char *const labels[5] = {"MASTER", "NODE1", "NODE2", "NODE3", "NODE4"};
    return source_id_valid(source_id) ? labels[source_id] : nullptr;
}

constexpr int64_t divide_round_nearest(int64_t numerator, int64_t denominator)
{
    return numerator >= 0 ? (numerator + denominator / 2) / denominator :
            (numerator - denominator / 2) / denominator;
}
constexpr int64_t scale_icm_accel_1e6(int16_t raw)
{
    return divide_round_nearest(static_cast<int64_t>(raw) * 4000000LL, 32768LL);
}
constexpr int64_t scale_icm_gyro_1e6(int16_t raw)
{
    return divide_round_nearest(static_cast<int64_t>(raw) * 2000000000LL, 32768LL);
}

class CsvRowWriter {
public:
    CsvRowWriter(char *output, size_t capacity) : output_(output), capacity_(capacity)
    {
        if (output_ != nullptr && capacity_ > 0U) output_[0] = '\0';
    }
    bool append_char(char value)
    {
        if (!reserve(1U)) return false;
        output_[length_++] = value; output_[length_] = '\0'; return true;
    }
    bool append_text(const char *value)
    {
        if (value == nullptr) return false;
        while (*value != '\0') if (!append_char(*value++)) return false;
        return true;
    }
    bool append_u64(uint64_t value)
    {
        char digits[20]; size_t count = 0U;
        do { digits[count++] = static_cast<char>('0' + value % 10U); value /= 10U; } while (value != 0U);
        if (!reserve(count)) return false;
        while (count > 0U) output_[length_++] = digits[--count];
        output_[length_] = '\0'; return true;
    }
    bool append_i64(int64_t value)
    {
        if (value < 0) return append_char('-') && append_u64(static_cast<uint64_t>(-(value + 1)) + 1U);
        return append_u64(static_cast<uint64_t>(value));
    }
    bool append_fixed_6(int64_t scaled)
    {
        uint64_t magnitude;
        if (scaled < 0) { if (!append_char('-')) return false; magnitude = static_cast<uint64_t>(-(scaled + 1)) + 1U; }
        else magnitude = static_cast<uint64_t>(scaled);
        if (!append_u64(magnitude / 1000000U) || !append_char('.')) return false;
        uint64_t fraction = magnitude % 1000000U;
        uint64_t divisor = 100000U;
        for (uint8_t i = 0U; i < 6U; ++i) { if (!append_char(static_cast<char>('0' + (fraction / divisor) % 10U))) return false; divisor /= 10U; }
        return true;
    }
    size_t length() const { return length_; }
private:
    bool reserve(size_t additional) const
    {
        return output_ != nullptr && capacity_ > 0U && additional < capacity_ && length_ <= capacity_ - additional - 1U;
    }
    char *output_; size_t capacity_; size_t length_ = 0U;
};

inline bool finite_value(double value) { return isfinite(value) != 0; }
inline int64_t scale_double_1e6(double value)
{
    return static_cast<int64_t>(value * 1000000.0 + (value > 0.0 ? 0.5 : (value < 0.0 ? -0.5 : 0.0)));
}
inline bool append_double(CsvRowWriter &writer, double value)
{
    return finite_value(value) && value <= 1000000.0 && value >= -1000000.0 && writer.append_fixed_6(scale_double_1e6(value));
}
inline bool append_double_or_blank(CsvRowWriter &writer, double value)
{
    if (!finite_value(value) || value > 1000000.0 || value < -1000000.0) return true;
    return append_double(writer, value);
}
inline bool append_field_double_or_blank(CsvRowWriter &writer, double value)
{
    return writer.append_char(',') && append_double_or_blank(writer, value);
}
inline bool append_blank_fields(CsvRowWriter &writer, uint8_t count)
{
    for (uint8_t i = 0U; i < count; ++i) if (!writer.append_char(',')) return false;
    return true;
}

inline bool append_common_fields(CsvRowWriter &writer, uint32_t session_id,
        uint32_t row_sequence, uint8_t source_id, uint32_t sensor_sequence,
        uint64_t session_time_us, const char *sensor_id,
        const TrainingCsvRowContext &context)
{
    const char *label = source_label(source_id);
    if (label == nullptr || !writer.append_u64(kSchemaVersion) || !writer.append_char(',') ||
            !writer.append_u64(session_id) || !writer.append_char(',') ||
            !writer.append_u64(row_sequence) || !writer.append_char(',') ||
            !writer.append_u64(source_id) || !writer.append_char(',') ||
            !writer.append_text(label) || !writer.append_char(',') ||
            !writer.append_text(sensor_id) || !writer.append_char(',') ||
            !writer.append_u64(sensor_sequence) || !writer.append_char(',') ||
            !writer.append_u64(session_time_us) || !writer.append_char(',')) return false;
    if (context.has_sample_delta && !writer.append_u64(context.sample_delta_us)) return false;
    if (!writer.append_char(',')) return false;
    if (context.has_effective_sample_rate && !append_double(writer, context.effective_sample_rate_hz)) return false;
    return writer.append_char(',') && writer.append_u64(context.source.target_rate_hz) &&
            writer.append_char(',') && writer.append_u64(context.source.attempted_count) &&
            writer.append_char(',') && writer.append_u64(context.source.captured_count) &&
            writer.append_char(',') && writer.append_u64(context.source.dropped_count) &&
            writer.append_char(',') && writer.append_u64(context.source.loss_flags) &&
            writer.append_char(',') && writer.append_u64(context.source.payload_crc32) &&
            writer.append_char(',') && writer.append_u64(context.timestamp_quality_flags);
}

inline bool finish_row(CsvRowWriter &writer, size_t &written)
{
    if (!writer.append_char('\r') || !writer.append_char('\n')) { written = 0U; return false; }
    written = writer.length(); return true;
}

struct BnoDerivedFeatures {
    double roll_deg = NAN;
    double pitch_deg = NAN;
    double yaw_deg = NAN;
    double linear_accel_magnitude = NAN;
    double gravity_magnitude = NAN;
    double gyro_magnitude = NAN;
};
inline BnoDerivedFeatures derive_bno_features(const Bno85Sample &sample)
{
    BnoDerivedFeatures out{};
    const double qx = sample.quat_i, qy = sample.quat_j, qz = sample.quat_k, qw = sample.quat_real;
    const double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (finite_value(norm) && norm > 1.0e-9) {
        const double x = qx / norm, y = qy / norm, z = qz / norm, w = qw / norm;
        const double sinr_cosp = 2.0 * (w*x + y*z);
        const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
        const double sinp = 2.0 * (w*y - z*x);
        const double siny_cosp = 2.0 * (w*z + x*y);
        const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
        constexpr double kRadToDeg = 57.2957795130823208768;
        out.roll_deg = atan2(sinr_cosp, cosr_cosp) * kRadToDeg;
        out.pitch_deg = (fabs(sinp) >= 1.0 ? copysign(1.57079632679489661923, sinp) : asin(sinp)) * kRadToDeg;
        out.yaw_deg = atan2(siny_cosp, cosy_cosp) * kRadToDeg;
    }
    const auto magnitude = [](double x, double y, double z) -> double {
        return (finite_value(x) && finite_value(y) && finite_value(z)) ? sqrt(x*x + y*y + z*z) : NAN;
    };
    out.linear_accel_magnitude = magnitude(sample.linear_accel_x, sample.linear_accel_y, sample.linear_accel_z);
    out.gravity_magnitude = magnitude(sample.gravity_x, sample.gravity_y, sample.gravity_z);
    out.gyro_magnitude = magnitude(sample.gyro_x, sample.gyro_y, sample.gyro_z);
    return out;
}

inline bool format_bno_row(char *output, size_t capacity, size_t &written,
        uint32_t session_id, uint32_t row_sequence, uint8_t source_id,
        uint32_t sensor_sequence, uint64_t session_time_us,
        const TrainingCsvRowContext &context, bool has_available_mask,
        uint8_t available_mask, const Bno85Sample &sample)
{
    written = 0U; CsvRowWriter writer(output, capacity);
    if (!append_common_fields(writer, session_id, row_sequence, source_id, sensor_sequence,
            session_time_us, "BNO85", context) || !writer.append_char(',')) return false;
    if (has_available_mask && !writer.append_u64(available_mask)) return false;
    const double raw[] = {sample.quat_i, sample.quat_j, sample.quat_k, sample.quat_real,
        sample.linear_accel_x, sample.linear_accel_y, sample.linear_accel_z,
        sample.gravity_x, sample.gravity_y, sample.gravity_z,
        sample.gyro_x, sample.gyro_y, sample.gyro_z};
    for (double value : raw) if (!append_field_double_or_blank(writer, value)) return false;
    if (!append_blank_fields(writer, 12U)) return false;
    const BnoDerivedFeatures derived = derive_bno_features(sample);
    const double values[] = {derived.roll_deg, derived.pitch_deg, derived.yaw_deg,
        derived.linear_accel_magnitude, derived.gravity_magnitude, derived.gyro_magnitude};
    for (double value : values) if (!append_field_double_or_blank(writer, value)) return false;
    if (!append_blank_fields(writer, 2U)) return false;
    return finish_row(writer, written);
}

inline bool format_icm_row(char *output, size_t capacity, size_t &written,
        uint32_t session_id, uint32_t row_sequence, uint8_t source_id,
        uint32_t sensor_sequence, uint64_t session_time_us,
        const TrainingCsvRowContext &context, const Icm45686Sample &sample)
{
    written = 0U; CsvRowWriter writer(output, capacity);
    if (!append_common_fields(writer, session_id, row_sequence, source_id, sensor_sequence,
            session_time_us, "ICM45686", context) || !append_blank_fields(writer, 14U)) return false;
    const int16_t raw[] = {sample.accel_x, sample.accel_y, sample.accel_z, sample.gyro_x, sample.gyro_y, sample.gyro_z};
    for (int16_t value : raw) if (!writer.append_char(',') || !writer.append_i64(value)) return false;
    const int64_t scaled[] = {scale_icm_accel_1e6(sample.accel_x), scale_icm_accel_1e6(sample.accel_y), scale_icm_accel_1e6(sample.accel_z),
        scale_icm_gyro_1e6(sample.gyro_x), scale_icm_gyro_1e6(sample.gyro_y), scale_icm_gyro_1e6(sample.gyro_z)};
    for (int64_t value : scaled) if (!writer.append_char(',') || !writer.append_fixed_6(value)) return false;
    if (!append_blank_fields(writer, 6U)) return false;
    const double ax = static_cast<double>(scaled[0]) / 1000000.0;
    const double ay = static_cast<double>(scaled[1]) / 1000000.0;
    const double az = static_cast<double>(scaled[2]) / 1000000.0;
    const double gx = static_cast<double>(scaled[3]) / 1000000.0;
    const double gy = static_cast<double>(scaled[4]) / 1000000.0;
    const double gz = static_cast<double>(scaled[5]) / 1000000.0;
    if (!append_field_double_or_blank(writer, sqrt(ax*ax + ay*ay + az*az)) ||
            !append_field_double_or_blank(writer, sqrt(gx*gx + gy*gy + gz*gz))) return false;
    return finish_row(writer, written);
}

} // namespace training_csv
} // namespace exo
#endif
