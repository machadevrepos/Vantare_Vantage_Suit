#ifndef MASTER_IMU_SWO_TELEMETRY_H_
#define MASTER_IMU_SWO_TELEMETRY_H_

#include <stdint.h>

namespace exo {
namespace imu_swo {

static constexpr uint32_t kDefaultIntervalMs = 500U;

constexpr int32_t scale_quaternion_1e4(float value)
{
    return static_cast<int32_t>(value * 10000.0f);
}

constexpr int32_t scale_bno_milli(float value)
{
    return static_cast<int32_t>(value * 1000.0f);
}

constexpr int32_t scale_icm_accel_mg(int16_t raw)
{
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 4000LL) / 32768LL);
}

constexpr int32_t scale_icm_gyro_mdps(int16_t raw)
{
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 2000000LL) / 32768LL);
}

} // namespace imu_swo
} // namespace exo

#endif /* MASTER_IMU_SWO_TELEMETRY_H_ */
