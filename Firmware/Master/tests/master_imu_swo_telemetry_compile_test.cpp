#include "MASTER_IMU_SWO_TELEMETRY.h"

using namespace exo::imu_swo;

static_assert(kDefaultIntervalMs == 500U, "Unexpected default telemetry interval");

static_assert(scale_quaternion_1e4(0.0f) == 0, "Quaternion zero scaling failed");
static_assert(scale_quaternion_1e4(0.5f) == 5000, "Quaternion positive scaling failed");
static_assert(scale_quaternion_1e4(-0.5f) == -5000, "Quaternion negative scaling failed");

static_assert(scale_bno_milli(0.0f) == 0, "BNO physical-unit zero scaling failed");
static_assert(scale_bno_milli(0.5f) == 500, "BNO physical-unit positive scaling failed");
static_assert(scale_bno_milli(-0.5f) == -500, "BNO physical-unit negative scaling failed");

static_assert(scale_icm_accel_mg(0) == 0, "ICM acceleration zero scaling failed");
static_assert(scale_icm_accel_mg(8192) == 1000, "ICM acceleration quarter-scale failed");
static_assert(scale_icm_accel_mg(32767) == 3999, "ICM acceleration positive full-scale failed");
static_assert(scale_icm_accel_mg(-32768) == -4000, "ICM acceleration negative full-scale failed");

static_assert(scale_icm_gyro_mdps(0) == 0, "ICM gyro zero scaling failed");
static_assert(scale_icm_gyro_mdps(16384) == 1000000, "ICM gyro half-scale failed");
static_assert(scale_icm_gyro_mdps(32767) == 1999938, "ICM gyro positive full-scale failed");
static_assert(scale_icm_gyro_mdps(-32768) == -2000000, "ICM gyro negative full-scale failed");

int main()
{
    return 0;
}
