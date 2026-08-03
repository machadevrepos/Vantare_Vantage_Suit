#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "MASTER_IMU_CSV_FORMATTER.h"

using namespace exo::imu_csv;

static_assert(scale_icm_accel_1e6(8192) == 1000000LL, "ICM +1 g conversion failed");
static_assert(scale_icm_accel_1e6(-32768) == -4000000LL, "ICM -4 g conversion failed");
static_assert(scale_icm_gyro_1e6(16384) == 1000000000LL, "ICM +1000 dps conversion failed");
static_assert(scale_icm_gyro_1e6(-32768) == -2000000000LL, "ICM -2000 dps conversion failed");
static_assert(csv_header_column_count() == 31U, "CSV header must contain exactly 31 columns");
static_assert(scale_bno_value_1e6(20.123457f) == 20123457LL, "BNO positive fixed-six rounding failed");
static_assert(scale_bno_value_1e6(-20.123457f) == -20123457LL, "BNO negative fixed-six rounding failed");

static int count_fields(const char *row)
{
    int fields = 1;
    for (const char *cursor = row; *cursor != '\0' && *cursor != '\r'; ++cursor) {
        if (*cursor == ',') {
            ++fields;
        }
    }
    return fields;
}

int main()
{
    if (count_fields(kCsvHeader) != 31) {
        return 1;
    }

    exo::Bno85Sample bno{};
    bno.quat_i = 0.5f;
    bno.quat_j = -0.0f;
    bno.quat_k = -0.25f;
    bno.quat_real = 1.0f;
    bno.linear_accel_x = 1.25f;
    bno.gravity_z = 9.80665f;
    bno.gyro_y = -0.125f;

    char row[640]{};
    size_t written = 0U;
    if (!format_bno_row(row, sizeof(row), written, 7U, 3U, 123000ULL, 0x0FU, bno)) {
        return 2;
    }
    if (count_fields(row) != 31 || strstr(row, "1,7,3,123000,BNO85,15,0.500000,0.000000,-0.250000,1.000000") != row) {
        return 3;
    }
    if (strstr(row, ",,,,,,,,,,,,\r\n") == nullptr || written != strlen(row)) {
        return 4;
    }

    exo::Icm45686Sample icm{};
    icm.accel_x = 8192;
    icm.accel_y = -8192;
    icm.accel_z = 0;
    icm.gyro_x = 16384;
    icm.gyro_y = -16384;
    icm.gyro_z = 0;

    memset(row, 0, sizeof(row));
    written = 0U;
    if (!format_icm_row(row, sizeof(row), written, 8U, 4U, 128000ULL, icm)) {
        return 5;
    }
    if (count_fields(row) != 31 || strstr(row, "1,8,4,128000,ICM45686,,,,,,,,,,,,,,,8192,-8192,0,16384,-16384,0,1.000000,-1.000000,0.000000,1000.000000,-1000.000000,0.000000\r\n") != row) {
        return 6;
    }

    char too_small[16]{};
    written = 99U;
    if (format_icm_row(too_small, sizeof(too_small), written, 0U, 0U, 0U, icm) || written != 0U) {
        return 7;
    }

    return 0;
}
