#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "MASTER_TRAINING_CSV_FORMATTER.h"

using namespace exo::training_csv;

static constexpr char kExpectedHeader[] =
        "schema_version,session_id,row_sequence,source_node_id,source_label,sensor_id,sensor_sequence,session_time_us,"
        "bno_available_mask,bno_qx,bno_qy,bno_qz,bno_qw,bno_linear_x_mps2,bno_linear_y_mps2,bno_linear_z_mps2,"
        "bno_gravity_x_mps2,bno_gravity_y_mps2,bno_gravity_z_mps2,bno_gyro_x_radps,bno_gyro_y_radps,bno_gyro_z_radps,"
        "icm_accel_x_raw,icm_accel_y_raw,icm_accel_z_raw,icm_gyro_x_raw,icm_gyro_y_raw,icm_gyro_z_raw,"
        "icm_accel_x_g,icm_accel_y_g,icm_accel_z_g,icm_gyro_x_dps,icm_gyro_y_dps,icm_gyro_z_dps\r\n";

static_assert(exo::training_csv::csv_header_column_count() == 34U, "training CSV column count");
static_assert(exo::training_csv::scale_icm_accel_1e6(8192) == 1000000LL, "+1 g");
static_assert(exo::training_csv::scale_icm_accel_1e6(-32768) == -4000000LL, "-4 g");
static_assert(exo::training_csv::scale_icm_gyro_1e6(16384) == 1000000000LL, "+1000 dps");
static_assert(exo::training_csv::scale_icm_gyro_1e6(-32768) == -2000000000LL, "-2000 dps");

static size_t split_fields(char *row, char *fields[], size_t field_capacity)
{
    size_t count = 0U;
    fields[count++] = row;
    char *cursor = row;
    for (; *cursor != '\0' && *cursor != '\r'; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            if (count < field_capacity) {
                fields[count] = cursor + 1;
            }
            ++count;
        }
    }
    *cursor = '\0';
    return count;
}

static bool common_fields_present(char *const fields[])
{
    return strcmp(fields[0], "1") == 0 && strcmp(fields[1], "42") == 0 &&
            strcmp(fields[2], "7") == 0 && strcmp(fields[3], "0") == 0 &&
            strcmp(fields[4], "MASTER") == 0 && strcmp(fields[6], "3") == 0 &&
            strcmp(fields[7], "123000") == 0;
}

int main()
{
    if (strcmp(kCsvHeader, kExpectedHeader) != 0 || source_label(5U) != nullptr ||
            strcmp(source_label(0U), "MASTER") != 0 || strcmp(source_label(1U), "NODE1") != 0 ||
            strcmp(source_label(2U), "NODE2") != 0 || strcmp(source_label(3U), "NODE3") != 0 ||
            strcmp(source_label(4U), "NODE4") != 0 || source_id_valid(5U)) {
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
    if (!format_bno_row(row, sizeof(row), written, 42U, 7U, 0U, 3U, 123000ULL,
            true, 0x0FU, bno) || written != strlen(row) ||
            written < 2U || row[written - 2U] != '\r' || row[written - 1U] != '\n') {
        return 2;
    }
    char *fields[34]{};
    if (split_fields(row, fields, 34U) != 34U || !common_fields_present(fields) ||
            strcmp(fields[5], "BNO85") != 0 || strcmp(fields[8], "15") != 0 ||
            strcmp(fields[9], "0.500000") != 0 || strcmp(fields[10], "0.000000") != 0 ||
            strcmp(fields[21], "0.000000") != 0) {
        return 3;
    }
    for (size_t index = 22U; index < 34U; ++index) {
        if (fields[index][0] != '\0') {
            return 4;
        }
    }

    exo::Icm45686Sample icm{};
    icm.accel_x = 8192;
    icm.accel_y = -8192;
    icm.gyro_x = 16384;
    icm.gyro_y = -16384;

    memset(row, 0, sizeof(row));
    written = 0U;
    if (!format_icm_row(row, sizeof(row), written, 42U, 8U, 3U, 4U, 128000ULL, icm) ||
            written != strlen(row) || written < 2U || row[written - 2U] != '\r' ||
            row[written - 1U] != '\n') {
        return 5;
    }
    if (split_fields(row, fields, 34U) != 34U || strcmp(fields[0], "1") != 0 ||
            strcmp(fields[1], "42") != 0 || strcmp(fields[2], "8") != 0 ||
            strcmp(fields[3], "3") != 0 || strcmp(fields[4], "NODE3") != 0 ||
            strcmp(fields[5], "ICM45686") != 0 || strcmp(fields[6], "4") != 0 ||
            strcmp(fields[7], "128000") != 0) {
        return 6;
    }
    for (size_t index = 8U; index < 22U; ++index) {
        if (fields[index][0] != '\0') {
            return 7;
        }
    }
    if (strcmp(fields[22], "8192") != 0 || strcmp(fields[23], "-8192") != 0 ||
            strcmp(fields[24], "0") != 0 || strcmp(fields[25], "16384") != 0 ||
            strcmp(fields[26], "-16384") != 0 || strcmp(fields[27], "0") != 0 ||
            strcmp(fields[28], "1.000000") != 0 || strcmp(fields[29], "-1.000000") != 0 ||
            strcmp(fields[30], "0.000000") != 0 || strcmp(fields[31], "1000.000000") != 0 ||
            strcmp(fields[32], "-1000.000000") != 0 || strcmp(fields[33], "0.000000") != 0) {
        return 8;
    }

    char too_small[16]{};
    written = 99U;
    if (format_bno_row(too_small, sizeof(too_small), written, 42U, 0U, 0U, 0U, 0U,
            false, 0U, bno) || written != 0U) {
        return 9;
    }
    return 0;
}
