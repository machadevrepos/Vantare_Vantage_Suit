#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <exo/sensors/master_training_csv_formatter.h>

static std::vector<std::string> split(const std::string &line)
{
    std::vector<std::string> fields;
    size_t start = 0U;
    for (size_t i = 0U; i <= line.size(); ++i) {
        if (i == line.size() || line[i] == ',') {
            fields.push_back(line.substr(start, i - start));
            start = i + 1U;
        }
    }
    if (!fields.empty() && fields.back().size() >= 2U &&
            fields.back().substr(fields.back().size() - 2U) == "\r\n") {
        fields.back().resize(fields.back().size() - 2U);
    }
    return fields;
}

static double number(const std::string &value)
{
    assert(!value.empty());
    return std::strtod(value.c_str(), nullptr);
}

int main()
{
    using namespace exo;
    using namespace exo::training_csv;
    static_assert(csv_header_column_count() == 51U, "schema-v2 column count");
    assert(std::string(source_label(0U)) == "MASTER");
    assert(std::string(source_label(4U)) == "NODE4");
    assert(source_label(5U) == nullptr);

    TrainingCsvRowContext first{};
    first.source.target_rate_hz = 100U;
    first.source.attempted_count = 1001U;
    first.source.captured_count = 1000U;
    first.source.dropped_count = 1U;
    first.source.loss_flags = 0x12U;
    first.source.payload_crc32 = 0x12345678U;

    Bno85Sample bno{};
    const double s = std::sqrt(0.5);
    bno.quat_i = static_cast<float>(s);
    bno.quat_real = static_cast<float>(s);
    bno.linear_accel_x = 3.0f;
    bno.linear_accel_y = 4.0f;
    bno.gravity_z = 9.81f;
    bno.gyro_z = 2.0f;

    char row[2048]{};
    size_t written = 0U;
    assert(format_bno_row(row, sizeof(row), written, 7U, 0U, 4U, 0U,
            10000U, first, true, 0x0FU, bno));
    auto fields = split(std::string(row, written));
    assert(fields.size() == 51U);
    assert(fields[0] == "2");
    assert(fields[4] == "NODE4");
    assert(fields[5] == "BNO85");
    assert(fields[8].empty());
    assert(fields[9].empty());
    assert(fields[10] == "100");
    assert(fields[15] == "305419896");
    assert(std::fabs(number(fields[43]) - 90.0) < 0.001);
    assert(std::fabs(number(fields[44])) < 0.001);
    assert(std::fabs(number(fields[45])) < 0.001);
    assert(std::fabs(number(fields[46]) - 5.0) < 0.000001);
    assert(std::fabs(number(fields[47]) - 9.81) < 0.00001);
    assert(std::fabs(number(fields[48]) - 2.0) < 0.000001);
    assert(fields[49].empty() && fields[50].empty());

    TrainingCsvRowContext next = first;
    next.has_sample_delta = true;
    next.sample_delta_us = 10000U;
    next.has_effective_sample_rate = true;
    next.effective_sample_rate_hz = 100.0;
    next.timestamp_quality_flags = kTimestampQualityNonMonotonic;
    Icm45686Sample icm{};
    icm.accel_x = 8192;
    icm.gyro_z = 16384;
    assert(format_icm_row(row, sizeof(row), written, 7U, 1U, 2U, 0U,
            20000U, next, icm));
    fields = split(std::string(row, written));
    assert(fields.size() == 51U);
    assert(fields[5] == "ICM45686");
    assert(fields[8] == "10000");
    assert(std::fabs(number(fields[9]) - 100.0) < 0.000001);
    assert(fields[16] == "1");
    assert(fields[31] == "8192");
    assert(std::fabs(number(fields[37]) - 1.0) < 0.000001);
    assert(std::fabs(number(fields[42]) - 1000.0) < 0.000001);
    assert(std::fabs(number(fields[49]) - 1.0) < 0.000001);
    assert(std::fabs(number(fields[50]) - 1000.0) < 0.000001);

    Bno85Sample invalid{};
    invalid.linear_accel_x = 1.0f;
    assert(format_bno_row(row, sizeof(row), written, 7U, 2U, 0U, 0U,
            30000U, first, false, 0U, invalid));
    fields = split(std::string(row, written));
    assert(fields[17].empty());
    assert(fields[43].empty() && fields[44].empty() && fields[45].empty());

    std::cout << "master training csv formatter v2 tests passed\n";
    return 0;
}
