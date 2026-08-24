#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include <exo/sensors/master_training_csv_logger.h>

static std::string g_output;
static FRESULT mkdir_ok(const TCHAR*) { return FR_OK; }
static FRESULT stat_missing(const TCHAR*, FILINFO*) { return FR_NO_FILE; }
static FRESULT open_ok(FIL*, const TCHAR*, BYTE) { return FR_OK; }
static FRESULT write_capture(FIL*, const void *data, UINT bytes, UINT *written)
{
    g_output.append(static_cast<const char*>(data), bytes);
    *written = bytes;
    return FR_OK;
}
static FRESULT sync_ok(FIL*) { return FR_OK; }
static FRESULT close_ok(FIL*) { return FR_OK; }
static FRESULT rename_ok(const TCHAR*, const TCHAR*) { return FR_OK; }

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
    return fields;
}

int main()
{
    const exo::training_csv::TrainingCsvFatFsOps ops = {
        mkdir_ok, stat_missing, open_ok, write_capture, sync_ok, close_ok, rename_ok
    };
    exo::MasterTrainingCsvLogger logger(&ops);
    assert(logger.begin(9U, 0x01U, 0U));
    exo::SessionHeader header{};
    header.node_id = 0U;
    header.session_id = 9U;
    header.bno85_target_rate_hz = 100U;
    header.bno85_attempted_count = 3U;
    header.bno85_captured_count = 3U;
    header.payload_crc32 = 0xAABBCCDDU;
    assert(logger.set_source_metadata(0U, header));

    exo::Bno85Sample sample{};
    sample.quat_real = 1.0f;
    assert(logger.append_bno(0U, 10000U, sample, false, 0U, 10U));
    assert(logger.append_bno(0U, 20000U, sample, false, 0U, 20U));
    assert(logger.append_bno(0U, 15000U, sample, false, 0U, 30U));
    assert(logger.service(2000U));

    std::vector<std::string> lines;
    size_t start = 0U;
    while (start < g_output.size()) {
        size_t end = g_output.find("\r\n", start);
        if (end == std::string::npos) break;
        lines.push_back(g_output.substr(start, end - start));
        start = end + 2U;
    }
    assert(lines.size() == 4U);
    auto first = split(lines[1]);
    auto second = split(lines[2]);
    auto third = split(lines[3]);
    assert(first.size() == 51U && second.size() == 51U && third.size() == 51U);
    assert(first[8].empty() && first[9].empty());
    assert(second[8] == "10000" && second[9] == "100.000000");
    assert(third[8].empty() && third[9].empty() && third[16] == "1");
    assert(first[10] == "100" && first[11] == "3" && first[12] == "3");
    assert(first[15] == "2864434397");

    std::cout << "master training csv logger v2 tests passed\n";
    return 0;
}
