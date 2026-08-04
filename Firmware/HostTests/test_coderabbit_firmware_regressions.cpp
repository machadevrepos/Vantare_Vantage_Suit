#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

#include "HUB_LEAF_BLE_MANAGER.h"
#include "MASTER_NODE_RELIABLE_CONTROL.h"
#include "MASTER_NODE_TRANSFER_WINDOW.h"
#include "MASTER_TRAINING_CSV_FORMATTER.h"

namespace {
int failures = 0;
#define EXPECT_TRUE(expr) do { if (!(expr)) { std::cerr << __FILE__ << ':' << __LINE__ << ": expected " #expr "\n"; ++failures; } } while (0)

struct Transport {
    std::vector<uint8_t> frame;
    static bool send(void *context, uint8_t, const uint8_t *data, uint16_t length)
    {
        auto &self = *static_cast<Transport *>(context);
        self.frame.assign(data, data + length);
        return true;
    }
};

exo::RecordReliableFrameHeader frame_header(const std::vector<uint8_t> &frame)
{
    exo::RecordReliableFrameHeader header{};
    if (frame.size() >= sizeof(header)) std::memcpy(&header, frame.data(), sizeof(header));
    return header;
}

template<typename Manager, typename = void>
struct CongestionProbe {
    static bool run() { return false; }
    static bool four_node_rate() { return false; }
};

template<typename Manager>
struct CongestionProbe<Manager, std::void_t<
    decltype(std::declval<Manager &>().peek_next_live_sample(
        std::declval<typename Manager::LiveSample &>(), uint32_t{})),
    decltype(std::declval<Manager &>().on_live_sample_send_result(bool{}, uint32_t{})),
    decltype(std::declval<const Manager &>().live_preview_congested())>> {
    static bool run()
    {
        Manager manager;
        uint8_t value = 1U;
        if (!manager.push_leaf_sample(1U, 1U, &value, 1U)) return false;
        value = 2U;
        if (!manager.push_leaf_sample(2U, 1U, &value, 1U)) return false;
        typename Manager::LiveSample sample{};
        if (!manager.peek_next_live_sample(sample, 0U) || sample.node_id != 1U) return false;
        manager.on_live_sample_send_result(false, 0U);
        if (!manager.live_preview_congested()) return false;
        if (manager.peek_next_live_sample(sample, 19U)) return false;
        if (!manager.peek_next_live_sample(sample, 20U) || sample.node_id != 2U) return false;
        manager.on_live_sample_send_result(true, 20U);
        if (manager.peek_next_live_sample(sample, 39U)) return false;
        if (!manager.peek_next_live_sample(sample, 40U) || sample.node_id != 1U) return false;
        manager.on_live_sample_send_result(true, 40U);
        value = 3U;
        if (!manager.push_leaf_sample(3U, 1U, &value, 1U)) return false;
        if (!manager.peek_next_live_sample(sample, 5000U) || sample.node_id != 3U) return false;
        manager.on_live_sample_send_result(true, 5000U);
        if (manager.live_preview_congested()) return false;
        value = 4U;
        if (!manager.push_leaf_sample(4U, 1U, &value, 1U)) return false;
        if (manager.peek_next_live_sample(sample, 5009U)) return false;
        return manager.peek_next_live_sample(sample, 5010U) && sample.node_id == 4U;
    }

    static bool four_node_rate()
    {
        Manager manager;
        uint8_t value = 1U;
        for (uint8_t node = 1U; node <= 4U; ++node) {
            value = node;
            if (!manager.push_leaf_sample(node, 1U, &value, 1U)) return false;
        }

        uint8_t per_source_count[5]{};
        typename Manager::LiveSample sample{};
        for (uint32_t now_ms = 0U; now_ms < 400U; now_ms += 10U) {
            if (!manager.peek_next_live_sample(sample, now_ms)) return false;
            if (sample.node_id < 1U || sample.node_id > 4U) return false;
            ++per_source_count[sample.node_id];
            manager.on_live_sample_send_result(true, now_ms);
            value = static_cast<uint8_t>(value + 1U);
            if (!manager.push_leaf_sample(sample.node_id, 1U, &value, 1U)) return false;
        }
        for (uint8_t node = 1U; node <= 4U; ++node) {
            if (per_source_count[node] < 10U) return false;
        }
        return true;
    }
};

template<typename Manager, typename = void>
struct RemoteLifecycleProbe {
    static bool run() { return false; }
};

template<typename Manager>
struct RemoteLifecycleProbe<Manager, std::void_t<
    decltype(std::declval<Manager &>().on_ble_reliable_pause(uint32_t{}, uint16_t{})),
    decltype(std::declval<Manager &>().on_ble_reliable_resume(uint32_t{}, uint16_t{})),
    decltype(std::declval<Manager &>().on_ble_reliable_cancel(uint32_t{}, uint16_t{})),
    decltype(std::declval<Manager &>().on_ble_reliable_verify_ok(
        uint32_t{}, uint16_t{}, uint32_t{})),
    decltype(std::declval<Manager &>().on_ble_session_complete(
        uint32_t{}, uint16_t{}, uint32_t{}))>> {
    static exo::RecordDoneMessage record_done(uint8_t node_id, uint32_t crc)
    {
        exo::RecordDoneMessage done{};
        done.command = exo::RecordCommand::RecordDone;
        done.node_id = node_id;
        done.session_id = 77U;
        done.total_size = static_cast<uint32_t>(sizeof(exo::SessionHeader));
        done.payload_crc32 = crc;
        return done;
    }

    static bool run()
    {
        const exo::RecordDoneMessage first = record_done(1U, 0x11111111U);
        const exo::RecordDoneMessage second = record_done(2U, 0x22222222U);
        exo::RecordDoneMessage selected{};

        Manager cancelled;
        if (!cancelled.queue_record_done(first) || !cancelled.queue_record_done(second)) return false;
        if (!cancelled.pop_next_record_done(selected) || selected.node_id != 1U) return false;
        if (cancelled.on_ble_reliable_pause(77U, 2U)) return false;
        if (!cancelled.on_ble_reliable_pause(77U, 1U) || !cancelled.paused()) return false;
        if (!cancelled.on_ble_reliable_resume(77U, 1U) || cancelled.paused()) return false;
        if (cancelled.on_ble_session_complete(77U, 1U, first.payload_crc32)) return false;
        if (!cancelled.on_ble_reliable_cancel(77U, 1U)) return false;
        if (cancelled.active_source_id() != 0U || cancelled.active_session_id() != 0U) return false;
        if (!cancelled.pop_next_record_done(selected) || selected.node_id != 2U) return false;

        Manager completed;
        if (!completed.queue_record_done(first) || !completed.queue_record_done(second)) return false;
        if (!completed.pop_next_record_done(selected) || selected.node_id != 1U) return false;
        if (completed.on_ble_reliable_verify_ok(77U, 1U, 0xDEADBEEFU)) return false;
        if (!completed.on_ble_reliable_verify_ok(77U, 1U, first.payload_crc32)) return false;
        if (completed.active_source_id() != 1U || completed.active_session_id() != 77U) return false;
        if (completed.on_ble_session_complete(77U, 1U, 0xDEADBEEFU)) return false;
        if (completed.active_source_id() != 1U) return false;
        if (!completed.on_ble_session_complete(77U, 1U, first.payload_crc32)) return false;
        if (completed.active_source_id() != 0U || completed.active_session_id() != 0U) return false;
        return completed.pop_next_record_done(selected) && selected.node_id == 2U;
    }
};
}

int main()
{
    exo::MasterNodeTransferWindow window;
    EXPECT_TRUE(window.begin(1U, 42U, 400U, 180U));
    const auto first = window.inspect(1U, 42U, 0U, 0U, 180U, true, false);
    EXPECT_TRUE(window.commit(first));
    const auto corrupt = window.inspect(1U, 42U, 0xFFFFFFFFU, 180U, 180U, false, false);
    EXPECT_TRUE(corrupt.decision == exo::NodeTransferDecision::NackCorrupt);
    EXPECT_TRUE(corrupt.request_chunk == 1U);

    Transport transport;
    exo::MasterNodeReliableControl control(&Transport::send, &transport);
    exo::RecordDoneMessage done{};
    done.command = exo::RecordCommand::RecordDone;
    done.node_id = 1U;
    done.session_id = 42U;
    done.total_size = 1000U;
    done.payload_crc32 = 0x12345678U;
    EXPECT_TRUE(control.begin(done, 180U));
    EXPECT_TRUE(control.service(1U));
    EXPECT_TRUE(control.nack_range(0xFFFFFFFFU));
    EXPECT_TRUE(control.service(2U));
    EXPECT_TRUE(frame_header(transport.frame).byte_offset == 0xFFFFFFFFU);
    EXPECT_TRUE(control.ack_window(0xFFFFFFFFU));
    EXPECT_TRUE(control.service(3U));
    EXPECT_TRUE(frame_header(transport.frame).byte_offset == 0xFFFFFFFFU);

    exo::ble_hub::HubLeafBleManager manager;
    exo::RecordDoneMessage done1{};
    done1.command = exo::RecordCommand::RecordDone;
    done1.node_id = 1U;
    done1.session_id = 77U;
    done1.total_size = static_cast<uint32_t>(sizeof(exo::SessionHeader));
    done1.payload_crc32 = 1U;
    exo::RecordDoneMessage invalid = done1;
    invalid.command = exo::RecordCommand::StartRecord;
    EXPECT_TRUE(!manager.queue_record_done(invalid));
    invalid = done1;
    invalid.session_id = 0U;
    EXPECT_TRUE(!manager.queue_record_done(invalid));
    invalid = done1;
    invalid.total_size = static_cast<uint32_t>(sizeof(exo::SessionHeader) - 1U);
    EXPECT_TRUE(!manager.queue_record_done(invalid));
    exo::RecordDoneMessage done2 = done1;
    done2.node_id = 2U;
    done2.payload_crc32 = 2U;
    EXPECT_TRUE(manager.queue_record_done(done1));
    EXPECT_TRUE(manager.queue_record_done(done2));
    exo::RecordDoneMessage selected{};
    EXPECT_TRUE(manager.pop_next_record_done(selected));
    EXPECT_TRUE(selected.node_id == 1U);
    EXPECT_TRUE(!manager.pop_next_record_done(selected));
    EXPECT_TRUE(manager.active_source_id() == 1U && manager.active_session_id() == 77U);

    char numeric[32]{};
    exo::training_csv::CsvRowWriter writer(numeric, sizeof(numeric));
    EXPECT_TRUE(exo::training_csv::append_double_or_blank(writer, 1.0e30));
    EXPECT_TRUE(writer.length() == 0U);
    exo::training_csv::TrainingCsvRowContext row_context{};
    row_context.source.target_rate_hz = 100U;
    row_context.source.attempted_count = 1U;
    row_context.source.captured_count = 1U;
    exo::Bno85Sample extreme{};
    extreme.quat_real = 1.0F;
    extreme.linear_accel_x = 1.0e30F;
    char row[2048]{};
    size_t written = 0U;
    EXPECT_TRUE(exo::training_csv::format_bno_row(row, sizeof(row), written,
        1U, 0U, 0U, 0U, 1000U, row_context, false, 0U, extreme));

    exo::Bno85Sample sample{};
    constexpr double pi = 3.14159265358979323846;
    const double roll = 30.0 * pi / 180.0;
    const double pitch = 20.0 * pi / 180.0;
    const double yaw = 10.0 * pi / 180.0;
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    sample.quat_real = static_cast<float>(cr * cp * cy + sr * sp * sy);
    sample.quat_i = static_cast<float>(sr * cp * cy - cr * sp * sy);
    sample.quat_j = static_cast<float>(cr * sp * cy + sr * cp * sy);
    sample.quat_k = static_cast<float>(cr * cp * sy - sr * sp * cy);
    const auto derived = exo::training_csv::derive_bno_features(sample);
    EXPECT_TRUE(std::fabs(derived.roll_deg - 30.0) < 0.001);
    EXPECT_TRUE(std::fabs(derived.pitch_deg - 20.0) < 0.001);
    EXPECT_TRUE(std::fabs(derived.yaw_deg - 10.0) < 0.001);

    EXPECT_TRUE(CongestionProbe<exo::ble_hub::HubLeafBleManager>::run());
    EXPECT_TRUE(CongestionProbe<exo::ble_hub::HubLeafBleManager>::four_node_rate());
    EXPECT_TRUE(RemoteLifecycleProbe<exo::ble_hub::HubLeafBleManager>::run());
    return failures == 0 ? 0 : 1;
}
