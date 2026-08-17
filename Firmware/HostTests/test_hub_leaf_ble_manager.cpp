#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "HUB_LEAF_BLE_MANAGER.h"

int main()
{
    exo::ble_hub::HubLeafBleManager manager;
    uint8_t payload = 0U;
    for (uint8_t node = 1U; node <= 4U; ++node) {
        for (uint8_t sensor = 1U; sensor <= 2U; ++sensor) {
            payload = static_cast<uint8_t>(node * 10U + sensor);
            assert(manager.push_leaf_sample(node, sensor, &payload, 1U));
        }
    }

    // Each node/sensor lane now preserves a short FIFO instead of replacing its
    // pending sample. Node1/BNO already contains its initial value (11), so
    // seven additional values fill the depth-eight lane.
    for (uint8_t value = 30U; value < 37U; ++value) {
        assert(manager.push_leaf_sample(1U, 1U, &value, 1U));
    }
    uint8_t overflow = 99U;
    assert(!manager.push_leaf_sample(1U, 1U, &overflow, 1U));
    assert(manager.pending_live_sample_count() == 15U);
    assert(manager.live_coalesced(1U, 1U) == 0U);
    assert(manager.live_dropped(1U, 1U) == 1U);

    std::set<std::pair<int, int>> pairs;
    std::vector<int> source_order;
    std::vector<int> node1_bno_values;
    exo::ble_hub::HubLeafBleManager::LiveSample sample{};
    while (manager.pop_next_live_sample(sample)) {
        pairs.insert({sample.node_id, sample.sensor_id});
        source_order.push_back(sample.node_id);
        if (sample.node_id == 1U && sample.sensor_id == 1U) {
            node1_bno_values.push_back(sample.payload[0]);
        }
    }

    assert(pairs.size() == 8U);
    for (int node = 1; node <= 4; ++node) {
        assert(pairs.count({node, 1}) == 1U);
        assert(pairs.count({node, 2}) == 1U);
    }
    const std::vector<int> expected_prefix{1, 2, 3, 4, 1, 2, 3, 4};
    assert(source_order.size() == 15U);
    assert(std::equal(expected_prefix.begin(), expected_prefix.end(), source_order.begin()));
    const std::vector<int> expected_node1_bno{11, 30, 31, 32, 33, 34, 35, 36};
    assert(node1_bno_values == expected_node1_bno);
    assert(manager.pending_live_sample_count() == 0U);

    std::array<exo::RecordDoneMessage, 4> done{};
    for (uint8_t node = 1U; node <= 4U; ++node) {
        done[node - 1U].command = exo::RecordCommand::RecordDone;
        done[node - 1U].node_id = node;
        done[node - 1U].session_id = 77U;
        done[node - 1U].total_size = static_cast<uint32_t>(1000U * node);
        done[node - 1U].payload_crc32 = static_cast<uint32_t>(0x11111111U * node);
    }
    for (int node = 4; node >= 1; --node) {
        assert(manager.queue_record_done(done[static_cast<size_t>(node - 1)]));
    }
    exo::RecordDoneMessage selected{};
    for (uint8_t expected = 1U; expected <= 4U; ++expected) {
        assert(manager.pop_next_record_done(selected));
        assert(selected.node_id == expected);
        assert(manager.active_source_id() == expected);
        manager.on_ble_reliable_verify_ok(77U, expected,
            done[expected - 1U].payload_crc32);
    }
    assert(!manager.pop_next_record_done(selected));

    std::cout << "hub leaf manager tests passed\n";
    return 0;
}
