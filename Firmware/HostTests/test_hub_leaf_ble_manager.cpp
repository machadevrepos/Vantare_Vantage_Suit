#include <cassert>
#include <cstdint>
#include <iostream>
#include <set>
#include <utility>

#include "HUB_LEAF_BLE_MANAGER.h"

int main()
{
    exo::ble_hub::HubLeafBleManager manager;
    const uint8_t p1[] = {1U};
    const uint8_t p2[] = {2U};
    const uint8_t p3[] = {3U};
    const uint8_t p4[] = {4U};

    assert(manager.push_leaf_sample(1U, 1U, p1, sizeof(p1)));
    assert(manager.push_leaf_sample(1U, 2U, p2, sizeof(p2)));
    assert(manager.push_leaf_sample(2U, 1U, p3, sizeof(p3)));
    assert(manager.push_leaf_sample(2U, 2U, p4, sizeof(p4)));

    for (uint8_t value = 5U; value < 30U; ++value) {
        assert(manager.push_leaf_sample(1U, 1U, &value, 1U));
    }

    std::set<std::pair<int, int>> pairs;
    exo::ble_hub::HubLeafBleManager::LiveSample sample{};
    int latest_node1_sensor1 = -1;
    int sample_count = 0;
    while (manager.pop_next_live_sample(sample)) {
        ++sample_count;
        pairs.insert({sample.node_id, sample.sensor_id});
        if (sample.node_id == 1U && sample.sensor_id == 1U) {
            latest_node1_sensor1 = sample.payload[0];
        }
    }

    assert(sample_count == 4);
    assert(pairs.size() == 4U);
    assert(pairs.count({1, 1}) == 1U);
    assert(pairs.count({1, 2}) == 1U);
    assert(pairs.count({2, 1}) == 1U);
    assert(pairs.count({2, 2}) == 1U);
    assert(latest_node1_sensor1 == 29);

    exo::RecordDoneMessage node2_done{};
    node2_done.command = exo::RecordCommand::RecordDone;
    node2_done.node_id = 2U;
    node2_done.session_id = 77U;
    node2_done.total_size = 2000U;
    node2_done.payload_crc32 = 0x22222222U;

    exo::RecordDoneMessage node1_done = node2_done;
    node1_done.node_id = 1U;
    node1_done.total_size = 1000U;
    node1_done.payload_crc32 = 0x11111111U;

    assert(manager.queue_record_done(node2_done));
    assert(manager.queue_record_done(node1_done));

    exo::RecordDoneMessage selected{};
    assert(manager.pop_next_record_done(selected));
    assert(selected.node_id == 1U);
    assert(manager.active_source_id() == 1U);
    manager.on_ble_reliable_verify_ok(77U, 1U, node1_done.payload_crc32);

    assert(manager.pop_next_record_done(selected));
    assert(selected.node_id == 2U);
    assert(manager.active_source_id() == 2U);

    std::cout << "hub leaf manager tests passed\n";
    return 0;
}
