#ifndef HUB_LEAF_BLE_MANAGER_H_
#define HUB_LEAF_BLE_MANAGER_H_

#include <string.h>
#include <stdint.h>
#include <BLE_RECORD_PROTOCOL.h>

namespace exo::ble_hub {

class HubLeafBleManager {
public:
  struct LiveSample {
    uint8_t node_id = 0U;
    uint8_t sensor_id = 0U;
    uint8_t payload_len = 0U;
    uint8_t payload[96]{};
  };

  HubLeafBleManager() = default;

  void begin() { discovery_generation_ = 1U; }
  void process() {}

  bool start_from_ble(const exo::StartRecordMessage &) {
    start_or_record_active_ = true;
    queued_done_count_ = 0U;
    queued_done_head_ = 0U;
    return true;
  }

  bool start_or_record_active() const {
    return start_or_record_active_ || (queued_done_count_ != 0U) || (live_sample_count_ != 0U);
  }

  bool pop_next_record_done(exo::RecordDoneMessage &out) {
    if (queued_done_count_ == 0U) {
      start_or_record_active_ = transfer_hold_;
      return false;
    }
    out = queued_done_[queued_done_head_];
    queued_done_head_ = static_cast<uint8_t>((queued_done_head_ + 1U) % kMaxLeaves);
    queued_done_count_--;
    if ((queued_done_count_ == 0U) && !transfer_hold_) {
      start_or_record_active_ = false;
    }
    return true;
  }

  bool push_leaf_sample(uint8_t node_id,
                        uint8_t sensor_id,
                        const uint8_t *payload,
                        uint8_t payload_len) {
    if (node_id == 0U || payload == nullptr || payload_len == 0U || payload_len > sizeof(LiveSample::payload)) {
      return false;
    }
    touch_node(node_id);
    if (live_sample_count_ >= kMaxQueuedSamples) {
      const uint8_t drop_index = live_sample_head_;
      live_sample_head_ = static_cast<uint8_t>((live_sample_head_ + 1U) % kMaxQueuedSamples);
      live_sample_count_--;
      live_samples_[drop_index] = LiveSample{};
    }
    const uint8_t write_index = static_cast<uint8_t>((live_sample_head_ + live_sample_count_) % kMaxQueuedSamples);
    live_samples_[write_index].node_id = node_id;
    live_samples_[write_index].sensor_id = sensor_id;
    live_samples_[write_index].payload_len = payload_len;
    memcpy(live_samples_[write_index].payload, payload, payload_len);
    live_sample_count_++;
    start_or_record_active_ = true;
    return true;
  }

  bool peek_next_live_sample(LiveSample &out) const {
    if (live_sample_count_ == 0U) {
      return false;
    }
    out = live_samples_[live_sample_head_];
    return true;
  }

  bool discard_next_live_sample() {
    if (live_sample_count_ == 0U) {
      return false;
    }
    live_samples_[live_sample_head_] = LiveSample{};
    live_sample_head_ = static_cast<uint8_t>((live_sample_head_ + 1U) % kMaxQueuedSamples);
    live_sample_count_--;
    if (live_sample_count_ == 0U && queued_done_count_ == 0U && !transfer_hold_) {
      start_or_record_active_ = false;
    }
    return true;
  }

  bool pop_next_live_sample(LiveSample &out) {
    if (!peek_next_live_sample(out)) {
      return false;
    }
    return discard_next_live_sample();
  }

  bool queue_record_done(const exo::RecordDoneMessage &message) {
    if (message.node_id == 0U) {
      return false;
    }
    touch_node(message.node_id);
    for (uint8_t i = 0U; i < queued_done_count_; ++i) {
      const uint8_t idx = static_cast<uint8_t>((queued_done_head_ + i) % kMaxLeaves);
      if (record_done_matches_(queued_done_[idx], message)) {
        start_or_record_active_ = true;
        return true;
      }
    }
    if (queued_done_count_ >= kMaxLeaves) {
      return false;
    }
    const uint8_t write_index = static_cast<uint8_t>((queued_done_head_ + queued_done_count_) % kMaxLeaves);
    queued_done_[write_index] = message;
    queued_done_count_++;
    start_or_record_active_ = true;
    return true;
  }

  bool reset_and_abort_all(bool) {
    queued_done_count_ = 0U;
    queued_done_head_ = 0U;
    live_sample_count_ = 0U;
    live_sample_head_ = 0U;
    start_or_record_active_ = false;
    transfer_hold_ = false;
    return true;
  }

  void set_transfer_hold(bool hold) {
    transfer_hold_ = hold;
    if (hold) {
      start_or_record_active_ = true;
    } else if (queued_done_count_ == 0U) {
      start_or_record_active_ = false;
    }
  }

  uint8_t copy_discovered_node_ids(uint8_t *out, uint8_t max_len) const {
    if ((out == nullptr) || (max_len == 0U)) {
      return 0U;
    }
    const uint8_t n = discovered_count_ < max_len ? discovered_count_ : max_len;
    for (uint8_t i = 0U; i < n; ++i) {
      out[i] = nodes_[i].node_id;
    }
    return n;
  }

  void rediscover_nodes() {
    ++discovery_generation_;
  }

  void touch_node(uint8_t node_id) {
    if (node_id == 0U) {
      return;
    }
    if (find_node_index(node_id) >= 0) {
      return;
    }
    const int8_t slot = find_free_slot();
    if (slot < 0) {
      return;
    }
    nodes_[slot].node_id = node_id;
    nodes_[slot].discovered = true;
    nodes_[slot].connected = true;
    recompute_count_and_sort();
  }

  bool provision_node_id(uint8_t current_id, uint8_t new_id) {
    if (new_id == 0U) {
      return false;
    }
    const int8_t existing_new = find_node_index(new_id);
    if (existing_new >= 0 && static_cast<uint8_t>(nodes_[existing_new].node_id) != current_id) {
      return false;
    }
    int8_t slot = find_node_index(current_id);
    if (slot < 0) {
      slot = find_free_slot();
      if (slot < 0) {
        return false;
      }
    }
    nodes_[slot].node_id = new_id;
    nodes_[slot].discovered = true;
    nodes_[slot].connected = false;
    recompute_count_and_sort();
    return true;
  }

  bool request_node_id(uint8_t node_id, uint8_t &resolved_id) const {
    const int8_t slot = find_node_index(node_id);
    if (slot >= 0) {
      resolved_id = nodes_[slot].node_id;
      return true;
    }
    if ((node_id == 0U) && (discovered_count_ == 1U)) {
      resolved_id = nodes_[0].node_id;
      return true;
    }
    return false;
  }

  void on_ble_reliable_nack_range(uint32_t, uint16_t, uint32_t, uint8_t) {}
  void on_ble_reliable_ack_window(uint32_t, uint16_t, uint32_t, uint8_t) {}
  void on_ble_reliable_pause(uint32_t, uint16_t) {}
  void on_ble_reliable_verify_ok(uint32_t, uint16_t, uint32_t) {}
  void on_ble_chunk_ack(uint32_t, uint16_t, uint32_t) {}

private:
  struct NodeSlot {
    uint8_t node_id = 0U;
    bool discovered = false;
    bool connected = false;
  };

  int8_t find_node_index(uint8_t node_id) const {
    if (node_id == 0U) {
      return -1;
    }
    for (uint8_t i = 0U; i < discovered_count_; ++i) {
      if (nodes_[i].discovered && nodes_[i].node_id == node_id) {
        return static_cast<int8_t>(i);
      }
    }
    return -1;
  }

  int8_t find_free_slot() const {
    for (uint8_t i = 0U; i < kMaxLeaves; ++i) {
      if (!nodes_[i].discovered || nodes_[i].node_id == 0U) {
        return static_cast<int8_t>(i);
      }
    }
    return -1;
  }

  void recompute_count_and_sort() {
    uint8_t write = 0U;
    for (uint8_t i = 0U; i < kMaxLeaves; ++i) {
      if (nodes_[i].discovered && nodes_[i].node_id != 0U) {
        if (write != i) {
          nodes_[write] = nodes_[i];
          nodes_[i] = NodeSlot{};
        }
        ++write;
      }
    }
    discovered_count_ = write;
    for (uint8_t i = 0U; i < discovered_count_; ++i) {
      for (uint8_t j = static_cast<uint8_t>(i + 1U); j < discovered_count_; ++j) {
        if (nodes_[j].node_id < nodes_[i].node_id) {
          const NodeSlot tmp = nodes_[i];
          nodes_[i] = nodes_[j];
          nodes_[j] = tmp;
        }
      }
    }
  }

  static bool record_done_matches_(const exo::RecordDoneMessage &lhs,
                                   const exo::RecordDoneMessage &rhs) {
    return lhs.node_id == rhs.node_id &&
           lhs.session_id == rhs.session_id &&
           lhs.total_size == rhs.total_size &&
           lhs.payload_crc32 == rhs.payload_crc32;
  }

  static constexpr uint8_t kMaxLeaves = 4U;
  static constexpr uint8_t kMaxQueuedSamples = 8U;
  NodeSlot nodes_[kMaxLeaves]{};
  uint8_t discovered_count_ = 0U;
  exo::RecordDoneMessage queued_done_[kMaxLeaves]{};
  uint8_t queued_done_count_ = 0U;
  uint8_t queued_done_head_ = 0U;
  LiveSample live_samples_[kMaxQueuedSamples]{};
  uint8_t live_sample_count_ = 0U;
  uint8_t live_sample_head_ = 0U;
  bool start_or_record_active_ = false;
  bool transfer_hold_ = false;
  uint32_t discovery_generation_ = 0U;
};

} // namespace exo::ble_hub

#endif
