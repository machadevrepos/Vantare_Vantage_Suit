#ifndef HUB_LEAF_BLE_MANAGER_H_
#define HUB_LEAF_BLE_MANAGER_H_

#include <stdint.h>
#include <string.h>
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

  void begin() { discovery_generation_ = 1U; }
  void process() {}

  bool start_from_ble(const exo::StartRecordMessage &) {
    start_or_record_active_ = true;
    queued_done_count_ = 0U;
    active_source_id_ = 0U;
    active_session_id_ = 0U;
    return true;
  }

  bool start_or_record_active() const {
    return start_or_record_active_ || queued_done_count_ != 0U ||
           live_sample_count_ != 0U || active_source_id_ != 0U;
  }

  bool pop_next_record_done(exo::RecordDoneMessage &out) {
    if (queued_done_count_ == 0U) {
      start_or_record_active_ = transfer_hold_ || active_source_id_ != 0U;
      return false;
    }
    uint8_t best = 0U;
    for (uint8_t i = 1U; i < queued_done_count_; ++i) {
      if (queued_done_[i].node_id < queued_done_[best].node_id) {
        best = i;
      }
    }
    out = queued_done_[best];
    for (uint8_t i = best; i + 1U < queued_done_count_; ++i) {
      queued_done_[i] = queued_done_[i + 1U];
    }
    queued_done_[queued_done_count_ - 1U] = exo::RecordDoneMessage{};
    --queued_done_count_;
    active_source_id_ = out.node_id;
    active_session_id_ = out.session_id;
    next_chunk_index_ = 0U;
    next_offset_ = 0U;
    receiver_credit_ = 0U;
    paused_ = false;
    start_or_record_active_ = true;
    return true;
  }

  bool push_leaf_sample(uint8_t node_id, uint8_t sensor_id,
                        const uint8_t *payload, uint8_t payload_len) {
    if (node_id == 0U || payload == nullptr || payload_len == 0U ||
        payload_len > sizeof(LiveSample::payload)) {
      return false;
    }
    touch_node(node_id);

    // Live preview is a latest-value display, not a lossless recording path.
    // Coalesce an unsent sample from the same Node/sensor pair so one high-rate
    // source cannot evict every other Node from the small BLE preview queue.
    for (uint8_t i = 0U; i < live_sample_count_; ++i) {
      const uint8_t index = static_cast<uint8_t>(
          (live_sample_head_ + i) % kMaxQueuedSamples);
      LiveSample &pending = live_samples_[index];
      if (pending.node_id == node_id && pending.sensor_id == sensor_id) {
        pending.payload_len = payload_len;
        memcpy(pending.payload, payload, payload_len);
        start_or_record_active_ = true;
        return true;
      }
    }

    if (live_sample_count_ >= kMaxQueuedSamples) {
      live_sample_head_ = static_cast<uint8_t>((live_sample_head_ + 1U) % kMaxQueuedSamples);
      --live_sample_count_;
    }
    const uint8_t write_index = static_cast<uint8_t>(
        (live_sample_head_ + live_sample_count_) % kMaxQueuedSamples);
    LiveSample &sample = live_samples_[write_index];
    sample.node_id = node_id;
    sample.sensor_id = sensor_id;
    sample.payload_len = payload_len;
    memcpy(sample.payload, payload, payload_len);
    ++live_sample_count_;
    start_or_record_active_ = true;
    return true;
  }

  bool peek_next_live_sample(LiveSample &out) const {
    if (live_sample_count_ == 0U) return false;
    out = live_samples_[live_sample_head_];
    return true;
  }

  bool discard_next_live_sample() {
    if (live_sample_count_ == 0U) return false;
    live_samples_[live_sample_head_] = LiveSample{};
    live_sample_head_ = static_cast<uint8_t>((live_sample_head_ + 1U) % kMaxQueuedSamples);
    --live_sample_count_;
    if (live_sample_count_ == 0U && queued_done_count_ == 0U &&
        !transfer_hold_ && active_source_id_ == 0U) {
      start_or_record_active_ = false;
    }
    return true;
  }

  bool pop_next_live_sample(LiveSample &out) {
    return peek_next_live_sample(out) && discard_next_live_sample();
  }

  bool queue_record_done(const exo::RecordDoneMessage &message) {
    if (message.node_id < 1U || message.node_id > kMaxLeaves) return false;
    touch_node(static_cast<uint8_t>(message.node_id));
    if (active_source_id_ == message.node_id && active_session_id_ == message.session_id) {
      start_or_record_active_ = true;
      return true;
    }
    for (uint8_t i = 0U; i < queued_done_count_; ++i) {
      if (record_done_matches_(queued_done_[i], message)) return true;
    }
    if (queued_done_count_ >= kMaxLeaves) return false;
    queued_done_[queued_done_count_++] = message;
    start_or_record_active_ = true;
    return true;
  }

  bool reset_and_abort_all(bool) {
    memset(queued_done_, 0, sizeof(queued_done_));
    queued_done_count_ = 0U;
    memset(live_samples_, 0, sizeof(live_samples_));
    live_sample_count_ = 0U;
    live_sample_head_ = 0U;
    active_source_id_ = 0U;
    active_session_id_ = 0U;
    next_chunk_index_ = 0U;
    next_offset_ = 0U;
    receiver_credit_ = 0U;
    paused_ = false;
    start_or_record_active_ = false;
    transfer_hold_ = false;
    return true;
  }

  void set_transfer_hold(bool hold) {
    transfer_hold_ = hold;
    if (hold) start_or_record_active_ = true;
    else if (queued_done_count_ == 0U && active_source_id_ == 0U)
      start_or_record_active_ = false;
  }

  uint8_t copy_discovered_node_ids(uint8_t *out, uint8_t max_len) const {
    if (out == nullptr || max_len == 0U) return 0U;
    const uint8_t n = discovered_count_ < max_len ? discovered_count_ : max_len;
    for (uint8_t i = 0U; i < n; ++i) out[i] = nodes_[i].node_id;
    return n;
  }

  void rediscover_nodes() { ++discovery_generation_; }

  void touch_node(uint8_t node_id) {
    if (node_id < 1U || node_id > kMaxLeaves || find_node_index(node_id) >= 0) return;
    const int8_t slot = find_free_slot();
    if (slot < 0) return;
    nodes_[slot].node_id = node_id;
    nodes_[slot].discovered = true;
    nodes_[slot].connected = true;
    recompute_count_and_sort();
  }

  bool provision_node_id(uint8_t current_id, uint8_t new_id) {
    if (new_id < 1U || new_id > kMaxLeaves) return false;
    const int8_t existing = find_node_index(new_id);
    if (existing >= 0 && nodes_[existing].node_id != current_id) return false;
    int8_t slot = find_node_index(current_id);
    if (slot < 0) slot = find_free_slot();
    if (slot < 0) return false;
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
    if (node_id == 0U && discovered_count_ == 1U) {
      resolved_id = nodes_[0].node_id;
      return true;
    }
    return false;
  }

  void on_ble_reliable_nack_range(uint32_t session_id, uint16_t source_id,
                                  uint32_t first_chunk, uint8_t count) {
    if (!owns_transfer_(session_id, source_id)) return;
    next_chunk_index_ = first_chunk;
    next_offset_ = first_chunk * exo::kRecordReliableDefaultChunkSize;
    receiver_credit_ = count == 0U ? 1U : count;
    paused_ = false;
  }

  void on_ble_reliable_ack_window(uint32_t session_id, uint16_t source_id,
                                  uint32_t next_chunk, uint8_t credit) {
    if (!owns_transfer_(session_id, source_id)) return;
    if (next_chunk >= next_chunk_index_) {
      next_chunk_index_ = next_chunk;
      next_offset_ = next_chunk * exo::kRecordReliableDefaultChunkSize;
    }
    receiver_credit_ = credit;
    paused_ = false;
  }

  void on_ble_reliable_pause(uint32_t session_id, uint16_t source_id) {
    if (!owns_transfer_(session_id, source_id)) return;
    paused_ = true;
    receiver_credit_ = 0U;
  }

  void on_ble_reliable_verify_ok(uint32_t session_id, uint16_t source_id,
                                 uint32_t payload_crc32) {
    if (!owns_transfer_(session_id, source_id)) return;
    verified_crc32_ = payload_crc32;
    active_source_id_ = 0U;
    active_session_id_ = 0U;
    receiver_credit_ = 0U;
    paused_ = false;
    if (queued_done_count_ == 0U && !transfer_hold_) start_or_record_active_ = false;
  }

  void on_ble_chunk_ack(uint32_t session_id, uint16_t source_id,
                        uint32_t next_offset) {
    if (!owns_transfer_(session_id, source_id)) return;
    if (next_offset >= next_offset_) {
      next_offset_ = next_offset;
      next_chunk_index_ = next_offset / exo::kRecordReliableDefaultChunkSize;
    }
  }

  uint16_t active_source_id() const { return active_source_id_; }
  uint32_t active_session_id() const { return active_session_id_; }
  uint32_t next_chunk_index() const { return next_chunk_index_; }
  uint32_t next_offset() const { return next_offset_; }
  uint8_t receiver_credit() const { return receiver_credit_; }
  bool paused() const { return paused_; }

private:
  struct NodeSlot { uint8_t node_id = 0U; bool discovered = false; bool connected = false; };

  bool owns_transfer_(uint32_t session_id, uint16_t source_id) const {
    return active_source_id_ == source_id && active_session_id_ == session_id;
  }

  int8_t find_node_index(uint8_t node_id) const {
    for (uint8_t i = 0U; i < discovered_count_; ++i)
      if (nodes_[i].discovered && nodes_[i].node_id == node_id) return static_cast<int8_t>(i);
    return -1;
  }

  int8_t find_free_slot() const {
    for (uint8_t i = 0U; i < kMaxLeaves; ++i)
      if (!nodes_[i].discovered || nodes_[i].node_id == 0U) return static_cast<int8_t>(i);
    return -1;
  }

  void recompute_count_and_sort() {
    uint8_t write = 0U;
    for (uint8_t i = 0U; i < kMaxLeaves; ++i) {
      if (nodes_[i].discovered && nodes_[i].node_id != 0U) {
        if (write != i) { nodes_[write] = nodes_[i]; nodes_[i] = NodeSlot{}; }
        ++write;
      }
    }
    discovered_count_ = write;
    for (uint8_t i = 0U; i < discovered_count_; ++i)
      for (uint8_t j = static_cast<uint8_t>(i + 1U); j < discovered_count_; ++j)
        if (nodes_[j].node_id < nodes_[i].node_id) {
          const NodeSlot tmp = nodes_[i]; nodes_[i] = nodes_[j]; nodes_[j] = tmp;
        }
  }

  static bool record_done_matches_(const exo::RecordDoneMessage &a,
                                   const exo::RecordDoneMessage &b) {
    return a.node_id == b.node_id && a.session_id == b.session_id &&
           a.total_size == b.total_size && a.payload_crc32 == b.payload_crc32;
  }

  static constexpr uint8_t kMaxLeaves = 4U;
  static constexpr uint8_t kMaxQueuedSamples = 8U;
  NodeSlot nodes_[kMaxLeaves]{};
  uint8_t discovered_count_ = 0U;
  exo::RecordDoneMessage queued_done_[kMaxLeaves]{};
  uint8_t queued_done_count_ = 0U;
  LiveSample live_samples_[kMaxQueuedSamples]{};
  uint8_t live_sample_count_ = 0U;
  uint8_t live_sample_head_ = 0U;
  bool start_or_record_active_ = false;
  bool transfer_hold_ = false;
  uint16_t active_source_id_ = 0U;
  uint32_t active_session_id_ = 0U;
  uint32_t next_chunk_index_ = 0U;
  uint32_t next_offset_ = 0U;
  uint32_t verified_crc32_ = 0U;
  uint8_t receiver_credit_ = 0U;
  bool paused_ = false;
  uint32_t discovery_generation_ = 0U;
};

} // namespace exo::ble_hub

#endif
