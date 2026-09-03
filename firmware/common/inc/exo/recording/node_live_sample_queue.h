#ifndef NODE_LIVE_SAMPLE_QUEUE_H_
#define NODE_LIVE_SAMPLE_QUEUE_H_

#include <stdint.h>
#include <string.h>

namespace exo {

template<uint8_t MaxPayload>
struct NodeLiveSample {
    uint8_t sensor_id = 0U;
    uint8_t payload_len = 0U;
    uint32_t acquisition_time_ms = 0U;
    uint8_t payload[MaxPayload] {};
};

template<uint8_t MaxPayload, uint8_t Capacity>
class NodeLiveSampleQueue {
public:
    static_assert(Capacity > 0U, "NodeLiveSampleQueue Capacity must be non-zero");

    static constexpr uint32_t kMinimumPreviewIntervalMs = 40U;
    static constexpr uint32_t kMaxPreviewIntervalMs = 80U;

    void configure(bool enabled, uint32_t interval_ms)
    {
        enabled_ = enabled;
        interval_ms_ = clamp_interval(interval_ms);
        clear();
    }

    bool offer(uint8_t sensor_id, const void *payload, uint8_t payload_len,
               uint32_t acquisition_time_ms)
    {
        if (!enabled_ || payload == nullptr || payload_len == 0U ||
            payload_len > MaxPayload) {
            return false;
        }
        const int8_t slot = sensor_slot(sensor_id);
        if (slot < 0) {
            return false;
        }
        const uint8_t index = static_cast<uint8_t>(slot);
        if (gate_valid_[index] &&
            static_cast<uint32_t>(acquisition_time_ms - gate_time_ms_[index]) <
                interval_ms_) {
            ++decimated_;
            return false;
        }
        gate_valid_[index] = true;
        gate_time_ms_[index] = acquisition_time_ms;

        if (count_ >= Capacity) {
            /* The link cannot drain fast enough right now. Keep the freshest
             * Capacity samples: for live ML inference on a fixed grid a recent
             * gap is far better than throttling production to a fraction of the
             * contract rate (the old 5 s congestion latch). Count the loss. */
            ++dropped_;
            (void)discard_front();
        }

        const uint8_t write_index =
            static_cast<uint8_t>((tail_ + count_) % Capacity);
        NodeLiveSample<MaxPayload> &entry = entries_[write_index];
        entry.sensor_id = sensor_id;
        entry.payload_len = payload_len;
        entry.acquisition_time_ms = acquisition_time_ms;
        memcpy(entry.payload, payload, payload_len);
        ++count_;
        ++accepted_;
        return true;
    }

    bool peek(NodeLiveSample<MaxPayload> &out) const
    {
        if (count_ == 0U) return false;
        out = entries_[tail_];
        return true;
    }

    bool discard_front()
    {
        if (count_ == 0U) return false;
        entries_[tail_] = NodeLiveSample<MaxPayload>{};
        tail_ = static_cast<uint8_t>((tail_ + 1U) % Capacity);
        --count_;
        return true;
    }

    bool pop(NodeLiveSample<MaxPayload> &out)
    {
        if (!peek(out)) {
            return false;
        }
        return discard_front();
    }

    uint32_t dropped() const { return dropped_; }
    uint32_t accepted() const { return accepted_; }
    uint32_t coalesced() const { return 0U; }
    uint32_t decimated() const { return decimated_; }
    bool congested() const { return false; }
    uint32_t effective_interval_ms() const { return interval_ms_; }
    uint32_t normal_interval_ms() const { return interval_ms_; }
    uint8_t count() const { return count_; }

    void clear()
    {
        for (uint8_t i = 0U; i < Capacity; ++i) {
            entries_[i] = NodeLiveSample<MaxPayload>{};
        }
        gate_valid_[0] = false;
        gate_valid_[1] = false;
        gate_time_ms_[0] = 0U;
        gate_time_ms_[1] = 0U;
        tail_ = 0U;
        count_ = 0U;
        dropped_ = 0U;
        accepted_ = 0U;
        decimated_ = 0U;
    }

private:
    static uint32_t clamp_interval(uint32_t interval_ms)
    {
        if (interval_ms < kMinimumPreviewIntervalMs) {
            return kMinimumPreviewIntervalMs;
        }
        if (interval_ms > kMaxPreviewIntervalMs) {
            return kMaxPreviewIntervalMs;
        }
        return interval_ms;
    }

    static int8_t sensor_slot(uint8_t sensor_id)
    {
        if (sensor_id == 1U) return 0;
        if (sensor_id == 2U) return 1;
        return -1;
    }

    NodeLiveSample<MaxPayload> entries_[Capacity] {};
    bool gate_valid_[2] { false, false };
    uint32_t gate_time_ms_[2] { 0U, 0U };
    uint8_t tail_ = 0U;
    uint8_t count_ = 0U;
    uint32_t interval_ms_ = kMinimumPreviewIntervalMs;
    uint32_t dropped_ = 0U;
    uint32_t accepted_ = 0U;
    uint32_t decimated_ = 0U;
    bool enabled_ = false;
};

} // namespace exo

#endif
