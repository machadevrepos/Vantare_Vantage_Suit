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
    void configure(bool enabled, uint32_t interval_ms)
    {
        enabled_ = enabled;
        interval_ms_ = interval_ms == 0U ? 1U : interval_ms;
        clear();
    }

    bool offer(uint8_t sensor_id, const void *payload, uint8_t payload_len,
               uint32_t acquisition_time_ms)
    {
        if (!enabled_ || payload == nullptr || payload_len == 0U ||
            payload_len > MaxPayload || Capacity == 0U) {
            return false;
        }
        const uint8_t gate = sensor_id == 2U ? 1U : 0U;
        if (gate_valid_[gate] &&
            static_cast<uint32_t>(acquisition_time_ms - gate_time_ms_[gate]) <
                interval_ms_) {
            return false;
        }
        gate_valid_[gate] = true;
        gate_time_ms_[gate] = acquisition_time_ms;
        if (count_ >= Capacity) {
            ++dropped_;
            tail_ = static_cast<uint8_t>((tail_ + 1U) % Capacity);
            --count_;
        }
        NodeLiveSample<MaxPayload> &entry = entries_[head_];
        entry.sensor_id = sensor_id;
        entry.payload_len = payload_len;
        entry.acquisition_time_ms = acquisition_time_ms;
        memcpy(entry.payload, payload, payload_len);
        head_ = static_cast<uint8_t>((head_ + 1U) % Capacity);
        ++count_;
        return true;
    }

    bool peek(NodeLiveSample<MaxPayload> &out) const
    {
        if (count_ == 0U) {
            return false;
        }
        out = entries_[tail_];
        return true;
    }

    bool discard_front()
    {
        if (count_ == 0U) {
            return false;
        }
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

    void clear()
    {
        head_ = 0U;
        tail_ = 0U;
        count_ = 0U;
        dropped_ = 0U;
        gate_valid_[0] = false;
        gate_valid_[1] = false;
        gate_time_ms_[0] = 0U;
        gate_time_ms_[1] = 0U;
    }

private:
    NodeLiveSample<MaxPayload> entries_[Capacity == 0U ? 1U : Capacity] {};
    uint8_t head_ = 0U;
    uint8_t tail_ = 0U;
    uint8_t count_ = 0U;
    uint32_t dropped_ = 0U;
    uint32_t interval_ms_ = 20U;
    uint32_t gate_time_ms_[2] { 0U, 0U };
    bool gate_valid_[2] { false, false };
    bool enabled_ = false;
};

} // namespace exo

#endif
