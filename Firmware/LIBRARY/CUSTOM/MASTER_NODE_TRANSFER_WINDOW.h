#ifndef MASTER_NODE_TRANSFER_WINDOW_H_
#define MASTER_NODE_TRANSFER_WINDOW_H_

#include <stdint.h>

namespace exo {

enum class NodeTransferDecision : uint8_t {
    Ignore,
    Accept,
    Duplicate,
    NackGap,
    NackCorrupt,
    Complete
};

struct NodeTransferInspection {
    NodeTransferDecision decision = NodeTransferDecision::Ignore;
    uint32_t request_chunk = 0U;
    uint32_t next_chunk = 0U;
    uint32_t next_offset = 0U;
};

class MasterNodeTransferWindow {
public:
    bool begin(uint8_t node_id, uint32_t session_id, uint32_t total_size,
            uint16_t chunk_size)
    {
        reset();
        if (node_id < 1U || node_id > 4U || session_id == 0U ||
                total_size == 0U || chunk_size == 0U) {
            return false;
        }
        active_ = true;
        node_id_ = node_id;
        session_id_ = session_id;
        total_size_ = total_size;
        chunk_size_ = chunk_size;
        return true;
    }

    void reset()
    {
        active_ = false;
        node_id_ = 0U;
        session_id_ = 0U;
        total_size_ = 0U;
        chunk_size_ = 0U;
        next_chunk_ = 0U;
        next_offset_ = 0U;
    }

    NodeTransferInspection inspect(uint8_t node_id, uint32_t session_id,
            uint32_t chunk_index, uint32_t byte_offset, uint16_t payload_size,
            bool payload_crc_ok, bool final_chunk) const
    {
        NodeTransferInspection result{};
        result.request_chunk = next_chunk_;
        result.next_chunk = next_chunk_;
        result.next_offset = next_offset_;
        if (!active_ || node_id != node_id_ || session_id != session_id_ ||
                payload_size == 0U) {
            return result;
        }
        if (!payload_crc_ok ||
                static_cast<uint64_t>(byte_offset) + payload_size > total_size_) {
            result.decision = NodeTransferDecision::NackCorrupt;
            result.request_chunk = chunk_index;
            return result;
        }
        if (byte_offset < next_offset_) {
            result.decision =
                    static_cast<uint64_t>(byte_offset) + payload_size <= next_offset_ ?
                    NodeTransferDecision::Duplicate : NodeTransferDecision::NackGap;
            return result;
        }
        if (byte_offset != next_offset_ || chunk_index != next_chunk_) {
            result.decision = NodeTransferDecision::NackGap;
            return result;
        }
        const uint32_t end = byte_offset + payload_size;
        if (final_chunk && end != total_size_) {
            result.decision = NodeTransferDecision::NackGap;
            return result;
        }
        result.decision = end == total_size_ ?
                NodeTransferDecision::Complete : NodeTransferDecision::Accept;
        result.next_chunk = next_chunk_ + 1U;
        result.next_offset = end;
        result.request_chunk = result.next_chunk;
        return result;
    }

    bool commit(const NodeTransferInspection &inspection)
    {
        if (!active_ ||
                (inspection.decision != NodeTransferDecision::Accept &&
                 inspection.decision != NodeTransferDecision::Complete) ||
                inspection.next_offset < next_offset_ ||
                inspection.next_offset > total_size_) {
            return false;
        }
        next_chunk_ = inspection.next_chunk;
        next_offset_ = inspection.next_offset;
        return true;
    }

    bool active() const { return active_; }
    uint8_t node_id() const { return node_id_; }
    uint32_t session_id() const { return session_id_; }
    uint32_t next_chunk() const { return next_chunk_; }
    uint32_t next_offset() const { return next_offset_; }
    bool complete() const { return active_ && next_offset_ == total_size_; }

private:
    bool active_ = false;
    uint8_t node_id_ = 0U;
    uint32_t session_id_ = 0U;
    uint32_t total_size_ = 0U;
    uint16_t chunk_size_ = 0U;
    uint32_t next_chunk_ = 0U;
    uint32_t next_offset_ = 0U;
};

} // namespace exo

#endif
