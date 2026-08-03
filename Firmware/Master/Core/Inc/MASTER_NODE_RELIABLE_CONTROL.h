#ifndef MASTER_NODE_RELIABLE_CONTROL_H_
#define MASTER_NODE_RELIABLE_CONTROL_H_

#include <stdint.h>
#include <string.h>
#include <BLE_RECORD_PROTOCOL.h>

extern "C" uint8_t exo_hub_ble_write(const uint8_t *payload, uint8_t length)
        __attribute__((weak));

namespace exo {

class MasterNodeReliableControl {
public:
    using SendFn = bool (*)(void *context, uint8_t node_id,
            const uint8_t *frame, uint16_t length);

    explicit MasterNodeReliableControl(SendFn send_fn = nullptr, void *context = nullptr)
        : send_fn_(send_fn), context_(context)
    {
    }

    void set_transport(SendFn send_fn, void *context)
    {
        send_fn_ = send_fn;
        context_ = context;
    }

    void reset()
    {
        node_id_ = 0U;
        session_id_ = 0U;
        file_crc32_ = 0U;
        chunk_size_ = kRecordReliableDefaultChunkSize;
        pending_length_ = 0U;
        pending_priority_ = 0U;
        pending_type_ = RecordReliableType::ManifestAck;
        last_attempt_ms_ = 0U;
        attempt_count_ = 0U;
    }

    bool begin(const RecordDoneMessage &done,
            uint16_t chunk_size = kRecordReliableDefaultChunkSize,
            uint8_t credit = kDefaultCredit)
    {
        reset();
        if (done.node_id < 1U || done.node_id > 4U || done.session_id == 0U ||
                done.total_size == 0U || chunk_size == 0U) {
            return false;
        }
        node_id_ = static_cast<uint8_t>(done.node_id);
        session_id_ = done.session_id;
        file_crc32_ = done.payload_crc32;
        chunk_size_ = chunk_size;
        RecordReliableManifestAckPayload body{};
        body.source_id = done.node_id;
        body.session_id = done.session_id;
        body.accepted_chunk_size = chunk_size;
        body.credit = sanitize_credit(credit);
        body.status = 0U;
        return queue_frame(RecordReliableType::ManifestAck, 0U, 0U,
                reinterpret_cast<const uint8_t*>(&body), sizeof(body), 4U);
    }

    bool ack_window(uint32_t next_chunk, uint8_t credit = kDefaultCredit)
    {
        if (!active()) return false;
        RecordReliableAckWindowPayload body{};
        body.source_id = node_id_;
        body.session_id = session_id_;
        body.next_chunk_index = next_chunk;
        body.credit = sanitize_credit(credit);
        return queue_frame(RecordReliableType::AckWindow, next_chunk,
                next_chunk * static_cast<uint32_t>(chunk_size_),
                reinterpret_cast<const uint8_t*>(&body), sizeof(body), 1U);
    }

    bool nack_range(uint32_t first_chunk, uint16_t count = 1U,
            uint16_t flags = 0U)
    {
        if (!active()) return false;
        RecordReliableNackRangePayload body{};
        body.source_id = node_id_;
        body.session_id = session_id_;
        body.first_chunk_index = first_chunk;
        body.chunk_count = count == 0U ? 1U : count;
        body.flags = flags;
        return queue_frame(RecordReliableType::NackRange, first_chunk,
                first_chunk * static_cast<uint32_t>(chunk_size_),
                reinterpret_cast<const uint8_t*>(&body), sizeof(body), 3U);
    }

    bool verify_ok(uint32_t crc32)
    {
        if (!active() || crc32 != file_crc32_) return false;
        RecordReliableVerifyPayload body{};
        body.source_id = node_id_;
        body.session_id = session_id_;
        body.file_crc32 = crc32;
        body.first_bad_chunk = 0U;
        body.flags = 0U;
        return queue_frame(RecordReliableType::VerifyOk, 0U, 0U,
                reinterpret_cast<const uint8_t*>(&body), sizeof(body), 5U);
    }

    bool service(uint32_t now_ms)
    {
        if (pending_length_ == 0U) return true;
        if (attempt_count_ != 0U &&
                static_cast<uint32_t>(now_ms - last_attempt_ms_) < kRetryIntervalMs) {
            return false;
        }
        last_attempt_ms_ = now_ms;
        if (attempt_count_ < 0xFFFFFFFFU) ++attempt_count_;
        if (!send_pending()) return false;
        pending_length_ = 0U;
        pending_priority_ = 0U;
        return true;
    }

    bool active() const
    {
        return node_id_ >= 1U && node_id_ <= 4U && session_id_ != 0U;
    }
    bool pending() const { return pending_length_ != 0U; }
    uint8_t node_id() const { return node_id_; }
    uint32_t session_id() const { return session_id_; }
    RecordReliableType pending_type() const { return pending_type_; }
    uint32_t attempt_count() const { return attempt_count_; }
    const uint8_t *pending_frame() const { return pending_frame_; }
    uint16_t pending_length() const { return pending_length_; }

    static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
    {
        uint16_t crc = 0xFFFFU;
        if (data == nullptr && length != 0U) return 0U;
        for (uint16_t i = 0U; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8U;
            for (uint8_t bit = 0U; bit < 8U; ++bit) {
                crc = (crc & 0x8000U) != 0U ?
                        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
                        static_cast<uint16_t>(crc << 1U);
            }
        }
        return crc;
    }

private:
    static constexpr uint8_t kDefaultCredit = 8U;
    static constexpr uint32_t kRetryIntervalMs = 20U;
    static constexpr uint16_t kMaxFrameBytes = 64U;

    static uint8_t sanitize_credit(uint8_t credit)
    {
        if (credit == 0U) return 1U;
        return credit > 24U ? 24U : credit;
    }

    bool queue_frame(RecordReliableType type, uint32_t chunk_index,
            uint32_t byte_offset, const uint8_t *body, uint16_t body_length,
            uint8_t priority)
    {
        if (!active() || body == nullptr ||
                sizeof(RecordReliableFrameHeader) + body_length > kMaxFrameBytes) {
            return false;
        }
        if (pending_length_ != 0U && priority < pending_priority_) {
            return false;
        }
        RecordReliableFrameHeader header{};
        header.command = RecordCommand::ReliableFrame;
        header.proto_version = kRecordReliableProtoVersion;
        header.magic = kRecordReliableMagic;
        header.frame_type = static_cast<uint8_t>(type);
        header.source_id = node_id_;
        header.session_id = session_id_;
        header.chunk_index = chunk_index;
        header.byte_offset = byte_offset;
        header.payload_len = body_length;
        header.payload_crc16 = crc16_ccitt(body, body_length);
        header.flags = 0U;
        memcpy(pending_frame_, &header, sizeof(header));
        memcpy(pending_frame_ + sizeof(header), body, body_length);
        pending_length_ = static_cast<uint16_t>(sizeof(header) + body_length);
        pending_priority_ = priority;
        pending_type_ = type;
        last_attempt_ms_ = 0U;
        attempt_count_ = 0U;
        return true;
    }

    bool send_pending()
    {
        if (send_fn_ != nullptr) {
            return send_fn_(context_, node_id_, pending_frame_, pending_length_);
        }
        if (exo_hub_ble_write == nullptr || pending_length_ > 0xFFU) {
            return false;
        }
        return exo_hub_ble_write(pending_frame_,
                static_cast<uint8_t>(pending_length_)) != 0U;
    }

    SendFn send_fn_ = nullptr;
    void *context_ = nullptr;
    uint8_t pending_frame_[kMaxFrameBytes]{};
    uint16_t pending_length_ = 0U;
    uint8_t pending_priority_ = 0U;
    RecordReliableType pending_type_ = RecordReliableType::ManifestAck;
    uint8_t node_id_ = 0U;
    uint32_t session_id_ = 0U;
    uint32_t file_crc32_ = 0U;
    uint16_t chunk_size_ = kRecordReliableDefaultChunkSize;
    uint32_t last_attempt_ms_ = 0U;
    uint32_t attempt_count_ = 0U;
};

} // namespace exo

#endif
