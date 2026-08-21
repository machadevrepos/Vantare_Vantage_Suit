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

    /* A node that has spent its credit stays silent until the next control
     * frame arrives, so a single dropped ACK_WINDOW deadlocks the transfer.
     * Re-advertising the window after this long gives the node a fresh grant
     * without waiting for the session stall timeout. */
    static constexpr uint32_t kAckKeepaliveMs = 300U;

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
        nack_pending_ = false;
        nack_first_chunk_ = 0U;
        nack_count_ = 1U;
        nack_flags_ = 0U;
        nack_last_attempt_ms_ = 0U;
        nack_attempt_count_ = 0U;
        ack_pending_ = false;
        ack_valid_ = false;
        ack_next_chunk_ = 0U;
        ack_credit_ = kDefaultCredit;
        ack_last_sent_ms_ = 0U;
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

    /* The window advertisement is pure idempotent state, so it gets its own slot
     * instead of competing with control frames for the single pending buffer.
     * Losing an ACK to a higher-priority NACK used to starve the node of credit. */
    /* Durability note: the stager may still hold up to one write block of
     * accepted chunks in RAM when this ACK is sent; a power loss before the
     * block flush loses acknowledged bytes. End-of-transfer CRC validation
     * catches that, so no retransmit protocol is needed for it. */
    bool ack_window(uint32_t next_chunk, uint8_t credit = kDefaultCredit)
    {
        if (!active()) return false;
        if (!chunk_offset_valid_(next_chunk, chunk_size_)) return false;
        ack_next_chunk_ = next_chunk;
        ack_credit_ = sanitize_credit(credit);
        ack_pending_ = true;
        return true;
    }

    bool nack_range(uint32_t first_chunk, uint16_t count = 1U,
            uint16_t flags = 0U)
    {
        if (!active()) return false;
        if (!chunk_offset_valid_(first_chunk, chunk_size_)) return false;
        /* Gap/corrupt recovery must never be lost, so the NACK gets its own slot
         * instead of competing with ManifestAck/VerifyOk for the single pending
         * buffer. A rejected NACK used to leave the node idle until the stall
         * timeout because an ACK_WINDOW alone cannot rewind the node cursor. A
         * newer NACK overwrites an unsent older one: the window is strictly
         * sequential, so both would request the same recovery chunk anyway. */
        nack_first_chunk_ = first_chunk;
        nack_count_ = count == 0U ? 1U : count;
        nack_flags_ = flags;
        nack_pending_ = true;
        nack_last_attempt_ms_ = 0U;
        nack_attempt_count_ = 0U;
        return true;
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
        if (pending_length_ != 0U) {
            if (attempt_count_ != 0U &&
                    static_cast<uint32_t>(now_ms - last_attempt_ms_) < kRetryIntervalMs) {
                return false;
            }
            last_attempt_ms_ = now_ms;
            if (attempt_count_ < 0xFFFFFFFFU) ++attempt_count_;
            if (!transmit(pending_frame_, pending_length_)) return false;
            pending_length_ = 0U;
            pending_priority_ = 0U;
            return true;
        }
        /* One frame per call keeps BLE-context work bounded. Recovery (NACK)
         * outranks credit refresh (ACK): an ACK_WINDOW is skip-ahead-only on the
         * node, so it cannot substitute for a pending gap retransmit request. */
        if (nack_pending_) return send_nack(now_ms);
        if (ack_pending_) return send_ack_window(now_ms);
        return true;
    }

    /* Re-advertise the last window if the node has gone quiet. The caller owns
     * the liveness decision because only it knows whether chunks are flowing. */
    bool rearm_ack_window(uint32_t now_ms, uint32_t idle_ms = kAckKeepaliveMs)
    {
        if (!active() || !ack_valid_ || ack_pending_ || pending_length_ != 0U) {
            return false;
        }
        if (static_cast<uint32_t>(now_ms - ack_last_sent_ms_) < idle_ms) {
            return false;
        }
        ack_pending_ = true;
        return true;
    }

    bool active() const
    {
        return node_id_ >= 1U && node_id_ <= 4U && session_id_ != 0U;
    }
    bool pending() const { return pending_length_ != 0U || nack_pending_ || ack_pending_; }
    uint8_t node_id() const { return node_id_; }
    uint32_t session_id() const { return session_id_; }
    RecordReliableType pending_type() const { return pending_type_; }
    uint32_t attempt_count() const { return attempt_count_; }
    const uint8_t *pending_frame() const { return pending_frame_; }
    uint16_t pending_length() const { return pending_length_; }
    uint32_t ack_next_chunk() const { return ack_next_chunk_; }
    bool ack_valid() const { return ack_valid_; }

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

    static bool chunk_offset_valid_(uint32_t chunk_index, uint16_t chunk_size)
    {
        const uint64_t offset = static_cast<uint64_t>(chunk_index) * chunk_size;
        return offset <= 0xFFFFFFFFULL;
    }

    static uint32_t chunk_byte_offset_(uint32_t chunk_index, uint16_t chunk_size)
    {
        const uint64_t offset = static_cast<uint64_t>(chunk_index) * chunk_size;
        return offset > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(offset);
    }

    uint16_t build_frame(uint8_t *out, RecordReliableType type, uint32_t chunk_index,
            uint32_t byte_offset, const uint8_t *body, uint16_t body_length) const
    {
        if (out == nullptr || body == nullptr ||
                sizeof(RecordReliableFrameHeader) + body_length > kMaxFrameBytes) {
            return 0U;
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
        memcpy(out, &header, sizeof(header));
        memcpy(out + sizeof(header), body, body_length);
        return static_cast<uint16_t>(sizeof(header) + body_length);
    }

    bool queue_frame(RecordReliableType type, uint32_t chunk_index,
            uint32_t byte_offset, const uint8_t *body, uint16_t body_length,
            uint8_t priority)
    {
        if (!active()) return false;
        if (pending_length_ != 0U && priority < pending_priority_) {
            return false;
        }
        const uint16_t length = build_frame(pending_frame_, type, chunk_index,
                byte_offset, body, body_length);
        if (length == 0U) return false;
        pending_length_ = length;
        pending_priority_ = priority;
        pending_type_ = type;
        last_attempt_ms_ = 0U;
        attempt_count_ = 0U;
        return true;
    }

    bool send_nack(uint32_t now_ms)
    {
        if (nack_attempt_count_ != 0U &&
                static_cast<uint32_t>(now_ms - nack_last_attempt_ms_) < kRetryIntervalMs) {
            return false;
        }
        nack_last_attempt_ms_ = now_ms;
        if (nack_attempt_count_ < 0xFFFFFFFFU) ++nack_attempt_count_;
        RecordReliableNackRangePayload body{};
        body.source_id = node_id_;
        body.session_id = session_id_;
        body.first_chunk_index = nack_first_chunk_;
        body.chunk_count = nack_count_;
        body.flags = nack_flags_;
        uint8_t frame[kMaxFrameBytes];
        const uint16_t length = build_frame(frame, RecordReliableType::NackRange,
                nack_first_chunk_, chunk_byte_offset_(nack_first_chunk_, chunk_size_),
                reinterpret_cast<const uint8_t*>(&body), sizeof(body));
        if (length == 0U || !transmit(frame, length)) return false;
        nack_pending_ = false;
        nack_attempt_count_ = 0U;
        return true;
    }

    bool send_ack_window(uint32_t now_ms)
    {
        RecordReliableAckWindowPayload body{};
        body.source_id = node_id_;
        body.session_id = session_id_;
        body.next_chunk_index = ack_next_chunk_;
        body.credit = ack_credit_;
        body.reserved0 = 0U;
        body.flags = 0U;
        uint8_t frame[kMaxFrameBytes];
        const uint16_t length = build_frame(frame, RecordReliableType::AckWindow,
                ack_next_chunk_, chunk_byte_offset_(ack_next_chunk_, chunk_size_),
                reinterpret_cast<const uint8_t*>(&body), sizeof(body));
        if (length == 0U || !transmit(frame, length)) return false;
        ack_pending_ = false;
        ack_valid_ = true;
        ack_last_sent_ms_ = now_ms;
        return true;
    }

    bool transmit(const uint8_t *frame, uint16_t length)
    {
        if (frame == nullptr || length == 0U) return false;
        if (send_fn_ != nullptr) {
            return send_fn_(context_, node_id_, frame, length);
        }
        if (exo_hub_ble_write == nullptr || length > 0xFFU) {
            return false;
        }
        return exo_hub_ble_write(frame, static_cast<uint8_t>(length)) != 0U;
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
    bool ack_pending_ = false;
    bool ack_valid_ = false;
    uint32_t ack_next_chunk_ = 0U;
    uint8_t ack_credit_ = kDefaultCredit;
    uint32_t ack_last_sent_ms_ = 0U;
    bool nack_pending_ = false;
    uint32_t nack_first_chunk_ = 0U;
    uint16_t nack_count_ = 1U;
    uint16_t nack_flags_ = 0U;
    uint32_t nack_last_attempt_ms_ = 0U;
    uint32_t nack_attempt_count_ = 0U;
};

} // namespace exo

#endif
