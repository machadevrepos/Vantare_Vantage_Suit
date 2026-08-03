#ifndef RS485_FRAME_PROTOCOL_H_
#define RS485_FRAME_PROTOCOL_H_

#include <stdint.h>
#include <string.h>

namespace exo::rs485_record {

static constexpr uint8_t kFrameSof0 = 0xA5U;
static constexpr uint8_t kFrameSof1 = 0x5AU;
static constexpr uint8_t kFrameVersion = 0x01U;
static constexpr uint8_t kTransferProtocolVersion = 0x02U;
static constexpr uint8_t kTransferProtocolVersionBurst = 0x03U;
static constexpr uint8_t kBroadcastNode = 0xFFU;
static constexpr uint16_t kMaxPayload = 560U;
static constexpr uint16_t kMaxFrameBytes = static_cast<uint16_t>(11U + kMaxPayload);

enum class MsgType : uint8_t {
    Ack = 0x01,
    Nack = 0x02,
    DiscoverReq = 0x03,
    DiscoverResp = 0x04,
    SetNodeIdReq = 0x05,
    GetNodeIdReq = 0x06,
    NodeIdResp = 0x07,
    StartRecord = 0x10,
    RecordStatus = 0x11,
    DataChunkReq = 0x12,
    DataChunk = 0x13,
    TransferCompleteAck = 0x14,
    Error = 0x15,
    ResetState = 0x16,
    GetManifest = 0x17,
    Manifest = 0x18,
    RequestChunk = 0x19,
    PauseTransfer = 0x1A,
    ResumeTransfer = 0x1B,
    CancelTransfer = 0x1C,
    CommitDone = 0x1D,
    DataRangeReq = 0x1E,
    DataAckBitmap = 0x1F
};

#pragma pack(push, 1)
struct StartRecordWire {
    uint8_t command;
    uint32_t session_id;
    uint64_t start_timestamp_us;
    uint32_t requested_duration_ms;
};

struct RecordDoneWire {
    uint8_t command;
    uint16_t node_id;
    uint32_t session_id;
    uint32_t actual_duration_ms;
    uint32_t total_size;
    uint32_t payload_crc32;
};

struct DataChunkReqWire {
    uint32_t session_id;
    uint32_t offset;
    uint16_t size;
    uint8_t req_flags;
    uint16_t expected_chunk_seq;
};

struct ManifestReqWire {
    uint32_t session_id;
};

struct ManifestWire {
    uint32_t session_id;
    uint16_t source_id;
    uint32_t total_size;
    uint16_t chunk_size;
    uint16_t total_chunks;
    uint32_t payload_crc32;
};

struct RequestChunkWire {
    uint32_t session_id;
    uint32_t chunk_index;
    uint16_t size;
    uint8_t req_flags;
};

struct DataRangeReqWire {
    uint32_t session_id;
    uint32_t start_chunk_index;
    uint16_t chunk_size;
    uint8_t chunk_count;
    uint8_t req_flags;
    uint16_t expected_chunk_seq;
};

struct DataAckBitmapWire {
    uint32_t session_id;
    uint32_t base_chunk_index;
    uint16_t bitmap; /* bit0 => base chunk received */
    uint8_t credit;
    uint8_t flags;
};

struct DataChunkHdrWire {
    uint32_t session_id;
    uint32_t offset;
    uint16_t size;
    uint16_t chunk_crc16;
    uint16_t chunk_seq;
    uint8_t origin_flags;
};

enum ChunkReqFlags : uint8_t {
    kChunkReqNormal = 0x00U,
    kChunkReqRecovery = 0x01U,
    kChunkReqFastRetransmit = 0x02U
};

enum ChunkOriginFlags : uint8_t {
    kChunkOriginFreshRead = 0x00U,
    kChunkOriginCacheHit = 0x01U
};

struct TransferCompleteAckWire {
    uint32_t session_id;
    uint32_t payload_crc32;
};

struct SetNodeIdWire {
    uint8_t new_id;
};

struct NodeIdRespWire {
    uint8_t node_id;
    uint8_t status; /* 0=ok, 1=invalid */
};
#pragma pack(pop)

struct Frame {
    MsgType msg_type = MsgType::Error;
    uint8_t sequence = 0U;
    uint8_t src_id = 0U;
    uint8_t dst_id = 0U;
    uint16_t payload_len = 0U;
    uint8_t payload[kMaxPayload] = {0U};
};

inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

inline bool encode_frame(const Frame &frame, uint8_t *out, uint16_t out_cap, uint16_t &out_len) {
    if (frame.payload_len > kMaxPayload) {
        return false;
    }
    const uint16_t total_len = static_cast<uint16_t>(11U + frame.payload_len);
    if (out == nullptr || out_cap < total_len) {
        return false;
    }

    out[0] = kFrameSof0;
    out[1] = kFrameSof1;
    out[2] = kFrameVersion;
    out[3] = static_cast<uint8_t>(frame.msg_type);
    out[4] = frame.sequence;
    out[5] = frame.src_id;
    out[6] = frame.dst_id;
    out[7] = static_cast<uint8_t>(frame.payload_len & 0xFFU);
    out[8] = static_cast<uint8_t>((frame.payload_len >> 8U) & 0xFFU);
    if (frame.payload_len > 0U) {
        memcpy(&out[9], frame.payload, frame.payload_len);
    }
    const uint16_t crc = crc16_ccitt(&out[2], static_cast<uint16_t>(7U + frame.payload_len));
    out[static_cast<uint16_t>(9U + frame.payload_len)] = static_cast<uint8_t>(crc & 0xFFU);
    out[static_cast<uint16_t>(10U + frame.payload_len)] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
    out_len = total_len;
    return true;
}

class FrameParser {
public:
    bool feed(uint8_t byte, Frame &out_frame) {
        switch (state_) {
            case State::WaitSof0:
                if (byte == kFrameSof0) {
                    raw_[0] = byte;
                    raw_len_ = 1U;
                    state_ = State::WaitSof1;
                }
                return false;
            case State::WaitSof1:
                if (byte == kFrameSof1) {
                    raw_[1] = byte;
                    raw_len_ = 2U;
                    state_ = State::ReadHeader;
                } else {
                    reset();
                }
                return false;
            case State::ReadHeader:
                raw_[raw_len_++] = byte;
                if (raw_len_ == 9U) {
                    if (raw_[2] != kFrameVersion) {
                        reset();
                        return false;
                    }
                    payload_len_ = static_cast<uint16_t>(raw_[7]) |
                                   static_cast<uint16_t>(raw_[8] << 8U);
                    if (payload_len_ > kMaxPayload) {
                        reset();
                        return false;
                    }
                    state_ = State::ReadPayloadAndCrc;
                }
                return false;
            case State::ReadPayloadAndCrc:
                raw_[raw_len_++] = byte;
                if (raw_len_ == static_cast<uint16_t>(11U + payload_len_)) {
                    const uint16_t frame_crc = static_cast<uint16_t>(raw_[9U + payload_len_]) |
                                               static_cast<uint16_t>(raw_[10U + payload_len_] << 8U);
                    const uint16_t calc_crc = crc16_ccitt(&raw_[2], static_cast<uint16_t>(7U + payload_len_));
                    if (frame_crc == calc_crc) {
                        out_frame.msg_type = static_cast<MsgType>(raw_[3]);
                        out_frame.sequence = raw_[4];
                        out_frame.src_id = raw_[5];
                        out_frame.dst_id = raw_[6];
                        out_frame.payload_len = payload_len_;
                        if (payload_len_ > 0U) {
                            memcpy(out_frame.payload, &raw_[9], payload_len_);
                        }
                        reset();
                        return true;
                    }
                    reset();
                }
                return false;
        }
        reset();
        return false;
    }

private:
    enum class State : uint8_t { WaitSof0, WaitSof1, ReadHeader, ReadPayloadAndCrc };

    void reset() {
        state_ = State::WaitSof0;
        raw_len_ = 0U;
        payload_len_ = 0U;
    }

    State state_ = State::WaitSof0;
    uint8_t raw_[kMaxFrameBytes] = {0U};
    uint16_t raw_len_ = 0U;
    uint16_t payload_len_ = 0U;
};

} // namespace exo::rs485_record

#endif
