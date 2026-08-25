#ifndef BLE_RECORD_PROTOCOL_H_
#define BLE_RECORD_PROTOCOL_H_

#include <stdint.h>

namespace exo {

enum class RecordCommand : uint8_t {
    StartRecord = 0x01,
    RecordDone = 0x02,
    SessionChunk = 0x05,
    ChunkAck = 0x06,
    SessionCompleteAck = 0x07,
    LaneFrameV3 = 0x09,
    ReliableFrame = 0x0A,
    PrepareRecord = 0x0B,
    CommitPreparedRecord = 0x0C,
    AbortPreparedRecord = 0x0D,
    StopRecord = 0x0E,
    StartSession = 0x0F,
    RetrySource = 0x10
};

static constexpr uint8_t kRecordReliableProtoVersion = 6U;
static constexpr uint16_t kRecordReliableMagic = 0x5845U; /* "EX" little endian */
/* Chunk-size budget (all on the same BLE notification):
 *   notification payload  = MTU(247) - 3            = 244 B
 *   blepipe envelope      = BLEPIPE_HDR_LEN(20) + CRC(2) = 22 B
 *   reliable frame header = sizeof(RecordReliableFrameHeader) = 25 B
 *   => max chunk payload  = 244 - 22 - 25           = 197 B
 * 192 stays 5 B under the ceiling for MTU-negotiation margin. A value above
 * 197 makes the wrapped frame exceed 244 B and the stack SILENTLY DROPS every
 * chunk (zero accepted -> SessionStall at +30 s). 200 and 220 both did this.
 * See the static_assert in Node/Core/Src/main.c that locks this invariant. */
static constexpr uint16_t kRecordReliableDefaultChunkSize = 192U;
/* Matches the credit the Master actually grants per window (8). Every sender
 * and the browser presets must agree on this default or tuning gets confusing. */
/* Chunks the node may keep in flight before a credit refresh. Sized to cover
 * the ACK round-trip at the fast (15 ms) upload interval so the node never
 * stalls waiting for a window; 24 is the controller-sanitized maximum. */
static constexpr uint8_t kRecordReliableDefaultCredit = 24U;

enum class RecordSourceId : uint16_t {
    Master = 0U,
    Node1 = 1U,
    Node2 = 2U,
    Node3 = 3U,
    Node4 = 4U
};

enum class RecordReliableType : uint8_t {
    Manifest = 0x01,
    ManifestAck = 0x02,
    AckWindow = 0x03,
    NackRange = 0x04,
    Chunk = 0x05,
    Pause = 0x06,
    Resume = 0x07,
    Cancel = 0x08,
    VerifyOk = 0x09,
    VerifyFail = 0x0A,
    BusyNotOwner = 0x0B,
    SourceWaiting = 0x0C,
    CommitDone = 0x0D
};

enum RecordReliableFlags : uint16_t {
    kRecordFlagFinalChunk = 0x0001U,
    kRecordFlagRetransmit = 0x0002U,
    kRecordFlagStorageMissing = 0x0004U,
    kRecordFlagCrcMismatch = 0x0008U
};

constexpr uint16_t record_reliable_chunk_flags(bool final_chunk, bool retransmit)
{
    return static_cast<uint16_t>(
            (final_chunk ? static_cast<uint16_t>(kRecordFlagFinalChunk) : 0U) |
            (retransmit ? static_cast<uint16_t>(kRecordFlagRetransmit) : 0U));
}

enum class RecordLaneId : uint8_t {
    Control = 0x00,
    MasterData = 0x01,
    NodeData = 0x02,
    Retransmit = 0x03
};

enum class RecordLaneMsgType : uint8_t {
    RecordDone = 0x01,
    SessionChunk = 0x02,
    SessionBarrier = 0x03
};

#pragma pack(push, 1)
struct StartRecordMessage {
    RecordCommand command;
    uint32_t session_id;
    uint64_t start_timestamp_us;
    uint32_t requested_duration_ms;
};

struct StopRecordMessage {
    RecordCommand command;
    uint32_t session_id;
};

/* Browser -> Master: re-pull one source that was written off after a stall.
 * The node still holds its flash copy (abandoned sources are never erased), so
 * the Master can re-stage the file and drive a fresh reliable window. */
struct RetrySourceMessage {
    RecordCommand command;
    uint8_t node_id;
};

struct StartSessionMessage {
    RecordCommand command;
    uint32_t session_id;
    uint64_t start_timestamp_us;
    uint32_t safety_duration_ms;
    uint8_t selected_node_mask;
    uint8_t stream_interval_ms;
};

struct RecordDoneMessage {
    RecordCommand command;
    uint16_t node_id;
    uint32_t session_id;
    uint32_t actual_duration_ms;
    uint32_t total_size;
    uint32_t payload_crc32;
};

struct SessionChunkHeader {
    RecordCommand command;
    uint32_t session_id;
    uint32_t offset;
    uint16_t payload_size;
    uint16_t sequence;
};

struct ChunkAckMessage {
    RecordCommand command;
    uint32_t session_id;
    uint16_t sequence;
    uint32_t next_offset;
};

struct ChunkAckRangeMessage {
    RecordCommand command;
    uint32_t session_id;
    uint16_t flags;       /* bit0: missing range valid */
    uint32_t next_offset; /* highest contiguous offset received by host */
    uint32_t missing_start;
    uint16_t missing_len;
};

struct ChunkAckV3Message {
    RecordCommand command;
    uint8_t proto_version;
    uint16_t source_id;
    uint16_t flags; /* bit0: missing range valid */
    uint32_t session_id;
    uint32_t next_offset; /* highest contiguous offset received by host */
    uint32_t missing_start;
    uint16_t missing_len;
};

/* Compact ACK format (proto_version=4), no missing range payload */
struct ChunkAckCompactMessage {
    RecordCommand command;
    uint8_t proto_version; /* currently 4 */
    uint16_t flags;        /* bit0: recovery/request retransmit */
    uint32_t session_id;
    uint32_t next_offset;  /* highest contiguous offset received by host */
};

/* Compact ACK with explicit source id */
struct ChunkAckCompactSourceMessage {
    RecordCommand command;
    uint8_t proto_version; /* currently 4 */
    uint16_t flags;        /* bit0: recovery/request retransmit */
    uint32_t session_id;
    uint32_t next_offset;  /* highest contiguous offset received by host */
    uint16_t source_id;
};

struct RecordLaneFrameV3Header {
    RecordCommand command;   /* LaneFrameV3 */
    uint8_t proto_version;   /* currently 3 */
    uint32_t session_id;
    uint16_t source_id;
    uint8_t lane_id;
    uint8_t msg_type;
    uint16_t sequence;
    uint32_t offset;
    uint16_t payload_len;
    uint16_t flags;
};

struct RecordReliableFrameHeader {
    RecordCommand command;       /* ReliableFrame */
    uint8_t proto_version;       /* kRecordReliableProtoVersion */
    uint16_t magic;              /* kRecordReliableMagic */
    uint8_t frame_type;          /* RecordReliableType */
    uint16_t source_id;
    uint32_t session_id;
    uint32_t chunk_index;
    uint32_t byte_offset;
    uint16_t payload_len;
    uint16_t payload_crc16;
    uint16_t flags;
};

struct RecordReliableManifestPayload {
    uint8_t protocol_version;
    uint8_t reserved0;
    uint16_t source_id;
    uint32_t session_id;
    uint32_t file_size;
    uint16_t chunk_size;
    uint16_t total_chunks;
    uint32_t file_crc32;
    uint32_t duration_ms;
    uint32_t timestamp_lo;
    uint32_t flags;
};

struct RecordReliableManifestAckPayload {
    uint16_t source_id;
    uint32_t session_id;
    uint16_t accepted_chunk_size;
    uint8_t credit;
    uint8_t status;
};

struct RecordReliableAckWindowPayload {
    uint16_t source_id;
    uint32_t session_id;
    uint32_t next_chunk_index;
    uint8_t credit;
    uint8_t reserved0;
    uint16_t flags;
};

struct RecordReliableNackRangePayload {
    uint16_t source_id;
    uint32_t session_id;
    uint32_t first_chunk_index;
    uint16_t chunk_count;
    uint16_t flags;
};

struct RecordReliableVerifyPayload {
    uint16_t source_id;
    uint32_t session_id;
    uint32_t file_crc32;
    uint32_t first_bad_chunk;
    uint16_t flags;
};

struct SessionCompleteAckMessage {
    RecordCommand command;
    uint32_t session_id;
    uint32_t payload_crc32;
};
#pragma pack(pop)

} // namespace exo

#endif
