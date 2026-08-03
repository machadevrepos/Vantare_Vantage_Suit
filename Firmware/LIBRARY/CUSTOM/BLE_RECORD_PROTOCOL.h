#ifndef BLE_RECORD_PROTOCOL_H_
#define BLE_RECORD_PROTOCOL_H_

#include <stdint.h>

namespace exo {

enum class RecordCommand : uint8_t {
    StartRecord = 0x01,
    RecordDone = 0x02,
    ListSessions = 0x03,
    FetchSession = 0x04,
    SessionChunk = 0x05,
    ChunkAck = 0x06,
    SessionCompleteAck = 0x07,
    EraseSession = 0x08,
    LaneFrameV3 = 0x09,
    ReliableFrame = 0x0AU,
    PrepareRecord = 0x0BU,
    CommitPreparedRecord = 0x0CU,
    AbortPreparedRecord = 0x0DU,
    StopRecord = 0x0EU,
    StartSession = 0x0FU
};

static constexpr uint8_t kRecordReliableProtoVersion = 6U;
static constexpr uint16_t kRecordReliableMagic = 0x5845U; /* "EX" little endian */
static constexpr uint16_t kRecordReliableDefaultChunkSize = 180U;
static constexpr uint8_t kRecordReliableDefaultCredit = 16U;

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
struct StartRecordMessage { RecordCommand command; uint32_t session_id; uint64_t start_timestamp_us; uint32_t requested_duration_ms; };
struct StopRecordMessage { RecordCommand command; uint32_t session_id; };
struct StartSessionMessage { RecordCommand command; uint32_t session_id; uint64_t start_timestamp_us; uint32_t safety_duration_ms; uint8_t selected_node_mask; uint8_t stream_interval_ms; };
struct RecordDoneMessage { RecordCommand command; uint16_t node_id; uint32_t session_id; uint32_t actual_duration_ms; uint32_t total_size; uint32_t payload_crc32; };
struct FetchSessionMessage { RecordCommand command; uint32_t session_id; uint32_t resume_offset; };
struct SessionChunkHeader { RecordCommand command; uint32_t session_id; uint32_t offset; uint16_t payload_size; uint16_t sequence; };
struct ChunkAckMessage { RecordCommand command; uint32_t session_id; uint16_t sequence; uint32_t next_offset; };
struct ChunkAckRangeMessage { RecordCommand command; uint32_t session_id; uint16_t flags; uint32_t next_offset; uint32_t missing_start; uint16_t missing_len; };
struct ChunkAckV3Message { RecordCommand command; uint8_t proto_version; uint16_t source_id; uint16_t flags; uint32_t session_id; uint32_t next_offset; uint32_t missing_start; uint16_t missing_len; };
struct ChunkAckCompactMessage { RecordCommand command; uint8_t proto_version; uint16_t flags; uint32_t session_id; uint32_t next_offset; };
struct ChunkAckCompactSourceMessage { RecordCommand command; uint8_t proto_version; uint16_t flags; uint32_t session_id; uint32_t next_offset; uint16_t source_id; };
struct RecordLaneFrameV3Header { RecordCommand command; uint8_t proto_version; uint32_t session_id; uint16_t source_id; uint8_t lane_id; uint8_t msg_type; uint16_t sequence; uint32_t offset; uint16_t payload_len; uint16_t flags; };
struct RecordReliableFrameHeader { RecordCommand command; uint8_t proto_version; uint16_t magic; uint8_t frame_type; uint16_t source_id; uint32_t session_id; uint32_t chunk_index; uint32_t byte_offset; uint16_t payload_len; uint16_t payload_crc16; uint16_t flags; };
struct RecordReliableManifestPayload { uint8_t protocol_version; uint8_t reserved0; uint16_t source_id; uint32_t session_id; uint32_t file_size; uint16_t chunk_size; uint16_t total_chunks; uint32_t file_crc32; uint32_t duration_ms; uint32_t timestamp_lo; uint32_t flags; };
struct RecordReliableManifestAckPayload { uint16_t source_id; uint32_t session_id; uint16_t accepted_chunk_size; uint8_t credit; uint8_t status; };
struct RecordReliableAckWindowPayload { uint16_t source_id; uint32_t session_id; uint32_t next_chunk_index; uint8_t credit; uint8_t reserved0; uint16_t flags; };
struct RecordReliableNackRangePayload { uint16_t source_id; uint32_t session_id; uint32_t first_chunk_index; uint16_t chunk_count; uint16_t flags; };
struct RecordReliableVerifyPayload { uint16_t source_id; uint32_t session_id; uint32_t file_crc32; uint32_t first_bad_chunk; uint16_t flags; };
struct SessionCompleteAckMessage { RecordCommand command; uint32_t session_id; uint32_t payload_crc32; };
struct EraseSessionMessage { RecordCommand command; uint32_t session_id; };
#pragma pack(pop)

} // namespace exo

#endif
