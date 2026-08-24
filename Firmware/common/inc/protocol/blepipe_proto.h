#ifndef BLEPIPE_PROTO_H
#define BLEPIPE_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define BLEPIPE_PROTO_VER          1U
#define BLEPIPE_MAX_NOTIFY_PAYLOAD 244U
#define BLEPIPE_HDR_LEN            20U
#define BLEPIPE_CRC_LEN            2U
#define BLEPIPE_MAX_APP_PAYLOAD    (BLEPIPE_MAX_NOTIFY_PAYLOAD - BLEPIPE_HDR_LEN - BLEPIPE_CRC_LEN)

#define BLEPIPE_ID_HUB             0x0001U
#define BLEPIPE_ID_LEAF_1          0x0101U
#define BLEPIPE_ID_LEAF_2          0x0102U
#define BLEPIPE_ID_LEAF_3          0x0103U
#define BLEPIPE_ID_LEAF_4          0x0104U
#define BLEPIPE_ID_BROADCAST       0xFFFFU

typedef enum {
  BLEPIPE_LANE_DATA_TX = 0,
  BLEPIPE_LANE_CONTROL_RX,
  BLEPIPE_LANE_CONTROL_TX,
  BLEPIPE_LANE_STATUS_TX,
  BLEPIPE_LANE_CONFIG_RW
} blepipe_lane_t;

typedef enum {
  BLEPIPE_MSG_LEAF_SAMPLE      = 0x01,
  BLEPIPE_MSG_HUB_AGGREGATE   = 0x02,
  BLEPIPE_MSG_RAW_FORWARD     = 0x03,

  BLEPIPE_MSG_COMMAND         = 0x10,
  BLEPIPE_MSG_COMMAND_RESP    = 0x11,
  BLEPIPE_MSG_ACK             = 0x12,
  BLEPIPE_MSG_NACK            = 0x13,

  BLEPIPE_MSG_STATUS          = 0x20,
  BLEPIPE_MSG_TOPOLOGY        = 0x21,
  BLEPIPE_MSG_LINK_STATS      = 0x22,
  BLEPIPE_MSG_EVENT           = 0x23,
  BLEPIPE_MSG_LOG             = 0x24,
  BLEPIPE_MSG_ERROR           = 0x25,

  BLEPIPE_MSG_TIME_SYNC       = 0x30,
  BLEPIPE_MSG_CONFIG_SET      = 0x31,
  BLEPIPE_MSG_STREAM_CONTROL  = 0x32,

  BLEPIPE_MSG_CONFIG_READ     = 0x40,
  BLEPIPE_MSG_CONFIG_WRITE    = 0x41,
  BLEPIPE_MSG_ROUTING_TABLE   = 0x42,
  BLEPIPE_MSG_DEVICE_INFO     = 0x43,
  BLEPIPE_MSG_STREAM_PROFILE  = 0x44
} blepipe_msg_type_t;

typedef enum {
  BLEPIPE_STATUS_OK = 0,
  BLEPIPE_STATUS_BAD_ARG,
  BLEPIPE_STATUS_TOO_SHORT,
  BLEPIPE_STATUS_TOO_LONG,
  BLEPIPE_STATUS_BAD_VERSION,
  BLEPIPE_STATUS_BAD_LENGTH,
  BLEPIPE_STATUS_BAD_CRC,
  BLEPIPE_STATUS_BAD_LANE
} blepipe_status_t;

#define BLEPIPE_STATUS_KIND_NODE_RECORD_READY       0x01U
#define BLEPIPE_STATUS_KIND_RECORD_START_HEARTBEAT 0x02U

#define BLEPIPE_RECORD_START_PHASE_WAIT_APP_READY     0x01U
#define BLEPIPE_RECORD_START_PHASE_RESET_COOLDOWN     0x02U
#define BLEPIPE_RECORD_START_PHASE_PREPARE_SENT       0x03U
#define BLEPIPE_RECORD_START_PHASE_WAIT_PREPARE_ACK   0x04U
#define BLEPIPE_RECORD_START_PHASE_APP_READY_ACQUIRED 0x05U
#define BLEPIPE_RECORD_START_PHASE_COMMIT_START       0x06U
#define BLEPIPE_RECORD_START_PHASE_NODE_RESET         0x20U
#define BLEPIPE_RECORD_START_PHASE_NODE_PREPARE       0x21U
#define BLEPIPE_RECORD_START_PHASE_NODE_COMMIT        0x22U
#define BLEPIPE_RECORD_START_PHASE_NODE_ABORT         0x23U
#define BLEPIPE_RECORD_START_PHASE_NODE_READY_WAIT    0x24U

typedef struct __attribute__((packed)) {
  uint8_t  proto_ver;
  uint8_t  msg_type;
  uint8_t  flags;
  uint8_t  hop_count;
  uint16_t src_id;
  uint16_t dst_id;
  uint32_t seq;
  uint32_t timestamp_ms;
  uint16_t payload_len;
} blepipe_hdr_t;

typedef struct __attribute__((packed)) {
  uint8_t  status_kind;
  uint8_t  node_id;
  uint8_t  record_ready;
  uint8_t  recorder_state;
  uint32_t session_id;
  uint32_t maximum_duration_ms;
} blepipe_node_record_ready_status_t;

typedef struct __attribute__((packed)) {
  uint8_t  status_kind;
  uint8_t  phase;
  uint8_t  in_progress;
  uint8_t  reserved;
  uint16_t source_id;
  uint32_t session_id;
  uint32_t extend_timeout_ms;
} blepipe_record_start_heartbeat_status_t;

uint16_t blepipe_crc16_ccitt(const uint8_t *data, size_t len);
blepipe_status_t blepipe_encode(uint8_t *dst, size_t dst_len, const blepipe_hdr_t *hdr,
                                const uint8_t *payload, uint16_t payload_len,
                                size_t *encoded_len);
blepipe_status_t blepipe_decode(const uint8_t *packet, size_t packet_len, blepipe_hdr_t *hdr,
                                const uint8_t **payload, uint16_t *payload_len);
uint8_t blepipe_msg_allowed_on_lane(blepipe_lane_t lane, uint8_t msg_type);

#ifdef __cplusplus
}
#endif

#endif /* BLEPIPE_PROTO_H */
