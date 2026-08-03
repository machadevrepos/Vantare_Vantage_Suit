#include "blepipe_proto.h"

#include <string.h>

static void put_le16(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)(value >> 8);
}

static void put_le32(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)(value >> 24);
}

static uint16_t get_le16(const uint8_t *src)
{
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t get_le32(const uint8_t *src)
{
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static void write_hdr(uint8_t *dst, const blepipe_hdr_t *hdr, uint16_t payload_len)
{
  dst[0] = hdr->proto_ver;
  dst[1] = hdr->msg_type;
  dst[2] = hdr->flags;
  dst[3] = hdr->hop_count;
  put_le16(&dst[4], hdr->src_id);
  put_le16(&dst[6], hdr->dst_id);
  put_le32(&dst[8], hdr->seq);
  put_le32(&dst[12], hdr->timestamp_ms);
  put_le16(&dst[16], payload_len);
  put_le16(&dst[18], 0U);
}

static void read_hdr(const uint8_t *src, blepipe_hdr_t *hdr)
{
  hdr->proto_ver = src[0];
  hdr->msg_type = src[1];
  hdr->flags = src[2];
  hdr->hop_count = src[3];
  hdr->src_id = get_le16(&src[4]);
  hdr->dst_id = get_le16(&src[6]);
  hdr->seq = get_le32(&src[8]);
  hdr->timestamp_ms = get_le32(&src[12]);
  hdr->payload_len = get_le16(&src[16]);
}

uint16_t blepipe_crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;

  if ((data == NULL) && (len > 0U)) {
    return 0U;
  }

  while (len-- > 0U) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

blepipe_status_t blepipe_encode(uint8_t *dst, size_t dst_len, const blepipe_hdr_t *hdr,
                                const uint8_t *payload, uint16_t payload_len,
                                size_t *encoded_len)
{
  size_t total_len = BLEPIPE_HDR_LEN + (size_t)payload_len + BLEPIPE_CRC_LEN;
  uint16_t crc;

  if ((dst == NULL) || (hdr == NULL) || (encoded_len == NULL)) {
    return BLEPIPE_STATUS_BAD_ARG;
  }
  if ((payload == NULL) && (payload_len > 0U)) {
    return BLEPIPE_STATUS_BAD_ARG;
  }
  if (payload_len > BLEPIPE_MAX_APP_PAYLOAD) {
    return BLEPIPE_STATUS_TOO_LONG;
  }
  if (dst_len < total_len) {
    return BLEPIPE_STATUS_TOO_SHORT;
  }

  write_hdr(dst, hdr, payload_len);
  if (payload_len > 0U) {
    (void)memcpy(&dst[BLEPIPE_HDR_LEN], payload, payload_len);
  }

  crc = blepipe_crc16_ccitt(dst, BLEPIPE_HDR_LEN + (size_t)payload_len);
  put_le16(&dst[BLEPIPE_HDR_LEN + payload_len], crc);

  *encoded_len = total_len;
  return BLEPIPE_STATUS_OK;
}

blepipe_status_t blepipe_decode(const uint8_t *packet, size_t packet_len, blepipe_hdr_t *hdr,
                                const uint8_t **payload, uint16_t *payload_len)
{
  uint16_t declared_len;
  uint16_t expected_crc;
  uint16_t actual_crc;
  size_t expected_len;

  if ((packet == NULL) || (hdr == NULL) || (payload == NULL) || (payload_len == NULL)) {
    return BLEPIPE_STATUS_BAD_ARG;
  }
  if (packet_len < (BLEPIPE_HDR_LEN + BLEPIPE_CRC_LEN)) {
    return BLEPIPE_STATUS_TOO_SHORT;
  }
  if (packet_len > BLEPIPE_MAX_NOTIFY_PAYLOAD) {
    return BLEPIPE_STATUS_TOO_LONG;
  }

  read_hdr(packet, hdr);
  if (hdr->proto_ver != BLEPIPE_PROTO_VER) {
    return BLEPIPE_STATUS_BAD_VERSION;
  }

  declared_len = hdr->payload_len;
  if (declared_len > BLEPIPE_MAX_APP_PAYLOAD) {
    return BLEPIPE_STATUS_TOO_LONG;
  }

  expected_len = BLEPIPE_HDR_LEN + (size_t)declared_len + BLEPIPE_CRC_LEN;
  if (packet_len != expected_len) {
    return BLEPIPE_STATUS_BAD_LENGTH;
  }

  expected_crc = get_le16(&packet[BLEPIPE_HDR_LEN + declared_len]);
  actual_crc = blepipe_crc16_ccitt(packet, BLEPIPE_HDR_LEN + (size_t)declared_len);
  if (actual_crc != expected_crc) {
    return BLEPIPE_STATUS_BAD_CRC;
  }

  *payload = &packet[BLEPIPE_HDR_LEN];
  *payload_len = declared_len;
  return BLEPIPE_STATUS_OK;
}

uint8_t blepipe_msg_allowed_on_lane(blepipe_lane_t lane, uint8_t msg_type)
{
  switch (lane) {
    case BLEPIPE_LANE_DATA_TX:
      return (uint8_t)((msg_type == BLEPIPE_MSG_LEAF_SAMPLE) ||
                       (msg_type == BLEPIPE_MSG_HUB_AGGREGATE) ||
                       (msg_type == BLEPIPE_MSG_RAW_FORWARD));

    case BLEPIPE_LANE_CONTROL_RX:
      return (uint8_t)((msg_type == BLEPIPE_MSG_COMMAND) ||
                       (msg_type == BLEPIPE_MSG_TIME_SYNC) ||
                       (msg_type == BLEPIPE_MSG_CONFIG_SET) ||
                       (msg_type == BLEPIPE_MSG_STREAM_CONTROL));

    case BLEPIPE_LANE_CONTROL_TX:
      return (uint8_t)((msg_type == BLEPIPE_MSG_COMMAND_RESP) ||
                       (msg_type == BLEPIPE_MSG_ACK) ||
                       (msg_type == BLEPIPE_MSG_NACK));

    case BLEPIPE_LANE_STATUS_TX:
      return (uint8_t)((msg_type == BLEPIPE_MSG_STATUS) ||
                       (msg_type == BLEPIPE_MSG_TOPOLOGY) ||
                       (msg_type == BLEPIPE_MSG_LINK_STATS) ||
                       (msg_type == BLEPIPE_MSG_EVENT) ||
                       (msg_type == BLEPIPE_MSG_LOG) ||
                       (msg_type == BLEPIPE_MSG_ERROR));

    case BLEPIPE_LANE_CONFIG_RW:
      return (uint8_t)((msg_type == BLEPIPE_MSG_CONFIG_READ) ||
                       (msg_type == BLEPIPE_MSG_CONFIG_WRITE) ||
                       (msg_type == BLEPIPE_MSG_ROUTING_TABLE) ||
                       (msg_type == BLEPIPE_MSG_DEVICE_INFO) ||
                       (msg_type == BLEPIPE_MSG_STREAM_PROFILE));

    default:
      return 0U;
  }
}
