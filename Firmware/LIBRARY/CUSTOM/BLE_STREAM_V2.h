#ifndef BLE_STREAM_V2_H_
#define BLE_STREAM_V2_H_

#include <stdint.h>
#include <string.h>

namespace exo {

static constexpr uint8_t kBleV2FrameId = 0xB1U;
static constexpr uint8_t kBleV2Version = 0x01U;

enum class BleSensorId : uint8_t {
    Bno85 = 1U,
    Icm45686 = 2U,
    Flex = 3U
};

#pragma pack(push, 1)
struct BleV2EnvelopeHeader {
    uint8_t frame_id;
    uint8_t version;
    uint16_t node_id;
    uint8_t sensor_id;
    uint16_t sequence;
    uint32_t time_ms;
    uint16_t payload_len;
    uint8_t reserved0;
};
#pragma pack(pop)

static_assert(sizeof(BleV2EnvelopeHeader) == 14U, "Unexpected BLE V2 header size");

inline uint8_t ble_v2_pack(uint16_t node_id, uint8_t sensor_id, uint16_t sequence, uint32_t time_ms,
                           const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint8_t out_cap) {
    if (out == nullptr || payload == nullptr) {
        return 0U;
    }
    const uint8_t total = static_cast<uint8_t>(sizeof(BleV2EnvelopeHeader) + payload_len);
    if (total > out_cap) {
        return 0U;
    }
    BleV2EnvelopeHeader hdr{};
    hdr.frame_id = kBleV2FrameId;
    hdr.version = kBleV2Version;
    hdr.node_id = node_id;
    hdr.sensor_id = sensor_id;
    hdr.sequence = sequence;
    hdr.time_ms = time_ms;
    hdr.payload_len = payload_len;
    hdr.reserved0 = 0U;
    memcpy(out, &hdr, sizeof(hdr));
    if (payload_len > 0U) {
        memcpy(out + sizeof(hdr), payload, payload_len);
    }
    return total;
}

} // namespace exo

#endif
