#pragma once

#include <cstdint>

namespace exo {

/* Compact Master 0xB6 report layout. Version 3 changes only the version byte
 * in the v2 prefix and appends benchmark evidence after byte 58. */
struct MasterTransferTelemetryWire {
  static constexpr uint16_t kLegacyLength = 19U;
  static constexpr uint16_t kV2Length = 59U;
  static constexpr uint16_t kV3Length = 69U;
  static constexpr uint8_t kVersionOffset = 19U;
  static constexpr uint8_t kVersion3 = 3U;
  static constexpr uint8_t kConfiguredFastIntervalOffset = 59U;
  static constexpr uint8_t kCounterSourceOffset = 60U;
  static constexpr uint8_t kUniqueAcceptedOffset = 61U;
  static constexpr uint8_t kDuplicateOffset = 65U;

  struct V3Fields {
    uint8_t configured_fast_interval = 12U;
    uint8_t counter_source_id = 0U;
    uint32_t unique_accepted_chunks = 0U;
    uint32_t duplicate_chunks = 0U;
  };

  static constexpr bool append_v3(uint8_t *payload, uint16_t length,
                                  const V3Fields &fields) {
    if (payload == nullptr || length < kV3Length) {
      return false;
    }
    payload[kVersionOffset] = kVersion3;
    payload[kConfiguredFastIntervalOffset] = fields.configured_fast_interval;
    payload[kCounterSourceOffset] = fields.counter_source_id;
    put_u32(payload, kUniqueAcceptedOffset, fields.unique_accepted_chunks);
    put_u32(payload, kDuplicateOffset, fields.duplicate_chunks);
    return true;
  }

 private:
  static constexpr void put_u32(uint8_t *payload, uint8_t offset, uint32_t value) {
    payload[offset] = static_cast<uint8_t>(value & 0xFFU);
    payload[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    payload[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    payload[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
  }
};

}  // namespace exo
