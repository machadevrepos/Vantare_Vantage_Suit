#pragma once

#include <cstdint>

namespace exo {

/* Browser-to-Master transfer tuning wire contract.
 *
 * Version 1 remains the original 12-byte command and therefore selects the
 * wire-compatible 15 ms default. Version 2 appends one connection-interval
 * byte at offset 12. The value is a BLE connection interval in 1.25 ms units;
 * unsupported values are accepted but sanitized to the default so the Master
 * can echo the effective request without pretending it was confirmed. */
struct RecordTransferTuningWire {
  static constexpr uint8_t kCommand = 0xB5U;
  static constexpr uint8_t kVersion1 = 1U;
  static constexpr uint8_t kVersion2 = 2U;
  static constexpr uint16_t kV1Length = 12U;
  static constexpr uint16_t kV2Length = 13U;
  static constexpr uint8_t kFastIntervalOffset = 12U;
  static constexpr uint8_t kDefaultFastInterval = 12U;
  /* Firmware-owned bulk collection interval (30 ms). The browser's legacy
   * 15/11.25/7.5 ms benchmark values remain wire compatible. */
  static constexpr uint8_t kBulkFastInterval = 24U;

  struct DecodeResult {
    bool valid = false;
    uint8_t version = 0U;
    uint8_t fast_interval = kDefaultFastInterval;
  };

  static constexpr bool is_supported_fast_interval(uint8_t interval) {
    return interval == kBulkFastInterval || interval == 12U ||
           interval == 9U || interval == 6U;
  }

  static constexpr uint8_t sanitize_fast_interval(uint8_t interval) {
    return is_supported_fast_interval(interval) ? interval : kDefaultFastInterval;
  }

  static constexpr DecodeResult decode(const uint8_t *payload, uint16_t length) {
    DecodeResult result{};
    if (payload == nullptr || length < 2U || payload[0] != kCommand) {
      return result;
    }
    result.version = payload[1];
    if (result.version == kVersion1) {
      result.valid = length >= kV1Length;
      return result;
    }
    if (result.version == kVersion2 && length >= kV2Length) {
      result.valid = true;
      result.fast_interval = sanitize_fast_interval(payload[kFastIntervalOffset]);
    }
    return result;
  }
};

}  // namespace exo
