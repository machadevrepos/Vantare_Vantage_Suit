#ifndef EXO_BLE_NODE_UPLOAD_PUMP_H
#define EXO_BLE_NODE_UPLOAD_PUMP_H

#include <stdint.h>

namespace exo {

/*
 * Foreground-owned admission gate for reliable Node upload notifications.
 * BLE callbacks only publish wake events; flash reads and GATT calls remain in
 * the foreground.  A watchdog is deliberately a recovery path for a lost
 * controller event, never the normal upload clock.
 */
class NodeUploadPump {
public:
  enum class SendResult : uint8_t {
    Success,
    Busy,
    InsufficientResources,
    OtherFailure,
  };

  struct Metrics {
    uint32_t accepted_count = 0U;
    uint32_t accepted_bytes = 0U;
    uint32_t busy_count = 0U;
    uint32_t resource_count = 0U;
    uint32_t notification_complete_count = 0U;
    uint32_t tx_pool_event_count = 0U;
    uint32_t watchdog_wake_count = 0U;
    uint16_t tx_pool_buffers = 0U;
  };

  static constexpr uint32_t kWatchdogMs = 750U;

  static constexpr uint16_t notification_blocks(uint16_t att_mtu)
  {
    /* BLE_MEM_BLOCK_X_TX(mtu) from STM32_WPAN ble_bufsize.h. */
    return static_cast<uint16_t>(((static_cast<uint32_t>(att_mtu) + 4U + 31U) / 32U) + 1U);
  }

  static constexpr uint16_t upload_extra_blocks(uint16_t att_mtu, uint8_t notification_buffers)
  {
    return static_cast<uint16_t>(notification_blocks(att_mtu) * notification_buffers);
  }

  void start(uint32_t now_ms, uint8_t credit)
  {
    active_ = true;
    blocked_ = false;
    credit_ = credit;
    last_blocked_ms_ = now_ms;
    send_ready_ = (credit_ != 0U);
  }

  void stop()
  {
    active_ = false;
    blocked_ = false;
    credit_ = 0U;
    send_ready_ = false;
  }

  void set_credit(uint8_t credit)
  {
    credit_ = credit;
    if (active_ && credit_ != 0U) {
      blocked_ = false;
      send_ready_ = true;
    }
  }

  void on_notification_complete()
  {
    ++metrics_.notification_complete_count;
    wake_from_controller_event();
  }

  void on_tx_pool_available(uint16_t available_buffers)
  {
    ++metrics_.tx_pool_event_count;
    metrics_.tx_pool_buffers = available_buffers;
    wake_from_controller_event();
  }

  void on_send_result(SendResult result, uint32_t now_ms)
  {
    switch (result) {
      case SendResult::Success:
        ++metrics_.accepted_count;
        if (credit_ != 0U) {
          --credit_;
        }
        send_ready_ = (credit_ != 0U);
        blocked_ = false;
        break;
      case SendResult::Busy:
        ++metrics_.busy_count;
        block(now_ms);
        break;
      case SendResult::InsufficientResources:
        ++metrics_.resource_count;
        block(now_ms);
        break;
      case SendResult::OtherFailure:
        /* Preserve the chunk/cursor. A later controller event or watchdog may retry it. */
        block(now_ms);
        break;
    }
  }

  bool ready(uint32_t now_ms)
  {
    if (!active_ || credit_ == 0U) {
      return false;
    }
    if (send_ready_) {
      return true;
    }
    if (blocked_ && static_cast<uint32_t>(now_ms - last_blocked_ms_) >= kWatchdogMs) {
      blocked_ = false;
      send_ready_ = true;
      ++metrics_.watchdog_wake_count;
      return true;
    }
    return false;
  }

  bool active() const { return active_; }
  bool live_preview_suppressed() const { return active_; }
  uint8_t credit() const { return credit_; }
  const Metrics &metrics() const { return metrics_; }

  void on_send_accepted(uint16_t bytes)
  {
    metrics_.accepted_bytes += bytes;
  }

private:
  void block(uint32_t now_ms)
  {
    blocked_ = true;
    send_ready_ = false;
    last_blocked_ms_ = now_ms;
  }

  void wake_from_controller_event()
  {
    if (active_ && credit_ != 0U) {
      blocked_ = false;
      send_ready_ = true;
    }
  }

  bool active_ = false;
  bool blocked_ = false;
  bool send_ready_ = false;
  uint8_t credit_ = 0U;
  uint32_t last_blocked_ms_ = 0U;
  Metrics metrics_{};
};

} // namespace exo

#endif
