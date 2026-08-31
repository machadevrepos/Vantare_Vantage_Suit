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
    uint32_t terminal_error_count = 0U;
    uint16_t tx_pool_buffers = 0U;
    uint16_t max_tx_pool_buffers = 0U;
    uint16_t consecutive_accepted_count = 0U;
    uint16_t max_consecutive_accepted_count = 0U;
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

  static constexpr bool record_done_eligible(bool session_ready,
                                             bool uploading,
                                             bool record_done_sent,
                                             bool upload_active)
  {
    return (session_ready || uploading) && !record_done_sent && !upload_active;
  }

  void start(uint32_t now_ms, uint8_t credit)
  {
    active_ = true;
    terminal_error_ = false;
    blocked_ = false;
    credit_ = credit;
    last_blocked_ms_ = now_ms;
    send_ready_ = (credit_ != 0U);
    metrics_.consecutive_accepted_count = 0U;
  }

  void stop()
  {
    active_ = false;
    blocked_ = false;
    credit_ = 0U;
    send_ready_ = false;
    metrics_.consecutive_accepted_count = 0U;
  }

  void set_credit(uint8_t credit)
  {
    credit_ = credit;
    if (active_ && credit_ != 0U) {
      blocked_ = false;
      send_ready_ = true;
    }
  }

  void on_notification_complete(uint32_t events = 1U)
  {
    metrics_.notification_complete_count += events;
    wake_from_controller_event();
  }

  void on_tx_pool_available(uint16_t available_buffers, uint32_t events = 1U)
  {
    metrics_.tx_pool_event_count += events;
    metrics_.tx_pool_buffers = available_buffers;
    if (available_buffers > metrics_.max_tx_pool_buffers) {
      metrics_.max_tx_pool_buffers = available_buffers;
    }
    wake_from_controller_event();
  }

  void on_send_result(SendResult result, uint32_t now_ms)
  {
    switch (result) {
      case SendResult::Success:
        ++metrics_.accepted_count;
        if (metrics_.consecutive_accepted_count != 0xFFFFU) {
          ++metrics_.consecutive_accepted_count;
        }
        if (metrics_.consecutive_accepted_count > metrics_.max_consecutive_accepted_count) {
          metrics_.max_consecutive_accepted_count = metrics_.consecutive_accepted_count;
        }
        if (credit_ != 0U) {
          --credit_;
        }
        send_ready_ = (credit_ != 0U);
        blocked_ = false;
        break;
      case SendResult::Busy:
        ++metrics_.busy_count;
        metrics_.consecutive_accepted_count = 0U;
        block(now_ms);
        break;
      case SendResult::InsufficientResources:
        ++metrics_.resource_count;
        metrics_.consecutive_accepted_count = 0U;
        block(now_ms);
        break;
      case SendResult::OtherFailure:
        /* A non-backpressure error has no expected controller wake. Stop the
         * pump, preserve the reliable cursor externally, and let foreground
         * control decide when a resumable session is restarted. */
        (void)now_ms;
        ++metrics_.terminal_error_count;
        metrics_.consecutive_accepted_count = 0U;
        terminal_error_ = true;
        active_ = false;
        blocked_ = false;
        send_ready_ = false;
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
  bool terminal_error() const { return terminal_error_; }
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
  bool terminal_error_ = false;
  bool blocked_ = false;
  bool send_ready_ = false;
  uint8_t credit_ = 0U;
  uint32_t last_blocked_ms_ = 0U;
  Metrics metrics_{};
};

/* Pure, bounded DLE commissioning state. The BLE adapter maps vendor status
 * codes onto RequestResult and performs commands only when request_due() says
 * so; an HCI data-length event is the sole confirmation source. */
class NodeDleCommissioner {
public:
  enum class State : uint8_t {
    Unknown,
    Requested,
    Confirmed,
    Degraded,
    Failed,
  };

  enum class RequestResult : uint8_t {
    Success,
    Transient,
    Failed,
  };

  static constexpr uint32_t kRetryMs = 250U;
  static constexpr uint32_t kCompletionTimeoutMs = 1000U;
  static constexpr uint8_t kMaxAttempts = 3U;

  void start(uint32_t now_ms)
  {
    state_ = State::Unknown;
    attempts_ = 0U;
    next_attempt_ms_ = now_ms;
    completion_deadline_ms_ = 0U;
    negotiated_tx_octets_ = 0U;
    negotiated_rx_octets_ = 0U;
  }

  bool request_due(uint32_t now_ms) const
  {
    return (state_ == State::Unknown || state_ == State::Degraded) &&
           attempts_ < kMaxAttempts &&
           static_cast<uint32_t>(now_ms - next_attempt_ms_) < 0x80000000U;
  }

  void on_request_result(RequestResult result, uint32_t now_ms)
  {
    if (!request_due(now_ms)) {
      return;
    }
    ++attempts_;
    if (result == RequestResult::Success) {
      state_ = State::Requested;
      completion_deadline_ms_ = now_ms + kCompletionTimeoutMs;
    } else if (result == RequestResult::Transient && attempts_ < kMaxAttempts) {
      state_ = State::Degraded;
      next_attempt_ms_ = now_ms + kRetryMs;
    } else {
      state_ = State::Failed;
    }
  }

  void process(uint32_t now_ms)
  {
    if (state_ == State::Requested &&
        static_cast<uint32_t>(now_ms - completion_deadline_ms_) < 0x80000000U) {
      if (attempts_ < kMaxAttempts) {
        state_ = State::Degraded;
        next_attempt_ms_ = now_ms + kRetryMs;
      } else {
        state_ = State::Failed;
      }
    }
  }

  void on_complete(uint16_t tx_octets, uint16_t rx_octets)
  {
    negotiated_tx_octets_ = tx_octets;
    negotiated_rx_octets_ = rx_octets;
    state_ = State::Confirmed;
  }

  State state() const { return state_; }
  uint8_t attempts() const { return attempts_; }
  uint16_t negotiated_tx_octets() const { return negotiated_tx_octets_; }
  uint16_t negotiated_rx_octets() const { return negotiated_rx_octets_; }

private:
  State state_ = State::Unknown;
  uint8_t attempts_ = 0U;
  uint32_t next_attempt_ms_ = 0U;
  uint32_t completion_deadline_ms_ = 0U;
  uint16_t negotiated_tx_octets_ = 0U;
  uint16_t negotiated_rx_octets_ = 0U;
};

} // namespace exo

#endif
