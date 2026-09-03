#ifndef EXO_BLE_NOTIFICATION_GATE_H_
#define EXO_BLE_NOTIFICATION_GATE_H_

#include <stdint.h>

namespace exo {

/*
 * Admission gate for a characteristic with one controller-owned notification
 * in flight.  Controller callbacks (notification-complete / TX-pool-available)
 * reopen the gate on the fast path; foreground code owns all GATT calls.
 *
 * The watchdog is a safety net, never the clock.  The notification-complete
 * event only exists when the characteristic was created with the
 * GATT_NOTIFY_NOTIFICATION_COMPLETION mask, and even then the stack may
 * coalesce it (one event per connection-event flush, not one per send).  The
 * TX-pool event only arms after a real BLE_STATUS_INSUFFICIENT_RESOURCES
 * return.  If the expected event is delayed, coalesced, or never emitted, the
 * gate still reopens after kWatchdogMs so the stream degrades to time-paced
 * instead of latching closed forever.  This mirrors NodeUploadPump::kWatchdogMs,
 * whose header notes the watchdog is "deliberately a recovery path for a lost
 * controller event, never the normal upload clock".
 *
 * reset() is intentionally explicit because a stream restart can occur after
 * the previous notification's completion event was already consumed.
 */
class BleNotificationGate {
public:
  /* Matches HubLeafBleManager::kNormalPreviewIntervalMs.  Short enough that a
   * link that never emits a completion event still paces at the pre-gate
   * cadence (~100 sends/s, well above one node's ~50 samples/s); long enough to
   * let a healthy completion / TX-pool event win the race on the fast path. */
  static constexpr uint32_t kWatchdogMs = 10U;

  bool ready(uint32_t now_ms) {
    if (ready_) {
      return true;
    }
    if (static_cast<uint32_t>(now_ms - closed_at_ms_) >= kWatchdogMs) {
      ready_ = true;
      ++watchdog_wake_count_;
      return true;
    }
    return false;
  }

  void reset() { ready_ = true; }
  void on_transport_available() { ready_ = true; }

  void on_send_accepted(uint32_t now_ms) {
    ready_ = false;
    closed_at_ms_ = now_ms;
  }
  void on_backpressure(uint32_t now_ms) {
    ready_ = false;
    closed_at_ms_ = now_ms;
  }
  void on_other_failure() { ready_ = true; }

  uint32_t watchdog_wake_count() const { return watchdog_wake_count_; }

private:
  bool ready_ = true;
  uint32_t closed_at_ms_ = 0U;
  uint32_t watchdog_wake_count_ = 0U;
};

} // namespace exo

#endif
