#include <exo/ble/notification_gate.h>

#include <assert.h>

int main()
{
  exo::BleNotificationGate gate;
  assert(gate.ready(0U));

  // A send closes the gate; a completion / TX-pool event reopens it on the
  // fast path well before the watchdog would.
  gate.on_send_accepted(100U);
  assert(!gate.ready(101U));
  gate.on_transport_available();
  assert(gate.ready(101U));

  // Backpressure closes the gate; the TX-pool event reopens it.
  gate.on_send_accepted(200U);
  gate.on_backpressure(200U);
  assert(!gate.ready(205U));
  gate.on_transport_available();
  assert(gate.ready(205U));

  // Watchdog: if no controller event ever arrives (PipeDataTx completion events
  // can be coalesced or, without the mask bit, absent), the gate must still
  // reopen after kWatchdogMs so the stream degrades to time-paced, never stalls.
  gate.on_send_accepted(1000U);
  assert(!gate.ready(1000U + exo::BleNotificationGate::kWatchdogMs - 1U));
  assert(gate.ready(1000U + exo::BleNotificationGate::kWatchdogMs));
  assert(gate.watchdog_wake_count() == 1U);

  // Watchdog also covers a backpressure close with no TX-pool event.
  gate.on_backpressure(2000U);
  assert(!gate.ready(2000U));
  assert(gate.ready(2000U + exo::BleNotificationGate::kWatchdogMs));
  assert(gate.watchdog_wake_count() == 2U);

  // tick wrap-around (HAL_GetTick is uint32_t and rolls over ~49 days).
  const uint32_t near_wrap = 0xFFFFFFFFU - 5U;
  gate.on_send_accepted(near_wrap);
  assert(!gate.ready(near_wrap + 3U));
  assert(gate.ready(near_wrap + exo::BleNotificationGate::kWatchdogMs));

  // A non-backpressure failure self-heals immediately (no expected event).
  gate.on_send_accepted(3000U);
  gate.on_other_failure();
  assert(gate.ready(3000U));

  // reset() re-arms a gate left closed by a prior stream session.
  gate.on_send_accepted(4000U);
  gate.reset();
  assert(gate.ready(4000U));
  return 0;
}
