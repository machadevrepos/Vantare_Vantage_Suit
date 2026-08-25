#include <exo/ble/node_upload_pump.h>

#include <assert.h>

static_assert(exo::NodeUploadPump::notification_blocks(247U) == 9U,
              "a 247-byte ATT notification needs nine 32-byte BLE blocks");
static_assert(exo::NodeUploadPump::upload_extra_blocks(247U, 4U) == 36U,
              "four complete 247-byte ATT notification buffers need 36 blocks");

int main()
{
  exo::NodeUploadPump pump;

  // A ready upload starts immediately and only suppresses live preview while active.
  pump.start(10U, 2U);
  assert(pump.live_preview_suppressed());
  assert(pump.ready(10U));
  pump.on_send_result(exo::NodeUploadPump::SendResult::Busy, 10U);
  assert(!pump.ready(11U));
  assert(pump.metrics().busy_count == 1U);

  // Notification completion is the normal wake path after BLE_STATUS_BUSY.
  pump.on_notification_complete();
  assert(pump.ready(11U));
  assert(pump.metrics().notification_complete_count == 1U);

  pump.on_send_result(exo::NodeUploadPump::SendResult::InsufficientResources, 12U);
  assert(!pump.ready(12U));
  assert(pump.metrics().resource_count == 1U);

  // TX pool availability is the normal wake path after resource exhaustion.
  pump.on_tx_pool_available(3U);
  assert(pump.ready(13U));
  assert(pump.metrics().tx_pool_event_count == 1U);
  assert(pump.metrics().tx_pool_buffers == 3U);

  // Sending consumes reliable-protocol credit; no notification is attempted at zero.
  pump.on_send_result(exo::NodeUploadPump::SendResult::Success, 14U);
  assert(pump.credit() == 1U);
  pump.on_send_result(exo::NodeUploadPump::SendResult::Success, 15U);
  assert(pump.credit() == 0U);
  assert(!pump.ready(16U));

  // Lost controller events may be recovered only by the long watchdog path.
  pump.set_credit(1U);
  pump.on_send_result(exo::NodeUploadPump::SendResult::Busy, 20U);
  assert(!pump.ready(20U + exo::NodeUploadPump::kWatchdogMs - 1U));
  assert(pump.ready(20U + exo::NodeUploadPump::kWatchdogMs));
  assert(pump.metrics().watchdog_wake_count == 1U);

  pump.stop();
  assert(!pump.live_preview_suppressed());
  assert(!pump.ready(1000U));
  return 0;
}
