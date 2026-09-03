#include <exo/ble/exo_hub_central_client.h>
#include <exo/ble/link_tune_state.h>
#include <exo/protocol/ble_record_protocol.h>

#include <string.h>

#include "main.h"
#include "app_conf.h"
#include <exo/ble/custom_app.h>
#include "dbg_trace.h"
/* STM32_WPAN ACI headers are plain C (no __cplusplus guards); give them C
 * linkage explicitly so call sites do not emit mangled references. */
#ifdef __cplusplus
extern "C" {
#endif
#include "ble_gap_aci.h"
#include "ble_gatt_aci.h"
#include "ble_hci_le.h"
#include "ble_events.h"
#include "ble_std.h"
#include "ble_types.h"
#ifdef __cplusplus
}
#endif
#include <exo/ble/exo_hub_leaf_bridge.h>

extern void exo_ble_debug_printf(const char *fmt, ...);
#define EXO_LOG exo_ble_debug_printf

/* Per-notification logging fires once per transferred chunk; at transfer
 * rates above ~10 chunks/s the console writes themselves stretch the
 * superloop. Keep them behind this default-off switch. */
#ifndef EXO_HUB_VERBOSE_PIPE_LOGS
#define EXO_HUB_VERBOSE_PIPE_LOGS 0
#endif

#pragma pack(push, 1)
typedef struct
{
  uint8_t command;
  uint16_t node_id;
  uint32_t session_id;
  uint32_t actual_duration_ms;
  uint32_t total_size;
  uint32_t payload_crc32;
} ExoRecordDoneWire;
#pragma pack(pop)

/* These live in app_ble.cpp / hub_leaf_ble_manager with C linkage (declared
 * extern "C" in their headers); keep the local declarations consistent or the
 * call sites emit mangled references. */
extern "C" void APP_BLE_LeafClientConnecting(void);
extern "C" void APP_BLE_LeafClientConnectIdle(void);
extern "C" uint8_t APP_BLE_LeafClientPrepareScan(void);
extern "C" void APP_BLE_LeafClientScanIdle(void);
extern "C" uint8_t APP_BLE_LeafClientPhoneConnected(void);
extern "C" void exo_hub_leaf_control_ingest(uint8_t node_id,
                                         uint8_t msg_type,
                                         const uint8_t *payload,
                                         uint16_t payload_len);
extern "C" uint8_t exo_master_training_owns_node_link(uint8_t node_id);
extern "C" uint8_t exo_master_training_raw_download_debug_enabled(void);
extern "C" void exo_master_training_note_suppressed_relay(void);

/* Four physical suit nodes; CFG_BLE_NUM_LINK still reserves the browser link. */
#define EXO_HUB_LEAF_MAX                 4U
#define EXO_HUB_SCAN_INTERVAL            0x0040U
#define EXO_HUB_SCAN_WINDOW              0x0030U
#define EXO_HUB_SCAN_INTERVAL_CONNECTED  0x00A0U
#define EXO_HUB_SCAN_WINDOW_CONNECTED    0x0010U
#define EXO_HUB_CONN_INTERVAL_MIN        0x0006U
#define EXO_HUB_CONN_INTERVAL_MAX        0x0008U
/* The first STM32WB central link influences later scheduler placement; use a shared multi-link interval from the first leaf onward. */
#define EXO_HUB_CONN_INTERVAL_MIN_MULTI  0x0018U
#define EXO_HUB_CONN_INTERVAL_MAX_MULTI  0x0028U
#define EXO_HUB_CONN_LATENCY             0x0000U
#define EXO_HUB_SUPERVISION_TIMEOUT      0x00C8U
/* STM32WB status 0x86 can mean the requested CE length does not fit the scheduler. Zero lets the controller choose. */
#define EXO_HUB_MIN_CE_LENGTH            0x0000U
#define EXO_HUB_MAX_CE_LENGTH            0x0000U
#define EXO_HUB_PROC_GENERAL_DISCOVERY   0x02U
#define EXO_HUB_PROC_GENERAL_CONNECTION  0x10U
#define EXO_HUB_PROC_DIRECT_CONNECTION   0x40U
#define EXO_HUB_NOTIFY_ENABLE            0x0001U
#define EXO_HUB_BACKOFF_MS               1500U
#define EXO_HUB_BLE_READY_SCAN_DELAY_MS  750U
#define EXO_HUB_SCAN_RETRY_MS            500U
/* Direct-address recovery of one dropped Node during a session hold: settle
 * delay before the connect attempt, and a cap on consecutive failed attempts
 * so a dead Node cannot churn the radio scheduler for the whole session. */
#define EXO_HUB_TARGETED_RECONNECT_DELAY_MS  250U
#define EXO_HUB_TARGETED_RECONNECT_MAX_ATTEMPTS 3U
#define EXO_HUB_SCAN_RESUME_MS           500U
#define EXO_HUB_SCAN_BUSY_RETRY_MS       3000U
#define EXO_HUB_CONNECT_AFTER_SCAN_MS    120U
#define EXO_HUB_SCAN_WINDOW_MS           5000U
#define EXO_HUB_PHONE_ADV_PAUSE_MS       5000U
#define EXO_HUB_DISC_REPORT_ID           0xB5U

#define EXO_ADV_TYPE_COMPLETE_NAME       0x09U
#define EXO_ADV_TYPE_SHORT_NAME          0x08U

typedef enum
{
  EXO_LEAF_SLOT_EMPTY = 0,
  EXO_LEAF_SLOT_DISCOVERED,
  EXO_LEAF_SLOT_CONNECTING,
  EXO_LEAF_SLOT_EXCHANGE_MTU,
  EXO_LEAF_SLOT_DISCOVER_SERVICE,
  EXO_LEAF_SLOT_DISCOVER_CHARS,
  EXO_LEAF_SLOT_ENABLE_CTRL_NOTIFY,
  EXO_LEAF_SLOT_ENABLE_STATUS_NOTIFY,
  EXO_LEAF_SLOT_ENABLE_DATA_NOTIFY,
  EXO_LEAF_SLOT_READY,
  EXO_LEAF_SLOT_BACKOFF
} exo_leaf_state_t;

typedef enum
{
  EXO_DISC_EVT_SCAN_REQUESTED = 0x01U,
  EXO_DISC_EVT_SCAN_STARTED = 0x02U,
  EXO_DISC_EVT_SCAN_FAILED = 0x03U,
  EXO_DISC_EVT_ADV_PARSED = 0x04U,
  EXO_DISC_EVT_ADV_SKIPPED = 0x05U,
  EXO_DISC_EVT_CONNECT_QUEUED = 0x06U,
  EXO_DISC_EVT_CONNECT_STARTED = 0x07U,
  EXO_DISC_EVT_CONNECT_FAILED = 0x08U,
  EXO_DISC_EVT_LEAF_READY = 0x09U,
  EXO_DISC_EVT_SCAN_DELAYED = 0x0AU,
  /* Link-tuning telemetry surfaced to the desktop console (no SWO needed):
   * value carries the negotiated figure - DLE MaxTxOctets, the TX/RX PHY byte
   * pair, and the applied connection interval respectively. */
  EXO_DISC_EVT_LINK_DLE = 0x10U,
  EXO_DISC_EVT_LINK_PHY = 0x11U,
  EXO_DISC_EVT_LINK_TIMING = 0x12U,
  /* Request-acceptance telemetry from the deferred tuning path: value = the hci
   * command status (0 = accepted). These fire even when the WB default mask
   * suppresses the DLE/PHY completion events, so the console still shows the
   * link tuning was issued and accepted. */
  EXO_DISC_EVT_LINK_DLE_REQ = 0x13U,
  EXO_DISC_EVT_LINK_PHY_REQ = 0x14U,
  EXO_DISC_EVT_LINK_INTERVAL_REQ = 0x15U,
  EXO_DISC_EVT_LINK_STATE = 0x16U
} exo_disc_event_t;

typedef struct
{
  exo_leaf_state_t state;
  uint8_t addr_type;
  uint8_t addr[6];
  uint8_t node_id;
  uint8_t node_hint;
  uint16_t connection_handle;
  uint16_t service_start_handle;
  uint16_t service_end_handle;
  uint16_t data_decl_handle;
  uint16_t data_value_handle;
  uint16_t ctrl_rx_value_handle;
  uint16_t ctrl_tx_decl_handle;
  uint16_t ctrl_tx_value_handle;
  uint16_t status_decl_handle;
  uint16_t status_value_handle;
  uint16_t config_value_handle;
  uint8_t notify_mask;
  uint8_t app_record_ready;
  uint8_t app_recorder_state;
  uint32_t app_record_session_id;
  uint32_t app_maximum_duration_ms;
  uint8_t mtu_exchange_done;
  uint32_t retry_after_ms;
  uint8_t seen_in_scan;
  /* Last negotiated link tuning, captured at connect and re-surfaced to the
   * desktop console at transfer start (the connect-time events usually fire
   * before the browser is attached). */
  uint16_t link_dle_tx_oct;
  uint16_t link_dle_rx_oct;
  uint8_t link_tx_phy;
  uint8_t link_rx_phy;
} exo_leaf_slot_t;

static exo_leaf_slot_t g_leaf_slots[EXO_HUB_LEAF_MAX];
static uint8_t g_scan_requested = 0U;
static uint8_t g_scan_active = 0U;
static uint8_t g_connect_busy = 0U;
static uint8_t g_discovery_active = 0U;
static uint8_t g_pending_slot = 0xFFU;
static uint8_t g_ble_ready = 0U;
static uint8_t g_connect_after_scan_slot = 0xFFU;
static uint32_t g_next_scan_after_ms = 0U;
static uint32_t g_connect_after_scan_ms = 0U;
static uint32_t g_scan_started_ms = 0U;
static uint8_t g_scan_timeout_stop = 0U;
static uint8_t g_scan_proc_code = EXO_HUB_PROC_GENERAL_DISCOVERY;
static uint8_t g_last_logged_all_ready_mask = 0U;
static uint8_t g_discovery_hold = 0U;
static uint8_t g_targeted_reconnect_node_id = 0U;
static uint32_t g_targeted_reconnect_after_ms = 0U;
static uint8_t g_targeted_reconnect_attempts = 0U;
static exo::LinkTuneState g_link_tune;
static exo::TransferLinkRearmState g_transfer_link_rearm;

/* Latest live-preview forwarding health reported by each leaf (LINK_STATS v2
 * tail). Indexed by node_id-1. Surfaced to the web log via the Master diag. */
typedef struct {
  uint32_t offered;
  uint32_t dropped;
  uint32_t sent;
  uint32_t gate_wdog;
  uint32_t gate_bp;
  uint32_t bno_fresh;
  uint32_t icm_fresh;
  uint16_t dle_tx_octets;
  uint8_t  stream_on;
  uint8_t  valid;
} exo_leaf_live_diag_t;
static exo_leaf_live_diag_t g_leaf_live_diag[EXO_HUB_LEAF_MAX];

static const uint8_t k_blepipe_service_uuid[16] = { 0x3f, 0x88, 0x10, 0x00, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };
static const uint8_t k_blepipe_data_uuid[16]    = { 0x3f, 0x88, 0x10, 0x01, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };
static const uint8_t k_blepipe_ctrl_rx_uuid[16] = { 0x3f, 0x88, 0x10, 0x02, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };
static const uint8_t k_blepipe_ctrl_tx_uuid[16] = { 0x3f, 0x88, 0x10, 0x03, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };
static const uint8_t k_blepipe_status_uuid[16]  = { 0x3f, 0x88, 0x10, 0x04, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };
static const uint8_t k_blepipe_config_uuid[16]  = { 0x3f, 0x88, 0x10, 0x05, 0xb4, 0xa5, 0x4f, 0x7c, 0x9b, 0x60, 0x98, 0xe0, 0xb5, 0xc8, 0xa0, 0x00 };

static uint8_t exo_leaf_slot_node_id(const exo_leaf_slot_t *slot);

static uint8_t exo_uuid_matches(const uint8_t *lhs, const uint8_t *rhs)
{
  uint8_t i;
  uint8_t equal = 1U;
  uint8_t reversed = 1U;
  for (i = 0U; i < 16U; ++i)
  {
    if (lhs[i] != rhs[i])
    {
      equal = 0U;
    }
    if (lhs[i] != rhs[15U - i])
    {
      reversed = 0U;
    }
  }
  return (uint8_t)((equal != 0U) || (reversed != 0U));
}

static void exo_leaf_slot_reset_handles(exo_leaf_slot_t *slot)
{
  slot->service_start_handle = 0U;
  slot->service_end_handle = 0U;
  slot->data_decl_handle = 0U;
  slot->data_value_handle = 0U;
  slot->ctrl_rx_value_handle = 0U;
  slot->ctrl_tx_decl_handle = 0U;
  slot->ctrl_tx_value_handle = 0U;
  slot->status_decl_handle = 0U;
  slot->status_value_handle = 0U;
  slot->config_value_handle = 0U;
  slot->notify_mask = 0U;
  slot->app_record_ready = 0U;
  slot->app_recorder_state = 0U;
  slot->app_record_session_id = 0U;
  slot->mtu_exchange_done = 0U;
}

static void exo_leaf_slot_mark_backoff(exo_leaf_slot_t *slot)
{
  g_transfer_link_rearm.on_link_disconnected(exo_leaf_slot_node_id(slot));
  slot->state = EXO_LEAF_SLOT_BACKOFF;
  slot->connection_handle = 0xFFFFU;
  slot->retry_after_ms = HAL_GetTick() + EXO_HUB_BACKOFF_MS;
  exo_leaf_slot_reset_handles(slot);
  g_connect_busy = 0U;
  g_discovery_active = 0U;
  if (g_pending_slot != 0xFFU && &g_leaf_slots[g_pending_slot] == slot)
  {
    g_pending_slot = 0xFFU;
  }
  if (g_connect_after_scan_slot != 0xFFU && &g_leaf_slots[g_connect_after_scan_slot] == slot)
  {
    g_connect_after_scan_slot = 0xFFU;
    g_connect_after_scan_ms = 0U;
  }
}

static exo_leaf_slot_t *exo_find_slot_by_conn(uint16_t connection_handle)
{
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].connection_handle == connection_handle &&
        g_leaf_slots[i].state != EXO_LEAF_SLOT_EMPTY)
    {
      return &g_leaf_slots[i];
    }
  }
  return 0;
}

static exo_leaf_slot_t *exo_find_slot_by_node(uint8_t node_id)
{
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_EMPTY)
    {
      continue;
    }
    if (g_leaf_slots[i].node_id == node_id || g_leaf_slots[i].node_hint == node_id)
    {
      return &g_leaf_slots[i];
    }
  }
  return 0;
}

static exo_leaf_slot_t *exo_find_slot_by_addr(uint8_t addr_type, const uint8_t *addr)
{
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_EMPTY)
    {
      continue;
    }
    if (g_leaf_slots[i].addr_type == addr_type &&
        memcmp(g_leaf_slots[i].addr, addr, 6U) == 0)
    {
      return &g_leaf_slots[i];
    }
  }
  return 0;
}

static exo_leaf_slot_t *exo_claim_slot(uint8_t addr_type, const uint8_t *addr)
{
  uint8_t i;
  exo_leaf_slot_t *slot = exo_find_slot_by_addr(addr_type, addr);
  if (slot != 0)
  {
    return slot;
  }
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_EMPTY)
    {
      memset(&g_leaf_slots[i], 0, sizeof(g_leaf_slots[i]));
      g_leaf_slots[i].state = EXO_LEAF_SLOT_DISCOVERED;
      g_leaf_slots[i].addr_type = addr_type;
      memcpy(g_leaf_slots[i].addr, addr, 6U);
      g_leaf_slots[i].connection_handle = 0xFFFFU;
      return &g_leaf_slots[i];
    }
  }
  return 0;
}

static uint8_t exo_ready_leaf_count(void)
{
  uint8_t count = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY &&
        g_leaf_slots[i].app_record_ready != 0U)
    {
      ++count;
    }
  }
  return count;
}

static uint8_t exo_active_leaf_count(void)
{
  uint8_t count = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state != EXO_LEAF_SLOT_EMPTY)
    {
      ++count;
    }
  }
  return count;
}

static uint8_t exo_ready_or_connecting_leaf_count(void)
{
  uint8_t count = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY ||
        g_leaf_slots[i].state == EXO_LEAF_SLOT_CONNECTING)
    {
      ++count;
    }
  }
  return count;
}

static uint8_t exo_should_resume_scan_after_timeout(void)
{
  if (g_scan_timeout_stop == 0U)
  {
    return 0U;
  }
  if (APP_BLE_LeafClientPhoneConnected() != 0U)
  {
    return 0U;
  }
  if (exo_ready_or_connecting_leaf_count() != 0U)
  {
    return 0U;
  }
  return 1U;
}

static uint8_t exo_find_next_connectable_slot(void)
{
  uint8_t i;
  uint8_t best_slot = 0xFFU;
  uint8_t best_node = 0xFFU;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_DISCOVERED)
    {
      const uint8_t node_id = exo_leaf_slot_node_id(&g_leaf_slots[i]);
      if (node_id < best_node)
      {
        best_node = node_id;
        best_slot = i;
      }
    }
  }
  return best_slot;
}

static uint8_t exo_leaf_slot_node_id(const exo_leaf_slot_t *slot)
{
  if (slot == 0)
  {
    return 0U;
  }
  return (uint8_t)(slot->node_id != 0U ? slot->node_id : slot->node_hint);
}

static uint16_t exo_scan_interval_for_state(void)
{
  return (exo_active_leaf_count() == 0U) ? EXO_HUB_SCAN_INTERVAL : EXO_HUB_SCAN_INTERVAL_CONNECTED;
}

static uint16_t exo_scan_window_for_state(void)
{
  return (exo_active_leaf_count() == 0U) ? EXO_HUB_SCAN_WINDOW : EXO_HUB_SCAN_WINDOW_CONNECTED;
}

static uint16_t exo_conn_interval_min_for_state(void)
{
  return EXO_HUB_CONN_INTERVAL_MIN_MULTI;
}

static uint16_t exo_conn_interval_max_for_state(void)
{
  return EXO_HUB_CONN_INTERVAL_MAX_MULTI;
}

static uint32_t exo_scan_retry_ms_for_status(tBleStatus status)
{
  return (status == BLE_STATUS_LENGTH_FAILED) ? EXO_HUB_SCAN_BUSY_RETRY_MS : EXO_HUB_SCAN_RETRY_MS;
}

static void exo_send_disc_report(exo_disc_event_t event_id,
                                 uint8_t node_id,
                                 uint8_t slot_index,
                                 uint8_t state,
                                 uint16_t value)
{
  uint8_t payload[8];
  payload[0] = (uint8_t)event_id;
  payload[1] = node_id;
  payload[2] = slot_index;
  payload[3] = state;
  payload[4] = (uint8_t)(value & 0xFFU);
  payload[5] = (uint8_t)((value >> 8U) & 0xFFU);
  payload[6] = exo_hub_central_client_ready_node_mask();
  payload[7] = exo_hub_central_client_transport_ready_node_mask();
  (void)Custom_APP_SendCmdReport(EXO_HUB_DISC_REPORT_ID, payload, (uint8_t)sizeof(payload));
}

static void exo_report_link_tune(uint8_t slot_index)
{
  const exo::LinkTuneState::Telemetry &t = g_link_tune.telemetry(slot_index);
  const exo_leaf_slot_t *const slot = &g_leaf_slots[slot_index];
  EXO_LOG("[BLE][HUB][LINK] node=%u slot=%u h=%04X gen=%lu state=%u "
          "dle=req%u/ok%u,%u phy=req%u,%u/ok%u,%u ci=req%u-%u/ok%u "
          "ce=req%u-%u prep=%lums retry=%u st=0x%02X\r\n",
          (unsigned)exo_leaf_slot_node_id(slot),
          (unsigned)slot_index,
          (unsigned)t.handle,
          (unsigned long)t.generation,
          (unsigned)t.state,
          (unsigned)t.requested_dle_octets,
          (unsigned)t.confirmed_dle_tx_octets,
          (unsigned)t.confirmed_dle_rx_octets,
          (unsigned)t.requested_tx_phy,
          (unsigned)t.requested_rx_phy,
          (unsigned)t.confirmed_tx_phy,
          (unsigned)t.confirmed_rx_phy,
          (unsigned)t.requested_interval_min,
          (unsigned)t.requested_interval_max,
          (unsigned)t.confirmed_interval,
          (unsigned)t.requested_min_ce_length,
          (unsigned)t.requested_max_ce_length,
          (unsigned long)t.preparation_duration_ms,
          (unsigned)t.retries,
          (unsigned)t.status);
  exo_send_disc_report(EXO_DISC_EVT_LINK_STATE,
                       exo_leaf_slot_node_id(slot),
                       slot_index,
                       (uint8_t)t.state,
                       (uint16_t)((uint16_t)t.status | ((uint16_t)t.retries << 8U)));
}

static void exo_issue_next_link_tune(uint32_t now)
{
  const exo::LinkTuneState::Request request = g_link_tune.issue_next(now);
  tBleStatus status = BLE_STATUS_INVALID_PARAMS;
  exo_leaf_slot_t *slot;
  if (!request.valid())
  {
    return;
  }
  slot = &g_leaf_slots[request.link];
  if (request.procedure == exo::LinkTuneState::Procedure::Dle)
  {
    status = hci_le_set_data_length(request.handle, request.dle_octets, request.dle_time_us);
    exo_send_disc_report(EXO_DISC_EVT_LINK_DLE_REQ, exo_leaf_slot_node_id(slot),
                         request.link, 0U, (uint16_t)status);
  }
  else if (request.procedure == exo::LinkTuneState::Procedure::Phy)
  {
    status = hci_le_set_phy(request.handle, 0U,
                            HCI_TX_PHYS_LE_2M_PREF, HCI_RX_PHYS_LE_2M_PREF, 0U);
    exo_send_disc_report(EXO_DISC_EVT_LINK_PHY_REQ, exo_leaf_slot_node_id(slot),
                         request.link, 0U, (uint16_t)status);
  }
  else
  {
    /* CPU2 wireless stacks v1.14+ removed the standard HCI LE Connection
     * Update opcode (OCF 0x0013 answers 0x01 Unknown Command) and route the
     * identical request through the vendor command below. Try the vendor
     * command first, then fall back to the legacy opcode so the same
     * firmware still tunes links on pre-v1.14 stacks. */
    status = aci_gap_start_connection_update(request.handle,
                                             request.interval_min, request.interval_max,
                                             EXO_HUB_CONN_LATENCY,
                                             EXO_HUB_SUPERVISION_TIMEOUT,
                                             request.min_ce_length,
                                             request.max_ce_length);
    if (status == 0x01U)
    {
      status = hci_le_connection_update(request.handle,
                                        request.interval_min, request.interval_max,
                                        EXO_HUB_CONN_LATENCY,
                                        EXO_HUB_SUPERVISION_TIMEOUT,
                                        request.min_ce_length,
                                        request.max_ce_length);
    }
    exo_send_disc_report(EXO_DISC_EVT_LINK_INTERVAL_REQ, exo_leaf_slot_node_id(slot),
                         request.link, (uint8_t)request.interval,
                         (uint16_t)(request.interval_min | ((uint16_t)status << 8U)));
  }
  (void)g_link_tune.on_request_status(request, (uint8_t)status, now);
  EXO_LOG("[BLE][HUB][LINK] issue node=%u slot=%u h=%04X gen=%lu proc=%u iv=%u-%u ce=%u-%u st=0x%02X\r\n",
          (unsigned)exo_leaf_slot_node_id(slot),
          (unsigned)request.link,
          (unsigned)request.handle,
          (unsigned long)request.generation,
          (unsigned)request.procedure,
          (unsigned)request.interval_min,
          (unsigned)request.interval_max,
          (unsigned)request.min_ce_length,
          (unsigned)request.max_ce_length,
          (unsigned)status);
  exo_report_link_tune(request.link);
}

uint8_t exo_hub_central_client_ready_node_mask(void)
{
  uint8_t mask = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY &&
        g_leaf_slots[i].app_record_ready != 0U)
    {
      const uint8_t node_id = exo_leaf_slot_node_id(&g_leaf_slots[i]);
      if (node_id != 0U && node_id < 8U)
      {
        mask = (uint8_t)(mask | (uint8_t)(1U << node_id));
      }
    }
  }
  return mask;
}

uint32_t exo_hub_central_client_maximum_duration_ms(uint8_t node_mask)
{
  uint32_t minimum_ms = 0xFFFFFFFFUL;
  uint8_t matched_mask = 0U;
  for (uint8_t i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    const exo_leaf_slot_t *slot = &g_leaf_slots[i];
    const uint8_t node_id = exo_leaf_slot_node_id(slot);
    if (node_id == 0U || node_id >= 8U)
    {
      continue;
    }
    const uint8_t bit = (uint8_t)(1U << node_id);
    if ((node_mask & bit) == 0U || slot->app_record_ready == 0U ||
        slot->app_maximum_duration_ms == 0U)
    {
      continue;
    }
    matched_mask = (uint8_t)(matched_mask | bit);
    if (slot->app_maximum_duration_ms < minimum_ms)
    {
      minimum_ms = slot->app_maximum_duration_ms;
    }
  }
  return matched_mask == node_mask ? minimum_ms : 0U;
}

uint8_t exo_hub_central_client_ready_node_count(void)
{
  return exo_ready_leaf_count();
}

uint8_t exo_hub_central_client_transport_ready_node_mask(void)
{
  uint8_t mask = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY)
    {
      const uint8_t node_id = exo_leaf_slot_node_id(&g_leaf_slots[i]);
      if (node_id != 0U && node_id < 8U)
      {
        mask = (uint8_t)(mask | (uint8_t)(1U << node_id));
      }
    }
  }
  return mask;
}

uint8_t exo_hub_central_client_transport_ready_node_count(void)
{
  uint8_t count = 0U;
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY)
    {
      ++count;
    }
  }
  return count;
}

static uint8_t exo_parse_leaf_name_id(const uint8_t *name, uint8_t len)
{
  uint8_t id = 0U;
  uint8_t i;
  uint8_t digits = 0U;
  if (name == 0 || len < 2U || name[0] != (uint8_t)'L')
  {
    return 0U;
  }
  for (i = 1U; i < len; ++i)
  {
    if (name[i] < (uint8_t)'0' || name[i] > (uint8_t)'9')
    {
      return 0U;
    }
    id = (uint8_t)((id * 10U) + (uint8_t)(name[i] - (uint8_t)'0'));
    ++digits;
  }
  if (digits >= 2U)
  {
    const uint8_t tail = (uint8_t)(((uint8_t)(name[len - 2U] - (uint8_t)'0') * 10U) +
                                   (uint8_t)(name[len - 1U] - (uint8_t)'0'));
    if (tail != 0U)
    {
      return tail;
    }
  }
  return id;
}

static void exo_log_adv_report(const Advertising_Report_t *report, uint8_t node_id)
{
  EXO_LOG("[DISC] adv n=%u ad=%02X:%02X:%02X:%02X:%02X:%02X r=%d t=%02X l=%u\r\n",
          (unsigned)node_id,
          (unsigned)report->Address[5],
          (unsigned)report->Address[4],
          (unsigned)report->Address[3],
          (unsigned)report->Address[2],
          (unsigned)report->Address[1],
          (unsigned)report->Address[0],
          (int)(int8_t)report->RSSI,
          (unsigned)report->Event_Type,
          (unsigned)report->Length_Data);
}

static uint8_t exo_extract_leaf_name(const Advertising_Report_t *report, uint8_t *node_id_out)
{
  uint8_t offset = 0U;
  if (report == 0 || node_id_out == 0 || report->Data == 0 || report->Length_Data > 31U)
  {
    return 0U;
  }
  while ((uint16_t)(offset + 1U) < report->Length_Data)
  {
    const uint8_t field_len = report->Data[offset];
    uint8_t field_type;
    if (field_len == 0U)
    {
      break;
    }
    if ((uint16_t)offset + (uint16_t)field_len >= (uint16_t)report->Length_Data)
    {
      EXO_LOG("[BLE][HUB][DISC] adv malformed offset=%u field_len=%u len=%u\r\n",
              (unsigned)offset,
              (unsigned)field_len,
              (unsigned)report->Length_Data);
      break;
    }
    field_type = report->Data[offset + 1U];
    if ((field_type == EXO_ADV_TYPE_COMPLETE_NAME || field_type == EXO_ADV_TYPE_SHORT_NAME) &&
        field_len >= 2U)
    {
      const uint8_t id = exo_parse_leaf_name_id(&report->Data[offset + 2U], (uint8_t)(field_len - 1U));
      if (id != 0U)
      {
        *node_id_out = id;
        return 1U;
      }
    }
    offset = (uint8_t)(offset + field_len + 1U);
  }
  return 0U;
}

static void exo_request_scan_if_needed(void)
{
  const uint32_t now = HAL_GetTick();
  const uint16_t scan_interval = exo_scan_interval_for_state();
  const uint16_t scan_window = exo_scan_window_for_state();
  tBleStatus status;
  if (g_ble_ready == 0U ||
      g_discovery_hold != 0U ||
      g_scan_active != 0U ||
      g_connect_busy != 0U ||
      g_discovery_active != 0U ||
      g_connect_after_scan_slot != 0xFFU ||
      g_scan_requested == 0U)
  {
    return;
  }
  if ((int32_t)(now - g_next_scan_after_ms) < 0)
  {
    return;
  }
  if (exo_ready_or_connecting_leaf_count() >= EXO_HUB_LEAF_MAX)
  {
    EXO_LOG("[BLE][HUB][DISC] scan skipped ready_or_connecting=%u/%u\r\n",
            (unsigned)exo_ready_or_connecting_leaf_count(),
            (unsigned)EXO_HUB_LEAF_MAX);
    g_scan_requested = 0U;
    APP_BLE_LeafClientScanIdle();
    return;
  }
  if (APP_BLE_LeafClientPrepareScan() == 0U)
  {
    EXO_LOG("[BLE][HUB][DISC] scan prepare failed ready=%u busy=%u discovery=%u pending=%u\r\n",
            (unsigned)g_ble_ready,
            (unsigned)g_connect_busy,
            (unsigned)g_discovery_active,
            (unsigned)g_connect_after_scan_slot);
    g_next_scan_after_ms = now + EXO_HUB_SCAN_RETRY_MS;
    exo_send_disc_report(EXO_DISC_EVT_SCAN_DELAYED,
                         0U,
                         0xFFU,
                         (uint8_t)g_discovery_active,
                         (uint16_t)EXO_HUB_SCAN_RETRY_MS);
    return;
  }
  EXO_LOG("[BLE][HUB][DISC] scan start gen-disc iv=0x%04X win=0x%04X rdy=%u pend=%u\r\n",
          (unsigned)scan_interval,
          (unsigned)scan_window,
          (unsigned)g_ble_ready,
          (unsigned)g_connect_after_scan_slot);
  exo_send_disc_report(EXO_DISC_EVT_SCAN_REQUESTED,
                       0U,
                       0xFFU,
                       (uint8_t)exo_active_leaf_count(),
                       scan_window);
  if (APP_BLE_LeafClientPhoneConnected() != 0U)
  {
    EXO_LOG("[BLE][HUB][DISC] scan held (phone conn) act=%u rdy=%u xport_m=0x%02X\r\n",
            (unsigned)exo_active_leaf_count(),
            (unsigned)exo_ready_leaf_count(),
            (unsigned)exo_hub_central_client_transport_ready_node_mask());
  }
  status = aci_gap_start_general_discovery_proc(scan_interval,
                                                scan_window,
                                                CFG_BLE_ADDRESS_TYPE,
                                                1U);
  if (status == BLE_STATUS_SUCCESS)
  {
    EXO_LOG("[BLE][HUB][DISC] scan started\r\n");
    g_scan_active = 1U;
    g_scan_proc_code = EXO_HUB_PROC_GENERAL_DISCOVERY;
    g_scan_started_ms = now;
    g_scan_timeout_stop = 0U;
    exo_send_disc_report(EXO_DISC_EVT_SCAN_STARTED,
                         0U,
                         0xFFU,
                         (uint8_t)exo_active_leaf_count(),
                         scan_window);
  }
  else
  {
    const uint32_t retry_ms = exo_scan_retry_ms_for_status(status);
    EXO_LOG("[BLE][HUB][DISC] scan start fail st=%u act=%u rdy=%u xport_m=0x%02X\r\n",
            (unsigned)status,
            (unsigned)exo_active_leaf_count(),
            (unsigned)exo_ready_leaf_count(),
            (unsigned)exo_hub_central_client_transport_ready_node_mask());
    EXO_LOG("[BLE][HUB][DISC] scan retry delayed status=%u retry=%lums\r\n",
            (unsigned)status,
            (unsigned long)retry_ms);
    g_next_scan_after_ms = now + retry_ms;
    exo_send_disc_report(EXO_DISC_EVT_SCAN_FAILED,
                         0U,
                         0xFFU,
                         (uint8_t)status,
                         (uint16_t)retry_ms);
    APP_BLE_LeafClientScanIdle();
  }
}

static void exo_start_pending_connection(void)
{
  exo_leaf_slot_t *slot;
  tBleStatus status;
  const uint16_t scan_interval = exo_scan_interval_for_state();
  const uint16_t scan_window = exo_scan_window_for_state();
  const uint16_t conn_interval_min = exo_conn_interval_min_for_state();
  const uint16_t conn_interval_max = exo_conn_interval_max_for_state();
  if (g_connect_after_scan_slot == 0xFFU ||
      g_connect_busy != 0U ||
      g_discovery_active != 0U)
  {
    return;
  }
  slot = &g_leaf_slots[g_connect_after_scan_slot];
  EXO_LOG("[DISC] conn pend s=%u h=%u st=%u ad=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
          (unsigned)g_connect_after_scan_slot,
          (unsigned)slot->node_hint,
          (unsigned)slot->state,
          (unsigned)slot->addr[5],
          (unsigned)slot->addr[4],
          (unsigned)slot->addr[3],
          (unsigned)slot->addr[2],
          (unsigned)slot->addr[1],
          (unsigned)slot->addr[0]);
  if (slot->state == EXO_LEAF_SLOT_READY ||
      slot->state == EXO_LEAF_SLOT_CONNECTING ||
      slot->state == EXO_LEAF_SLOT_EMPTY)
  {
    EXO_LOG("[BLE][HUB][DISC] connect skipped slot=%u state=%u\r\n",
            (unsigned)g_connect_after_scan_slot,
            (unsigned)slot->state);
    g_connect_after_scan_slot = 0xFFU;
    APP_BLE_LeafClientScanIdle();
    return;
  }
  APP_BLE_LeafClientConnecting();
  if (conn_interval_min < EXO_HUB_CONN_INTERVAL_MIN_MULTI)
  {
    EXO_LOG("[DISC] WARN fast ivl N%u cn=%04X-%04X blocks multi-link\r\n",
            (unsigned)exo_leaf_slot_node_id(slot),
            (unsigned)conn_interval_min,
            (unsigned)conn_interval_max);
  }
  EXO_LOG("[DISC] pa N%u s=%u sc=%04X/%04X cn=%04X-%04X la=%u to=%04X ce=%04X/%04X\r\n",
          (unsigned)exo_leaf_slot_node_id(slot),
          (unsigned)g_connect_after_scan_slot,
          (unsigned)scan_interval,
          (unsigned)scan_window,
          (unsigned)conn_interval_min,
          (unsigned)conn_interval_max,
          (unsigned)EXO_HUB_CONN_LATENCY,
          (unsigned)EXO_HUB_SUPERVISION_TIMEOUT,
          (unsigned)EXO_HUB_MIN_CE_LENGTH,
          (unsigned)EXO_HUB_MAX_CE_LENGTH);
  EXO_LOG("[DISC] pb N%u rm=%02X xm=%02X a=%u rd=%u b=%u p=%u t=%lu\r\n",
          (unsigned)exo_leaf_slot_node_id(slot),
          (unsigned)exo_hub_central_client_ready_node_mask(),
          (unsigned)exo_hub_central_client_transport_ready_node_mask(),
          (unsigned)exo_active_leaf_count(),
          (unsigned)exo_ready_leaf_count(),
          (unsigned)g_connect_busy,
          (unsigned)g_pending_slot,
          (unsigned long)HAL_GetTick());
  status = aci_gap_create_connection(scan_interval,
                                     scan_window,
                                     slot->addr_type,
                                     slot->addr,
                                     CFG_BLE_ADDRESS_TYPE,
                                     conn_interval_min,
                                     conn_interval_max,
                                     EXO_HUB_CONN_LATENCY,
                                     EXO_HUB_SUPERVISION_TIMEOUT,
                                     EXO_HUB_MIN_CE_LENGTH,
                                     EXO_HUB_MAX_CE_LENGTH);
  if (status == BLE_STATUS_SUCCESS)
  {
    EXO_LOG("[BLE][HUB][DISC] connect start slot=%u node_hint=%u\r\n",
            (unsigned)g_connect_after_scan_slot,
            (unsigned)slot->node_hint);
    exo_send_disc_report(EXO_DISC_EVT_CONNECT_STARTED,
                         exo_leaf_slot_node_id(slot),
                         g_connect_after_scan_slot,
                         (uint8_t)slot->state,
                         scan_window);
    slot->state = EXO_LEAF_SLOT_CONNECTING;
    g_connect_busy = 1U;
    g_pending_slot = g_connect_after_scan_slot;
    g_connect_after_scan_slot = 0xFFU;
    g_connect_after_scan_ms = 0U;
  }
  else
  {
    const uint32_t retry_ms = exo_scan_retry_ms_for_status(status);
    EXO_LOG("[BLE][HUB][DISC] connect start failed slot=%u node_hint=%u status=%u\r\n",
            (unsigned)g_connect_after_scan_slot,
            (unsigned)slot->node_hint,
            (unsigned)status);
    EXO_LOG("[DISC] fail N%u s=%u st=%u/%02X %s r=%lu ce=%04X/%04X cn=%04X-%04X sc=%04X/%04X la=%u to=%04X rm=%02X xm=%02X a=%u rd=%u b=%u p=%u t=%lu\r\n",
            (unsigned)exo_leaf_slot_node_id(slot),
            (unsigned)g_connect_after_scan_slot,
            (unsigned)status,
            (unsigned)status,
            (status == BLE_STATUS_LENGTH_FAILED) ? "BLE_STATUS_LENGTH_FAILED" : "",
            (unsigned long)retry_ms,
            (unsigned)EXO_HUB_MIN_CE_LENGTH,
            (unsigned)EXO_HUB_MAX_CE_LENGTH,
            (unsigned)conn_interval_min,
            (unsigned)conn_interval_max,
            (unsigned)scan_interval,
            (unsigned)scan_window,
            (unsigned)EXO_HUB_CONN_LATENCY,
            (unsigned)EXO_HUB_SUPERVISION_TIMEOUT,
            (unsigned)exo_hub_central_client_ready_node_mask(),
            (unsigned)exo_hub_central_client_transport_ready_node_mask(),
            (unsigned)exo_active_leaf_count(),
            (unsigned)exo_ready_leaf_count(),
            (unsigned)g_connect_busy,
            (unsigned)g_pending_slot,
            (unsigned long)HAL_GetTick());
    exo_send_disc_report(EXO_DISC_EVT_CONNECT_FAILED,
                         exo_leaf_slot_node_id(slot),
                         g_connect_after_scan_slot,
                         (uint8_t)status,
                         (uint16_t)retry_ms);
    g_connect_after_scan_slot = 0xFFU;
    g_connect_after_scan_ms = 0U;
    exo_leaf_slot_mark_backoff(slot);
    slot->retry_after_ms = HAL_GetTick() + retry_ms;
    g_scan_requested = 1U;
    g_next_scan_after_ms = HAL_GetTick() + retry_ms;
    if (g_discovery_hold != 0U &&
        g_targeted_reconnect_attempts < EXO_HUB_TARGETED_RECONNECT_MAX_ATTEMPTS)
    {
      /* The targeted attempt failed at the radio; retry after the status
       * backoff while the session hold is still active. */
      g_targeted_reconnect_node_id = exo_leaf_slot_node_id(slot);
      g_targeted_reconnect_after_ms = HAL_GetTick() + retry_ms;
    }
    APP_BLE_LeafClientConnectIdle();
    APP_BLE_LeafClientScanIdle();
  }
}

static void exo_begin_service_discovery(exo_leaf_slot_t *slot)
{
  if (slot == 0)
  {
    return;
  }
  exo_leaf_slot_reset_handles(slot);
  slot->state = EXO_LEAF_SLOT_DISCOVER_SERVICE;
  g_discovery_active = 1U;
  if (aci_gatt_disc_all_primary_services(slot->connection_handle) != BLE_STATUS_SUCCESS)
  {
    exo_leaf_slot_mark_backoff(slot);
    g_scan_requested = 1U;
  }
}

static void exo_begin_mtu_exchange(exo_leaf_slot_t *slot)
{
  tBleStatus status;
  if (slot == 0)
  {
    return;
  }
  slot->state = EXO_LEAF_SLOT_EXCHANGE_MTU;
  g_discovery_active = 1U;
  status = aci_gatt_exchange_config(slot->connection_handle);
  EXO_LOG("[BLE][HUB][DISC] mtu exchange start slot=%u handle=0x%04X status=0x%02X\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)slot->connection_handle,
          (unsigned)status);
  if (status != BLE_STATUS_SUCCESS)
  {
    exo_begin_service_discovery(slot);
  }
}

static void exo_begin_char_discovery(exo_leaf_slot_t *slot)
{
  if (slot == 0)
  {
    return;
  }
  slot->state = EXO_LEAF_SLOT_DISCOVER_CHARS;
  if (aci_gatt_disc_all_char_of_service(slot->connection_handle,
                                        slot->service_start_handle,
                                        slot->service_end_handle) != BLE_STATUS_SUCCESS)
  {
    exo_leaf_slot_mark_backoff(slot);
    g_scan_requested = 1U;
  }
}

static void exo_enable_notify_step(exo_leaf_slot_t *slot)
{
  const uint8_t cccd_value[2] = { 0x01U, 0x00U };
  uint16_t cccd_handle = 0U;
  if (slot == 0)
  {
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_ENABLE_CTRL_NOTIFY && slot->ctrl_tx_value_handle != 0U)
  {
    cccd_handle = (uint16_t)(slot->ctrl_tx_value_handle + 1U);
  }
  else if (slot->state == EXO_LEAF_SLOT_ENABLE_STATUS_NOTIFY && slot->status_value_handle != 0U)
  {
    cccd_handle = (uint16_t)(slot->status_value_handle + 1U);
  }
  else if (slot->state == EXO_LEAF_SLOT_ENABLE_DATA_NOTIFY && slot->data_value_handle != 0U)
  {
    cccd_handle = (uint16_t)(slot->data_value_handle + 1U);
  }
  if (cccd_handle == 0U ||
      aci_gatt_write_char_value(slot->connection_handle,
                                cccd_handle,
                                (uint8_t)sizeof(cccd_value),
                                cccd_value) != BLE_STATUS_SUCCESS)
  {
    exo_leaf_slot_mark_backoff(slot);
    g_scan_requested = 1U;
  }
}

static uint8_t exo_send_blepipe_packet(exo_leaf_slot_t *slot,
                                       uint8_t msg_type,
                                       uint16_t src_id,
                                       uint16_t dst_id,
                                       const uint8_t *payload,
                                       uint16_t payload_len)
{
  uint8_t packet[BLEPIPE_MAX_NOTIFY_PAYLOAD];
  size_t encoded_len = 0U;
  blepipe_hdr_t hdr;
  tBleStatus status;
  if (slot == 0 || slot->state != EXO_LEAF_SLOT_READY || slot->ctrl_rx_value_handle == 0U)
  {
    EXO_LOG("[BLE][HUB][TX] leaf not ready msg=0x%02X dst=0x%04X len=%u\r\n",
            (unsigned)msg_type,
            (unsigned)dst_id,
            (unsigned)payload_len);
    return 0U;
  }
  memset(&hdr, 0, sizeof(hdr));
  hdr.proto_ver = BLEPIPE_PROTO_VER;
  hdr.msg_type = msg_type;
  hdr.src_id = src_id;
  hdr.dst_id = dst_id;
  hdr.timestamp_ms = HAL_GetTick();
  hdr.payload_len = payload_len;
  if (blepipe_encode(packet, sizeof(packet), &hdr, payload, payload_len, &encoded_len) != BLEPIPE_STATUS_OK ||
      encoded_len > 244U)
  {
    EXO_LOG("[BLE][HUB][TX] encode failed msg=0x%02X dst=0x%04X payload=%u encoded=%u\r\n",
            (unsigned)msg_type,
            (unsigned)dst_id,
            (unsigned)payload_len,
            (unsigned)encoded_len);
    return 0U;
  }
  status = aci_gatt_write_without_resp(slot->connection_handle,
                                       slot->ctrl_rx_value_handle,
                                       (uint8_t)encoded_len,
                                       packet);
  EXO_LOG("[BLE][HUB][TX] node=%u msg=0x%02X dst=0x%04X handle=0x%04X len=%u status=0x%02X\r\n",
          (unsigned)(slot->node_id != 0U ? slot->node_id : slot->node_hint),
          (unsigned)msg_type,
          (unsigned)dst_id,
          (unsigned)slot->ctrl_rx_value_handle,
          (unsigned)encoded_len,
          (unsigned)status);
  if (status != BLE_STATUS_SUCCESS)
  {
    const tBleStatus retry_status = aci_gatt_write_char_value(slot->connection_handle,
                                                              slot->ctrl_rx_value_handle,
                                                              (uint8_t)encoded_len,
                                                              packet);
    EXO_LOG("[TX] retry n=%u m=%02X d=%04X h=%04X l=%u s=%02X\r\n",
            (unsigned)(slot->node_id != 0U ? slot->node_id : slot->node_hint),
            (unsigned)msg_type,
            (unsigned)dst_id,
            (unsigned)slot->ctrl_rx_value_handle,
            (unsigned)encoded_len,
            (unsigned)retry_status);
    status = retry_status;
  }
  return (uint8_t)(status == BLE_STATUS_SUCCESS);
}

static void exo_touch_node_from_pipe(exo_leaf_slot_t *slot, const blepipe_hdr_t *hdr)
{
  if (slot == 0 || hdr == 0)
  {
    return;
  }
  if (hdr->src_id != 0U && hdr->src_id <= 0x00FFU)
  {
    slot->node_id = (uint8_t)hdr->src_id;
    exo_hub_leaf_topology_touch(slot->node_id);
  }
  else if (slot->node_hint != 0U)
  {
    exo_hub_leaf_topology_touch(slot->node_hint);
  }
}

static uint8_t exo_hub_maybe_queue_record_done(const uint8_t *payload,
                                               uint16_t length,
                                               const char *reason)
{
  ExoRecordDoneWire done;
  uint8_t accepted;

  if (payload == 0 || length < sizeof(done) || payload[0] != 0x02U)
  {
    return 0U;
  }

  memcpy(&done, payload, sizeof(done));
  accepted = exo_hub_leaf_record_done_ingest(payload, length);
  if (accepted != 0U)
  {
    EXO_LOG("[LEAF] done queued r=%s n=%u s=%lu sz=%lu c=%08lX\r\n",
            reason,
            (unsigned)done.node_id,
            (unsigned long)done.session_id,
            (unsigned long)done.total_size,
            (unsigned long)done.payload_crc32);
    return 1U;
  }

  EXO_LOG("[LEAF] done qfail fwd r=%s n=%u s=%lu sz=%lu\r\n",
          reason,
          (unsigned)done.node_id,
          (unsigned long)done.session_id,
          (unsigned long)done.total_size);
  return 0U;
}

static uint8_t exo_is_raw_artifact_frame(const uint8_t *payload, uint16_t length)
{
  exo::RecordReliableFrameHeader header;
  if (payload == 0 || length == 0U)
  {
    return 0U;
  }
  if (payload[0] == static_cast<uint8_t>(exo::RecordCommand::RecordDone))
  {
    return 1U;
  }
  if (length < sizeof(header))
  {
    return 0U;
  }
  memcpy(&header, payload, sizeof(header));
  if (header.command != exo::RecordCommand::ReliableFrame ||
      header.proto_version != exo::kRecordReliableProtoVersion ||
      header.magic != exo::kRecordReliableMagic)
  {
    return 0U;
  }
  return (header.frame_type == static_cast<uint8_t>(exo::RecordReliableType::Manifest) ||
          header.frame_type == static_cast<uint8_t>(exo::RecordReliableType::Chunk)) ? 1U : 0U;
}

static uint8_t exo_suppress_raw_artifact_relay(uint8_t node_id,
                                                const uint8_t *payload,
                                                uint16_t length)
{
  if (node_id == 0U || exo_is_raw_artifact_frame(payload, length) == 0U ||
      exo_master_training_owns_node_link(node_id) == 0U ||
      exo_master_training_raw_download_debug_enabled() != 0U)
  {
    return 0U;
  }
  exo_master_training_note_suppressed_relay();
  return 1U;
}

static void exo_clear_app_record_ready_mask(uint8_t mask)
{
  uint8_t i;
  if (mask != 0U)
  {
    g_last_logged_all_ready_mask = 0U;
  }
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    const uint8_t node_id = exo_leaf_slot_node_id(&g_leaf_slots[i]);
    if (node_id != 0U && node_id < 8U &&
        (mask & (uint8_t)(1U << node_id)) != 0U)
    {
      g_leaf_slots[i].app_record_ready = 0U;
      g_leaf_slots[i].app_recorder_state = 0U;
      g_leaf_slots[i].app_record_session_id = 0U;
    }
  }
}

static void exo_handle_leaf_status(exo_leaf_slot_t *slot,
                                   const blepipe_hdr_t *hdr,
                                   const uint8_t *payload,
                                   uint16_t payload_len)
{
  blepipe_node_record_ready_status_t status;
  uint8_t status_node_id;
  if (slot == 0 || hdr == 0 || payload == 0 ||
      hdr->msg_type != BLEPIPE_MSG_STATUS ||
      payload_len < sizeof(status))
  {
    return;
  }
  memcpy(&status, payload, sizeof(status));
  if (status.status_kind != BLEPIPE_STATUS_KIND_NODE_RECORD_READY)
  {
    return;
  }
  status_node_id = status.node_id != 0U ? status.node_id : exo_leaf_slot_node_id(slot);
  if (status_node_id == 0U || status_node_id >= 8U)
  {
    return;
  }
  if (hdr->src_id != 0U && hdr->src_id <= 0x00FFU &&
      (uint8_t)hdr->src_id != status_node_id)
  {
    return;
  }
  const uint8_t previous_ready = slot->app_record_ready;
  const uint8_t previous_state = slot->app_recorder_state;
  const uint32_t previous_session = slot->app_record_session_id;
  slot->node_id = status_node_id;
  slot->app_record_ready = status.record_ready != 0U ? 1U : 0U;
  slot->app_recorder_state = status.recorder_state;
  slot->app_record_session_id = status.session_id;
  slot->app_maximum_duration_ms = status.maximum_duration_ms;
  exo_hub_leaf_topology_touch(status_node_id);
  if (previous_ready != slot->app_record_ready ||
      previous_state != slot->app_recorder_state ||
      previous_session != slot->app_record_session_id)
  {
    EXO_LOG("[READY][MASTER] received node=%u ready=%u state=%u session=%lu\r\n",
            (unsigned)status_node_id,
            (unsigned)slot->app_record_ready,
            (unsigned)slot->app_recorder_state,
            (unsigned long)slot->app_record_session_id);
  }
  const uint8_t ready_mask = exo_hub_central_client_ready_node_mask();
  const uint8_t transport_mask = exo_hub_central_client_transport_ready_node_mask();
  if (transport_mask != 0U && ready_mask == transport_mask)
  {
    if (g_last_logged_all_ready_mask != ready_mask)
    {
      g_last_logged_all_ready_mask = ready_mask;
      EXO_LOG("[READY][MASTER] all devices ready mask=0x%02X count=%u\r\n",
              (unsigned)ready_mask,
              (unsigned)exo_hub_central_client_ready_node_count());
    }
  }
  else
  {
    g_last_logged_all_ready_mask = 0U;
  }
  EXO_LOG("[BLE][HUB][LEAF] app-ready slot=%u node=%u ready=%u state=%u session=%lu\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)status_node_id,
          (unsigned)slot->app_record_ready,
          (unsigned)slot->app_recorder_state,
          (unsigned long)slot->app_record_session_id);
}

static void exo_handle_pipe_packet(exo_leaf_slot_t *slot,
                                   uint8_t lane_kind,
                                   const uint8_t *data,
                                   uint8_t length)
{
  blepipe_hdr_t hdr;
  const uint8_t *payload = 0;
  uint16_t payload_len = 0U;
  blepipe_status_t decode_status;
  uint8_t decoded_frame_len;
  if (slot == 0 || data == 0 || length == 0U)
  {
    return;
  }
  decoded_frame_len = length;
  decode_status = blepipe_decode(data, length, &hdr, &payload, &payload_len);
  if ((decode_status == BLEPIPE_STATUS_BAD_LENGTH) &&
      ((lane_kind == BLEPIPE_LANE_DATA_TX) ||
       (lane_kind == BLEPIPE_LANE_CONTROL_TX) ||
       (lane_kind == BLEPIPE_LANE_STATUS_TX)) &&
      (length >= (BLEPIPE_HDR_LEN + BLEPIPE_CRC_LEN)) &&
      (data[0] == BLEPIPE_PROTO_VER))
  {
    uint16_t declared_len;
    uint16_t expected_len;

    declared_len = (uint16_t)data[16] | ((uint16_t)data[17] << 8);
    expected_len = (uint16_t)(BLEPIPE_HDR_LEN + declared_len + BLEPIPE_CRC_LEN);
    if ((expected_len <= (uint16_t)length) && (expected_len <= 255U))
    {
      blepipe_status_t trim_status;

      trim_status = blepipe_decode(data, (uint8_t)expected_len, &hdr, &payload, &payload_len);
      if (trim_status == BLEPIPE_STATUS_OK)
      {
        EXO_LOG("[BLE][HUB][LEAF] decode_trim lane=%u slot=%u len=%u used=%u declared=%u\r\n",
                (unsigned)lane_kind,
                (unsigned)(slot - &g_leaf_slots[0]),
                (unsigned)length,
                (unsigned)expected_len,
                (unsigned)declared_len);
        decode_status = trim_status;
        decoded_frame_len = (uint8_t)expected_len;
      }
    }
  }
  if (decode_status != BLEPIPE_STATUS_OK)
  {
    if (lane_kind == BLEPIPE_LANE_DATA_TX)
    {
      EXO_LOG("[BLE][HUB][LEAF] raw_forward decode_fail slot=%u status=%u len=%u first=0x%02X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)decode_status,
              (unsigned)length,
              (unsigned)data[0]);
      exo_hub_leaf_record_frame_ingest(exo_leaf_slot_node_id(slot), data, length);
      (void)exo_hub_maybe_queue_record_done(data, length, "decode_fail");
      if (exo_suppress_raw_artifact_relay(exo_leaf_slot_node_id(slot), data, length) != 0U)
      {
        return;
      }
      (void)Custom_APP_SendRecordFrame(data, length);
    }
    return;
  }

  exo_touch_node_from_pipe(slot, &hdr);

  if (lane_kind == BLEPIPE_LANE_CONTROL_TX)
  {
    exo_hub_leaf_control_ingest(exo_leaf_slot_node_id(slot), hdr.msg_type, payload, payload_len);
    (void)Custom_APP_SendCmdNotify(data, decoded_frame_len);
    return;
  }
  if (lane_kind == BLEPIPE_LANE_STATUS_TX)
  {
    if (hdr.msg_type == BLEPIPE_MSG_LINK_STATS && payload_len >= 89U && payload[0] == 1U)
    {
      /* Version-1 frame, v2 live-preview tail (bytes 65..88). */
      const uint8_t node = exo_leaf_slot_node_id(slot);
      if (node >= 1U && node <= EXO_HUB_LEAF_MAX)
      {
        exo_leaf_live_diag_t *const d = &g_leaf_live_diag[node - 1U];
        d->offered  = (uint32_t)payload[65] | ((uint32_t)payload[66] << 8U) |
                      ((uint32_t)payload[67] << 16U) | ((uint32_t)payload[68] << 24U);
        d->dropped  = (uint32_t)payload[69] | ((uint32_t)payload[70] << 8U) |
                      ((uint32_t)payload[71] << 16U) | ((uint32_t)payload[72] << 24U);
        d->sent     = (uint32_t)payload[73] | ((uint32_t)payload[74] << 8U) |
                      ((uint32_t)payload[75] << 16U) | ((uint32_t)payload[76] << 24U);
        d->gate_wdog = (uint32_t)payload[77] | ((uint32_t)payload[78] << 8U) |
                       ((uint32_t)payload[79] << 16U) | ((uint32_t)payload[80] << 24U);
        d->gate_bp  = (uint32_t)payload[81] | ((uint32_t)payload[82] << 8U) |
                      ((uint32_t)payload[83] << 16U) | ((uint32_t)payload[84] << 24U);
        d->dle_tx_octets = (uint16_t)payload[7] | ((uint16_t)payload[8] << 8U);
        d->stream_on = payload[85];
        if (payload_len >= 97U)
        {
          d->bno_fresh = (uint32_t)payload[89] | ((uint32_t)payload[90] << 8U) |
                         ((uint32_t)payload[91] << 16U) | ((uint32_t)payload[92] << 24U);
          d->icm_fresh = (uint32_t)payload[93] | ((uint32_t)payload[94] << 8U) |
                         ((uint32_t)payload[95] << 16U) | ((uint32_t)payload[96] << 24U);
        }
        d->valid = 1U;
      }
    }
    if (hdr.msg_type == BLEPIPE_MSG_LINK_STATS && payload_len >= 65U && payload[0] == 1U)
    {
      /* Full-length version-1 payload: everything the 57-byte frame carried,
       * plus the node's CPU2 wireless stack identity (bytes 57..63). */
      const uint16_t max_pool = (uint16_t)payload[49] | ((uint16_t)payload[50] << 8U);
      const uint16_t streak = (uint16_t)payload[51] | ((uint16_t)payload[52] << 8U);
      const uint16_t max_streak = (uint16_t)payload[53] | ((uint16_t)payload[54] << 8U);
      EXO_LOG("[BLE][HUB][UPLOAD] node=%u max_pool=%u streak=%u max_streak=%u cfg_buffers=%u burst=%u cpu2_fw=%u.%u.%u b%u fus=%u.%u.%u\r\n",
              (unsigned)exo_leaf_slot_node_id(slot),
              (unsigned)max_pool,
              (unsigned)streak,
              (unsigned)max_streak,
              (unsigned)payload[55],
              (unsigned)payload[56],
              (unsigned)payload[57],
              (unsigned)payload[58],
              (unsigned)payload[59],
              (unsigned)payload[60],
              (unsigned)payload[61],
              (unsigned)payload[62],
              (unsigned)payload[63]);
    }
    else if (hdr.msg_type == BLEPIPE_MSG_LINK_STATS && payload_len >= 57U && payload[0] == 1U)
    {
      const uint16_t max_pool = (uint16_t)payload[49] | ((uint16_t)payload[50] << 8U);
      const uint16_t streak = (uint16_t)payload[51] | ((uint16_t)payload[52] << 8U);
      const uint16_t max_streak = (uint16_t)payload[53] | ((uint16_t)payload[54] << 8U);
      EXO_LOG("[BLE][HUB][UPLOAD] node=%u max_pool=%u streak=%u max_streak=%u cfg_buffers=%u burst=%u\r\n",
              (unsigned)exo_leaf_slot_node_id(slot),
              (unsigned)max_pool,
              (unsigned)streak,
              (unsigned)max_streak,
              (unsigned)payload[55],
              (unsigned)payload[56]);
    }
    exo_handle_leaf_status(slot, &hdr, payload, payload_len);
    (void)Custom_APP_SendRecoveryFrame(data, decoded_frame_len);
    return;
  }
  if (hdr.msg_type == BLEPIPE_MSG_LEAF_SAMPLE && payload_len >= 3U && payload[0] == 0x03U)
  {
    /* Bundled live frame: [0x03][bno_len][bno...][icm_len][icm...]. Split into
     * the two per-sensor ingests the rest of the pipeline already expects. */
    const uint8_t node = slot->node_id != 0U ? slot->node_id : slot->node_hint;
    const uint8_t bno_len = payload[1];
    uint16_t off = 2U;
    if (bno_len > 0U && (uint16_t)(off + bno_len) <= payload_len)
    {
      (void)exo_hub_leaf_stream_ingest(node, 1U, payload + off, bno_len);
    }
    off = (uint16_t)(off + bno_len);
    if (off < payload_len)
    {
      const uint8_t icm_len = payload[off];
      off = (uint16_t)(off + 1U);
      if (icm_len > 0U && (uint16_t)(off + icm_len) <= payload_len)
      {
        (void)exo_hub_leaf_stream_ingest(node, 2U, payload + off, icm_len);
      }
    }
    return;
  }
  if (hdr.msg_type == BLEPIPE_MSG_LEAF_SAMPLE && payload_len > 1U)
  {
    (void)exo_hub_leaf_stream_ingest(slot->node_id != 0U ? slot->node_id : slot->node_hint,
                                     payload[0],
                                     payload + 1U,
                                     (uint8_t)(payload_len - 1U));
    return;
  }
  if (hdr.msg_type == BLEPIPE_MSG_RAW_FORWARD && payload_len > 0U && payload_len <= 255U)
  {
#if EXO_HUB_VERBOSE_PIPE_LOGS
    EXO_LOG("[BLE][HUB][LEAF] raw_forward slot=%u src=0x%04X len=%u first=0x%02X status=%u\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)hdr.src_id,
            (unsigned)payload_len,
            (unsigned)payload[0],
            (unsigned)decode_status);
#endif
    exo_hub_leaf_record_frame_ingest(exo_leaf_slot_node_id(slot), payload, payload_len);
    (void)exo_hub_maybe_queue_record_done(payload, payload_len, "raw_forward");
    if (exo_suppress_raw_artifact_relay(exo_leaf_slot_node_id(slot), payload, payload_len) != 0U)
    {
      return;
    }
    (void)Custom_APP_SendRecordFrame(payload, (uint8_t)payload_len);
    return;
  }
}

void exo_hub_central_client_init(void)
{
  memset(g_leaf_slots, 0, sizeof(g_leaf_slots));
  g_link_tune = exo::LinkTuneState{};
  g_transfer_link_rearm = exo::TransferLinkRearmState{};
  g_scan_requested = 0U;
  g_scan_active = 0U;
  g_connect_busy = 0U;
  g_discovery_active = 0U;
  g_pending_slot = 0xFFU;
  g_ble_ready = 0U;
  g_connect_after_scan_slot = 0xFFU;
  g_next_scan_after_ms = 0U;
  g_connect_after_scan_ms = 0U;
  g_scan_started_ms = 0U;
  g_scan_timeout_stop = 0U;
  g_scan_proc_code = EXO_HUB_PROC_GENERAL_DISCOVERY;
  g_last_logged_all_ready_mask = 0U;
  g_discovery_hold = 0U;
  EXO_LOG("[BLE][HUB][DISC] init leaf_max=%u cfg_links=%u\r\n",
          (unsigned)EXO_HUB_LEAF_MAX,
          (unsigned)CFG_BLE_NUM_LINK);
}

static uint8_t exo_arm_active_transfer_slot(uint8_t slot_index)
{
  exo_leaf_slot_t *const slot = &g_leaf_slots[slot_index];
  const uint8_t node_id = exo_leaf_slot_node_id(slot);
  const exo::LinkTuneState::Telemetry &t = g_link_tune.telemetry(slot_index);
  if (!g_transfer_link_rearm.on_link_connected(node_id, t.generation))
  {
    return 0U;
  }
  return (uint8_t)g_link_tune.begin_fast_preparation(
      slot_index, t.generation, g_transfer_link_rearm.fast_interval());
}

/* Queue upload timing through the same completion-driven arbiter as DLE/PHY.
 * Only the selected source participates: making inactive-link scheduling a
 * prerequisite for initial credit can deadlock an otherwise healthy upload
 * when STM32WB rejects a concurrent interval update. */
void exo_hub_central_client_set_transfer_timing(uint8_t node_id, uint8_t fast,
                                                uint8_t fast_interval)
{
  uint8_t i;
  if (fast != 0U)
  {
    (void)g_transfer_link_rearm.begin_source(
        node_id, exo::RecordTransferTuningWire::kBulkFastInterval);
  }
  else
  {
    (void)g_transfer_link_rearm.end_source(node_id);
  }
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (exo_leaf_slot_node_id(&g_leaf_slots[i]) == node_id &&
        g_leaf_slots[i].connection_handle != 0xFFFFU)
    {
      const exo::LinkTuneState::Telemetry &t = g_link_tune.telemetry(i);
      const uint8_t queued = (uint8_t)(fast != 0U
          ? exo_arm_active_transfer_slot(i)
          : g_link_tune.begin_slow_restore(i, t.generation));
      EXO_LOG("[BLE][HUB][XFER] timing queue node=%u slot=%u fast=%u ci_cfg=%u ci_bulk=%u queued=%u gen=%lu\r\n",
              (unsigned)node_id, (unsigned)i, (unsigned)(fast != 0U),
              (unsigned)exo::RecordTransferTuningWire::sanitize_fast_interval(fast_interval),
              (unsigned)exo::RecordTransferTuningWire::kBulkFastInterval,
              (unsigned)queued, (unsigned long)t.generation);
      exo_report_link_tune(i);
      return;
    }
  }
}

uint8_t exo_hub_central_client_set_live_link_timing(uint8_t fast)
{
  uint8_t i;
  uint8_t queued = 0U;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    exo_leaf_slot_t *const slot = &g_leaf_slots[i];
    if (exo_leaf_slot_node_id(slot) == 0U ||
        slot->connection_handle == 0xFFFFU)
    {
      continue;
    }
    const exo::LinkTuneState::Telemetry &t = g_link_tune.telemetry(i);
    if (fast != 0U)
    {
      /* Idempotent: skip a leaf that has already settled at a fast-ish interval
       * (Ready or Degraded, <= 30 ms). Re-requesting a Degraded link every few
       * seconds just makes it thrash between NeedInterval/WaitInterval and
       * never rest - a stable 20 ms Degraded link delivers more than a link
       * stuck re-negotiating. Only a genuinely slow (>30 ms) or freshly
       * reconnected leaf is re-armed. */
      const bool settled_fast =
          (t.state == exo::LinkTuneState::State::Ready ||
           t.state == exo::LinkTuneState::State::Degraded) &&
          t.confirmed_interval != 0U &&
          t.confirmed_interval <= exo::LinkTuneState::kSlowIntervalMin;
      if (settled_fast)
      {
        continue;
      }
    }
    const bool ok = (fast != 0U)
        ? g_link_tune.begin_fast_preparation(i, t.generation)
        : g_link_tune.begin_slow_restore(i, t.generation);
    if (ok)
    {
      ++queued;
      exo_report_link_tune(i);
    }
  }
  if (queued != 0U)
  {
    EXO_LOG("[BLE][HUB][LIVE] link timing fast=%u queued=%u\r\n", (unsigned)fast, (unsigned)queued);
  }
  return queued;
}

uint8_t exo_hub_central_client_transfer_preparation_resolved(uint8_t node_id)
{
  uint8_t i;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    const exo_leaf_slot_t *const slot = &g_leaf_slots[i];
    if (exo_leaf_slot_node_id(slot) != node_id ||
        slot->connection_handle == 0xFFFFU)
    {
      continue;
    }
    const exo::LinkTuneState::Telemetry &t = g_link_tune.telemetry(i);
    return (uint8_t)g_transfer_link_rearm.preparation_resolved(
        node_id, t.generation,
        g_link_tune.transfer_preparation_resolved(i, t.generation));
  }
  return 0U;
}

static const exo::LinkTuneState::Telemetry *exo_leaf_link_telemetry_for(uint8_t node_id)
{
  for (uint8_t i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    const exo_leaf_slot_t *const slot = &g_leaf_slots[i];
    if (exo_leaf_slot_node_id(slot) == node_id &&
        slot->connection_handle != 0xFFFFU)
    {
      return &g_link_tune.telemetry(i);
    }
  }
  return nullptr;
}

/* SWO-free leaf-link telemetry, folded into the Master's live diag log line. */
uint16_t exo_hub_central_client_leaf_link_interval_raw(uint8_t node_id)
{
  const exo::LinkTuneState::Telemetry *const t = exo_leaf_link_telemetry_for(node_id);
  return t != nullptr ? t->confirmed_interval : 0U;
}

uint8_t exo_hub_central_client_leaf_link_state(uint8_t node_id)
{
  const exo::LinkTuneState::Telemetry *const t = exo_leaf_link_telemetry_for(node_id);
  return t != nullptr ? (uint8_t)t->state : 0xFFU;
}

uint8_t exo_hub_central_client_leaf_link_retries(uint8_t node_id)
{
  const exo::LinkTuneState::Telemetry *const t = exo_leaf_link_telemetry_for(node_id);
  return t != nullptr ? t->retries : 0U;
}

uint8_t exo_hub_central_client_leaf_link_tx_phy(uint8_t node_id)
{
  const exo::LinkTuneState::Telemetry *const t = exo_leaf_link_telemetry_for(node_id);
  return t != nullptr ? t->confirmed_tx_phy : 0U;
}

/* Live-preview forwarding health last reported by the node (LINK_STATS v2). */
void exo_hub_central_client_leaf_live_diag(uint8_t node_id, uint32_t *offered,
    uint32_t *dropped, uint32_t *sent, uint32_t *bno_fresh, uint32_t *icm_fresh,
    uint16_t *dle_tx_octets, uint8_t *stream_on)
{
  const exo_leaf_live_diag_t *d = (node_id >= 1U && node_id <= EXO_HUB_LEAF_MAX)
      ? &g_leaf_live_diag[node_id - 1U] : nullptr;
  static const exo_leaf_live_diag_t zero = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  if (d == nullptr || d->valid == 0U) { d = &zero; }
  if (offered) *offered = d->offered;
  if (dropped) *dropped = d->dropped;
  if (sent) *sent = d->sent;
  if (bno_fresh) *bno_fresh = d->bno_fresh;
  if (icm_fresh) *icm_fresh = d->icm_fresh;
  if (dle_tx_octets) *dle_tx_octets = d->dle_tx_octets;
  if (stream_on) *stream_on = d->stream_on;
}

void exo_hub_central_client_reset_leaf_live_diag(void)
{
  for (uint8_t i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    g_leaf_live_diag[i] = exo_leaf_live_diag_t{};
  }
}


void exo_hub_central_client_set_ble_ready(void)
{
  g_ble_ready = 1U;
  g_scan_requested = 1U;
  g_next_scan_after_ms = HAL_GetTick() + EXO_HUB_BLE_READY_SCAN_DELAY_MS;
  EXO_LOG("[BLE][HUB][DISC] ble-ready delay=%lums\r\n",
          (unsigned long)EXO_HUB_BLE_READY_SCAN_DELAY_MS);
}

void exo_hub_central_client_process(void)
{
  uint8_t i;
  const uint32_t now = HAL_GetTick();
  {
    const exo::LinkTuneState::Request active = g_link_tune.active_request();
    uint8_t resolved_by_readback = 0U;
    if (active.procedure == exo::LinkTuneState::Procedure::Phy &&
        g_link_tune.active_timeout_due(now))
    {
      uint8_t tx_phy = 0U;
      uint8_t rx_phy = 0U;
      const tBleStatus read_status = hci_le_read_phy(active.handle, &tx_phy, &rx_phy);
      if (read_status == BLE_STATUS_SUCCESS)
      {
        resolved_by_readback = (uint8_t)g_link_tune.on_phy_complete(
            active.handle, active.generation, exo::LinkTuneState::kStatusSuccess,
            tx_phy, rx_phy, now);
        EXO_LOG("[BLE][HUB][LINK] PHY completion missing; readback h=%04X st=0x%02X tx=%u rx=%u accepted=%u\r\n",
                (unsigned)active.handle, (unsigned)read_status,
                (unsigned)tx_phy, (unsigned)rx_phy,
                (unsigned)resolved_by_readback);
        if (resolved_by_readback != 0U)
        {
          exo_report_link_tune(active.link);
        }
      }
    }
    if (resolved_by_readback == 0U && g_link_tune.on_timeout(now))
    {
      EXO_LOG("[BLE][HUB][LINK] completion timeout slot=%u\r\n", (unsigned)active.link);
      exo_report_link_tune(active.link);
    }
  }
  /* The global model reserves at most one LL procedure. It remains serviced
   * during a discovery hold so an established source can finish commissioning
   * or restore its idle interval without reopening scan/connect work. */
  exo_issue_next_link_tune(now);
  if (g_discovery_hold != 0U)
  {
    /* Do not let scan/connect scheduler work compete with an established
     * recording or reliable Node->Master artifact transfer. Existing GATT
     * links and their notifications continue normally.
     * Exception: one dropped Node may be direct-connected by its stored
     * address so a session source can self-heal without general discovery. */
    if (g_targeted_reconnect_node_id != 0U &&
        (int32_t)(now - g_targeted_reconnect_after_ms) >= 0)
    {
      if (g_connect_busy != 0U || g_discovery_active != 0U || g_scan_active != 0U)
      {
        /* Radio still busy (e.g., a scan window opened before the hold):
         * keep the request armed and retry shortly rather than dropping it. */
        g_targeted_reconnect_after_ms = now + EXO_HUB_TARGETED_RECONNECT_DELAY_MS;
      }
      else
      {
        exo_leaf_slot_t *const slot = exo_find_slot_by_node(g_targeted_reconnect_node_id);
        g_targeted_reconnect_node_id = 0U;
        if (slot != 0 &&
            (slot->state == EXO_LEAF_SLOT_BACKOFF ||
             slot->state == EXO_LEAF_SLOT_DISCOVERED) &&
            g_targeted_reconnect_attempts < EXO_HUB_TARGETED_RECONNECT_MAX_ATTEMPTS)
        {
          ++g_targeted_reconnect_attempts;
          slot->retry_after_ms = now;
          slot->state = EXO_LEAF_SLOT_DISCOVERED;
          g_connect_after_scan_slot = (uint8_t)(slot - &g_leaf_slots[0]);
          g_connect_after_scan_ms = now;
          EXO_LOG("[BLE][HUB][DISC] targeted reconnect arm slot=%u node=%u attempt=%u\r\n",
                  (unsigned)(slot - &g_leaf_slots[0]),
                  (unsigned)exo_leaf_slot_node_id(slot),
                  (unsigned)g_targeted_reconnect_attempts);
          exo_start_pending_connection();
        }
      }
    }
    return;
  }
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_BACKOFF &&
        (int32_t)(now - g_leaf_slots[i].retry_after_ms) >= 0)
    {
      g_leaf_slots[i].state = EXO_LEAF_SLOT_DISCOVERED;
    }
  }
  if (g_connect_after_scan_slot == 0xFFU &&
      g_scan_active == 0U &&
      g_connect_busy == 0U &&
      g_discovery_active == 0U)
  {
    const uint8_t next_slot = exo_find_next_connectable_slot();
    if (next_slot != 0xFFU)
    {
      g_connect_after_scan_slot = next_slot;
      g_connect_after_scan_ms = now;
      EXO_LOG("[BLE][HUB][DISC] connect queued from discovered slot=%u node_hint=%u\r\n",
              (unsigned)next_slot,
              (unsigned)g_leaf_slots[next_slot].node_hint);
      exo_send_disc_report(EXO_DISC_EVT_CONNECT_QUEUED,
                           exo_leaf_slot_node_id(&g_leaf_slots[next_slot]),
                           next_slot,
                           (uint8_t)g_leaf_slots[next_slot].state,
                           0U);
    }
  }
  if (g_connect_after_scan_slot != 0xFFU &&
      g_scan_active == 0U &&
      (int32_t)(now - g_connect_after_scan_ms) >= 0)
  {
    EXO_LOG("[BLE][HUB][DISC] connect-after-scan armed slot=%u\r\n",
            (unsigned)g_connect_after_scan_slot);
    exo_start_pending_connection();
  }
  if (g_scan_active != 0U &&
      g_scan_timeout_stop == 0U &&
      (int32_t)(now - (g_scan_started_ms + EXO_HUB_SCAN_WINDOW_MS)) >= 0)
  {
    g_scan_timeout_stop = 1U;
    EXO_LOG("[BLE][HUB][DISC] scan window expired ready=%u/%u\r\n",
            (unsigned)exo_ready_leaf_count(),
            (unsigned)EXO_HUB_LEAF_MAX);
    if (aci_gap_terminate_gap_proc(g_scan_proc_code) != BLE_STATUS_SUCCESS)
    {
      g_scan_active = 0U;
      g_scan_timeout_stop = 0U;
      g_scan_requested = 1U;
      g_next_scan_after_ms = now + EXO_HUB_SCAN_BUSY_RETRY_MS;
      APP_BLE_LeafClientScanIdle();
      EXO_LOG("[BLE][HUB][DISC] scan timeout terminate failed retry=%lums\r\n",
              (unsigned long)EXO_HUB_SCAN_BUSY_RETRY_MS);
      exo_send_disc_report(EXO_DISC_EVT_SCAN_DELAYED,
                           0U,
                           0xFFU,
                           (uint8_t)BLE_STATUS_LENGTH_FAILED,
                           (uint16_t)EXO_HUB_SCAN_BUSY_RETRY_MS);
    }
  }
  exo_request_scan_if_needed();
}

void exo_hub_central_client_request_scan(void)
{
  uint8_t i;
  g_scan_requested = 1U;
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    g_leaf_slots[i].seen_in_scan = 0U;
  }
}

uint8_t exo_hub_central_client_broadcast_blepipe(uint8_t msg_type,
                                                 uint16_t src_id,
                                                 const uint8_t *payload,
                                                 uint16_t payload_len)
{
  return (uint8_t)(exo_hub_central_client_broadcast_blepipe_mask(msg_type,
                                                                 src_id,
                                                                 payload,
                                                                 payload_len) != 0U);
}

uint8_t exo_hub_central_client_broadcast_blepipe_mask(uint8_t msg_type,
                                                      uint16_t src_id,
                                                      const uint8_t *payload,
                                                      uint16_t payload_len)
{
  uint8_t sent_mask = 0U;
  uint8_t i;
  const uint8_t clears_record_ready =
      (uint8_t)((msg_type == BLEPIPE_MSG_COMMAND &&
                 payload != 0 &&
                 payload_len >= 1U &&
                 payload[0] == 0xB3U) ? 1U : 0U);
  const uint8_t requires_record_ready =
      (uint8_t)((msg_type == BLEPIPE_MSG_COMMAND &&
                 payload != 0 &&
                 payload_len >= 1U &&
                 (payload[0] == 0x01U ||
                  payload[0] == 0x0BU)) ? 1U : 0U);
  for (i = 0U; i < EXO_HUB_LEAF_MAX; ++i)
  {
    if (g_leaf_slots[i].state == EXO_LEAF_SLOT_READY &&
        (requires_record_ready == 0U || g_leaf_slots[i].app_record_ready != 0U))
    {
      const uint8_t node_id = exo_leaf_slot_node_id(&g_leaf_slots[i]);
      const uint8_t sent = exo_send_blepipe_packet(&g_leaf_slots[i],
                                                   msg_type,
                                                   src_id,
                                                   node_id,
                                                   payload,
                                                   payload_len);
      if ((sent != 0U) && (node_id != 0U) && (node_id < 8U))
      {
        sent_mask = (uint8_t)(sent_mask | (uint8_t)(1U << node_id));
      }
    }
  }
  if (clears_record_ready != 0U)
  {
    exo_clear_app_record_ready_mask(sent_mask);
  }
  return sent_mask;
}

uint8_t exo_hub_central_client_send_blepipe_to_node(uint8_t node_id,
                                                    uint8_t msg_type,
                                                    uint16_t src_id,
                                                    const uint8_t *payload,
                                                    uint16_t payload_len)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_node(node_id);
  uint8_t sent;
  if (slot == 0)
  {
    return 0U;
  }
  sent = exo_send_blepipe_packet(slot, msg_type, src_id, node_id, payload, payload_len);
  if (sent != 0U &&
      msg_type == BLEPIPE_MSG_COMMAND &&
      payload != 0 &&
      payload_len >= 1U &&
      payload[0] == 0xB3U &&
      node_id < 8U)
  {
    exo_clear_app_record_ready_mask((uint8_t)(1U << node_id));
  }
  return sent;
}

uint8_t exo_hub_central_client_send_raw_to_node(uint8_t node_id,
                                                const uint8_t *payload,
                                                uint16_t payload_len)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_node(node_id);
  if (slot == 0 || slot->state != EXO_LEAF_SLOT_READY || slot->ctrl_rx_value_handle == 0U || payload_len > 244U)
  {
    return 0U;
  }
  return (uint8_t)(aci_gatt_write_without_resp(slot->connection_handle,
                                               slot->ctrl_rx_value_handle,
                                               (uint8_t)payload_len,
                                               payload) == BLE_STATUS_SUCCESS);
}

void exo_hub_central_client_on_connection_complete(uint8_t initiated_as_client,
                                                   uint8_t status,
                                                   uint16_t connection_handle,
                                                   uint8_t peer_address_type,
                                                   const uint8_t *peer_address)
{
  exo_leaf_slot_t *slot;
  if (initiated_as_client == 0U)
  {
    return;
  }
  g_connect_busy = 0U;
  if (g_pending_slot == 0xFFU)
  {
    EXO_LOG("[BLE][HUB][DISC] connection ignored status=%u handle=0x%04X role=%u\r\n",
            (unsigned)status,
            (unsigned)connection_handle,
            (unsigned)initiated_as_client);
    return;
  }
  slot = &g_leaf_slots[g_pending_slot];
  g_pending_slot = 0xFFU;
  EXO_LOG("[DISC] conn done s=%u st=%u h=%04X pt=%u ad=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)status,
          (unsigned)connection_handle,
          (unsigned)peer_address_type,
          peer_address != 0 ? (unsigned)peer_address[5] : 0U,
          peer_address != 0 ? (unsigned)peer_address[4] : 0U,
          peer_address != 0 ? (unsigned)peer_address[3] : 0U,
          peer_address != 0 ? (unsigned)peer_address[2] : 0U,
          peer_address != 0 ? (unsigned)peer_address[1] : 0U,
          peer_address != 0 ? (unsigned)peer_address[0] : 0U);
  if (status != 0U)
  {
    const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
    exo_leaf_slot_mark_backoff(slot);
    g_scan_requested = 1U;
    EXO_LOG("[BLE][HUB][DISC] connection failed slot=%u backoff_until=%lu\r\n",
            (unsigned)slot_index,
            (unsigned long)slot->retry_after_ms);
    exo_send_disc_report(EXO_DISC_EVT_CONNECT_FAILED,
                         exo_leaf_slot_node_id(slot),
                         slot_index,
                         status,
                         (uint16_t)EXO_HUB_BACKOFF_MS);
    if (g_discovery_hold != 0U &&
        g_targeted_reconnect_attempts < EXO_HUB_TARGETED_RECONNECT_MAX_ATTEMPTS)
    {
      /* Asynchronous establishment failure (e.g., 0x3E failed-to-establish):
       * keep direct-address recovery alive across the backoff instead of
       * wedging the source until the hold lifts. */
      g_targeted_reconnect_node_id = exo_leaf_slot_node_id(slot);
      g_targeted_reconnect_after_ms = HAL_GetTick() + EXO_HUB_BACKOFF_MS;
    }
    return;
  }
  if (peer_address != 0)
  {
    if (memcmp(slot->addr, peer_address, 6U) != 0)
    {
      const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
      EXO_LOG("[DISC] addr mm s=%u h=%04X e=%02X:%02X:%02X:%02X:%02X:%02X g=%02X:%02X:%02X:%02X:%02X:%02X bo\r\n",
              (unsigned)slot_index,
              (unsigned)connection_handle,
              (unsigned)slot->addr[5],
              (unsigned)slot->addr[4],
              (unsigned)slot->addr[3],
              (unsigned)slot->addr[2],
              (unsigned)slot->addr[1],
              (unsigned)slot->addr[0],
              (unsigned)peer_address[5],
              (unsigned)peer_address[4],
              (unsigned)peer_address[3],
              (unsigned)peer_address[2],
              (unsigned)peer_address[1],
              (unsigned)peer_address[0]);
      exo_leaf_slot_mark_backoff(slot);
      g_scan_requested = 1U;
      exo_send_disc_report(EXO_DISC_EVT_CONNECT_FAILED,
                           exo_leaf_slot_node_id(slot),
                           slot_index,
                           HCI_CONNECTION_ALREADY_EXISTS_ERR_CODE,
                           (uint16_t)EXO_HUB_BACKOFF_MS);
      if (g_discovery_hold != 0U &&
          g_targeted_reconnect_attempts < EXO_HUB_TARGETED_RECONNECT_MAX_ATTEMPTS)
      {
        g_targeted_reconnect_node_id = exo_leaf_slot_node_id(slot);
        g_targeted_reconnect_after_ms = HAL_GetTick() + EXO_HUB_BACKOFF_MS;
      }
      return;
    }
    slot->addr_type = peer_address_type;
    memcpy(slot->addr, peer_address, 6U);
  }
  slot->connection_handle = connection_handle;
  /* A recovered link restores the source; give future drops a fresh attempt
   * budget and drop any stale armed timer. */
  g_targeted_reconnect_attempts = 0U;
  g_targeted_reconnect_node_id = 0U;
  /* LL requests stay out of this connection-complete event. The model starts
   * after the 600 ms feature-exchange settle delay and advances only after the
   * matching completion event, not after a guessed inter-command delay. */
  const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
  (void)g_link_tune.connect(slot_index, connection_handle, HAL_GetTick());
  if (g_transfer_link_rearm.active_node_id() == exo_leaf_slot_node_id(slot))
  {
    const uint8_t queued = exo_arm_active_transfer_slot(slot_index);
    EXO_LOG("[BLE][HUB][XFER] reconnect source=%u slot=%u queued=%u gen=%lu\r\n",
            (unsigned)exo_leaf_slot_node_id(slot), (unsigned)slot_index,
            (unsigned)queued,
            (unsigned long)g_link_tune.telemetry(slot_index).generation);
  }
  exo_report_link_tune(slot_index);
  exo_begin_mtu_exchange(slot);
}

void exo_hub_central_client_on_disconnection_complete(uint16_t connection_handle,
                                                      uint8_t reason)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(connection_handle);
  (void)reason;
  if (slot == 0)
  {
    return;
  }
  (void)g_link_tune.disconnect((uint8_t)(slot - &g_leaf_slots[0]), connection_handle);
  exo_leaf_slot_mark_backoff(slot);
  g_scan_requested = 1U;
  if (g_discovery_hold != 0U)
  {
    /* Mid-session drop: general discovery is held, so arm direct-address
     * recovery for this source instead of waiting for the hold to lift. */
    exo_hub_central_client_request_targeted_reconnect(exo_leaf_slot_node_id(slot));
  }
}

void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     const Advertising_Report_t *Advertising_Report)
{
  uint8_t node_id = 0U;
  exo_leaf_slot_t *slot;
  exo_leaf_slot_t *logical_slot;
  (void)Num_Reports;
  if (Advertising_Report == 0 || g_discovery_hold != 0U ||
      g_scan_active == 0U || g_connect_busy != 0U)
  {
    return;
  }
  if (exo_extract_leaf_name(Advertising_Report, &node_id) == 0U)
  {
//    EXO_LOG("[BLE][HUB][DISC] adv ignored type=0x%02X len=%u rssi=%d\r\n",
//            (unsigned)Advertising_Report->Event_Type,
//            (unsigned)Advertising_Report->Length_Data,
//            (int)(int8_t)Advertising_Report->RSSI);
    return;
  }
  exo_log_adv_report(Advertising_Report, node_id);

  /* Logical Node ID is the routing identity. Never allow two BLE addresses to
   * occupy independent central slots for the same Node ID: masks collapse by
   * Node ID, while control writes otherwise pick whichever duplicate slot is
   * found first. That can route ACK/credit to the wrong physical link. */
  logical_slot = exo_find_slot_by_node(node_id);
  if (logical_slot != 0 &&
      (logical_slot->addr_type != Advertising_Report->Address_Type ||
       memcmp(logical_slot->addr, Advertising_Report->Address, 6U) != 0))
  {
    if (logical_slot->state == EXO_LEAF_SLOT_DISCOVERED ||
        logical_slot->state == EXO_LEAF_SLOT_BACKOFF)
    {
      EXO_LOG("[BLE][HUB][DISC] logical rebind node=%u slot=%u old_state=%u\r\n",
              (unsigned)node_id,
              (unsigned)(logical_slot - &g_leaf_slots[0]),
              (unsigned)logical_slot->state);
      exo_leaf_slot_reset_handles(logical_slot);
      logical_slot->state = EXO_LEAF_SLOT_DISCOVERED;
      logical_slot->addr_type = Advertising_Report->Address_Type;
      memcpy(logical_slot->addr, Advertising_Report->Address, 6U);
      logical_slot->connection_handle = 0xFFFFU;
      logical_slot->retry_after_ms = 0U;
      slot = logical_slot;
    }
    else
    {
      EXO_LOG("[BLE][HUB][DISC] dup logical node ign node=%u own_slot=%u own_state=%u\r\n",
              (unsigned)node_id,
              (unsigned)(logical_slot - &g_leaf_slots[0]),
              (unsigned)logical_slot->state);
      exo_send_disc_report(EXO_DISC_EVT_ADV_SKIPPED,
                           node_id,
                           (uint8_t)(logical_slot - &g_leaf_slots[0]),
                           (uint8_t)logical_slot->state,
                           0xD001U);
      return;
    }
  }
  else
  {
    slot = exo_claim_slot(Advertising_Report->Address_Type, Advertising_Report->Address);
  }
  if (slot == 0)
  {
    EXO_LOG("[BLE][HUB][DISC] adv claim failed node=%u\r\n", (unsigned)node_id);
    return;
  }
  slot->node_hint = node_id;
  slot->seen_in_scan = 1U;
  exo_hub_leaf_topology_touch(node_id);
  exo_send_disc_report(EXO_DISC_EVT_ADV_PARSED,
                       node_id,
                       (uint8_t)(slot - &g_leaf_slots[0]),
                       (uint8_t)slot->state,
                       (uint16_t)((uint8_t)Advertising_Report->RSSI));
  EXO_LOG("[BLE][HUB][DISC] leaf slot=%u hint=%u state=%u addr=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)slot->node_hint,
          (unsigned)slot->state,
          (unsigned)slot->addr[5],
          (unsigned)slot->addr[4],
          (unsigned)slot->addr[3],
          (unsigned)slot->addr[2],
          (unsigned)slot->addr[1],
          (unsigned)slot->addr[0]);
  if (slot->state == EXO_LEAF_SLOT_READY || slot->state == EXO_LEAF_SLOT_CONNECTING)
  {
    EXO_LOG("[BLE][HUB][DISC] adv skip node=%u slot=%u state=%u rdy/con=%u/%u\r\n",
            (unsigned)node_id,
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)slot->state,
            (unsigned)exo_ready_or_connecting_leaf_count(),
            (unsigned)EXO_HUB_LEAF_MAX);
    exo_send_disc_report(EXO_DISC_EVT_ADV_SKIPPED,
                         node_id,
                         (uint8_t)(slot - &g_leaf_slots[0]),
                         (uint8_t)slot->state,
                         (uint16_t)exo_ready_or_connecting_leaf_count());
    return;
  }
  if (exo_active_leaf_count() < EXO_HUB_LEAF_MAX)
  {
    EXO_LOG("[BLE][HUB][DISC] adv queued for later connect node=%u slot=%u active=%u/%u\r\n",
            (unsigned)node_id,
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)exo_active_leaf_count(),
            (unsigned)EXO_HUB_LEAF_MAX);
    return;
  }
  g_connect_after_scan_slot = exo_find_next_connectable_slot();
  if (g_connect_after_scan_slot == 0xFFU)
  {
    return;
  }
  g_connect_after_scan_ms = HAL_GetTick() + EXO_HUB_CONNECT_AFTER_SCAN_MS;
  EXO_LOG("[BLE][HUB][DISC] connect queued slot=%u delay=%lums reason=slots-full\r\n",
          (unsigned)g_connect_after_scan_slot,
          (unsigned long)EXO_HUB_CONNECT_AFTER_SCAN_MS);
  exo_send_disc_report(EXO_DISC_EVT_CONNECT_QUEUED,
                       exo_leaf_slot_node_id(&g_leaf_slots[g_connect_after_scan_slot]),
                       g_connect_after_scan_slot,
                       (uint8_t)g_leaf_slots[g_connect_after_scan_slot].state,
                       (uint16_t)EXO_HUB_CONNECT_AFTER_SCAN_MS);
  if (aci_gap_terminate_gap_proc(g_scan_proc_code) == BLE_STATUS_SUCCESS)
  {
    EXO_LOG("[BLE][HUB][DISC] scan terminate requested slot=%u\r\n",
            (unsigned)g_connect_after_scan_slot);
    return;
  }
  g_scan_active = 0U;
  EXO_LOG("[BLE][HUB][DISC] scan terminate failed, connecting now slot=%u\r\n",
          (unsigned)g_connect_after_scan_slot);
  exo_start_pending_connection();
}

void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 const uint8_t *Data)
{
  (void)Data_Length;
  (void)Data;
  if ((Procedure_Code == EXO_HUB_PROC_GENERAL_DISCOVERY) ||
      (Procedure_Code == EXO_HUB_PROC_GENERAL_CONNECTION) ||
      (Procedure_Code == EXO_HUB_PROC_DIRECT_CONNECTION))
  {
    g_scan_active = 0U;
    EXO_LOG("[BLE][HUB][DISC] gap proc complete proc=0x%02X status=%u pending=%u busy=%u\r\n",
            (unsigned)Procedure_Code,
            (unsigned)Status,
            (unsigned)g_connect_after_scan_slot,
            (unsigned)g_connect_busy);
    if (g_discovery_hold != 0U &&
        g_connect_busy == 0U &&
        g_discovery_active == 0U)
    {
      g_connect_after_scan_slot = 0xFFU;
      g_connect_after_scan_ms = 0U;
      g_scan_requested = 0U;
      g_scan_timeout_stop = 0U;
      EXO_LOG("[BLE][HUB][DISC] scan complete held idle adv resume\r\n");
      APP_BLE_LeafClientScanIdle();
      return;
    }
    if (g_connect_after_scan_slot == 0xFFU &&
        g_connect_busy == 0U &&
        Status == 0U)
    {
      const uint8_t next_slot = exo_find_next_connectable_slot();
      if (next_slot != 0xFFU)
      {
        g_connect_after_scan_slot = next_slot;
        g_connect_after_scan_ms = HAL_GetTick() + EXO_HUB_CONNECT_AFTER_SCAN_MS;
        EXO_LOG("[BLE][HUB][DISC] scan complete queued discovered slot=%u node_hint=%u delay=%lums\r\n",
                (unsigned)next_slot,
                (unsigned)g_leaf_slots[next_slot].node_hint,
                (unsigned long)EXO_HUB_CONNECT_AFTER_SCAN_MS);
        exo_send_disc_report(EXO_DISC_EVT_CONNECT_QUEUED,
                             exo_leaf_slot_node_id(&g_leaf_slots[next_slot]),
                             next_slot,
                             (uint8_t)g_leaf_slots[next_slot].state,
                             (uint16_t)EXO_HUB_CONNECT_AFTER_SCAN_MS);
      }
    }
    if (g_connect_after_scan_slot != 0xFFU)
    {
      if ((int32_t)(HAL_GetTick() - g_connect_after_scan_ms) >= 0)
      {
        g_connect_after_scan_ms = HAL_GetTick() + EXO_HUB_CONNECT_AFTER_SCAN_MS;
      }
      EXO_LOG("[BLE][HUB][DISC] scan complete connect deferred slot=%u delay=%lums\r\n",
              (unsigned)g_connect_after_scan_slot,
              (unsigned long)EXO_HUB_CONNECT_AFTER_SCAN_MS);
    }
    else if (g_connect_busy == 0U)
    {
      if (exo_should_resume_scan_after_timeout() != 0U)
      {
        g_scan_requested = 1U;
        g_next_scan_after_ms = HAL_GetTick() + EXO_HUB_SCAN_RESUME_MS;
        EXO_LOG("[BLE][HUB][DISC] scan idle resume=%lums act=%u rdy/con=%u/%u xport_m=0x%02X\r\n",
                (unsigned long)EXO_HUB_SCAN_RESUME_MS,
                (unsigned)exo_active_leaf_count(),
                (unsigned)exo_ready_or_connecting_leaf_count(),
                (unsigned)EXO_HUB_LEAF_MAX,
                (unsigned)exo_hub_central_client_transport_ready_node_mask());
      }
      else
      {
        g_scan_requested = 0U;
        EXO_LOG("[BLE][HUB][DISC] scan idle\r\n");
      }
      g_scan_timeout_stop = 0U;
      APP_BLE_LeafClientScanIdle();
    }
  }
}

void aci_att_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t Attribute_Data_Length,
                                           uint8_t Data_Length,
                                           const uint8_t *Attribute_Data_List)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(Connection_Handle);
  uint8_t offset = 0U;
  if (slot == 0 || slot->state != EXO_LEAF_SLOT_DISCOVER_SERVICE || Attribute_Data_List == 0)
  {
    return;
  }
  while ((uint16_t)(offset + Attribute_Data_Length) <= Data_Length)
  {
    const uint8_t *entry = &Attribute_Data_List[offset];
    if (Attribute_Data_Length >= 20U && exo_uuid_matches(&entry[4], k_blepipe_service_uuid) != 0U)
    {
      slot->service_start_handle = (uint16_t)(entry[0] | ((uint16_t)entry[1] << 8U));
      slot->service_end_handle = (uint16_t)(entry[2] | ((uint16_t)entry[3] << 8U));
      EXO_LOG("[BLE][HUB][DISC] service slot=%u range=0x%04X-0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)slot->service_start_handle,
              (unsigned)slot->service_end_handle);
      break;
    }
    offset = (uint8_t)(offset + Attribute_Data_Length);
  }
}

void aci_att_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint8_t Data_Length,
                                     const uint8_t *Handle_Value_Pair_Data)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(Connection_Handle);
  uint8_t offset = 0U;
  if (slot == 0 || slot->state != EXO_LEAF_SLOT_DISCOVER_CHARS || Handle_Value_Pair_Data == 0)
  {
    return;
  }
  while ((uint16_t)(offset + Handle_Value_Pair_Length) <= Data_Length)
  {
    const uint8_t *entry = &Handle_Value_Pair_Data[offset];
    const uint16_t decl_handle = (uint16_t)(entry[0] | ((uint16_t)entry[1] << 8U));
    const uint16_t value_handle = (uint16_t)(entry[3] | ((uint16_t)entry[4] << 8U));
    const uint8_t *uuid = &entry[5];
    if (Handle_Value_Pair_Length >= 21U && exo_uuid_matches(uuid, k_blepipe_data_uuid) != 0U)
    {
      slot->data_decl_handle = decl_handle;
      slot->data_value_handle = value_handle;
      EXO_LOG("[BLE][HUB][DISC] char data slot=%u value=0x%04X decl=0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)value_handle,
              (unsigned)decl_handle);
    }
    else if (Handle_Value_Pair_Length >= 21U && exo_uuid_matches(uuid, k_blepipe_ctrl_rx_uuid) != 0U)
    {
      slot->ctrl_rx_value_handle = value_handle;
      EXO_LOG("[BLE][HUB][DISC] char ctrl_rx slot=%u value=0x%04X decl=0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)value_handle,
              (unsigned)decl_handle);
    }
    else if (Handle_Value_Pair_Length >= 21U && exo_uuid_matches(uuid, k_blepipe_ctrl_tx_uuid) != 0U)
    {
      slot->ctrl_tx_decl_handle = decl_handle;
      slot->ctrl_tx_value_handle = value_handle;
      EXO_LOG("[BLE][HUB][DISC] char ctrl_tx slot=%u value=0x%04X decl=0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)value_handle,
              (unsigned)decl_handle);
    }
    else if (Handle_Value_Pair_Length >= 21U && exo_uuid_matches(uuid, k_blepipe_status_uuid) != 0U)
    {
      slot->status_decl_handle = decl_handle;
      slot->status_value_handle = value_handle;
      EXO_LOG("[BLE][HUB][DISC] char status slot=%u value=0x%04X decl=0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)value_handle,
              (unsigned)decl_handle);
    }
    else if (Handle_Value_Pair_Length >= 21U && exo_uuid_matches(uuid, k_blepipe_config_uuid) != 0U)
    {
      slot->config_value_handle = value_handle;
      EXO_LOG("[BLE][HUB][DISC] char config slot=%u value=0x%04X decl=0x%04X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)value_handle,
              (unsigned)decl_handle);
    }
    offset = (uint8_t)(offset + Handle_Value_Pair_Length);
  }
}

void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 const uint8_t *Attribute_Value)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(Connection_Handle);
  if (slot == 0 || Attribute_Value == 0)
  {
    return;
  }
  if (Attribute_Handle == slot->ctrl_tx_value_handle)
  {
    EXO_LOG("[BLE][HUB][DISC] notify ctrl_tx slot=%u handle=0x%04X len=%u\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)Attribute_Handle,
            (unsigned)Attribute_Value_Length);
    exo_handle_pipe_packet(slot, BLEPIPE_LANE_CONTROL_TX, Attribute_Value, Attribute_Value_Length);
  }
  else if (Attribute_Handle == slot->status_value_handle)
  {
    EXO_LOG("[BLE][HUB][DISC] notify status slot=%u handle=0x%04X len=%u\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)Attribute_Handle,
            (unsigned)Attribute_Value_Length);
    exo_handle_pipe_packet(slot, BLEPIPE_LANE_STATUS_TX, Attribute_Value, Attribute_Value_Length);
  }
  else if (Attribute_Handle == slot->data_value_handle)
  {
#if EXO_HUB_VERBOSE_PIPE_LOGS
    EXO_LOG("[BLE][HUB][DISC] notify data slot=%u handle=0x%04X len=%u\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)Attribute_Handle,
            (unsigned)Attribute_Value_Length);
#endif
    exo_handle_pipe_packet(slot, BLEPIPE_LANE_DATA_TX, Attribute_Value, Attribute_Value_Length);
  }
}

void aci_gatt_indication_event(uint16_t Connection_Handle,
                               uint16_t Attribute_Handle,
                               uint8_t Attribute_Value_Length,
                               const uint8_t *Attribute_Value)
{
  aci_gatt_notification_event(Connection_Handle,
                              Attribute_Handle,
                              Attribute_Value_Length,
                              Attribute_Value);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(Connection_Handle);
  if (slot == 0)
  {
    return;
  }
  EXO_LOG("[BLE][HUB][DISC] mtu exchange resp slot=%u mtu=%u state=%u\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)Server_RX_MTU,
          (unsigned)slot->state);
  if (slot->state == EXO_LEAF_SLOT_EXCHANGE_MTU)
  {
    slot->mtu_exchange_done = 1U;
  }
}

void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  exo_leaf_slot_t *slot = exo_find_slot_by_conn(Connection_Handle);
  if (slot == 0)
  {
    return;
  }
  EXO_LOG("[BLE][HUB][DISC] gatt proc complete slot=%u err=%u state=%u\r\n",
          (unsigned)(slot - &g_leaf_slots[0]),
          (unsigned)Error_Code,
          (unsigned)slot->state);
  if (Error_Code != BLE_STATUS_SUCCESS)
  {
    exo_leaf_slot_mark_backoff(slot);
    g_scan_requested = 1U;
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_EXCHANGE_MTU)
  {
    EXO_LOG("[BLE][HUB][DISC] mtu exchange complete slot=%u seen_resp=%u\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)slot->mtu_exchange_done);
    exo_begin_service_discovery(slot);
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_DISCOVER_SERVICE)
  {
    if (slot->service_start_handle == 0U || slot->service_end_handle == 0U)
    {
      exo_leaf_slot_mark_backoff(slot);
      g_scan_requested = 1U;
      return;
    }
    exo_begin_char_discovery(slot);
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_DISCOVER_CHARS)
  {
    if (slot->ctrl_rx_value_handle == 0U || slot->ctrl_tx_value_handle == 0U || slot->status_value_handle == 0U)
    {
      exo_leaf_slot_mark_backoff(slot);
      g_scan_requested = 1U;
      return;
    }
    slot->state = EXO_LEAF_SLOT_ENABLE_CTRL_NOTIFY;
    exo_enable_notify_step(slot);
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_ENABLE_CTRL_NOTIFY)
  {
    slot->state = EXO_LEAF_SLOT_ENABLE_STATUS_NOTIFY;
    slot->notify_mask |= 0x01U;
    exo_enable_notify_step(slot);
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_ENABLE_STATUS_NOTIFY)
  {
    slot->notify_mask |= 0x02U;
    if (slot->data_value_handle == 0U)
    {
      slot->state = EXO_LEAF_SLOT_READY;
      g_discovery_active = 0U;
      g_scan_requested = 1U;
      g_next_scan_after_ms = HAL_GetTick() + EXO_HUB_SCAN_RESUME_MS;
      APP_BLE_LeafClientScanIdle();
      exo_hub_leaf_topology_touch(slot->node_id != 0U ? slot->node_id : slot->node_hint);
      EXO_LOG("[DISC] leaf rdy s=%u n=%u nm=%02X rs=%lums rm=%02X xm=%02X\r\n",
              (unsigned)(slot - &g_leaf_slots[0]),
              (unsigned)(slot->node_id != 0U ? slot->node_id : slot->node_hint),
              (unsigned)slot->notify_mask,
              (unsigned long)EXO_HUB_SCAN_RESUME_MS,
              (unsigned)exo_hub_central_client_ready_node_mask(),
              (unsigned)exo_hub_central_client_transport_ready_node_mask());
      exo_send_disc_report(EXO_DISC_EVT_LEAF_READY,
                           exo_leaf_slot_node_id(slot),
                           (uint8_t)(slot - &g_leaf_slots[0]),
                           (uint8_t)slot->state,
                           (uint16_t)slot->notify_mask);
      return;
    }
    slot->state = EXO_LEAF_SLOT_ENABLE_DATA_NOTIFY;
    exo_enable_notify_step(slot);
    return;
  }
  if (slot->state == EXO_LEAF_SLOT_ENABLE_DATA_NOTIFY)
  {
    slot->notify_mask |= 0x04U;
    slot->state = EXO_LEAF_SLOT_READY;
    g_discovery_active = 0U;
    g_scan_requested = 1U;
    g_next_scan_after_ms = HAL_GetTick() + EXO_HUB_SCAN_RESUME_MS;
    APP_BLE_LeafClientScanIdle();
    exo_hub_leaf_topology_touch(slot->node_id != 0U ? slot->node_id : slot->node_hint);
    EXO_LOG("[DISC] leaf rdy s=%u n=%u nm=%02X rs=%lums rm=%02X xm=%02X\r\n",
            (unsigned)(slot - &g_leaf_slots[0]),
            (unsigned)(slot->node_id != 0U ? slot->node_id : slot->node_hint),
            (unsigned)slot->notify_mask,
            (unsigned long)EXO_HUB_SCAN_RESUME_MS,
            (unsigned)exo_hub_central_client_ready_node_mask(),
            (unsigned)exo_hub_central_client_transport_ready_node_mask());
    exo_send_disc_report(EXO_DISC_EVT_LEAF_READY,
                         exo_leaf_slot_node_id(slot),
                         (uint8_t)(slot - &g_leaf_slots[0]),
                         (uint8_t)slot->state,
                         (uint16_t)slot->notify_mask);
  }
}

void exo_hub_central_client_request_targeted_reconnect(uint8_t node_id)
{
  if (node_id < 1U || node_id > 4U || g_discovery_hold == 0U)
  {
    return;
  }
  g_targeted_reconnect_node_id = node_id;
  g_targeted_reconnect_after_ms = HAL_GetTick() + EXO_HUB_TARGETED_RECONNECT_DELAY_MS;
  EXO_LOG("[BLE][HUB][DISC] targeted reconnect requested node=%u delay=%ums attempts=%u\r\n",
          (unsigned)node_id,
          (unsigned)EXO_HUB_TARGETED_RECONNECT_DELAY_MS,
          (unsigned)g_targeted_reconnect_attempts);
}

void exo_hub_central_client_set_discovery_hold(uint8_t hold)
{
  const uint8_t requested = (uint8_t)(hold != 0U ? 1U : 0U);
  if (requested == g_discovery_hold)
  {
    return;
  }
  g_discovery_hold = requested;
  EXO_LOG("[BLE][HUB][DISC] session hold=%u scan=%u connect=%u discovery=%u transport_mask=0x%02X\r\n",
          (unsigned)g_discovery_hold,
          (unsigned)g_scan_active,
          (unsigned)g_connect_busy,
          (unsigned)g_discovery_active,
          (unsigned)exo_hub_central_client_transport_ready_node_mask());
  if (g_discovery_hold != 0U)
  {
    /* Cancel only a passive scan. Never tear down an established leaf link or
     * a GATT procedure. Discovered slots remain available after the session. */
    g_scan_requested = 0U;
    g_connect_after_scan_slot = 0xFFU;
    g_connect_after_scan_ms = 0U;
    if (g_scan_active != 0U && g_connect_busy == 0U && g_discovery_active == 0U)
    {
      const tBleStatus status = aci_gap_terminate_gap_proc(g_scan_proc_code);
      g_scan_timeout_stop = 1U;
      if (status != BLE_STATUS_SUCCESS)
      {
        g_scan_active = 0U;
        g_scan_timeout_stop = 0U;
        EXO_LOG("[BLE][HUB][DISC] hold stop-scan failed status=%u, idling app radio\r\n",
                (unsigned)status);
        APP_BLE_LeafClientScanIdle();
      }
    }
    else if (g_connect_busy == 0U && g_discovery_active == 0U)
    {
      APP_BLE_LeafClientScanIdle();
    }
  }
  else
  {
    g_scan_requested = 1U;
    g_next_scan_after_ms = HAL_GetTick() + EXO_HUB_SCAN_RETRY_MS;
    g_targeted_reconnect_node_id = 0U;
    g_targeted_reconnect_attempts = 0U;
  }
}

/* HCI completion callbacks for the one Master-wide LL arbiter.  The event
 * handler validates handle/generation/state before it may release the next
 * request, so an unrelated or disconnected completion cannot advance a link. */
void exo_hub_central_client_on_data_length_change(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  EXO_LOG("[BLE][HUB][LINK] DLE change h=%04X tx_oct=%u tx_us=%u rx_oct=%u rx_us=%u\r\n",
          (unsigned)Connection_Handle,
          (unsigned)MaxTxOctets,
          (unsigned)MaxTxTime,
          (unsigned)MaxRxOctets,
          (unsigned)MaxRxTime);
  exo_leaf_slot_t *const slot = exo_find_slot_by_conn(Connection_Handle);
  if (slot != 0)
  {
    const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
    const uint32_t generation = g_link_tune.telemetry(slot_index).generation;
    if (g_link_tune.on_dle_complete(Connection_Handle, generation,
                                    MaxTxOctets, MaxRxOctets, HAL_GetTick()))
    {
      slot->link_dle_tx_oct = MaxTxOctets;
      slot->link_dle_rx_oct = MaxRxOctets;
      exo_send_disc_report(EXO_DISC_EVT_LINK_DLE, exo_leaf_slot_node_id(slot), slot_index,
                           (uint8_t)(MaxRxOctets & 0xFFU), MaxTxOctets);
      exo_report_link_tune(slot_index);
    }
  }
}

void exo_hub_central_client_on_phy_update_complete(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t TX_PHY,
                                      uint8_t RX_PHY)
{
  EXO_LOG("[BLE][HUB][LINK] PHY update h=%04X status=%u tx_phy=0x%02X rx_phy=0x%02X\r\n",
          (unsigned)Connection_Handle,
          (unsigned)Status,
          (unsigned)TX_PHY,
          (unsigned)RX_PHY);
  exo_leaf_slot_t *const slot = exo_find_slot_by_conn(Connection_Handle);
  if (slot != 0)
  {
    const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
    const uint32_t generation = g_link_tune.telemetry(slot_index).generation;
    const uint8_t accepted = (uint8_t)g_link_tune.on_phy_complete(
        Connection_Handle, generation, Status, TX_PHY, RX_PHY, HAL_GetTick());
    if (accepted != 0U)
    {
      slot->link_tx_phy = TX_PHY;
      slot->link_rx_phy = RX_PHY;
      exo_send_disc_report(EXO_DISC_EVT_LINK_PHY, exo_leaf_slot_node_id(slot), slot_index, Status,
                           (uint16_t)((uint16_t)TX_PHY | ((uint16_t)RX_PHY << 8U)));
      exo_report_link_tune(slot_index);
    }
  }
}

void exo_hub_central_client_on_connection_update_complete(uint8_t Status,
                                             uint16_t Connection_Handle,
                                             uint16_t Conn_Interval,
                                             uint16_t Conn_Latency,
                                             uint16_t Supervision_Timeout)
{
  exo_leaf_slot_t *const slot = exo_find_slot_by_conn(Connection_Handle);
  (void)Conn_Latency;
  (void)Supervision_Timeout;
  if (slot == 0)
  {
    return;
  }
  const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
  const uint32_t generation = g_link_tune.telemetry(slot_index).generation;
  const uint8_t accepted = (uint8_t)g_link_tune.on_interval_complete(
      Connection_Handle, generation, Status, Conn_Interval, HAL_GetTick());
  if (accepted != 0U)
  {
    exo_send_disc_report(EXO_DISC_EVT_LINK_TIMING, exo_leaf_slot_node_id(slot), slot_index,
                         Status, Conn_Interval);
    exo_report_link_tune(slot_index);
  }
}
