#include "exo_hub_central_client.h"

#include <string.h>

#include "main.h"
#include "app_conf.h"
#include "custom_app.h"
#include "dbg_trace.h"
#include "ble_gap_aci.h"
#include "ble_gatt_aci.h"
#include "ble_events.h"
#include "ble_std.h"
#include "ble_types.h"
#include "EXO_HUB_LEAF_BRIDGE.h"

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

extern void APP_BLE_LeafClientConnecting(void);
extern void APP_BLE_LeafClientConnectIdle(void);
extern uint8_t APP_BLE_LeafClientPrepareScan(void);
extern void APP_BLE_LeafClientScanIdle(void);
extern uint8_t APP_BLE_LeafClientPhoneConnected(void);
extern void exo_hub_leaf_control_ingest(uint8_t node_id,
                                        uint8_t msg_type,
                                        const uint8_t *payload,
                                        uint16_t payload_len);

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
  EXO_DISC_EVT_SCAN_DELAYED = 0x0AU
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
  EXO_LOG("[BLE][HUB][DISC] adv node=%u addr=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d type=0x%02X len=%u\r\n",
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
  EXO_LOG("[BLE][HUB][DISC] scan start general-disc interval=0x%04X window=0x%04X ready=%u pending=%u\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] scan while phone-connected active=%u ready=%u transport_mask=0x%02X\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] scan start failed status=%u active=%u ready=%u transport_mask=0x%02X\r\n",
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
  EXO_LOG("[BLE][HUB][DISC] connect pending slot=%u node_hint=%u state=%u addr=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] DISC WARN fast_leaf_interval: NODE%u conn=0x%04X-0x%04X may block multi-link scheduling\r\n",
            (unsigned)exo_leaf_slot_node_id(slot),
            (unsigned)conn_interval_min,
            (unsigned)conn_interval_max);
  }
  EXO_LOG("[BLE][HUB][DISC] DISC create_conn_params_a: NODE%u slot=%u scan=0x%04X/0x%04X conn=0x%04X-0x%04X latency=%u timeout=0x%04X ce=0x%04X/0x%04X\r\n",
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
  EXO_LOG("[BLE][HUB][DISC] DISC create_conn_params_b: NODE%u ready_mask=0x%02X transport_mask=0x%02X active=%u ready=%u busy=%u pending=%u tick=%lu\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] DISC connect_failed_detail: NODE%u slot=%u status=%u/0x%02X %s retry_ms=%lu ce=0x%04X/0x%04X conn=0x%04X-0x%04X scan=0x%04X/0x%04X latency=%u timeout=0x%04X ready_mask=0x%02X transport_mask=0x%02X active=%u ready=%u busy=%u pending=%u tick=%lu\r\n",
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
    EXO_LOG("[BLE][HUB][TX] retry-write node=%u msg=0x%02X dst=0x%04X handle=0x%04X len=%u status=0x%02X\r\n",
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
    EXO_LOG("[BLE][HUB][LEAF] record_done suppress_phone reason=%s node=%u session=%lu size=%lu crc=0x%08lX\r\n",
            reason,
            (unsigned)done.node_id,
            (unsigned long)done.session_id,
            (unsigned long)done.total_size,
            (unsigned long)done.payload_crc32);
    return 1U;
  }

  EXO_LOG("[BLE][HUB][LEAF] record_done queue_fail forward_phone reason=%s node=%u session=%lu size=%lu\r\n",
          reason,
          (unsigned)done.node_id,
          (unsigned long)done.session_id,
          (unsigned long)done.total_size);
  return 0U;
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
      if (exo_hub_maybe_queue_record_done(data, length, "decode_fail") != 0U)
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
    exo_handle_leaf_status(slot, &hdr, payload, payload_len);
    (void)Custom_APP_SendRecoveryFrame(data, decoded_frame_len);
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
    if (exo_hub_maybe_queue_record_done(payload, payload_len, "raw_forward") != 0U)
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
  if (g_discovery_hold != 0U)
  {
    /* Do not let scan/connect scheduler work compete with an established
     * recording or reliable Node->Master artifact transfer. Existing GATT
     * links and their notifications continue normally. */
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
  EXO_LOG("[BLE][HUB][DISC] connection complete slot=%u status=%u handle=0x%04X peer_type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
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
    return;
  }
  if (peer_address != 0)
  {
    if (memcmp(slot->addr, peer_address, 6U) != 0)
    {
      const uint8_t slot_index = (uint8_t)(slot - &g_leaf_slots[0]);
      EXO_LOG("[BLE][HUB][DISC] connection addr mismatch slot=%u handle=0x%04X expected=%02X:%02X:%02X:%02X:%02X:%02X got=%02X:%02X:%02X:%02X:%02X:%02X backoff\r\n",
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
      return;
    }
    slot->addr_type = peer_address_type;
    memcpy(slot->addr, peer_address, 6U);
  }
  slot->connection_handle = connection_handle;
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
  exo_leaf_slot_mark_backoff(slot);
  g_scan_requested = 1U;
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
      EXO_LOG("[BLE][HUB][DISC] duplicate logical node ignored node=%u owner_slot=%u owner_state=%u\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] adv skipped node=%u slot=%u state=%u ready_or_connecting=%u/%u\r\n",
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
        EXO_LOG("[BLE][HUB][DISC] scan idle resume=%lums active=%u ready_or_connecting=%u/%u transport_mask=0x%02X\r\n",
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
      EXO_LOG("[BLE][HUB][DISC] leaf ready slot=%u node=%u notify_mask=0x%02X resume=%lums ready_mask=0x%02X transport_mask=0x%02X\r\n",
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
    EXO_LOG("[BLE][HUB][DISC] leaf ready slot=%u node=%u notify_mask=0x%02X resume=%lums ready_mask=0x%02X transport_mask=0x%02X\r\n",
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
  }
}
