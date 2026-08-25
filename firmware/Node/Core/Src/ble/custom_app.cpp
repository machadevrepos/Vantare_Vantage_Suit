/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include <exo/ble/custom_app.h>
#include <exo/ble/custom_stm.h>
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* BLEPipeService */
  uint8_t               Pipedatatx_Notification_Status;
  uint8_t               Pipectrltx_Notification_Status;
  uint8_t               Pipectrltx_Indication_Status;
  uint8_t               Pipestattx_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */


/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
typedef struct {
  uint8_t length;
  uint8_t payload[64];
} NodeControlFrame_t;
static NodeControlFrame_t g_pipe_control_frames[4];
static volatile uint8_t g_pipe_control_head;
static volatile uint8_t g_pipe_control_tail;
static volatile uint32_t g_pipe_control_drop_count;
static uint32_t g_pipe_control_drop_reported;
static uint8_t g_pipe_data_notify_enabled;
static uint8_t g_pipe_control_notify_enabled;
static uint8_t g_pipe_status_notify_enabled;
static volatile uint32_t g_pipe_notification_complete_count;
static volatile uint32_t g_pipe_tx_pool_event_count;
static volatile uint16_t g_pipe_tx_pool_buffers;
static volatile uint32_t g_pipe_disconnect_count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* BLEPipeService */
static void Custom_Pipedatatx_Update_Char(void);

/* USER CODE BEGIN PFP */
static void Custom_APP_StoreControlWrite(Custom_STM_App_Notification_evt_t *pNotification);
#ifdef __cplusplus
extern "C" {
#endif
void exo_node_ble_log(const char *format, ...);
uint8_t exo_node_ble_write(const uint8_t *payload, uint8_t length);
uint8_t exo_node_ble_status_notify_enabled(void);
#ifdef __cplusplus
}
#endif
#define NODE_BLE_LOG(...) exo_node_ble_log(__VA_ARGS__)

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* BLEPipeService */
    case CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT */
      g_pipe_data_notify_enabled = 1U;

      /* USER CODE END CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT */
      g_pipe_data_notify_enabled = 0U;

      /* USER CODE END CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT */
      Custom_APP_StoreControlWrite(pNotification);

      /* USER CODE END CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_PIPECTRLRX_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLRX_WRITE_EVT */
      Custom_APP_StoreControlWrite(pNotification);

      /* USER CODE END CUSTOM_STM_PIPECTRLRX_WRITE_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT */
      g_pipe_control_notify_enabled = 1U;

      /* USER CODE END CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT */
      g_pipe_control_notify_enabled = 0U;

      /* USER CODE END CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_INDICATE_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_PIPECTRLTX_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_INDICATE_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_PIPECTRLTX_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_PIPESTATTX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPESTATTX_READ_EVT */

      /* USER CODE END CUSTOM_STM_PIPESTATTX_READ_EVT */
      break;

    case CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT */
      g_pipe_status_notify_enabled = 1U;

      /* USER CODE END CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT */
      g_pipe_status_notify_enabled = 0U;

      /* USER CODE END CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_PIPECFGRW_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECFGRW_READ_EVT */

      /* USER CODE END CUSTOM_STM_PIPECFGRW_READ_EVT */
      break;

    case CUSTOM_STM_PIPECFGRW_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECFGRW_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_PIPECFGRW_WRITE_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      Custom_APP_NotificationComplete();

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      g_pipe_data_notify_enabled = 0U;
      g_pipe_control_notify_enabled = 0U;
      g_pipe_status_notify_enabled = 0U;
      g_pipe_control_head = g_pipe_control_tail;
      ++g_pipe_disconnect_count;

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
tBleStatus Custom_APP_SendPipeFrame(Custom_STM_Char_Opcode_t char_opcode, const uint8_t *payload, uint8_t length)
{
  if ((payload == NULL) || (length == 0U)) {
    return BLE_STATUS_INVALID_PARAMS;
  }

  switch (char_opcode) {
    case CUSTOM_STM_PIPEDATATX:
      if ((g_pipe_data_notify_enabled == 0U) || (length > (uint8_t)SizePipedatatx)) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPECTRLTX:
      if ((g_pipe_control_notify_enabled == 0U) || (length > (uint8_t)SizePipectrltx)) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPESTATTX:
      if ((g_pipe_status_notify_enabled == 0U) || (length > (uint8_t)SizePipestattx)) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPECFGRW:
      if (length > (uint8_t)SizePipecfgrw) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    default:
      return BLE_STATUS_INVALID_PARAMS;
  }

  return Custom_STM_App_Update_Char_Variable_Length(char_opcode, (uint8_t *)payload, length);
}

extern "C" uint8_t Custom_APP_PipeDataNotifyEnabled(void)
{
  return g_pipe_data_notify_enabled;
}

extern "C" void Custom_APP_NotificationComplete(void)
{
  ++g_pipe_notification_complete_count;
}

extern "C" void Custom_APP_TxPoolAvailable(uint16_t available_buffers)
{
  g_pipe_tx_pool_buffers = available_buffers;
  ++g_pipe_tx_pool_event_count;
}

extern "C" uint32_t Custom_APP_NotificationCompleteCount(void)
{
  return g_pipe_notification_complete_count;
}

extern "C" uint32_t Custom_APP_TxPoolEventCount(void)
{
  return g_pipe_tx_pool_event_count;
}

extern "C" uint16_t Custom_APP_LastTxPoolBuffers(void)
{
  return g_pipe_tx_pool_buffers;
}

extern "C" uint32_t Custom_APP_DisconnectCount(void)
{
  return g_pipe_disconnect_count;
}

extern "C" void Custom_APP_ProcessControlWrites(void)
{
  if (g_pipe_control_drop_reported != g_pipe_control_drop_count) {
    g_pipe_control_drop_reported = g_pipe_control_drop_count;
    NODE_BLE_LOG("[BLE][NODE][RX] control queue overflow drops=%lu\r\n",
                 (unsigned long)g_pipe_control_drop_reported);
  }
  while (g_pipe_control_head != g_pipe_control_tail) {
    __DMB();
    NodeControlFrame_t *frame = &g_pipe_control_frames[g_pipe_control_head];
    const uint8_t length = frame->length;
    NODE_BLE_LOG("[BLE][NODE][RX] ctrl len=%u first=0x%02X\r\n",
                (unsigned)length,
                length != 0U ? (unsigned)frame->payload[0] : 0U);
    const uint8_t handled = exo_node_ble_write(frame->payload, length);
    NODE_BLE_LOG("[BLE][NODE][RX] handled=%u\r\n", (unsigned)handled);
    g_pipe_control_head = (uint8_t)((g_pipe_control_head + 1U) % 4U);
  }
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* BLEPipeService */
__USED void Custom_Pipedatatx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipedatatx_UC_1*/

  /* USER CODE END Pipedatatx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPEDATATX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Pipedatatx_UC_Last*/

  /* USER CODE END Pipedatatx_UC_Last*/
  return;
}

static void Custom_APP_StoreControlWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  uint8_t copy_len;

  if ((pNotification == NULL) || (pNotification->DataTransfered.pPayload == NULL)) {
    return;
  }

  copy_len = pNotification->DataTransfered.Length;
  if (copy_len > (uint8_t)sizeof(g_pipe_control_frames[0].payload)) {
    copy_len = (uint8_t)sizeof(g_pipe_control_frames[0].payload);
  }

  if (copy_len == 0U) {
    return;
  }

  const uint8_t next_tail = (uint8_t)((g_pipe_control_tail + 1U) % 4U);
  if (next_tail == g_pipe_control_head) {
    ++g_pipe_control_drop_count;
    return;
  }
  NodeControlFrame_t *frame = &g_pipe_control_frames[g_pipe_control_tail];
  memcpy(frame->payload, pNotification->DataTransfered.pPayload, copy_len);
  frame->length = copy_len;
  __DMB();
  g_pipe_control_tail = next_tail;
}

uint8_t exo_node_ble_status_notify_enabled(void)
{
  return g_pipe_status_notify_enabled;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
