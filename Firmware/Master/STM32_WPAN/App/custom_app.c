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
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
extern void exo_hub_ble_evt_trace(uint8_t evt, const uint8_t *payload, uint8_t length);
extern void exo_hub_ble_notify_state_trace(uint8_t channel, uint8_t enabled);
extern uint8_t exo_hub_ble_write(const uint8_t *payload, uint8_t length);

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

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
static uint8_t g_cmd_ack_buf[244];
static uint8_t g_last_cmd_write_payload[64];
static uint8_t g_last_cmd_write_len = 0U;
static uint32_t g_last_cmd_write_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* BLEPipeService */
static void Custom_Pipedatatx_Update_Char(void);
static void Custom_Pipedatatx_Send_Notification(void);
static void Custom_Pipectrltx_Update_Char(void);
static void Custom_Pipectrltx_Send_Notification(void);
static void Custom_Pipectrltx_Send_Indication(void);
static void Custom_Pipestattx_Update_Char(void);
static void Custom_Pipestattx_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Custom_APP_StoreLastWrite(Custom_STM_App_Notification_evt_t *pNotification);

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
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_IMU, 1U);
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_RECORD, 1U);

      /* USER CODE END CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT */
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_IMU, 0U);
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_RECORD, 0U);

      /* USER CODE END CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT */
      Custom_APP_StoreLastWrite(pNotification);
      exo_hub_ble_evt_trace((uint8_t)CUSTOM_STM_CMD_WRITE_NO_RESP_EVT,
                            pNotification->DataTransfered.pPayload,
                            pNotification->DataTransfered.Length);
      (void)exo_hub_ble_write(pNotification->DataTransfered.pPayload,
                              pNotification->DataTransfered.Length);

      /* USER CODE END CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_PIPECTRLRX_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLRX_WRITE_EVT */
      Custom_APP_StoreLastWrite(pNotification);
      exo_hub_ble_evt_trace((uint8_t)CUSTOM_STM_CMD_WRITE_NO_RESP_EVT,
                            pNotification->DataTransfered.pPayload,
                            pNotification->DataTransfered.Length);
      (void)exo_hub_ble_write(pNotification->DataTransfered.pPayload,
                              pNotification->DataTransfered.Length);

      /* USER CODE END CUSTOM_STM_PIPECTRLRX_WRITE_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT */
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_CMD_ACK, 1U);

      /* USER CODE END CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT */
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_CMD_ACK, 0U);

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
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_RECOVERY, 1U);

      /* USER CODE END CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT */
      exo_hub_ble_notify_state_trace(CUSTOM_STM_CHANNEL_RECOVERY, 0U);

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
      {
        uint8_t attr[2];
        attr[0] = (uint8_t)(pNotification->AttrHandle & 0xFFU);
        attr[1] = (uint8_t)((pNotification->AttrHandle >> 8) & 0xFFU);
        exo_hub_ble_evt_trace((uint8_t)CUSTOM_STM_NOTIFICATION_COMPLETE_EVT, attr, 2U);
      }

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
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */
  static uint32_t s_last_conn_log_ms = 0U;
  static uint32_t s_last_disconn_log_ms = 0U;

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
      if ((HAL_GetTick() - s_last_conn_log_ms) > 100U) {
        APP_DBG_MSG("[BLE][DBG][LINK] connected\r\n");
        s_last_conn_log_ms = HAL_GetTick();
      }

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      if ((HAL_GetTick() - s_last_disconn_log_ms) > 100U) {
        APP_DBG_MSG("[BLE][DBG][LINK] disconnected, waiting reconnect\r\n");
        s_last_disconn_log_ms = HAL_GetTick();
      }

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
      if (length > (uint8_t)SizePipedatatx) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPECTRLRX:
      if (length > (uint8_t)SizePipectrlrx) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPECTRLTX:
      if (length > (uint8_t)SizePipectrltx) {
        return BLE_STATUS_INVALID_PARAMS;
      }
      break;

    case CUSTOM_STM_PIPESTATTX:
      if (length > (uint8_t)SizePipestattx) {
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

tBleStatus Custom_APP_SendImuFrame(const uint8_t *payload, uint8_t length)
{
  tBleStatus status = Custom_APP_SendPipeFrame(CUSTOM_STM_PIPEDATATX, payload, length);
  if (status == BLE_STATUS_SUCCESS) {
    exo_hub_ble_evt_trace(0xF0U, payload, length);
  }
  return status;
}

tBleStatus Custom_APP_SendCmdAck(const uint8_t *payload, uint8_t length, uint8_t status)
{
  uint8_t out_len = 0U;

  if ((payload == NULL) || (length == 0U)) {
    return BLE_STATUS_INVALID_PARAMS;
  }
  if (length > ((uint8_t)sizeof(g_cmd_ack_buf) - 3U)) {
    return BLE_STATUS_INVALID_PARAMS;
  }
  g_cmd_ack_buf[out_len++] = 0xE1U; /* command ACK marker */
  g_cmd_ack_buf[out_len++] = status; /* 1=accepted, 0=rejected */
  g_cmd_ack_buf[out_len++] = length; /* original cmd payload length */
  memcpy(&g_cmd_ack_buf[out_len], payload, length);
  out_len = (uint8_t)(out_len + length);
  exo_hub_ble_evt_trace(0xF1U, g_cmd_ack_buf, out_len);
  return Custom_APP_SendPipeFrame(CUSTOM_STM_PIPECTRLTX, g_cmd_ack_buf, out_len);
}

tBleStatus Custom_APP_SendCmdReport(uint8_t report_id, const uint8_t *payload, uint8_t length)
{
  uint8_t out_len = 0U;

  if (length > ((uint8_t)sizeof(g_cmd_ack_buf) - 3U)) {
    return BLE_STATUS_INVALID_PARAMS;
  }
  g_cmd_ack_buf[out_len++] = 0xE2U; /* command report marker */
  g_cmd_ack_buf[out_len++] = report_id;
  g_cmd_ack_buf[out_len++] = length;
  if ((payload != NULL) && (length > 0U)) {
    memcpy(&g_cmd_ack_buf[out_len], payload, length);
    out_len = (uint8_t)(out_len + length);
  }
  exo_hub_ble_evt_trace(0xF4U, g_cmd_ack_buf, out_len);
  return Custom_APP_SendPipeFrame(CUSTOM_STM_PIPECTRLTX, g_cmd_ack_buf, out_len);
}

tBleStatus Custom_APP_SendCmdTrace(const uint8_t *payload, uint8_t length, uint8_t status)
{
  return Custom_APP_SendCmdAck(payload, length, status);
}

tBleStatus Custom_APP_SendCmdNotify(const uint8_t *payload, uint8_t length)
{
  tBleStatus status = Custom_APP_SendPipeFrame(CUSTOM_STM_PIPECTRLTX, payload, length);
  if (status == BLE_STATUS_SUCCESS) {
    exo_hub_ble_evt_trace(0xF3U, payload, length);
  }
  return status;
}

tBleStatus Custom_APP_SendRecordFrame(const uint8_t *payload, uint8_t length)
{
  tBleStatus status = Custom_APP_SendPipeFrame(CUSTOM_STM_PIPEDATATX, payload, length);
  if (status == BLE_STATUS_SUCCESS) {
    exo_hub_ble_evt_trace(0xF2U, payload, length);
  }
  return status;
}

tBleStatus Custom_APP_SendRecoveryFrame(const uint8_t *payload, uint8_t length)
{
  tBleStatus status = Custom_APP_SendPipeFrame(CUSTOM_STM_PIPESTATTX, payload, length);
  if (status == BLE_STATUS_SUCCESS) {
    exo_hub_ble_evt_trace(0xF5U, payload, length);
  }
  return status;
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

void Custom_Pipedatatx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipedatatx_NS_1*/

  /* USER CODE END Pipedatatx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPEDATATX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Pipedatatx_NS_Last*/

  /* USER CODE END Pipedatatx_NS_Last*/

  return;
}

__USED void Custom_Pipectrltx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipectrltx_UC_1*/

  /* USER CODE END Pipectrltx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPECTRLTX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Pipectrltx_UC_Last*/

  /* USER CODE END Pipectrltx_UC_Last*/
  return;
}

void Custom_Pipectrltx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipectrltx_NS_1*/

  /* USER CODE END Pipectrltx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPECTRLTX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Pipectrltx_NS_Last*/

  /* USER CODE END Pipectrltx_NS_Last*/

  return;
}

void Custom_Pipectrltx_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipectrltx_IS_1*/

  /* USER CODE END Pipectrltx_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPECTRLTX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Pipectrltx_IS_Last*/

  /* USER CODE END Pipectrltx_IS_Last*/

  return;
}

__USED void Custom_Pipestattx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipestattx_UC_1*/

  /* USER CODE END Pipestattx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPESTATTX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Pipestattx_UC_Last*/

  /* USER CODE END Pipestattx_UC_Last*/
  return;
}

void Custom_Pipestattx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pipestattx_NS_1*/

  /* USER CODE END Pipestattx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PIPESTATTX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Pipestattx_NS_Last*/

  /* USER CODE END Pipestattx_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void Custom_APP_StoreLastWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  uint8_t copy_len;

  if ((pNotification == NULL) || (pNotification->DataTransfered.pPayload == NULL)) {
    return;
  }

  copy_len = pNotification->DataTransfered.Length;
  if (copy_len > (uint8_t)sizeof(g_last_cmd_write_payload)) {
    copy_len = (uint8_t)sizeof(g_last_cmd_write_payload);
  }

  if (copy_len > 0U) {
    memcpy(g_last_cmd_write_payload, pNotification->DataTransfered.pPayload, copy_len);
  }
  g_last_cmd_write_len = copy_len;
  g_last_cmd_write_tick = HAL_GetTick();
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
