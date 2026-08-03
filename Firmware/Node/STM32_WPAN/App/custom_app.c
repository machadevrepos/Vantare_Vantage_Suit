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
static uint8_t g_pipe_last_control_rx[64];
static uint8_t g_pipe_last_control_rx_len;
static uint8_t g_pipe_data_notify_enabled;
static uint8_t g_pipe_control_notify_enabled;
static uint8_t g_pipe_status_notify_enabled;

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
      NODE_BLE_LOG("[BLE][NODE][GATT] ctrl write-no-rsp len=%u\r\n",
                  (unsigned)pNotification->DataTransfered.Length);
      Custom_APP_StoreControlWrite(pNotification);

      /* USER CODE END CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_PIPECTRLRX_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPECTRLRX_WRITE_EVT */
      NODE_BLE_LOG("[BLE][NODE][GATT] ctrl write len=%u\r\n",
                  (unsigned)pNotification->DataTransfered.Length);
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
        NODE_BLE_LOG("[BLE][NODE][LINK] connected\r\n");
        s_last_conn_log_ms = HAL_GetTick();
      }

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      if ((HAL_GetTick() - s_last_disconn_log_ms) > 100U) {
        NODE_BLE_LOG("[BLE][NODE][LINK] disconnected\r\n");
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
  tBleStatus status;

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

  status = Custom_STM_App_Update_Char_Variable_Length(char_opcode, (uint8_t *)payload, length);
  if (char_opcode == CUSTOM_STM_PIPEDATATX) {
    NODE_BLE_LOG("[BLE][NODE][TX] data notify len=%u status=0x%02X enabled=%u\r\n",
                 (unsigned)length,
                 (unsigned)status,
                 (unsigned)g_pipe_data_notify_enabled);
  }
  return status;
}

uint8_t Custom_APP_PipeDataNotifyEnabled(void)
{
  return g_pipe_data_notify_enabled;
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
static void Custom_APP_StoreControlWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  uint8_t copy_len;

  if ((pNotification == NULL) || (pNotification->DataTransfered.pPayload == NULL)) {
    return;
  }

  copy_len = pNotification->DataTransfered.Length;
  if (copy_len > (uint8_t)sizeof(g_pipe_last_control_rx)) {
    copy_len = (uint8_t)sizeof(g_pipe_last_control_rx);
  }

  if (copy_len > 0U) {
    uint8_t handled;
    memcpy(g_pipe_last_control_rx, pNotification->DataTransfered.pPayload, copy_len);
    NODE_BLE_LOG("[BLE][NODE][RX] ctrl len=%u first=0x%02X\r\n",
                (unsigned)copy_len,
                (unsigned)g_pipe_last_control_rx[0]);
    handled = exo_node_ble_write(g_pipe_last_control_rx, copy_len);
    NODE_BLE_LOG("[BLE][NODE][RX] handled=%u\r\n", (unsigned)handled);
  }
  g_pipe_last_control_rx_len = copy_len;
}

uint8_t exo_node_ble_status_notify_enabled(void)
{
  return g_pipe_status_notify_enabled;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
