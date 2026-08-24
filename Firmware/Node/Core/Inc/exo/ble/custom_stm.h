/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble_types.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* BLEPipeService */
  CUSTOM_STM_PIPEDATATX,
  CUSTOM_STM_PIPECTRLRX,
  CUSTOM_STM_PIPECTRLTX,
  CUSTOM_STM_PIPESTATTX,
  CUSTOM_STM_PIPECFGRW,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* PipeDataTx */
  CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT,
  /* PipeControlRx */
  CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT,
  CUSTOM_STM_PIPECTRLRX_WRITE_EVT,
  /* PipeControlTx */
  CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT,
  CUSTOM_STM_PIPECTRLTX_INDICATE_ENABLED_EVT,
  CUSTOM_STM_PIPECTRLTX_INDICATE_DISABLED_EVT,
  /* PipeStatusTx */
  CUSTOM_STM_PIPESTATTX_READ_EVT,
  CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT,
  /* PipeConfigRw */
  CUSTOM_STM_PIPECFGRW_READ_EVT,
  CUSTOM_STM_PIPECFGRW_WRITE_EVT,
  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint16_t SizePipedatatx;
extern uint16_t SizePipectrlrx;
extern uint16_t SizePipectrltx;
extern uint16_t SizePipestattx;
extern uint16_t SizePipecfgrw;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size);
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
