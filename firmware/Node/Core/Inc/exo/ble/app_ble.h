/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_ble.h
  * @author  MCD Application Team
  * @brief   Header for ble application
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
#ifndef APP_BLE_H
#define APP_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef enum
{
  APP_BLE_IDLE,
  APP_BLE_FAST_ADV,
  APP_BLE_LP_ADV,
  APP_BLE_SCAN,
  APP_BLE_LP_CONNECTING,
  APP_BLE_CONNECTED_SERVER,
  APP_BLE_CONNECTED_CLIENT
} APP_BLE_ConnStatus_t;

/* USER CODE BEGIN ET */
typedef struct
{
  uint16_t controller_max_tx_octets;
  uint16_t negotiated_tx_octets;
  uint16_t negotiated_rx_octets;
  uint8_t requested;
  uint8_t confirmed;
  uint8_t request_status;
  uint8_t commission_state; /* 0=unknown, 1=requested, 2=confirmed, 3=degraded, 4=failed */
  uint8_t commission_attempts;
} exo_node_ble_dle_status_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void APP_BLE_Init(void);
APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void);

/* USER CODE BEGIN EF */
/* Foreground-only DLE commissioning. HCI callbacks merely publish state. */
void exo_node_ble_link_process(void);
exo_node_ble_dle_status_t exo_node_ble_dle_status(void);

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*APP_BLE_H */
