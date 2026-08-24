/*
 * CAN.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dhanv
 *  STM32 FDCAN Library Wrapper
 *  Reference: fdcan.c, RS485.h
 */

#ifndef CAN_H_
#define CAN_H_

#include <INCLUDER.h>
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

// CAN message structure
struct CAN_MSG {
		uint32_t id;
		uint8_t data[8];
		uint8_t len;
		uint32_t timestamp;
};

extern FDCAN_HandleTypeDef hfdcan1;
extern osMessageQueueId_t CAN_QHandle;

class CAN {
	public:
		FDCAN_HandleTypeDef *hcan;

		CAN(FDCAN_HandleTypeDef *hcan_i) :
				hcan(hcan_i) {
		}

		void INIT() {
			FDCAN_FilterTypeDef sFilterConfig;
			HAL_FDCAN_Start(hcan);
			sFilterConfig.IdType = FDCAN_EXTENDED_ID;
			sFilterConfig.FilterIndex = 0;
			sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
			sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			sFilterConfig.FilterID1 = 0x0000000;
			sFilterConfig.FilterID2 = 0x2222222;

			if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
					{
				Error_Handler();
			}
			if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
					{
				Error_Handler();
			}

		}

		bool SEND(uint32_t id, uint8_t *data, uint8_t len) {
			FDCAN_TxHeaderTypeDef txHeader = { };
			txHeader.Identifier = id;
			txHeader.IdType = FDCAN_EXTENDED_ID;
			txHeader.TxFrameType = FDCAN_DATA_FRAME;
			txHeader.DataLength = FDCAN_DLC_BYTES_8; // FDCAN_DLC_BYTES_8 for 8 bytes
			txHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
			txHeader.BitRateSwitch = FDCAN_BRS_OFF;
			txHeader.FDFormat = FDCAN_CLASSIC_CAN;
			txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			txHeader.MessageMarker = 0;
			if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &txHeader, data) == HAL_OK) {
				return true;
			}
			return false;
		}

		bool READ(CAN_MSG &msg, uint32_t timeout = 10) {
			FDCAN_RxHeaderTypeDef rxHeader = { };
			uint8_t rxData[8] = { 0 };
			if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
				msg.id = rxHeader.Identifier;
				msg.len = rxHeader.DataLength;
				memcpy(msg.data, rxData, msg.len);
				msg.timestamp = rxHeader.RxTimestamp;
				return true;
			}
			return false;
		}

		void SET_FILTER(uint32_t id, uint32_t mask = 0x7FF) {
			FDCAN_FilterTypeDef sFilterConfig = { };
			sFilterConfig.IdType = FDCAN_EXTENDED_ID;
			sFilterConfig.FilterIndex = 0;
			sFilterConfig.FilterType = FDCAN_FILTER_MASK;
			sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			sFilterConfig.FilterID1 = id;
			sFilterConfig.FilterID2 = mask;
			HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);
		}
};

#endif /* CAN_H_ */
