/*
 * RS485.h
 *
 *  Created on: Dec 4, 2024
 *      Author: dhanv
 */

#ifndef RS485_H_
#define RS485_H_

#include <INCLUDER.h>
//#include "FreeRTOS.h"
//#include "task.h"
#include <stdint.h>
#include <string.h>
//#include "ArduinoJson.h"

static uint8_t interrupt_data[513] = { 0 };
#define RS485_STREAM_BUF_SIZE 1024

static uint8_t rs485_stream_buf[RS485_STREAM_BUF_SIZE] = {0};
static uint16_t rs485_stream_len = 0;

#define RS485_RX_DEBUG_ENABLE 0
#define RS485_RX_DEBUG_PREVIEW_BYTES 24U
#define RS485_RX_DEBUG_LOG_EVERY_N 10U
#define RS485_MODBUS_FLAG_TOKEN_PRESENT 0x01U
#define RS485_MODBUS_FLAG_IP_PRESENT 0x02U
#define RS485_MODBUS_FLAG_MSG_ID_PRESENT 0x04U
#define RS485_MODBUS_FLAG_WRITE_RESULT 0x40U
#define RS485_MODBUS_FLAG_WRITE_REQ 0x80U
#define STM32_BRIDGE_FLAG_TOKEN_PRESENT RS485_MODBUS_FLAG_TOKEN_PRESENT
#define STM32_BRIDGE_FLAG_IP_PRESENT RS485_MODBUS_FLAG_IP_PRESENT
#define STM32_BRIDGE_FLAG_MSG_ID_PRESENT RS485_MODBUS_FLAG_MSG_ID_PRESENT
#if defined(SLS_HUB)
String SPOKE_TEMPLATE = "->SPK^%s^%u<-";
#endif

#if defined(SLS_RELAY) || defined(SLS_DALI)
String HUB_TEMPLATE = "->HUB^%s<-";
#endif

String UID_TEMPLATE = "->UID^%s<-";

// Copy defined in freeRTOS.c
struct RS485_STC {
		uint16_t Size;
		uint16_t Index;
};



extern void save_to_eeprom(char *data, uint32_t size);
extern bool STM32_RS485_BridgeHandleRx(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);

static inline void RS485_DebugHexPreview(const uint8_t *data, uint16_t size, char *out, uint16_t out_size) {
	if ((out == NULL) || (out_size == 0U)) {
		return;
	}
	out[0] = '\0';
	if ((data == NULL) || (size == 0U)) {
		return;
	}

	uint16_t preview = size;
	if (preview > RS485_RX_DEBUG_PREVIEW_BYTES) {
		preview = RS485_RX_DEBUG_PREVIEW_BYTES;
	}

	uint16_t idx = 0U;
	for (uint16_t i = 0U; i < preview; i++) {
		int wrote = snprintf(&out[idx], out_size - idx, "%02X%s", data[i],
				(i + 1U < preview) ? " " : "");
		if (wrote <= 0) {
			break;
		}
		idx = (uint16_t) (idx + (uint16_t) wrote);
		if (idx >= out_size) {
			break;
		}
	}
}

static inline void RS485_RestartReceive(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t max_size) {
	HAL_StatusTypeDef dma_status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, max_size);
	if (dma_status != HAL_OK) {
		HAL_StatusTypeDef it_status = HAL_UARTEx_ReceiveToIdle_IT(huart, buffer, max_size);
#if RS485_RX_DEBUG_ENABLE
		if (huart == &huart3) {
			static uint32_t dma_fail_count = 0U;
			dma_fail_count++;
			if ((dma_fail_count <= 5U)
					|| ((dma_fail_count % RS485_RX_DEBUG_LOG_EVERY_N) == 0U)) {
				debug.snprint("[USART3][RX_RESTART] DMA_FAIL=%u IT=%u max=%u cnt=%lu\r\n",
						(unsigned) dma_status, (unsigned) it_status, (unsigned) max_size,
						(unsigned long) dma_fail_count);
			}
		}
#endif
	} else if (huart->hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

#define STM32_BRIDGE_A6_HEADER 0xA6
void RS485_ProcessStream(UART_HandleTypeDef *huart, uint8_t *stream, uint16_t *len) {
	while (*len >= 3U) {
		if (stream[0] != STM32_BRIDGE_A6_HEADER) {
			// Ignore noise until header found.
			debug.snprint("[USART3][RX_STREAM] dropping byte 0x%02X\r\n", (unsigned) stream[0]);
			memmove(stream, stream + 1, --(*len));
			continue;
		}

		uint8_t payload_len = stream[1];
		uint8_t flags = stream[2];
		uint16_t header_len = 3U;
		if ((flags & RS485_MODBUS_FLAG_MSG_ID_PRESENT) != 0U) {
			header_len = (uint16_t) (header_len + 4U);
		}
		if ((flags & RS485_MODBUS_FLAG_TOKEN_PRESENT) != 0U) {
			header_len = (uint16_t) (header_len + 4U);
		}
		if ((flags & RS485_MODBUS_FLAG_IP_PRESENT) != 0U) {
			header_len = (uint16_t) (header_len + 4U);
		}
		uint16_t frame_len = header_len + payload_len;

		if (frame_len > RS485_STREAM_BUF_SIZE) {
			// malformed frame, discard header and retry.
			debug.snprint("[USART3][RX_STREAM] malformed frame_len=%u\r\n", (unsigned) frame_len);
			memmove(stream, stream + 1, --(*len));
			continue;
		}

		if (*len < frame_len) {
			break; // wait for more bytes
		}

		if (!STM32_RS485_BridgeHandleRx(huart, stream, frame_len)) {
			// we still advance to prevent deadlock on repeated headers.
			debug.snprint("[USART3][RX_STREAM] bridge rejected A6 frame\r\n");
		}

		memmove(stream, stream + frame_len, *len - frame_len);
		*len -= frame_len;
	}
}

void DMX_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
#if RS485_RX_DEBUG_ENABLE
	if (huart == &huart3) {
		static uint32_t event_count = 0U;
		event_count++;
		bool should_log = (event_count <= 10U)
				|| ((event_count % RS485_RX_DEBUG_LOG_EVERY_N) == 0U) || (Size == 0U);
		if (should_log) {
			char hex_preview[(RS485_RX_DEBUG_PREVIEW_BYTES * 3U) + 1U] = { 0 };
			RS485_DebugHexPreview(interrupt_data, Size, hex_preview, sizeof(hex_preview));
			debug.snprint("[USART3][RX_DMA] ev=%lu size=%u hex=%s%s\r\n",
					(unsigned long) event_count, (unsigned) Size,
					hex_preview,
					(Size > RS485_RX_DEBUG_PREVIEW_BYTES) ? " ..." : "");
		}
	}
#endif

	if (Size == 0) {
		RS485_RestartReceive(huart, interrupt_data, 500);
		return;
	}

	if (rs485_stream_len + Size > sizeof(rs485_stream_buf)) {
		// Overflow; reset parser state.
		rs485_stream_len = 0;
		debug.snprint("[USART3][RX_STREAM] overflow; dropping %u bytes\r\n", (unsigned) Size);
	}

	memcpy(rs485_stream_buf + rs485_stream_len, interrupt_data, Size);
	rs485_stream_len += Size;

	RS485_ProcessStream(huart, rs485_stream_buf, &rs485_stream_len);
	RS485_RestartReceive(huart, interrupt_data, 500);
}

class RS485 {
	public:
		UART_HandleTypeDef *uptr;
		DIG_PIN &rede;
		IRQn_Type irqn;

		RS485(UART_HandleTypeDef *uptr_i, DIG_PIN &rede_i, IRQn_Type irqn) :
				uptr(uptr_i), rede(rede_i), irqn(irqn) {
			rede.SET(0);
//			RS485_QHandle = osMessageQueueNew(16, sizeof(RS485_STC), &RS485_Q_attributes);
		}

		void INIT() {
			HAL_NVIC_EnableIRQ(irqn);
			HAL_StatusTypeDef cb_status = HAL_UART_RegisterRxEventCallback(uptr,
					DMX_RxEventCallback);

#if RS485_RX_DEBUG_ENABLE
			if (uptr == &huart3) {
				debug.snprint("[USART3][INIT] RxEventCallback=%u\r\n",
						(unsigned) cb_status);
			}
#endif
			RS485_RestartReceive(uptr, (uint8_t*) interrupt_data, 500);
		}

		void START_READ_IT() {
			RS485_RestartReceive(uptr, (uint8_t*) interrupt_data, 500);
		}

		String READ(uint32_t timeout) {
			String read_string;
//			char pData[100] = { 0 };
//			uint16_t read_len = 0;
			rede.SET(0); // open recieve
//			HAL_UART_Init(uptr);
			UART_ReceiveStringToIdle(uptr, read_string, timeout);

//			HAL_UARTEx_ReceiveToIdle(uptr, (uint8_t*) pData, 100, &read_len, timeout);
//			debug.Print_cstr(pData, read_len);
//			String read_string(pData, read_len);
//			if (read_string.size() > 0) {
//				rede.SET(0); // close recieve
			delay_ms(1);
//				debug.Print("Received : " + read_string + "\r\n");
//				return read_string;
//			}
//			while (read_string.back() == '\n' || read_string.back() == '\r') {
//				read_string.pop_back();
//			}
			return read_string;

		}

		uint16_t READ_RAW(uint8_t *buffer, uint16_t size, uint32_t timeout) {
			uint16_t received_len = 0;
			rede.SET(0); // open recieve
//			HAL_UART
			HAL_UARTEx_ReceiveToIdle(uptr, buffer, 1, &received_len, timeout);
			delay_ms(1);
			return received_len;
		}
		void SEND(String out) {
			rede.SET(1); // open transmit
			HAL_UART_Transmit(uptr, (uint8_t*) out.c_str(), out.size(), 1000);
			rede.SET(0);
		}

		void SEND(uint8_t *data, uint16_t size) {
			rede.SET(1); // open transmit
			HAL_UART_Transmit(uptr, data, size, 1000);
			rede.SET(0);
		}
		void SNPRINTF(const char *format, ...) {
			va_list args;
			va_start(args, format);
			char buffer[512];
			vsnprintf(buffer, 512, format, args);
			va_end(args);
			SEND(buffer);
		}

};

#endif /* RS485_H_ */
