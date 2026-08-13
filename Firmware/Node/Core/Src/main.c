/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "ipcc.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "node_swo_trace.h"
#include <stdarg.h>
#include <stdio.h>
#define EXO_LOG(...) NodeSwo_Logf(__VA_ARGS__)
#include <INCLUDER.h>
#ifndef EXO_NODE_FLASH_ENABLED
#define EXO_NODE_FLASH_ENABLED 1
#endif
#if EXO_NODE_FLASH_ENABLED
#include <NODE_RECORDING_APP.h>
#endif
#ifndef EXO_NODE_SENSOR_TEST_ENABLE
#define EXO_NODE_SENSOR_TEST_ENABLE 0
#endif
#if EXO_NODE_SENSOR_TEST_ENABLE
#include <HUB_SENSOR_TEST_APP.h>
#endif
#include <NODE_RUNTIME_CONFIG.h>
#include <BLE_RECORD_PROTOCOL.h>
#include "blepipe_proto.h"
#include "app_ble.h"
#include "custom_app.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_exti.h"
#include "stm32wbxx_ll_pwr.h"
#include "stm32wbxx_ll_rcc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef EXO_NODE_FLASH_ENABLED
#define EXO_NODE_FLASH_ENABLED 0
#endif

#ifndef EXO_PROFILE_DIAG
#define EXO_PROFILE_DIAG 0
#endif

#ifndef EXO_PROFILE_MINIMAL
#define EXO_PROFILE_MINIMAL (EXO_PROFILE_DIAG ? 0 : 1)
#endif

#ifndef EXO_NODE_BLE_FORWARD_ENABLE
#define EXO_NODE_BLE_FORWARD_ENABLE 1
#endif

#ifndef EXO_NODE_FLASH_TEST_BOOT_ENABLE
#if EXO_PROFILE_MINIMAL
#define EXO_NODE_FLASH_TEST_BOOT_ENABLE 0
#else
#define EXO_NODE_FLASH_TEST_BOOT_ENABLE 1
#endif
#endif

#ifndef EXO_NODE_FLASH_TEST_API_ENABLE
#if EXO_PROFILE_MINIMAL
#define EXO_NODE_FLASH_TEST_API_ENABLE 0
#else
#define EXO_NODE_FLASH_TEST_API_ENABLE 1
#endif
#endif

#ifndef EXO_NODE_ID
#define EXO_NODE_ID 1U
#endif

#ifndef EXO_BLE_LOG_LEVEL
#define EXO_BLE_LOG_LEVEL 2 /* 0=off,1=error,2=info,3=debug */
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if EXO_NODE_SENSOR_TEST_ENABLE
static exo::HubSensorTestApp hub_sensor_test_app(hi2c1, hi2c3, 0x4BU, 0x69U);
#endif

#if EXO_NODE_FLASH_ENABLED
static const exo::NodeRecordingConfig NODE_RECORDING_CONFIG = {
	EXO_NODE_ID,
	0x4BU,
	0x69U,
	0U,
	0U,
	0U,
};
static exo::NodeRecordingApp node_recording_app(NODE_RECORDING_CONFIG, hi2c1, hi2c3, hspi1,
		EXO_NODE_FLASH_CS_GPIO_Port, EXO_NODE_FLASH_CS_Pin);
#endif
static bool node_stream_enabled = false;
static uint8_t node_stream_interval_ms = 20U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static constexpr uint16_t kNodeSwoBufferSize = 1024U;
static constexpr size_t kNodeSwoMaxWriteSize = 256U;
static uint8_t g_node_swo_buffer[kNodeSwoBufferSize];
static volatile uint16_t g_node_swo_head = 0U;
static volatile uint16_t g_node_swo_tail = 0U;
static volatile uint32_t g_node_swo_dropped_bytes = 0U;

extern "C" void NodeSwo_Init(uint32_t core_clock_hz, uint32_t swo_clock_hz)
{
	if ((core_clock_hz == 0U) || (swo_clock_hz == 0U)
			|| (core_clock_hz < swo_clock_hz)) {
		return;
	}

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio_init = {};
	gpio_init.Pin = GPIO_PIN_3;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init.Alternate = GPIO_AF0_JTD_TRACE;
	HAL_GPIO_Init(GPIOB, &gpio_init);

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
	ITM->LAR = 0xC5ACCE55UL;
	ITM->TER = 0U;
	ITM->TCR = 0U;
	TPI->SPPR = 2U;
	TPI->ACPR = (core_clock_hz / swo_clock_hz) - 1U;
	TPI->FFCR = 0x00000100U;
	ITM->TPR = 0U;
	ITM->TER = 1UL;
	ITM->TCR = (1UL << ITM_TCR_TraceBusID_Pos)
			| ITM_TCR_SWOENA_Msk
			| ITM_TCR_DWTENA_Msk
			| ITM_TCR_SYNCENA_Msk
			| ITM_TCR_ITMENA_Msk;
	__DSB();
	__ISB();
}

extern "C" uint8_t NodeSwo_TryWrite(uint8_t value)
{
	if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
			|| ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U)
			|| ((ITM->TER & 1UL) == 0U)
			|| (ITM->PORT[0U].u32 == 0U)) {
		return 0U;
	}

	ITM->PORT[0U].u8 = value;
	return 1U;
}

extern "C" size_t NodeSwo_Write(const uint8_t *data, size_t size)
{
	if ((data == nullptr) || (size == 0U)) {
		return 0U;
	}
	if (size > kNodeSwoMaxWriteSize) {
		const uint32_t interrupt_state = __get_PRIMASK();
		__disable_irq();
		g_node_swo_dropped_bytes += static_cast<uint32_t>(size);
		if (interrupt_state == 0U) {
			__enable_irq();
		}
		return 0U;
	}

	const uint32_t interrupt_state = __get_PRIMASK();
	__disable_irq();

	const uint16_t head = g_node_swo_head;
	const uint16_t tail = g_node_swo_tail;
	const size_t used = (head >= tail)
			? static_cast<size_t>(head - tail)
			: static_cast<size_t>(kNodeSwoBufferSize - (tail - head));
	const size_t available = (kNodeSwoBufferSize - 1U) - used;
	if (size > available) {
		g_node_swo_dropped_bytes += static_cast<uint32_t>(size);
		if (interrupt_state == 0U) {
			__enable_irq();
		}
		return 0U;
	}

	for (size_t accepted = 0U; accepted < size; ++accepted) {
		const uint16_t next_head =
				static_cast<uint16_t>((g_node_swo_head + 1U) % kNodeSwoBufferSize);
		g_node_swo_buffer[g_node_swo_head] = data[accepted];
		g_node_swo_head = next_head;
	}

	if (interrupt_state == 0U) {
		__enable_irq();
	}
	return size;
}

extern "C" void NodeSwo_Process(void)
{
	while (g_node_swo_tail != g_node_swo_head) {
		const uint8_t value = g_node_swo_buffer[g_node_swo_tail];
		if (NodeSwo_TryWrite(value) == 0U) {
			break;
		}
		g_node_swo_tail =
				static_cast<uint16_t>((g_node_swo_tail + 1U) % kNodeSwoBufferSize);
	}
}

extern "C" void NodeSwo_Logf(const char *format, ...)
{
	if (format == nullptr) {
		return;
	}

	char message[224];
	va_list args;
	va_start(args, format);
	const int length = vsnprintf(message, sizeof(message), format, args);
	va_end(args);

	if (length <= 0) {
		return;
	}

	const size_t bytes_to_write =
			(static_cast<size_t>(length) < sizeof(message))
			? static_cast<size_t>(length)
			: (sizeof(message) - 1U);
	(void)NodeSwo_Write(reinterpret_cast<const uint8_t *>(message), bytes_to_write);
}

PWM_PIN ERM_PWM(&htim1, TIM_CHANNEL_3, BUZZER_GPIO_Port, BUZZER_Pin);
PWM_PIN BUZZER(&htim1, TIM_CHANNEL_4, ERM_GPIO_Port, ERM_Pin);

RGB_LED RGB(RGB_R_GPIO_Port, RGB_R_Pin, RGB_G_GPIO_Port, RGB_G_Pin, RGB_B_GPIO_Port, RGB_B_Pin, 1, 1);

static uint32_t g_node_blepipe_tx_seq = 1U;
static uint32_t g_node_record_ready_last_status_ms = 0U;
static uint8_t g_node_record_ready_last_notify_enabled = 0U;
static uint8_t g_node_record_start_heartbeat_phase = 0U;
static uint32_t g_node_record_start_heartbeat_last_ms = 0U;
static bool g_node_touch_test_active = false;
static bool g_node_poweroff_test_pending = false;
static uint32_t g_node_touch_test_until_ms = 0U;
static uint32_t g_node_poweroff_test_at_ms = 0U;
static constexpr uint8_t kBlepipeStatusKindTouch = 0x03U;
static constexpr uint8_t kTouchStatusReleased = 0U;
static constexpr uint8_t kTouchStatusPressed = 1U;
static constexpr uint8_t kTouchStatusShutdownArmed = 2U;
static constexpr uint8_t kTouchStatusTurningOff = 3U;
static constexpr uint16_t kNodeRecordChunkPayloadBytes = exo::kRecordReliableDefaultChunkSize;
static constexpr uint32_t kNodeRecordChunkGapMs = 8U;
static constexpr uint8_t kNodeRecordBurstLimit = 1U;
static constexpr uint32_t kNodeRecordDoneRetryMs = 500U;
static constexpr uint32_t kNodeRecordTxFailLogMs = 500U;
/* An upload that has run out of credit is only restarted by an inbound control
 * frame. If that frame is lost the transfer deadlocks until the receiver's
 * session watchdog fires, so re-arm a single chunk after this long to give the
 * receiver something fresh to ACK. */
static constexpr uint32_t kNodeRecordCreditStarveMs = 500U;
static bool g_node_record_done_sent = false;
static bool g_node_upload_active = false;
static uint32_t g_node_upload_session_id = 0U;
static uint32_t g_node_upload_total_size = 0U;
static uint32_t g_node_upload_crc32 = 0U;
static uint32_t g_node_upload_next_chunk = 0U;
static uint8_t g_node_upload_credit = 0U;
/* Chunks still owed to an accepted NackRange. While non-zero the send cursor is
 * owned by gap recovery and must not be dragged forward by an ACK_WINDOW. */
static uint8_t g_node_upload_retx_remaining = 0U;
static uint32_t g_node_upload_credit_idle_ms = 0U;
static uint32_t g_node_upload_last_chunk_ms = 0U;
static uint32_t g_node_upload_last_fail_log_ms = 0U;
static uint32_t g_node_record_done_last_send_ms = 0U;
static uint16_t g_node_record_done_retry_count = 0U;
static volatile bool g_node_actuator_override_enabled = false;
static volatile uint8_t g_node_rgb_mask = 0U;

extern "C" uint8_t Custom_APP_PipeDataNotifyEnabled(void);
extern "C" uint8_t exo_node_ble_status_notify_enabled(void);

static uint16_t node_blepipe_current_id()
{
#if EXO_NODE_FLASH_ENABLED
	/* Cached in RAM: this is on the path of every BLE frame header and every
	 * upload chunk, and it must never write to flash. */
	return static_cast<uint16_t>(exo::node_runtime_config::current_node_id(EXO_NODE_ID));
#else
	return static_cast<uint16_t>(EXO_NODE_ID);
#endif
}

static bool node_blepipe_send_with_status(Custom_STM_Char_Opcode_t char_opcode,
		uint8_t msg_type,
		uint16_t dst_id,
		const uint8_t *payload,
		uint16_t payload_len,
		size_t *encoded_len_out,
		tBleStatus *tx_status_out)
{
	uint8_t packet[BLEPIPE_MAX_NOTIFY_PAYLOAD];
	size_t encoded_len = 0U;
	blepipe_hdr_t hdr{};
	hdr.proto_ver = BLEPIPE_PROTO_VER;
	hdr.msg_type = msg_type;
	hdr.flags = 0U;
	hdr.hop_count = 0U;
	hdr.src_id = node_blepipe_current_id();
	hdr.dst_id = dst_id;
	hdr.seq = g_node_blepipe_tx_seq++;
	hdr.timestamp_ms = HAL_GetTick();
	hdr.payload_len = payload_len;
	const blepipe_status_t status = blepipe_encode(packet,
			sizeof(packet),
			&hdr,
			payload,
			payload_len,
			&encoded_len);
	if (encoded_len_out != nullptr) {
		*encoded_len_out = encoded_len;
	}
	if (status != BLEPIPE_STATUS_OK || encoded_len > 255U) {
		if (tx_status_out != nullptr) {
			*tx_status_out = BLE_STATUS_INVALID_PARAMS;
		}
		EXO_LOG("[BLEPIPE][NODE] encode failed msg=0x%02X status=%u len=%u\r\n",
				static_cast<unsigned>(msg_type),
				static_cast<unsigned>(status),
				static_cast<unsigned>(payload_len));
		return false;
	}
	const tBleStatus tx_status = Custom_APP_SendPipeFrame(char_opcode,
			packet,
			static_cast<uint8_t>(encoded_len));
	if (tx_status_out != nullptr) {
		*tx_status_out = tx_status;
	}
	return tx_status == BLE_STATUS_SUCCESS;
}

static bool node_blepipe_send(Custom_STM_Char_Opcode_t char_opcode,
		uint8_t msg_type,
		uint16_t dst_id,
		const uint8_t *payload,
		uint16_t payload_len)
{
	return node_blepipe_send_with_status(char_opcode,
			msg_type,
			dst_id,
			payload,
			payload_len,
			nullptr,
			nullptr);
}

static void node_blepipe_process_live_samples()
{
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
	if (Custom_APP_PipeDataNotifyEnabled() == 0U) {
		return;
	}
	exo::NodeRecordingApp::LiveSample sample{};
	for (uint8_t sent = 0U; sent < 2U && node_recording_app.peek_live_sample(sample); ++sent) {
		uint8_t payload[1U + exo::NodeRecordingApp::kMaxLivePayload]{};
		payload[0] = sample.sensor_id;
		memcpy(payload + 1U, sample.payload, sample.payload_len);
		if (!node_blepipe_send(CUSTOM_STM_PIPEDATATX,
				BLEPIPE_MSG_LEAF_SAMPLE,
				BLEPIPE_ID_HUB,
				payload,
				static_cast<uint16_t>(sample.payload_len + 1U))) {
			// The BLE stack commonly accepts only one notification at a time.
			// Retain this sample and retry on the next process pass instead of
			// turning transient backpressure into a hole in the live plot.
			break;
		}
		(void)node_recording_app.discard_live_sample();
	}
#endif
}

static void node_blepipe_send_touch_status(uint8_t state)
{
	if (APP_BLE_Get_Server_Connection_Status() != APP_BLE_CONNECTED_SERVER ||
			exo_node_ble_status_notify_enabled() == 0U) {
		return;
	}
	const uint8_t payload[3] = {
		kBlepipeStatusKindTouch,
		static_cast<uint8_t>(node_blepipe_current_id()),
		state
	};
	(void)node_blepipe_send(CUSTOM_STM_PIPESTATTX,
			BLEPIPE_MSG_STATUS,
			BLEPIPE_ID_HUB,
			payload,
			static_cast<uint16_t>(sizeof(payload)));
}

static void node_prepare_touch_wakeup_before_poweroff()
{
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
	LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
	LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);
}

static void node_poweroff_pcb_and_wait_for_release()
{
	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
	ERM_PWM.SET_PERCENT(0);
	BUZZER.SET_PERCENT(0);
	RGB.OFF();

	while (HAL_GPIO_ReadPin(TOUCH_MCU_GPIO_Port, TOUCH_MCU_Pin) == GPIO_PIN_SET) {
		HAL_Delay(10);
	}
	HAL_Delay(10000);

	NVIC_SystemReset();
}

static void node_blepipe_send_ack(const blepipe_hdr_t &request_hdr, uint8_t accepted, uint8_t command_id)
{
	const uint8_t payload[2] = { command_id, accepted };
	(void)node_blepipe_send(CUSTOM_STM_PIPECTRLTX,
			accepted ? BLEPIPE_MSG_ACK : BLEPIPE_MSG_NACK,
			request_hdr.src_id,
			payload,
			static_cast<uint16_t>(sizeof(payload)));
}

static void node_blepipe_send_response(const blepipe_hdr_t &request_hdr,
		const uint8_t *payload,
		uint16_t payload_len)
{
	(void)node_blepipe_send(CUSTOM_STM_PIPECTRLTX,
			BLEPIPE_MSG_COMMAND_RESP,
			request_hdr.src_id,
			payload,
			payload_len);
}

static bool node_blepipe_send_record_payload(const uint8_t *payload, uint16_t payload_len)
{
	return node_blepipe_send(CUSTOM_STM_PIPEDATATX,
			BLEPIPE_MSG_RAW_FORWARD,
			BLEPIPE_ID_HUB,
			payload,
			payload_len);
}

static bool node_blepipe_send_record_payload_with_status(const uint8_t *payload,
		uint16_t payload_len,
		size_t *encoded_len_out,
		tBleStatus *tx_status_out)
{
	return node_blepipe_send_with_status(CUSTOM_STM_PIPEDATATX,
			BLEPIPE_MSG_RAW_FORWARD,
			BLEPIPE_ID_HUB,
			payload,
			payload_len,
			encoded_len_out,
			tx_status_out);
}

static bool node_blepipe_send_reliable_frame(exo::RecordReliableType type,
		uint16_t source_id,
		uint32_t session_id,
		uint32_t chunk_index,
		uint32_t byte_offset,
		const uint8_t *payload,
		uint16_t payload_len,
		uint16_t flags = 0U,
		size_t *encoded_len_out = nullptr,
		tBleStatus *tx_status_out = nullptr)
{
	uint8_t packet[244];
	exo::RecordReliableFrameHeader hdr{};
	hdr.command = exo::RecordCommand::ReliableFrame;
	hdr.proto_version = exo::kRecordReliableProtoVersion;
	hdr.magic = exo::kRecordReliableMagic;
	hdr.frame_type = static_cast<uint8_t>(type);
	hdr.source_id = source_id;
	hdr.session_id = session_id;
	hdr.chunk_index = chunk_index;
	hdr.byte_offset = byte_offset;
	hdr.payload_len = payload_len;
	hdr.payload_crc16 = blepipe_crc16_ccitt(payload, payload_len);
	hdr.flags = flags;
	const uint16_t total = static_cast<uint16_t>(sizeof(hdr) + payload_len);
	if (total > sizeof(packet)) {
		return false;
	}
	memcpy(packet, &hdr, sizeof(hdr));
	if (payload != nullptr && payload_len > 0U) {
		memcpy(packet + sizeof(hdr), payload, payload_len);
	}
	return node_blepipe_send_record_payload_with_status(packet,
			total,
			encoded_len_out,
			tx_status_out);
}

static void node_blepipe_reset_upload_state()
{
	g_node_record_done_sent = false;
	g_node_upload_active = false;
	g_node_upload_session_id = 0U;
	g_node_upload_total_size = 0U;
	g_node_upload_crc32 = 0U;
	g_node_upload_next_chunk = 0U;
	g_node_upload_credit = 0U;
	g_node_upload_retx_remaining = 0U;
	g_node_upload_credit_idle_ms = 0U;
	g_node_upload_last_chunk_ms = 0U;
	g_node_upload_last_fail_log_ms = 0U;
	g_node_record_done_last_send_ms = 0U;
	g_node_record_done_retry_count = 0U;
}

static bool node_blepipe_apply_legacy_chunk_ack(const uint8_t *payload, uint8_t length)
{
	if (payload == nullptr || length < sizeof(exo::ChunkAckMessage)) {
		return false;
	}

	uint32_t session_id = 0U;
	uint16_t source_id = 0U;
	uint32_t next_offset = 0U;
	if (length >= sizeof(exo::ChunkAckCompactSourceMessage) &&
	    payload[1] == 4U) {
		exo::ChunkAckCompactSourceMessage ack{};
		memcpy(&ack, payload, sizeof(ack));
		session_id = ack.session_id;
		source_id = ack.source_id;
		next_offset = ack.next_offset;
	} else if (length >= sizeof(exo::ChunkAckCompactMessage) &&
	           payload[1] == 4U) {
		exo::ChunkAckCompactMessage ack{};
		memcpy(&ack, payload, sizeof(ack));
		session_id = ack.session_id;
		next_offset = ack.next_offset;
	} else if (length >= sizeof(exo::ChunkAckV3Message)) {
		exo::ChunkAckV3Message ack{};
		memcpy(&ack, payload, sizeof(ack));
		session_id = ack.session_id;
		source_id = ack.source_id;
		next_offset = ack.next_offset;
	} else if (length >= sizeof(exo::ChunkAckRangeMessage)) {
		exo::ChunkAckRangeMessage ack{};
		memcpy(&ack, payload, sizeof(ack));
		session_id = ack.session_id;
		next_offset = ack.next_offset;
	} else {
		exo::ChunkAckMessage ack{};
		memcpy(&ack, payload, sizeof(ack));
		session_id = ack.session_id;
		next_offset = ack.next_offset;
	}

	if ((source_id != 0U && source_id != node_blepipe_current_id()) ||
	    session_id != g_node_upload_session_id) {
		return false;
	}

	const bool upload_ready = node_recording_app.session_ready() || node_recording_app.state() == exo::RecorderState::Uploading;
	if (!upload_ready) {
		return false;
	}
	if (node_recording_app.state() != exo::RecorderState::Uploading &&
	    !node_recording_app.begin_upload()) {
		return false;
	}
	if (g_node_upload_total_size == 0U) {
		g_node_upload_total_size = node_recording_app.make_upload_reader().total_size();
		g_node_upload_crc32 = node_recording_app.make_record_done().payload_crc32;
	}
	g_node_upload_active = true;
	g_node_upload_next_chunk = next_offset / static_cast<uint32_t>(kNodeRecordChunkPayloadBytes);
	g_node_upload_credit = exo::kRecordReliableDefaultCredit;
	return true;
}

static void node_blepipe_process_recording_upload()
{
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
	if (node_recording_app.session_ready() && !g_node_record_done_sent && !g_node_upload_active &&
	    (g_node_record_done_last_send_ms == 0U ||
	     (HAL_GetTick() - g_node_record_done_last_send_ms) >= kNodeRecordDoneRetryMs)) {
		const exo::RecordDoneMessage done = node_recording_app.make_record_done();
		size_t encoded_len = 0U;
		tBleStatus tx_status = BLE_STATUS_INVALID_PARAMS;
		const bool sent = node_blepipe_send_record_payload_with_status(reinterpret_cast<const uint8_t *>(&done),
				static_cast<uint16_t>(sizeof(done)),
				&encoded_len,
				&tx_status);
		g_node_record_done_last_send_ms = HAL_GetTick();
		++g_node_record_done_retry_count;
		if (sent) {
			g_node_upload_session_id = done.session_id;
			g_node_upload_total_size = done.total_size;
			g_node_upload_crc32 = done.payload_crc32;
		}
		EXO_LOG("[BLE][NODE][REC] RecordDone send attempt=%u sent=%u status=0x%02X enc_len=%u data_notify=%u node=%u session=%lu size=%lu\r\n",
				static_cast<unsigned>(g_node_record_done_retry_count),
				static_cast<unsigned>(sent ? 1U : 0U),
				static_cast<unsigned>(tx_status),
				static_cast<unsigned>(encoded_len),
				static_cast<unsigned>(Custom_APP_PipeDataNotifyEnabled()),
				static_cast<unsigned>(done.node_id),
				static_cast<unsigned long>(done.session_id),
				static_cast<unsigned long>(done.total_size));
	}

	if (!g_node_upload_active) {
		return;
	}

	if (g_node_upload_credit == 0U) {
		/* Credit is only replenished by an inbound ACK_WINDOW/NACK_RANGE. If that
		 * frame never lands the upload would stay silent forever, so re-arm one
		 * chunk periodically. This self-limits to one chunk per starve interval
		 * because the credit drops straight back to zero after it is spent. */
		const uint32_t now = HAL_GetTick();
		if (g_node_upload_credit_idle_ms == 0U) {
			g_node_upload_credit_idle_ms = now;
			return;
		}
		if ((now - g_node_upload_credit_idle_ms) < kNodeRecordCreditStarveMs) {
			return;
		}
		g_node_upload_credit_idle_ms = now;
		g_node_upload_credit = 1U;
		EXO_LOG("[BLE][NODE][REC] credit starved, re-arming session=%lu chunk=%lu retx=%u\r\n",
				static_cast<unsigned long>(g_node_upload_session_id),
				static_cast<unsigned long>(g_node_upload_next_chunk),
				static_cast<unsigned>(g_node_upload_retx_remaining));
	}

	if ((HAL_GetTick() - g_node_upload_last_chunk_ms) < kNodeRecordChunkGapMs) {
		return;
	}

	uint8_t burst_sent = 0U;
	while (g_node_upload_active &&
	       g_node_upload_credit > 0U &&
	       burst_sent < kNodeRecordBurstLimit) {
		const uint32_t offset = g_node_upload_next_chunk * static_cast<uint32_t>(kNodeRecordChunkPayloadBytes);
		if (offset >= g_node_upload_total_size) {
			g_node_upload_active = false;
			g_node_upload_credit = 0U;
			g_node_upload_retx_remaining = 0U;
			g_node_upload_credit_idle_ms = 0U;
			return;
		}

		uint8_t chunk[kNodeRecordChunkPayloadBytes];
		const uint32_t remaining = g_node_upload_total_size - offset;
		const uint16_t chunk_size = static_cast<uint16_t>(remaining > kNodeRecordChunkPayloadBytes ?
				kNodeRecordChunkPayloadBytes : remaining);
		exo::SessionUploadReader reader = node_recording_app.make_upload_reader();
		if (!reader.read(offset, chunk, chunk_size)) {
			EXO_LOG("[BLE][NODE][REC] chunk read failed session=%lu off=%lu size=%u\r\n",
					static_cast<unsigned long>(g_node_upload_session_id),
					static_cast<unsigned long>(offset),
					static_cast<unsigned>(chunk_size));
			g_node_upload_credit = 0U;
			return;
		}

		size_t encoded_len = 0U;
		tBleStatus tx_status = BLE_STATUS_INVALID_PARAMS;
		if (!node_blepipe_send_reliable_frame(exo::RecordReliableType::Chunk,
				static_cast<uint16_t>(node_blepipe_current_id()),
				g_node_upload_session_id,
				g_node_upload_next_chunk,
				offset,
				chunk,
				chunk_size,
				((offset + chunk_size) >= g_node_upload_total_size) ? exo::kRecordFlagFinalChunk : 0U,
				&encoded_len,
				&tx_status)) {
			const uint32_t now = HAL_GetTick();
			g_node_upload_last_chunk_ms = now;
			if (g_node_upload_last_fail_log_ms == 0U ||
			    (now - g_node_upload_last_fail_log_ms) >= kNodeRecordTxFailLogMs) {
				g_node_upload_last_fail_log_ms = now;
				EXO_LOG("[BLE][NODE][REC] chunk tx busy session=%lu chunk=%lu off=%lu size=%u credit=%u enc_len=%u status=0x%02X\r\n",
						static_cast<unsigned long>(g_node_upload_session_id),
						static_cast<unsigned long>(g_node_upload_next_chunk),
						static_cast<unsigned long>(offset),
						static_cast<unsigned>(chunk_size),
						static_cast<unsigned>(g_node_upload_credit),
						static_cast<unsigned>(encoded_len),
						static_cast<unsigned>(tx_status));
			}
			return;
		}
		g_node_upload_next_chunk++;
		g_node_upload_credit--;
		if (g_node_upload_retx_remaining > 0U) {
			--g_node_upload_retx_remaining;
		}
		if (g_node_upload_credit == 0U) {
			g_node_upload_credit_idle_ms = HAL_GetTick();
		}
		g_node_upload_last_chunk_ms = HAL_GetTick();
		g_node_upload_last_fail_log_ms = 0U;
		++burst_sent;
	}
#endif
}

static void node_blepipe_send_topology(const blepipe_hdr_t &request_hdr)
{
	const uint8_t current_id = static_cast<uint8_t>(node_blepipe_current_id());
	const uint8_t payload[2] = { 1U, current_id };
	(void)node_blepipe_send(CUSTOM_STM_PIPESTATTX,
			BLEPIPE_MSG_TOPOLOGY,
			request_hdr.src_id,
			payload,
			static_cast<uint16_t>(sizeof(payload)));
}

static void node_blepipe_send_stream_status(const blepipe_hdr_t &request_hdr,
		uint8_t command_id)
{
	const uint8_t payload[4] = {
		command_id,
		static_cast<uint8_t>(node_blepipe_current_id()),
		node_stream_enabled ? 1U : 0U,
		node_stream_interval_ms
	};
	node_blepipe_send_response(request_hdr, payload, static_cast<uint16_t>(sizeof(payload)));
}

static bool node_blepipe_send_record_ready_status(bool force)
{
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
	const uint32_t now_ms = HAL_GetTick();
	const uint8_t notify_enabled = exo_node_ble_status_notify_enabled();
	if (notify_enabled == 0U) {
		g_node_record_ready_last_notify_enabled = 0U;
		return false;
	}
	if (!force &&
	    g_node_record_ready_last_notify_enabled != 0U &&
	    (now_ms - g_node_record_ready_last_status_ms) < 1000U) {
		return false;
	}

	blepipe_node_record_ready_status_t status{};
	status.status_kind = BLEPIPE_STATUS_KIND_NODE_RECORD_READY;
	status.node_id = static_cast<uint8_t>(node_blepipe_current_id());
	status.recorder_state = static_cast<uint8_t>(node_recording_app.state());
	status.record_ready = (node_recording_app.ready() &&
			(node_recording_app.can_start_recording() ||
			node_recording_app.state() == exo::RecorderState::Idle)) ? 1U : 0U;
	status.session_id = node_recording_app.session_ready() ?
			node_recording_app.make_record_done().session_id :
			0U;
	status.maximum_duration_ms = node_recording_app.maximum_duration_ms();

	const bool sent = node_blepipe_send(CUSTOM_STM_PIPESTATTX,
			BLEPIPE_MSG_STATUS,
			BLEPIPE_ID_HUB,
			reinterpret_cast<const uint8_t *>(&status),
			static_cast<uint16_t>(sizeof(status)));
	g_node_record_ready_last_status_ms = now_ms;
	g_node_record_ready_last_notify_enabled = notify_enabled;
	EXO_LOG("[BLE][NODE][READY] sent=%u ready=%u state=%u session=%lu\r\n",
			static_cast<unsigned>(sent ? 1U : 0U),
			static_cast<unsigned>(status.record_ready),
			static_cast<unsigned>(status.recorder_state),
			static_cast<unsigned long>(status.session_id));
	return sent;
#else
	(void)force;
	return false;
#endif
}

static bool node_blepipe_send_record_start_heartbeat(uint8_t phase, uint32_t session_id, bool force)
{
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
	const uint32_t now_ms = HAL_GetTick();
	if (exo_node_ble_status_notify_enabled() == 0U) {
		return false;
	}
	if (!force &&
	    g_node_record_start_heartbeat_phase == phase &&
	    (now_ms - g_node_record_start_heartbeat_last_ms) < 1000U) {
		return false;
	}
	blepipe_record_start_heartbeat_status_t status{};
	status.status_kind = BLEPIPE_STATUS_KIND_RECORD_START_HEARTBEAT;
	status.phase = phase;
	status.in_progress = 1U;
	status.source_id = node_blepipe_current_id();
	status.session_id = session_id;
	status.extend_timeout_ms = 5000U;
	const bool sent = node_blepipe_send(CUSTOM_STM_PIPESTATTX,
			BLEPIPE_MSG_STATUS,
			BLEPIPE_ID_HUB,
			reinterpret_cast<const uint8_t *>(&status),
			static_cast<uint16_t>(sizeof(status)));
	g_node_record_start_heartbeat_phase = phase;
	g_node_record_start_heartbeat_last_ms = now_ms;
	EXO_LOG("[BLE][NODE][START_HB] sent=%u phase=%u session=%lu\r\n",
			static_cast<unsigned>(sent ? 1U : 0U),
			static_cast<unsigned>(phase),
			static_cast<unsigned long>(session_id));
	return sent;
#else
	(void)phase;
	(void)session_id;
	(void)force;
	return false;
#endif
}

static uint8_t node_clamp_u8(uint32_t value, uint8_t min_value, uint8_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return static_cast<uint8_t>(value);
}

static bool node_apply_actuator_command(const uint8_t *payload, uint16_t length)
{
	if (payload == nullptr || length < 2U) {
		return false;
	}
	switch (payload[0]) {
		case 0xA3U:
			ERM_PWM.SET_PERCENT(node_clamp_u8(payload[1], 0U, 100U));
			g_node_actuator_override_enabled = true;
			EXO_LOG("[BLE][NODE][CTRL] ERM=%u%%\r\n", static_cast<unsigned>(node_clamp_u8(payload[1], 0U, 100U)));
			return true;
		case 0xA4U:
			BUZZER.SET_PERCENT(node_clamp_u8(payload[1], 0U, 99U));
			g_node_actuator_override_enabled = true;
			EXO_LOG("[BLE][NODE][CTRL] buzzer=%u%%\r\n", static_cast<unsigned>(node_clamp_u8(payload[1], 0U, 99U)));
			return true;
		case 0xA5U:
			if ((payload[1] & 0x80U) != 0U) {
				g_node_actuator_override_enabled = false;
				EXO_LOG("[BLE][NODE][CTRL] actuator override=OFF\r\n");
				return true;
			}
			g_node_rgb_mask = static_cast<uint8_t>(payload[1] & 0x07U);
			g_node_actuator_override_enabled = true;
			RGB.SET((g_node_rgb_mask & 0x01U) != 0U,
					(g_node_rgb_mask & 0x02U) != 0U,
					(g_node_rgb_mask & 0x04U) != 0U);
			EXO_LOG("[BLE][NODE][CTRL] RGB mask=0x%02X\r\n", static_cast<unsigned>(g_node_rgb_mask));
			return true;
		case 0xA6U:
			if (payload[1] == 0U) {
				g_node_actuator_override_enabled = true;
				g_node_touch_test_active = true;
				g_node_touch_test_until_ms = HAL_GetTick() + 500U;
				BUZZER.SET_PERCENT(50);
				RGB.SET(true, true, true);
				node_blepipe_send_touch_status(kTouchStatusPressed);
				EXO_LOG("[BLE][NODE][CTRL] touch test feedback\r\n");
				return true;
			}
			if (payload[1] == 1U) {
				g_node_actuator_override_enabled = true;
				g_node_touch_test_active = false;
				g_node_poweroff_test_pending = true;
				g_node_poweroff_test_at_ms = HAL_GetTick() + 1000U;
				ERM_PWM.SET_PERCENT(0);
				BUZZER.SET_PERCENT(0);
				RGB.SET(true, false, false);
				node_blepipe_send_touch_status(kTouchStatusTurningOff);
				EXO_LOG("[BLE][NODE][CTRL] poweroff test armed\r\n");
				return true;
			}
			return false;
		default:
			return false;
	}
}

static bool node_handle_blepipe_command(const blepipe_hdr_t &hdr,
		const uint8_t *payload,
		uint16_t length)
{
	if (payload == nullptr || length == 0U) {
		EXO_LOG("[BLEPIPE][NODE][CMD] empty msg=0x%02X src=0x%04X dst=0x%04X\r\n",
				static_cast<unsigned>(hdr.msg_type),
				static_cast<unsigned>(hdr.src_id),
				static_cast<unsigned>(hdr.dst_id));
		node_blepipe_send_ack(hdr, 0U, 0U);
		return false;
	}
	EXO_LOG("[BLEPIPE][NODE][CMD] msg=0x%02X cmd=0x%02X src=0x%04X dst=0x%04X len=%u node=%u\r\n",
			static_cast<unsigned>(hdr.msg_type),
			static_cast<unsigned>(payload[0]),
			static_cast<unsigned>(hdr.src_id),
			static_cast<unsigned>(hdr.dst_id),
			static_cast<unsigned>(length),
			static_cast<unsigned>(node_blepipe_current_id()));
	switch (payload[0]) {
	case 0xA0U:
		if (length == 1U) {
			node_stream_enabled = true;
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
			node_recording_app.set_live_stream_enabled(true);
#endif
			node_blepipe_send_stream_status(hdr, payload[0]);
			node_blepipe_send_ack(hdr, 1U, payload[0]);
			return true;
		}
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xA1U:
		if (length == 1U) {
			node_stream_enabled = false;
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
			node_recording_app.set_live_stream_enabled(false);
#endif
			node_blepipe_send_stream_status(hdr, payload[0]);
			node_blepipe_send_ack(hdr, 1U, payload[0]);
			return true;
		}
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xA2U:
		if (length >= 2U) {
			node_stream_interval_ms = payload[1] == 0U ? 1U : payload[1];
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
			node_recording_app.set_live_interval_ms(payload[1]);
#endif
			node_blepipe_send_stream_status(hdr, payload[0]);
			node_blepipe_send_ack(hdr, 1U, payload[0]);
			return true;
		}
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xA3U:
	case 0xA4U:
	case 0xA5U:
	case 0xA6U:
	{
		const bool ok = node_apply_actuator_command(payload, length);
		node_blepipe_send_ack(hdr, ok ? 1U : 0U, payload[0]);
		return ok;
	}
	case static_cast<uint8_t>(exo::RecordCommand::StartRecord):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			EXO_LOG("[BLE][NODE][START] session=%lu lead_us=%lu duration_ms=%lu\r\n",
					static_cast<unsigned long>(message.session_id),
					static_cast<unsigned long>(message.start_timestamp_us),
					static_cast<unsigned long>(message.requested_duration_ms));
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_PREPARE, message.session_id, true);
			node_blepipe_reset_upload_state();
			const uint8_t ok = node_recording_app.start_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][NODE][START] result=%u state=%u\r\n",
					static_cast<unsigned>(ok),
					static_cast<unsigned>(node_recording_app.state()));
			(void)node_blepipe_send_record_ready_status(true);
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
		EXO_LOG("[BLE][NODE][START] bad len=%u expect=%u\r\n",
				static_cast<unsigned>(length),
				static_cast<unsigned>(sizeof(exo::StartRecordMessage)));
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case static_cast<uint8_t>(exo::RecordCommand::PrepareRecord):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			EXO_LOG("[BLE][NODE][PREP] session=%lu lead_us=%lu duration_ms=%lu\r\n",
					static_cast<unsigned long>(message.session_id),
					static_cast<unsigned long>(message.start_timestamp_us),
					static_cast<unsigned long>(message.requested_duration_ms));
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_PREPARE, message.session_id, true);
			node_blepipe_reset_upload_state();
			const uint8_t ok = node_recording_app.prepare_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][NODE][PREP] result=%u state=%u\r\n",
					static_cast<unsigned>(ok),
					static_cast<unsigned>(node_recording_app.state()));
			(void)node_blepipe_send_record_ready_status(true);
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
		EXO_LOG("[BLE][NODE][PREP] bad len=%u expect=%u\r\n",
				static_cast<unsigned>(length),
				static_cast<unsigned>(sizeof(exo::StartRecordMessage)));
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case static_cast<uint8_t>(exo::RecordCommand::CommitPreparedRecord):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_COMMIT, message.session_id, true);
			const uint8_t ok = node_recording_app.commit_prepared_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][NODE][COMMIT] session=%lu result=%u state=%u\r\n",
					static_cast<unsigned long>(message.session_id),
					static_cast<unsigned>(ok),
					static_cast<unsigned>(node_recording_app.state()));
			(void)node_blepipe_send_record_ready_status(true);
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case static_cast<uint8_t>(exo::RecordCommand::AbortPreparedRecord):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_ABORT, 0U, true);
		node_recording_app.abort_prepared_recording();
		node_blepipe_reset_upload_state();
		EXO_LOG("[BLE][NODE][ABORT_PREP]\r\n");
		(void)node_blepipe_send_record_ready_status(true);
		node_blepipe_send_ack(hdr, 1U, payload[0]);
		return true;
#else
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
#endif
	case static_cast<uint8_t>(exo::RecordCommand::StopRecord):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == sizeof(exo::StopRecordMessage)) {
			exo::StopRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			const uint8_t ok = node_recording_app.stop_recording(message) ? 1U : 0U;
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == sizeof(exo::SessionCompleteAckMessage)) {
			const uint8_t ok = (node_recording_app.transfer_complete() && node_recording_app.acknowledge_and_erase()) ? 1U : 0U;
			if (ok != 0U) {
				node_blepipe_reset_upload_state();
			}
			(void)node_blepipe_send_record_ready_status(true);
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case static_cast<uint8_t>(exo::RecordCommand::ChunkAck):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (node_blepipe_apply_legacy_chunk_ack(payload, length)) {
			node_blepipe_send_ack(hdr, 1U, payload[0]);
			return true;
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xB3U:
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length == 1U) {
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_RESET, 0U, true);
			const uint8_t ok = node_recording_app.reset_to_idle_and_erase() ? 1U : 0U;
			if (ok != 0U) {
				node_blepipe_reset_upload_state();
			}
			(void)node_blepipe_send_record_ready_status(true);
			node_blepipe_send_ack(hdr, ok, payload[0]);
			return ok != 0U;
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xB0U:
		if (length >= 2U) {
			const uint8_t requested_id = payload[1];
			const uint8_t new_id = payload[length - 1U];
			if (!exo::node_runtime_config::is_valid_node_id(new_id) ||
			    !exo::node_runtime_config::store_node_id(new_id)) {
				node_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			}
#if EXO_NODE_FLASH_ENABLED
			node_recording_app.set_node_id(new_id);
#endif
			const uint8_t response[4] = { 0xB0U, requested_id, new_id, 1U };
			node_blepipe_send_response(hdr, response, static_cast<uint16_t>(sizeof(response)));
			node_blepipe_send_ack(hdr, 1U, payload[0]);
			return true;
		}
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	case 0xB1U:
	{
		const uint8_t current_id = static_cast<uint8_t>(node_blepipe_current_id());
		const uint8_t target_id = length >= 2U ? payload[1] : current_id;
		const uint8_t response[4] = { 0xB1U, target_id, current_id, 1U };
		node_blepipe_send_response(hdr, response, static_cast<uint16_t>(sizeof(response)));
		node_blepipe_send_ack(hdr, 1U, payload[0]);
		return true;
	}
	case 0xB2U:
		node_blepipe_send_topology(hdr);
		node_blepipe_send_ack(hdr, 1U, payload[0]);
		return true;
	case 0xB4U:
		node_blepipe_send_topology(hdr);
		node_blepipe_send_ack(hdr, 1U, payload[0]);
		return true;
	case static_cast<uint8_t>(exo::RecordCommand::ReliableFrame):
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		if (length >= sizeof(exo::RecordReliableFrameHeader)) {
			exo::RecordReliableFrameHeader rel{};
			memcpy(&rel, payload, sizeof(rel));
			if (rel.proto_version != exo::kRecordReliableProtoVersion ||
			    rel.magic != exo::kRecordReliableMagic ||
			    (sizeof(rel) + rel.payload_len) > length) {
				node_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			}
			const uint8_t *body = payload + sizeof(rel);
			const exo::RecordReliableType type = static_cast<exo::RecordReliableType>(rel.frame_type);
			switch (type) {
			case exo::RecordReliableType::ManifestAck:
				if (rel.payload_len >= sizeof(exo::RecordReliableManifestAckPayload)) {
					exo::RecordReliableManifestAckPayload ack{};
					memcpy(&ack, body, sizeof(ack));
					const bool upload_ready = node_recording_app.session_ready() || node_recording_app.state() == exo::RecorderState::Uploading;
					if (ack.source_id == node_blepipe_current_id() &&
					    upload_ready &&
					    (node_recording_app.state() == exo::RecorderState::Uploading || node_recording_app.begin_upload())) {
						const bool same_active_session = g_node_upload_active &&
								g_node_upload_session_id == ack.session_id &&
								g_node_upload_total_size != 0U;
						g_node_upload_active = true;
						g_node_upload_session_id = ack.session_id;
						g_node_upload_total_size = node_recording_app.make_upload_reader().total_size();
						g_node_upload_crc32 = node_recording_app.make_record_done().payload_crc32;
						if (!same_active_session) {
							g_node_upload_next_chunk = 0U;
							g_node_upload_last_chunk_ms = 0U;
							g_node_upload_last_fail_log_ms = 0U;
						}
						g_node_upload_retx_remaining = 0U;
						g_node_upload_credit_idle_ms = 0U;
						g_node_upload_credit = ack.credit == 0U ? exo::kRecordReliableDefaultCredit : ack.credit;
						g_node_record_done_sent = true;
						EXO_LOG("[BLE][NODE][REC] ManifestAck source=%u session=%lu credit=%u next=%lu active=%u start upload\r\n",
								static_cast<unsigned>(ack.source_id),
								static_cast<unsigned long>(ack.session_id),
								static_cast<unsigned>(g_node_upload_credit),
								static_cast<unsigned long>(g_node_upload_next_chunk),
								static_cast<unsigned>(same_active_session ? 1U : 0U));
						node_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
				}
				break;
			case exo::RecordReliableType::AckWindow:
				if (rel.payload_len >= sizeof(exo::RecordReliableAckWindowPayload)) {
					exo::RecordReliableAckWindowPayload ack{};
					memcpy(&ack, body, sizeof(ack));
					if (ack.source_id == node_blepipe_current_id() &&
					    ack.session_id == g_node_upload_session_id) {
						g_node_upload_active = true;
						/* ACK_WINDOW also doubles as a periodic progress heartbeat from the
						 * app (bypass=1), reporting its contiguous-received count. Because
						 * this node bursts ahead using its granted credit without waiting
						 * for each chunk to be confirmed, next_chunk_index is expected to
						 * trail behind our own send cursor most of the time. Only use it to
						 * move forward (skip-ahead) or to refresh credit; never rewind here
						 * on a stale/behind value, or every heartbeat would needlessly
						 * re-send chunks that already went out. Genuine gap recovery is
						 * handled explicitly via NackRange below.
						 *
						 * While a NackRange is still being serviced the send cursor belongs
						 * to gap recovery. An ACK_WINDOW generated before that NACK was
						 * handled would otherwise drag the cursor forward again and the
						 * requested chunks would never be retransmitted. */
						if (g_node_upload_retx_remaining == 0U &&
						    ack.next_chunk_index > g_node_upload_next_chunk) {
							g_node_upload_next_chunk = ack.next_chunk_index;
						}
						g_node_upload_credit = ack.credit == 0U ? exo::kRecordReliableDefaultCredit : ack.credit;
						g_node_upload_credit_idle_ms = 0U;
						g_node_record_done_sent = true;
						node_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
				}
				break;
			case exo::RecordReliableType::NackRange:
				if (rel.payload_len >= 12U) {
					const uint16_t source_id = static_cast<uint16_t>(body[0] | (static_cast<uint16_t>(body[1]) << 8U));
					const uint32_t session_id = static_cast<uint32_t>(body[2]) |
							(static_cast<uint32_t>(body[3]) << 8U) |
							(static_cast<uint32_t>(body[4]) << 16U) |
							(static_cast<uint32_t>(body[5]) << 24U);
					const uint32_t first_chunk = static_cast<uint32_t>(body[6]) |
							(static_cast<uint32_t>(body[7]) << 8U) |
							(static_cast<uint32_t>(body[8]) << 16U) |
							(static_cast<uint32_t>(body[9]) << 24U);
					const uint16_t chunk_count = static_cast<uint16_t>(body[10] | (static_cast<uint16_t>(body[11]) << 8U));
					if (source_id == node_blepipe_current_id() &&
					    session_id == g_node_upload_session_id) {
						g_node_upload_active = true;
						g_node_upload_next_chunk = first_chunk;
						g_node_upload_credit = chunk_count == 0U ? 1U :
								static_cast<uint8_t>(chunk_count > exo::kRecordReliableDefaultCredit ?
										exo::kRecordReliableDefaultCredit : chunk_count);
						/* Own the send cursor until every requested chunk has gone out, so a
						 * stale ACK_WINDOW cannot cancel the retransmit. */
						g_node_upload_retx_remaining = g_node_upload_credit;
						g_node_upload_credit_idle_ms = 0U;
						g_node_upload_last_chunk_ms = 0U;
						g_node_upload_last_fail_log_ms = 0U;
						g_node_record_done_sent = true;
						EXO_LOG("[BLE][NODE][REC] NackRange source=%u session=%lu first=%lu count=%u credit=%u\r\n",
								static_cast<unsigned>(source_id),
								static_cast<unsigned long>(session_id),
								static_cast<unsigned long>(first_chunk),
								static_cast<unsigned>(chunk_count),
								static_cast<unsigned>(g_node_upload_credit));
						node_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
				}
				break;
			case exo::RecordReliableType::Pause:
				g_node_upload_active = false;
				node_blepipe_send_ack(hdr, 1U, payload[0]);
				return true;
			case exo::RecordReliableType::Resume:
				if (rel.source_id == node_blepipe_current_id() &&
				    rel.session_id == g_node_upload_session_id) {
					g_node_upload_active = true;
					if (g_node_upload_credit == 0U) {
						g_node_upload_credit = exo::kRecordReliableDefaultCredit;
					}
					g_node_upload_credit_idle_ms = 0U;
					node_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				break;
			case exo::RecordReliableType::Cancel:
				if (node_recording_app.reset_to_idle_and_erase()) {
					node_blepipe_reset_upload_state();
					node_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				break;
			case exo::RecordReliableType::VerifyOk:
				if (rel.payload_len >= sizeof(exo::RecordReliableVerifyPayload)) {
					exo::RecordReliableVerifyPayload verify{};
					memcpy(&verify, body, sizeof(verify));
					if (verify.source_id == node_blepipe_current_id() &&
					    verify.session_id == g_node_upload_session_id &&
					    verify.file_crc32 == g_node_upload_crc32 &&
					    node_recording_app.transfer_complete() &&
					    node_recording_app.acknowledge_and_erase()) {
						node_blepipe_reset_upload_state();
						node_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
				}
				break;
			case exo::RecordReliableType::VerifyFail:
				if (rel.source_id == node_blepipe_current_id() &&
				    rel.session_id == g_node_upload_session_id) {
					g_node_upload_active = true;
					g_node_upload_next_chunk = rel.chunk_index;
					g_node_upload_credit = 1U;
					g_node_upload_retx_remaining = 1U;
					g_node_upload_credit_idle_ms = 0U;
					node_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				break;
			case exo::RecordReliableType::CommitDone:
				if (rel.source_id == node_blepipe_current_id() &&
				    rel.session_id == g_node_upload_session_id) {
					if (!node_recording_app.transfer_complete()) {
						EXO_LOG("[BLE][NODE][REC] CommitDone early ignored source=%u session=%lu next=%lu credit=%u\r\n",
								static_cast<unsigned>(rel.source_id),
								static_cast<unsigned long>(rel.session_id),
								static_cast<unsigned long>(g_node_upload_next_chunk),
								static_cast<unsigned>(g_node_upload_credit));
						node_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
					node_blepipe_reset_upload_state();
					node_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				break;
			default:
				break;
			}
		}
#endif
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	default:
		node_blepipe_send_ack(hdr, 0U, payload[0]);
		return false;
	}
}

#if EXO_PROFILE_DIAG
static uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c)
{
	uint8_t count = 0;

	for (uint16_t address = 1U; address < 129; ++address)
			{
		uint8_t pData[20] = { 0 };
		if (HAL_I2C_Master_Receive(hi2c, (uint16_t) (address << 1), pData, 20, 20) == HAL_OK)
				{
			EXO_LOG("I2C%d device at address 0x%02X responded with data: ", (hi2c == &hi2c1 ? 1 : 3), address);
			for (int i = 0; i < 20; ++i)
					{
				EXO_LOG("%02X ", pData[i]);
			}
			EXO_LOG("\r\n");
			++count;
		}
		HAL_Delay(20);
	}

	return count;
}

static const char* I2cReadyStr(HAL_StatusTypeDef status)
		{
	switch (status) {
	case HAL_OK:
		return "ready";
	case HAL_BUSY:
		return "busy";
	case HAL_TIMEOUT:
		return "timeout";
	case HAL_ERROR:
	default:
		return "error";
	}
}
#endif

static uint32_t SpiNextFasterPrescaler(uint32_t current)
{
	switch (current) {
		case SPI_BAUDRATEPRESCALER_256: return SPI_BAUDRATEPRESCALER_128;
		case SPI_BAUDRATEPRESCALER_128: return SPI_BAUDRATEPRESCALER_64;
		case SPI_BAUDRATEPRESCALER_64: return SPI_BAUDRATEPRESCALER_32;
		case SPI_BAUDRATEPRESCALER_32: return SPI_BAUDRATEPRESCALER_16;
		case SPI_BAUDRATEPRESCALER_16: return SPI_BAUDRATEPRESCALER_8;
		case SPI_BAUDRATEPRESCALER_8: return SPI_BAUDRATEPRESCALER_4;
		case SPI_BAUDRATEPRESCALER_4: return SPI_BAUDRATEPRESCALER_2;
		default: return current;
	}
}

#if EXO_PROFILE_DIAG
static HAL_StatusTypeDef I2C1_RecoverAndProbe(uint16_t addr7bit)
{
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, (addr7bit << 1U), 2U, 20U);
	if (status != HAL_BUSY) {
		return status;
	}

	(void)HAL_I2C_DeInit(&hi2c1);
	HAL_Delay(2);
	MX_I2C1_Init();
	HAL_Delay(2);
	return HAL_I2C_IsDeviceReady(&hi2c1, (addr7bit << 1U), 2U, 20U);
}
#endif

#if EXO_NODE_FLASH_ENABLED
static bool NodeFlashRead(uint32_t address, void *data, uint32_t size)
{
	return node_recording_app.flash_read_raw(address, data, size);
}

static bool NodeFlashWrite(uint32_t address, const void *data, uint32_t size)
{
	return node_recording_app.flash_write_raw(address, data, size);
}

static bool NodeFlashErase(uint32_t address, uint32_t size)
{
	return node_recording_app.flash_erase_raw(address, size);
}
#endif

#if EXO_NODE_FLASH_ENABLED && (EXO_NODE_FLASH_TEST_BOOT_ENABLE || EXO_NODE_FLASH_TEST_API_ENABLE)
static void PrintHexBlock(const char *label, const uint8_t *data, uint32_t len)
{
	EXO_LOG("%s", label);
	for (uint32_t i = 0U; i < len; ++i)
	{
		if ((i % 16U) == 0U)
		{
			EXO_LOG("\r\n%03lu: ", i);
		}
		EXO_LOG("%02X ", data[i]);
	}
	EXO_LOG("\r\n");
}

static uint8_t RunFlashTest128()
{
	const uint32_t test_address = 0x0007F000U;
	const uint32_t seed = HAL_GetTick() ^ 0x1A2B3C4DU;
	uint16_t mismatch_count = 0U;
	uint16_t first_mismatch_index = 0xFFFFU;
	uint8_t written[128] = {0U};
	uint8_t readback[128] = {0U};
	uint8_t manufacturer = 0U;
	uint8_t device_id_hi = 0U;
	uint8_t device_id_lo = 0U;

	if (!node_recording_app.flash_get_jedec(manufacturer, device_id_hi, device_id_lo))
	{
		EXO_LOG("W25Q JEDEC read failed\r\n");
		return 2U;
	}
	EXO_LOG("W25Q JEDEC: %02X %02X %02X\r\n", manufacturer, device_id_hi, device_id_lo);

	const bool ok = node_recording_app.flash_self_test_128(test_address, seed, mismatch_count, first_mismatch_index, written, readback);
	PrintHexBlock("W25Q written(128B):", written, sizeof(written));
	PrintHexBlock("W25Q read(128B):", readback, sizeof(readback));
	if (ok)
	{
		EXO_LOG("W25Q 128B test PASS addr=0x%08lX\r\n", test_address);
		return 0U;
	}
	const exo::W25Q256Flash::DebugInfo info = node_recording_app.flash_debug_info();

	EXO_LOG("W25Q 128B test FAIL addr=0x%08lX mismatch_count=%u first_idx=%u\r\n",
			test_address, mismatch_count, first_mismatch_index);
	EXO_LOG("W25Q dbg: err=%s drv=%u jedec=%02X %02X %02X spi_cmd=0x%02X hal=%u hdr=%lu\r\n",
			node_recording_app.flash_last_error_string(),
			info.last_driver_result,
			info.jedec_manufacturer,
			info.jedec_device_hi,
			info.jedec_device_lo,
			info.last_spi_instruction,
			info.last_hal_status,
			info.last_header_len);
	return 1U;
}
#endif

#if EXO_NODE_FLASH_ENABLED && EXO_NODE_FLASH_TEST_API_ENABLE
extern "C" uint8_t exo_node_flash_test_128b(void)
{
	return RunFlashTest128();
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  NodeSwo_Init(HAL_RCC_GetHCLKFreq(), 2000000U);
  NodeSwo_Logf("[SWO][NODE] boot hclk=%lu swo=%lu\r\n",
		  static_cast<unsigned long>(HAL_RCC_GetHCLKFreq()),
		  2000000UL);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
	{
		const uint32_t old_prescaler = hspi1.Init.BaudRatePrescaler;
		const uint32_t new_prescaler = SpiNextFasterPrescaler(old_prescaler);
		if (new_prescaler != old_prescaler) {
			hspi1.Init.BaudRatePrescaler = new_prescaler;
			if (HAL_SPI_Init(&hspi1) != HAL_OK) {
				hspi1.Init.BaudRatePrescaler = old_prescaler;
				(void)HAL_SPI_Init(&hspi1);
			}
		}
		EXO_LOG("SPI1 prescaler (node flash) old=%lu new=%lu\r\n",
				static_cast<unsigned long>(old_prescaler),
				static_cast<unsigned long>(hspi1.Init.BaudRatePrescaler));
	}

	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
	node_prepare_touch_wakeup_before_poweroff();
	HAL_Delay(200);
	{
		GPIO_InitTypeDef lpuart_rx_cfg = {0};
		lpuart_rx_cfg.Pin = GPIO_PIN_3;
		lpuart_rx_cfg.Mode = GPIO_MODE_AF_PP;
		lpuart_rx_cfg.Pull = GPIO_PULLUP;
		lpuart_rx_cfg.Speed = GPIO_SPEED_FREQ_LOW;
		lpuart_rx_cfg.Alternate = GPIO_AF8_LPUART1;
		HAL_GPIO_Init(GPIOA, &lpuart_rx_cfg);
	}
	RGB.OFF();
	ERM_PWM.SET_PERCENT(0U);
	BUZZER.SET_PERCENT(0U);

	EXO_LOG("[BUILD][NODE] ble-rx-log-bridge active\r\n");
	EXO_LOG("UART callbacks idle; BLE-only transport active\r\n");
#if EXO_PROFILE_DIAG
	{
		uint8_t devices_on_i2c1 = I2C_ScanBus(&hi2c1);
		uint8_t devices_on_i2c3 = I2C_ScanBus(&hi2c3);
		EXO_LOG("I2C1: %d device(s) found\r\n", devices_on_i2c1);
		EXO_LOG("I2C3: %d device(s) found\r\n", devices_on_i2c3);
	}
	const HAL_StatusTypeDef bno_probe = HAL_I2C_IsDeviceReady(&hi2c3, (0x4BU << 1U), 2U, 20U);
	const HAL_StatusTypeDef icm_probe = I2C1_RecoverAndProbe(0x69U);
	EXO_LOG("I2C probe: BNO85(0x4B,I2C3)=%s, ICM45686(0x69,I2C1)=%s\r\n",
			I2cReadyStr(bno_probe), I2cReadyStr(icm_probe));
#endif

#if EXO_NODE_FLASH_ENABLED
	const bool node_recording_ready = node_recording_app.begin();
	EXO_LOG("Node recording: %s\r\n", node_recording_ready ? "ready" : "not ready");
	if (node_recording_ready)
	{
		exo::node_runtime_config::set_storage_hooks(&NodeFlashRead, &NodeFlashWrite, &NodeFlashErase);
		(void)exo::node_runtime_config::set_flash_capacity(
				node_recording_app.detected_flash_capacity());
		/* Commission once, here, now that the real flash capacity is known and the
		 * settings sector base is correct. Everything after this point only reads. */
		const bool node_id_provisioned = exo::node_runtime_config::provision_node_id(EXO_NODE_ID);
		const uint8_t runtime_node_id = exo::node_runtime_config::current_node_id(EXO_NODE_ID);
		node_recording_app.set_node_id(runtime_node_id);
		EXO_LOG("Node ID runtime=%u (default=%u provisioned=%u)\r\n",
				static_cast<unsigned>(runtime_node_id),
				static_cast<unsigned>(EXO_NODE_ID),
				static_cast<unsigned>(node_id_provisioned ? 1U : 0U));
		EXO_LOG("Node flash capacity=%lu max_duration_ms=%lu\r\n",
				static_cast<unsigned long>(node_recording_app.detected_flash_capacity()),
				static_cast<unsigned long>(node_recording_app.maximum_duration_ms()));
	}
	else
	{
		EXO_LOG("Node ID runtime fallback=%u (flash unavailable)\r\n",
				static_cast<unsigned>(EXO_NODE_ID));
	}
	if (!node_recording_ready)
	{
		const exo::W25Q256Flash::DebugInfo info = node_recording_app.flash_debug_info();
		EXO_LOG("W25Q init dbg: err=%s drv=%u jedec=%02X %02X %02X spi_cmd=0x%02X hal=%u hdr=%lu\r\n",
				node_recording_app.flash_last_error_string(),
				info.last_driver_result,
				info.jedec_manufacturer,
				info.jedec_device_hi,
				info.jedec_device_lo,
				info.last_spi_instruction,
				info.last_hal_status,
				info.last_header_len);
		EXO_LOG("W25Q init xfer: stage=%u in=%lu out=%lu\r\n",
				info.last_spi_stage, info.last_in_len, info.last_out_len);
	}
#if EXO_NODE_FLASH_TEST_BOOT_ENABLE
	if (node_recording_ready)
	{
		(void)RunFlashTest128();
	}
#endif
#else
	EXO_LOG("Node recording disabled until flash SPI/CS CubeMX setup is complete\r\n");
#endif

#if EXO_NODE_SENSOR_TEST_ENABLE
	const bool hub_sensor_test_ready = hub_sensor_test_app.begin();
	EXO_LOG("Hub sensor test: %s\r\n", hub_sensor_test_ready ? "ready" : "not ready");
#endif
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */
    MX_APPE_Process();

  /* USER CODE BEGIN 3 */
		NodeSwo_Process();
		enum class TouchShutdownState : uint8_t {
			Idle,
			Held,
			WaitingForRelease,
			ShutdownDelay,
			PoweredOff
		};
		static TouchShutdownState touch_shutdown_state = TouchShutdownState::Idle;
		static uint32_t touch_start_ms = 0U;
		static uint32_t touch_release_ms = 0U;
		static uint32_t touch_feedback_until_ms = 0U;
		static bool ignore_touch_until_release = (HAL_GPIO_ReadPin(TOUCH_MCU_GPIO_Port, TOUCH_MCU_Pin) == GPIO_PIN_SET);

#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
		node_recording_app.process();
		node_blepipe_process_live_samples();
		node_blepipe_process_recording_upload();
		(void)node_blepipe_send_record_ready_status(false);
#endif
#if EXO_NODE_SENSOR_TEST_ENABLE
		(void)hub_sensor_test_app.process();
#endif
		const GPIO_PinState touch_pin_state = HAL_GPIO_ReadPin(TOUCH_MCU_GPIO_Port, TOUCH_MCU_Pin);
		const bool touch_active = (touch_pin_state == GPIO_PIN_SET);
		const uint32_t now_ms = HAL_GetTick();
		static uint32_t swo_alive_last_ms = 0U;
		if ((now_ms - swo_alive_last_ms) >= 1000U) {
			swo_alive_last_ms = now_ms;
			NodeSwo_Logf("[SWO][NODE] alive tick=%lu\r\n",
					static_cast<unsigned long>(now_ms));
		}

		if (g_node_touch_test_active && static_cast<int32_t>(now_ms - g_node_touch_test_until_ms) >= 0) {
			g_node_touch_test_active = false;
			g_node_actuator_override_enabled = false;
			ERM_PWM.SET_PERCENT(0);
			BUZZER.SET_PERCENT(0);
			RGB.OFF();
			node_blepipe_send_touch_status(kTouchStatusReleased);
		}
		if (g_node_poweroff_test_pending && static_cast<int32_t>(now_ms - g_node_poweroff_test_at_ms) >= 0) {
			g_node_poweroff_test_pending = false;
			ERM_PWM.SET_PERCENT(0);
			BUZZER.SET_PERCENT(0);
			RGB.OFF();
			node_blepipe_send_touch_status(kTouchStatusTurningOff);
			node_poweroff_pcb_and_wait_for_release();
		}
		if (g_node_touch_test_active || g_node_poweroff_test_pending) {
			continue;
		}

		if (ignore_touch_until_release) {
			if (!touch_active) {
				ignore_touch_until_release = false;
			}
			continue;
		}

		switch (touch_shutdown_state) {
			case TouchShutdownState::Idle:
				if (touch_active) {
					BUZZER.SET_PERCENT(50);
					RGB.ON();
					touch_start_ms = now_ms;
					touch_feedback_until_ms = now_ms + 500U;
					touch_shutdown_state = TouchShutdownState::Held;
					node_blepipe_send_touch_status(kTouchStatusPressed);
					EXO_LOG("Touched\r\n");
				} else if (g_node_actuator_override_enabled == false) {
					const bool blink_on = ((now_ms % 3000U) < 500U);
					const bool connected = (APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER);

					if (blink_on) {
						RGB.SET(false, connected, !connected);
					} else {
						RGB.OFF();
					}
				}
				break;

			case TouchShutdownState::Held:
				if (!touch_active) {
					BUZZER.SET_PERCENT(0);
					ERM_PWM.SET_PERCENT(0);
					RGB.OFF();
					touch_shutdown_state = TouchShutdownState::Idle;
					node_blepipe_send_touch_status(kTouchStatusReleased);
					EXO_LOG("Released\r\n");
				} else if ((now_ms - touch_start_ms) >= 5000U) {
					BUZZER.SET_PERCENT(0);
					ERM_PWM.SET_PERCENT(0);
					RGB.SET(true, false, false);
					node_blepipe_send_touch_status(kTouchStatusShutdownArmed);
					EXO_LOG("Touch shutdown armed\r\n");
					EXO_LOG("Power off\r\n");
					node_poweroff_pcb_and_wait_for_release();
				} else if (static_cast<int32_t>(now_ms - touch_feedback_until_ms) >= 0) {
					BUZZER.SET_PERCENT(0);
					ERM_PWM.SET_PERCENT(0);
					RGB.OFF();
				}
				break;

			case TouchShutdownState::WaitingForRelease:
				if (!touch_active) {
					touch_release_ms = now_ms;
					touch_shutdown_state = TouchShutdownState::ShutdownDelay;
					node_blepipe_send_touch_status(kTouchStatusTurningOff);
					EXO_LOG("Touch shutdown release\r\n");
				}
				break;

			case TouchShutdownState::ShutdownDelay:
				RGB.SET(true, false, false);
				if ((now_ms - touch_release_ms) >= 1000U) {
					BUZZER.SET_PERCENT(0);
					ERM_PWM.SET_PERCENT(0);
					RGB.OFF();
					EXO_LOG("Power off\r\n");
					node_blepipe_send_touch_status(kTouchStatusTurningOff);
					node_poweroff_pcb_and_wait_for_release();
					touch_shutdown_state = TouchShutdownState::PoweredOff;
				}
				break;

			case TouchShutdownState::PoweredOff:
				break;
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV2);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
extern "C" uint8_t exo_node_ble_runtime_id(void)
{
#if EXO_NODE_FLASH_ENABLED
	return exo::node_runtime_config::current_node_id(EXO_NODE_ID);
#else
	return EXO_NODE_ID;
#endif
}

extern "C" void exo_node_ble_log(const char *format, ...)
{
	if (format == nullptr) {
		return;
	}

	char message[224] = {0};
	va_list args;
	va_start(args, format);
	const int written = vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (written > 0) {
		EXO_LOG("%s", message);
	}
}

extern "C" uint8_t exo_node_ble_write(const uint8_t *payload, uint8_t length)
{
	if (payload == nullptr || length == 0U) {
		EXO_LOG("[BLE][WRITE] empty payload len=%u\r\n", (unsigned) length);
		return 0U;
	}
	blepipe_hdr_t pipe_hdr{};
	const uint8_t *pipe_payload = nullptr;
	uint16_t pipe_payload_len = 0U;
	const blepipe_status_t pipe_status = blepipe_decode(payload,
			length,
			&pipe_hdr,
			&pipe_payload,
			&pipe_payload_len);
	if (pipe_status == BLEPIPE_STATUS_OK &&
	    blepipe_msg_allowed_on_lane(BLEPIPE_LANE_CONTROL_RX, pipe_hdr.msg_type) != 0U) {
		EXO_LOG("[BLEPIPE][NODE][WRITE] msg=0x%02X src=0x%04X dst=0x%04X len=%u\r\n",
				(unsigned)pipe_hdr.msg_type,
				(unsigned)pipe_hdr.src_id,
				(unsigned)pipe_hdr.dst_id,
				(unsigned)pipe_payload_len);
		switch (pipe_hdr.msg_type) {
		case BLEPIPE_MSG_COMMAND:
		case BLEPIPE_MSG_STREAM_CONTROL:
		case BLEPIPE_MSG_CONFIG_SET:
			return node_handle_blepipe_command(pipe_hdr, pipe_payload, pipe_payload_len) ? 1U : 0U;
		default:
			node_blepipe_send_ack(pipe_hdr, 0U, pipe_payload_len > 0U ? pipe_payload[0] : 0U);
			return 0U;
		}
	}
#if EXO_BLE_LOG_LEVEL >= 2
	EXO_LOG("[BLE][DBG][WRITE] cmd=0x%02X len=%u\r\n", (unsigned) payload[0], (unsigned) length);
#endif
	if (payload[0] == BLEPIPE_PROTO_VER) {
		EXO_LOG("[BLEPIPE][NODE][WRITE] invalid frame status=%u len=%u\r\n",
				static_cast<unsigned>(pipe_status),
				static_cast<unsigned>(length));
		return 0U;
	}
#if EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED
	switch (payload[0]) {
	case static_cast<uint8_t>(exo::RecordCommand::StartRecord):
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			node_blepipe_reset_upload_state();
			const uint8_t ok = node_recording_app.start_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][START] session=%lu result=%u\r\n",
					(unsigned long) message.session_id, (unsigned) ok);
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_PREPARE, message.session_id, true);
			(void)node_blepipe_send_record_ready_status(true);
			return ok;
		}
		EXO_LOG("[BLE][START] bad len=%u expect=%u\r\n",
				(unsigned) length, (unsigned) sizeof(exo::StartRecordMessage));
		break;
	case static_cast<uint8_t>(exo::RecordCommand::PrepareRecord):
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			node_blepipe_reset_upload_state();
			const uint8_t ok = node_recording_app.prepare_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][PREP] session=%lu result=%u\r\n",
					(unsigned long) message.session_id, (unsigned) ok);
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_PREPARE, message.session_id, true);
			(void)node_blepipe_send_record_ready_status(true);
			return ok;
		}
		EXO_LOG("[BLE][PREP] bad len=%u expect=%u\r\n",
				(unsigned) length, (unsigned) sizeof(exo::StartRecordMessage));
		break;
	case static_cast<uint8_t>(exo::RecordCommand::CommitPreparedRecord):
		if (length == sizeof(exo::StartRecordMessage)) {
			exo::StartRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			const uint8_t ok = node_recording_app.commit_prepared_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][COMMIT] session=%lu result=%u\r\n",
					(unsigned long) message.session_id, (unsigned) ok);
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_COMMIT, message.session_id, true);
			(void)node_blepipe_send_record_ready_status(true);
			return ok;
		}
		EXO_LOG("[BLE][COMMIT] bad len=%u expect=%u\r\n",
				(unsigned) length, (unsigned) sizeof(exo::StartRecordMessage));
		break;
	case static_cast<uint8_t>(exo::RecordCommand::AbortPreparedRecord):
		node_recording_app.abort_prepared_recording();
		node_blepipe_reset_upload_state();
		EXO_LOG("[BLE][ABORT_PREP]\r\n");
		(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_ABORT, 0U, true);
		(void)node_blepipe_send_record_ready_status(true);
		return 1U;
	case static_cast<uint8_t>(exo::RecordCommand::StopRecord):
		if (length == sizeof(exo::StopRecordMessage)) {
			exo::StopRecordMessage message{};
			memcpy(&message, payload, sizeof(message));
			const uint8_t ok = node_recording_app.stop_recording(message) ? 1U : 0U;
			EXO_LOG("[BLE][STOP] session=%lu result=%u\r\n",
					(unsigned long)message.session_id, (unsigned)ok);
			return ok;
		}
		break;
	case static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck):
		if (length == sizeof(exo::SessionCompleteAckMessage)) {
			const uint8_t ok = (node_recording_app.transfer_complete() && node_recording_app.acknowledge_and_erase()) ? 1U : 0U;
			if (ok != 0U) {
				node_blepipe_reset_upload_state();
			}
			EXO_LOG("[BLE][COMPLETE_ACK] result=%u\r\n", (unsigned) ok);
			(void)node_blepipe_send_record_ready_status(true);
			return ok;
		}
		EXO_LOG("[BLE][COMPLETE_ACK] bad len=%u expect=%u\r\n",
				(unsigned) length, (unsigned) sizeof(exo::SessionCompleteAckMessage));
		break;
	case static_cast<uint8_t>(exo::RecordCommand::ChunkAck):
	{
		const uint8_t ok = node_blepipe_apply_legacy_chunk_ack(payload, length) ? 1U : 0U;
		EXO_LOG("[BLE][CHUNK_ACK] result=%u len=%u\r\n", (unsigned) ok, (unsigned) length);
		return ok;
	}
	case 0xB3U:
	{
		if (length == 1U) {
			const uint8_t ok = node_recording_app.reset_to_idle_and_erase() ? 1U : 0U;
			if (ok != 0U) {
				node_blepipe_reset_upload_state();
			}
			EXO_LOG("[BLE][RESET] result=%u\r\n", (unsigned) ok);
			(void)node_blepipe_send_record_start_heartbeat(BLEPIPE_RECORD_START_PHASE_NODE_RESET, 0U, true);
			(void)node_blepipe_send_record_ready_status(true);
			return ok;
		}
		EXO_LOG("[BLE][RESET] bad len=%u expect=1\r\n", (unsigned) length);
		break;
	}
	case 0xB0U: /* set node id: payload[1]=new_id */
	{
		if (length < 2U) {
			EXO_LOG("[BLE][CFG] set-node-id bad len=%u expect>=2\r\n", (unsigned) length);
			return 0U;
		}
		const uint8_t new_id = payload[1];
		if (!exo::node_runtime_config::is_valid_node_id(new_id)) {
			EXO_LOG("[BLE][CFG] set-node-id invalid=%u\r\n", (unsigned) new_id);
			return 0U;
		}
		if (!exo::node_runtime_config::store_node_id(new_id)) {
			EXO_LOG("[BLE][CFG] set-node-id persist failed id=%u\r\n", (unsigned) new_id);
			return 0U;
		}
		node_recording_app.set_node_id(new_id);
		EXO_LOG("[BLE][CFG] set-node-id ok=%u (reboot not required)\r\n", (unsigned) new_id);
		return 1U;
	}
	case 0xB1U: /* get node id */
	{
		const uint8_t current_id = exo::node_runtime_config::current_node_id(EXO_NODE_ID);
		EXO_LOG("[BLE][CFG] get-node-id=%u\r\n", (unsigned) current_id);
		return 1U;
	}
	default:
		EXO_LOG("[BLE][WRITE] unsupported cmd=0x%02X len=%u\r\n", (unsigned) payload[0], (unsigned) length);
		break;
	}
#else
	EXO_LOG("[BLE][WRITE] forwarding disabled (EXO_NODE_BLE_FORWARD_ENABLE && EXO_NODE_FLASH_ENABLED == 0)\r\n");
	(void)payload;
	(void)length;
#endif
	return 0U;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	(void)huart;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	(void)huart;
	(void)Size;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	(void)huart;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
