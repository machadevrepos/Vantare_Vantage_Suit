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
#include "app_fatfs.h"
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
#include <INCLUDER.h>
#include <ACQUISITION_DIAGNOSTICS.h>
#include <BLE_RECORD_PROTOCOL.h>
#include <BLE_SESSION_CONTROL.h>
#include <BLE_STREAM_V2.h>
#include <EXO_LOGGER.h>
#include <HUB_SENSOR_TEST_APP.h>
#include <LIVE_STREAM_TIMING.h>
#include <MASTER_IMU_CSV_LOGGER.h>
#include <MASTER_IMU_SWO_TELEMETRY.h>
#include <MASTER_SD_SESSION_RECORDER.h>
#include <MASTER_TRAINING_CSV_COORDINATOR.h>
#include <RECORDING_BRIDGE.h>
#include <RECORDING_TYPES.h>
#include "HUB_LEAF_BLE_MANAGER.h"
#include "blepipe_proto.h"
#include "custom_app.h"
#include "app_ble.h"
#include "exo_hub_central_client.h"
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
#ifndef EXO_MASTER_VERBOSE_DIAG
#define EXO_MASTER_VERBOSE_DIAG 0
#endif

#ifndef EXO_MASTER_IMU_SWO_ENABLE
#define EXO_MASTER_IMU_SWO_ENABLE 1
#endif

#ifndef EXO_MASTER_IMU_SWO_INTERVAL_MS
#define EXO_MASTER_IMU_SWO_INTERVAL_MS exo::imu_swo::kDefaultIntervalMs
#endif

#ifndef EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
#define EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE 0
#endif

#ifndef EXO_BLE_LOG_LEVEL
#define EXO_BLE_LOG_LEVEL 2 /* 0=off,1=error,2=info,3=debug */
#endif

#ifndef EXO_BLE_HEX_DUMP_ENABLE
#define EXO_BLE_HEX_DUMP_ENABLE 0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static exo::HubSensorTestApp hub_sensor_test_app(hi2c1, hi2c3, 0x4BU, 0x69U);
#if EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
static exo::MasterImuCsvLogger master_imu_csv_logger;
#endif
static exo::MasterTrainingCsvCoordinator master_training_csv_coordinator;
static exo::TrainingCsvState master_training_csv_reported_state = exo::TrainingCsvState::Idle;
static uint8_t master_training_csv_reported_completed_mask = 0U;
static bool master_training_csv_reported_partial = false;
#if EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
static bool master_imu_csv_error_reported = false;
#endif
static bool HubRs485BleSend(const uint8_t *payload, uint8_t length);
static void HubRs485TxDone();
static exo::ble_hub::HubLeafBleManager master_rs485_recording;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static void MasterRs485_StartUartDmaRx();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PWM_PIN ERM_PWM(&htim1, TIM_CHANNEL_4, ERM_GPIO_Port, ERM_Pin);
PWM_PIN BUZZER(&htim1, TIM_CHANNEL_3, BUZZER_GPIO_Port, BUZZER_Pin);

RGB_LED RGB(RGB_R_GPIO_Port, RGB_R_Pin, RGB_G_GPIO_Port, RGB_G_Pin, RGB_B_GPIO_Port, RGB_B_Pin, 1, 1);

#if EXO_MASTER_VERBOSE_DIAG
static uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c)
		{
	uint8_t count = 0;

	for (uint16_t address = 1U; address < 129; ++address)
			{
//		if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t) (address << 1), 1U, 10U) == HAL_OK)
//				{
//			EXO_LOG("I2C%d device found at address 0x%02X\r\n", (hi2c == &hi2c1 ? 1 : 3), address);
//			++count;
//		}
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
#endif

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

static __attribute__((unused)) const char* HalStatusStr(HAL_StatusTypeDef status)
		{
	switch (status) {
		case HAL_OK:
			return "HAL_OK";
		case HAL_BUSY:
			return "HAL_BUSY";
		case HAL_TIMEOUT:
			return "HAL_TIMEOUT";
		case HAL_ERROR:
			default:
			return "HAL_ERROR";
	}
}

static uint32_t SpiNextFasterPrescaler(uint32_t current)
		{
	switch (current) {
		case SPI_BAUDRATEPRESCALER_256:
			return SPI_BAUDRATEPRESCALER_128;
		case SPI_BAUDRATEPRESCALER_128:
			return SPI_BAUDRATEPRESCALER_64;
		case SPI_BAUDRATEPRESCALER_64:
			return SPI_BAUDRATEPRESCALER_32;
		case SPI_BAUDRATEPRESCALER_32:
			return SPI_BAUDRATEPRESCALER_16;
		case SPI_BAUDRATEPRESCALER_16:
			return SPI_BAUDRATEPRESCALER_8;
		case SPI_BAUDRATEPRESCALER_8:
			return SPI_BAUDRATEPRESCALER_4;
		case SPI_BAUDRATEPRESCALER_4:
			return SPI_BAUDRATEPRESCALER_2;
		default:
			return current;
	}
}

static bool HubRs485BleSend(const uint8_t *payload, uint8_t length)
		{
	(void) payload;
	(void) length;
	return true;
}

static void HubRs485TxDone()
{
}

static void MasterRs485_StartUartDmaRx()
{
	/* RS485 removed in BLE-only architecture. */
}

namespace {
	constexpr uint32_t kDefaultStreamIntervalMs = 20U;
	constexpr uint32_t kMinStreamIntervalMs = 10U;
	constexpr uint32_t kMaxStreamIntervalMs = 100U;
	constexpr uint16_t kRecordChunkPayloadBytes = 180U;
	constexpr uint32_t kLocalRecordChunkGapMs = 1U;
	constexpr uint8_t kLocalRecordBurstLimit = 4U;
	constexpr uint8_t kRecordTransferTuningCmd = 0xB5U;
	constexpr uint8_t kRecordTransferTuningVersion = 1U;
	constexpr uint8_t kTrainingCsvStatusReportId = 0xB6U;
	constexpr uint16_t kMasterNodeId = 0U;
	constexpr uint8_t kPendingNodeDoneQueueSize = 4U;
	constexpr uint32_t kDefaultRecordDurationMs = 10000U;
	constexpr uint64_t kDefaultLeadTimeUs = 300000ULL;
	constexpr uint32_t kMasterBleSessionTag = 0x80000000UL;
	constexpr uint8_t kRecordProtoV3 = 3U;

	static bool master_blepipe_send(Custom_STM_Char_Opcode_t char_opcode,
			uint8_t msg_type,
			uint16_t dst_id,
			const uint8_t *payload,
			uint16_t payload_len);

	static uint32_t master_ble_session_id(uint32_t base_session_id)
			{
		return (base_session_id | kMasterBleSessionTag);
	}

	static bool send_record_lane_v3(uint32_t session_id,
			uint16_t source_id,
			exo::RecordLaneId lane_id,
			exo::RecordLaneMsgType msg_type,
			uint16_t sequence,
			uint32_t offset,
			const uint8_t *payload,
			uint16_t payload_len,
			uint16_t flags = 0U)
			{
		uint8_t packet[244];
		exo::RecordLaneFrameV3Header hdr { };
		hdr.command = exo::RecordCommand::LaneFrameV3;
		hdr.proto_version = kRecordProtoV3;
		hdr.session_id = session_id;
		hdr.source_id = source_id;
		hdr.lane_id = static_cast<uint8_t>(lane_id);
		hdr.msg_type = static_cast<uint8_t>(msg_type);
		hdr.sequence = sequence;
		hdr.offset = offset;
		hdr.payload_len = payload_len;
		hdr.flags = flags;
		const uint16_t total = static_cast<uint16_t>(sizeof(hdr) + payload_len);
		if (total > sizeof(packet)) {
			return false;
		}
		memcpy(packet, &hdr, sizeof(hdr));
		if (payload != nullptr && payload_len > 0U) {
			memcpy(packet + sizeof(hdr), payload, payload_len);
		}
		return Custom_APP_SendRecordFrame(packet, static_cast<uint8_t>(total)) == BLE_STATUS_SUCCESS;
	}

	static bool send_reliable_record_frame(exo::RecordReliableType type,
			uint16_t source_id,
			uint32_t session_id,
			uint32_t chunk_index,
			uint32_t byte_offset,
			const uint8_t *payload,
			uint16_t payload_len,
			uint16_t flags = 0U)
			{
		uint8_t packet[244];
		exo::RecordReliableFrameHeader hdr { };
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
		return Custom_APP_SendRecordFrame(packet, static_cast<uint8_t>(total)) == BLE_STATUS_SUCCESS;
	}

	static uint16_t reliable_total_chunks(uint32_t total_size)
			{
		return static_cast<uint16_t>((total_size + kRecordChunkPayloadBytes - 1U) / kRecordChunkPayloadBytes);
	}

	static bool send_reliable_manifest(uint16_t source_id,
			uint32_t session_id,
			uint32_t total_size,
			uint32_t payload_crc32,
			uint32_t duration_ms)
			{
		exo::RecordReliableManifestPayload manifest { };
		manifest.protocol_version = exo::kRecordReliableProtoVersion;
		manifest.source_id = source_id;
		manifest.session_id = session_id;
		manifest.file_size = total_size;
		manifest.chunk_size = kRecordChunkPayloadBytes;
		manifest.total_chunks = reliable_total_chunks(total_size);
		manifest.file_crc32 = payload_crc32;
		manifest.duration_ms = duration_ms;
		return send_reliable_record_frame(exo::RecordReliableType::Manifest,
				source_id,
				session_id,
				0U,
				0U,
				reinterpret_cast<const uint8_t*>(&manifest),
				static_cast<uint16_t>(sizeof(manifest)));
	}

	static void master_blepipe_send_topology(const blepipe_hdr_t *request_hdr);

	static volatile bool g_ble_stream_enabled = false;
	static volatile uint32_t g_ble_stream_interval_ms = kDefaultStreamIntervalMs;
	static volatile uint32_t g_ble_tx_drop_count = 0U;
	static uint16_t g_ble_sequence = 0U;
	static volatile bool g_ble_actuator_override_enabled = false;
	static volatile uint8_t g_ble_erm_percent = 0U;
	static volatile uint8_t g_ble_buzzer_percent = 0U;
	static volatile uint8_t g_ble_rgb_mask = 0U;
	static volatile bool g_master_ble_status_notify_enabled = false;
	static exo::StartRecordMessage g_last_ble_start_msg { };
	static uint32_t g_last_ble_start_tick_ms = 0U;
	static bool g_have_last_ble_start_msg = false;
	static exo::StartRecordMessage g_last_accepted_ble_start_msg { };
	static uint32_t g_last_accepted_ble_start_tick_ms = 0U;
	static bool g_have_last_accepted_ble_start_msg = false;
	static uint32_t g_last_chunk_ack_session_id = 0U;
	static uint32_t g_last_chunk_ack_next_offset = 0U;
	static uint16_t g_last_chunk_ack_source_id = 0U;
	static uint32_t g_last_chunk_ack_tick_ms = 0U;
	static bool g_have_last_chunk_ack = false;
	static volatile bool g_ble_record_transfer_mode = false;
	static volatile bool g_ble_start_or_record_in_progress = false;
	static uint32_t g_last_master_reset_tick_ms = 0U;
	static bool g_have_last_master_reset_tick = false;

	enum class LocalRecordPhase : uint8_t {
		Idle,
		Capturing,
		RecordDoneWait,
		Manifest,
		TransferActive,
		TransferPaused,
		Resync,
		Verifying,
		Finished,
		Cancelled,
		ErrorStoredCanResume
	};

	static LocalRecordPhase g_local_record_phase = LocalRecordPhase::Idle;
	static exo::StartRecordMessage g_local_start_msg { };
	static bool g_local_record_armed = false;
	static uint32_t g_local_arm_tick_ms = 0U;
	static uint32_t g_local_capture_start_ms = 0U;
	static uint16_t g_local_chunk_seq = 1U;
	static uint32_t g_local_chunk_offset = 0U;
	static uint32_t g_local_last_chunk_tick = 0U;
	static exo::RecordDoneMessage g_local_done { };
	static uint32_t g_local_session_size = 0U;
	static bool g_local_done_notified = false;
	static bool g_local_manifest_acked = false;
	static uint8_t g_local_receiver_credit = 0U;
	static uint32_t g_local_requested_chunk = 0U;
	static uint32_t g_local_stream_cursor_chunk = 0U;
	static uint32_t g_local_retx_cursor_chunk = 0U;
	static uint8_t g_local_retx_remaining = 0U;
	static uint32_t g_local_forward_progress_tick_ms = 0U;
	static uint8_t g_local_stale_ack_repeat = 0U;
	static uint32_t g_local_pending_chunk = 0U;
	static bool g_local_waiting_verify = false;
	static uint32_t g_local_manifest_last_tick = 0U;
	static exo::RecordDoneMessage g_pending_node_done { };
	static bool g_have_pending_node_done = false;
	static exo::RecordDoneMessage g_pending_node_done_queue[kPendingNodeDoneQueueSize] { };
	static uint8_t g_pending_node_done_head = 0U;
	static uint8_t g_pending_node_done_count = 0U;
	static exo::RecordDoneMessage g_seen_node_done_queue[kPendingNodeDoneQueueSize] { };
	static uint8_t g_seen_node_done_head = 0U;
	static uint8_t g_seen_node_done_count = 0U;
	static exo::RecordDoneMessage g_training_node_done[4] { };
	static bool g_training_node_done_valid[4] { };
	struct TrainingPendingVerifyOk {
		bool valid;
		exo::RecordReliableVerifyPayload verify;
		uint8_t frame[UINT8_MAX];
		uint8_t length;
	};
	static TrainingPendingVerifyOk g_training_pending_verify_ok[4] { };
	static uint32_t g_pending_node_manifest_last_tick = 0U;
	static uint32_t g_pending_node_defer_last_log_ms = 0U;
	static bool g_remote_transfer_active = false;
	static uint16_t g_remote_transfer_source_id = 0U;
	static uint32_t g_remote_transfer_session_id = 0U;
	static constexpr uint32_t kRemoteResetCooldownMs = 2500U;
	static constexpr uint32_t kRecordPrepareTimeoutMs = 120000U;
	static constexpr uint32_t kRecordAppReadyTimeoutMs = 120000U;
	static constexpr uint8_t kRecordSyncNoCommand = 0x00U;
	static uint32_t g_remote_reset_busy_until_ms = 0U;
	enum class RecordSyncPhoneAckMode : uint8_t {
		None,
		Raw,
		Blepipe
	};
	struct RecordSyncState {
			bool active = false;
			exo::StartRecordMessage message { };
			uint8_t target_mask = 0U;
			uint8_t prepared_mask = 0U;
			uint8_t failed_mask = 0U;
			uint32_t deadline_ms = 0U;
			bool prepare_sent = false;
			RecordSyncPhoneAckMode ack_mode = RecordSyncPhoneAckMode::None;
			blepipe_hdr_t request_hdr { };
			uint8_t raw_payload[sizeof(exo::StartRecordMessage)] { };
			uint8_t raw_len = 0U;
			uint8_t command_id = kRecordSyncNoCommand;
			uint8_t heartbeat_phase = 0U;
			uint32_t heartbeat_last_ms = 0U;
			uint8_t stream_interval_ms = 20U;
	};
	static RecordSyncState g_record_sync { };
	static uint32_t g_local_bno_append_fail = 0U;
	static uint32_t g_local_icm_append_fail = 0U;
	static bool g_local_stop_requested = false;
	static uint32_t g_local_finalize_duration_ms = 0U;
	static uint8_t g_active_session_node_mask = 0U;
	static uint32_t g_active_session_id = 0U;
	static exo::MasterSdSessionRecorder g_local_session_recorder { };
#if EXO_ACQ_DIAG_ENABLE
	static exo::diag::AcquisitionDiagnostics g_acq_diag { };

	/* Runs only after the capture has ended, so the formatting cost never lands
	 * inside a sensor deadline. Split across several EXO_LOG calls because the
	 * logger truncates at 224 bytes per call. */
	static void acq_diag_emit_summary(void)
	{
		const exo::diag::AcquisitionDiagnostics &d = g_acq_diag;
		const uint32_t ms = d.session_duration_ms();
		EXO_LOG("[ACQ][DIAG] session_ms=%lu loop_iters=%lu loop_max_us=%lu loop_mean_us=%lu gt5=%lu gt10=%lu gt20=%lu gt100=%lu\r\n",
				static_cast<unsigned long>(ms),
				static_cast<unsigned long>(d.loop.count),
				static_cast<unsigned long>(d.loop.max_us),
				static_cast<unsigned long>(d.loop.mean_us()),
				static_cast<unsigned long>(d.loop.over_5ms),
				static_cast<unsigned long>(d.loop.over_10ms),
				static_cast<unsigned long>(d.loop.over_20ms),
				static_cast<unsigned long>(d.loop.over_100ms));
		EXO_LOG("[ACQ][DIAG] bno svc=%lu ok=%lu rate_chz=%lu max_gap_us=%lu svc_max_us=%lu svc_total_us=%lu i2c_err=%lu\r\n",
				static_cast<unsigned long>(d.bno_service_calls),
				static_cast<unsigned long>(d.bno_take_latest_ok),
				static_cast<unsigned long>(d.rate_centihz(d.bno_take_latest_ok)),
				static_cast<unsigned long>(d.bno_gap.max_gap_us),
				static_cast<unsigned long>(d.bno_service.max_us),
				static_cast<unsigned long>(d.bno_service.total_us),
				static_cast<unsigned long>(d.bno_i2c_errors));
		EXO_LOG("[ACQ][DIAG] bno events=%lu rv=%lu lin=%lu grav=%lu gyro=%lu other=%lu\r\n",
				static_cast<unsigned long>(d.bno_sensor_events),
				static_cast<unsigned long>(d.bno_reports[exo::diag::kBnoSlotRotation]),
				static_cast<unsigned long>(d.bno_reports[exo::diag::kBnoSlotLinearAccel]),
				static_cast<unsigned long>(d.bno_reports[exo::diag::kBnoSlotGravity]),
				static_cast<unsigned long>(d.bno_reports[exo::diag::kBnoSlotGyro]),
				static_cast<unsigned long>(d.bno_reports[exo::diag::kBnoSlotOther]));
		EXO_LOG("[ACQ][DIAG] icm att=%lu ok=%lu fail=%lu rate_chz=%lu max_gap_us=%lu rd_max_us=%lu rd_total_us=%lu\r\n",
				static_cast<unsigned long>(d.icm_attempts),
				static_cast<unsigned long>(d.icm_ok),
				static_cast<unsigned long>(d.icm_fail),
				static_cast<unsigned long>(d.rate_centihz(d.icm_ok)),
				static_cast<unsigned long>(d.icm_gap.max_gap_us),
				static_cast<unsigned long>(d.icm_read.max_us),
				static_cast<unsigned long>(d.icm_read.total_us));
		EXO_LOG("[ACQ][DIAG] sd bno_ok=%lu bno_fail=%lu icm_ok=%lu icm_fail=%lu flushes=%lu w_max_us=%lu gt5=%lu gt10=%lu gt20=%lu gt100=%lu\r\n",
				static_cast<unsigned long>(d.sd_bno_append_ok),
				static_cast<unsigned long>(d.sd_bno_append_fail),
				static_cast<unsigned long>(d.sd_icm_append_ok),
				static_cast<unsigned long>(d.sd_icm_append_fail),
				static_cast<unsigned long>(d.sd_flushes),
				static_cast<unsigned long>(d.sd_write.max_us),
				static_cast<unsigned long>(d.sd_write.over_5ms),
				static_cast<unsigned long>(d.sd_write.over_10ms),
				static_cast<unsigned long>(d.sd_write.over_20ms),
				static_cast<unsigned long>(d.sd_write.over_100ms));
		EXO_LOG("[ACQ][DIAG] comms ble_max_us=%lu ble_gt5=%lu central_max_us=%lu central_gt5=%lu rs485_max_us=%lu rs485_gt5=%lu\r\n",
				static_cast<unsigned long>(d.comms_ble.max_us),
				static_cast<unsigned long>(d.comms_ble.over_5ms),
				static_cast<unsigned long>(d.comms_central.max_us),
				static_cast<unsigned long>(d.comms_central.over_5ms),
				static_cast<unsigned long>(d.comms_rs485.max_us),
				static_cast<unsigned long>(d.comms_rs485.over_5ms));
		EXO_LOG("[ACQ][DIAG] modes sd_suppress=%u quiet_comms=%u icm_only=%u bno_only=%u bno_rv_only=%u summaries=%lu\r\n",
				static_cast<unsigned>(EXO_ACQ_DIAG_SUPPRESS_SD),
				static_cast<unsigned>(EXO_ACQ_DIAG_QUIET_COMMS),
				static_cast<unsigned>(EXO_ACQ_DIAG_ICM_ONLY),
				static_cast<unsigned>(EXO_ACQ_DIAG_BNO_ONLY),
				static_cast<unsigned>(EXO_ACQ_DIAG_BNO_RV_ONLY),
				static_cast<unsigned long>(g_acq_diag.summaries_emitted));
	}
#endif
	struct RecordTransferRuntimeConfig {
			uint8_t credit = exo::kRecordReliableDefaultCredit;
			uint8_t ack_every_chunks = 32U;
			uint16_t ack_every_ms = 350U;
			uint16_t control_heartbeat_ms = 400U;
			uint8_t nack_burst_chunks = 4U;
			uint8_t master_burst_limit = kLocalRecordBurstLimit;
			uint8_t master_chunk_gap_ms = static_cast<uint8_t>(kLocalRecordChunkGapMs);
			uint8_t flags = 0U;
	};
	static RecordTransferRuntimeConfig g_record_transfer_runtime { };

	enum class RecoveryJobKind : uint8_t {
		MasterLocalRetx,
		NodeUartRetx,
		VerifyRetx
	};

	struct RecoveryJob {
			bool active = false;
			RecoveryJobKind kind = RecoveryJobKind::MasterLocalRetx;
			uint16_t source_id = 0U;
			uint32_t session_id = 0U;
			uint32_t first_chunk = 0U;
			uint8_t count = 0U;
			uint8_t retry_count = 0U;
			uint32_t created_ms = 0U;
	};

	static constexpr uint8_t kRecoveryJobQueueSize = 8U;
	static RecoveryJob g_recovery_jobs[kRecoveryJobQueueSize] { };
	static uint32_t g_recovery_queue_drops = 0U;

	static void recovery_queue_clear()
	{
		for (RecoveryJob &job : g_recovery_jobs) {
			job = RecoveryJob { };
		}
	}

	static uint8_t recovery_queue_count()
	{
		uint8_t count = 0U;
		for (const RecoveryJob &job : g_recovery_jobs) {
			if (job.active) {
				count++;
			}
		}
		return count;
	}

	static bool recovery_job_overlaps(const RecoveryJob &job,
			RecoveryJobKind kind,
			uint16_t source_id,
			uint32_t session_id,
			uint32_t first_chunk,
			uint8_t count)
			{
		if (!job.active || job.kind != kind || job.source_id != source_id || job.session_id != session_id) {
			return false;
		}
		const uint32_t a0 = job.first_chunk;
		const uint32_t a1 = job.first_chunk + job.count;
		const uint32_t b0 = first_chunk;
		const uint32_t b1 = first_chunk + count;
		return (a0 <= b1) && (b0 <= a1);
	}

	static void recovery_queue_remove(uint8_t index)
			{
		if (index >= kRecoveryJobQueueSize) {
			return;
		}
		for (uint8_t i = index; (i + 1U) < kRecoveryJobQueueSize; ++i) {
			g_recovery_jobs[i] = g_recovery_jobs[i + 1U];
		}
		g_recovery_jobs[kRecoveryJobQueueSize - 1U] = RecoveryJob { };
	}

	static bool recovery_queue_push(RecoveryJobKind kind,
			uint16_t source_id,
			uint32_t session_id,
			uint32_t first_chunk,
			uint8_t count)
			{
		if (count == 0U) {
			count = 1U;
		}
		if (count > 16U) {
			count = 16U;
		}
		for (RecoveryJob &job : g_recovery_jobs) {
			if (recovery_job_overlaps(job, kind, source_id, session_id, first_chunk, count)) {
				const uint32_t start = (first_chunk < job.first_chunk) ? first_chunk : job.first_chunk;
				const uint32_t end_a = job.first_chunk + job.count;
				const uint32_t end_b = first_chunk + count;
				const uint32_t end = (end_a > end_b) ? end_a : end_b;
				job.first_chunk = start;
				job.count = static_cast<uint8_t>((end - start) > 16U ? 16U : (end - start));
				job.created_ms = HAL_GetTick();
				return true;
			}
		}
		for (RecoveryJob &job : g_recovery_jobs) {
			if (!job.active) {
				job.active = true;
				job.kind = kind;
				job.source_id = source_id;
				job.session_id = session_id;
				job.first_chunk = first_chunk;
				job.count = count;
				job.retry_count = 0U;
				job.created_ms = HAL_GetTick();
				EXO_LOG("[BLE][REC][Q] enqueue source=%u session=%lu first=%lu count=%u depth=%u\r\n",
						static_cast<unsigned>(source_id),
						static_cast<unsigned long>(session_id),
						static_cast<unsigned long>(first_chunk),
						static_cast<unsigned>(count),
						static_cast<unsigned>(recovery_queue_count()));
				return true;
			}
		}
		g_recovery_queue_drops++;
		EXO_LOG("[BLE][REC][Q] overflow source=%u session=%lu first=%lu count=%u drops=%lu\r\n",
				static_cast<unsigned>(source_id),
				static_cast<unsigned long>(session_id),
				static_cast<unsigned long>(first_chunk),
				static_cast<unsigned>(count),
				static_cast<unsigned long>(g_recovery_queue_drops));
		return false;
	}

	static int16_t clamp_i16(int32_t value)
			{
		if (value > 32767) {
			return 32767;
		}
		if (value < -32768) {
			return -32768;
		}
		return static_cast<int16_t>(value);
	}

	static int16_t rad_to_mdeg(float value_rad)
			{
		const float value_deg = value_rad * (180.0f / 3.14159265f);
		return clamp_i16(static_cast<int32_t>(value_deg * 1000.0f));
	}

	static uint8_t clamp_u8(uint32_t value, uint8_t min_value, uint8_t max_value)
			{
		if (value < min_value) {
			return min_value;
		}
		if (value > max_value) {
			return max_value;
		}
		return static_cast<uint8_t>(value);
	}

	static uint16_t clamp_u16(uint32_t value, uint16_t min_value, uint16_t max_value)
			{
		if (value < min_value) {
			return min_value;
		}
		if (value > max_value) {
			return max_value;
		}
		return static_cast<uint16_t>(value);
	}

	static uint8_t sanitize_receiver_credit(uint8_t credit)
			{
		const uint32_t requested = (credit == 0U) ? exo::kRecordReliableDefaultCredit : credit;
		return clamp_u8(requested, 1U, 24U);
	}

	static uint8_t runtime_local_record_burst_limit()
	{
		return clamp_u8(g_record_transfer_runtime.master_burst_limit, 1U, 8U);
	}

	static uint32_t runtime_local_record_chunk_gap_ms()
	{
		return static_cast<uint32_t>(clamp_u8(g_record_transfer_runtime.master_chunk_gap_ms, 0U, 5U));
	}

	static bool apply_record_transfer_tuning_payload(const uint8_t *payload, uint16_t length)
			{
		if (payload == nullptr || length < 12U || payload[0] != kRecordTransferTuningCmd || payload[1] != kRecordTransferTuningVersion) {
			return false;
		}
		g_record_transfer_runtime.credit = clamp_u8(payload[2], 1U, 24U);
		g_record_transfer_runtime.ack_every_chunks = clamp_u8(payload[3], 8U, 128U);
		g_record_transfer_runtime.ack_every_ms = clamp_u16(static_cast<uint32_t>(payload[4]) |
				(static_cast<uint32_t>(payload[5]) << 8), 100U, 5000U);
		g_record_transfer_runtime.control_heartbeat_ms = clamp_u16(static_cast<uint32_t>(payload[6]) |
				(static_cast<uint32_t>(payload[7]) << 8), 100U, 5000U);
		g_record_transfer_runtime.nack_burst_chunks = clamp_u8(payload[8], 1U, 16U);
		g_record_transfer_runtime.master_burst_limit = clamp_u8(payload[9], 1U, 8U);
		g_record_transfer_runtime.master_chunk_gap_ms = clamp_u8(payload[10], 0U, 5U);
		g_record_transfer_runtime.flags = payload[11];
		EXO_LOG("[BLE][REC][CFG] credit=%u ack_chunks=%u ack_ms=%u heartbeat_ms=%u nack_burst=%u master_burst=%u master_gap_ms=%u flags=0x%02X\r\n",
				static_cast<unsigned>(g_record_transfer_runtime.credit),
				static_cast<unsigned>(g_record_transfer_runtime.ack_every_chunks),
				static_cast<unsigned>(g_record_transfer_runtime.ack_every_ms),
				static_cast<unsigned>(g_record_transfer_runtime.control_heartbeat_ms),
				static_cast<unsigned>(g_record_transfer_runtime.nack_burst_chunks),
				static_cast<unsigned>(g_record_transfer_runtime.master_burst_limit),
				static_cast<unsigned>(g_record_transfer_runtime.master_chunk_gap_ms),
				static_cast<unsigned>(g_record_transfer_runtime.flags));
		return true;
	}

	static uint16_t resolve_remote_source_id(uint16_t source_id, uint32_t session_id)
			{
		if (source_id != 0U) {
			return source_id;
		}
		if (session_id == g_local_done.session_id) {
			switch (g_local_record_phase) {
				case LocalRecordPhase::RecordDoneWait:
					case LocalRecordPhase::Manifest:
					case LocalRecordPhase::TransferActive:
					case LocalRecordPhase::TransferPaused:
					case LocalRecordPhase::Resync:
					case LocalRecordPhase::Verifying:
					return kMasterNodeId;
				default:
					break;
			}
		}
		if (g_remote_transfer_active && g_remote_transfer_session_id == session_id &&
				g_remote_transfer_source_id != 0U) {
			return g_remote_transfer_source_id;
		}
		if (g_have_pending_node_done && g_pending_node_done.session_id == session_id &&
				g_pending_node_done.node_id != 0U) {
			return static_cast<uint16_t>(g_pending_node_done.node_id);
		}
		for (uint8_t i = 0U; i < g_pending_node_done_count; ++i) {
			const uint8_t idx = static_cast<uint8_t>((g_pending_node_done_head + i) % kPendingNodeDoneQueueSize);
			if (g_pending_node_done_queue[idx].session_id == session_id &&
					g_pending_node_done_queue[idx].node_id != 0U) {
				return static_cast<uint16_t>(g_pending_node_done_queue[idx].node_id);
			}
		}
		return source_id;
	}

	static void apply_ble_erm_output()
	{
		ERM_PWM.SET_PERCENT(g_ble_erm_percent);
	}

	static void apply_ble_buzzer_output()
	{
		BUZZER.SET_PERCENT(g_ble_buzzer_percent);
	}

	static void apply_ble_rgb_output()
	{
		RGB.SET((g_ble_rgb_mask & 0x01U) != 0U, (g_ble_rgb_mask & 0x02U) != 0U, (g_ble_rgb_mask & 0x04U) != 0U);
	}

	static bool g_ble_touch_test_active = false;
	static bool g_ble_poweroff_test_pending = false;
	static uint32_t g_ble_touch_test_until_ms = 0U;
	static uint32_t g_ble_poweroff_test_at_ms = 0U;
	static constexpr uint8_t kBlepipeStatusKindTouch = 0x03U;
	static constexpr uint8_t kTouchStatusReleased = 0U;
	static constexpr uint8_t kTouchStatusPressed = 1U;
	static constexpr uint8_t kTouchStatusShutdownArmed = 2U;
	static constexpr uint8_t kTouchStatusTurningOff = 3U;

	static void prepare_touch_wakeup_before_poweroff()
	{
		LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
		LL_EXTI_DisableIT_32_63 (LL_EXTI_LINE_48);
		LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
		HAL_PWR_DisableWakeUpPin (PWR_WAKEUP_PIN4);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);
	}

	static void poweroff_pcb_and_wait_for_release()
	{
		const uint8_t training_expected_mask = master_training_csv_coordinator.expected_source_mask();
		const uint8_t training_completed_mask = master_training_csv_coordinator.completed_source_mask();
		if (training_expected_mask != 0U &&
				(training_completed_mask & training_expected_mask) != training_expected_mask) {
			EXO_LOG("[TRAIN][CSV] incomplete session=%lu expected=0x%02X completed=0x%02X\r\n",
					static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
					static_cast<unsigned>(training_expected_mask),
					static_cast<unsigned>(training_completed_mask));
		}
		master_training_csv_coordinator.shutdown(HAL_GetTick());
#if EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
		if (master_imu_csv_logger.shutdown()) {
			EXO_LOG("[IMU][CSV] shutdown rows=%lu bno=%lu icm=%lu\r\n",
					static_cast<unsigned long>(master_imu_csv_logger.row_count()),
					static_cast<unsigned long>(master_imu_csv_logger.bno_count()),
					static_cast<unsigned long>(master_imu_csv_logger.icm_count()));
		} else {
			EXO_LOG("[IMU][CSV] shutdown failed op=%s fr=%d\r\n",
					exo::imu_csv::csv_log_operation_name(master_imu_csv_logger.last_operation()),
					static_cast<int>(master_imu_csv_logger.last_result()));
		}
#endif
		HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
		BUZZER.SET_PERCENT(0);
		ERM_PWM.SET_PERCENT(0);
		RGB.OFF();

		while (HAL_GPIO_ReadPin(TOUCH_MCU_GPIO_Port, TOUCH_MCU_Pin) == GPIO_PIN_SET) {
			HAL_Delay(10);
		}
		HAL_Delay(10000);

		NVIC_SystemReset();
	}

	static void send_touch_status(uint8_t node_id, uint8_t state)
			{
		if (APP_BLE_Get_Server_Connection_Status() != APP_BLE_CONNECTED_SERVER ||
				g_master_ble_status_notify_enabled == false) {
			return;
		}
		const uint8_t payload[3] = {
				kBlepipeStatusKindTouch,
				node_id,
				state
		};
		(void) master_blepipe_send(CUSTOM_STM_PIPESTATTX,
				BLEPIPE_MSG_STATUS,
				BLEPIPE_ID_BROADCAST,
				payload,
				static_cast<uint16_t>(sizeof(payload)));
	}

	static bool apply_ble_actuator_command(const uint8_t *payload, uint16_t length)
			{
		if (payload == nullptr || length < 2U) {
			return false;
		}
		switch (payload[0]) {
			case 0xA3U:
				g_ble_erm_percent = clamp_u8(payload[1], 0U, 100U);
				g_ble_actuator_override_enabled = true;
				apply_ble_erm_output();
				EXO_LOG("[BLE][HUB][CTRL] ERM=%u%%\r\n", static_cast<unsigned>(g_ble_erm_percent));
				return true;
			case 0xA4U:
				g_ble_buzzer_percent = clamp_u8(payload[1], 0U, 99U);
				g_ble_actuator_override_enabled = true;
				apply_ble_buzzer_output();
				EXO_LOG("[BLE][HUB][CTRL] buzzer=%u%%\r\n", static_cast<unsigned>(g_ble_buzzer_percent));
				return true;
			case 0xA5U:
				if ((payload[1] & 0x80U) != 0U) {
					g_ble_actuator_override_enabled = false;
					EXO_LOG("[BLE][HUB][CTRL] actuator override=OFF\r\n");
					return true;
				}
				g_ble_rgb_mask = static_cast<uint8_t>(payload[1] & 0x07U);
				g_ble_actuator_override_enabled = true;
				apply_ble_rgb_output();
				EXO_LOG("[BLE][HUB][CTRL] RGB mask=0x%02X\r\n", static_cast<unsigned>(g_ble_rgb_mask));
				return true;
			case 0xA6U:
				if (payload[1] == 0U) {
					g_ble_actuator_override_enabled = true;
					g_ble_touch_test_active = true;
					g_ble_touch_test_until_ms = HAL_GetTick() + 500U;
					g_ble_buzzer_percent = 50U;
					g_ble_rgb_mask = 0x07U;
					apply_ble_buzzer_output();
					apply_ble_rgb_output();
					send_touch_status(kMasterNodeId, kTouchStatusPressed);
					EXO_LOG("[BLE][HUB][CTRL] touch test feedback\r\n");
					return true;
				}
				if (payload[1] == 1U) {
					g_ble_actuator_override_enabled = true;
					g_ble_touch_test_active = false;
					g_ble_poweroff_test_pending = true;
					g_ble_poweroff_test_at_ms = HAL_GetTick() + 1000U;
					g_ble_erm_percent = 0U;
					g_ble_buzzer_percent = 0U;
					g_ble_rgb_mask = 0x01U;
					apply_ble_erm_output();
					apply_ble_buzzer_output();
					apply_ble_rgb_output();
					send_touch_status(kMasterNodeId, kTouchStatusTurningOff);
					EXO_LOG("[BLE][HUB][CTRL] poweroff test armed\r\n");
					return true;
				}
				return false;
			default:
				return false;
		}
	}

	static bool send_ble_v2_sample(uint16_t node_id, uint8_t sensor_id, const uint8_t *payload, uint8_t payload_len)
			{
		if (APP_BLE_Get_Server_Connection_Status() != APP_BLE_CONNECTED_SERVER) {
			return false;
		}
		uint8_t packet[244];
		const uint8_t packed_len = exo::ble_v2_pack(node_id, sensor_id, g_ble_sequence++, HAL_GetTick(),
				payload, payload_len, packet, static_cast<uint8_t>(sizeof(packet)));
		if (packed_len == 0U) {
			return false;
		}
		return Custom_APP_SendImuFrame(packet, packed_len) == BLE_STATUS_SUCCESS;
	}

	static void drain_leaf_stream_passthrough()
	{
		exo::ble_hub::HubLeafBleManager::LiveSample sample { };
		if (!master_rs485_recording.peek_next_live_sample(sample)) {
			return;
		}
		if (!g_ble_stream_enabled || g_ble_record_transfer_mode) {
			(void)master_rs485_recording.discard_next_live_sample();
			return;
		}
		if (!send_ble_v2_sample(sample.node_id,
				sample.sensor_id,
				sample.payload,
				sample.payload_len)) {
			// Keep the queued sample until the peripheral BLE stack accepts it.
			// This prevents transient phone-link backpressure from becoming a
			// visible gap in the Node trace.
			return;
		}
		(void)master_rs485_recording.discard_next_live_sample();
	}

	static bool is_duplicate_start_record(const exo::StartRecordMessage &message)
			{
		if (!g_have_last_ble_start_msg) {
			g_last_ble_start_msg = message;
			g_last_ble_start_tick_ms = HAL_GetTick();
			g_have_last_ble_start_msg = true;
			return false;
		}

		const bool same_message =
				(g_last_ble_start_msg.command == message.command) &&
						(g_last_ble_start_msg.session_id == message.session_id) &&
						(g_last_ble_start_msg.start_timestamp_us == message.start_timestamp_us) &&
						(g_last_ble_start_msg.requested_duration_ms == message.requested_duration_ms);
		const uint32_t now = HAL_GetTick();
		const bool in_guard_window = (now - g_last_ble_start_tick_ms) <= 250U;

		g_last_ble_start_msg = message;
		g_last_ble_start_tick_ms = now;
		g_have_last_ble_start_msg = true;
		return same_message && in_guard_window;
	}

	static bool is_probable_replay_start_record(const exo::StartRecordMessage &message)
			{
		if (!g_have_last_accepted_ble_start_msg) {
			return false;
		}
		const uint32_t now = HAL_GetTick();
		const bool same_session = (g_last_accepted_ble_start_msg.command == message.command) &&
				(g_last_accepted_ble_start_msg.session_id == message.session_id);
		const bool in_long_guard_window = (now - g_last_accepted_ble_start_tick_ms) <= 5000U;
		return same_session && in_long_guard_window;
	}

	static void mark_start_record_accepted(const exo::StartRecordMessage &message)
			{
		g_last_accepted_ble_start_msg = message;
		g_last_accepted_ble_start_tick_ms = HAL_GetTick();
		g_have_last_accepted_ble_start_msg = true;
	}

	static bool is_duplicate_chunk_ack(uint32_t session_id, uint16_t source_id, uint32_t next_offset)
			{
		const uint32_t now = HAL_GetTick();
		if (!g_have_last_chunk_ack) {
			g_last_chunk_ack_session_id = session_id;
			g_last_chunk_ack_source_id = source_id;
			g_last_chunk_ack_next_offset = next_offset;
			g_last_chunk_ack_tick_ms = now;
			g_have_last_chunk_ack = true;
			return false;
		}
		const bool same = (g_last_chunk_ack_session_id == session_id) &&
				(g_last_chunk_ack_source_id == source_id) &&
				(g_last_chunk_ack_next_offset == next_offset);
		const bool in_window = (now - g_last_chunk_ack_tick_ms) <= 80U;
		g_last_chunk_ack_session_id = session_id;
		g_last_chunk_ack_source_id = source_id;
		g_last_chunk_ack_next_offset = next_offset;
		g_last_chunk_ack_tick_ms = now;
		g_have_last_chunk_ack = true;
		return same && in_window;
	}

	__attribute__((optimize("Os"))) static void local_record_reset()
	{
		g_local_record_phase = LocalRecordPhase::Idle;
		g_local_record_armed = false;
		g_local_arm_tick_ms = 0U;
		g_local_capture_start_ms = 0U;
		g_local_chunk_seq = 1U;
		g_local_chunk_offset = 0U;
		g_local_last_chunk_tick = 0U;
		g_local_session_size = 0U;
		g_local_done_notified = false;
		g_local_manifest_acked = false;
		g_local_receiver_credit = 0U;
		g_local_requested_chunk = 0U;
		g_local_stream_cursor_chunk = 0U;
		g_local_retx_cursor_chunk = 0U;
		g_local_retx_remaining = 0U;
		g_local_forward_progress_tick_ms = 0U;
		g_local_stale_ack_repeat = 0U;
		g_local_pending_chunk = 0U;
		g_local_waiting_verify = false;
		g_local_manifest_last_tick = 0U;
		g_local_bno_append_fail = 0U;
		g_local_icm_append_fail = 0U;
		g_local_stop_requested = false;
		g_local_finalize_duration_ms = 0U;
		g_local_session_recorder.reset();
		recovery_queue_clear();
		memset(&g_local_done, 0, sizeof(g_local_done));
	}

	static void local_record_finish_without_transfer()
	{
		g_local_record_phase = LocalRecordPhase::Finished;
		master_rs485_recording.set_transfer_hold(false);
	}

	static void local_record_pause_for_resume()
	{
		g_local_receiver_credit = 0U;
		g_local_record_phase = LocalRecordPhase::TransferPaused;
		EXO_LOG("[MASTER][REC][REL] pause session=%lu chunk=%lu\r\n",
				static_cast<unsigned long>(g_local_done.session_id),
				static_cast<unsigned long>(g_local_requested_chunk));
	}

	static void process_recovery_queue()
	{
		for (uint8_t i = 0U; i < kRecoveryJobQueueSize; ++i) {
			RecoveryJob &job = g_recovery_jobs[i];
			if (!job.active) {
				continue;
			}
			if (job.kind == RecoveryJobKind::MasterLocalRetx || job.kind == RecoveryJobKind::VerifyRetx) {
				if (job.source_id != kMasterNodeId || job.session_id != g_local_done.session_id) {
					recovery_queue_remove(i);
					i--;
					continue;
				}
				if (g_local_retx_remaining == 0U) {
					g_local_retx_cursor_chunk = job.first_chunk;
					g_local_retx_remaining = job.count == 0U ? 1U : job.count;
					g_local_receiver_credit = g_local_retx_remaining;
					g_local_record_phase = LocalRecordPhase::Resync;
					EXO_LOG("[MASTER][REC][REL] queue drain session=%lu first=%lu count=%u depth=%u\r\n",
							static_cast<unsigned long>(job.session_id),
							static_cast<unsigned long>(job.first_chunk),
							static_cast<unsigned>(job.count),
							static_cast<unsigned>(recovery_queue_count()));
					recovery_queue_remove(i);
					i--;
				}
				return;
			}

			master_rs485_recording.on_ble_reliable_nack_range(job.session_id,
					job.source_id,
					job.first_chunk,
					job.count == 0U ? 1U : job.count);
			EXO_LOG("[BLE][HUB][REC][Q] drain source=%u session=%lu first=%lu count=%u\r\n",
					static_cast<unsigned>(job.source_id),
					static_cast<unsigned long>(job.session_id),
					static_cast<unsigned long>(job.first_chunk),
					static_cast<unsigned>(job.count));
			recovery_queue_remove(i);
			i--;
			return;
		}
	}

	static void MasterRs485_RecoverUartRx()
	{
		/* RS485 removed in BLE-only architecture. */
	}

	static bool local_reliable_transfer_owns_ble()
	{
		switch (g_local_record_phase) {
			case LocalRecordPhase::RecordDoneWait:
				case LocalRecordPhase::Manifest:
				case LocalRecordPhase::TransferActive:
				case LocalRecordPhase::TransferPaused:
				case LocalRecordPhase::Resync:
				case LocalRecordPhase::Verifying:
				return true;
			default:
				return false;
		}
	}

	static bool local_transfer_blocks_node_upload()
	{
		return local_reliable_transfer_owns_ble() &&
				g_local_record_phase != LocalRecordPhase::Finished &&
				g_local_record_phase != LocalRecordPhase::ErrorStoredCanResume &&
				g_local_record_phase != LocalRecordPhase::Cancelled;
	}

	static uint8_t pending_node_done_depth()
	{
		return static_cast<uint8_t>(g_pending_node_done_count + (g_have_pending_node_done ? 1U : 0U));
	}

	static void pending_node_done_clear()
	{
		g_have_pending_node_done = false;
		g_pending_node_manifest_last_tick = 0U;
		g_pending_node_defer_last_log_ms = 0U;
		memset(&g_pending_node_done, 0, sizeof(g_pending_node_done));
		memset(g_pending_node_done_queue, 0, sizeof(g_pending_node_done_queue));
		g_pending_node_done_head = 0U;
		g_pending_node_done_count = 0U;
		memset(g_seen_node_done_queue, 0, sizeof(g_seen_node_done_queue));
		g_seen_node_done_head = 0U;
		g_seen_node_done_count = 0U;
		memset(g_training_node_done, 0, sizeof(g_training_node_done));
		memset(g_training_node_done_valid, 0, sizeof(g_training_node_done_valid));
		memset(g_training_pending_verify_ok, 0, sizeof(g_training_pending_verify_ok));
	}

	static bool pending_node_done_matches(const exo::RecordDoneMessage &lhs,
			const exo::RecordDoneMessage &rhs)
			{
		return lhs.node_id == rhs.node_id &&
				lhs.session_id == rhs.session_id &&
				lhs.total_size == rhs.total_size &&
				lhs.payload_crc32 == rhs.payload_crc32;
	}

	static bool seen_node_done_contains(const exo::RecordDoneMessage &done)
			{
		for (uint8_t i = 0U; i < g_seen_node_done_count; ++i) {
			const uint8_t idx = static_cast<uint8_t>((g_seen_node_done_head + i) % kPendingNodeDoneQueueSize);
			if (pending_node_done_matches(g_seen_node_done_queue[idx], done)) {
				return true;
			}
		}
		return false;
	}

	static void seen_node_done_remember(const exo::RecordDoneMessage &done)
			{
		if (seen_node_done_contains(done)) {
			return;
		}
		if (g_seen_node_done_count < kPendingNodeDoneQueueSize) {
			const uint8_t tail = static_cast<uint8_t>((g_seen_node_done_head + g_seen_node_done_count) % kPendingNodeDoneQueueSize);
			g_seen_node_done_queue[tail] = done;
			++g_seen_node_done_count;
			return;
		}
		g_seen_node_done_queue[g_seen_node_done_head] = done;
		g_seen_node_done_head = static_cast<uint8_t>((g_seen_node_done_head + 1U) % kPendingNodeDoneQueueSize);
	}

	static bool pending_node_done_push(const exo::RecordDoneMessage &done)
			{
		if (done.node_id == 0U || done.node_id == kMasterNodeId || done.total_size == 0U) {
			return false;
		}
		if (g_have_pending_node_done && pending_node_done_matches(g_pending_node_done, done)) {
			return true;
		}
		for (uint8_t i = 0U; i < g_pending_node_done_count; ++i) {
			const uint8_t idx = static_cast<uint8_t>((g_pending_node_done_head + i) % kPendingNodeDoneQueueSize);
			if (pending_node_done_matches(g_pending_node_done_queue[idx], done)) {
				return true;
			}
		}
		if (g_pending_node_done_count >= kPendingNodeDoneQueueSize) {
			EXO_LOG("[BLE][REC][REL][NODEQ] overflow drop node=%u session=%lu size=%lu\r\n",
					static_cast<unsigned>(done.node_id),
					static_cast<unsigned long>(done.session_id),
					static_cast<unsigned long>(done.total_size));
			return false;
		}
		const uint8_t tail = static_cast<uint8_t>((g_pending_node_done_head + g_pending_node_done_count) % kPendingNodeDoneQueueSize);
		g_pending_node_done_queue[tail] = done;
		++g_pending_node_done_count;
		EXO_LOG("[BLE][REC][REL][NODEQ] queued node=%u session=%lu size=%lu queue=%u\r\n",
				static_cast<unsigned>(done.node_id),
				static_cast<unsigned long>(done.session_id),
				static_cast<unsigned long>(done.total_size),
				static_cast<unsigned>(pending_node_done_depth()));
		return true;
	}

	static bool pending_node_done_pop(exo::RecordDoneMessage &done)
			{
		if (g_pending_node_done_count == 0U) {
			return false;
		}
		done = g_pending_node_done_queue[g_pending_node_done_head];
		memset(&g_pending_node_done_queue[g_pending_node_done_head], 0, sizeof(g_pending_node_done_queue[g_pending_node_done_head]));
		g_pending_node_done_head = static_cast<uint8_t>((g_pending_node_done_head + 1U) % kPendingNodeDoneQueueSize);
		--g_pending_node_done_count;
		return true;
	}

	static void drain_pending_node_record_done()
	{
		exo::RecordDoneMessage done { };
		while (g_pending_node_done_count < kPendingNodeDoneQueueSize &&
				master_rs485_recording.pop_next_record_done(done)) {
			(void) pending_node_done_push(done);
		}
	}

	static void master_training_csv_replay_pending_node_done()
	{
		if (master_training_csv_coordinator.state() != exo::TrainingCsvState::WaitingForNode) {
			return;
		}
		const uint32_t session_id = master_training_csv_coordinator.active_session_id();
		if (!g_have_pending_node_done || g_pending_node_done.session_id != session_id ||
				g_pending_node_done.node_id < 1U || g_pending_node_done.node_id > 4U) {
			return;
		}
		const uint8_t index = static_cast<uint8_t>(g_pending_node_done.node_id - 1U);
		if (!g_training_node_done_valid[index] ||
				!pending_node_done_matches(g_training_node_done[index], g_pending_node_done)) {
			return;
		}
		master_training_csv_coordinator.on_node_record_done(g_training_node_done[index]);
		if (master_training_csv_coordinator.state() != exo::TrainingCsvState::WaitingForNode) {
			g_training_node_done_valid[index] = false;
		}
	}

	static bool master_training_csv_node_manifest_ready(const exo::RecordDoneMessage &done)
	{
		if (master_training_csv_coordinator.active_session_id() != done.session_id ||
				done.node_id < 1U || done.node_id > 4U ||
				(master_training_csv_coordinator.expected_source_mask() &
						static_cast<uint8_t>(1U << done.node_id)) == 0U) {
			return true;
		}
		master_training_csv_replay_pending_node_done();
		const exo::TrainingCsvState state = master_training_csv_coordinator.state();
		if (state == exo::TrainingCsvState::ReceiveNode &&
				master_training_csv_coordinator.active_node_id() == done.node_id) {
			return true;
		}
		return state == exo::TrainingCsvState::Idle ||
				state == exo::TrainingCsvState::Complete ||
				state == exo::TrainingCsvState::CsvError ||
				state == exo::TrainingCsvState::StageError;
	}

	static void start_next_pending_node_manifest_now()
	{
		if (g_have_pending_node_done || g_remote_transfer_active) {
			return;
		}
		drain_pending_node_record_done();
		g_have_pending_node_done = pending_node_done_pop(g_pending_node_done);
		if (!g_have_pending_node_done) {
			return;
		}
		g_pending_node_manifest_last_tick = 0U;
		master_training_csv_replay_pending_node_done();
		if (!master_training_csv_node_manifest_ready(g_pending_node_done)) {
			return;
		}
		EXO_LOG("[BLE][REC][REL][NODEQ] start node manifest immediate source=%u session=%lu size=%lu queue=%u\r\n",
				static_cast<unsigned>(g_pending_node_done.node_id),
				static_cast<unsigned long>(g_pending_node_done.session_id),
				static_cast<unsigned long>(g_pending_node_done.total_size),
				static_cast<unsigned>(pending_node_done_depth()));
		const bool node_manifest_ok = send_reliable_manifest(
				g_pending_node_done.node_id,
				g_pending_node_done.session_id,
				g_pending_node_done.total_size,
				g_pending_node_done.payload_crc32,
				g_pending_node_done.actual_duration_ms);
		if (node_manifest_ok) {
			g_pending_node_manifest_last_tick = HAL_GetTick();
			EXO_LOG("[BLE][REC][REL] MANIFEST source=%u session=%lu size=%lu crc=0x%08lX immediate=1\r\n",
					static_cast<unsigned>(g_pending_node_done.node_id),
					static_cast<unsigned long>(g_pending_node_done.session_id),
					static_cast<unsigned long>(g_pending_node_done.total_size),
					static_cast<unsigned long>(g_pending_node_done.payload_crc32));
		}
	}

	static uint8_t master_record_reset_all(bool erase_remote)
			{
		master_training_csv_coordinator.shutdown(HAL_GetTick());
		if (master_training_csv_coordinator.state() != exo::TrainingCsvState::Idle) {
			EXO_LOG("[CSV][TRAIN][RESET] cleanup_pending state=%u\r\n",
					static_cast<unsigned>(master_training_csv_coordinator.state()));
		}
		g_ble_record_transfer_mode = false;
		g_ble_start_or_record_in_progress = false;
		g_have_last_chunk_ack = false;
		g_last_chunk_ack_session_id = 0U;
		g_last_chunk_ack_source_id = 0U;
		g_last_chunk_ack_next_offset = 0U;
		g_last_chunk_ack_tick_ms = 0U;
		pending_node_done_clear();
		g_record_sync = RecordSyncState { };
		g_remote_transfer_active = false;
		g_remote_transfer_source_id = 0U;
		g_remote_transfer_session_id = 0U;
		local_record_reset();
		const bool ok = master_rs485_recording.reset_and_abort_all(erase_remote);
		if (ok) {
			g_last_master_reset_tick_ms = HAL_GetTick();
			g_have_last_master_reset_tick = true;
		}
		EXO_LOG("[BLE][HUB][RESET] erase_remote=%u result=%u\r\n",
				static_cast<unsigned>(erase_remote ? 1U : 0U),
				static_cast<unsigned>(ok ? 1U : 0U));
		return ok ? 1U : 0U;
	}

	static void ble_send_discovered_nodes_report()
	{
		uint8_t discovered[8] = { 0U };
		const uint8_t count = master_rs485_recording.copy_discovered_node_ids(
				discovered, static_cast<uint8_t>(sizeof(discovered)));
		uint8_t report[9] = { 0U };
		report[0] = count;
		for (uint8_t i = 0U; i < count && i < (uint8_t) (sizeof(report) - 1U); ++i) {
			report[1U + i] = discovered[i];
		}
		(void) Custom_APP_SendCmdReport(0xB4U, report, static_cast<uint8_t>(1U + count));
	}

	__attribute__((optimize("Os"))) static bool local_record_start(const exo::StartRecordMessage &msg,
			bool start_immediately = false)
			{
		local_record_reset();
		g_local_start_msg = msg;
		if (g_local_start_msg.requested_duration_ms == 0U) {
			g_local_start_msg.requested_duration_ms = kDefaultRecordDurationMs;
		}
		if (g_local_start_msg.start_timestamp_us == 0ULL) {
			g_local_start_msg.start_timestamp_us = kDefaultLeadTimeUs;
		}
		if (!g_local_session_recorder.start(kMasterNodeId,
				g_local_start_msg.session_id,
				g_local_start_msg.start_timestamp_us,
				g_local_start_msg.requested_duration_ms)) {
			EXO_LOG("[MASTER][REC] SD start failed fr=%d\r\n",
					static_cast<int>(g_local_session_recorder.last_error()));
			local_record_finish_without_transfer();
			return false;
		}
		if (start_immediately) {
			g_local_record_armed = false;
			g_local_arm_tick_ms = 0U;
			g_local_capture_start_ms = HAL_GetTick();
			g_local_record_phase = LocalRecordPhase::Capturing;
			EXO_LOG("[MASTER][REC] started immediate session=%lu duration_ms=%lu\r\n",
					static_cast<unsigned long>(g_local_start_msg.session_id),
					static_cast<unsigned long>(g_local_start_msg.requested_duration_ms));
			EXO_LOG("[RECORD][MASTER] START session=%lu duration_ms=%lu mode=immediate\r\n",
					static_cast<unsigned long>(g_local_start_msg.session_id),
					static_cast<unsigned long>(g_local_start_msg.requested_duration_ms));
		} else {
			g_local_record_armed = true;
			g_local_arm_tick_ms = HAL_GetTick();
			g_local_record_phase = LocalRecordPhase::Idle;
		}
		return true;
	}

	static uint32_t g_blepipe_control_tx_seq = 1U;

	static bool master_blepipe_send(Custom_STM_Char_Opcode_t char_opcode,
			uint8_t msg_type,
			uint16_t dst_id,
			const uint8_t *payload,
			uint16_t payload_len)
			{
		uint8_t packet[BLEPIPE_MAX_NOTIFY_PAYLOAD];
		size_t encoded_len = 0U;
		blepipe_hdr_t hdr { };
		hdr.proto_ver = BLEPIPE_PROTO_VER;
		hdr.msg_type = msg_type;
		hdr.flags = 0U;
		hdr.hop_count = 0U;
		hdr.src_id = BLEPIPE_ID_HUB;
		hdr.dst_id = dst_id;
		hdr.seq = g_blepipe_control_tx_seq++;
		hdr.timestamp_ms = HAL_GetTick();
		hdr.payload_len = payload_len;
		const blepipe_status_t status = blepipe_encode(packet,
				sizeof(packet),
				&hdr,
				payload,
				payload_len,
				&encoded_len);
		if (status != BLEPIPE_STATUS_OK || encoded_len > 255U) {
			EXO_LOG("[BLEPIPE][HUB] encode failed msg=0x%02X status=%u len=%u\r\n",
					static_cast<unsigned>(msg_type),
					static_cast<unsigned>(status),
					static_cast<unsigned>(payload_len));
			return false;
		}
		return Custom_APP_SendPipeFrame(char_opcode,
				packet,
				static_cast<uint8_t>(encoded_len)) == BLE_STATUS_SUCCESS;
	}

	static void record_sync_send_heartbeat(uint8_t phase, bool force)
			{
		if (!g_record_sync.active) {
			return;
		}
		const uint32_t now_ms = HAL_GetTick();
		if (!force &&
				g_record_sync.heartbeat_phase == phase &&
				(now_ms - g_record_sync.heartbeat_last_ms) < 1000U) {
			return;
		}
		blepipe_record_start_heartbeat_status_t status { };
		status.status_kind = BLEPIPE_STATUS_KIND_RECORD_START_HEARTBEAT;
		status.phase = phase;
		status.in_progress = 1U;
		status.source_id = 0U;
		status.session_id = g_record_sync.message.session_id;
		status.extend_timeout_ms = 5000U;
		(void) master_blepipe_send(CUSTOM_STM_PIPESTATTX,
				BLEPIPE_MSG_STATUS,
				BLEPIPE_ID_BROADCAST,
				reinterpret_cast<const uint8_t*>(&status),
				static_cast<uint16_t>(sizeof(status)));
		g_record_sync.heartbeat_phase = phase;
		g_record_sync.heartbeat_last_ms = now_ms;
	}

	static void master_blepipe_send_ack(const blepipe_hdr_t &request_hdr,
			uint8_t accepted,
			uint8_t command_id)
			{
		const uint8_t payload[2] = { command_id, accepted };
		(void) master_blepipe_send(CUSTOM_STM_PIPECTRLTX,
				accepted ? BLEPIPE_MSG_ACK : BLEPIPE_MSG_NACK,
				request_hdr.src_id,
				payload,
				static_cast<uint16_t>(sizeof(payload)));
	}

	static void master_blepipe_send_command_resp(const blepipe_hdr_t &request_hdr,
			const uint8_t *payload,
			uint16_t payload_len)
			{
		(void) master_blepipe_send(CUSTOM_STM_PIPECTRLTX,
				BLEPIPE_MSG_COMMAND_RESP,
				request_hdr.src_id,
				payload,
				payload_len);
	}

	static uint8_t record_sync_node_bit(uint8_t node_id)
			{
		return node_id < 8U ? static_cast<uint8_t>(1U << node_id) : 0U;
	}

	static uint8_t record_sync_send_to_target_mask(uint8_t target_mask,
			uint8_t msg_type,
			uint16_t src_id,
			const uint8_t *payload,
			uint16_t payload_len)
			{
		uint8_t sent_mask = 0U;
		for (uint8_t node_id = 1U; node_id < 8U; ++node_id) {
			const uint8_t bit = record_sync_node_bit(node_id);
			if ((target_mask & bit) == 0U) {
				continue;
			}
			if (exo_hub_central_client_send_blepipe_to_node(node_id,
					msg_type,
					src_id,
					payload,
					payload_len) != 0U) {
				sent_mask = static_cast<uint8_t>(sent_mask | bit);
			}
		}
		return sent_mask;
	}

	static void record_sync_normalize_message(exo::StartRecordMessage &message)
			{
		message.command = exo::RecordCommand::StartRecord;
		if (message.requested_duration_ms == 0U) {
			message.requested_duration_ms = kDefaultRecordDurationMs;
		}
		if (message.start_timestamp_us == 0ULL) {
			message.start_timestamp_us = kDefaultLeadTimeUs;
		}
	}

	static uint32_t remote_reset_cooldown_remaining_ms(uint32_t now_ms)
			{
		const int32_t remaining = static_cast<int32_t>(g_remote_reset_busy_until_ms - now_ms);
		return remaining > 0 ? static_cast<uint32_t>(remaining) : 0U;
	}

	static void mark_remote_reset_cooldown_if_needed(uint8_t target_mask)
			{
		if (target_mask == 0U) {
			return;
		}
		g_remote_reset_busy_until_ms = HAL_GetTick() + kRemoteResetCooldownMs;
		EXO_LOG("[BLE][HUB][RESET] remote cooldown target=0x%02X ms=%lu\r\n",
				static_cast<unsigned>(target_mask),
				static_cast<unsigned long>(kRemoteResetCooldownMs));
	}

	static void record_sync_send_phone_ack(uint8_t accepted)
			{
		if (g_record_sync.ack_mode == RecordSyncPhoneAckMode::Blepipe) {
			master_blepipe_send_ack(g_record_sync.request_hdr, accepted, g_record_sync.command_id);
		} else if (g_record_sync.ack_mode == RecordSyncPhoneAckMode::Raw && g_record_sync.raw_len > 0U) {
			(void) Custom_APP_SendCmdAck(g_record_sync.raw_payload, g_record_sync.raw_len, accepted);
		}
	}

	static void record_sync_clear(void)
			{
		g_record_sync = RecordSyncState { };
	}

	static void record_sync_abort(uint8_t notify_phone)
			{
		if (g_record_sync.target_mask != 0U) {
			exo::StartRecordMessage abort_msg = g_record_sync.message;
			abort_msg.command = exo::RecordCommand::AbortPreparedRecord;
			(void) record_sync_send_to_target_mask(g_record_sync.target_mask,
					BLEPIPE_MSG_COMMAND,
					BLEPIPE_ID_HUB,
					reinterpret_cast<const uint8_t*>(&abort_msg),
					static_cast<uint16_t>(sizeof(abort_msg)));
		}
		EXO_LOG("[BLE][HUB][SYNC] abort session=%lu target=0x%02X prepared=0x%02X failed=0x%02X\r\n",
				static_cast<unsigned long>(g_record_sync.message.session_id),
				static_cast<unsigned>(g_record_sync.target_mask),
				static_cast<unsigned>(g_record_sync.prepared_mask),
				static_cast<unsigned>(g_record_sync.failed_mask));
		if (notify_phone != 0U) {
			record_sync_send_phone_ack(0U);
		}
		g_ble_start_or_record_in_progress = false;
		g_ble_record_transfer_mode = false;
		record_sync_clear();
	}

	static uint8_t record_sync_commit_and_start(void)
			{
		const uint8_t prepared_target_mask = static_cast<uint8_t>(g_record_sync.prepared_mask & g_record_sync.target_mask);
		if (g_record_sync.target_mask != 0U && prepared_target_mask != g_record_sync.target_mask) {
			g_record_sync.failed_mask = static_cast<uint8_t>(g_record_sync.target_mask & ~prepared_target_mask);
			EXO_LOG("[BLE][HUB][SYNC] commit blocked missing_prepare session=%lu target=0x%02X prepared=0x%02X missing=0x%02X\r\n",
					static_cast<unsigned long>(g_record_sync.message.session_id),
					static_cast<unsigned>(g_record_sync.target_mask),
					static_cast<unsigned>(g_record_sync.prepared_mask),
					static_cast<unsigned>(g_record_sync.failed_mask));
			record_sync_abort(1U);
			return 0U;
		}
		record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_COMMIT_START, true);
		exo::StartRecordMessage commit_msg = g_record_sync.message;
		commit_msg.command = exo::RecordCommand::CommitPreparedRecord;
		const uint32_t commit_send_ms = HAL_GetTick();
		const uint8_t commit_mask = g_record_sync.target_mask != 0U ?
																		record_sync_send_to_target_mask(g_record_sync.target_mask,
																				BLEPIPE_MSG_COMMAND,
																				BLEPIPE_ID_HUB,
																				reinterpret_cast<const uint8_t*>(&commit_msg),
																				static_cast<uint16_t>(sizeof(commit_msg))) :
																		0U;
		if (g_record_sync.target_mask != 0U && commit_mask != g_record_sync.target_mask) {
			g_record_sync.failed_mask = static_cast<uint8_t>(g_record_sync.target_mask & ~commit_mask);
			EXO_LOG("[BLE][HUB][SYNC] commit send failed session=%lu target=0x%02X commit=0x%02X\r\n",
					static_cast<unsigned long>(g_record_sync.message.session_id),
					static_cast<unsigned>(g_record_sync.target_mask),
					static_cast<unsigned>(commit_mask));
			record_sync_abort(1U);
			return 0U;
		}
		g_ble_start_or_record_in_progress = true;
		const uint8_t ok = master_rs485_recording.start_from_ble(g_record_sync.message) ? 1U : 0U;
		g_ble_record_transfer_mode = false;
		if (ok == 0U) {
			EXO_LOG("[BLE][HUB][SYNC] local start failed session=%lu\r\n",
					static_cast<unsigned long>(g_record_sync.message.session_id));
			record_sync_abort(1U);
			return 0U;
		}
		master_rs485_recording.set_transfer_hold(false);
		const exo::StartRecordMessage &message = g_record_sync.message;
		if (!local_record_start(g_record_sync.message, true)) {
			EXO_LOG("[TRAIN][CSV] skipped session=%lu reason=master_binary_start\r\n",
					static_cast<unsigned long>(message.session_id));
		} else {
			const uint8_t expected_mask = exo::session_expected_source_mask(g_record_sync.target_mask);
			if (!master_training_csv_coordinator.begin_session(message.session_id, expected_mask)) {
				EXO_LOG("[TRAIN][CSV] setup failed session=%lu expected=0x%02X state=%u\r\n",
						static_cast<unsigned long>(message.session_id),
						static_cast<unsigned>(expected_mask),
						static_cast<unsigned>(master_training_csv_coordinator.state()));
			} else {
				master_training_csv_reported_state = exo::TrainingCsvState::WaitingForMaster;
				master_training_csv_reported_completed_mask = 0U;
				master_training_csv_reported_partial = false;
			}
		}
		g_active_session_node_mask = g_record_sync.target_mask;
		g_active_session_id = message.session_id;
		g_ble_stream_enabled = true;
		const uint8_t interval_command[2] = { 0xA2U, g_record_sync.stream_interval_ms };
		const uint8_t start_stream_command[1] = { 0xA0U };
		(void)record_sync_send_to_target_mask(g_active_session_node_mask,
				BLEPIPE_MSG_STREAM_CONTROL,
				BLEPIPE_ID_HUB,
				interval_command,
				static_cast<uint16_t>(sizeof(interval_command)));
		(void)record_sync_send_to_target_mask(g_active_session_node_mask,
				BLEPIPE_MSG_STREAM_CONTROL,
				BLEPIPE_ID_HUB,
				start_stream_command,
				static_cast<uint16_t>(sizeof(start_stream_command)));
		const uint32_t master_start_ms = HAL_GetTick();
		mark_start_record_accepted(g_record_sync.message);
		EXO_LOG("[BLE][HUB][SYNC] commit session=%lu target=0x%02X commit=0x%02X prepared=0x%02X\r\n",
				static_cast<unsigned long>(g_record_sync.message.session_id),
				static_cast<unsigned>(g_record_sync.target_mask),
				static_cast<unsigned>(commit_mask),
				static_cast<unsigned>(g_record_sync.prepared_mask));
		EXO_LOG("[RECORD][MASTER] COMMIT session=%lu target=0x%02X commit=0x%02X commit_send_ms=%lu master_start_ms=%lu\r\n",
				static_cast<unsigned long>(g_record_sync.message.session_id),
				static_cast<unsigned>(g_record_sync.target_mask),
				static_cast<unsigned>(commit_mask),
				static_cast<unsigned long>(commit_send_ms),
				static_cast<unsigned long>(master_start_ms));
		record_sync_send_phone_ack(1U);
		record_sync_clear();
		return 1U;
	}

	static uint8_t record_sync_send_prepare(void)
			{
		exo::StartRecordMessage prepare_msg = g_record_sync.message;
		prepare_msg.command = exo::RecordCommand::PrepareRecord;
		const uint8_t sent_mask = record_sync_send_to_target_mask(g_record_sync.target_mask,
				BLEPIPE_MSG_COMMAND,
				BLEPIPE_ID_HUB,
				reinterpret_cast<const uint8_t*>(&prepare_msg),
				static_cast<uint16_t>(sizeof(prepare_msg)));
		g_record_sync.prepare_sent = true;
		g_record_sync.deadline_ms = HAL_GetTick() + kRecordPrepareTimeoutMs;
		record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_PREPARE_SENT, true);
		EXO_LOG("[BLE][HUB][SYNC] prepare session=%lu target=0x%02X sent=0x%02X timeout=%lums\r\n",
				static_cast<unsigned long>(g_record_sync.message.session_id),
				static_cast<unsigned>(g_record_sync.target_mask),
				static_cast<unsigned>(sent_mask),
				static_cast<unsigned long>(kRecordPrepareTimeoutMs));
		if (sent_mask != g_record_sync.target_mask) {
			g_record_sync.failed_mask = static_cast<uint8_t>(g_record_sync.target_mask & ~sent_mask);
			record_sync_abort(1U);
			return 0U;
		}
		return 1U;
	}

	static uint8_t record_sync_begin(exo::StartRecordMessage message,
			RecordSyncPhoneAckMode ack_mode,
			const blepipe_hdr_t *request_hdr,
			const uint8_t *raw_payload,
			uint8_t raw_len,
			uint8_t requested_node_mask = 0U,
			uint8_t stream_interval_ms = 20U)
			{
		record_sync_normalize_message(message);
		if (g_record_sync.active) {
			const bool mask_matches = requested_node_mask == 0U ||
					requested_node_mask == g_record_sync.target_mask;
			const bool same_session =
					g_record_sync.message.session_id == message.session_id &&
					g_record_sync.message.start_timestamp_us == message.start_timestamp_us;
			if (mask_matches && same_session) {
				EXO_LOG("[BLE][HUB][SYNC] duplicate active start ignored session=%lu prepare_sent=%u prepared=0x%02X target=0x%02X\r\n",
						static_cast<unsigned long>(message.session_id),
						static_cast<unsigned>(g_record_sync.prepare_sent ? 1U : 0U),
						static_cast<unsigned>(g_record_sync.prepared_mask),
						static_cast<unsigned>(g_record_sync.target_mask));
				return 1U;
			}
			return 0U;
		}
		const uint8_t ready_node_mask = exo_hub_central_client_ready_node_mask();
		if (requested_node_mask != 0U &&
				(!exo::session_node_mask_valid(requested_node_mask) ||
				exo::session_missing_node_mask(requested_node_mask, ready_node_mask) != 0U)) {
			EXO_LOG("[BLE][HUB][SYNC] reject selected=0x%02X ready=0x%02X missing=0x%02X\r\n",
					static_cast<unsigned>(requested_node_mask),
					static_cast<unsigned>(ready_node_mask),
					static_cast<unsigned>(exo::session_missing_node_mask(requested_node_mask, ready_node_mask)));
			return 3U;
		}
		if (requested_node_mask != 0U) {
			const uint32_t capacity_duration_ms =
					exo_hub_central_client_maximum_duration_ms(requested_node_mask);
			if (capacity_duration_ms == 0U) {
				EXO_LOG("[BLE][HUB][SYNC] reject missing capacity selected=0x%02X\r\n",
						static_cast<unsigned>(requested_node_mask));
				return 3U;
			}
			if (message.requested_duration_ms > capacity_duration_ms) {
				EXO_LOG("[BLE][HUB][SYNC] clamp safety duration requested=%lu capacity=%lu mask=0x%02X\r\n",
						static_cast<unsigned long>(message.requested_duration_ms),
						static_cast<unsigned long>(capacity_duration_ms),
						static_cast<unsigned>(requested_node_mask));
				message.requested_duration_ms = capacity_duration_ms;
			}
		}
		if (g_ble_start_or_record_in_progress) {
			return 0U;
		}
		if (is_duplicate_start_record(message) || is_probable_replay_start_record(message)) {
			return 2U;
		}
		const uint32_t now_ms = HAL_GetTick();
		const bool recent_reset = g_have_last_master_reset_tick &&
				((now_ms - g_last_master_reset_tick_ms) <= 800U);
		if (!recent_reset && (master_record_reset_all(true) == 0U)) {
			return 0U;
		}

		g_record_sync = RecordSyncState { };
		g_record_sync.active = true;
		g_record_sync.message = message;
		g_record_sync.target_mask = requested_node_mask != 0U ?
				requested_node_mask : ready_node_mask;
		g_record_sync.stream_interval_ms = stream_interval_ms == 0U ? 20U : stream_interval_ms;
		const uint8_t ready_leaf_count = exo_hub_central_client_ready_node_count();
		const uint8_t transport_ready_leaf_count = exo_hub_central_client_transport_ready_node_count();
		const uint8_t transport_ready_mask = exo_hub_central_client_transport_ready_node_mask();
		g_record_sync.ack_mode = ack_mode;
		g_record_sync.command_id = static_cast<uint8_t>(exo::RecordCommand::StartRecord);
		if (request_hdr != nullptr) {
			g_record_sync.request_hdr = *request_hdr;
		}
		if (raw_payload != nullptr && raw_len > 0U) {
			if (raw_len > sizeof(g_record_sync.raw_payload)) {
				raw_len = sizeof(g_record_sync.raw_payload);
			}
			memcpy(g_record_sync.raw_payload, raw_payload, raw_len);
			g_record_sync.raw_len = raw_len;
		}

		if (ready_leaf_count != 0U && g_record_sync.target_mask == 0U) {
			EXO_LOG("[BLE][HUB][SYNC] reject ready leaves without node id count=%u session=%lu\r\n",
					static_cast<unsigned>(ready_leaf_count),
					static_cast<unsigned long>(message.session_id));
			record_sync_send_phone_ack(0U);
			record_sync_clear();
			return 3U;
		}

		if (g_record_sync.target_mask == 0U && transport_ready_leaf_count != 0U) {
			g_record_sync.deadline_ms = HAL_GetTick() + kRecordAppReadyTimeoutMs;
			record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_WAIT_APP_READY, true);
			EXO_LOG("[BLE][HUB][SYNC] wait app-ready session=%lu transport=0x%02X timeout=%lums\r\n",
					static_cast<unsigned long>(message.session_id),
					static_cast<unsigned>(transport_ready_mask),
					static_cast<unsigned long>(kRecordAppReadyTimeoutMs));
			return 1U;
		}

		if (g_record_sync.target_mask == 0U) {
			EXO_LOG("[BLE][HUB][SYNC] no ready leaves, local-only session=%lu\r\n",
					static_cast<unsigned long>(message.session_id));
			return record_sync_commit_and_start();
		}

		const uint32_t cooldown_ms = remote_reset_cooldown_remaining_ms(HAL_GetTick());
		if (cooldown_ms != 0U) {
			record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_RESET_COOLDOWN, true);
			EXO_LOG("[BLE][HUB][SYNC] prepare deferred reset_cooldown_ms=%lu session=%lu target=0x%02X\r\n",
					static_cast<unsigned long>(cooldown_ms),
					static_cast<unsigned long>(message.session_id),
					static_cast<unsigned>(g_record_sync.target_mask));
			return 1U;
		}
		return record_sync_send_prepare();
	}

	static bool stop_active_session(const exo::StopRecordMessage &message)
	{
		if (message.command != exo::RecordCommand::StopRecord) {
			return false;
		}
		if (g_record_sync.active) {
			if (g_record_sync.message.session_id != message.session_id) {
				return false;
			}
			record_sync_abort(0U);
			g_ble_stream_enabled = false;
			return true;
		}
		if (g_active_session_id != message.session_id) {
			return false;
		}
		if (g_local_stop_requested ||
				g_local_record_phase != LocalRecordPhase::Capturing) {
			return true;
		}
		g_ble_stream_enabled = false;
		g_ble_record_transfer_mode = true;
		g_local_stop_requested = true;
		const uint8_t stop_stream_command[1] = { 0xA1U };
		(void)record_sync_send_to_target_mask(g_active_session_node_mask,
				BLEPIPE_MSG_STREAM_CONTROL,
				BLEPIPE_ID_HUB,
				stop_stream_command,
				static_cast<uint16_t>(sizeof(stop_stream_command)));
		const uint8_t sent_mask = record_sync_send_to_target_mask(g_active_session_node_mask,
				BLEPIPE_MSG_COMMAND,
				BLEPIPE_ID_HUB,
				reinterpret_cast<const uint8_t*>(&message),
				static_cast<uint16_t>(sizeof(message)));
		EXO_LOG("[BLE][HUB][STOP] session=%lu target=0x%02X sent=0x%02X\r\n",
				static_cast<unsigned long>(message.session_id),
				static_cast<unsigned>(g_active_session_node_mask),
				static_cast<unsigned>(sent_mask));
		return sent_mask == g_active_session_node_mask;
	}

	static void record_sync_process(void)
			{
		if (!g_record_sync.active) {
			return;
		}
		if (!g_record_sync.prepare_sent) {
			if (g_record_sync.target_mask == 0U) {
				const uint8_t ready_mask = exo_hub_central_client_ready_node_mask();
				const uint8_t transport_mask = exo_hub_central_client_transport_ready_node_mask();
				if (ready_mask == 0U ||
						(transport_mask != 0U && ready_mask != transport_mask)) {
					if ((int32_t) (HAL_GetTick() - g_record_sync.deadline_ms) >= 0) {
						EXO_LOG("[BLE][HUB][SYNC] app-ready timeout session=%lu ready=0x%02X transport=0x%02X\r\n",
								static_cast<unsigned long>(g_record_sync.message.session_id),
								static_cast<unsigned>(ready_mask),
								static_cast<unsigned>(transport_mask));
						record_sync_abort(1U);
					}
					record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_WAIT_APP_READY, false);
					return;
				}
				g_record_sync.target_mask = ready_mask;
				EXO_LOG("[BLE][HUB][SYNC] app-ready acquired session=%lu target=0x%02X\r\n",
						static_cast<unsigned long>(g_record_sync.message.session_id),
						static_cast<unsigned>(g_record_sync.target_mask));
				record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_APP_READY_ACQUIRED, true);
			}
			if (remote_reset_cooldown_remaining_ms(HAL_GetTick()) != 0U) {
				record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_RESET_COOLDOWN, false);
				return;
			}
			(void) record_sync_send_prepare();
			return;
		}
		if (g_record_sync.failed_mask != 0U) {
			record_sync_abort(1U);
			return;
		}
		if ((g_record_sync.prepared_mask & g_record_sync.target_mask) == g_record_sync.target_mask) {
			(void) record_sync_commit_and_start();
			return;
		}
		record_sync_send_heartbeat(BLEPIPE_RECORD_START_PHASE_WAIT_PREPARE_ACK, false);
		if ((int32_t) (HAL_GetTick() - g_record_sync.deadline_ms) >= 0) {
			g_record_sync.failed_mask = static_cast<uint8_t>(g_record_sync.target_mask & ~g_record_sync.prepared_mask);
			EXO_LOG("[BLE][HUB][SYNC] prepare timeout session=%lu missing=0x%02X\r\n",
					static_cast<unsigned long>(g_record_sync.message.session_id),
					static_cast<unsigned>(g_record_sync.failed_mask));
			record_sync_abort(1U);
		}
	}

	static void master_blepipe_send_topology(const blepipe_hdr_t *request_hdr)
			{
		uint8_t discovered[8] = { 0U };
		const uint8_t count = master_rs485_recording.copy_discovered_node_ids(
				discovered,
				static_cast<uint8_t>(sizeof(discovered)));
		uint8_t payload[9] = { 0U };
		payload[0] = count;
		for (uint8_t i = 0U; i < count && i < static_cast<uint8_t>(sizeof(payload) - 1U); ++i) {
			payload[1U + i] = discovered[i];
		}
		(void) master_blepipe_send(CUSTOM_STM_PIPESTATTX,
				BLEPIPE_MSG_TOPOLOGY,
				request_hdr != nullptr ? request_hdr->src_id : BLEPIPE_ID_BROADCAST,
				payload,
				static_cast<uint16_t>(1U + count));
	}

	static bool master_handle_blepipe_command(const blepipe_hdr_t &hdr,
			const uint8_t *payload,
			uint16_t length)
			{
		if (payload == nullptr || length == 0U) {
			master_blepipe_send_ack(hdr, 0U, 0U);
			return false;
		}
		switch (payload[0]) {
			case 0xA0U:
				g_ble_stream_enabled = true;
				(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
						hdr.src_id,
						payload,
						length);
				master_blepipe_send_ack(hdr, 1U, payload[0]);
				return true;
			case 0xA1U:
				g_ble_stream_enabled = false;
				(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
						hdr.src_id,
						payload,
						length);
				master_blepipe_send_ack(hdr, 1U, payload[0]);
				return true;
			case 0xA2U:
				if (length >= 2U) {
					uint32_t requested = payload[1];
					if (requested < kMinStreamIntervalMs) {
						requested = kMinStreamIntervalMs;
					} else if (requested > kMaxStreamIntervalMs) {
						requested = kMaxStreamIntervalMs;
					}
					g_ble_stream_interval_ms = requested;
					(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
							hdr.src_id,
							payload,
							length);
					master_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case 0xA3U:
				case 0xA4U:
				case 0xA5U:
				case 0xA6U:
				if (length < 2U) {
					master_blepipe_send_ack(hdr, 0U, payload[0]);
					return false;
				}
				if (hdr.dst_id == BLEPIPE_ID_HUB || hdr.dst_id == kMasterNodeId || hdr.dst_id == BLEPIPE_ID_BROADCAST) {
					const bool ok = apply_ble_actuator_command(payload, length);
					master_blepipe_send_ack(hdr, ok ? 1U : 0U, payload[0]);
					return ok;
				}
				if (hdr.dst_id <= 0x00FFU &&
						exo_hub_central_client_send_blepipe_to_node(static_cast<uint8_t>(hdr.dst_id),
								BLEPIPE_MSG_COMMAND,
								hdr.src_id,
								payload,
								length) != 0U) {
					master_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case 0xB0U:
				if (length >= 3U) {
					const uint8_t current_id = payload[1];
					const uint8_t new_id = payload[2];
					if (exo_hub_central_client_send_blepipe_to_node(current_id,
							BLEPIPE_MSG_CONFIG_SET,
							hdr.src_id,
							payload,
							length) != 0U) {
						master_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
					const uint8_t ok = master_rs485_recording.provision_node_id(current_id, new_id) ? 1U : 0U;
					const uint8_t response[4] = { 0xB0U, current_id, new_id, ok };
					master_blepipe_send_command_resp(hdr, response, static_cast<uint16_t>(sizeof(response)));
					master_blepipe_send_topology(&hdr);
					master_blepipe_send_ack(hdr, ok, payload[0]);
					return ok != 0U;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case 0xB1U:
				{
				if (length >= 2U) {
					const uint8_t requested_id = payload[1];
					if (exo_hub_central_client_send_blepipe_to_node(requested_id,
							BLEPIPE_MSG_COMMAND,
							hdr.src_id,
							payload,
							length) != 0U) {
						master_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
					uint8_t resolved_id = 0U;
					const uint8_t ok = master_rs485_recording.request_node_id(requested_id, resolved_id) ? 1U : 0U;
					const uint8_t response[4] = { 0xB1U, requested_id, resolved_id, ok };
					master_blepipe_send_command_resp(hdr, response, static_cast<uint16_t>(sizeof(response)));
					master_blepipe_send_ack(hdr, ok, payload[0]);
					return ok != 0U;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			}
			case 0xB2U:
				exo_hub_central_client_request_scan();
				master_rs485_recording.rediscover_nodes();
				master_blepipe_send_topology(&hdr);
				master_blepipe_send_ack(hdr, 1U, payload[0]);
				return true;
			case 0xB3U:
				{
				const uint8_t target_mask = exo_hub_central_client_ready_node_mask();
				(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_COMMAND,
						hdr.src_id,
						payload,
						length);
				mark_remote_reset_cooldown_if_needed(target_mask);
				const uint8_t ok = master_record_reset_all(true);
				master_blepipe_send_topology(&hdr);
				master_blepipe_send_ack(hdr, ok, payload[0]);
				return ok != 0U;
			}
			case 0xB4U:
				master_blepipe_send_topology(&hdr);
				master_blepipe_send_ack(hdr, 1U, payload[0]);
				return true;
			case kRecordTransferTuningCmd:
				if (apply_record_transfer_tuning_payload(payload, length)) {
					master_blepipe_send_ack(hdr, 1U, payload[0]);
					return true;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case static_cast<uint8_t>(exo::RecordCommand::StartRecord):
				if (length == sizeof(exo::StartRecordMessage)) {
					exo::StartRecordMessage message { };
					memcpy(&message, payload, sizeof(message));
					const uint8_t sync_result = record_sync_begin(message,
							RecordSyncPhoneAckMode::Blepipe,
							&hdr,
							nullptr,
							0U);
					if (sync_result == 0U) {
						master_blepipe_send_ack(hdr, 0U, payload[0]);
						return false;
					}
					if (sync_result == 2U) {
						master_blepipe_send_ack(hdr, 1U, payload[0]);
						return true;
					}
					return sync_result == 1U;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case static_cast<uint8_t>(exo::RecordCommand::StartSession):
				if (length == sizeof(exo::StartSessionMessage)) {
					exo::StartSessionMessage session{};
					memcpy(&session, payload, sizeof(session));
					if (!exo::session_node_mask_valid(session.selected_node_mask) ||
							session.safety_duration_ms < 1000U ||
							session.stream_interval_ms == 0U) {
						master_blepipe_send_ack(hdr, 0U, payload[0]);
						return false;
					}
					exo::StartRecordMessage message{};
					message.command = exo::RecordCommand::StartRecord;
					message.session_id = session.session_id;
					message.start_timestamp_us = session.start_timestamp_us;
					message.requested_duration_ms = session.safety_duration_ms;
					const uint8_t sync_result = record_sync_begin(message,
							RecordSyncPhoneAckMode::Blepipe,
							&hdr,
							nullptr,
							0U,
							session.selected_node_mask,
							session.stream_interval_ms);
					if (g_record_sync.active) {
						g_record_sync.command_id =
								static_cast<uint8_t>(exo::RecordCommand::StartSession);
					}
					if (sync_result == 0U || sync_result == 3U) {
						master_blepipe_send_ack(hdr, 0U, payload[0]);
						return false;
					}
					if (sync_result == 2U) {
						master_blepipe_send_ack(hdr, 1U, payload[0]);
					}
					return true;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case static_cast<uint8_t>(exo::RecordCommand::StopRecord):
				if (length == sizeof(exo::StopRecordMessage)) {
					exo::StopRecordMessage message{};
					memcpy(&message, payload, sizeof(message));
					const bool ok = stop_active_session(message);
					master_blepipe_send_ack(hdr, ok ? 1U : 0U, payload[0]);
					return ok;
				}
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
			case static_cast<uint8_t>(exo::RecordCommand::ReliableFrame):
				case static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck):
				case static_cast<uint8_t>(exo::RecordCommand::ChunkAck):
				return false;
			default:
				master_blepipe_send_ack(hdr, 0U, payload[0]);
				return false;
		}
	}

	static uint8_t forward_remote_record_control(uint16_t source_id,
			const uint8_t *payload,
			uint8_t length)
			{
		if (source_id == 0U || source_id == kMasterNodeId || payload == nullptr || length == 0U) {
			return 0U;
		}
		return exo_hub_central_client_send_blepipe_to_node(static_cast<uint8_t>(source_id),
				BLEPIPE_MSG_COMMAND,
				BLEPIPE_ID_HUB,
				payload,
				length);
	}

	static bool master_training_csv_should_hold_node_verify(
			const exo::RecordReliableVerifyPayload &verify)
	{
		const exo::TrainingCsvState state = master_training_csv_coordinator.state();
		if (state == exo::TrainingCsvState::Idle || state == exo::TrainingCsvState::Complete) {
			return false;
		}
		if (verify.source_id < 1U || verify.source_id > 4U ||
				master_training_csv_coordinator.active_session_id() != verify.session_id) {
			return false;
		}
		const uint8_t source_bit = static_cast<uint8_t>(1U << verify.source_id);
		return (master_training_csv_coordinator.expected_source_mask() & source_bit) != 0U &&
				(master_training_csv_coordinator.completed_source_mask() & source_bit) == 0U;
	}

	static void training_pending_verify_store(
			const exo::RecordReliableVerifyPayload &verify,
			const uint8_t *payload,
			uint8_t length)
	{
		TrainingPendingVerifyOk &pending = g_training_pending_verify_ok[verify.source_id - 1U];
		pending.verify = verify;
		pending.length = length;
		memcpy(pending.frame, payload, length);
		pending.valid = true;
		if (g_remote_transfer_active &&
				g_remote_transfer_source_id == verify.source_id &&
				g_remote_transfer_session_id == verify.session_id) {
			g_remote_transfer_active = false;
			g_remote_transfer_source_id = 0U;
			g_remote_transfer_session_id = 0U;
			start_next_pending_node_manifest_now();
		}
		EXO_LOG("[TRAIN][CSV] VerifyOk held source=%u session=%lu completed=0x%02X\r\n",
				static_cast<unsigned>(verify.source_id),
				static_cast<unsigned long>(verify.session_id),
				static_cast<unsigned>(master_training_csv_coordinator.completed_source_mask()));
	}

	static void release_remote_node_verify_ok(
			const exo::RecordReliableVerifyPayload &verify,
			const uint8_t *payload,
			uint8_t length)
	{
		master_rs485_recording.on_ble_reliable_verify_ok(verify.session_id,
				verify.source_id,
				verify.file_crc32);
		if (g_remote_transfer_active &&
				g_remote_transfer_source_id == verify.source_id &&
				g_remote_transfer_session_id == verify.session_id) {
			g_remote_transfer_active = false;
			g_remote_transfer_source_id = 0U;
			g_remote_transfer_session_id = 0U;
			EXO_LOG("[BLE][REC][REL][NODEQ] node transfer complete source=%u session=%lu queue=%u\r\n",
					static_cast<unsigned>(verify.source_id),
					static_cast<unsigned long>(verify.session_id),
					static_cast<unsigned>(pending_node_done_depth()));
		}
		(void) send_reliable_record_frame(exo::RecordReliableType::CommitDone,
				verify.source_id,
				verify.session_id,
				0U,
				0U,
				reinterpret_cast<const uint8_t*>(&verify),
				static_cast<uint16_t>(sizeof(verify)));
		(void) forward_remote_record_control(verify.source_id, payload, length);
	}

	static void master_training_csv_release_completed_verify_ok()
	{
		const exo::TrainingCsvState state = master_training_csv_coordinator.state();
		if (state != exo::TrainingCsvState::Complete ||
				!master_training_csv_coordinator.logger().published()) {
			return;
		}
		const uint8_t completed_mask = master_training_csv_coordinator.completed_source_mask();
		for (uint8_t index = 0U; index < 4U; ++index) {
			TrainingPendingVerifyOk &pending = g_training_pending_verify_ok[index];
			const uint8_t source_bit = static_cast<uint8_t>(1U << (index + 1U));
			if (!pending.valid || (completed_mask & source_bit) == 0U) {
				continue;
			}
			TrainingPendingVerifyOk approved = pending;
			pending = TrainingPendingVerifyOk { };
			EXO_LOG("[TRAIN][CSV] VerifyOk released source=%u session=%lu completed=0x%02X\r\n",
					static_cast<unsigned>(approved.verify.source_id),
					static_cast<unsigned long>(approved.verify.session_id),
					static_cast<unsigned>(completed_mask));
			release_remote_node_verify_ok(approved.verify, approved.frame, approved.length);
			start_next_pending_node_manifest_now();
			return;
		}
	}

	__attribute__((optimize("Os"))) static void local_record_collect(const exo::HubSensorSnapshot &hub_snapshot)
			{
		if (g_local_record_armed) {
			const uint32_t lead_time_ms = static_cast<uint32_t>(g_local_start_msg.start_timestamp_us / 1000ULL);
			const uint32_t elapsed_from_cmd_ms = HAL_GetTick() - g_local_arm_tick_ms;
			if (elapsed_from_cmd_ms < lead_time_ms) {
				return;
			}
			g_local_record_armed = false;
			g_local_capture_start_ms = HAL_GetTick();
			g_local_record_phase = LocalRecordPhase::Capturing;
#if EXO_ACQ_DIAG_ENABLE
			/* Counters start clean here so a second session never inherits the
			 * first session's maxima. */
			hub_sensor_test_app.reset_bno_report_counts();
			g_acq_diag.begin_session(EXO_ACQ_DIAG_NOW_US());
#endif
			EXO_LOG("[RECORD][MASTER] START session=%lu duration_ms=%lu mode=delayed\r\n",
					static_cast<unsigned long>(g_local_start_msg.session_id),
					static_cast<unsigned long>(g_local_start_msg.requested_duration_ms));
		}
		if (g_local_record_phase != LocalRecordPhase::Capturing) {
			return;
		}
		const uint32_t sample_elapsed_ms = HAL_GetTick() - g_local_capture_start_ms;
		const uint32_t sample_offset_us = sample_elapsed_ms * 1000U;
		if (hub_snapshot.has_bno85) {
			exo::Bno85Sample bno_sample = hub_snapshot.bno85;
			// Force record-local timeline so MASTER overlaps NODE timelines in UI.
			bno_sample.offset_us = sample_offset_us;
#if EXO_ACQ_DIAG_SUPPRESS_SD
			/* Experiment B: acquisition keeps running, storage does not. */
			(void)bno_sample;
			const bool bno_stored = true;
#else
			const bool bno_stored = g_local_session_recorder.append_bno85(bno_sample);
#endif
			if (!bno_stored) {
				++g_local_bno_append_fail;
			}
#if EXO_ACQ_DIAG_ENABLE
			if (bno_stored) {
				++g_acq_diag.sd_bno_append_ok;
			} else {
				++g_acq_diag.sd_bno_append_fail;
			}
#endif
		}
		if (hub_snapshot.has_icm45686) {
			exo::Icm45686Sample icm_sample = hub_snapshot.icm45686;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
			icm_sample.offset_us = sample_offset_us;
			icm_sample.sample_timestamp_us = sample_offset_us;
#elif EXO_SAMPLE_FORMAT_VERSION == 4U
			icm_sample.offset_us = sample_offset_us;
#endif
#if EXO_ACQ_DIAG_SUPPRESS_SD
			/* Experiment B: acquisition keeps running, storage does not. */
			(void)icm_sample;
			const bool icm_stored = true;
#else
			const bool icm_stored = g_local_session_recorder.append_icm45686(icm_sample);
#endif
			if (icm_stored) {
				(void)master_training_csv_coordinator.note_master_icm_time(
						g_local_start_msg.session_id, sample_offset_us);
			} else {
				++g_local_icm_append_fail;
			}
#if EXO_ACQ_DIAG_ENABLE
			if (icm_stored) {
				++g_acq_diag.sd_icm_append_ok;
			} else {
				++g_acq_diag.sd_icm_append_fail;
			}
#endif
		}
		const uint32_t elapsed_ms = HAL_GetTick() - g_local_capture_start_ms;
		if (!g_local_stop_requested &&
				elapsed_ms < g_local_start_msg.requested_duration_ms) {
			return;
		}
		if (!g_local_stop_requested) {
			exo::StopRecordMessage safety_stop{};
			safety_stop.command = exo::RecordCommand::StopRecord;
			safety_stop.session_id = g_local_start_msg.session_id;
			(void)stop_active_session(safety_stop);
			EXO_LOG("[BLE][HUB][STOP] automatic capacity guard session=%lu elapsed_ms=%lu\r\n",
					static_cast<unsigned long>(safety_stop.session_id),
					static_cast<unsigned long>(elapsed_ms));
		}
		g_local_finalize_duration_ms = elapsed_ms;
#if EXO_ACQ_DIAG_ENABLE
		/* Capture is over from here on; the summary is only queued so the
		 * printing happens back in the superloop, never mid-capture. */
		g_acq_diag.end_session(EXO_ACQ_DIAG_NOW_US());
#endif

		g_local_session_recorder.set_capture_failures(
				g_local_bno_append_fail, g_local_icm_append_fail);
		if (!g_local_session_recorder.finalize(g_local_finalize_duration_ms)) {
			EXO_LOG("[MASTER][REC] finalize failed fr=%d bno_fail=%lu icm_fail=%lu\r\n",
					static_cast<int>(g_local_session_recorder.last_error()),
					static_cast<unsigned long>(g_local_bno_append_fail),
					static_cast<unsigned long>(g_local_icm_append_fail));
			if (master_training_csv_coordinator.active_session_id() ==
					g_local_start_msg.session_id) {
				(void)master_training_csv_coordinator.cancel_session();
			}
			local_record_finish_without_transfer();
			return;
		}
		master_training_csv_coordinator.on_master_finalized(g_local_session_recorder);
		if (master_training_csv_coordinator.state() == exo::TrainingCsvState::ConvertMasterBno) {
			if (!g_local_session_recorder.archive_to_index(
					master_training_csv_coordinator.logger().file_index())) {
				EXO_LOG("[TRAIN][CSV] master binary archive failed session=%lu index=%u\r\n",
						static_cast<unsigned long>(g_local_start_msg.session_id),
						static_cast<unsigned>(master_training_csv_coordinator.logger().file_index()));
				master_training_csv_coordinator.fail_durability();
			}
			EXO_LOG("[TRAIN][CSV] opened session=%lu path=%s expected=0x%02X\r\n",
					static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
					master_training_csv_coordinator.logger().path(),
					static_cast<unsigned>(master_training_csv_coordinator.expected_source_mask()));
			master_training_csv_reported_state = exo::TrainingCsvState::ConvertMasterBno;
		}
		const exo::SessionHeader &session_header = g_local_session_recorder.header();
		g_local_session_size = g_local_session_recorder.total_size();
		EXO_LOG("[MASTER][REC] finalized session=%lu bno=%lu icm=%lu size=%lu bno_fail=%lu icm_fail=%lu\r\n",
				static_cast<unsigned long>(session_header.session_id),
				static_cast<unsigned long>(session_header.bno85_sample_count),
				static_cast<unsigned long>(session_header.icm45686_sample_count),
				static_cast<unsigned long>(g_local_session_size),
				static_cast<unsigned long>(g_local_bno_append_fail),
				static_cast<unsigned long>(g_local_icm_append_fail));
		EXO_LOG("[RECORD][MASTER] END session=%lu duration_ms=%lu size=%lu bno=%lu icm=%lu\r\n",
				static_cast<unsigned long>(session_header.session_id),
				static_cast<unsigned long>(g_local_finalize_duration_ms),
				static_cast<unsigned long>(g_local_session_size),
				static_cast<unsigned long>(session_header.bno85_sample_count),
				static_cast<unsigned long>(session_header.icm45686_sample_count));

		g_local_done.command = exo::RecordCommand::RecordDone;
		g_local_done.node_id = kMasterNodeId;
		g_local_done.session_id = g_local_start_msg.session_id;
		g_local_done.actual_duration_ms = g_local_finalize_duration_ms;
		g_local_done.total_size = g_local_session_size;
		g_local_done.payload_crc32 = session_header.payload_crc32;
		g_local_record_phase = LocalRecordPhase::Manifest;
		master_rs485_recording.set_transfer_hold(true);
	}
}

#define HUB_START_LOG0(msg) EXO_LOG("[BLE][HUB][START] " msg "\r\n")
#define HUB_START_LOG1(fmt, a1) EXO_LOG("[BLE][HUB][START] " fmt "\r\n", (a1))
#define HUB_START_LOG2(fmt, a1, a2) EXO_LOG("[BLE][HUB][START] " fmt "\r\n", (a1), (a2))

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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_RTC_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_LPUART1_UART_Init();
	MX_USART1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	if (MX_FATFS_Init() != APP_OK) {
		Error_Handler();
	}
	MX_RF_Init();
	/* USER CODE BEGIN 2 */
	{
		const uint32_t old_prescaler = hspi1.Init.BaudRatePrescaler;
		const uint32_t new_prescaler = SpiNextFasterPrescaler(old_prescaler);
		if (new_prescaler != old_prescaler) {
			hspi1.Init.BaudRatePrescaler = new_prescaler;
			if (HAL_SPI_Init(&hspi1) != HAL_OK) {
				hspi1.Init.BaudRatePrescaler = old_prescaler;
				(void) HAL_SPI_Init(&hspi1);
			}
		}
		EXO_LOG("SPI1 prescaler (master SD/sensors) old=%lu new=%lu\r\n",
				static_cast<unsigned long>(old_prescaler),
				static_cast<unsigned long>(hspi1.Init.BaudRatePrescaler));
	}
	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
	prepare_touch_wakeup_before_poweroff();
	HAL_Delay(200);
#if EXO_MASTER_VERBOSE_DIAG
	EXO_LOG(
			"I2C setup: I2C1[timing=0x%08lX,addrMode=%lu,noStretch=%lu] I2C3[timing=0x%08lX,addrMode=%lu,noStretch=%lu]\r\n",
			static_cast<unsigned long>(hi2c1.Init.Timing),
			static_cast<unsigned long>(hi2c1.Init.AddressingMode),
			static_cast<unsigned long>(hi2c1.Init.NoStretchMode),
			static_cast<unsigned long>(hi2c3.Init.Timing),
			static_cast<unsigned long>(hi2c3.Init.AddressingMode),
			static_cast<unsigned long>(hi2c3.Init.NoStretchMode));
	{
		uint8_t devices_on_i2c1 = I2C_ScanBus(&hi2c1);
		uint8_t devices_on_i2c3 = I2C_ScanBus(&hi2c3);
		EXO_LOG("I2C1: %d device(s) found\r\n", devices_on_i2c1);
		EXO_LOG("I2C3: %d device(s) found\r\n", devices_on_i2c3);
	}
#endif
	const HAL_StatusTypeDef bno_probe = HAL_I2C_IsDeviceReady(&hi2c3, (0x4BU << 1U), 2U, 20U);
	const HAL_StatusTypeDef icm_probe = HAL_I2C_IsDeviceReady(&hi2c1, (0x69U << 1U), 2U, 20U);
	EXO_LOG("I2C probe: BNO85(0x4B,I2C3)=%s, ICM45686(0x69,I2C1)=%s\r\n",
			I2cReadyStr(bno_probe), I2cReadyStr(icm_probe));
#if EXO_ACQ_DIAG_ENABLE
	/* DWT-based; HAL_GetTick() cannot resolve the 5 ms ICM budget. */
	exo::diag::MicroClock::begin();
	hub_sensor_test_app.set_diagnostics(&g_acq_diag);
	g_local_session_recorder.set_diagnostics(&g_acq_diag);
	EXO_LOG("[ACQ][DIAG] clock started=%u core_hz=%lu\r\n",
			static_cast<unsigned>(exo::diag::MicroClock::started() ? 1U : 0U),
			static_cast<unsigned long>(SystemCoreClock));
#endif
	const bool hub_sensor_test_ready = hub_sensor_test_app.begin();
#if EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
	if (master_imu_csv_logger.begin(HAL_GetTick())) {
		master_imu_csv_error_reported = false;
		EXO_LOG("[IMU][CSV] opened path=%s\r\n", master_imu_csv_logger.path());
	} else {
		master_imu_csv_error_reported = true;
		EXO_LOG("[IMU][CSV] disabled op=%s fr=%d\r\n",
				exo::imu_csv::csv_log_operation_name(master_imu_csv_logger.last_operation()),
				static_cast<int>(master_imu_csv_logger.last_result()));
	}
#endif
#if EXO_MASTER_VERBOSE_DIAG
	EXO_LOG("Hub sensor test: %s, BNO85=%s, ICM45686=%s, SD log=%s, BNO_begin_status=%d, SD fr[mnt=%d mk=%d op=%d wr=%d bw=%u sy=%d]\r\n",
			hub_sensor_test_ready ? "ready" : "not ready",
			hub_sensor_test_app.bno_ready() ? "ready" : "not ready",
			hub_sensor_test_app.icm_ready() ? "ready" : "not ready",
			hub_sensor_test_app.sd_ready() ? "ready" : "not ready",
			hub_sensor_test_app.bno_begin_status(),
			hub_sensor_test_app.sd_last_mount_result(),
			hub_sensor_test_app.sd_last_mkdir_result(),
			hub_sensor_test_app.sd_last_open_result(),
			hub_sensor_test_app.sd_last_write_result(),
			hub_sensor_test_app.sd_last_written_bytes(),
			hub_sensor_test_app.sd_last_sync_result());
	EXO_LOG(
			"BNO diag: open=%d cb=%d cfg=%d probe=%s rd_hdr=%s rd_pkt=%s wr=%s i2c_err=0x%08lX ev=%lu rid=%u dec=%d\r\n",
			hub_sensor_test_app.bno_begin_open_status(),
			hub_sensor_test_app.bno_begin_callback_status(),
			hub_sensor_test_app.bno_begin_config_status(),
			HalStatusStr(hub_sensor_test_app.bno_open_probe_hal_status()),
			HalStatusStr(hub_sensor_test_app.bno_read_header_hal_status()),
			HalStatusStr(hub_sensor_test_app.bno_read_packet_hal_status()),
			HalStatusStr(hub_sensor_test_app.bno_write_hal_status()),
			static_cast<unsigned long>(hub_sensor_test_app.bno_last_i2c_error()),
			static_cast<unsigned long>(hub_sensor_test_app.bno_sensor_event_count()),
			static_cast<unsigned>(hub_sensor_test_app.bno_last_report_id()),
			hub_sensor_test_app.bno_last_decode_status());
#else
	EXO_LOG("Hub sensor test: %s\r\n", hub_sensor_test_ready ? "ready" : "not ready");
#endif
//#endif

#if 0
	for (int i = 0; i < 100; ++i) {
		ERM_PWM.SET_PERCENT(i);
		delay_ms(50);
	}
	for (int i = 100; i > -1; --i) {
		ERM_PWM.SET_PERCENT(i);
		delay_ms(50);
	}
#endif
	EXO_LOG("[BUILD][MASTER] ble-mtu-node-log-fix active\r\n");
	EXO_LOG("BLE stream init: frame=v2 interval_ms=%lu\r\n",
			static_cast<unsigned long>(kDefaultStreamIntervalMs));
	MasterRs485_StartUartDmaRx();
	master_rs485_recording.begin();
	exo_hub_central_client_init();
	{
		uint8_t discovered[8] = { 0U };
		const uint8_t count = master_rs485_recording.copy_discovered_node_ids(discovered, static_cast<uint8_t>(sizeof(discovered)));
		EXO_LOG("BLE leaf discovery: found %u node(s)\r\n", static_cast<unsigned>(count));
		for (uint8_t i = 0U; i < count; ++i) {
			EXO_LOG("BLE leaf[%u]=%u\r\n",
					static_cast<unsigned>(i),
					static_cast<unsigned>(discovered[i]));
		}
	}
	/* USER CODE END 2 */

	/* Init code for STM32_WPAN */
	MX_APPE_Init();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
#if EXO_ACQ_DIAG_ENABLE
		{
			/* Full iteration period, measured top-of-loop to top-of-loop, so it
			 * includes MX_APPE_Process() and every service call below it. */
			static uint64_t acq_loop_prev_us = 0U;
			static bool acq_loop_have_prev = false;
			const uint64_t acq_loop_now_us = EXO_ACQ_DIAG_NOW_US();
			if (acq_loop_have_prev && g_acq_diag.capturing) {
				g_acq_diag.loop.note(static_cast<uint32_t>(acq_loop_now_us - acq_loop_prev_us));
			}
			acq_loop_prev_us = acq_loop_now_us;
			acq_loop_have_prev = true;
		}
		{
			EXO_ACQ_DIAG_SCOPE(g_acq_diag.comms_ble);
			MX_APPE_Process();
		}
#else
		MX_APPE_Process();
#endif

		/* USER CODE BEGIN 3 */
		static uint32_t last_hub_sensor_print_ms = 0U;
		static uint32_t last_hub_sensor_rate_log_ms = 0U;
		static uint32_t hub_bno_rate_count = 0U;
		static uint32_t hub_icm_rate_count = 0U;
		static bool have_last_bno = false;
		static exo::Bno85Sample last_bno_sample { };
		static bool have_last_icm = false;
		static exo::Icm45686Sample last_icm_sample { };
		static exo::LiveStreamGate bno_stream_gate;
		static exo::LiveStreamGate icm_stream_gate;
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
		const exo::HubSensorSnapshot hub_snapshot = hub_sensor_test_app.process();
#if EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE
		const uint32_t csv_now_ms = HAL_GetTick();
		if (master_imu_csv_logger.ready() && (hub_snapshot.has_bno85 || hub_snapshot.has_icm45686)) {
			const uint64_t csv_timestamp_us = static_cast<uint64_t>(csv_now_ms) * 1000ULL;
			if (hub_snapshot.has_bno85) {
				(void)master_imu_csv_logger.append_bno(hub_snapshot.bno85,
						hub_sensor_test_app.bno_available_mask(),
						csv_timestamp_us,
						csv_now_ms);
			}
			if (hub_snapshot.has_icm45686 && master_imu_csv_logger.ready()) {
				(void)master_imu_csv_logger.append_icm(hub_snapshot.icm45686,
						csv_timestamp_us,
						csv_now_ms);
			}
		}
		if (master_imu_csv_logger.ready()) {
			(void)master_imu_csv_logger.service(csv_now_ms);
		}
		if (!master_imu_csv_logger.ready() && !master_imu_csv_error_reported) {
			master_imu_csv_error_reported = true;
			EXO_LOG("[IMU][CSV] disabled op=%s fr=%d\r\n",
					exo::imu_csv::csv_log_operation_name(master_imu_csv_logger.last_operation()),
					static_cast<int>(master_imu_csv_logger.last_result()));
		}
#endif
		local_record_collect(hub_snapshot);
		master_training_csv_replay_pending_node_done();
#if EXO_ACQ_DIAG_ENABLE
		if (g_acq_diag.claim_summary()) {
			/* Exactly one summary per completed session, emitted after capture. */
			acq_diag_emit_summary();
		}
		{
			EXO_ACQ_DIAG_SCOPE(g_acq_diag.comms_rs485);
			master_rs485_recording.process();
		}
		{
			EXO_ACQ_DIAG_SCOPE(g_acq_diag.comms_central);
			exo_hub_central_client_process();
		}
#else
		master_rs485_recording.process();
		exo_hub_central_client_process();
#endif
		record_sync_process();
		drain_leaf_stream_passthrough();
		master_training_csv_coordinator.service(g_local_session_recorder, HAL_GetTick());
		master_training_csv_release_completed_verify_ok();
		const exo::TrainingCsvState training_state = master_training_csv_coordinator.state();
		const exo::MasterTrainingCsvLogger &training_logger = master_training_csv_coordinator.logger();
		if (training_state == exo::TrainingCsvState::ConvertMasterBno &&
				master_training_csv_reported_state != exo::TrainingCsvState::ConvertMasterBno) {
			EXO_LOG("[TRAIN][CSV] opened session=%lu path=%s expected=0x%02X\r\n",
					static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
					training_logger.path(),
					static_cast<unsigned>(master_training_csv_coordinator.expected_source_mask()));
		}
		const uint8_t training_completed_mask = master_training_csv_coordinator.completed_source_mask();
		const uint8_t newly_completed_mask = static_cast<uint8_t>(training_completed_mask &
				~master_training_csv_reported_completed_mask);
		for (uint8_t source_id = 0U; source_id <= 4U; ++source_id) {
			if ((newly_completed_mask & static_cast<uint8_t>(1U << source_id)) != 0U) {
				EXO_LOG("[TRAIN][CSV] source complete source=%u bno=%lu icm=%lu completed=0x%02X\r\n",
						static_cast<unsigned>(source_id),
						static_cast<unsigned long>(training_logger.bno_count(source_id)),
						static_cast<unsigned long>(training_logger.icm_count(source_id)),
						static_cast<unsigned>(training_completed_mask));
			}
		}
		if (training_state != master_training_csv_reported_state) {
			if (training_state == exo::TrainingCsvState::StageError) {
				EXO_LOG("[TRAIN][CSV] validation failed session=%lu source=%u site=%u stage_op=%u stage_fr=%d csv_op=%u csv_fr=%d\r\n",
						static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
						static_cast<unsigned>(master_training_csv_coordinator.failure_node_id()),
						static_cast<unsigned>(master_training_csv_coordinator.failure_site()),
						static_cast<unsigned>(master_training_csv_coordinator.failure_stager_operation()),
						static_cast<int>(master_training_csv_coordinator.failure_stager_result()),
						static_cast<unsigned>(master_training_csv_coordinator.failure_csv_operation()),
						static_cast<int>(master_training_csv_coordinator.failure_csv_result()));
			} else if (training_state == exo::TrainingCsvState::CsvError) {
				EXO_LOG("[TRAIN][CSV] failed session=%lu site=%u op=%s fr=%d expected=0x%02X completed=0x%02X\r\n",
						static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
						static_cast<unsigned>(master_training_csv_coordinator.failure_site()),
						exo::training_csv::training_csv_log_operation_name(
								master_training_csv_coordinator.failure_csv_operation()),
						static_cast<int>(master_training_csv_coordinator.failure_csv_result()),
						static_cast<unsigned>(master_training_csv_coordinator.expected_source_mask()),
						static_cast<unsigned>(training_completed_mask));
			} else if (training_state == exo::TrainingCsvState::Complete) {
				EXO_LOG("[TRAIN][CSV] complete session=%lu expected=0x%02X completed=0x%02X rows=%lu\r\n",
						static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
						static_cast<unsigned>(master_training_csv_coordinator.expected_source_mask()),
						static_cast<unsigned>(training_completed_mask),
						static_cast<unsigned long>(training_logger.row_count()));
			}
		}
		/* SWO is not always attached and can lose the interesting window, so every training
		 * state change is also reported to the desktop tool over the existing report channel. */
		if (training_state != master_training_csv_reported_state ||
				training_completed_mask != master_training_csv_reported_completed_mask) {
			uint8_t training_report[9];
			training_report[0] = static_cast<uint8_t>(training_state);
			training_report[1] = master_training_csv_coordinator.expected_source_mask();
			training_report[2] = training_completed_mask;
			training_report[3] = static_cast<uint8_t>(master_training_csv_coordinator.failure_site());
			training_report[4] = master_training_csv_coordinator.failure_node_id();
			training_report[5] = static_cast<uint8_t>(master_training_csv_coordinator.failure_stager_operation());
			training_report[6] = static_cast<uint8_t>(master_training_csv_coordinator.failure_stager_result());
			training_report[7] = static_cast<uint8_t>(master_training_csv_coordinator.failure_csv_operation());
			training_report[8] = static_cast<uint8_t>(master_training_csv_coordinator.failure_csv_result());
			(void) Custom_APP_SendCmdReport(kTrainingCsvStatusReportId,
					training_report, static_cast<uint8_t>(sizeof(training_report)));
		}
		{
			const bool training_partial = master_training_csv_coordinator.partial_finalized();
			if (training_partial && !master_training_csv_reported_partial) {
				EXO_LOG("[TRAIN][CSV] incomplete retained session=%lu path=%s expected=0x%02X completed=0x%02X rows=%lu state=%u\r\n",
						static_cast<unsigned long>(master_training_csv_coordinator.active_session_id()),
						training_logger.path(),
						static_cast<unsigned>(master_training_csv_coordinator.expected_source_mask()),
						static_cast<unsigned>(training_completed_mask),
						static_cast<unsigned long>(training_logger.row_count()),
						static_cast<unsigned>(training_state));
			}
			master_training_csv_reported_partial = training_partial;
		}
		master_training_csv_reported_state = training_state;
		master_training_csv_reported_completed_mask = training_completed_mask;
		if (g_ble_start_or_record_in_progress &&
				!g_ble_record_transfer_mode &&
				!master_rs485_recording.start_or_record_active()) {
			g_ble_start_or_record_in_progress = false;
		}
		if (g_ble_touch_test_active && static_cast<int32_t>(HAL_GetTick() - g_ble_touch_test_until_ms) >= 0) {
			g_ble_touch_test_active = false;
			g_ble_actuator_override_enabled = false;
			g_ble_erm_percent = 0U;
			g_ble_buzzer_percent = 0U;
			g_ble_rgb_mask = 0U;
			apply_ble_erm_output();
			apply_ble_buzzer_output();
			apply_ble_rgb_output();
			send_touch_status(kMasterNodeId, kTouchStatusReleased);
		}
		if (g_ble_poweroff_test_pending && static_cast<int32_t>(HAL_GetTick() - g_ble_poweroff_test_at_ms) >= 0) {
			g_ble_poweroff_test_pending = false;
			g_ble_erm_percent = 0U;
			g_ble_buzzer_percent = 0U;
			g_ble_rgb_mask = 0U;
			apply_ble_erm_output();
			apply_ble_buzzer_output();
			apply_ble_rgb_output();
			send_touch_status(kMasterNodeId, kTouchStatusTurningOff);
			poweroff_pcb_and_wait_for_release();
		}
		if (g_ble_record_transfer_mode || master_rs485_recording.start_or_record_active()) {
			drain_pending_node_record_done();
			if (pending_node_done_depth() > 0U) {
				g_ble_record_transfer_mode = true;
				g_ble_start_or_record_in_progress = true;
			}
			if (pending_node_done_depth() > 0U && local_transfer_blocks_node_upload()) {
				const uint32_t now_ms = HAL_GetTick();
				if (g_pending_node_defer_last_log_ms == 0U ||
						(now_ms - g_pending_node_defer_last_log_ms) >= 1000U) {
					g_pending_node_defer_last_log_ms = now_ms;
					EXO_LOG("[BLE][REC][REL][NODEQ] deferred master_active queue=%u active=%u\r\n",
							static_cast<unsigned>(pending_node_done_depth()),
							static_cast<unsigned>(g_have_pending_node_done ? 1U : 0U));
				}
			}
			if (!g_have_pending_node_done &&
					!g_remote_transfer_active &&
					!local_transfer_blocks_node_upload()) {
				g_have_pending_node_done = pending_node_done_pop(g_pending_node_done);
				if (g_have_pending_node_done) {
					g_pending_node_manifest_last_tick = 0U;
					master_training_csv_replay_pending_node_done();
					EXO_LOG("[BLE][REC][REL][NODEQ] start node manifest source=%u session=%lu size=%lu queue=%u\r\n",
							static_cast<unsigned>(g_pending_node_done.node_id),
							static_cast<unsigned long>(g_pending_node_done.session_id),
							static_cast<unsigned long>(g_pending_node_done.total_size),
							static_cast<unsigned>(pending_node_done_depth()));
				}
			}
			if (g_have_pending_node_done &&
					!g_remote_transfer_active &&
					!local_transfer_blocks_node_upload() &&
					master_training_csv_node_manifest_ready(g_pending_node_done) &&
					(g_pending_node_manifest_last_tick == 0U || (HAL_GetTick() - g_pending_node_manifest_last_tick) >= 500U)) {
				const bool node_manifest_ok = send_reliable_manifest(
						g_pending_node_done.node_id,
						g_pending_node_done.session_id,
						g_pending_node_done.total_size,
						g_pending_node_done.payload_crc32,
						g_pending_node_done.actual_duration_ms);
				if (node_manifest_ok) {
					g_pending_node_manifest_last_tick = HAL_GetTick();
					EXO_LOG("[BLE][REC][REL] MANIFEST source=%u session=%lu size=%lu crc=0x%08lX\r\n",
							static_cast<unsigned>(g_pending_node_done.node_id),
							static_cast<unsigned long>(g_pending_node_done.session_id),
							static_cast<unsigned long>(g_pending_node_done.total_size),
							static_cast<unsigned long>(g_pending_node_done.payload_crc32));
				}
			}

			if ((g_local_record_phase == LocalRecordPhase::Manifest ||
					g_local_record_phase == LocalRecordPhase::RecordDoneWait) &&
					!g_local_manifest_acked &&
					(g_local_manifest_last_tick == 0U || (HAL_GetTick() - g_local_manifest_last_tick) >= 500U)) {
				const bool manifest_ok = send_reliable_manifest(kMasterNodeId,
						g_local_done.session_id,
						g_local_session_size,
						g_local_done.payload_crc32,
						g_local_done.actual_duration_ms);
				if (manifest_ok) {
					g_local_done_notified = true;
					g_local_record_phase = LocalRecordPhase::RecordDoneWait;
					g_local_manifest_last_tick = HAL_GetTick();
					g_local_chunk_seq = 1U;
					g_local_last_chunk_tick = HAL_GetTick();
					EXO_LOG("[MASTER][REC][REL] MANIFEST session=%lu size=%lu chunks=%u crc=0x%08lX\r\n",
							static_cast<unsigned long>(g_local_done.session_id),
							static_cast<unsigned long>(g_local_session_size),
							static_cast<unsigned>(reliable_total_chunks(g_local_session_size)),
							static_cast<unsigned long>(g_local_done.payload_crc32));
				}
			}

			process_recovery_queue();

			uint8_t local_burst_sent = 0U;
			const uint8_t local_burst_limit = runtime_local_record_burst_limit();
			const uint32_t local_chunk_gap_ms = runtime_local_record_chunk_gap_ms();
			while ((g_local_record_phase == LocalRecordPhase::TransferActive ||
					g_local_record_phase == LocalRecordPhase::Resync) &&
					g_local_receiver_credit > 0U &&
					local_burst_sent < local_burst_limit &&
					(local_burst_sent > 0U || (HAL_GetTick() - g_local_last_chunk_tick) >= local_chunk_gap_ms)) {
				const bool retransmit_chunk = (g_local_retx_remaining > 0U);
				const uint32_t chunk_index = retransmit_chunk ? g_local_retx_cursor_chunk : g_local_stream_cursor_chunk;
				const uint32_t offset = chunk_index * static_cast<uint32_t>(kRecordChunkPayloadBytes);
				if (offset >= g_local_session_size) {
					if (retransmit_chunk) {
						g_local_retx_remaining = 0U;
						g_local_receiver_credit = 0U;
						g_local_record_phase = LocalRecordPhase::TransferActive;
					} else {
						g_local_record_phase = LocalRecordPhase::Verifying;
						g_local_waiting_verify = true;
						g_local_receiver_credit = 0U;
						EXO_LOG("[MASTER][REC][REL] all chunks sent session=%lu waiting VERIFY_OK\r\n",
								static_cast<unsigned long>(g_local_done.session_id));
					}
				} else {
					const uint32_t remain = g_local_session_size - offset;
					const uint16_t chunk_sz = static_cast<uint16_t>(remain > kRecordChunkPayloadBytes ? kRecordChunkPayloadBytes : remain);
					uint8_t payload[kRecordChunkPayloadBytes];
					if (!g_local_session_recorder.read(offset, payload, chunk_sz)) {
						EXO_LOG("[MASTER][REC] chunk read failed off=%lu size=%u\r\n",
								static_cast<unsigned long>(offset),
								static_cast<unsigned>(chunk_sz));
						g_local_record_phase = LocalRecordPhase::ErrorStoredCanResume;
						g_local_receiver_credit = 0U;
					} else if (send_reliable_record_frame(exo::RecordReliableType::Chunk,
							kMasterNodeId,
							g_local_done.session_id,
							chunk_index,
							offset,
							payload,
							chunk_sz,
							(((offset + chunk_sz) >= g_local_session_size) ? exo::kRecordFlagFinalChunk : 0U) |
									(retransmit_chunk ? exo::kRecordFlagRetransmit : 0U))) {
						g_local_pending_chunk = chunk_index;
						if (retransmit_chunk) {
							g_local_retx_cursor_chunk = chunk_index + 1U;
							if (g_local_retx_remaining > 0U) {
								g_local_retx_remaining--;
							}
							if (g_local_retx_remaining == 0U) {
								g_local_record_phase = LocalRecordPhase::TransferActive;
							}
						} else {
							g_local_stream_cursor_chunk = chunk_index + 1U;
						}
						g_local_requested_chunk = g_local_stream_cursor_chunk;
						g_local_chunk_offset = offset + chunk_sz;
						g_local_chunk_seq++;
						g_local_receiver_credit--;
						g_local_last_chunk_tick = HAL_GetTick();
						g_local_forward_progress_tick_ms = g_local_last_chunk_tick;
						g_local_stale_ack_repeat = 0U;
						++local_burst_sent;
						EXO_LOG("[MASTER][REC][REL] CHUNK session=%lu idx=%lu off=%lu size=%u credit=%u retx=%u burst=%u\r\n",
								static_cast<unsigned long>(g_local_done.session_id),
								static_cast<unsigned long>(chunk_index),
								static_cast<unsigned long>(offset),
								static_cast<unsigned>(chunk_sz),
								static_cast<unsigned>(g_local_receiver_credit),
								static_cast<unsigned>(retransmit_chunk ? 1U : 0U),
								static_cast<unsigned>(local_burst_sent));
					} else {
						break;
					}
				}
			}

			if (g_local_record_phase == LocalRecordPhase::Finished) {
				if (!g_remote_transfer_active &&
						!g_have_pending_node_done &&
						g_pending_node_done_count == 0U &&
						!master_rs485_recording.start_or_record_active()) {
					g_ble_record_transfer_mode = false;
					g_ble_start_or_record_in_progress = false;
					local_record_reset();
				}
			}
		}
		if (hub_snapshot.has_bno85) {
			last_bno_sample = hub_snapshot.bno85;
			have_last_bno = true;
			++hub_bno_rate_count;
		}
		if (hub_snapshot.has_icm45686) {
			last_icm_sample = hub_snapshot.icm45686;
			have_last_icm = true;
			++hub_icm_rate_count;
		}
		if (last_hub_sensor_rate_log_ms == 0U) {
			last_hub_sensor_rate_log_ms = HAL_GetTick();
		}
		const uint32_t hub_rate_elapsed_ms = HAL_GetTick() - last_hub_sensor_rate_log_ms;
		if (hub_rate_elapsed_ms >= 10000U) {
			const uint32_t bno_rate_x100 = (hub_bno_rate_count * 100000U) / hub_rate_elapsed_ms;
			const uint32_t icm_rate_x100 = (hub_icm_rate_count * 100000U) / hub_rate_elapsed_ms;
			EXO_LOG("HUB IMU rate: BNO=%lu.%02lu Hz (%lu/%lu ms) ICM=%lu.%02lu Hz (%lu/%lu ms)\r\n",
					static_cast<unsigned long>(bno_rate_x100 / 100U),
					static_cast<unsigned long>(bno_rate_x100 % 100U),
					static_cast<unsigned long>(hub_bno_rate_count),
					static_cast<unsigned long>(hub_rate_elapsed_ms),
					static_cast<unsigned long>(icm_rate_x100 / 100U),
					static_cast<unsigned long>(icm_rate_x100 % 100U),
					static_cast<unsigned long>(hub_icm_rate_count),
					static_cast<unsigned long>(hub_rate_elapsed_ms));
			hub_bno_rate_count = 0U;
			hub_icm_rate_count = 0U;
			last_hub_sensor_rate_log_ms = HAL_GetTick();
		}

#if EXO_MASTER_IMU_SWO_ENABLE
		const uint32_t hub_sensor_print_now_ms = HAL_GetTick();
#if EXO_ACQ_DIAG_QUIET_COMMS
		/* Experiment C: per-sample SWO telemetry is diagnostic-only, so it is
		 * held off while capturing. Record-control traffic is untouched. */
		const bool hub_sensor_print_allowed = !g_acq_diag.capturing;
#else
		const bool hub_sensor_print_allowed = true;
#endif
		if (hub_sensor_print_allowed &&
				(hub_sensor_print_now_ms - last_hub_sensor_print_ms) >= EXO_MASTER_IMU_SWO_INTERVAL_MS &&
				(have_last_bno || have_last_icm)) {
			if (have_last_bno) {
				const uint8_t bno_available_mask = hub_sensor_test_app.bno_available_mask();
				EXO_LOG(
						"[IMU][BNO][Q] t_ms=%lu av=0x%02X q1e4=%ld,%ld,%ld,%ld\r\n",
						static_cast<unsigned long>(hub_sensor_print_now_ms),
						static_cast<unsigned>(bno_available_mask),
						static_cast<long>(exo::imu_swo::scale_quaternion_1e4(last_bno_sample.quat_i)),
						static_cast<long>(exo::imu_swo::scale_quaternion_1e4(last_bno_sample.quat_j)),
						static_cast<long>(exo::imu_swo::scale_quaternion_1e4(last_bno_sample.quat_k)),
						static_cast<long>(exo::imu_swo::scale_quaternion_1e4(last_bno_sample.quat_real)));
				if ((bno_available_mask & 0x0EU) == 0x0EU) {
					EXO_LOG(
							"[IMU][BNO][MOTION] lin_mms2=%ld,%ld,%ld grav_mms2=%ld,%ld,%ld gyro_mrads=%ld,%ld,%ld\r\n",
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.linear_accel_x)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.linear_accel_y)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.linear_accel_z)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gravity_x)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gravity_y)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gravity_z)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gyro_x)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gyro_y)),
							static_cast<long>(exo::imu_swo::scale_bno_milli(last_bno_sample.gyro_z)));
				}
			}
			if (have_last_icm) {
				EXO_LOG(
						"[IMU][ICM][RAW] t_ms=%lu accel_lsb=%d,%d,%d gyro_lsb=%d,%d,%d\r\n",
						static_cast<unsigned long>(hub_sensor_print_now_ms),
						static_cast<int>(last_icm_sample.accel_x),
						static_cast<int>(last_icm_sample.accel_y),
						static_cast<int>(last_icm_sample.accel_z),
						static_cast<int>(last_icm_sample.gyro_x),
						static_cast<int>(last_icm_sample.gyro_y),
						static_cast<int>(last_icm_sample.gyro_z));
				EXO_LOG(
						"[IMU][ICM][SCALED] accel_mg=%ld,%ld,%ld gyro_mdps=%ld,%ld,%ld\r\n",
						static_cast<long>(exo::imu_swo::scale_icm_accel_mg(last_icm_sample.accel_x)),
						static_cast<long>(exo::imu_swo::scale_icm_accel_mg(last_icm_sample.accel_y)),
						static_cast<long>(exo::imu_swo::scale_icm_accel_mg(last_icm_sample.accel_z)),
						static_cast<long>(exo::imu_swo::scale_icm_gyro_mdps(last_icm_sample.gyro_x)),
						static_cast<long>(exo::imu_swo::scale_icm_gyro_mdps(last_icm_sample.gyro_y)),
						static_cast<long>(exo::imu_swo::scale_icm_gyro_mdps(last_icm_sample.gyro_z)));
			}
			last_hub_sensor_print_ms = hub_sensor_print_now_ms;
		}
#endif

		const uint32_t ble_stream_now_ms = HAL_GetTick();
#if EXO_ACQ_DIAG_QUIET_COMMS
		/* Experiment C: the live plot stream is nonessential during a recorded
		 * capture; the stored session is the deliverable. */
		const bool g_ble_stream_enabled_now = (g_ble_stream_enabled != false) && !g_acq_diag.capturing;
#else
		const bool g_ble_stream_enabled_now = (g_ble_stream_enabled != false);
#endif
		if (g_ble_stream_enabled_now &&
				bno_stream_gate.accept(hub_snapshot.has_bno85,
						ble_stream_now_ms, g_ble_stream_interval_ms)) {
			const exo::Bno85Sample &bno_payload = hub_snapshot.bno85;
			if (!send_ble_v2_sample(0U, static_cast<uint8_t>(exo::BleSensorId::Bno85),
					reinterpret_cast<const uint8_t*>(&bno_payload), static_cast<uint8_t>(sizeof(bno_payload)))) {
				++g_ble_tx_drop_count;
			}
		}
		if (g_ble_stream_enabled_now &&
				icm_stream_gate.accept(hub_snapshot.has_icm45686,
						ble_stream_now_ms, g_ble_stream_interval_ms)) {
			exo::Icm45686Sample icm_payload { };
#if EXO_SAMPLE_FORMAT_VERSION == 2U
			icm_payload.offset_us = hub_snapshot.icm45686.offset_us;
#elif EXO_SAMPLE_FORMAT_VERSION == 4U
			icm_payload.offset_us = hub_snapshot.icm45686.offset_us;
			icm_payload.sequence = hub_snapshot.icm45686.sequence;
#endif
			icm_payload.accel_x = hub_snapshot.icm45686.accel_x;
			icm_payload.accel_y = hub_snapshot.icm45686.accel_y;
			icm_payload.accel_z = hub_snapshot.icm45686.accel_z;
			icm_payload.gyro_x = hub_snapshot.icm45686.gyro_x;
			icm_payload.gyro_y = hub_snapshot.icm45686.gyro_y;
			icm_payload.gyro_z = hub_snapshot.icm45686.gyro_z;
#if EXO_SAMPLE_FORMAT_VERSION == 2U
			icm_payload.temperature = hub_snapshot.icm45686.temperature;
			icm_payload.data_valid = hub_snapshot.icm45686.data_valid;
			icm_payload.read_status = hub_snapshot.icm45686.read_status;
			icm_payload.reserved0 = 0U;
			icm_payload.sample_timestamp_us = hub_snapshot.icm45686.sample_timestamp_us;
#endif
			if (!send_ble_v2_sample(0U, static_cast<uint8_t>(exo::BleSensorId::Icm45686),
					reinterpret_cast<const uint8_t*>(&icm_payload), static_cast<uint8_t>(sizeof(icm_payload)))) {
				++g_ble_tx_drop_count;
			}
		}

		const GPIO_PinState touch_pin_state = HAL_GPIO_ReadPin(TOUCH_MCU_GPIO_Port, TOUCH_MCU_Pin);
		const bool touch_active = (touch_pin_state == GPIO_PIN_SET);
		const uint32_t now_ms = HAL_GetTick();

		if (ignore_touch_until_release) {
			if (!touch_active) {
				ignore_touch_until_release = false;
			}
		} else if (g_ble_actuator_override_enabled == false) {
			switch (touch_shutdown_state) {
				case TouchShutdownState::Idle:
					if (touch_active) {
						BUZZER.SET_PERCENT(50);
						RGB.ON();
						touch_start_ms = now_ms;
						touch_feedback_until_ms = now_ms + 500U;
						touch_shutdown_state = TouchShutdownState::Held;
						send_touch_status(kMasterNodeId, kTouchStatusPressed);
						EXO_LOG("Touched\r\n");
					} else {
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
						send_touch_status(kMasterNodeId, kTouchStatusReleased);
						EXO_LOG("Released\r\n");
					} else if ((now_ms - touch_start_ms) >= 5000U) {
						BUZZER.SET_PERCENT(0);
						ERM_PWM.SET_PERCENT(0);
						RGB.SET(true, false, false);
						send_touch_status(kMasterNodeId, kTouchStatusShutdownArmed);
						EXO_LOG("Touch shutdown armed\r\n");
						EXO_LOG("Power off\r\n");
						poweroff_pcb_and_wait_for_release();
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
						send_touch_status(kMasterNodeId, kTouchStatusTurningOff);
						EXO_LOG("Touch shutdown release\r\n");
					}
					break;

				case TouchShutdownState::ShutdownDelay:
					RGB.SET(true, false, false);
					if ((now_ms - touch_release_ms) >= 1000U) {
						BUZZER.SET_PERCENT(0);
						ERM_PWM.SET_PERCENT(0);
						RGB.OFF();
						send_touch_status(kMasterNodeId, kTouchStatusTurningOff);
						EXO_LOG("Power off\r\n");
						poweroff_pcb_and_wait_for_release();
						touch_shutdown_state = TouchShutdownState::PoweredOff;
					}
					break;

				case TouchShutdownState::PoweredOff:
					break;
			}
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI1
			| RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_MSI;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2
			| RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS | RCC_PERIPHCLK_RFWAKEUP;
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
extern "C" void exo_hub_ble_evt_trace(uint8_t evt, const uint8_t *payload, uint8_t length)
		{
	static uint32_t s_last_trace_ms = 0U;
	const uint32_t now = HAL_GetTick();
	if (s_last_trace_ms != 0U && (now - s_last_trace_ms) < 100U) {
		return;
	}
	s_last_trace_ms = now;
#if EXO_BLE_LOG_LEVEL >= 2
	EXO_LOG("[BLE][DBG][EVT] id=%u len=%u", static_cast<unsigned>(evt), static_cast<unsigned>(length));
#if (EXO_BLE_LOG_LEVEL >= 3) && (EXO_BLE_HEX_DUMP_ENABLE != 0)
	if (payload != nullptr && length > 0U) {
		const uint8_t preview = (length > 16U) ? 16U : length;
		EXO_LOG(" data=");
		for (uint8_t i = 0U; i < preview; ++i) {
			EXO_LOG("%02X", static_cast<unsigned>(payload[i]));
			if (i + 1U < preview) {
				EXO_LOG(" ");
			}
		}
		if (length > preview) {
			EXO_LOG(" ...");
		}
	}
#endif
	EXO_LOG("\r\n");
#else
	(void)evt;
	(void)payload;
	(void)length;
#endif
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
		{
	(void) huart;
	(void) Size;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
		{
	(void) huart;
}

extern "C" void exo_hub_ble_notify_state_trace(uint8_t channel, uint8_t enabled)
		{
	const char *name = "UNKNOWN";
	switch (channel) {
		case CUSTOM_STM_CHANNEL_IMU:
			name = "IMU";
			break;
		case CUSTOM_STM_CHANNEL_CMD:
			name = "CMD";
			break;
		case CUSTOM_STM_CHANNEL_CMD_ACK:
			name = "CMD_ACK";
			break;
		case CUSTOM_STM_CHANNEL_RECORD:
			name = "RECORD";
			break;
		case CUSTOM_STM_CHANNEL_RECOVERY:
			name = "RECOVERY";
			g_master_ble_status_notify_enabled = (enabled != 0U);
			break;
		default:
			break;
	}
	EXO_LOG("[BLE][HUB][CCCD] %s notify=%s\r\n", name, enabled != 0U ? "ON" : "OFF");
}

extern "C" uint8_t exo_hub_leaf_stream_ingest(uint8_t node_id,
		uint8_t sensor_id,
		const uint8_t *payload,
		uint8_t payload_len)
		{
	const uint8_t ok = master_rs485_recording.push_leaf_sample(node_id,
			sensor_id,
			payload,
			payload_len) ? 1U : 0U;
	if (ok != 0U) {
		EXO_LOG("[BLE][HUB][LEAF] sample queued node=%u sensor=%u len=%u\r\n",
				static_cast<unsigned>(node_id),
				static_cast<unsigned>(sensor_id),
				static_cast<unsigned>(payload_len));
	}
	return ok;
}

extern "C" uint8_t exo_hub_leaf_record_done_ingest(const uint8_t *payload, uint16_t length)
		{
	if (payload == nullptr || length < sizeof(exo::RecordDoneMessage)) {
		return 0U;
	}
	exo::RecordDoneMessage message { };
	memcpy(&message, payload, sizeof(message));
	if (message.command == exo::RecordCommand::RecordDone &&
			message.node_id >= 1U && message.node_id <= 4U &&
			message.total_size >= sizeof(exo::SessionHeader)) {
		const uint8_t training_index = static_cast<uint8_t>(message.node_id - 1U);
		g_training_node_done[training_index] = message;
		g_training_node_done_valid[training_index] = true;
	}
	master_training_csv_coordinator.on_node_record_done(message);
	if (master_training_csv_coordinator.state() == exo::TrainingCsvState::ReceiveNode &&
			master_training_csv_coordinator.active_session_id() == message.session_id &&
			master_training_csv_coordinator.active_node_id() == message.node_id &&
			message.node_id >= 1U && message.node_id <= 4U) {
		g_training_node_done_valid[message.node_id - 1U] = false;
	}
	if (seen_node_done_contains(message)) {
		EXO_LOG("[BLE][HUB][LEAF] record_done duplicate ignored node=%u session=%lu size=%lu\r\n",
				static_cast<unsigned>(message.node_id),
				static_cast<unsigned long>(message.session_id),
				static_cast<unsigned long>(message.total_size));
		return 1U;
	}
	const uint8_t ok = master_rs485_recording.queue_record_done(message) ? 1U : 0U;
	if (ok != 0U) {
		seen_node_done_remember(message);
		g_ble_record_transfer_mode = true;
		g_ble_start_or_record_in_progress = true;
		EXO_LOG("[BLE][HUB][LEAF] record_done queued node=%u session=%lu size=%lu\r\n",
				static_cast<unsigned>(message.node_id),
				static_cast<unsigned long>(message.session_id),
				static_cast<unsigned long>(message.total_size));
		EXO_LOG("[BLE][REC][REL][NODEQ] scheduler armed from leaf record_done\r\n");
	}
	return ok;
}

extern "C" void exo_hub_leaf_record_frame_ingest(uint8_t node_id,
		const uint8_t *payload,
		uint16_t payload_len)
		{
	master_training_csv_coordinator.on_node_reliable_frame(node_id, payload, payload_len);
}

extern "C" void exo_hub_leaf_control_ingest(uint8_t node_id,
		uint8_t msg_type,
		const uint8_t *payload,
		uint16_t payload_len)
		{
	if (!g_record_sync.active || payload == nullptr || payload_len < 2U) {
		return;
	}
	const uint8_t command_id = payload[0];
	const uint8_t accepted = payload[1];
	if (command_id != static_cast<uint8_t>(exo::RecordCommand::PrepareRecord)) {
		return;
	}
	const uint8_t bit = record_sync_node_bit(node_id);
	if ((bit == 0U) || ((g_record_sync.target_mask & bit) == 0U)) {
		return;
	}
	if (msg_type == BLEPIPE_MSG_ACK && accepted != 0U) {
		g_record_sync.prepared_mask = static_cast<uint8_t>(g_record_sync.prepared_mask | bit);
		EXO_LOG("[BLE][HUB][SYNC] prepare ACK node=%u prepared=0x%02X target=0x%02X\r\n",
				static_cast<unsigned>(node_id),
				static_cast<unsigned>(g_record_sync.prepared_mask),
				static_cast<unsigned>(g_record_sync.target_mask));
	} else {
		g_record_sync.failed_mask = static_cast<uint8_t>(g_record_sync.failed_mask | bit);
		EXO_LOG("[BLE][HUB][SYNC] prepare NACK node=%u msg=0x%02X accepted=%u failed=0x%02X\r\n",
				static_cast<unsigned>(node_id),
				static_cast<unsigned>(msg_type),
				static_cast<unsigned>(accepted),
				static_cast<unsigned>(g_record_sync.failed_mask));
	}
}

extern "C" void exo_hub_leaf_topology_touch(uint8_t node_id)
		{
	static uint32_t last_topology_mask = 0xFFFFFFFFUL;
	master_rs485_recording.touch_node(node_id);
	uint8_t discovered[8] = { 0U };
	const uint8_t count = master_rs485_recording.copy_discovered_node_ids(
			discovered,
			static_cast<uint8_t>(sizeof(discovered)));
	uint32_t mask = 0UL;
	for (uint8_t i = 0U; i < count; ++i) {
		if (discovered[i] < 32U) {
			mask |= (1UL << discovered[i]);
		}
	}
	if (mask != last_topology_mask) {
		last_topology_mask = mask;
		EXO_LOG("[BLE][HUB][LEAF] topology count=%u mask=0x%08lX\r\n",
				static_cast<unsigned>(count),
				static_cast<unsigned long>(mask));
		master_blepipe_send_topology(nullptr);
	}
}

extern "C" void exo_ble_debug_printf(const char *format, ...)
		{
	va_list args;
	char message[224] = { 0 };
	if (format == nullptr) {
		return;
	}
	va_start(args, format);
	const int written = vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (written <= 0) {
		return;
	}
	debug.snprint("[%010lu ms] ", static_cast<unsigned long>(HAL_GetTick()));
	debug.snprint("%s", message);
}

extern "C" uint8_t exo_hub_ble_write(const uint8_t *payload, uint8_t length)
		{
	if (payload == nullptr || length == 0U) {
		EXO_LOG("[BLE][HUB][WRITE] empty payload len=%u\r\n", static_cast<unsigned>(length));
		return 0U;
	}
	blepipe_hdr_t pipe_hdr { };
	const uint8_t *pipe_payload = nullptr;
	uint16_t pipe_payload_len = 0U;
	const blepipe_status_t pipe_status = blepipe_decode(payload,
			length,
			&pipe_hdr,
			&pipe_payload,
			&pipe_payload_len);
	if (pipe_status == BLEPIPE_STATUS_OK &&
			blepipe_msg_allowed_on_lane(BLEPIPE_LANE_CONTROL_RX, pipe_hdr.msg_type) != 0U) {
		EXO_LOG("[BLEPIPE][HUB][WRITE] msg=0x%02X src=0x%04X dst=0x%04X len=%u\r\n",
				static_cast<unsigned>(pipe_hdr.msg_type),
				static_cast<unsigned>(pipe_hdr.src_id),
				static_cast<unsigned>(pipe_hdr.dst_id),
				static_cast<unsigned>(pipe_payload_len));
		switch (pipe_hdr.msg_type) {
			case BLEPIPE_MSG_COMMAND:
				case BLEPIPE_MSG_STREAM_CONTROL:
				case BLEPIPE_MSG_CONFIG_SET:
				if (master_handle_blepipe_command(pipe_hdr, pipe_payload, pipe_payload_len)) {
					return 1U;
				}
				if (pipe_payload != nullptr && pipe_payload_len > 0U &&
						(pipe_payload[0] == static_cast<uint8_t>(exo::RecordCommand::ReliableFrame) ||
								pipe_payload[0] == static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck) ||
								pipe_payload[0] == static_cast<uint8_t>(exo::RecordCommand::ChunkAck))) {
					payload = pipe_payload;
					length = static_cast<uint8_t>(pipe_payload_len);
					break;
				}
				return 0U;
			default:
				master_blepipe_send_ack(pipe_hdr, 0U, pipe_payload_len > 0U ? pipe_payload[0] : 0U);
				return 0U;
		}
	}
	EXO_LOG("[BLE][HUB][WRITE] cmd=0x%02X len=%u\r\n",
			static_cast<unsigned>(payload[0]),
			static_cast<unsigned>(length));
	if (payload[0] != static_cast<uint8_t>(exo::RecordCommand::ChunkAck)) {
		EXO_LOG("[BLE][HUB][WRITE] data=");
		for (uint8_t i = 0U; i < length; ++i) {
			EXO_LOG("%02X", static_cast<unsigned>(payload[i]));
			if ((i + 1U) < length) {
				EXO_LOG(" ");
			}
		}
		EXO_LOG("\r\n");
	}
	switch (payload[0]) {
		case static_cast<uint8_t>(exo::RecordCommand::StartRecord):
			if (length >= sizeof(exo::StartRecordMessage)) {
				exo::StartRecordMessage message { };
				memcpy(&message, payload, sizeof(message));
				const uint8_t sync_result = record_sync_begin(message,
						RecordSyncPhoneAckMode::Raw,
						nullptr,
						payload,
						static_cast<uint8_t>(sizeof(exo::StartRecordMessage)));
				if (sync_result == 0U) {
					HUB_START_LOG1("reject busy session=%lu", static_cast<unsigned long>(message.session_id));
					(void) Custom_APP_SendCmdAck(payload, length, 0U);
					return 0U;
				}
				if (sync_result == 2U) {
					HUB_START_LOG1("dup session=%lu", static_cast<unsigned long>(message.session_id));
					const tBleStatus dup_ack_status = Custom_APP_SendCmdAck(payload, length, 1U);
					HUB_START_LOG1("dup ack=0x%02X", static_cast<unsigned>(dup_ack_status));
					return 1U;
				}
				return sync_result == 1U ? 1U : 0U;
			}
			HUB_START_LOG2("bad len=%u min=%u",
					static_cast<unsigned>(length),
					static_cast<unsigned>(sizeof(exo::StartRecordMessage)));
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			return 0U;
		case 0xA0U: /* start stream */
			g_ble_stream_enabled = true;
			(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
			BLEPIPE_ID_HUB,
					payload,
					length);
			EXO_LOG("[BLE][HUB][CTRL] stream=ON\r\n");
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA1U: /* stop stream */
			g_ble_stream_enabled = false;
			(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
			BLEPIPE_ID_HUB,
					payload,
					length);
			EXO_LOG("[BLE][HUB][CTRL] stream=OFF\r\n");
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA2U: /* set stream interval ms */
			if (length < 2U) {
				EXO_LOG("[BLE][HUB][CTRL] stream interval missing arg len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			{
				uint32_t interval = payload[1];
				if (interval < kMinStreamIntervalMs) {
					interval = kMinStreamIntervalMs;
				} else if (interval > kMaxStreamIntervalMs) {
					interval = kMaxStreamIntervalMs;
				}
				g_ble_stream_interval_ms = interval;
				(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_STREAM_CONTROL,
				BLEPIPE_ID_HUB,
						payload,
						length);
				EXO_LOG("[BLE][HUB][CTRL] stream interval=%lu ms\r\n", static_cast<unsigned long>(interval));
			}
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA3U: /* set ERM percent 0..100 */
			if (!apply_ble_actuator_command(payload, length)) {
				EXO_LOG("[BLE][HUB][CTRL] ERM missing arg len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA4U: /* set passive buzzer percent 0..99 */
			if (!apply_ble_actuator_command(payload, length)) {
				EXO_LOG("[BLE][HUB][CTRL] buzzer missing arg len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA5U: /* set RGB mask bit0=R,bit1=G,bit2=B; bit7 clears BLE override */
			if (!apply_ble_actuator_command(payload, length)) {
				EXO_LOG("[BLE][HUB][CTRL] RGB missing arg len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xA6U: /* test touch feedback / long-press power off */
			if (!apply_ble_actuator_command(payload, length)) {
				EXO_LOG("[BLE][HUB][CTRL] touch test bad arg len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xB0U: /* provision node ID: payload[1]=current_id payload[2]=new_id */
			if (length < 3U) {
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			{
				const uint8_t current_id = payload[1];
				const uint8_t new_id = payload[2];
				if (exo_hub_central_client_send_blepipe_to_node(current_id,
						BLEPIPE_MSG_CONFIG_SET,
						BLEPIPE_ID_HUB,
						payload,
						length) != 0U) {
					(void) Custom_APP_SendCmdAck(payload, length, 1U);
					return 1U;
				}
				const uint8_t ok = master_rs485_recording.provision_node_id(current_id, new_id) ? 1U : 0U;
				uint8_t report[3] = { current_id, new_id, ok };
				(void) Custom_APP_SendCmdReport(0xC0U, report, 3U);
				(void) Custom_APP_SendCmdAck(payload, length, ok);
				return ok;
			}
		case 0xB1U: /* query node ID: payload[1]=node_id */
			if (length < 2U) {
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			{
				const uint8_t target_id = payload[1];
				if (exo_hub_central_client_send_blepipe_to_node(target_id,
						BLEPIPE_MSG_COMMAND,
						BLEPIPE_ID_HUB,
						payload,
						length) != 0U) {
					(void) Custom_APP_SendCmdAck(payload, length, 1U);
					return 1U;
				}
				uint8_t resolved_id = 0U;
				const uint8_t ok = master_rs485_recording.request_node_id(target_id, resolved_id) ? 1U : 0U;
				uint8_t report[3] = { target_id, resolved_id, ok };
				(void) Custom_APP_SendCmdReport(0xC1U, report, 3U);
				(void) Custom_APP_SendCmdAck(payload, length, ok);
				return ok;
			}
		case 0xB2U: /* force rediscovery of all nodes */
			exo_hub_central_client_request_scan();
			master_rs485_recording.rediscover_nodes();
			ble_send_discovered_nodes_report();
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case 0xB3U: /* reset all recording/transfer state to Idle and erase session data */
		{
			const uint8_t target_mask = exo_hub_central_client_ready_node_mask();
			(void) exo_hub_central_client_broadcast_blepipe(BLEPIPE_MSG_COMMAND,
			BLEPIPE_ID_HUB,
					payload,
					length);
			mark_remote_reset_cooldown_if_needed(target_mask);
			const uint8_t ok = master_record_reset_all(true);
			ble_send_discovered_nodes_report();
			(void) Custom_APP_SendCmdAck(payload, length, ok);
			return ok;
		}
		case 0xB4U: /* get discovered nodes report */
			ble_send_discovered_nodes_report();
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case kRecordTransferTuningCmd:
			if (apply_record_transfer_tuning_payload(payload, length)) {
				(void) Custom_APP_SendCmdAck(payload, length, 1U);
				return 1U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			return 0U;
		case static_cast<uint8_t>(exo::RecordCommand::ReliableFrame):
			{
			if (length < sizeof(exo::RecordReliableFrameHeader)) {
				EXO_LOG("[BLE][REC][REL] short control len=%u\r\n", static_cast<unsigned>(length));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			exo::RecordReliableFrameHeader hdr { };
			memcpy(&hdr, payload, sizeof(hdr));
			if (hdr.proto_version != exo::kRecordReliableProtoVersion ||
					hdr.magic != exo::kRecordReliableMagic ||
					(sizeof(hdr) + hdr.payload_len) > length) {
				EXO_LOG("[BLE][REC][REL] bad header proto=%u magic=0x%04X len=%u payload=%u\r\n",
						static_cast<unsigned>(hdr.proto_version),
						static_cast<unsigned>(hdr.magic),
						static_cast<unsigned>(length),
						static_cast<unsigned>(hdr.payload_len));
				(void) Custom_APP_SendCmdAck(payload, length, 0U);
				return 0U;
			}
			const uint8_t *body = payload + sizeof(hdr);
			const exo::RecordReliableType type = static_cast<exo::RecordReliableType>(hdr.frame_type);
			switch (type) {
				case exo::RecordReliableType::ManifestAck:
					if (hdr.payload_len >= sizeof(exo::RecordReliableManifestAckPayload)) {
						exo::RecordReliableManifestAckPayload ack { };
						memcpy(&ack, body, sizeof(ack));
						ack.source_id = resolve_remote_source_id(ack.source_id, ack.session_id);
						EXO_LOG("[BLE][REC][REL] MANIFEST_ACK source=%u session=%lu credit=%u status=%u\r\n",
								static_cast<unsigned>(ack.source_id),
								static_cast<unsigned long>(ack.session_id),
								static_cast<unsigned>(ack.credit),
								static_cast<unsigned>(ack.status));
						if (ack.source_id == kMasterNodeId && ack.session_id == g_local_done.session_id) {
							g_local_manifest_acked = true;
							g_local_receiver_credit = sanitize_receiver_credit(ack.credit);
							g_local_requested_chunk = 0U;
							g_local_stream_cursor_chunk = 0U;
							g_local_retx_cursor_chunk = 0U;
							g_local_retx_remaining = 0U;
							g_local_record_phase = LocalRecordPhase::TransferActive;
						} else {
							if (g_have_pending_node_done &&
									g_pending_node_done.node_id == ack.source_id &&
									g_pending_node_done.session_id == ack.session_id) {
								g_remote_transfer_active = true;
								g_remote_transfer_source_id = ack.source_id;
								g_remote_transfer_session_id = ack.session_id;
								g_have_pending_node_done = false;
								g_pending_node_manifest_last_tick = 0U;
								EXO_LOG("[BLE][REC][REL][NODEQ] node manifest ack source=%u session=%lu credit=%u queue=%u\r\n",
										static_cast<unsigned>(ack.source_id),
										static_cast<unsigned long>(ack.session_id),
										static_cast<unsigned>(ack.credit),
										static_cast<unsigned>(pending_node_done_depth()));
							}
							master_rs485_recording.on_ble_reliable_ack_window(ack.session_id,
									ack.source_id,
									0U,
									sanitize_receiver_credit(ack.credit));
							(void) forward_remote_record_control(ack.source_id, payload, length);
						}
						(void) Custom_APP_SendCmdAck(payload, length, 1U);
						return 1U;
					}
					break;
				case exo::RecordReliableType::AckWindow:
					if (hdr.payload_len >= sizeof(exo::RecordReliableAckWindowPayload)) {
						exo::RecordReliableAckWindowPayload ack { };
						memcpy(&ack, body, sizeof(ack));
						ack.source_id = resolve_remote_source_id(ack.source_id, ack.session_id);
						EXO_LOG("[BLE][REC][REL] ACK_WINDOW source=%u session=%lu chunk=%lu credit=%u\r\n",
								static_cast<unsigned>(ack.source_id),
								static_cast<unsigned long>(ack.session_id),
								static_cast<unsigned long>(ack.next_chunk_index),
								static_cast<unsigned>(ack.credit));
						if (ack.source_id == kMasterNodeId && ack.session_id == g_local_done.session_id) {
							const uint8_t rx_credit = sanitize_receiver_credit(ack.credit);
							if (g_local_record_phase == LocalRecordPhase::Resync || g_local_retx_remaining > 0U) {
								EXO_LOG("[BLE][REC][REL] ACK_WINDOW ignored during recovery source=%u session=%lu chunk=%lu retx_rem=%u cur=%lu\r\n",
										static_cast<unsigned>(ack.source_id),
										static_cast<unsigned long>(ack.session_id),
										static_cast<unsigned long>(ack.next_chunk_index),
										static_cast<unsigned>(g_local_retx_remaining),
										static_cast<unsigned long>(g_local_stream_cursor_chunk));
								(void) Custom_APP_SendCmdAck(payload, length, 1U);
								return 1U;
							}
							g_local_receiver_credit = rx_credit;
							if (ack.next_chunk_index < g_local_stream_cursor_chunk) {
								g_local_stale_ack_repeat = static_cast<uint8_t>(g_local_stale_ack_repeat < 255U ? (g_local_stale_ack_repeat + 1U) : 255U);
								const uint32_t now = HAL_GetTick();
								if ((g_local_record_phase == LocalRecordPhase::Resync || g_local_retx_remaining > 0U) &&
										g_local_stale_ack_repeat >= 6U &&
										(now - g_local_forward_progress_tick_ms) >= 200U) {
									g_local_retx_cursor_chunk = ack.next_chunk_index;
									g_local_retx_remaining = static_cast<uint8_t>(rx_credit > 0U ? (rx_credit > 4U ? 4U : rx_credit) : 1U);
									g_local_record_phase = LocalRecordPhase::Resync;
									EXO_LOG("[BLE][REC][REL] ACK_WINDOW stale escape source=%u session=%lu chunk=%lu retx=%u cur=%lu\r\n",
											static_cast<unsigned>(ack.source_id),
											static_cast<unsigned long>(ack.session_id),
											static_cast<unsigned long>(ack.next_chunk_index),
											static_cast<unsigned>(g_local_retx_remaining),
											static_cast<unsigned long>(g_local_stream_cursor_chunk));
								}
								EXO_LOG("[BLE][REC][REL] ACK_WINDOW stale ignored source=%u session=%lu chunk=%lu cur=%lu\r\n",
										static_cast<unsigned>(ack.source_id),
										static_cast<unsigned long>(ack.session_id),
										static_cast<unsigned long>(ack.next_chunk_index),
										static_cast<unsigned long>(g_local_stream_cursor_chunk));
								(void) Custom_APP_SendCmdAck(payload, length, 1U);
								return 1U;
							}
							g_local_stale_ack_repeat = 0U;
							if (ack.next_chunk_index > g_local_stream_cursor_chunk) {
								g_local_stream_cursor_chunk = ack.next_chunk_index;
							}
							g_local_requested_chunk = g_local_stream_cursor_chunk;
							g_local_chunk_offset = g_local_stream_cursor_chunk * static_cast<uint32_t>(kRecordChunkPayloadBytes);
							g_local_record_phase = LocalRecordPhase::TransferActive;
							g_local_waiting_verify = false;
						} else {
							master_rs485_recording.on_ble_reliable_ack_window(ack.session_id,
									ack.source_id,
									ack.next_chunk_index,
									sanitize_receiver_credit(ack.credit));
							(void) forward_remote_record_control(ack.source_id, payload, length);
						}
						(void) Custom_APP_SendCmdAck(payload, length, 1U);
						return 1U;
					}
					break;
				case exo::RecordReliableType::NackRange:
					if (hdr.payload_len >= 12U) {
						exo::RecordReliableNackRangePayload nack { };
						/* Accept both compact (12-byte) and full (14-byte) NACK payloads. */
						nack.source_id = static_cast<uint16_t>(body[0] | (static_cast<uint16_t>(body[1]) << 8U));
						nack.session_id = static_cast<uint32_t>(body[2]) |
								(static_cast<uint32_t>(body[3]) << 8U) |
								(static_cast<uint32_t>(body[4]) << 16U) |
								(static_cast<uint32_t>(body[5]) << 24U);
						nack.first_chunk_index = static_cast<uint32_t>(body[6]) |
								(static_cast<uint32_t>(body[7]) << 8U) |
								(static_cast<uint32_t>(body[8]) << 16U) |
								(static_cast<uint32_t>(body[9]) << 24U);
						nack.chunk_count = static_cast<uint16_t>(body[10] | (static_cast<uint16_t>(body[11]) << 8U));
						nack.flags = 0U;
						if (hdr.payload_len >= sizeof(exo::RecordReliableNackRangePayload)) {
							nack.flags = static_cast<uint16_t>(body[12] | (static_cast<uint16_t>(body[13]) << 8U));
						}
						nack.source_id = resolve_remote_source_id(nack.source_id, nack.session_id);
						EXO_LOG("[BLE][REC][REL] NACK_RANGE source=%u session=%lu first=%lu count=%u\r\n",
								static_cast<unsigned>(nack.source_id),
								static_cast<unsigned long>(nack.session_id),
								static_cast<unsigned long>(nack.first_chunk_index),
								static_cast<unsigned>(nack.chunk_count));
						if (nack.source_id == kMasterNodeId && nack.session_id == g_local_done.session_id) {
							const uint8_t nack_count = nack.chunk_count == 0U ? 1U : static_cast<uint8_t>(nack.chunk_count > 16U ? 16U : nack.chunk_count);
							bool deferred_active_retx = false;
							if (g_local_retx_remaining > 0U) {
								const uint32_t active_start = g_local_retx_cursor_chunk;
								const uint32_t active_end_exclusive = g_local_retx_cursor_chunk + static_cast<uint32_t>(g_local_retx_remaining);
								const uint32_t req_start = nack.first_chunk_index;
								const uint32_t req_end_exclusive = nack.first_chunk_index + static_cast<uint32_t>(nack_count);
								const bool overlaps_active = (active_start < req_end_exclusive) && (req_start < active_end_exclusive);
								if (overlaps_active) {
									deferred_active_retx = true;
									EXO_LOG("[BLE][REC][REL] NACK_RANGE deferred active_retx session=%lu first=%lu count=%u active=%lu rem=%u trigger=queue_after_active\r\n",
											static_cast<unsigned long>(nack.session_id),
											static_cast<unsigned long>(nack.first_chunk_index),
											static_cast<unsigned>(nack_count),
											static_cast<unsigned long>(g_local_retx_cursor_chunk),
											static_cast<unsigned>(g_local_retx_remaining));
								}
							}
							(void) recovery_queue_push(RecoveryJobKind::MasterLocalRetx,
									nack.source_id,
									nack.session_id,
									nack.first_chunk_index,
									nack_count);
							if (deferred_active_retx) {
								EXO_LOG("[BLE][REC][REL] NACK_RANGE queued deferred session=%lu first=%lu count=%u\r\n",
										static_cast<unsigned long>(nack.session_id),
										static_cast<unsigned long>(nack.first_chunk_index),
										static_cast<unsigned>(nack_count));
							} else {
								EXO_LOG("[BLE][REC][REL] NACK_RANGE accepted queued session=%lu first=%lu count=%u\r\n",
										static_cast<unsigned long>(nack.session_id),
										static_cast<unsigned long>(nack.first_chunk_index),
										static_cast<unsigned>(nack_count));
							}
						} else {
							(void) recovery_queue_push(RecoveryJobKind::NodeUartRetx,
									nack.source_id,
									nack.session_id,
									nack.first_chunk_index,
									nack.chunk_count == 0U ? 1U : static_cast<uint8_t>(nack.chunk_count > 16U ? 16U : nack.chunk_count));
							(void) forward_remote_record_control(nack.source_id, payload, length);
						}
						(void) Custom_APP_SendCmdAck(payload, length, 1U);
						return 1U;
					}
					break;
				case exo::RecordReliableType::Pause:
					case exo::RecordReliableType::Resume:
					case exo::RecordReliableType::Cancel:
					if (hdr.source_id == kMasterNodeId && hdr.session_id == g_local_done.session_id) {
						if (type == exo::RecordReliableType::Cancel) {
							g_local_record_phase = LocalRecordPhase::Cancelled;
							g_local_receiver_credit = 0U;
						} else if (type == exo::RecordReliableType::Pause) {
							local_record_pause_for_resume();
						} else {
							g_local_record_phase = LocalRecordPhase::TransferActive;
						}
					} else {
						master_rs485_recording.on_ble_reliable_pause(hdr.session_id, hdr.source_id);
						(void) forward_remote_record_control(hdr.source_id, payload, length);
					}
					(void) Custom_APP_SendCmdAck(payload, length, 1U);
					return 1U;
				case exo::RecordReliableType::VerifyOk:
					if (hdr.payload_len >= sizeof(exo::RecordReliableVerifyPayload)) {
						exo::RecordReliableVerifyPayload verify { };
						memcpy(&verify, body, sizeof(verify));
						verify.source_id = resolve_remote_source_id(verify.source_id, verify.session_id);
						(void) Custom_APP_SendCmdAck(payload, length, 1U);
						if (verify.source_id == kMasterNodeId && verify.session_id == g_local_done.session_id &&
								verify.file_crc32 == g_local_done.payload_crc32) {
							g_local_record_phase = LocalRecordPhase::Finished;
							g_local_waiting_verify = false;
							(void) send_reliable_record_frame(exo::RecordReliableType::CommitDone,
									kMasterNodeId,
									g_local_done.session_id,
									0U,
									0U,
									reinterpret_cast<const uint8_t*>(&verify),
									static_cast<uint16_t>(sizeof(verify)));
							master_rs485_recording.set_transfer_hold(false);
							EXO_LOG("[MASTER][REC][REL] VERIFY_OK session=%lu crc=0x%08lX\r\n",
									static_cast<unsigned long>(verify.session_id),
									static_cast<unsigned long>(verify.file_crc32));
							EXO_LOG("[BLE][REC][REL][NODEQ] master complete queue=%u\r\n",
									static_cast<unsigned>(pending_node_done_depth()));
							start_next_pending_node_manifest_now();
						} else {
							if (master_training_csv_should_hold_node_verify(verify)) {
								training_pending_verify_store(verify, payload, length);
								return 1U;
							}
							release_remote_node_verify_ok(verify, payload, length);
						}
						return 1U;
					}
					break;
				case exo::RecordReliableType::VerifyFail:
					if (hdr.source_id == kMasterNodeId && hdr.session_id == g_local_done.session_id) {
						(void) recovery_queue_push(RecoveryJobKind::VerifyRetx,
								hdr.source_id,
								hdr.session_id,
								hdr.chunk_index,
								1U);
					} else {
						(void) recovery_queue_push(RecoveryJobKind::NodeUartRetx,
								hdr.source_id,
								hdr.session_id,
								hdr.chunk_index,
								1U);
						(void) forward_remote_record_control(hdr.source_id, payload, length);
					}
					(void) Custom_APP_SendCmdAck(payload, length, 1U);
					return 1U;
				default:
					break;
			}
			EXO_LOG("[BLE][REC][REL] unsupported control type=%u len=%u\r\n",
					static_cast<unsigned>(hdr.frame_type),
					static_cast<unsigned>(length));
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			return 0U;
		}
		case static_cast<uint8_t>(exo::RecordCommand::SessionChunk):
			case static_cast<uint8_t>(exo::RecordCommand::RecordDone):
			EXO_LOG("[BLE][HUB][WRITE] passthrough cmd=0x%02X\r\n", static_cast<unsigned>(payload[0]));
			(void) Custom_APP_SendCmdAck(payload, length, 1U);
			return 1U;
		case static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck):
			{
			if (length == sizeof(exo::SessionCompleteAckMessage)) {
				exo::SessionCompleteAckMessage ack { };
				memcpy(&ack, payload, sizeof(ack));
				uint16_t source_id = resolve_remote_source_id(0U, ack.session_id);
				if (source_id == 0U && ack.session_id == g_local_done.session_id) {
					source_id = kMasterNodeId;
				}
				if (source_id == kMasterNodeId && ack.session_id == g_local_done.session_id &&
						ack.payload_crc32 == g_local_done.payload_crc32) {
					g_local_record_phase = LocalRecordPhase::Finished;
					g_local_receiver_credit = 0U;
					g_local_waiting_verify = false;
					master_rs485_recording.set_transfer_hold(false);
					EXO_LOG("[BLE][REC][REL][NODEQ] master complete queue=%u\r\n",
							static_cast<unsigned>(pending_node_done_depth()));
					start_next_pending_node_manifest_now();
				} else if (source_id != 0U) {
					if (g_remote_transfer_active &&
							g_remote_transfer_source_id == source_id &&
							g_remote_transfer_session_id == ack.session_id) {
						g_remote_transfer_active = false;
						g_remote_transfer_source_id = 0U;
						g_remote_transfer_session_id = 0U;
						EXO_LOG("[BLE][REC][REL][NODEQ] node transfer complete source=%u session=%lu queue=%u\r\n",
								static_cast<unsigned>(source_id),
								static_cast<unsigned long>(ack.session_id),
								static_cast<unsigned>(pending_node_done_depth()));
					}
					(void) forward_remote_record_control(source_id, payload, length);
				}
				(void) Custom_APP_SendCmdAck(payload, length, 1U);
				return 1U;
			}
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			return 0U;
		}
		case static_cast<uint8_t>(exo::RecordCommand::ChunkAck):
			{
			if (length >= sizeof(exo::ChunkAckMessage)) {
				uint32_t session_id = 0U;
				uint16_t source_id = 0U;
				uint32_t next_offset = 0U;
				if (length >= sizeof(exo::ChunkAckCompactSourceMessage) &&
						payload[1] == 4U) {
					exo::ChunkAckCompactSourceMessage ack { };
					memcpy(&ack, payload, sizeof(ack));
					session_id = ack.session_id;
					source_id = ack.source_id;
					next_offset = ack.next_offset;
				} else if (length >= sizeof(exo::ChunkAckCompactMessage) &&
						payload[1] == 4U) {
					exo::ChunkAckCompactMessage ack { };
					memcpy(&ack, payload, sizeof(ack));
					session_id = ack.session_id;
					next_offset = ack.next_offset;
				} else if (length >= sizeof(exo::ChunkAckV3Message)) {
					exo::ChunkAckV3Message ack { };
					memcpy(&ack, payload, sizeof(ack));
					session_id = ack.session_id;
					source_id = ack.source_id;
					next_offset = ack.next_offset;
				} else if (length >= sizeof(exo::ChunkAckRangeMessage)) {
					exo::ChunkAckRangeMessage ack { };
					memcpy(&ack, payload, sizeof(ack));
					session_id = ack.session_id;
					next_offset = ack.next_offset;
				} else {
					exo::ChunkAckMessage ack { };
					memcpy(&ack, payload, sizeof(ack));
					session_id = ack.session_id;
					next_offset = ack.next_offset;
				}
				source_id = resolve_remote_source_id(source_id, session_id);
				if (source_id == 0U && session_id == g_local_done.session_id) {
					source_id = kMasterNodeId;
				}
				if (is_duplicate_chunk_ack(session_id, source_id, next_offset)) {
					(void) Custom_APP_SendCmdAck(payload, length, 1U);
					return 1U;
				}
				master_rs485_recording.on_ble_chunk_ack(session_id, source_id, next_offset);
				if (source_id != 0U && source_id != kMasterNodeId) {
					(void) forward_remote_record_control(source_id, payload, length);
				}
				(void) Custom_APP_SendCmdAck(payload, length, 1U);
				return 1U;
			}
			EXO_LOG("[BLE][HUB][WRITE] chunk_ack short len=%u\r\n", static_cast<unsigned>(length));
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			return 0U;
		}
		default:
			EXO_LOG("[BLE][HUB][WRITE] unsupported cmd=0x%02X len=%u\r\n",
					static_cast<unsigned>(payload[0]),
					static_cast<unsigned>(length));
			(void) Custom_APP_SendCmdAck(payload, length, 0U);
			break;
	}
	return 0U;
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
