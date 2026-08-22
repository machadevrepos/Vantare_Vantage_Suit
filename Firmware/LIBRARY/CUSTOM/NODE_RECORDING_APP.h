#ifndef NODE_RECORDING_APP_H_
#define NODE_RECORDING_APP_H_

#include "BLE_RECORD_PROTOCOL.h"
#include "BNO85_STM32.h"
#include "EXO_LOGGER.h"
#include "ICM45686_STM32.h"
#include "NODE_LIVE_SAMPLE_QUEUE.h"
#include "NODE_RECORDER.h"
#include "SESSION_TRANSFER.h"
#include "W25Q256_FLASH.h"

namespace exo {

	struct NodeRecordingConfig {
			uint16_t node_id;
			uint8_t bno85_address;
			uint8_t icm45686_address;
			uint32_t flash_base_address;
			uint32_t flash_capacity;
			uint32_t bno85_region_size;
	};

	class NodeRecordingApp {
		public:
			static constexpr uint32_t kDefaultDurationMs = 10000U;
			static constexpr uint32_t kDefaultLeadTimeUs = 300000U;
			static constexpr uint32_t kDataRateLogPeriodMs = 10000U;
			static constexpr uint32_t kIcmPeriodUs = 5000U;
			/* Recorded fusion target. The BNO wrapper queues one composite sample
			 * per 100 Hz game-rotation-vector report; Node ICM captures via the
			 * documented 200 Hz hardware FIFO (register-poll fallback if unavailable). */
			static constexpr uint16_t kBnoTargetRateHz = 100U;
			static constexpr uint16_t kIcmTargetRateHz = 200U;
			static constexpr uint8_t kMaxBnoDrainPerTick = 8U;
			static constexpr uint8_t kBnoLiveSensorId = 1U;
			static constexpr uint8_t kIcmLiveSensorId = 2U;
			static constexpr uint8_t kMaxLivePayload =
					sizeof(Bno85Sample) > sizeof(Icm45686Sample) ?
					sizeof(Bno85Sample) : sizeof(Icm45686Sample);
			using LiveSample = NodeLiveSample<kMaxLivePayload>;

			NodeRecordingApp(const NodeRecordingConfig &config, I2C_HandleTypeDef &icm_bus, I2C_HandleTypeDef &bno_bus,
					SPI_HandleTypeDef &flash_bus, GPIO_TypeDef *flash_cs_port, uint16_t flash_cs_pin)
			:
					config_(config),
							bno85_(bno_bus, config.bno85_address),
							icm45686_(icm_bus, config.icm45686_address),
							flash_(flash_bus, flash_cs_port, flash_cs_pin),
							recorder_(flash_, config.flash_base_address, config.flash_capacity, config.bno85_region_size) {
			}

			bool begin() {
#if defined(BNO_INT_GPIO_Port) && defined(BNO_INT_Pin)
				bno85_.set_interrupt_pin(BNO_INT_GPIO_Port, BNO_INT_Pin);
#endif
				if (!flash_.begin()) {
					ready_ = false;
					return false;
				}
				detected_flash_capacity_ = flash_.detected_capacity_bytes();
				if (detected_flash_capacity_ < kMinimumSupportedFlashSize) {
					ready_ = false;
					return false;
				}
				const uint32_t detected_usable = detected_flash_capacity_ - kReservedFlashBytes;
				usable_flash_capacity_ = (config_.flash_capacity == 0U ||
						config_.flash_capacity > detected_usable) ? detected_usable : config_.flash_capacity;
				if (!flash_layout_valid()) {
					ready_ = false;
					return false;
				}
				/* Session headers live in the reserved recovery sector (the
				 * settings sector occupies the top of flash). */
				recorder_.set_header_address(detected_flash_capacity_ - kReservedFlashBytes);
				if (recorder_.recover_after_boot()) {
					EXO_LOG("[RECORD][NODE%u] salvaged finalized session=%lu size=%lu after reboot; ReadyForUpload\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(recorder_.header().session_id),
							static_cast<unsigned long>(recorder_.total_size()));
				}
				ready_ = bno85_.begin() && icm45686_.begin();
				return ready_;
			}

			void set_node_id(uint16_t node_id) {
				config_.node_id = node_id;
			}

			bool start_recording(const StartRecordMessage &message) {
				if (message.command != RecordCommand::StartRecord || !can_start_recording()) {
					return false;
				}
				StartRecordMessage effective = message;
				normalize_start_message(effective);
				return arm_recording(effective);
			}

			bool prepare_recording(const StartRecordMessage &message) {
				if (message.command != RecordCommand::PrepareRecord || !can_start_recording()) {
					return false;
				}
				const uint32_t prepare_start_ms = HAL_GetTick();
				prepared_start_ = message;
				prepared_start_.command = RecordCommand::StartRecord;
				normalize_start_message(prepared_start_);
				if (!configure_session_layout(prepared_start_.requested_duration_ms)) {
					memset(&prepared_start_, 0, sizeof(prepared_start_));
					prepared_ = false;
					return false;
				}
				reset_record_buffers();
				bno_append_fail_count_ = 0U;
				icm_append_fail_count_ = 0U;
				/* RAM-arm the session, then cover the region with the background
				 * eraser: a whole-region erase takes seconds-to-minutes and must
				 * never block the BLE command handler. */
				if (!recorder_.prepare(config_.node_id,
						prepared_start_.session_id,
						prepared_start_.start_timestamp_us,
						prepared_start_.requested_duration_ms)) {
					memset(&prepared_start_, 0, sizeof(prepared_start_));
					prepared_ = false;
					EXO_LOG("[RECORD][NODE%u] PREPARE failed session=%lu elapsed_ms=%lu state=%u\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(message.session_id),
							static_cast<unsigned long>(HAL_GetTick() - prepare_start_ms),
							static_cast<unsigned>(recorder_.state()));
					return false;
				}
				const NodeRecorder::EraseStep prepare_erase =
						recorder_.begin_region_erase();
				if (prepare_erase == NodeRecorder::EraseStep::Failed) {
					memset(&prepared_start_, 0, sizeof(prepared_start_));
					prepared_ = false;
					EXO_LOG("[RECORD][NODE%u] PREPARE erase failed session=%lu\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(message.session_id));
					return false;
				}
				prepare_erase_pending_ =
						prepare_erase == NodeRecorder::EraseStep::InProgress;
				if (!prepare_erase_pending_ && !recorder_.finish_prepare_header()) {
					memset(&prepared_start_, 0, sizeof(prepared_start_));
					prepared_ = false;
					return false;
				}
				prepared_ = true;
				EXO_LOG("NODE%u record prepared: session=%lu lead_ms=%lu duration_ms=%lu state=%u\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(prepared_start_.session_id),
						static_cast<unsigned long>(prepared_start_.start_timestamp_us / 1000ULL),
						static_cast<unsigned long>(prepared_start_.requested_duration_ms),
						static_cast<unsigned>(recorder_.state()));
				EXO_LOG("[RECORD][NODE%u] PREPARE session=%lu duration_ms=%lu elapsed_ms=%lu state=%u\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(prepared_start_.session_id),
						static_cast<unsigned long>(prepared_start_.requested_duration_ms),
						static_cast<unsigned long>(HAL_GetTick() - prepare_start_ms),
						static_cast<unsigned>(recorder_.state()));
				return true;
			}

			bool commit_prepared_recording(const StartRecordMessage &message) {
				if (message.command != RecordCommand::CommitPreparedRecord || !prepared_) {
					return false;
				}
				if ((message.session_id != 0U) && (message.session_id != prepared_start_.session_id)) {
					return false;
				}
				if (prepare_erase_pending_) {
					/* Commit arrived while the prepare erase is still covering
					 * the region (first boot or a retained previous session:
					 * long layouts erase for ~a minute). Accept it; capture
					 * starts the moment the header is armed. */
					pending_commit_ = message;
					commit_pending_ = true;
					return true;
				}
				StartRecordMessage effective = prepared_start_;
				if (message.start_timestamp_us != 0ULL) {
					effective.start_timestamp_us = message.start_timestamp_us;
				}
				if (message.requested_duration_ms != 0U) {
					effective.requested_duration_ms = message.requested_duration_ms;
				}
				normalize_start_message(effective);
				const bool started = start_recording_now(effective);
				if (started) {
					prepared_ = false;
				}
				return started;
			}

			void set_live_stream_enabled(bool enabled) {
				live_stream_enabled_ = enabled;
				live_queue_.configure(enabled, live_interval_ms_);
			}

			void set_live_interval_ms(uint32_t interval_ms) {
				live_interval_ms_ = interval_ms == 0U ? 1U : interval_ms;
				live_queue_.configure(live_stream_enabled_, live_interval_ms_);
			}

			bool pop_live_sample(LiveSample &sample) {
				return live_queue_.pop(sample);
			}

			bool peek_live_sample(LiveSample &sample) const {
				return live_queue_.peek(sample);
			}

			bool discard_live_sample() {
				return live_queue_.discard_front();
			}

			uint32_t live_drop_count() const {
				return live_queue_.dropped();
			}

			bool stop_recording(const StopRecordMessage &message) {
				if (message.command != RecordCommand::StopRecord) {
					return false;
				}
				if (prepared_ || armed_) {
					const uint32_t pending_session = prepared_ ?
							prepared_start_.session_id : pending_start_.session_id;
					if (message.session_id != pending_session) {
						return false;
					}
					abort_prepared_recording();
					set_live_stream_enabled(false);
					return true;
				}
				if (recorder_.state() == RecorderState::ReadyForUpload) {
					return recorder_.header().session_id == message.session_id;
				}
				if (recorder_.state() != RecorderState::Recording ||
						pending_start_.session_id != message.session_id) {
					return false;
				}
				finalize_duration_ms_ = HAL_GetTick() - recording_started_ms_;
				bno85_.set_capture_queue_enabled(false);
				record_finalize_pending_ = true;
				set_live_stream_enabled(false);
				return true;
			}

			void abort_prepared_recording() {
				prepared_ = false;
				prepare_erase_pending_ = false;
				commit_pending_ = false;
				start_erase_pending_ = false;
				memset(&prepared_start_, 0, sizeof(prepared_start_));
				recorder_.cancel_prepared();
				if (armed_ && recorder_.state() != RecorderState::Recording) {
					armed_ = false;
					armed_tick_ms_ = 0U;
					memset(&pending_start_, 0, sizeof(pending_start_));
					reset_record_buffers();
				}
			}

			void process() {
				if (!ready_) {
					return;
				}
				/* Advance the background region eraser first: a completed erase
				 * may arm a deferred prepare/start, and the post-ack erase only
				 * progresses from here. */
				service_background_erase();
				const RecorderState current_state = recorder_.state();
				const bool transfer_priority =
						current_state == RecorderState::ReadyForUpload ||
						current_state == RecorderState::Uploading ||
						current_state == RecorderState::AwaitingAck ||
						current_state == RecorderState::EraseAfterAck;
				/* Once capture has finalized, BLE upload/control has priority.
				 * Do not burn foreground time draining a sensor stream whose
				 * samples cannot be used in the retained session. */
				if (!transfer_priority && !record_finalize_pending_) {
					bno85_.service();
				}
				if (armed_) {
					const uint32_t lead_time_ms = static_cast<uint32_t>(pending_start_.start_timestamp_us / 1000ULL);
					const uint32_t elapsed_from_cmd_ms = HAL_GetTick() - armed_tick_ms_;
					if (elapsed_from_cmd_ms >= lead_time_ms) {
						if (!recorder_.region_pre_erased()) {
							/* The region is not erased yet (first boot, retained
							 * previous session, or a layout change). Erase in
							 * background chunks and start capture the moment it
							 * completes — never block the loop with a whole-region
							 * erase inside the capture window. */
							if (!start_erase_pending_) {
								(void)recorder_.begin_region_erase();
								start_erase_pending_ = true;
							}
							return;
						}
						start_delayed_capture_now();
					} else {
						return;
					}
				}
				if (transfer_priority) {
					return;
				}
				if (recorder_.state() != RecorderState::Recording) {
					const uint32_t now_us = HAL_GetTick() * 1000U;

					Bno85Sample bno_sample { };
					if (bno85_.take_latest(now_us, bno_sample)) {
						++data_rate_bno_count_;
					}

					Icm45686Sample icm_sample { };
					if (icm45686_.read_sample(now_us, icm_sample)) {
						++data_rate_icm_count_;
					}
				} else if (finalize_failed_) {
					/* Capture is over and finalize gave up: stop sampling into a
					 * dead flash so the node stays quiet but fully responsive. */
				} else {
					process_recording_ticks();
					process_writer_budget();
					try_finalize_recording();
				}

				if (data_rate_last_log_ms_ == 0U) {
					reset_data_rate_log();
				}
				const uint32_t rate_elapsed_ms = HAL_GetTick() - data_rate_last_log_ms_;
				if (rate_elapsed_ms >= kDataRateLogPeriodMs) {
					const uint32_t bno_rate_x100 = (data_rate_bno_count_ * 100000U) / rate_elapsed_ms;
					const uint32_t icm_rate_x100 = (data_rate_icm_count_ * 100000U) / rate_elapsed_ms;
					EXO_LOG("NODE%u IMU rate: BNO=%lu.%02lu Hz (%lu/%lu ms) ICM=%lu.%02lu Hz (%lu/%lu ms)\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(bno_rate_x100 / 100U),
							static_cast<unsigned long>(bno_rate_x100 % 100U),
							static_cast<unsigned long>(data_rate_bno_count_),
							static_cast<unsigned long>(rate_elapsed_ms),
							static_cast<unsigned long>(icm_rate_x100 / 100U),
							static_cast<unsigned long>(icm_rate_x100 % 100U),
							static_cast<unsigned long>(data_rate_icm_count_),
							static_cast<unsigned long>(rate_elapsed_ms));
					reset_data_rate_log();
				}
			}

			bool session_ready() const {
				return recorder_.state() == RecorderState::ReadyForUpload;
			}

			bool ready() const {
				return ready_;
			}

			uint32_t detected_flash_capacity() const {
				return detected_flash_capacity_;
			}

			uint32_t maximum_duration_ms() const {
				const uint64_t bytes_per_second =
						static_cast<uint64_t>(sizeof(Bno85Sample)) * kBnoTargetRateHz +
						static_cast<uint64_t>(sizeof(Icm45686Sample)) * kIcmTargetRateHz;
				if (bytes_per_second == 0U || usable_flash_capacity_ <= sizeof(SessionHeader)) {
					return 0U;
				}
				return static_cast<uint32_t>(
						(static_cast<uint64_t>(usable_flash_capacity_ - sizeof(SessionHeader)) * 1000ULL) /
						bytes_per_second);
			}

			bool can_start_recording() const {
				const RecorderState state = recorder_.state();
				/* ReadyForUpload is deliberately excluded: the node's flash copy
				 * is the only durable record until the Master has validated the
				 * upload, so a plain StartRecord must never wipe it. The
				 * sanctioned wipe is the explicit reset/erase (0xB3) the Master
				 * sends before arming a new session, or the post-ack erase. */
				return ready_ && !armed_ && !prepared_ && state == RecorderState::Idle;
			}

			bool reset_to_idle_and_erase() {
				if (!ready_) {
					return false;
				}
				/* Returns to Idle immediately; the region erase continues in
				 * background chunks from process(). */
				const bool ok = recorder_.force_reset_and_background_erase();
				if (!ok) {
					return false;
				}
				bno85_.set_capture_queue_enabled(false);
				bno85_.clear_capture_queue();
				if (record_icm_fifo_active_) {
					(void)icm45686_.end_fifo_capture();
					record_icm_fifo_active_ = false;
				}
				armed_ = false;
				prepared_ = false;
				prepare_erase_pending_ = false;
				start_erase_pending_ = false;
				commit_pending_ = false;
				armed_tick_ms_ = 0U;
				recording_started_ms_ = 0U;
				reset_capture_schedule();
				record_finalize_pending_ = false;
				bno_append_fail_count_ = 0U;
				icm_append_fail_count_ = 0U;
				memset(&pending_start_, 0, sizeof(pending_start_));
				memset(&prepared_start_, 0, sizeof(prepared_start_));
				memset(&pending_commit_, 0, sizeof(pending_commit_));
				memset(&pending_capture_start_, 0, sizeof(pending_capture_start_));
				reset_record_buffers();
				reset_data_rate_log();
				return true;
			}

			RecordDoneMessage make_record_done() const {
				RecordDoneMessage message { };
				message.command = RecordCommand::RecordDone;
				message.node_id = recorder_.header().node_id;
				message.session_id = recorder_.header().session_id;
				message.actual_duration_ms = recorder_.header().actual_duration_ms;
				message.total_size = recorder_.total_size();
				message.payload_crc32 = recorder_.header().payload_crc32;
				return message;
			}

			SessionUploadReader make_upload_reader() {
				/* The finalized header lives in recovery slot B; the logical
				 * [header|BNO|ICM] stream maps it from there. */
				return SessionUploadReader(flash_, recorder_.header(), recorder_.finalized_header_address(),
						recorder_.bno85_payload_address(), recorder_.icm45686_payload_address());
			}

			bool begin_upload() {
				return recorder_.begin_upload();
			}
			bool transfer_complete() {
				return recorder_.mark_transfer_complete();
			}
			bool acknowledge_and_erase() {
				/* Non-blocking: flips to EraseAfterAck and starts the chunked
				 * background erase; completes from process(). */
				return recorder_.acknowledge_and_begin_erase();
			}
			RecorderState state() const {
				return recorder_.state();
			}
			bool flash_get_jedec(uint8_t &manufacturer, uint8_t &device_id_hi, uint8_t &device_id_lo) {
				return flash_.get_jedec_id(manufacturer, device_id_hi, device_id_lo);
			}
			bool flash_self_test_128(uint32_t address, uint32_t seed, uint16_t &mismatch_count, uint16_t &first_mismatch_index,
					uint8_t *written_out = nullptr, uint8_t *read_out = nullptr) {
				return flash_.random_write_read_test_128(address, seed, mismatch_count, first_mismatch_index, written_out, read_out);
			}
			const char* flash_last_error_string() const {
				return flash_.last_error_string();
			}
			exo::W25Q256Flash::DebugInfo flash_debug_info() const {
				return flash_.debug_info();
			}
			bool flash_erase_raw(uint32_t address, uint32_t size) {
				return flash_.erase_region(address, size);
			}
			/* Reserved recovery-sector base (0 before begin()). Safe for the
			 * DIAG boot self-test: it is below the settings sector and is always
			 * re-erased by the background eraser before the next session's
			 * headers are written, so the test can never corrupt a session. */
			uint32_t recovery_sector_address() const {
				if (detected_flash_capacity_ == 0U) {
					return 0U;
				}
				return detected_flash_capacity_ - kReservedFlashBytes;
			}
			bool flash_write_raw(uint32_t address, const void *data, uint32_t size) {
				return flash_.write(address, data, size);
			}
			bool flash_read_raw(uint32_t address, void *data, uint32_t size) {
				return flash_.read(address, data, size);
			}

		private:
			static constexpr uint32_t kMinimumSupportedFlashSize = 2U * 1024U * 1024U;
			static constexpr uint32_t kSettingsSectorSize = 4096U;
			static constexpr uint32_t kRecoverySectorSize = 4096U;
			static constexpr uint32_t kReservedFlashBytes = kSettingsSectorSize + kRecoverySectorSize;

			void normalize_start_message(StartRecordMessage &message) const {
				message.command = RecordCommand::StartRecord;
				if (message.requested_duration_ms == 0U) {
					message.requested_duration_ms = kDefaultDurationMs;
				}
				if (message.start_timestamp_us == 0U) {
					message.start_timestamp_us = kDefaultLeadTimeUs;
				}
			}

			bool arm_recording(const StartRecordMessage &message) {
				if (!configure_session_layout(message.requested_duration_ms)) {
					return false;
				}
				reset_record_buffers();
				bno_append_fail_count_ = 0U;
				icm_append_fail_count_ = 0U;
				pending_start_ = message;
				armed_tick_ms_ = HAL_GetTick();
				armed_ = true;
				EXO_LOG("NODE%u record armed: session=%lu lead_ms=%lu duration_ms=%lu state=%u\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(pending_start_.session_id),
						static_cast<unsigned long>(pending_start_.start_timestamp_us / 1000ULL),
						static_cast<unsigned long>(pending_start_.requested_duration_ms),
						static_cast<unsigned>(recorder_.state()));
				return true;
			}

			bool start_recording_now(const StartRecordMessage &message) {
				const RecorderState state = recorder_.state();
				if (state != RecorderState::Idle && state != RecorderState::ReadyForUpload && state != RecorderState::Armed) {
					return false;
				}
				if (state != RecorderState::Armed && !configure_session_layout(message.requested_duration_ms)) {
					return false;
				}
				if (state == RecorderState::Idle && !recorder_.region_pre_erased() &&
						recorder_.erase_in_progress()) {
					/* Immediate start requested while the background erase is
					 * still covering the region: accept and start the moment it
					 * completes. */
					pending_capture_start_ = message;
					start_erase_pending_ = true;
					return true;
				}
				reset_record_buffers();
				bno_append_fail_count_ = 0U;
				icm_append_fail_count_ = 0U;
				pending_start_ = message;
				armed_ = false;
				armed_tick_ms_ = 0U;
				recording_started_ms_ = HAL_GetTick();
				const bool started = state == RecorderState::Armed ?
						recorder_.start_prepared(config_.node_id,
								pending_start_.session_id,
								pending_start_.start_timestamp_us,
								pending_start_.requested_duration_ms) :
						recorder_.start(config_.node_id,
								pending_start_.session_id,
								pending_start_.start_timestamp_us,
								pending_start_.requested_duration_ms);
				if (!started) {
					EXO_LOG("NODE%u record start failed: session=%lu duration_ms=%lu state=%u\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(pending_start_.session_id),
							static_cast<unsigned long>(pending_start_.requested_duration_ms),
							static_cast<unsigned>(recorder_.state()));
					return false;
				}
				bno85_.set_capture_queue_enabled(true);
				/* Node ICM records through the documented 200 Hz hardware FIFO
				 * (real per-frame timestamps, no polled duplicates). If the
				 * FIFO cannot be armed, record_icm_fifo_active_ stays false and
				 * the 5 ms direct-register path runs as an automatic fallback. */
				record_icm_fifo_active_ = icm45686_.begin_fifo_capture_200hz();
				reset_capture_schedule();
				record_finalize_pending_ = false;
				reset_data_rate_log();
				EXO_LOG("NODE%u record started: session=%lu duration_ms=%lu immediate=1\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(pending_start_.session_id),
						static_cast<unsigned long>(pending_start_.requested_duration_ms));
				EXO_LOG("[RECORD][NODE%u] START session=%lu duration_ms=%lu mode=immediate\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(pending_start_.session_id),
						static_cast<unsigned long>(pending_start_.requested_duration_ms));
				return true;
			}

			bool flash_layout_valid() const {
				if (detected_flash_capacity_ == 0U ||
						config_.flash_base_address >= detected_flash_capacity_) {
					return false;
				}
				if (usable_flash_capacity_ == 0U) {
					return false;
				}
				const uint32_t end = config_.flash_base_address + usable_flash_capacity_;
				if (end < config_.flash_base_address) {
					return false;
				}
				if (end > (detected_flash_capacity_ - kReservedFlashBytes)) {
					return false;
				}
				return true;
			}

			bool configure_session_layout(uint32_t duration_ms) {
				if (duration_ms == 0U) {
					return false;
				}
				const uint64_t bno_count =
						(static_cast<uint64_t>(duration_ms) * kBnoTargetRateHz + 999ULL) / 1000ULL;
				const uint64_t icm_count =
						(static_cast<uint64_t>(duration_ms) * kIcmTargetRateHz + 999ULL) / 1000ULL;
				const uint64_t bno_bytes = bno_count * sizeof(Bno85Sample);
				const uint64_t icm_bytes = icm_count * sizeof(Icm45686Sample);
				const uint64_t required = sizeof(SessionHeader) + bno_bytes + icm_bytes;
				const uint64_t erase_bytes = (required + 4095ULL) & ~4095ULL;
				if (bno_bytes > 0xFFFFFFFFULL || erase_bytes > usable_flash_capacity_) {
					return false;
				}
				return recorder_.configure_layout(static_cast<uint32_t>(erase_bytes),
						static_cast<uint32_t>(bno_bytes));
			}

			template<typename Sample, uint32_t BatchSize>
			struct DoubleBatchBuffer {
					Sample data[2][BatchSize] { };
					uint32_t count[2] = { 0U, 0U };
					bool pending[2] = { false, false };
					uint8_t active = 0U;
					uint32_t drops = 0U;

					void reset() {
						count[0] = 0U;
						count[1] = 0U;
						pending[0] = false;
						pending[1] = false;
						active = 0U;
						drops = 0U;
					}

					bool enqueue(const Sample &sample) {
						if (count[active] >= BatchSize) {
							pending[active] = true;
							const uint8_t other = static_cast<uint8_t>(active ^ 1U);
							if (pending[other] || count[other] != 0U) {
								++drops;
								return false;
							}
							active = other;
							count[active] = 0U;
						}
						data[active][count[active]++] = sample;
						if (count[active] >= BatchSize) {
							pending[active] = true;
							const uint8_t other = static_cast<uint8_t>(active ^ 1U);
							if (!pending[other] && count[other] == 0U) {
								active = other;
							}
						}
						return true;
					}

					void mark_partial_pending() {
						if (count[active] > 0U) {
							pending[active] = true;
						}
					}

					bool has_pending() const {
						return pending[0] || pending[1];
					}
			};

			void process_recording_ticks() {
				const uint32_t elapsed_us = (HAL_GetTick() - recording_started_ms_) * 1000U;
				const uint32_t duration_us = pending_start_.requested_duration_ms * 1000U;
			if (record_finalize_pending_) {
				return;
			}
			/* BNO samples are produced by the sensor-driven capture queue at
			 * the report rate, not by this loop's polling cadence, so a slow
			 * iteration only delays the drain instead of losing reports. */
			const uint8_t bno_pop = bno85_.pop_samples(bno_drain_buf_, kMaxBnoDrainPerTick);
			for (uint8_t i = 0U; i < bno_pop; ++i) {
				++bno_captured_count_;
				++data_rate_bno_count_;
				if (!bno_record_buf_.enqueue(bno_drain_buf_[i])) {
					loss_flags_ |= kSessionLossBnoBuffer;
				}
				(void)live_queue_.offer(kBnoLiveSensorId, &bno_drain_buf_[i],
						static_cast<uint8_t>(sizeof(bno_drain_buf_[i])),
						recording_started_ms_ + (bno_drain_buf_[i].offset_us / 1000U));
			}

			if (record_icm_fifo_active_) {
				const uint8_t icm_count =
						icm45686_.read_fifo_samples(icm_drain_buf_, kMaxIcmDrainPerTick);
				for (uint8_t i = 0U; i < icm_count; ++i) {
					++icm_captured_count_;
					++data_rate_icm_count_;
					if (!icm_record_buf_.enqueue(icm_drain_buf_[i])) {
						loss_flags_ |= kSessionLossIcmBuffer;
					}
					(void)live_queue_.offer(kIcmLiveSensorId, &icm_drain_buf_[i],
							static_cast<uint8_t>(sizeof(icm_drain_buf_[i])),
							recording_started_ms_ + (icm_drain_buf_[i].offset_us / 1000U));
				}
			} else {
				uint8_t catchup = 0U;
				while (record_next_icm_us_ < duration_us &&
						static_cast<int32_t>(elapsed_us - record_next_icm_us_) >= 0 && catchup < 4U) {
					++icm_attempted_count_;
					Icm45686Sample sample { };
					if (icm45686_.read_sample(record_next_icm_us_, sample)) {
						sample.sequence = icm_captured_count_;
						++icm_captured_count_;
						++data_rate_icm_count_;
						if (!icm_record_buf_.enqueue(sample)) {
							loss_flags_ |= kSessionLossIcmBuffer;
						}
						(void)live_queue_.offer(kIcmLiveSensorId, &sample,
								static_cast<uint8_t>(sizeof(sample)),
								recording_started_ms_ + (record_next_icm_us_ / 1000U));
					} else {
						loss_flags_ |= kSessionLossIcmRead;
					}
					record_next_icm_us_ += kIcmPeriodUs;
					++catchup;
				}
			}

			if (elapsed_us >= duration_us &&
					(record_icm_fifo_active_ || record_next_icm_us_ >= duration_us) &&
					!finalize_failed_) {
				finalize_duration_ms_ = HAL_GetTick() - recording_started_ms_;
				bno85_.set_capture_queue_enabled(false);
				record_finalize_pending_ = true;
				set_live_stream_enabled(false);
			}
		}

			/* Delayed-path capture start, called once the lead time has elapsed
			 * AND the region erase has completed. */
			void start_delayed_capture_now() {
				recording_started_ms_ = HAL_GetTick();
				const bool started = recorder_.start(config_.node_id, pending_start_.session_id, pending_start_.start_timestamp_us,
						pending_start_.requested_duration_ms);
				armed_ = false;
				if (!started) {
					EXO_LOG("NODE%u record start failed: session=%lu duration_ms=%lu state=%u\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(pending_start_.session_id),
							static_cast<unsigned long>(pending_start_.requested_duration_ms),
							static_cast<unsigned>(recorder_.state()));
					return;
				}
				bno85_.set_capture_queue_enabled(true);
				/* Node ICM records through the documented 200 Hz hardware FIFO
				 * (real per-frame timestamps, no polled duplicates). If the
				 * FIFO cannot be armed, record_icm_fifo_active_ stays false and
				 * the 5 ms direct-register path runs as an automatic fallback. */
				record_icm_fifo_active_ = icm45686_.begin_fifo_capture_200hz();
				reset_capture_schedule();
				record_finalize_pending_ = false;
				reset_data_rate_log();
				EXO_LOG("NODE%u record started: session=%lu duration_ms=%lu\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(pending_start_.session_id),
						static_cast<unsigned long>(pending_start_.requested_duration_ms));
				EXO_LOG("[RECORD][NODE%u] START session=%lu duration_ms=%lu mode=delayed\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(pending_start_.session_id),
						static_cast<unsigned long>(pending_start_.requested_duration_ms));
			}

			/* Advance the background region eraser by a bounded number of
			 * sectors per tick. A 10-minute layout erases for ~a minute; the
			 * chunking keeps BLE dispatch, heartbeats and the radio alive
			 * throughout instead of blocking the whole node. */
			void service_background_erase() {
				if (!recorder_.erase_in_progress()) {
					return;
				}
				const NodeRecorder::EraseStep step =
						recorder_.service_region_erase(kEraseSectorsPerTick);
				if (step != NodeRecorder::EraseStep::Complete &&
						step != NodeRecorder::EraseStep::Failed) {
					return;
				}
				if (step == NodeRecorder::EraseStep::Complete) {
					if (recorder_.state() == RecorderState::EraseAfterAck) {
						(void)recorder_.erase_after_ack_done();
						EXO_LOG("[RECORD][NODE%u] post-ack erase complete; region pre-erased for next session\r\n",
								static_cast<unsigned>(config_.node_id));
						return;
					}
					if (prepare_erase_pending_) {
						prepare_erase_pending_ = false;
						if (recorder_.finish_prepare_header()) {
							EXO_LOG("[RECORD][NODE%u] PREPARE erase complete session=%lu state=%u\r\n",
									static_cast<unsigned>(config_.node_id),
									static_cast<unsigned long>(prepared_start_.session_id),
									static_cast<unsigned>(recorder_.state()));
							if (commit_pending_) {
								commit_pending_ = false;
								(void)commit_prepared_recording(pending_commit_);
							}
						} else {
							prepared_ = false;
							memset(&prepared_start_, 0, sizeof(prepared_start_));
							EXO_LOG("[RECORD][NODE%u] PREPARE header write failed\r\n",
									static_cast<unsigned>(config_.node_id));
						}
						return;
					}
					if (start_erase_pending_) {
						start_erase_pending_ = false;
						if (armed_) {
							start_delayed_capture_now();
						} else {
							(void)start_recording_now(pending_capture_start_);
						}
					}
					return;
				}
				/* Erase failed: clear the pending intents so the node stays
				 * responsive; the failure is visible via status and logs. */
				prepare_erase_pending_ = false;
				start_erase_pending_ = false;
				commit_pending_ = false;
				if (recorder_.state() == RecorderState::EraseAfterAck) {
					(void)recorder_.erase_after_ack_failed();
				}
				EXO_LOG("[RECORD][NODE%u] region erase failed; recording unavailable until flash recovers\r\n",
						static_cast<unsigned>(config_.node_id));
			}

			bool flush_one_bno_pending() {
				uint8_t idx = bno_flush_rr_;
				for (uint8_t n = 0U; n < 2U; ++n) {
					if (bno_record_buf_.pending[idx] && bno_record_buf_.count[idx] > 0U) {
						const uint32_t count = bno_record_buf_.count[idx];
						if (!recorder_.append_bno85_batch(bno_record_buf_.data[idx], count)) {
							/* Counted once if the retry budget gives up (see
							 * drop_pending_batches_), never per failed attempt. */
							last_flush_write_failed_ = true;
							return false;
						}
						bno_record_buf_.pending[idx] = false;
						bno_record_buf_.count[idx] = 0U;
						bno_flush_rr_ = static_cast<uint8_t>(idx ^ 1U);
						return true;
					}
					idx = static_cast<uint8_t>(idx ^ 1U);
				}
				return false;
			}

			bool flush_one_icm_pending() {
				uint8_t idx = icm_flush_rr_;
				for (uint8_t n = 0U; n < 2U; ++n) {
					if (icm_record_buf_.pending[idx] && icm_record_buf_.count[idx] > 0U) {
						const uint32_t count = icm_record_buf_.count[idx];
						if (!recorder_.append_icm45686_batch(icm_record_buf_.data[idx], count)) {
							last_flush_write_failed_ = true;
							return false;
						}
						icm_record_buf_.pending[idx] = false;
						icm_record_buf_.count[idx] = 0U;
						icm_flush_rr_ = static_cast<uint8_t>(idx ^ 1U);
						return true;
					}
					idx = static_cast<uint8_t>(idx ^ 1U);
				}
				return false;
			}

			/* Give up on batches the flash refuses to take after the retry
			 * budget: count them once as write losses (they were never stored)
			 * and let finalization complete with the data that IS durable. */
			void drop_pending_batches() {
				bool dropped_bno = false;
				bool dropped_icm = false;
				for (uint8_t idx = 0U; idx < 2U; ++idx) {
					if (bno_record_buf_.pending[idx] && bno_record_buf_.count[idx] > 0U) {
						bno_append_fail_count_ += bno_record_buf_.count[idx];
						bno_record_buf_.pending[idx] = false;
						bno_record_buf_.count[idx] = 0U;
						dropped_bno = true;
					}
					if (icm_record_buf_.pending[idx] && icm_record_buf_.count[idx] > 0U) {
						icm_append_fail_count_ += icm_record_buf_.count[idx];
						icm_record_buf_.pending[idx] = false;
						icm_record_buf_.count[idx] = 0U;
						dropped_icm = true;
					}
				}
				if (dropped_bno) {
					loss_flags_ |= kSessionLossBnoWrite;
				}
				if (dropped_icm) {
					loss_flags_ |= kSessionLossIcmWrite;
				}
				write_fail_streak_ = 0U;
				EXO_LOG("[RECORD][NODE%u] flash writes kept failing; dropped pending batches bno_fail=%lu icm_fail=%lu\r\n",
						static_cast<unsigned>(config_.node_id),
						static_cast<unsigned long>(bno_append_fail_count_),
						static_cast<unsigned long>(icm_append_fail_count_));
			}

			void process_writer_budget() {
				/* The loop exits as soon as no batch is pending, so the cap only
				 * bounds the catch-up burst after a stall; at 400 Hz the double
				 * buffer holds ~40 ms of samples and one flush moves 8. */
				for (uint8_t flushes = 0U; flushes < kFlushBatchesPerTick; ++flushes) {
					last_flush_write_failed_ = false;
					bool wrote = false;
					if (prefer_bno_flush_) {
						wrote = flush_one_bno_pending();
						if (!wrote && !last_flush_write_failed_) {
							wrote = flush_one_icm_pending();
						}
					} else {
						wrote = flush_one_icm_pending();
						if (!wrote && !last_flush_write_failed_) {
							wrote = flush_one_bno_pending();
						}
					}
					if (last_flush_write_failed_) {
						/* A dead flash must fail the session cleanly instead of
						 * wedging the node in Recording forever. */
						++write_fail_streak_;
						if (write_fail_streak_ >= kMaxConsecutiveWriteFails) {
							drop_pending_batches();
						}
						return;
					}
					if (!wrote) {
						break;
					}
					prefer_bno_flush_ = !prefer_bno_flush_;
				}
				write_fail_streak_ = 0U;
			}

			void try_finalize_recording() {
				if (!record_finalize_pending_ || recorder_.state() != RecorderState::Recording) {
					return;
				}
				/* Freeze the producer, then drain the preserved tail before
				 * declaring the double-buffer partial batch final. */
				bno85_.set_capture_queue_enabled(false);
				const uint8_t bno_tail = bno85_.pop_samples(
						bno_drain_buf_, kMaxBnoDrainPerTick);
				for (uint8_t i = 0U; i < bno_tail; ++i) {
					++bno_captured_count_;
					++data_rate_bno_count_;
					if (!bno_record_buf_.enqueue(bno_drain_buf_[i])) {
						loss_flags_ |= kSessionLossBnoBuffer;
					}
				}
				if (bno_tail != 0U) {
					process_writer_budget();
					return;
				}
				if (record_icm_fifo_active_) {
					const uint8_t icm_tail =
							icm45686_.read_fifo_samples(icm_drain_buf_, kMaxIcmDrainPerTick);
					if (icm_tail == 0U && icm45686_.last_read_status() != 0) {
						/* An I2C error and an empty FIFO both read as zero frames.
						 * Only the empty FIFO means "caught the tail": retry a
						 * transient error so a bus glitch cannot silently drop
						 * the ICM tail, but bounded so a dead bus cannot stall
						 * finalization forever. */
						if (icm_tail_read_fail_count_ < kMaxIcmTailReadFails) {
							++icm_tail_read_fail_count_;
							return;
						}
						loss_flags_ |= kSessionLossIcmRead;
					}
					for (uint8_t i = 0U; i < icm_tail; ++i) {
						++icm_captured_count_;
						++data_rate_icm_count_;
						if (!icm_record_buf_.enqueue(icm_drain_buf_[i])) {
							loss_flags_ |= kSessionLossIcmBuffer;
						}
					}
					if (icm_tail >= kMaxIcmDrainPerTick) {
						process_writer_budget();
						return;
					}
					/* Less than a full drain means we caught the FIFO tail.
					 * Stop it immediately so new frames cannot keep finalization
					 * alive forever. */
					(void)icm45686_.end_fifo_capture();
					record_icm_fifo_active_ = false;
				}
				bno_record_buf_.mark_partial_pending();
			icm_record_buf_.mark_partial_pending();
			process_writer_budget();
			if (bno_record_buf_.has_pending() || icm_record_buf_.has_pending()) {
				return;
			}
			/* The converter validates captured_count == sample_count, so the
			 * header must describe what is durably stored, never what was
			 * merely popped from the driver queue. Attempted is the target
			 * rate over the achieved duration (or the stored count when the
			 * sensor over-delivered); anything missing shows up as dropped. */
			const uint32_t stored_bno = recorder_.header().bno85_sample_count;
			const uint32_t stored_icm = recorder_.header().icm45686_sample_count;
			const uint32_t expected_bno =
					((finalize_duration_ms_ * kBnoTargetRateHz) + 999U) / 1000U;
			const uint32_t expected_icm =
					((finalize_duration_ms_ * kIcmTargetRateHz) + 999U) / 1000U;
			const uint32_t bno_local_loss =
					bno85_.queue_drops() + bno_record_buf_.drops + bno_append_fail_count_;
			const uint32_t icm_local_loss =
					icm_record_buf_.drops + icm_append_fail_count_;
			/* Known FIFO/buffer/write losses are real attempts even when the sensor
			 * temporarily over-delivered versus the nominal duration-derived rate.
			 * Never allow a loss flag with dropped_count == 0. */
			const uint32_t observed_bno_attempts = stored_bno + bno_local_loss;
			const uint32_t observed_icm_attempts = stored_icm + icm_local_loss;
			bno_attempted_count_ =
					(expected_bno > observed_bno_attempts) ? expected_bno : observed_bno_attempts;
			icm_attempted_count_ =
					(expected_icm > observed_icm_attempts) ? expected_icm : observed_icm_attempts;
			if (bno85_.queue_drops() != 0U) {
				loss_flags_ |= kSessionLossBnoBuffer;
			}
			if ((bno_attempted_count_ - stored_bno) > bno_local_loss) {
				loss_flags_ |= kSessionLossBnoRead;
			}
			if ((icm_attempted_count_ - stored_icm) > icm_local_loss) {
				loss_flags_ |= kSessionLossIcmRead;
			}
			recorder_.set_capture_metadata(kBnoTargetRateHz, kIcmTargetRateHz,
					bno_attempted_count_, icm_attempted_count_,
					stored_bno, stored_icm,
					bno_attempted_count_ - stored_bno,
					icm_attempted_count_ - stored_icm,
					loss_flags_);
				if (recorder_.finalize(finalize_duration_ms_)) {
					EXO_LOG("NODE%u record finalized: duration_ms=%lu bno=%lu icm=%lu size=%lu bno_fail=%lu icm_fail=%lu bno_drop=%lu icm_drop=%lu\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(finalize_duration_ms_),
							static_cast<unsigned long>(recorder_.header().bno85_sample_count),
							static_cast<unsigned long>(recorder_.header().icm45686_sample_count),
							static_cast<unsigned long>(recorder_.total_size()),
							static_cast<unsigned long>(bno_append_fail_count_),
							static_cast<unsigned long>(icm_append_fail_count_),
							static_cast<unsigned long>(bno_record_buf_.drops),
							static_cast<unsigned long>(icm_record_buf_.drops));
					EXO_LOG("[RECORD][NODE%u] END session=%lu duration_ms=%lu size=%lu bno=%lu icm=%lu\r\n",
							static_cast<unsigned>(config_.node_id),
							static_cast<unsigned long>(recorder_.header().session_id),
							static_cast<unsigned long>(finalize_duration_ms_),
							static_cast<unsigned long>(recorder_.total_size()),
							static_cast<unsigned long>(recorder_.header().bno85_sample_count),
							static_cast<unsigned long>(recorder_.header().icm45686_sample_count));
					finalize_fail_count_ = 0U;
				} else {
					/* A finalize that keeps failing (flash CRC read or header
					 * write) must not retry forever: after the attempt budget,
					 * latch the failure so the node stays responsive (BLE,
					 * Cancel, reboot) instead of silently wedging in Recording. */
					++finalize_fail_count_;
					if (finalize_fail_count_ >= kMaxFinalizeAttempts) {
						finalize_failed_ = true;
						EXO_LOG("[RECORD][NODE%u] finalize failed %u times; giving up (data retained on flash, node remains responsive)\r\n",
								static_cast<unsigned>(config_.node_id),
								static_cast<unsigned>(finalize_fail_count_));
					}
				}
				bno85_.clear_capture_queue();
				record_finalize_pending_ = false;
			}

			void reset_data_rate_log() {
				data_rate_last_log_ms_ = HAL_GetTick();
				data_rate_bno_count_ = 0U;
				data_rate_icm_count_ = 0U;
			}

			void reset_record_buffers() {
				bno_record_buf_.reset();
				icm_record_buf_.reset();
				bno_flush_rr_ = 0U;
				icm_flush_rr_ = 0U;
				prefer_bno_flush_ = true;
			}

			void reset_capture_schedule() {
				record_next_icm_us_ = 0U;
				bno_attempted_count_ = 0U;
				icm_attempted_count_ = 0U;
				bno_captured_count_ = 0U;
				icm_captured_count_ = 0U;
				loss_flags_ = 0U;
				last_flush_write_failed_ = false;
				write_fail_streak_ = 0U;
				finalize_fail_count_ = 0U;
				finalize_failed_ = false;
				icm_tail_read_fail_count_ = 0U;
				finalize_duration_ms_ = 0U;
				record_finalize_pending_ = false;
			}

			static constexpr uint32_t kBnoBatchSamples = 8U;
			static constexpr uint32_t kIcmBatchSamples = 8U;
			static constexpr uint8_t kFlushBatchesPerTick = 4U;
			static constexpr uint8_t kMaxIcmDrainPerTick = 16U;
			/* Retry budgets: a persistently failing flash must fail the session
			 * cleanly (data retained, node responsive) instead of wedging the
			 * node in Recording forever. */
			static constexpr uint8_t kMaxConsecutiveWriteFails = 8U;
			static constexpr uint8_t kMaxFinalizeAttempts = 3U;
			static constexpr uint8_t kMaxIcmTailReadFails = 5U;
			/* Background erase chunk: 2 sectors (~90 ms typ) per process tick.
			 * Large enough that a 10-min layout erases in ~1 minute of ticks,
			 * small enough that BLE dispatch and heartbeats keep flowing. */
			static constexpr uint8_t kEraseSectorsPerTick = 2U;

			NodeRecordingConfig config_;
			Bno85Stm32 bno85_;
			Icm45686Stm32 icm45686_;
			W25Q256Flash flash_;
			NodeRecorder recorder_;
			uint32_t recording_started_ms_ = 0U;
			uint32_t armed_tick_ms_ = 0U;
			StartRecordMessage pending_start_ { };
			StartRecordMessage prepared_start_ { };
			uint32_t data_rate_last_log_ms_ = 0U;
			uint32_t data_rate_bno_count_ = 0U;
			uint32_t data_rate_icm_count_ = 0U;
			DoubleBatchBuffer<Bno85Sample, kBnoBatchSamples> bno_record_buf_ { };
			DoubleBatchBuffer<Icm45686Sample, kIcmBatchSamples> icm_record_buf_ { };
			Bno85Sample bno_drain_buf_[kMaxBnoDrainPerTick] { };
			Icm45686Sample icm_drain_buf_[kMaxIcmDrainPerTick] { };
			bool record_icm_fifo_active_ = false;
			uint8_t bno_flush_rr_ = 0U;
			uint8_t icm_flush_rr_ = 0U;
			bool prefer_bno_flush_ = true;
			uint32_t record_next_icm_us_ = 0U;
			uint32_t bno_attempted_count_ = 0U;
			uint32_t icm_attempted_count_ = 0U;
			uint32_t bno_captured_count_ = 0U;
			uint32_t icm_captured_count_ = 0U;
			uint32_t loss_flags_ = 0U;
			uint32_t finalize_duration_ms_ = 0U;
			bool record_finalize_pending_ = false;
			bool last_flush_write_failed_ = false;
			uint8_t write_fail_streak_ = 0U;
			uint8_t finalize_fail_count_ = 0U;
			bool finalize_failed_ = false;
			uint8_t icm_tail_read_fail_count_ = 0U;
			/* Background-erase sequencing: prepare/start/commit deferred while
			 * the region eraser is still covering the session region. */
			bool prepare_erase_pending_ = false;
			bool start_erase_pending_ = false;
			bool commit_pending_ = false;
			StartRecordMessage pending_commit_ { };
			StartRecordMessage pending_capture_start_ { };
			uint32_t bno_append_fail_count_ = 0U;
			uint32_t icm_append_fail_count_ = 0U;
			bool armed_ = false;
			bool live_stream_enabled_ = false;
			uint32_t live_interval_ms_ = 20U;
			NodeLiveSampleQueue<kMaxLivePayload, 8U> live_queue_ { };
			bool prepared_ = false;
			bool ready_ = false;
			uint32_t detected_flash_capacity_ = 0U;
			uint32_t usable_flash_capacity_ = 0U;
	};

} // namespace exo

#endif
