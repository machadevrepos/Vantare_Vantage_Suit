#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
bno = (ROOT / "Firmware/LIBRARY/CUSTOM/BNO85_STM32.h").read_text()
icm = (ROOT / "Firmware/LIBRARY/CUSTOM/ICM45686_STM32.h").read_text()
hub = (ROOT / "Firmware/LIBRARY/CUSTOM/HUB_SENSOR_TEST_APP.h").read_text()
node = (ROOT / "Firmware/LIBRARY/CUSTOM/NODE_RECORDING_APP.h").read_text()
master = (ROOT / "Firmware/Master/Core/Src/main.c").read_text()
recorder = (ROOT / "Firmware/LIBRARY/CUSTOM/MASTER_SD_SESSION_RECORDER.h").read_text()
central = (ROOT / "Firmware/Master/STM32_WPAN/App/exo_hub_central_client.c").read_text()
central_h = (ROOT / "Firmware/Master/STM32_WPAN/App/exo_hub_central_client.h").read_text()
node_main = (ROOT / "Firmware/Node/Core/Src/main.c").read_text()
stager = (ROOT / "Firmware/Master/Core/Inc/MASTER_NODE_SESSION_STAGER.h").read_text()
coordinator = (ROOT / "Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h").read_text()
reliable = (ROOT / "Firmware/Master/Core/Inc/MASTER_NODE_RELIABLE_CONTROL.h").read_text()

checks = [
    ("void service_pending(uint8_t max_packets)" in bno,
     "BNO wrapper must expose bounded pending-packet service"),
    ("HAL_GPIO_ReadPin(interrupt_port_, interrupt_pin_) == GPIO_PIN_RESET" in bno,
     "BNO service must gate reads from the active-low data-ready level"),
    ("#define EXO_BNO85_RV_REPORT_INTERVAL_US 10000U" in bno and
     "kBnoTargetRateHz = 100U" in node and
     "header_.bno85_target_rate_hz = 100U" in recorder,
     "recording contract must use the reliable 100 Hz BNO target end-to-end"),
    ("capture_start_us_ = micros32()" in bno and
     "latest_sensor_timestamp_us_ - capture_timestamp_origin_us_" not in bno,
     "BNO record offsets must use the monotonic local capture clock"),
    ("exo_hub_central_client_set_discovery_hold" in central_h and
     "g_discovery_hold" in central and
     "duplicate logical node ignored" in central,
     "central client must protect active sessions and enforce one slot per logical Node ID"),
    ("next_read_len_" not in bno,
     "BNO HAL must not expose physical I2C chunking as SHTP fragments"),
    ("cargo_remaining" in bno and "chunk + kShtpHeaderLen" in bno and
     "return static_cast<int>(transfer_len)" in bno,
     "BNO HAL must reconstruct one complete SHTP transfer per read"),
    ("#define EXO_BNO85_AUX_REPORT_INTERVAL_US 20000U" in bno,
     "BNO auxiliary reports must be 50 Hz, not the accidental 200 Hz load"),
    ("clear_capture_queue()" in bno and "capture_start_us_" in bno,
     "BNO capture queue must preserve a drainable tail and use the local capture clock"),
    ("bno85_.set_interrupt_pin(BNO_INT_GPIO_Port, BNO_INT_Pin)" in hub and
     "bno85_.set_interrupt_pin(BNO_INT_GPIO_Port, BNO_INT_Pin)" in node,
     "Master and Node must wire the existing BNO INT pin into the wrapper"),
    ("begin_fifo_capture_200hz" in icm and
     "ACCEL_CONFIG0_ACCEL_ODR_200_HZ" in icm and
     "GYRO_CONFIG0_GYRO_ODR_200_HZ" in icm,
     "Master ICM recording must configure a real 200 Hz FIFO"),
    ("inv_imu_get_frame_count" in icm and "inv_imu_get_fifo_frame" in icm,
     "ICM recording must drain hardware FIFO frames"),
    ("TMST_WOM_CONFIG_TMST_RESOL_16_US" in icm and "* 16U;" in icm,
     "ICM FIFO timestamps must use explicit 16 us resolution and scale raw ticks to microseconds"),
    ("record_icm_fifo_active_" in hub and "drain_record_icm_samples" in hub,
     "Hub app must separate recording FIFO acquisition from live direct reads"),
    ("g_local_stop_waiting_for_nodes" in master and
     "local stop released after node ACKs" in master,
     "manual Master stop must wait for Node StopRecord ACKs"),
    ("automatic capacity guard" in master and
     "g_local_stop_waiting_for_nodes = false;" in master,
     "automatic duration guard must remain a hard local stop boundary"),
    ("hub_sensor_test_app.begin_record_capture()" in master and
     "hub_sensor_test_app.end_record_capture()" in master,
     "Master must bracket recording with ICM FIFO capture lifecycle"),
    ("fifo_samples[32]" in master and
     "append_icm45686(fifo_samples[i])" in master,
     "Master must append all drained FIFO samples, not synthetic repeated register reads"),
    ("expected_bno" in recorder and "expected_icm" in recorder and
     "kSessionLossBnoRead" in recorder and "kSessionLossIcmRead" in recorder,
     "Master SessionHeader must truthfully expose acquisition shortfall"),
    ("observed_icm_attempts = stored_icm + icm_local_loss" in node and
     "observed_bno_attempts = stored_bno + bno_local_loss" in node,
     "Node attempted/dropped metadata must include known buffer/write losses"),
    ("freeze_record_bno_capture" in master and "drain_record_bno_samples" in master,
     "Master stop must freeze then drain the final BNO queue tail"),
    ("begin_fifo_capture_200hz" in node and "read_fifo_samples(icm_drain_buf_" in node and
     "record_icm_fifo_active_ = icm45686_.begin_fifo_capture_200hz();" in node,
     "Node recording must arm the real 200 Hz ICM FIFO instead of synthetic catch-up reads"),
    ("kLocalRecordFinalizeIcmDrainPasses" in master and
     "WARN ICM FIFO tail drain reached safety bound" in master,
     "Master stop must drain the ICM FIFO tail before ending capture"),
    (node_main.index("node_blepipe_process_recording_upload();") <
     node_main.index("node_recording_app.process();"),
     "Node reliable upload/control must run before sensor housekeeping"),
    ("Preferred source has no frame" in master,
     "Master live BLE sender must fall back when the preferred sensor has no frame"),
    ("void service_bno()" in hub and "bno85_.service();" in hub,
     "Hub app must expose a bounded BNO drain for extra superloop service points"),
    (master.count("hub_sensor_test_app.service_bno();") >= 2 and
     master.index("hub_sensor_test_app.service_bno();") > master.index("MX_APPE_Process();") and
     "local_record_collect(hub_snapshot);\n\t\t/* SD appends/flushes above can block" in master,
     "Master superloop must service the BNO after BLE dispatch and after SD collect, not once per iteration"),
    ("!hub_sensor_test_app.record_bno_queue_active()" in master,
     "Master must hold SWO telemetry and the live plot stream while a recorded capture owns the sensors"),
    ("flush_write_buffer" in stager and "write_buffer_len_" in stager and
     stager.count("if (!flush_write_buffer()) {") >= 2 and
     stager.rindex("if (!flush_write_buffer()) {") < stager.index("NodeSessionStageOperation::CloseWrite"),
     "Node session stager must buffer staged writes and flush before validation close"),
    ("(void)flush_write_buffer();" in stager,
     "Stager shutdown must best-effort flush the RAM tail so abandoned sessions keep every byte"),
    ("service_reliable_control" in coordinator and
     "reliable_defer_services_" in coordinator,
     "Coordinator must expose an early reliable-control flush that honors the manifest defer counter"),
    ("master_training_csv_coordinator.service_reliable_control(HAL_GetTick());" in master and
     master.index("master_training_csv_coordinator.service_reliable_control(HAL_GetTick());") <
     master.index("hub_snapshot = hub_sensor_test_app.process()"),
     "Chunk ACKs must flush right after BLE dispatch, not once per superloop"),
    (master.count("!g_ble_record_transfer_mode && !g_remote_transfer_active") >= 2,
     "Master must hold SWO telemetry and the live plot stream while a node upload owns the links"),
    ("#define EXO_HUB_VERBOSE_PIPE_LOGS 0" in central and
     central.count("#if EXO_HUB_VERBOSE_PIPE_LOGS") >= 2,
     "Per-chunk pipe logs must stay behind a default-off verbose switch"),
    ("void exo_hub_central_client_request_targeted_reconnect(uint8_t node_id);" in central_h and
     "g_targeted_reconnect_node_id" in central,
     "Central client must expose targeted direct-address recovery for a dropped session source"),
    ("exo_hub_central_client_request_targeted_reconnect(exo_leaf_slot_node_id(slot));" in central and
     "targeted reconnect arm slot=" in central,
     "A Node drop during discovery hold must arm direct-address recovery instead of waiting"),
    ("nack_pending_" in reliable and
     reliable.index("if (nack_pending_) return send_nack(now_ms);") <
     reliable.index("if (ack_pending_) return send_ack_window(now_ms);") and
     reliable.index("nack_pending_ = true;") < reliable.index("bool send_nack(uint32_t now_ms)"),
     "Gap/corrupt NACKs must own a dedicated slot outranking ACK windows so a queued ManifestAck can never drop recovery"),
    ("abandon_and_unlink" in stager and "stage_started_" in stager and
     coordinator.count("stager_.abandon_and_unlink();") >= 3,
     "Abandoned/failed node stages must be unlinked so run indexes stay reusable and no truncated R####N#.BIN survives"),
    ("RetrySource = 0x10" in (ROOT / "Firmware/LIBRARY/CUSTOM/BLE_RECORD_PROTOCOL.h").read_text() and
     "master_retry_failed_source" in master and
     "retry_failed_source" in coordinator and
     "master_retry_failed_source(message.node_id)" in master,
     "A source written off after a stall must be re-pullable from the retained RecordDone (RetrySource 0x10)"),
    ("kMaxConsecutiveWriteFails" in node and "drop_pending_batches" in node and
     "kMaxFinalizeAttempts" in node and "finalize_failed_" in node,
     "A persistently failing node flash must fail the session cleanly (data retained, node responsive) instead of wedging in Recording forever"),
    ("kNodeRecordBurstLimit = 4U" in node_main,
     "Node upload pacing must not cap the link at ~22 KB/s (one chunk per 8 ms gap) — the 10-min offload target needs headroom"),
]

failures = [message for ok, message in checks if not ok]
if failures:
    for failure in failures:
        print(f"ERROR: {failure}", file=sys.stderr)
    raise SystemExit(1)
print("acquisition demo invariants passed")