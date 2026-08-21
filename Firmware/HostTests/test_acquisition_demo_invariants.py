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
    ("!hub_sensor_test_app.record_bno_queue_active();" in master,
     "Master must hold SWO telemetry and the live plot stream while a recorded capture owns the sensors"),
]

failures = [message for ok, message in checks if not ok]
if failures:
    for failure in failures:
        print(f"ERROR: {failure}", file=sys.stderr)
    raise SystemExit(1)
print("acquisition demo invariants passed")