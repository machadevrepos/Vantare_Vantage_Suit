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

checks = [
    ("void service_pending(uint8_t max_packets)" in bno,
     "BNO wrapper must expose bounded pending-packet service"),
    ("HAL_GPIO_ReadPin(interrupt_port_, interrupt_pin_) == GPIO_PIN_RESET" in bno,
     "BNO service must gate reads from the active-low data-ready level"),
    ("next_read_len_ != kShtpHeaderLen" in bno,
     "BNO service must finish partial SHTP transfers regardless of INT level"),
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
    ("fifo_samples[16]" in master and
     "append_icm45686(fifo_samples[i])" in master,
     "Master must append all drained FIFO samples, not synthetic repeated register reads"),
    ("expected_bno" in recorder and "expected_icm" in recorder and
     "kSessionLossBnoRead" in recorder and "kSessionLossIcmRead" in recorder,
     "Master SessionHeader must truthfully expose acquisition shortfall"),
]

failures = [message for ok, message in checks if not ok]
if failures:
    for failure in failures:
        print(f"ERROR: {failure}", file=sys.stderr)
    raise SystemExit(1)
print("acquisition demo invariants passed")