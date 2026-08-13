#!/usr/bin/env python3
"""Regression guards for cross-file firmware invariants repaired in 2nd_Branch."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
    failures: list[str] = []

    runtime_config = read("Firmware/LIBRARY/CUSTOM/NODE_RUNTIME_CONFIG.h")
    require(
        "static constexpr uint8_t kNodeIdMax = 4U;" in runtime_config,
        "Node runtime configuration must reject IDs above the supported 1..4 topology",
        failures,
    )

    node_main = read("Firmware/Node/Core/Src/main.c")
    require(
        "PWM_PIN ERM_PWM(&htim1, TIM_CHANNEL_4, ERM_GPIO_Port, ERM_Pin);" in node_main,
        "ERM_PWM must use generated TIM1_CH4 / ERM pin mapping",
        failures,
    )
    require(
        "PWM_PIN BUZZER(&htim1, TIM_CHANNEL_3, BUZZER_GPIO_Port, BUZZER_Pin);" in node_main,
        "BUZZER must use generated TIM1_CH3 / BUZZER pin mapping",
        failures,
    )

    pwm = read("Firmware/LIBRARY/CUSTOM/PWM_PIN.h")
    zero_start = pwm.find("if (percent == 0U)")
    zero_end = pwm.find("const uint64_t scaled_ticks", zero_start)
    zero_block = pwm[zero_start:zero_end] if zero_start >= 0 and zero_end >= 0 else ""
    require("SET_COMPARE(0U);" in zero_block, "0% PWM must write CCR=0 without unsigned underflow", failures)

    compare_pos = pwm.find("SET_COMPARE(ticks == 0U ? 0U : (ticks - 1U));")
    start_pos = pwm.find("START();", compare_pos)
    require(
        compare_pos >= 0 and start_pos > compare_pos,
        "PWM compare must be programmed before a stopped channel is started",
        failures,
    )

    custom_stm = read("Firmware/Master/STM32_WPAN/App/custom_stm.c")
    permit_case = custom_stm.find("case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:")
    permit_end = custom_stm.find("/* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */", permit_case)
    permit_block = custom_stm[permit_case:permit_end] if permit_case >= 0 and permit_end >= 0 else ""
    require(
        "CUSTOM_STM_PIPECTRLRX_WRITE_EVT" in permit_block,
        "ATT Write Request must dispatch the acknowledged-write opcode",
        failures,
    )
    require(
        "CUSTOM_STM_CMD_WRITE_NO_RESP_EVT" not in permit_block,
        "ATT Write Request must not be mislabeled as write-without-response",
        failures,
    )
    permit_pos = permit_block.find("aci_gatt_permit_write(")
    notify_pos = permit_block.find("Custom_STM_App_Notification(&Notification);")
    require(
        permit_pos >= 0 and notify_pos > permit_pos,
        "Master must send the ATT permit response before synchronous application dispatch",
        failures,
    )
    require(
        "permit_status == BLE_STATUS_SUCCESS" in permit_block,
        "Master must dispatch the accepted write only after permit success",
        failures,
    )

    desktop = read("Firmware/DesktopTools/Exoskeleton.html")
    fn_start = desktop.find("function writeCharacteristic(char, payload)")
    fn_end = desktop.find("function decodeTrainingCsvStatus", fn_start)
    write_block = desktop[fn_start:fn_end] if fn_start >= 0 and fn_end >= 0 else ""
    with_response = write_block.find("char.properties?.write")
    without_response = write_block.find("char.properties?.writeWithoutResponse")
    require(
        with_response >= 0 and without_response > with_response,
        "Desktop BLE helper must prefer write-with-response when both properties are available",
        failures,
    )

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1

    print("repair invariant guards passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
