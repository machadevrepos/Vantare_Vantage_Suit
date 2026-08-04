#!/usr/bin/env python3
from __future__ import annotations

import re
import shutil
from pathlib import Path

import apply_validated_firmware_cleanup as cleanup


PRESERVE = [
    'Docs/Superpowers/Plans/2026-08-03-ble-only-firmware-cleanup.md',
    'Docs/Superpowers/Specs/2026-08-03-ble-only-firmware-cleanup-design.md',
    'Firmware/HostTests/test_ble_only_cleanup.py',
]

DELETED_PATHS = [
    'Firmware/LIBRARY/CUSTOM/RECORDING_BRIDGE.h',
    'Firmware/LIBRARY/CUSTOM/RECORDING_INTEGRATION_NOTES.md',
    'Firmware/LIBRARY/CUSTOM/RS485.h',
    'Firmware/LIBRARY/CUSTOM/RS485_FRAME_PROTOCOL.h',
    'Firmware/LIBRARY/CUSTOM/RS485_RECORD_MASTER_APP.h',
    'Firmware/LIBRARY/CUSTOM/RS485_RECORD_NODE_APP.h',
]


def replace_once(path: str, old: str, new: str) -> None:
    target = cleanup.WORK / path
    text = target.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected one occurrence, found {count}: {old[:80]!r}')
    target.write_text(text.replace(old, new, 1))


def copy_preserved_files() -> None:
    for relative in PRESERVE:
        source = cleanup.REPO / relative
        if not source.exists():
            raise RuntimeError(f'missing preserved cleanup contract: {relative}')
        destination = cleanup.WORK / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def clean_master_main() -> None:
    path = cleanup.WORK / 'Firmware/Master/Core/Src/main.c'
    text = path.read_text()

    required = {
        '#include <RECORDING_BRIDGE.h>': 1,
        'master_rs485_recording': 1,
        'comms_rs485': 1,
    }
    for token, minimum in required.items():
        if text.count(token) < minimum:
            raise RuntimeError(f'Master main.c: expected token not found: {token}')

    text = text.replace('#include <RECORDING_BRIDGE.h>\n', '', 1)
    text = text.replace(
        'static bool HubRs485BleSend(const uint8_t *payload, uint8_t length);\n'
        'static void HubRs485TxDone();\n'
        'static exo::ble_hub::HubLeafBleManager master_rs485_recording;\n',
        'static exo::ble_hub::HubLeafBleManager leaf_ble_manager;\n',
        1,
    )
    text = text.replace('static void MasterRs485_StartUartDmaRx();\n', '', 1)

    dead_bridge = re.compile(
        r'\nstatic bool HubRs485BleSend\(const uint8_t \*payload, uint8_t length\)\n'
        r'\s*\{\n\s*\(void\) payload;\n\s*\(void\) length;\n\s*return true;\n\}\n\n'
        r'static void HubRs485TxDone\(\)\n\{\n\}\n\n'
        r'static void MasterRs485_StartUartDmaRx\(\)\n\{\n'
        r'\s*/\* RS485 removed in BLE-only architecture\. \*/\n\}\n',
        re.MULTILINE,
    )
    text, count = dead_bridge.subn('\n', text, count=1)
    if count != 1:
        raise RuntimeError('Master main.c: dead BLE/UART compatibility block not found')

    recover = re.compile(
        r'\n\tstatic void MasterRs485_RecoverUartRx\(\)\n\t\{\n'
        r'\t\t/\* RS485 removed in BLE-only architecture\. \*/\n\t\}\n',
        re.MULTILINE,
    )
    text, count = recover.subn('\n', text, count=1)
    if count != 1:
        raise RuntimeError('Master main.c: dead UART recovery block not found')

    text = re.sub(
        r'^\s*MasterRs485_(?:StartUartDmaRx|RecoverUartRx)\(\);\s*\n',
        '',
        text,
        flags=re.MULTILINE,
    )

    text = text.replace('master_rs485_recording', 'leaf_ble_manager')
    text = text.replace('comms_rs485', 'comms_leaf')
    text = text.replace('NodeUartRetx', 'NodeTransferRetx')
    text = text.replace('rs485_max_us', 'leaf_max_us')
    text = text.replace('rs485_gt5', 'leaf_gt5')

    forbidden = [
        'MasterRs485_', 'HubRs485', 'master_rs485_recording',
        'comms_rs485', 'NodeUartRetx', 'RECORDING_BRIDGE.h',
    ]
    leftovers = [token for token in forbidden if token in text]
    if leftovers:
        raise RuntimeError(f'Master main.c: stale identifiers remain: {leftovers}')

    rs485_lines = [
        f'{index + 1}: {line}'
        for index, line in enumerate(text.splitlines())
        if 'rs485' in line.lower()
    ]
    if rs485_lines:
        raise RuntimeError('Master main.c: unclassified RS485 text remains:\n' + '\n'.join(rs485_lines))

    path.write_text(text)


def clean_node_main() -> None:
    path = cleanup.WORK / 'Firmware/Node/Core/Src/main.c'
    text = path.read_text()

    text = text.replace('#include <RECORDING_BRIDGE.h>\n', '', 1)

    mode_block = (
        '#ifndef EXO_NODE_UART_RS485_MODE\n'
        '#define EXO_NODE_UART_RS485_MODE 0\n'
        '#endif\n\n'
    )
    if text.count(mode_block) != 1:
        raise RuntimeError('Node main.c: disabled RS485 mode block not found')
    text = text.replace(mode_block, '', 1)

    responder = re.compile(
        r'namespace \{\nclass BleOnlyNodeResponder \{.*?\n\};\n\}\n'
        r'static BleOnlyNodeResponder node_rs485_recording;\n',
        re.DOTALL,
    )
    replacement = (
        'static bool node_stream_enabled = false;\n'
        'static uint8_t node_stream_interval_ms = 20U;\n'
    )
    text, count = responder.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError('Node main.c: BLE-only responder scaffold not found')

    text = text.replace('node_rs485_recording.stream_enabled()', 'node_stream_enabled')
    text = text.replace('node_rs485_recording.stream_interval_ms()', 'node_stream_interval_ms')
    text = text.replace('node_rs485_recording.set_stream_enabled(true);', 'node_stream_enabled = true;')
    text = text.replace('node_rs485_recording.set_stream_enabled(false);', 'node_stream_enabled = false;')
    text = text.replace(
        'node_rs485_recording.set_stream_interval_ms(payload[1]);',
        'node_stream_interval_ms = payload[1] == 0U ? 1U : payload[1];',
    )

    text = re.sub(
        r'^\s*node_rs485_recording\.(?:set_node_id\([^\n;]*\)|begin\(\)|process\(\));\s*\n',
        '',
        text,
        flags=re.MULTILINE,
    )

    callback_replacements = {
        re.compile(
            r'extern "C" void HAL_UART_RxCpltCallback\(UART_HandleTypeDef \*huart\)\n'
            r'\{\n\s*if \(huart == &hlpuart1\) \{\n'
            r'\s*node_rs485_recording\.on_uart_rx_byte\(\);\n\s*\}\n\}',
            re.MULTILINE,
        ): (
            'extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)\n'
            '{\n\t(void)huart;\n}'
        ),
        re.compile(
            r'extern "C" void HAL_UARTEx_RxEventCallback\(UART_HandleTypeDef \*huart, uint16_t Size\)\n'
            r'\{\n\s*if \(huart == &hlpuart1\) \{\n'
            r'\s*node_rs485_recording\.on_uart_rx_idle_event\(Size\);\n\s*\}\n\}',
            re.MULTILINE,
        ): (
            'extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)\n'
            '{\n\t(void)huart;\n\t(void)Size;\n}'
        ),
        re.compile(
            r'extern "C" void HAL_UART_ErrorCallback\(UART_HandleTypeDef \*huart\)\n'
            r'\{\n\s*if \(huart == &hlpuart1\) \{\n'
            r'\s*node_rs485_recording\.on_uart_error\(\);\n\s*\}\n\}',
            re.MULTILINE,
        ): (
            'extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)\n'
            '{\n\t(void)huart;\n}'
        ),
    }
    for pattern, new in callback_replacements.items():
        text, count = pattern.subn(new, text, count=1)
        if count != 1:
            raise RuntimeError(f'Node main.c: UART callback block not found: {pattern.pattern[:60]}')

    text = text.replace('rs485_rx_cfg', 'lpuart_rx_cfg')
    text = text.replace(
        'UART RS485 transport: removed, BLE-only mode active',
        'UART callbacks idle; BLE-only transport active',
    )

    forbidden = [
        'BleOnlyNodeResponder', 'node_rs485_recording',
        'EXO_NODE_UART_RS485_MODE', 'RECORDING_BRIDGE.h',
    ]
    leftovers = [token for token in forbidden if token in text]
    if leftovers:
        raise RuntimeError(f'Node main.c: stale identifiers remain: {leftovers}')
    if 'node_stream_enabled' not in text or 'node_stream_interval_ms' not in text:
        raise RuntimeError('Node main.c: direct BLE stream state was not installed')

    rs485_lines = [
        f'{index + 1}: {line}'
        for index, line in enumerate(text.splitlines())
        if 'rs485' in line.lower()
    ]
    if rs485_lines:
        raise RuntimeError('Node main.c: unclassified RS485 text remains:\n' + '\n'.join(rs485_lines))

    path.write_text(text)


def clean_diagnostics_and_tests() -> None:
    replace_once(
        'Firmware/LIBRARY/CUSTOM/ACQUISITION_DIAGNOSTICS.h',
        '    LatencyStat comms_rs485;\n',
        '    LatencyStat comms_leaf;\n',
    )

    test_path = cleanup.WORK / 'Firmware/Master/tests/test_acquisition_diagnostics_source.ps1'
    test = test_path.read_text()
    old_pair = "        @('comms_rs485', 'master_rs485_recording\\.process\\(\\)'),"
    new_pair = "        @('comms_leaf', 'leaf_ble_manager\\.process\\(\\)'),"
    if test.count(old_pair) != 1:
        raise RuntimeError('diagnostics source test: RS485 timing pair not found')
    test = test.replace(old_pair, new_pair, 1)
    test = test.replace('master_rs485_recording\\.process\\(\\)', 'leaf_ble_manager\\.process\\(\\)')
    if 'comms_rs485' in test or 'master_rs485_recording' in test:
        raise RuntimeError('diagnostics source test: stale RS485 contract remains')
    test_path.write_text(test)

    includer = cleanup.WORK / 'Firmware/LIBRARY/CUSTOM/INCLUDER.h'
    includer_text = includer.read_text()
    commented = (
        '//#include <RS485.h>\n'
        '//DIG_PIN rs485_1_rede(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, 1);\n'
        '//DIG_PIN rs485_2_rede(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, 1);\n\n'
        '//RS485 rs485_1(&huart1, rs485_1_rede, USART1_IRQn);\n'
    )
    if commented in includer_text:
        includer_text = includer_text.replace(commented, '', 1)
    else:
        includer_text = re.sub(
            r'^//\s*#include <RS485\.h>\n(?:^//.*rs485.*\n){1,5}',
            '',
            includer_text,
            count=1,
            flags=re.MULTILINE | re.IGNORECASE,
        )
    includer.write_text(includer_text)


def rewrite_project_details() -> None:
    path = cleanup.WORK / 'Firmware/Project Details.md'
    path.write_text('''# Vantare Vantage Suit Firmware Architecture

## System roles

The firmware uses one **Master** and up to four commissioned **Nodes**.

- Source ID `0` is the Master.
- Source IDs `1` through `4` are Nodes.
- The browser communicates with the Master over the Master BLE peripheral link.
- The Master communicates with Nodes over BLE central links.
- No active RS485 recording or streaming transport remains.

## Live preview

Each device samples its local BNO85 and ICM45686 sensors. Nodes retain the latest pending value per sensor and forward preview samples to the Master. The Master services Node/sensor slots fairly and forwards the five-source BLE V2 stream to the browser. Recording remains independent of preview delivery.

The Master uses a shared preview pacing gate: 40 ms under normal conditions and 80 ms after BLE backpressure. A failed latest-value sample is retained, other ready sources remain eligible, and normal cadence resumes after a stable recovery period.

## Recording and transfer

The Master records its own full-rate sensor data to SD. Each Node records full-rate sensor data to local flash. At recording completion, the Master owns reliable sequential collection from Nodes 1 through 4 using manifests, ACK windows, NACK ranges, CRC validation, and VerifyOk completion. Browser connectivity is not required for an active Node upload to progress.

Validated Node binaries remain on Master SD before conversion. Transfer ownership is not replaced until the active source completes verification.

## Training CSV version 2

The Master converts validated source binaries into a single training CSV containing raw sensor values, source counts, loss indicators, payload CRCs, per-sensor sequence and timing fields, timestamp-quality flags, quaternion-derived roll/pitch/yaw using the intrinsic Z-Y-X yaw-pitch-roll convention, and vector magnitudes.

A `.TMP` file is renamed to `.CSV` only after all expected sources are committed. The `.OK` marker is created, synchronized, and closed before the session is reported as published.

## Hardware boundary

STM32CubeMX-generated initialization, sensor buses, GPIO assignments, flash/SD interfaces, power controls, and generic HAL callback signatures remain unchanged by the BLE-only cleanup. Both STM32CubeIDE projects and the complete Master/four-Node workflow must be validated on hardware before this cleanup branch is merged.
''')


def delete_obsolete_files() -> None:
    for relative in DELETED_PATHS:
        target = cleanup.WORK / relative
        if not target.exists():
            raise RuntimeError(f'approved deletion path missing from base: {relative}')
        target.unlink()


def verify_manual_cleanup() -> None:
    active_roots = [
        cleanup.WORK / 'Firmware/Master/Core',
        cleanup.WORK / 'Firmware/Master/STM32_WPAN',
        cleanup.WORK / 'Firmware/Node/Core',
        cleanup.WORK / 'Firmware/Node/STM32_WPAN',
        cleanup.WORK / 'Firmware/LIBRARY/CUSTOM',
    ]
    forbidden = [
        'MasterRs485_', 'HubRs485', 'master_rs485_recording',
        'node_rs485_recording', 'comms_rs485', 'RS485_RECORD_',
        'EXO_NODE_UART_RS485_MODE', '#include <RS485.h>',
        '#include "RS485.h"', 'RECORDING_BRIDGE.h',
        'RS485_FRAME_PROTOCOL.h', 'RS485_RECORD_MASTER_APP.h',
        'RS485_RECORD_NODE_APP.h',
    ]
    failures: list[str] = []
    for root in active_roots:
        for path in root.rglob('*'):
            if not path.is_file() or path.suffix.lower() not in {'.c', '.cc', '.cpp', '.h', '.hpp', '.inc'}:
                continue
            try:
                text = path.read_text(encoding='utf-8')
            except UnicodeDecodeError:
                continue
            for token in forbidden:
                if token in text:
                    failures.append(f'{path.relative_to(cleanup.WORK)}: stale {token}')
    for relative in DELETED_PATHS:
        if (cleanup.WORK / relative).exists():
            failures.append(f'{relative}: file still exists')
    if failures:
        raise RuntimeError('manual cleanup verification failed:\n' + '\n'.join(failures))


def remove_transport_and_mapping_files() -> None:
    cleanup.remove_transport_machinery()
    for relative in [
        '.github/workflows/execute-manual-validated-cleanup.yml',
        '.github/workflows/execute-validated-cleanup-v2.yml',
        '.github/workflows/actions-probe.yml',
        '.github/scripts/materialize_validated_firmware_cleanup.py',
        '.github/scripts/prepare_executor_v2.py',
        '.github/scripts/run_validated_cleanup_v4.py',
        '.github/scripts/run_validated_cleanup_v5.py',
        '.github/scripts/run_validated_cleanup_v6.py',
        '.github/scripts/run_manual_validated_cleanup.py',
        '.github/scripts/map_cleanup_sources.py',
        '.github/scripts/map_cleanup_focus.py',
        'actions-probe.txt',
        'cleanup-executor.log',
        'cleanup-source-map.txt',
        'cleanup-map-master.txt',
        'cleanup-map-node.txt',
        'cleanup-map-diagnostics.txt',
        'cleanup-map-diagnostics-test.txt',
        'cleanup-map-project-details.txt',
    ]:
        target = cleanup.WORK / relative
        if target.is_dir() and not target.is_symlink():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()
    for relative in ['.github/scripts', '.github/workflows', '.github']:
        directory = cleanup.WORK / relative
        if directory.exists() and directory.is_dir() and not any(directory.iterdir()):
            directory.rmdir()


def main() -> int:
    current_head = cleanup.output('git', 'rev-parse', 'HEAD')
    branch = cleanup.output('git', 'branch', '--show-current')
    if branch != 'ble-only-firmware-cleanup':
        raise RuntimeError(f'expected cleanup branch, got {branch!r}')

    cleanup.prepare_worktree(current_head)
    copy_preserved_files()
    clean_master_main()
    clean_node_main()
    clean_diagnostics_and_tests()
    rewrite_project_details()
    delete_obsolete_files()
    verify_manual_cleanup()

    cleanup.add_regression_tests()
    cleanup.verify_red()
    cleanup.apply_confirmed_fixes()
    cleanup.run_green_suite()
    verify_manual_cleanup()
    remove_transport_and_mapping_files()
    cleanup.run('git', 'diff', '--check', cwd=cleanup.WORK)

    commit = cleanup.commit_and_push(current_head)
    print(f'published manual validated cleanup commit {commit}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
