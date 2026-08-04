#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

ROOT = Path.cwd()
ACTIVE_ROOTS = [
    ROOT / 'Firmware/Master/Core',
    ROOT / 'Firmware/Master/STM32_WPAN',
    ROOT / 'Firmware/Node/Core',
    ROOT / 'Firmware/Node/STM32_WPAN',
    ROOT / 'Firmware/LIBRARY/CUSTOM',
]
SUFFIXES = {'.c', '.cc', '.cpp', '.h', '.hpp', '.inc'}
TERMS = [
    'MasterRs485_',
    'HubRs485',
    'master_rs485_recording',
    'node_rs485_recording',
    'comms_rs485',
    'RS485_RECORD_',
    'EXO_NODE_UART_RS485_MODE',
    'RECORDING_BRIDGE.h',
    'RS485.h',
    'RS485_FRAME_PROTOCOL.h',
    'RS485_RECORD_MASTER_APP.h',
    'RS485_RECORD_NODE_APP.h',
    'BleOnlyNodeResponder',
]


def files() -> list[Path]:
    result: list[Path] = []
    for root in ACTIVE_ROOTS:
        if root.exists():
            result.extend(
                path for path in root.rglob('*')
                if path.is_file() and path.suffix.lower() in SUFFIXES
            )
    return sorted(set(result))


def main() -> int:
    output: list[str] = ['# BLE cleanup active-source map', '']
    count = 0
    for path in files():
        try:
            lines = path.read_text(encoding='utf-8').splitlines()
        except UnicodeDecodeError:
            continue
        matches = [
            index for index, line in enumerate(lines)
            if any(term.lower() in line.lower() for term in TERMS)
        ]
        if not matches:
            continue
        output.append(f'## {path.relative_to(ROOT)}')
        emitted: set[int] = set()
        for index in matches:
            start = max(0, index - 4)
            end = min(len(lines), index + 5)
            for line_index in range(start, end):
                if line_index in emitted:
                    continue
                marker = '>' if line_index == index else ' '
                output.append(f'{marker}{line_index + 1:5d}: {lines[line_index]}')
                emitted.add(line_index)
            output.append('')
            count += 1
    output.append(f'Occurrences: {count}')
    (ROOT / 'cleanup-source-map.txt').write_text('\n'.join(output) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
