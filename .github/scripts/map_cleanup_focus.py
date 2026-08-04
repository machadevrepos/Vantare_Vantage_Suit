#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

ROOT = Path.cwd()
TARGETS = {
    'cleanup-map-master.txt': ROOT / 'Firmware/Master/Core/Src/main.c',
    'cleanup-map-node.txt': ROOT / 'Firmware/Node/Core/Src/main.c',
    'cleanup-map-diagnostics.txt': ROOT / 'Firmware/LIBRARY/CUSTOM/ACQUISITION_DIAGNOSTICS.h',
    'cleanup-map-diagnostics-test.txt': ROOT / 'Firmware/Master/tests/test_acquisition_diagnostics_source.ps1',
    'cleanup-map-project-details.txt': ROOT / 'Firmware/Project Details.md',
}
TERMS = [
    'rs485',
    'recording_bridge',
    'BleOnlyNodeResponder',
    'exo_hub_ble_write',
    'exo_node_ble_write',
    'node_stream_enabled',
    'uart',
]


def write_map(output_path: Path, source_path: Path) -> None:
    lines = source_path.read_text(encoding='utf-8').splitlines()
    matches = [
        index for index, line in enumerate(lines)
        if any(term.lower() in line.lower() for term in TERMS)
    ]
    output = [f'# {source_path.relative_to(ROOT)}', '']
    emitted: set[int] = set()
    for index in matches:
        start = max(0, index - 5)
        end = min(len(lines), index + 6)
        for line_index in range(start, end):
            if line_index in emitted:
                continue
            marker = '>' if line_index == index else ' '
            output.append(f'{marker}{line_index + 1:5d}: {lines[line_index]}')
            emitted.add(line_index)
        output.append('')
    output.append(f'Matches: {len(matches)}')
    output_path.write_text('\n'.join(output) + '\n')


def main() -> int:
    for output_name, source_path in TARGETS.items():
        write_map(ROOT / output_name, source_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
