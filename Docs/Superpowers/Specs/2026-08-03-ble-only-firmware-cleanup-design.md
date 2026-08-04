# BLE-Only Firmware Cleanup Design

## Goal

Create a hardware-untested cleanup branch that preserves the current BLE sensing, recording, transfer, SD, CSV, and browser behavior while removing verified RS485-era dead code, obsolete compatibility wrappers, stale naming, and redundant diagnostics.

## Branch and safety boundary

- Base: `2nd_Branch` at `3c1137d50cb464cb678882f92d39d7722fa148b6`.
- Working branch: `ble-only-firmware-cleanup`.
- Do not modify `main` or `2nd_Branch`.
- Keep the cleanup pull request draft and unmerged until Master and Node STM32 builds and physical regression tests are available.
- Preserve all STM32CubeMX-generated sections and peripheral initialization unless a generated item is proven removable and separately regenerated in CubeMX.

## Approved scope

The cleanup uses a moderate strategy:

1. Apply the uploaded BLE-only patch where it matches the current branch.
2. Remove RS485 headers and integration notes that have no remaining consumers.
3. Rename remaining RS485-era runtime symbols to BLE/leaf terminology.
4. Remove no-op RS485 UART helpers and the Node `BleOnlyNodeResponder` shim.
5. Remove disabled RS485 feature flags and callbacks whose only purpose was the deleted transport.
6. Retain generic UART HAL callbacks required by generated HAL integration, but leave them transport-neutral.
7. Remove redundant includes and obsolete wrappers after repository-wide reference checks.
8. Update diagnostics, tests, and architecture documentation to BLE-only terminology.

## Preserved behavior

The cleanup must not change:

- Master source ID `0` and Node IDs `1` through `4`.
- Browser-to-Master BLE peripheral behavior.
- Master-to-Node BLE central behavior.
- Four-Node live preview fairness and adaptive preview cadence.
- Local full-rate recording on Master SD and Node flash.
- Master-owned reliable Node transfer, ACK/NACK recovery, CRC validation, and sequential upload order.
- Training CSV schema version 2 and final `.OK` durability semantics.
- Node persistent ID storage.
- Sensor buses, GPIO assignments, flash/SD interfaces, or power-control behavior.

## Code organization

- `Firmware/Master/Core/Src/main.c` uses `leaf_ble_manager` terminology and contains no RS485 transport setup or recovery stubs.
- `Firmware/Node/Core/Src/main.c` tracks BLE stream state directly and contains no RS485 responder shim or RS485 mode flag.
- `ACQUISITION_DIAGNOSTICS.h` reports `comms_leaf` instead of `comms_rs485`.
- Deleted RS485 implementation headers are not replaced by empty compatibility files.
- BLE bridge declarations remain local to their actual users instead of requiring the obsolete `RECORDING_BRIDGE.h` wrapper.

## Verification

Without hardware, completion requires:

- Immutable compressed patch SHA-256 is recorded and validated as artifact digest completion criteria.
- Trusted per-file preimage and target blob IDs are validated as hash manifest completion criteria.
- Active firmware source searches (explicitly scoped to active firmware sources, not repository-wide) find no references to deleted headers RS485.h, RS485_FRAME_PROTOCOL.h, RS485_RECORD_MASTER_APP.h, RS485_RECORD_NODE_APP.h, and RECORDING_BRIDGE.h (including their include/reference forms), or obsolete runtime identifiers, with explicit historical allowlist for patch and documentation artifacts.
- Existing host tests and cleanup source tests pass where the required host toolchain is available.
- C/C++ syntax/static checks run against changed portable headers and source-level invariants.
- The branch diff is reviewed for accidental generated-code changes, protocol changes, or sensor/recording behavior changes.

## Hardware validation boundary

The branch remains hardware-untested. Before merge, build and flash Master and Node firmware, verify four Node connections and live graphs, run a complete recording, validate all five binary files and the schema-v2 CSV, and confirm browser-disconnect transfer recovery.