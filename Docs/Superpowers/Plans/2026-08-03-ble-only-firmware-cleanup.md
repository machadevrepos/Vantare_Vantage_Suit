# BLE-Only Firmware Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove verified RS485-era dead code and stale compatibility layers while preserving all current BLE, recording, transfer, sensor, SD, and CSV behavior.

**Architecture:** Apply the uploaded cleanup patch only after confirming its preimage hashes match the branch. Keep generated STM32 code intact, replace transport-era names with BLE/leaf names inside USER CODE regions, delete unreferenced headers atomically, and add source-level checks that prevent RS485 code from returning.

**Tech Stack:** STM32CubeWB firmware, C/C++, BLE central/peripheral APIs, FatFs, PowerShell source tests, Python static checks, GitHub branches and draft pull requests.

## Global Constraints

- Modify only `ble-only-firmware-cleanup`.
- Preserve protocol formats, session formats, sensor rates, GPIO mappings, and generated CubeMX sections.
- No claim of hardware validation.
- Every deletion requires a repository-wide consumer check.
- Keep commits reviewable and the PR draft.

---

### Task 1: Validate and apply the uploaded BLE-only patch

**Files:**
- Modify: `Firmware/Master/Core/Src/main.c`
- Modify: `Firmware/Node/Core/Src/main.c`
- Modify: `Firmware/LIBRARY/CUSTOM/ACQUISITION_DIAGNOSTICS.h`
- Modify: `Firmware/Master/tests/test_acquisition_diagnostics_source.ps1`
- Delete: `Firmware/LIBRARY/CUSTOM/RS485.h`
- Delete: `Firmware/LIBRARY/CUSTOM/RS485_FRAME_PROTOCOL.h`
- Delete: `Firmware/LIBRARY/CUSTOM/RS485_RECORD_MASTER_APP.h`
- Delete: `Firmware/LIBRARY/CUSTOM/RS485_RECORD_NODE_APP.h`
- Delete: `Firmware/LIBRARY/CUSTOM/RECORDING_BRIDGE.h`
- Delete: `Firmware/LIBRARY/CUSTOM/RECORDING_INTEGRATION_NOTES.md`

**Interfaces:**
- Consumes: existing BLE bridge functions `exo_hub_ble_write()` and `exo_node_ble_write()`.
- Produces: `leaf_ble_manager` naming and `AcquisitionDiagnostics::comms_leaf`.

- [ ] Confirm the patch preimage blob IDs match the current branch files.
- [ ] Reconstruct and hash the patch target files; require the target hashes from the patch headers.
- [ ] Apply the patch changes atomically on the cleanup branch.
- [ ] Verify no deleted header remains included or referenced.
- [ ] Commit with `chore: remove RS485 dead code and rename BLE leaf paths`.

### Task 2: Rewrite architecture documentation for the actual BLE-only system

**Files:**
- Replace: `Firmware/Project Details.md`

**Interfaces:**
- Consumes: current Master peripheral-to-browser and central-to-Node design.
- Produces: concise BLE-only architecture documentation covering live preview and recorded data paths.

- [ ] Replace stale RS485 descriptions with Browser ↔ Master BLE and Master ↔ Node BLE flows.
- [ ] Document source IDs, full-rate local recording, adaptive preview, sequential upload, CRC validation, and CSV v2.
- [ ] Remove obsolete manual integration notes already superseded by the current firmware.
- [ ] Commit with `docs: document BLE-only firmware architecture`.

### Task 3: Add cleanup regression checks

**Files:**
- Create: `Firmware/HostTests/test_ble_only_cleanup.py`
- Modify: `Firmware/Master/tests/test_acquisition_diagnostics_source.ps1`

**Interfaces:**
- Consumes: repository source tree.
- Produces: a zero-dependency source check that fails on deleted includes, RS485 runtime identifiers, stale diagnostics names, or missing BLE leaf names.

- [ ] Add assertions that deleted files do not exist.
- [ ] Scan active firmware sources for `RS485_RECORD_`, `MasterRs485_`, `HubRs485`, `master_rs485_recording`, `node_rs485_recording`, and `comms_rs485`.
- [ ] Permit historical mentions only inside the uploaded patch or cleanup documentation when explicitly excluded from the active-source scan.
- [ ] Require `leaf_ble_manager` and `comms_leaf` in the expected active files.
- [ ] Run `python Firmware/HostTests/test_ble_only_cleanup.py` and require PASS.
- [ ] Commit with `test: guard BLE-only firmware cleanup`.

### Task 4: Moderate dead-code and redundancy audit

**Files:**
- Modify only files identified by reference checks.

**Interfaces:**
- Consumes: symbol/include search results after Task 1.
- Produces: smaller active firmware without obsolete disabled wrappers.

- [ ] Search for unused compatibility headers, disabled feature flags, no-op wrappers, duplicate declarations, and inactive diagnostic branches.
- [ ] Remove only items with no active consumer and no generated-code ownership.
- [ ] Preserve UART initialization and generated callbacks unless CubeMX regeneration is available.
- [ ] Run the cleanup regression check after every removal batch.
- [ ] Commit each independently reviewable cleanup batch.

### Task 5: Final verification and draft PR

**Files:**
- Update: draft PR description.

**Interfaces:**
- Produces: an auditable, hardware-untested cleanup branch.

- [ ] Run Python syntax checks and BLE-only cleanup regression checks.
- [ ] Run available PowerShell and C++ host tests.
- [ ] Compare the cleanup branch against `2nd_Branch` and inspect every changed file.
- [ ] Verify `main` and `2nd_Branch` are unchanged.
- [ ] Create a draft PR from `ble-only-firmware-cleanup` to `2nd_Branch` with hardware validation requirements.
- [ ] Record exact passed checks and unresolved STM32/hardware validation boundaries.