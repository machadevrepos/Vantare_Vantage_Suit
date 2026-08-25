# Node-to-Master Transfer Throughput Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make sequential Node-to-Master GATT artifact upload completion-driven, observable, and capable of at least 30 KiB/s per Node without weakening integrity.

**Architecture:** Keep the existing reliable Manifest/Chunk/ACK/NACK/Verify protocol. Commission each BLE link through a globally serialized completion-driven Master state machine, pace Node notifications from controller backpressure events, and keep raw artifact bytes on the Node-to-Master/SD plane while publishing compact browser telemetry.

**Tech Stack:** STM32WB55 C++17 firmware, STM32 WPAN HCI/ACI APIs, FatFs, header-only host-testable protocol components, PowerShell/CMake host tests, STM32CubeIDE headless builds.

**Spec:** Approved user plan in the Codex task dated 2026-08-25.

## Global Constraints

- Preserve wire compatibility for existing reliable record frames.
- Sequential collection remains the default: one active uploading Node.
- Reliability and recoverability outrank peak benchmark speed.
- Existing dirty changes in the source checkout are user-owned and must remain untouched.
- Link telemetry must distinguish unknown/requested/confirmed/degraded/failed.
- Never claim DLE, PHY, or interval from command acceptance; require completion evidence.

---

### Task 1: Completion-driven Master link commissioning

**Files:**
- Create: `firmware/common/inc/exo/ble/link_tune_state.h`
- Modify: `firmware/Master/Core/Src/ble/exo_hub_central_client.cpp`
- Modify: `firmware/Master/Core/Src/ble/app_ble.cpp`
- Test: `host/tests/cpp/test_link_tune_state.cpp`

**Interfaces:**
- Produces: a pure `exo::LinkTuneState` transition model with states `NeedDle`, `WaitDle`, `NeedPhy`, `WaitPhy`, `NeedInterval`, `WaitInterval`, `Ready`, `Degraded`, and `Failed`.
- Produces: one global Master HCI arbiter that issues at most one LL procedure globally and advances only from completion callbacks or classified timeout/retry transitions.
- Produces: actual per-link DLE octets, PHY, interval, preparation duration, retry/status, and state telemetry.

- [ ] Write host tests that fail against the missing state model for serialized four-link arbitration, transient retry, permanent error, missing-completion degradation, stale/disconnected completion rejection, and fast-to-slow generation ordering.
- [ ] Run the focused host test and confirm the expected missing-interface failures.
- [ ] Implement the minimal pure transition model and make the focused tests pass.
- [ ] Integrate it into the Master central client, preserving discovery-hold servicing and moving all DLE/PHY/interval requests behind one global arbiter.
- [ ] Forward data-length, PHY-update, and connection-update completion events into the model; report requested and confirmed values separately.
- [ ] Read/report controller maximum data-length capability at BLE startup.
- [ ] On fast preparation timeout, mark degraded and allow reliable collection; restore the slow interval only after source completion.
- [ ] Run host tests, firmware syntax checks, and Master Debug/Release builds.
- [ ] Commit the task.

### Task 2: Event-driven Node notification pump and DLE confirmation

**Files:**
- Create: `firmware/common/inc/exo/ble/node_upload_pump.h`
- Modify: `firmware/Node/Core/Src/main.cpp`
- Modify: `firmware/Node/Core/Src/ble/custom_app.cpp`
- Modify: `firmware/Node/Core/Src/ble/custom_stm.cpp`
- Modify: `firmware/Node/Core/Src/ble/app_ble.cpp`
- Modify: `firmware/Node/Core/Inc/app_conf.h`
- Test: `host/tests/cpp/test_node_upload_pump.cpp`

**Interfaces:**
- Produces: a pure upload-pump gate that runs while ready, blocks on BLE busy/insufficient resources, wakes on notification-complete/TX-pool-available, and uses a watchdog only for lost events.
- Produces: deferred Node DLE commissioning outside HCI callbacks plus Node-to-Master status telemetry for actual DLE outcome.

- [ ] Write failing tests for busy, insufficient resources, notification completion, TX-pool wakeup, watchdog recovery, credit exhaustion, and upload-time live-preview suppression.
- [ ] Run the focused test and confirm expected missing-interface failures.
- [ ] Implement the pump model and integrate foreground chunk pumping; remove normal 8 ms polling as the pacing source.
- [ ] Make callbacks set flags/counters only; perform flash reads and notifications from foreground context.
- [ ] Replace the connection-callback DLE request with deferred bounded commissioning and capture the data-length completion event.
- [ ] Report controller maximum length, negotiated octets, accepted bytes, busy/resource counts, completion/pool events, watchdog wakes, and flash-read timing once per second.
- [ ] Suppress live preview while upload is active.
- [ ] Size BLE pool extras for four complete 247-byte ATT notification buffers and confirm the Node RAM map.
- [ ] Run host tests, firmware syntax checks, and Node Debug/Release builds.
- [ ] Commit the task.

### Task 3: Master receive-plane isolation and effective flow control

**Files:**
- Modify: `firmware/common/inc/exo/protocol/master_training_csv_coordinator.h`
- Modify: `firmware/Master/Core/Src/main.cpp`
- Modify: `firmware/Master/Core/Src/ble/exo_hub_central_client.cpp`
- Test: `firmware/Master/tests/master_training_csv_coordinator_test.cpp`
- Test: `host/tests/cpp/test_master_node_session_stager_sequential.cpp`

**Interfaces:**
- Produces: coordinator setters for receiver credit, ACK chunk threshold, and ACK timeout with enforced `threshold < credit`.
- Produces: a Master ownership predicate used by the central client to suppress raw Manifest/Chunk browser relay while retaining compact progress/status.
- Produces: receive/ACK/queue/relay/SD aggregate telemetry.

- [ ] Write failing coordinator tests for credit/ACK sanitization, threshold-based ACK, timeout partial ACK, final/gap/duplicate immediate control, 24-entry pending capacity, and no synchronous staging when full.
- [ ] Run the focused tests and confirm expected behavioral failures.
- [ ] Expand the pending queue to 24, advertise no more credit than free queue capacity, and remove BLE-context SD fallback.
- [ ] Stop unconditional partial ACK flush during every queue drain; flush only for threshold, timeout/idle, final, gap, or duplicate.
- [ ] Wire runtime credit/ACK settings into the coordinator and echo effective values.
- [ ] Suppress raw Manifest/Chunk relay while the coordinator owns the Node link; retain compact progress, link state, errors, validation, and terminal outcome.
- [ ] Add counters for received chunks, queue high-water/overflow, ACK attempt/status, suppressed relay, SD flush count, and maximum duration.
- [ ] Run coordinator/stager tests, all host tests, and Master Debug/Release builds.
- [ ] Commit the task.

### Task 4: Browser truthfulness, integration verification, and benchmark handoff

**Files:**
- Modify: `host/desktop_tool/Exoskeleton.html`
- Modify: `docs/guides/four-node-live-csv-validation.md`
- Test: `host/tests/python/test_exoskeleton_contract.py`

**Interfaces:**
- Consumes: confirmed/degraded link state and effective transfer configuration from Tasks 1-3.
- Produces: operator-visible telemetry that never maps unknown zero values to confirmed defaults.

- [ ] Write failing UI contract tests for unknown/requested/confirmed/degraded/failed link wording and effective transfer configuration display.
- [ ] Update parsing/logging so unknown DLE is labeled unknown, confirmed values are explicit, and suppressed raw relay is represented by compact progress telemetry.
- [ ] Document the hardware benchmark matrix: intervals 15/11.25/7.5 ms; credits 8/16/24 with thresholds below credit; three realistic transfers per Node and three complete four-Node collections.
- [ ] Document acceptance gates: at least 30 KiB/s per Node, exact bytes/CRC, no stalls/overflow/unclassified errors, retransmits below 0.1%, responsive browser, and independent acquisition-quality gates.
- [ ] Run the UI contract test, full host suite, both firmware Debug/Release builds, and inspect RAM/flash maps.
- [ ] Commit the task.

## Conditional Gate

If hardware confirms DLE >=247, 2M PHY, and the selected fast interval but sustained payload rate remains below 30 KiB/s after the documented matrix, stop tuning constants. Design a separate capability-negotiated L2CAP CoC bulk lane for Manifest/Chunk while retaining GATT control/status and automatic GATT fallback.
