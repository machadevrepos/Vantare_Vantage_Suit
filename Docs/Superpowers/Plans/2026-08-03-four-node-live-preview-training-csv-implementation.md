# Four-Node Live Preview and Training CSV Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Support one Master and Nodes 1–4 with non-starving live graphs during recording, autonomous recoverable Node uploads, and a schema-v2 training CSV containing raw measurements, deterministic derived features, timing, and capture-quality metadata.

**Architecture:** Keep acquisition/recording lossless and independent from preview. Nodes and the Master use bounded latest-value preview slots; the Master drains Node sources in strict round-robin order while retaining the existing BLE V2 frames understood by the validated HTML dashboard. After stop, the Master receives Nodes sequentially, owns reliable ACK/NACK/Verify control, validates each SD binary, and converts all expected sources into one CSV.

**Tech Stack:** STM32WB55 C/C++ firmware, BLEPipe, FatFs, header-only C++ helpers, browser Web Bluetooth/Canvas dashboard, host tests compiled with `g++ -std=c++17 -Wall -Wextra -Werror`.

## Global Constraints

- Modify `2nd_Branch` only; do not merge `main`.
- Keep pull request #1 draft until Master/Node STM32 builds and the four-PCB test pass.
- Preserve STM32-generated sections outside `USER CODE` blocks.
- Source IDs are Master `0`, Nodes `1–4`; a complete four-Node mask is `0x1F`.
- Preview loss/coalescing must never mutate recording buffers or stored sample counts.
- Retain existing BLE V2 browser frames so the currently validated five-source HTML remains compatible.
- Store full-rate binary data as the source of truth; derived CSV columns supplement raw columns.
- Create `.OK` only when every expected source is CRC-valid and committed.

---

### Task 1: Adaptive latest-value Node preview queue

**Files:**
- Modify: `Firmware/LIBRARY/CUSTOM/NODE_LIVE_SAMPLE_QUEUE.h`
- Modify: `Firmware/LIBRARY/CUSTOM/NODE_RECORDING_APP.h`
- Create: `Firmware/HostTests/test_node_live_sample_queue.cpp`

**Interfaces:**
- Consumes: `NodeLiveSample<MaxPayload>`, acquisition timestamps supplied by `NodeRecordingApp`.
- Produces: unchanged `configure`, `offer`, `peek`, `discard_front`, and `pop` API; adds `coalesced()`, `decimated()`, `congested()`, and `effective_interval_ms()` diagnostics.

- [ ] **Step 1: Write the failing queue test**

Create a test that offers BNO85 and ICM45686 samples at high rate, verifies latest-value replacement, strict sensor fairness, 40 ms normal gating, transition to 80 ms after repeated unsent replacements, and return to normal after five stable seconds.

- [ ] **Step 2: Run the test and confirm failure**

Run:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -IFirmware/LIBRARY/CUSTOM Firmware/HostTests/test_node_live_sample_queue.cpp -o /tmp/test_node_live_sample_queue && /tmp/test_node_live_sample_queue
```

Expected: compilation or assertion failure because the new diagnostics and fixed-slot behavior do not exist.

- [ ] **Step 3: Replace the FIFO with two fixed latest-value slots**

Use one slot per sensor. Overwriting a valid unsent slot increments `coalesced_`; samples rejected by the time gate increment `decimated_`. `peek`/`discard_front` alternate sensor preference whenever both slots are ready.

- [ ] **Step 4: Add adaptive interval behavior**

Normal interval is clamped to 40–80 ms. Four coalescing events inside one second enter 80 ms congested mode. Five seconds without another coalescing event restores the configured normal interval.

- [ ] **Step 5: Clamp the Node recording app preview interval**

`NodeRecordingApp::set_live_interval_ms` stores the queue’s effective normal interval rather than allowing the browser’s legacy 20 ms request to drive four Nodes at excessive notification rates.

- [ ] **Step 6: Run the queue test**

Expected: `node live sample queue tests passed`.

- [ ] **Step 7: Commit**

```bash
git add Firmware/LIBRARY/CUSTOM/NODE_LIVE_SAMPLE_QUEUE.h Firmware/LIBRARY/CUSTOM/NODE_RECORDING_APP.h Firmware/HostTests/test_node_live_sample_queue.cpp
git commit -m "feat: add adaptive node live preview queue"
```

### Task 2: Four-Node fair Master preview scheduler

**Files:**
- Modify: `Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h`
- Modify: `Firmware/HostTests/test_hub_leaf_ble_manager.cpp`

**Interfaces:**
- Consumes: `push_leaf_sample(node_id, sensor_id, payload, payload_len)` from the central client bridge.
- Produces: unchanged `peek_next_live_sample`, `discard_next_live_sample`, and `pop_next_live_sample`; adds per-pair coalescing counters and deterministic Node1→Node2→Node3→Node4 service.

- [ ] **Step 1: Extend the failing manager test**

Populate both sensors for Nodes 1–4, flood Node 1, then assert the first eight pops contain every `(node,sensor)` pair and that no Node is emitted twice while another ready Node is waiting. Queue RecordDone in reverse order and assert transfer order 1,2,3,4.

- [ ] **Step 2: Run and confirm the current FIFO ordering fails the strict fairness assertion**

```bash
g++ -std=c++17 -Wall -Wextra -Werror -IFirmware/LIBRARY/CUSTOM -IFirmware/Master/Core/Inc Firmware/HostTests/test_hub_leaf_ble_manager.cpp -o /tmp/test_hub_leaf && /tmp/test_hub_leaf
```

- [ ] **Step 3: Implement fixed source/sensor slots**

Store eight latest-value slots indexed by Node 1–4 and BNO/ICM. Replacements never evict another pair. Track a source cursor and per-source sensor preference.

- [ ] **Step 4: Implement strict round-robin selection**

Each successful discard advances the source cursor. Inactive sources are skipped. When both sensors are pending for one source, alternate them across visits.

- [ ] **Step 5: Preserve recording-transfer state and topology APIs**

Keep deterministic RecordDone ordering and all existing ACK/NACK/pause/verify getters intact.

- [ ] **Step 6: Run the manager test**

Expected: `hub leaf manager tests passed`.

- [ ] **Step 7: Commit**

```bash
git add Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h Firmware/HostTests/test_hub_leaf_ble_manager.cpp
git commit -m "feat: schedule four-node preview fairly"
```

### Task 3: Master-owned reliable Node transfer control

**Files:**
- Create: `Firmware/Master/Core/Inc/MASTER_NODE_RELIABLE_CONTROL.h`
- Modify: `Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h`
- Create: `Firmware/HostTests/test_master_node_reliable_control.cpp`
- Modify: `Firmware/HostTests/test_master_node_transfer_window.cpp`

**Interfaces:**
- Consumes: `RecordDoneMessage`, `RecordReliableFrameHeader`, `MasterNodeTransferWindow`, and a transport callback matching `bool send(uint8_t node_id, const uint8_t *frame, uint16_t length)`.
- Produces: `begin`, `ack_window`, `nack_range`, `verify_ok`, and `service` operations; coordinator getters for next expected chunk and control-send diagnostics.

- [ ] **Step 1: Write failing control-frame tests**

Verify packed ManifestAck, AckWindow, NackRange, and VerifyOk frames have protocol version 6, correct source/session fields, payload CRC16, and retry after a simulated busy transport.

- [ ] **Step 2: Extend transfer-window tests**

Verify a forward gap requests the current expected chunk, a corrupt chunk requests itself, duplicate chunks do not advance state, and a final contiguous chunk completes exactly at total size.

- [ ] **Step 3: Implement the reliable-control helper**

Use a bounded pending frame buffer. ACK windows are coalesced to the newest next-chunk value; NACK and Verify controls take priority. The default firmware transport weakly calls `exo_hub_central_client_send_blepipe_to_node(..., BLEPIPE_MSG_COMMAND, BLEPIPE_ID_HUB, ...)`; host tests inject a fake sender.

- [ ] **Step 4: Wire the transfer window into the coordinator**

On RecordDone, begin staging/window/control and send ManifestAck. On each chunk, inspect before writing. Duplicate → ACK current position; gap/corrupt → NACK without entering StageError; accepted → write, commit window, ACK; complete → validate.

- [ ] **Step 5: Send VerifyOk only after staged binary validation succeeds**

The Node’s local flash remains authoritative until the Master has closed the write file and verified header and payload CRCs from SD.

- [ ] **Step 6: Run control/window/coordinator host tests**

Expected: all tests pass; no gap path changes the coordinator to `StageError` unless FatFs itself fails.

- [ ] **Step 7: Commit**

```bash
git add Firmware/Master/Core/Inc/MASTER_NODE_RELIABLE_CONTROL.h Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h Firmware/HostTests/test_master_node_reliable_control.cpp Firmware/HostTests/test_master_node_transfer_window.cpp
git commit -m "feat: make master own node upload control"
```

### Task 4: Training CSV schema version 2

**Files:**
- Modify: `Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_FORMATTER.h`
- Modify: `Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h`
- Modify: `Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h`
- Create: `Firmware/HostTests/test_master_training_csv_formatter_v2.cpp`

**Interfaces:**
- Consumes: `SessionHeader`, BNO85 quaternion/vector data, ICM45686 raw data, source/sensor timestamp history.
- Produces: `TrainingCsvSourceMetadata`, `TrainingCsvRowContext`, schema-v2 BNO/ICM rows, and `MasterTrainingCsvLogger::set_source_metadata`.

- [ ] **Step 1: Write failing formatter tests**

Check the exact header column count, identity and 90-degree quaternion Euler vectors, acceleration/gyro magnitudes, first-row empty timing fields, monotonic delta/rate fields, non-monotonic timestamp quality flag, empty unavailable values, and sources 0–4 labels.

- [ ] **Step 2: Run and confirm schema-v1 failure**

```bash
g++ -std=c++17 -Wall -Wextra -Werror -IFirmware/LIBRARY/CUSTOM -IFirmware/Master/Core/Inc Firmware/HostTests/test_master_training_csv_formatter_v2.cpp -o /tmp/test_csv_v2 && /tmp/test_csv_v2
```

- [ ] **Step 3: Add schema-v2 common fields**

Add delta/rate, target/attempted/captured/dropped counts, loss flags, payload CRC, and timestamp-quality fields while retaining every schema-v1 raw field.

- [ ] **Step 4: Add deterministic derived features**

Use normalized quaternion `(x,y,z,w)` with right-handed intrinsic XYZ / roll-pitch-yaw formulas. Add Euler degrees and BNO/ICM vector magnitudes. Invalid quaternions produce empty Euler cells rather than fabricated zeroes.

- [ ] **Step 5: Track metadata and timestamps in the logger**

Store each source’s validated `SessionHeader`. Compute per-source/per-sensor delta and effective rate. Repeated or decreasing timestamps set a quality bit and leave rate fields empty.

- [ ] **Step 6: Wire Master and Node headers into the logger**

Set Master metadata after `logger.begin`; set Node metadata immediately after staged validation and before the first Node CSV row.

- [ ] **Step 7: Run formatter and coordinator tests**

Expected: all schema-v2 and existing completion tests pass.

- [ ] **Step 8: Commit**

```bash
git add Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_FORMATTER.h Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h Firmware/HostTests/test_master_training_csv_formatter_v2.cpp
git commit -m "feat: generate feature-rich training CSV v2"
```

### Task 5: Four-source validation and documentation

**Files:**
- Modify: `Firmware/HostTests/validate_training_csv.py`
- Create: `Docs/FourNode_Live_Csv_Validation.md`
- Modify: pull request #1 description

**Interfaces:**
- Consumes: completed CSV and SD directory listing.
- Produces: non-zero source/sensor coverage checks, schema-v2 required-column checks, timing/derived-feature sanity checks, and the physical acceptance procedure.

- [ ] **Step 1: Extend validator failure cases**

Require sources `0,1,2,3,4`, both sensors for each source, schema version 2, raw columns, derived columns, and finite derived values on representative valid rows.

- [ ] **Step 2: Run validator against the uploaded two-Node CSV**

Expected: schema/source failure is explicit because the baseline file is schema 1 and lacks Nodes 3–4.

- [ ] **Step 3: Add four-Node hardware procedure**

Document commissioning IDs 1–4, topology/ready mask checks, ten-second live/recording run, graph starvation limits, expected files, CRC checks, CSV validation command, and browser-disconnect resume test.

- [ ] **Step 4: Run all available host tests with warnings as errors**

Record exact commands and outputs. Do not claim STM32 compilation or hardware success unless performed.

- [ ] **Step 5: Update draft PR scope and validation boundary**

Keep the PR draft and unmerged.

- [ ] **Step 6: Commit**

```bash
git add Firmware/HostTests/validate_training_csv.py Docs/FourNode_Live_Csv_Validation.md
git commit -m "test: add four-node live and CSV validation"
```
