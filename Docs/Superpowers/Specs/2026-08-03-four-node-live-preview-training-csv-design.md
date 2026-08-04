# Four-Node Live Preview and Training CSV Design

## Status

Approved architecture, pending review of this written specification before implementation.

## Objective

Scale the Vantage Suit to one Master and four Nodes while preserving two independent data products:

1. A smooth, low-latency BLE preview for continuous browser graphs during acquisition and recording.
2. A full-fidelity, recoverable recording from every source for model-training CSV generation.

The preview path is allowed to coalesce or decimate samples. The recording path is not.

## Validated Baseline

The completed two-Node hardware run produced `R0001M.BIN`, `R0001N1.BIN`, `R0001N2.BIN`, `TRN0001.CSV`, and `TRN0001.OK`, with no remaining `.TMP` file. The CSV contained both sensors for sources 0, 1, and 2. This establishes the current binary staging and CSV finalization pipeline as the baseline to extend rather than replace.

The same run also showed that post-recording Node uploads can contain many duplicate and recovery frames. The four-Node design therefore keeps transfers sequential and isolated from the preview scheduler.

## Source Model

| Source | ID | Role |
|---|---:|---|
| Master | 0 | BLE peripheral to browser and BLE central to Nodes |
| Node 1 | 1 | Sensor and local-flash recorder |
| Node 2 | 2 | Sensor and local-flash recorder |
| Node 3 | 3 | Sensor and local-flash recorder |
| Node 4 | 4 | Sensor and local-flash recorder |

A complete four-Node session uses expected source mask `0x1F`.

## Architecture

### 1. Full-fidelity acquisition and recording

Each source records its local sensors independently of BLE preview delivery.

- BNO85 samples are retained at the rate actually produced by the configured sensor.
- ICM45686 samples are retained at the configured acquisition rate, nominally 200 Hz.
- Every retained sample keeps its source ID, sensor ID, sequence, and session-relative timestamp.
- Nodes write to external flash while the Master writes its own session to SD.
- Preview congestion, preview drops, browser rendering delays, and browser disconnects must not reduce the stored sample set.

The binary session format remains the source of truth. CSV generation occurs only from CRC-validated binary artifacts.

### 2. Preview bundles

The live path uses a source-level preview bundle rather than forwarding every acquisition sample as an independent phone notification.

Each preview bundle contains:

- source ID;
- bundle sequence;
- BNO85 sample-present flag, sample sequence, acquisition timestamp, and latest payload;
- ICM45686 sample-present flag, sample sequence, acquisition timestamp, and latest payload;
- preview coalesced-count and dropped-count diagnostics;
- connection or sensor-health flags.

The bundle must fit within the existing 247-byte ATT MTU.

Nodes maintain one latest pending BNO85 sample and one latest pending ICM45686 sample. New samples replace unsent samples from the same sensor. This is intentional latest-value telemetry, not a lossless queue.

### 3. Preview rates and adaptation

Normal operating target:

- one bundle per active source every 40 ms, approximately 25 Hz;
- five active sources produce at most approximately 125 browser-bound preview notifications per second;
- graph latency target below 200 ms under normal conditions.

Congested operating target:

- one bundle per active source every 80 ms, approximately 12.5 Hz;
- enter congested mode when either three browser-notification failures occur within a rolling one-second window, or the oldest ready source exceeds 160 ms of queue age for three consecutive scheduler passes;
- return to normal mode only after five continuous seconds with no browser-notification failures and with the oldest ready source below 100 ms of queue age.

The adaptive state is global on the Master so all sources remain visually synchronized and no source is penalized independently.

### 4. Fair Master scheduler

The Master maintains one preview slot for each `(source_id, sensor_id)` pair and emits source bundles in strict round-robin order:

`MASTER -> NODE1 -> NODE2 -> NODE3 -> NODE4 -> repeat`

Rules:

- inactive sources are skipped without delaying active sources;
- a source cannot emit twice while another ready source is waiting;
- failed browser notification attempts leave the latest source data pending but do not build an unbounded backlog;
- preview scheduling continues during recording;
- record transfer traffic never shares the preview queue;
- when record transfer is active, preview remains enabled at the congested 12.5 Hz target unless the BLE stack reports persistent send failure.

### 5. Browser behavior

The HTML dashboard displays Master and Nodes 1-4 continuously.

For every source it shows:

- BNO85 orientation;
- BNO85 linear acceleration;
- BNO85 angular velocity;
- ICM45686 acceleration;
- ICM45686 angular velocity;
- observed preview rate;
- last-sample age;
- coalesced or dropped-preview count;
- connected, stale, recording, and transfer state.

Rendering is decoupled from BLE callbacks. BLE callbacks update the latest data store; a fixed 30 Hz animation/render loop updates charts. A stale badge appears when no bundle has arrived for 500 ms. Existing graph data is not erased merely because a source becomes temporarily stale.

## Recording Completion and Transfer

After recording stops:

1. Master finalizes and validates its local binary.
2. Nodes finalize local flash recordings.
3. Master schedules Node uploads sequentially in source order 1, 2, 3, 4.
4. Master owns ManifestAck, AckWindow, NackRange, resume, CRC validation, and VerifyOk decisions.
5. The browser observes progress but is not required to keep a Node upload moving.
6. Each validated Node binary remains on SD as `RxxxxNn.BIN`.
7. CSV conversion processes one validated source at a time using sequential FatFs reads.
8. `TRNxxxx.OK` is created only after every expected source is committed and the final CSV has been synchronized and renamed from `.TMP`.

A browser disconnect must not pause or invalidate a transfer already controlled by the Master. A Node disconnect pauses only that Node's transfer; after the Node reconnects, the Master resumes from its next expected chunk. When the browser reconnects, it reads the current transfer and CSV state from the Master.

## Training CSV Schema Version 2

The existing raw columns remain. Schema version 2 adds deterministic, reproducible derived features and source-quality fields.

### Common columns

- `schema_version`
- `session_id`
- `row_sequence`
- `source_node_id`
- `source_label`
- `sensor_id`
- `sensor_sequence`
- `session_time_us`
- `sample_delta_us`
- `effective_sample_rate_hz`
- `source_target_rate_hz`
- `source_attempted_count`
- `source_captured_count`
- `source_dropped_count`
- `source_loss_flags`
- `source_payload_crc32`
- `timestamp_quality_flags`

### BNO85 raw and calibrated columns

Retain the existing quaternion, linear-acceleration, gravity, angular-velocity, and availability fields.

Add:

- `bno_roll_deg`
- `bno_pitch_deg`
- `bno_yaw_deg`
- `bno_linear_accel_magnitude_mps2`
- `bno_gravity_magnitude_mps2`
- `bno_gyro_magnitude_radps`

Euler angles use normalized quaternion columns `(qx, qy, qz, qw)`, a right-handed intrinsic Z-Y-X yaw-pitch-roll convention, and output degrees. The pitch `asin` input is clamped to `[-1, 1]`. Quaternion values remain authoritative to avoid information loss and angle-wrap ambiguity.

### ICM45686 raw and calibrated columns

Retain the existing raw accelerometer, raw gyroscope, acceleration-in-g, and angular-rate-in-degrees-per-second fields.

Add:

- `icm_accel_magnitude_g`
- `icm_gyro_magnitude_dps`

### Derived-feature rules

- Features are calculated on the Master during CSV conversion.
- Features use only values in the validated binary source.
- Invalid or unavailable source fields produce empty CSV cells, not fabricated zeroes.
- The first sample for a source/sensor has an empty `sample_delta_us` and `effective_sample_rate_hz`.
- Non-monotonic or repeated timestamps set a quality flag and leave rate-derived fields empty.
- Derived columns supplement raw columns and never replace them.

## Failure Handling

### Preview failures

Preview loss is non-fatal. The system increments diagnostics, coalesces to the latest value, and continues recording. Preview queues are bounded.

### Node disconnect during recording

The Node continues recording locally. The browser marks the source stale. The Master reconnects without restarting or erasing the session. Transfer begins or resumes after the Node is reachable.

### Missing or corrupt recording source

The Master keeps validated sources and the `.TMP` file, reports the failing source and operation, and does not create `.OK`. Corrupt data is never silently included in a completed training CSV.

### Partial four-Node session

A session configured with expected mask `0x1F` is complete only when completed mask is also `0x1F`. Partial sessions remain explicitly partial and are not represented as successful model-training artifacts.

## Implementation Boundaries

Files expected to change:

- Node live-preview queue and Node recording application;
- Master hub/leaf BLE manager and central-client preview routing;
- Master main-loop preview and transfer scheduling;
- Master training CSV coordinator and CSV schema writer;
- Desktop HTML protocol decoder, graph store, rendering loop, and source diagnostics;
- host tests and hardware validation documentation.

STM32-generated code outside USER CODE sections must not be edited manually.

## Verification Plan

### Host tests

1. Five-source, two-sensor preview fairness with a high-rate noisy source.
2. Source-bundle encoding, decoding, bounds, and MTU-size checks.
3. Adaptive preview transition from 25 Hz to 12.5 Hz and recovery after five stable seconds.
4. Preview queue coalescing without mutation of recording buffers.
5. Deterministic Master, Node1, Node2, Node3, Node4 transfer order.
6. Gap, duplicate, corrupt-chunk, Node reconnect, and resume behavior.
7. CSV schema-version-2 header and row generation.
8. Quaternion-to-Euler and magnitude feature calculations against fixed vectors.
9. Empty-field and timestamp-quality behavior.
10. CSV coverage validation for sources 0, 1, 2, 3, and 4 with both sensors.

### Firmware builds

- STM32CubeIDE Master build with warnings reviewed.
- STM32CubeIDE Node build with warnings reviewed.
- Flash verification for one Master and four Nodes.

### Hardware acceptance test

Use one Master, four commissioned Nodes, and the browser dashboard.

1. Topology reports Nodes 1-4 and all are ready.
2. All 25 graph panels update during a ten-second recording.
3. Each active source sustains at least 10 displayed updates per second.
4. Normal-condition last-sample age remains below 250 ms for at least 95 percent of observations.
5. No active source is starved for more than 500 ms.
6. Recording completes with `RxxxxM.BIN`, `RxxxxN1.BIN`, `RxxxxN2.BIN`, `RxxxxN3.BIN`, `RxxxxN4.BIN`, `TRNxxxx.CSV`, and `TRNxxxx.OK`.
7. No `TRNxxxx.TMP` remains for the completed index.
8. Binary header and payload CRC checks pass for every source.
9. The CSV contains non-zero BNO85 and ICM45686 rows for sources 0-4.
10. Raw column values match decoded binary samples and derived-feature spot checks pass.
11. Disconnect the browser during a Node upload and verify that the Master continues the transfer; reconnect the browser and verify that it reports the current state without restarting the session.
12. Disconnect one Node during its upload, reconnect it, and verify resume from the next expected chunk.
13. Preview diagnostics may report coalescing, but recording dropped counts must remain within the existing accepted capture-quality limits.

## Non-Goals

- Sending every full-rate sensor sample to the browser.
- Replacing local flash or SD recording with browser storage.
- Adding activity labels, subject identifiers, or model-specific window features in this phase.
- Merging the draft pull request before both STM32 builds and the four-Node hardware test pass.
