# Four-Node Live Preview and Training CSV Validation

## Scope

This procedure validates one Master and four commissioned Nodes from the same reviewed throughput build. It covers topology, continuous live graphs during recording, autonomous sequential Node uploads, binary durability, transfer throughput, and training CSV schema version 2.

The pull request must remain draft until every mandatory check below passes on physical hardware.

## Hardware and firmware preparation

1. Build and flash the Master project from `firmware/Master` and record the Git commit plus Debug/Release image sizes.
2. Build and flash the Node project from `firmware/Node` from the same commit to all four Node PCBs; record its Debug/Release image sizes.
3. Commission the persistent Node IDs as 1, 2, 3, and 4.
4. Power-cycle every PCB and query each Node ID before assembling the test topology.
5. Use the matching `host/desktop_tool/Exoskeleton.html` file.

## Topology gate

Power the Master and Nodes 1–4, then connect the browser to the Master.

Required observations:

- discovered Nodes: `NODE1, NODE2, NODE3, NODE4`;
- transport-ready Node mask: `0x1E`;
- recording-ready Node mask: `0x1E`;
- complete expected recording source mask: `0x1F` including the Master;
- no duplicate or stale Node IDs after a Master reset and rediscovery.

Do not start a four-Node recording unless all four Nodes are transport-ready and recorder-ready.

## Continuous live-preview test

1. Start the live stream with all five sources active.
2. Observe the Master and Nodes 1–4 for at least 30 seconds before recording.
3. Start a ten-second recording with selected Node mask `0x1E`.
4. Continue observing the graphs throughout the recording interval.
5. Stop the recording and keep the system powered while the Master collects and converts each Node session.

Required graph behavior during recording:

- all five source sections continue updating;
- each source shows BNO85 orientation, linear acceleration, and angular velocity;
- each source shows ICM45686 acceleration and angular velocity;
- all 25 graph panels receive data during the recording;
- each active source sustains at least 10 displayed updates per second under normal RF conditions;
- normal last-sample age remains below 250 ms for at least 95 percent of observations;
- no active source is starved for more than 500 ms;
- preview coalescing is allowed, but it must not change the stored recording counts.

The preview path is latest-value telemetry. It is intentionally rate-limited and may coalesce unsent values. The binary recording path remains full fidelity and independent.

## Sequential transfer and browser-disconnect test

The expected upload order is:

`NODE1 -> NODE2 -> NODE3 -> NODE4`

During one test run:

1. Wait until one Node upload is active, then record the active transfer's Node ID, session ID, and next expected chunk.
2. Disconnect or close the browser.
3. Leave the Master and Nodes powered while the browser remains disconnected.
4. Reconnect after several seconds and wait for the next compact `0xB6` status update. Confirm that staged bytes advanced or that the source entered validation/completed state. A debug-probe trace may be retained as secondary evidence, but it is not required.
5. Reconnect the browser after several seconds.
6. Confirm the Master continues from the advanced transfer state rather than restarting the complete session or discarding validated data.

Required behavior:

- the Master owns ManifestAck, AckWindow, NackRange, and VerifyOk decisions;
- a missing chunk produces a recoverable NACK, not a terminal `StageError`;
- a corrupt chunk produces a recoverable CRC NACK;
- duplicate chunks do not advance the staged offset;
- VerifyOk is issued only after the staged SD binary passes header and payload CRC validation;
- the next Node does not start until the current Node has been validated and committed.

## Required SD artifacts

For session index `xxxx`, the completed directory must contain:

```text
RxxxxM.BIN
RxxxxN1.BIN
RxxxxN2.BIN
RxxxxN3.BIN
RxxxxN4.BIN
TRNxxxx.CSV
TRNxxxx.OK
```

The completed index must not contain `TRNxxxx.TMP`.

Each binary must pass:

- session magic and format-version checks;
- source ID and session ID checks;
- completion flag check;
- header CRC32 check;
- payload-size consistency check;
- payload CRC32 check.

A session configured for expected mask `0x1F` is incomplete unless the completed mask is also `0x1F`. An incomplete or corrupt session must retain its `.TMP` recovery artifact and must not create `.OK`.

## CSV schema version 2 validation

Copy the completed CSV to the repository workstation and run:

```powershell
python Firmware/HostTests/validate_training_csv.py E:\SESSIONS\TRNxxxx.CSV --sources 0,1,2,3,4
```

The validator must report non-zero BNO85 and ICM45686 rows for:

- MASTER;
- NODE1;
- NODE2;
- NODE3;
- NODE4.

It also verifies:

- `schema_version` equals 2;
- row and sensor sequences are coherent;
- source labels match source IDs;
- source target, attempted, captured, dropped, loss-flag, and payload-CRC metadata are present;
- first samples have empty delta/rate fields;
- monotonic samples have correct `sample_delta_us` and `effective_sample_rate_hz`;
- non-monotonic timestamps are quality-flagged and do not contain fabricated timing features;
- BNO quaternion, vector, Euler-angle, and magnitude fields contain finite representative values;
- ICM raw, scaled, and magnitude fields contain finite representative values.

## Stored-data acceptance

For each source and sensor:

- stored sample count equals the validated binary header count;
- captured count equals the number of stored samples;
- attempted, captured, and dropped fields are internally consistent;
- preview drop/coalescing counters are not substituted for recording dropped counts;
- raw CSV values match spot-checked decoded binary samples;
- derived features match independent calculations within numeric formatting tolerance.

## Operator-visible transfer evidence

Keep raw download/debug relay **off** for every throughput run. The default path suppresses raw Manifest/Chunk relay to the browser and retains compact progress, which prevents browser work from becoming part of the Node-to-Master benchmark. The explicit raw download/debug checkbox may be used for diagnosis only; it is read-only, leaves ACK/credit/verify ownership with the Master, and invalidates a performance run because it adds browser traffic.

For each active Node, capture the browser fields below in the saved console log:

- Master link state is one of `unknown`, `requested`, `confirmed`, `degraded`, or `failed`;
- request acceptance is not confirmation: DLE octets, PHY, and interval count only when a completion event reports them;
- a zero DLE value remains `unknown`; it is never interpreted as a confirmed 27-byte default;
- Node `LINK_STATS` DLE outcome/request status/attempts, controller maximum, negotiated TX/RX octets, accepted bytes and notifications, busy/resource counts, notification-complete events, TX-pool events/buffers, watchdog wakes, terminal errors, and flash-read time;
- effective `0xB6` receiver credit, ACK chunk threshold/timeout, queue pending/high-water/overflow, received chunks, ACK attempts/success/failure/last status, suppressed raw-relay count, and SD flush count/maximum duration;
- staged/total bytes, instantaneous and per-Node average KiB/s, validation result, and terminal collection state.

If the browser reports a legacy 19-byte `0xB6` status, the collection state remains usable but the effective flow-control fields are unavailable. Do not use that run for the throughput benchmark.

## Hardware throughput benchmark matrix

Benchmark all nine interval/credit combinations below. ACK threshold must always be strictly below the effective credit; the suggested values keep the comparison reproducible.

| Confirmed connection interval | Effective credit 8 | Effective credit 16 | Effective credit 24 |
|---|---:|---:|---:|
| 15 ms | ACK threshold 4 | ACK threshold 8 | ACK threshold 8 |
| 11.25 ms | ACK threshold 4 | ACK threshold 8 | ACK threshold 8 |
| 7.5 ms | ACK threshold 4 | ACK threshold 8 | ACK threshold 8 |

For every matrix cell:

1. Apply the requested credit and ACK threshold, then use the **effective** values echoed in `0xB6` as the run configuration. Reject a run if the echo differs and the difference is not recorded.
2. Require completion evidence for the selected interval. Record requested and confirmed interval separately; command acceptance alone does not qualify the cell.
3. Use realistic production-sized session artifacts, not a small synthetic transfer. Record the requested capture duration and exact artifact size.
4. Run three transfers for each Node (`NODE1` through `NODE4`), resetting only the session/run index between trials. This gives 12 per-Node observations for the cell.
5. Run three complete four-Node collections in the normal sequential order `NODE1 -> NODE2 -> NODE3 -> NODE4`.
6. Save the browser log and the five SD binaries for every complete collection. Record RF setup, board IDs, power source, browser/OS version, firmware commit, build configuration, and any reconnect.

For every Node transfer, calculate sustained payload throughput as:

```text
KiB/s = exact validated artifact bytes / 1024 / (last staged byte time - first staged byte time in seconds)
```

Do not use ATT packet size, accepted-notification bytes, aggregate four-Node time, or browser-arrival rate as the numerator. Report all three trials, median, minimum, and maximum; do not discard slow trials.

## Throughput and integrity acceptance gates

A configuration passes only when every required run meets all of these gates:

- sustained payload throughput is at least **30 KiB/s for every Node transfer**, not only as an average across Nodes or trials;
- the manifest size, staged byte count, committed SD file size, and independently copied workstation file size are exactly equal;
- header CRC32 and payload CRC32 pass independently for every Master and Node binary;
- every source reaches an explicit validated/committed terminal state; there are no stalls, queue overflows, terminal pump errors, ACK failures, or unclassified errors;
- retransmission ratio is below **0.1%** for every Node, calculated as retransmitted chunks divided by received unique chunks times 100; record the raw numerator and denominator even when both recovery counters and NACKs are zero;
- effective ACK threshold remains below effective credit, pending queue never exceeds capacity, and queue overflow remains zero;
- raw-relay suppression advances during Node artifact upload while staged-byte compact progress remains visible;
- the browser remains responsive: controls react, the console can scroll/copy, progress continues to update, and there is no visible freeze lasting one second or longer; record any browser long task, disconnect, or dropped notification as a failed run;
- all three complete four-Node collections finish in order with exact bytes/CRC and without manual recovery.

Transfer admission is independent of acquisition admission. A CRC-clean, 30 KiB/s artifact is still rejected for AI training if any of these separate gates fail:

- captured/attempted/dropped counts and loss flags do not meet the predeclared per-sensor completeness thresholds;
- BNO85 or ICM45686 timestamp gaps/rates exceed their declared timing limits;
- source start/stop timing or cross-device alignment cannot be reconstructed within the declared synchronization tolerance;
- preview quality, transfer counters, or CRC results are substituted for stored sensor sample evidence.

Record binary integrity, acquisition completeness, timing, synchronization, transfer performance, and operator observability as separate pass/fail columns.

## Conditional L2CAP CoC stop gate

If completion telemetry confirms DLE of at least 247 octets, 2M PHY, and the selected fast interval, yet sustained payload throughput remains below 30 KiB/s after the full matrix, stop changing GATT constants. Open a separate design task for a capability-negotiated L2CAP CoC Manifest/Chunk bulk lane. Keep GATT for control/status, preserve exact byte/CRC verification, and require automatic fallback to the current reliable GATT path when either peer does not advertise CoC support. Do not implement or claim CoC as part of this benchmark task.

## Pass criteria

The four-Node implementation is accepted only when all of the following are true:

1. Nodes 1–4 remain connected and ready.
2. All 25 graph panels update continuously during recording.
3. No source exceeds the starvation limit.
4. The Master completes autonomous sequential collection of all four Nodes.
5. Browser disconnect does not invalidate or lose a captured session.
6. All five binary artifacts are retained and CRC-valid.
7. The CSV finalizes with `.OK` and without `.TMP`.
8. The schema-v2 validator passes for sources `0,1,2,3,4`.
9. STM32CubeIDE Master and Node builds complete with warnings reviewed.
10. The draft pull request remains unmerged until these checks are recorded.
