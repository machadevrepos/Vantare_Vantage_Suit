# Four-Node Live Preview and Training CSV Validation

## Scope

This procedure validates one Master and four commissioned Nodes after flashing the `2nd_Branch` firmware. It covers topology, continuous live graphs during recording, autonomous sequential Node uploads, binary durability, and training CSV schema version 2.

The pull request must remain draft until every mandatory check below passes on physical hardware.

## Hardware and firmware preparation

1. Build and flash the Master project from `Firmware/Master` on `2nd_Branch`.
2. Build and flash the Node project from `Firmware/Node` on `2nd_Branch` to all four Node PCBs.
3. Commission the persistent Node IDs as 1, 2, 3, and 4.
4. Power-cycle every PCB and query each Node ID before assembling the test topology.
5. Use the current `Firmware/DesktopTools/Exoskeleton.html` file.

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

1. Wait until one Node upload is active, then record the active Node ID and next expected chunk.
2. Disconnect or close the browser.
3. Leave the Master and Nodes powered while the browser remains disconnected.
4. Before reconnecting, confirm from the Master diagnostics that the next expected chunk advanced or that the active Node moved into the validated state.
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
