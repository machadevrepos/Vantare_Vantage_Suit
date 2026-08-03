# Two-Node Live Preview and Training CSV Fix

## Failure reproduced from the captured run

The Master discovered and prepared both NODE1 and NODE2 (`selected_node_mask = 0x06`). The browser received at least one live sample from each Node, but the rolling graphs later contained zero Node samples. The Master completed its own recording transfer, then completed the NODE1 binary transfer. The session was still converting NODE1 ICM rows when the browser disconnected; NODE2 had not started its sequential upload yet. Consequently the incomplete session retained `TRNxxxx.TMP` and did not contain NODE2 data.

## Firmware changes

### Fair live preview queue

`HUB_LEAF_BLE_MANAGER.h` now coalesces pending live samples by `(node_id, sensor_id)`. Live preview is latest-value telemetry, not the lossless recording channel. Replacing an unsent sample from the same source prevents one high-rate stream from filling all eight queue entries and evicting the other Node/sensor streams.

The host test verifies that repeated NODE1/BNO updates retain the latest NODE1/BNO value while preserving NODE1/ICM, NODE2/BNO, and NODE2/ICM. It also verifies deterministic NODE1-then-NODE2 `RecordDone` scheduling.

### Sequential SD reads during CSV conversion

`MASTER_NODE_SESSION_STAGER.h` now tracks the FatFs read cursor. Sequential BNO and ICM rows are read contiguously, with a seek only when the requested offset is not already at the current file position. This removes one `f_lseek` call per CSV row and shortens the `ConvertNodeBno` / `ConvertNodeIcm` interval that previously delayed the next Node upload.

The host test builds a valid staged Node session, validates its CRCs, reads all BNO and ICM samples, and asserts that the conversion path performs one sample-positioning seek rather than one seek per row.

### CSV source coverage validator

Run:

```bash
python Firmware/HostTests/validate_training_csv.py TRN0002.CSV --sources 0,1,2
```

A valid two-Node training CSV must report non-zero BNO85 and ICM45686 rows for MASTER, NODE1, and NODE2. The command exits non-zero when any required source or sensor is absent.

## Hardware retest

1. Flash the updated `2nd_Branch` Master firmware. The Node firmware does not need a change for these two fixes.
2. Power the Master, NODE1, and NODE2.
3. Confirm the topology and ready masks contain Nodes 1 and 2.
4. Start streaming. Verify that all five charts under MASTER, NODE1, and NODE2 continue updating throughout the rolling window.
5. Start a recording with selected mask `0x06`, then stop it.
6. Keep the browser connected until the training state reports `Complete` and the completed mask is `MASTER+NODE1+NODE2`.
7. Confirm the SD session contains `RxxxxM.BIN`, `RxxxxN1.BIN`, `RxxxxN2.BIN`, `TRNxxxx.CSV`, and `TRNxxxx.OK`, with no remaining `TRNxxxx.TMP` for that file index.
8. Run the CSV validator with `--sources 0,1,2`.

## Remaining validation boundary

These changes are host-tested but have not been compiled with STM32CubeIDE or exercised on the physical two-Node BLE/SD system. The pull request must remain draft until both firmware builds and the hardware retest pass.
