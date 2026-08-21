# Vantage Suit Firmware — Issues & Fixes

**Branch:** `suit_testing`
**Date:** 2026-08-21
**Scope:** BLE bring-up regression, first 4-node data-collection session (session_id = 1), acquisition data-quality analysis, and the node ICM FIFO fix.

---

## Summary

| # | Issue | Severity | Status |
|---|-------|----------|--------|
| 1 | BLE path broken (Master won't advertise, nodes won't connect) | Blocker | **Resolved** (environmental, not a source regression) |
| 2 | Node ICM data is ~50 % duplicated register reads + synthetic timestamps | High (training data) | **Fixed in firmware** |
| 3 | Master BNO ~78 % sample loss (~22 Hz vs 100 Hz) | High (training data) | **Open** — root cause identified |
| 4 | Node1 upload failed; abandoned after 30 s stall (0-byte staged file) | Medium | **Open** — link-recovery gap identified |
| 5 | Node→Master transfer throughput ~1.8 KB/s | High (blocks 10-min sessions) | **Open** — root cause hypothesis |

The only issue fixed in code so far is **#2 (node ICM FIFO)**. The rest are documented with root cause and recommended direction.

---

## System context

- **Hardware:** STM32WB55 (Cortex-M4 + BLE coprocessor). One **Master** (source id 0) + up to four **Nodes** (source ids 1–4).
- **Sensors per device:** BNO085 (fused quaternion + linear-accel/gravity/gyro, ~100 Hz, on I2C3 @ addr 0x4B) and ICM45686 (raw accel/gyro, 200 Hz, on I2C1 @ addr 0x69).
- **Roles:** Master is a BLE **peripheral** to the browser ([Exoskeleton.html](../Firmware/DesktopTools/Exoskeleton.html)) and a BLE **central** to the nodes.
- **Recording:** Master records to SD (`MREC.BIN` → validated `R####M.BIN`); each node records to W25Q256 SPI flash (32 MB detected), then uploads to Master SD as `R####N#.BIN` via a reliable windowed transfer. Off-device conversion to CSV is done by [`vantage_bin_to_csv.py`](../Firmware/DesktopTools/vantage_bin_to_csv.py).
- **Sample format:** ESOX v4 — `Bno85SampleV3` (56 B) and `Icm45686SampleV4` (20 B, includes `offset_us` + `sequence`).

---

## Issue 1 — BLE communication path not working

### Symptoms
- Master would not activate its own BLE peripheral (browser could not connect to `Exoskeleton.html`).
- Nodes would not connect to the Master.
- Reported working on `origin/2nd_Branch`, broken on `suit_testing`.

### Investigation
`origin/2nd_Branch` (tip `147c589`) is a **direct ancestor** of `suit_testing`; the current branch is `2nd_Branch` + 7 commits. Every file in the BLE bring-up path was diffed across those commits:

| Component | File | Result |
|---|---|---|
| BLE stack init / GAP / advertising | `STM32_WPAN/App/app_ble.c` | **unchanged** |
| BLE config (links, name, MTU) | `Core/Inc/app_conf.h`, `App/ble_conf.h` | **unchanged** |
| IPCC transport (CPU1↔CPU2) | `Target/hw_ipcc.c` | **unchanged** |
| WPAN init entry (`MX_APPE_Init`) | `Core/Src/app_entry.c` | **unchanged** |
| RF / clock config | `.ioc` | **only I2C sensor-bus speed changed** (100→400 kHz); RF/clock/IPCC untouched |
| Boot ordering (sensor init → `MX_APPE_Init`) | `Core/Src/main.c` | **unchanged** (first `main()` diff hunk is inside the while-loop, after BLE start) |
| Advertised name | `HUB0001` | **unchanged**, matches HTML `deviceNamePrefix` |
| Link budget | `CFG_BLE_NUM_LINK = 6` | **unchanged** — supports 4 nodes + browser + 1 spare |
| Browser BLE contract (UUIDs, name prefixes) | `Exoskeleton.html` | **unchanged** |

Additional rule-outs:
- **Boot hang** — the BNO `begin()` is bounded (5 attempts, then returns `false`); it cannot spin, so boot always reaches `MX_APPE_Init()`.
- **Loop starvation** — the BNO `service()` polls the INT pin and issues zero I2C when no packet is pending (INT is wired via `main.h`), so it cannot block the radio.
- **I2C 400 kHz timing** — the value `0x1F081539` decodes to a valid ~400 kHz timing; the non-zero reserved nibble is ignored by hardware.

### Conclusion
**No BLE-bring-up regression exists in the source diff.** The activation, service/characteristic contract, scanning and link budget are functionally identical to the working branch. The regression is **environmental**, with the leading causes:

1. **CPU2 wireless-stack firmware** (most likely). On STM32WB the BLE stack is a separate binary on the radio coprocessor. If reflashing `suit_testing` erased it, installed a mismatched version, or left FUS in a bad state, BLE dies identically on Master and every node.
2. **Wrong/stale build** — verify the **FLASH** linker output (`STM32WB55CCUX_FLASH.ld`, not the RAM `.ld`) was flashed and that the branch compiled clean.
3. **Boot-log check** — on SWO/UART confirm `[BUILD][MASTER] ble-mtu-node-log-fix active` prints followed by BLE init logs.

### Resolution
BLE came up after the coprocessor/build environment was corrected — session 1 ran successfully (connect, 4-node discovery, session start, recording, and collection of 3/4 nodes). **No firmware change was required for this issue.**

---

## Issue 2 — Node ICM: ~50 % duplicate samples + synthetic timestamps (FIXED)

### Evidence (session 1 converted CSVs)
Comparing the two ICM acquisition methods on the same session under identical (still) conditions:

| Stream | Method | `offset_us` | Sequence | Duplicate rows |
|---|---|---|---|---|
| **Master ICM** | hardware FIFO | real, ~5008 µs, jitter to 110 ms | 0→11442 continuous | **0 %** |
| Node2 ICM | 5 ms register poll | **hardcoded 5000 µs** | continuous | **49.6 %** |
| Node3 ICM | 5 ms register poll | **hardcoded 5000 µs** | continuous | **49.4 %** |
| Node4 ICM | 5 ms register poll | **hardcoded 5000 µs** | continuous | **49.6 %** |

Roughly **half of every node's ICM rows are bit-for-bit identical to the previous row.** This is not physical stillness — the Master ICM (same sensor, same session, same conditions) has **0 % duplicates** because it reads the hardware FIFO. The node polls registers every 5 ms with no data-ready gate, so when the sensor has not produced a new sample it re-reads the previous one. Net effect: despite writing ~8 600 rows at a nominal 200 Hz, each node carried only **~75–100 Hz of unique ICM data**, with synthetic (constant-step) timestamps.

### Root cause
The node ICM FIFO path was **deliberately disabled**. The FIFO capture (`begin_fifo_capture_200hz` / `read_fifo_samples`) exists in the shared driver and is used successfully by the Master, and all node-side plumbing (FIFO read in `process_recording_ticks`, FIFO-tail drain in `try_finalize_recording`) is present. The most recent sensor patch (`da21379`, `sensor_acquisition_icm_fix_v3`) hardcoded `record_icm_fifo_active_ = false` on the node and reverted it to 5 ms register polling.

The revert had **no stated technical justification**. Its own diff flipped the host-test invariant from *requiring* the FIFO to *forbidding* it — the original test read: *"Node recording must use the real 200 Hz ICM FIFO instead of synthetic catch-up reads."* The revert reintroduced exactly the "synthetic catch-up reads" defect that the FIFO was written to avoid, which is what the session-1 data shows.

### Fix
Re-enabled the FIFO path on the node and brought the shared FIFO driver code up to the InvenSense reference (`motion.mcu.icm45686.driver/examples/basic_read_fifo/basic_read_fifo.c`).

**`Firmware/LIBRARY/CUSTOM/ICM45686_STM32.h` — `begin_fifo_capture_200hz()`**
- Read-modify-write the FIFO config via `inv_imu_get_fifo_config()` before setting fields (documented pattern; preserves driver-managed reserved bits) instead of zero-initialising.
- Set the low-noise anti-alias bandwidth **`BW = ODR/4`** (`inv_imu_set_accel_ln_bw` / `inv_imu_set_gyro_ln_bw` with `LPFBW_DIV_4`), matching the reference for a clean 200 Hz capture.

**`Firmware/LIBRARY/CUSTOM/ICM45686_STM32.h` — `read_fifo_samples()`**
- Reject frames carrying the documented **`INVALID_VALUE_FIFO` (0x8000)** sentinel on any accel/gyro axis (settling/unavailable artefacts must not enter the recorded stream).
- A fixed startup-sample discard was **intentionally not added**: the node/Master ICM runs continuously in low-noise mode from boot, so `begin_fifo_capture_200hz` only changes ODR and flushes — a fixed 70 ms discard would drop *valid* data and inflate drop counts. The `INVALID_VALUE_FIFO` check handles genuine settling frames without guessing a count.

**`Firmware/LIBRARY/CUSTOM/NODE_RECORDING_APP.h`** — both record-start sites (delayed/armed start in `process()`, and `start_recording_now()`) now arm the FIFO:
```cpp
record_icm_fifo_active_ = icm45686_.begin_fifo_capture_200hz();
```
If arming fails, `record_icm_fifo_active_` stays `false` and the existing 5 ms register path runs as an **automatic fallback** (no behavioural loss on hardware that cannot arm the FIFO).

**`Firmware/HostTests/test_acquisition_demo_invariants.py`** — invariant flipped back to require the node FIFO. **Passing.**

### Expected result
Node ICM streams should match the Master's clean output: real per-frame `offset_us` (16 µs TMST-derived), **0 % duplicate rows**, ~200 Hz with drops near zero, and the `icm_read` loss flag cleared. Both Master and Node ICM additionally gain proper anti-alias filtering and invalid-frame rejection.

### Verification plan (single-node rollout)
1. Build Master + Node in STM32CubeIDE; confirm clean compile.
2. Flash **Master + one node**; leave the other three on the previous build. Run a ~1-min session; convert.
3. On the new node's ICM CSV confirm: `consecutive identical raw rows = 0 %` and `offset_us` gaps that vary around 5000 µs (not a flat 5000). Confirm Master ICM is still clean (BW change did not regress it).
4. If clean, flash the remaining nodes.

---

## Issue 3 — Master BNO: ~78 % sample loss (OPEN)

### Evidence
Master session-1 header: BNO `captured = 1323`, `attempted = 6007`, `dropped = 4684` → **~22 Hz effective vs 100 Hz target**, `loss_flags` bit `bno_read` set. The `offset_us` gap distribution is bimodal (median ~48 µs bursts, 258 gaps > 15 ms), i.e. long BNO-silent stretches.

### Root cause
The Master is the busiest device (BLE peripheral + 4 central links + live streaming). Its BNO orientation reports are drained by `sh2_service()` inside the superloop; when the loop is busy, `sh2_service()` is called too infrequently and the BNO's internal SHTP buffer overflows, losing reports at the source.

### Recommended direction (not yet implemented)
- Increase the BNO service budget per loop and/or service the BNO more than once per iteration when the capture queue is enabled.
- Reduce competing per-loop work during recording (e.g. de-prioritise live streaming while capture is active).
- Re-measure `captured/attempted` after tuning.

---

## Issue 4 — Node1 upload failed; abandoned after 30 s stall (OPEN)

### Evidence
- Console: `Master SD Collection: … completed=MASTER failed=NODE1 node=NODE1`, ~30 s after all four nodes reported `state=4` (ReadyForUpload).
- Output: `R0001N1.BIN` is **0 bytes** (staging file created, no chunk ever written). Nodes 2/3/4 completed normally.
- Node1 **recorded successfully** to its own flash (reached ReadyForUpload, `session=1`); it simply never advanced to `state=5` (Uploading), so zero chunks were sent, and the coordinator abandoned it after `kNodeStallMs = 30 s`.

### Root cause (hypothesis)
Node1 was the first source pulled (lowest id). Its central link appears to have stalled at collection start. During collection, `g_remote_transfer_active` sets `discovery_hold = 1` on the Master, which makes `exo_hub_central_client_process()` **return early** — the Master will not re-scan/reconnect a dropped node link mid-session. Nodes 2/3/4 kept their links and uploaded; Node1 could not self-heal.

### Notes
- **Node1's data is preserved** on its flash — the abandon path (`stager_.shutdown()`) leaves `discarded_` clear, so the session can be re-pulled later.
- Recommended direction: allow controlled link recovery for the active source during collection (without tearing down healthy links), and/or a re-pull command for a `failed_source_mask` node.

---

## Issue 5 — Node→Master transfer throughput ~1.8 KB/s (OPEN)

### Evidence
Collecting one node's ~495 KB took **~4.6 min** (e.g. Node2: `ReceivingNode` at +189301 ms → `ValidatingNode` at +467286 ms) ≈ **~1.8 KB/s (~10 chunks/s)** — roughly an order of magnitude below expected BLE throughput.

### Impact on the 10-minute goal
Capacity is fine (32 MB node flash ≈ 55 min max; `capacity_ms = 3 494 390` observed). But at ~1.8 KB/s a 10-min session (~5.7 MB/node) would take **~50 min per node (~3.3 h for four)**. The 30 s stall timer resets on progress so it would not *abandon*, but offload is impractical at this rate. **This is the hard gate on 10-minute sessions.**

### Root cause (hypothesis)
The Master superloop continues full BNO/ICM sampling + live preview + servicing all central links *during* collection, starving reliable-chunk processing (~100 ms per chunk round-trip). Recommended direction: profile the collection loop, reduce non-transfer work while a node upload is active, and revisit credit/ack cadence (`credit`, `ack_chunks`, burst limits).

---

## Session 1 — raw per-source analysis (reference)

Recording duration: nodes ~57.6–57.9 s (stopped on command), Master 60.06 s (ran to the 60 s safety window). All written binaries structurally valid (magic `ESOX`, v4, completion `0xA5`, payload size == file size).

| Source | File (bytes) | BNO capt/attempt (drop) | BNO rate | ICM capt/attempt (drop) | ICM rate | ICM dup rows | loss_flags |
|---|---|---|---|---|---|---|---|
| Master | 303 036 | 1323 / 6007 (4684) | ~22 Hz | 11443 / 12013 (570) | ~188 Hz | 0 % | 0x03 |
| Node1 | **0** | — (transfer failed) | — | — | — | — | — |
| Node2 | 495 416 | 5768 / 5780 (12) | ~100 Hz | 8616 / 11560 (2944) | ~149 Hz | 49.6 % | 0x03 |
| Node3 | 493 992 | 5779 / 5779 (0) | ~100 Hz | 8514 / 11520 (3006) | ~148 Hz | 49.4 % | 0x02 |
| Node4 | 496 384 | 5776 / 5795 (19) | ~100 Hz | 8642 / 11590 (2948) | ~149 Hz | 49.6 % | 0x03 |

Cleanest streams before the fix: **Master ICM** (FIFO) and **Node BNO** (interrupt-driven queue, count-complete). Weakest: **Node ICM** (polled — fixed by Issue 2) and **Master BNO** (service starvation — Issue 3).

Note: the CSV schema omits derived roll/pitch/yaw (the v4 sample format dropped those fields; only v2 carried them). Raw quaternion is present and RPY is derivable off-device.

---

## Files changed (Issue 2 fix)

- `Firmware/LIBRARY/CUSTOM/ICM45686_STM32.h` — documented FIFO config (RMW + `BW=ODR/4`) and `INVALID_VALUE_FIFO` validation.
- `Firmware/LIBRARY/CUSTOM/NODE_RECORDING_APP.h` — node arms the 200 Hz FIFO at both record-start sites, with automatic register-poll fallback; stale comments updated.
- `Firmware/HostTests/test_acquisition_demo_invariants.py` — invariant restored to require the node FIFO.
