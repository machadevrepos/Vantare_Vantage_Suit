# Vantage Suit Firmware — Issues & Fixes

**Branch:** `fix/Firmware_Fixes` (session-1 analysis originally captured on `suit_testing`; all fixes and the full review below verified on `fix/Firmware_Fixes` @ `999cba4`)
**Date:** 2026-08-21
**Scope:** BLE bring-up regression, first 4-node data-collection session (session_id = 1), acquisition data-quality analysis, the node ICM FIFO fix, and a full-codebase adversarial review (Master, Node, protocol, tests) — see [Open issue register (#6+)](#full-codebase-review--open-issue-register-6).

---

## Summary

| # | Issue | Severity | Status |
|---|-------|----------|--------|
| 1 | BLE path broken (Master won't advertise, nodes won't connect) | Blocker | **Resolved** (environmental, not a source regression) |
| 2 | Node ICM data is ~50 % duplicated register reads + synthetic timestamps | High (training data) | **Fixed in firmware** |
| 3 | Master BNO ~78 % sample loss (~22 Hz vs 100 Hz) | High (training data) | **Fixed in firmware** (`b894711`) — awaiting hardware verification |
| 4 | Node1 upload failed; abandoned after 30 s stall (0-byte staged file) | Medium | **Fixed in firmware** (`5f5d2e4`) — awaiting hardware verification |
| 5 | Node→Master transfer throughput ~1.8 KB/s | High (blocks 10-min sessions) | **Fixed in firmware** (`9a808c7`) — awaiting hardware verification |

Issues #3/#4/#5 were fixed in code on 2026-08-21; the sections below retain the original session-1 analysis. The only issue fixed in code before that date was **#2 (node ICM FIFO)**.

A full-codebase review on 2026-08-21 (branch `fix/Firmware_Fixes`) re-verified all four fixes in code (see [Verified-fix audit](#verified-fix-audit)) and opened a new register: **#6–#13 (P1)**, #14–#29 (P2), plus minor items — see the [last section](#full-codebase-review--open-issue-register-6) for evidence (`file:line`), impact, and the staged fix roadmap. **Three of the new P1s gate the 10-minute-session goal** (#6 node erase time, #7 Master archive block, #13 pacing ceiling).

**2026-08-22: the entire register was fixed in code** — 11 commits, one patch per issue cluster, all Python invariant tests green after each step; adversarial review findings fixed (`77b7927`). See [Fix campaign applied 2026-08-22](#fix-campaign-applied-2026-08-22-branch-fixfirmware_fixes) for the commit map, deferred items, and the hardware verification checklist. **Everything now awaits CubeIDE compilation + hardware verification.**

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

## Issue 3 — Master BNO: ~78 % sample loss (FIXED in firmware — awaiting hardware verification)

### Evidence
Master session-1 header: BNO `captured = 1323`, `attempted = 6007`, `dropped = 4684` → **~22 Hz effective vs 100 Hz target**, `loss_flags` bit `bno_read` set. The `offset_us` gap distribution is bimodal (median ~48 µs bursts, 258 gaps > 15 ms), i.e. long BNO-silent stretches.

### Root cause
The Master is the busiest device (BLE peripheral + 4 central links + live streaming). Its BNO orientation reports are drained by `sh2_service()` inside the superloop; when the loop is busy, `sh2_service()` is called too infrequently and the BNO's internal SHTP buffer overflows, losing reports at the source.

### Recommended direction (not yet implemented)
- Increase the BNO service budget per loop and/or service the BNO more than once per iteration when the capture queue is enabled.
- Reduce competing per-loop work during recording (e.g. de-prioritise live streaming while capture is active).
- Re-measure `captured/attempted` after tuning.

---

## Issue 4 — Node1 upload failed; abandoned after 30 s stall (FIXED in firmware — awaiting hardware verification)

### Evidence
- Console: `Master SD Collection: … completed=MASTER failed=NODE1 node=NODE1`, ~30 s after all four nodes reported `state=4` (ReadyForUpload).
- Output: `R0001N1.BIN` is **0 bytes** (staging file created, no chunk ever written). Nodes 2/3/4 completed normally.
- Node1 **recorded successfully** to its own flash (reached ReadyForUpload, `session=1`); it simply never advanced to `state=5` (Uploading), so zero chunks were sent, and the coordinator abandoned it after `kNodeStallMs = 30 s`.

### Root cause (hypothesis)
Node1 was the first source pulled (lowest id). Its central link appears to have stalled at collection start. During collection, `g_remote_transfer_active` sets `discovery_hold = 1` on the Master, which makes `exo_hub_central_client_process()` **return early** — the Master will not re-scan/reconnect a dropped node link mid-session. Nodes 2/3/4 kept their links and uploaded; Node1 could not self-heal.

### Notes
- **Node1's data is preserved** on its flash — the abandon path (`stager_.shutdown()`) leaves `discarded_` clear, so the session can be re-pulled later.
- Recommended direction: allow controlled link recovery for the active source during collection (without tearing down healthy links), and/or a re-pull command for a `failed_source_mask` node.
- **Follow-up (2026-08-21 review):** the link-recovery half is fixed in code (`5f5d2e4`); the **re-pull command for an already-abandoned source remains open** (roadmap step 3). The review also found that an abandoned upload leaves a truncated `R####N#.BIN` on the Master SD that permanently consumes the run index — tracked as issue **#9**.

---

## Issue 5 — Node→Master transfer throughput ~1.8 KB/s (FIXED in firmware — awaiting hardware verification)

### Evidence
Collecting one node's ~495 KB took **~4.6 min** (e.g. Node2: `ReceivingNode` at +189301 ms → `ValidatingNode` at +467286 ms) ≈ **~1.8 KB/s (~10 chunks/s)** — roughly an order of magnitude below expected BLE throughput.

### Impact on the 10-minute goal
Capacity is fine (32 MB node flash ≈ 55 min max; `capacity_ms = 3 494 390` observed). But at ~1.8 KB/s a 10-min session (~5.7 MB/node) would take **~50 min per node (~3.3 h for four)**. The 30 s stall timer resets on progress so it would not *abandon*, but offload is impractical at this rate. **This is the hard gate on 10-minute sessions.**

### Root cause (hypothesis)
The Master superloop continues full BNO/ICM sampling + live preview + servicing all central links *during* collection, starving reliable-chunk processing (~100 ms per chunk round-trip). Recommended direction: profile the collection loop, reduce non-transfer work while a node upload is active, and revisit credit/ack cadence (`credit`, `ack_chunks`, burst limits).

### Review caveat (2026-08-21): hard pacing ceiling just above the target
The node sender paces one 180 B chunk per 8 ms with burst limit 1 (`kNodeRecordChunkGapMs = 8`, `kNodeRecordBurstLimit = 1` — `Firmware/Node/Core/Src/main.c:304-305`, enforced at `:696-703`). That caps the link at **~22 KB/s theoretical regardless of every other fix**; the ≥10× verification target (≥18 KB/s) sits only ~20 % under this ceiling. The `9a808c7` fix did not touch these constants. Hardware throughput measurement must confirm the achieved rate; if it lands short of target, raising burst limit / shortening the gap (plus credit) is the next lever — tracked as issue **#13**.

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

---

## Fixes applied 2026-08-21 (Issues 3, 4, 5)

### Issue 3 — `b894711` applied master_bno_service_recovery_patch
- `HUB_SENSOR_TEST_APP.h`: new `service_bno()` bounded drain (same capture/idle packet-budget policy as `process()`).
- Master superloop: two extra BNO service points — right after `MX_APPE_Process()` and right after `local_record_collect()` — so the worst-case gap between SHTP drains is the longest single blocking region, not the whole loop period.
- Quiet capture: phone-facing live IMU stream and per-sample SWO telemetry are paused while `record_bno_queue_active()` is true; both resume automatically when recording ends.

### Issue 5 — `9a808c7` applied node_transfer_throughput_fix_patch
- ACK cadence: coordinator gained `service_reliable_control(now_ms)` (honors the post-manifest defer counter); the superloop calls it straight after BLE dispatch, so a chunk ACK is transmitted within the same iteration it was queued instead of one full loop later.
- Stager write buffering: sequential chunk appends accumulate in a 4 KB RAM block (`MASTER_NODE_SESSION_STAGER.h`) and flush on full / before duplicate read-back / before validation close / best-effort at shutdown, removing the per-chunk blocking `f_lseek+f_write` from BLE event context.
- Quiet transfer: live stream + SWO prints also pause while `g_ble_record_transfer_mode || g_remote_transfer_active`; per-chunk pipe logs moved behind default-off `EXO_HUB_VERBOSE_PIPE_LOGS`.

### Issue 4 — `5f5d2e4` applied session_link_recovery_patch
- New `exo_hub_central_client_request_targeted_reconnect(node_id)`: while discovery hold is active, a dropped node is direct-connected from its stored address (no general scan; healthy links untouched).
- `exo_hub_central_client_process()` hold branch executes the armed targeted connect; the disconnect handler arms it automatically mid-session. Consecutive failed attempts are capped at 3 (budget resets on any successful connection or when hold releases), and radio failures re-arm after the status backoff.
- Once the link is READY again, existing manifest re-send scheduling delivers MANIFEST and the upload resumes; a re-pull command for already-abandoned sources remains future work.

### Verification
- Host suite: all Python invariant tests pass (extended with checks for each fix). C++ host tests (incl. new stager buffered-write scenarios) must be compiled in the STM32CubeIDE environment — no host toolchain on the dev box.
- Hardware: build Master + Nodes, then confirm (a) Master BNO `captured/attempted` ≈ 100 Hz, (b) collection throughput ≥10× the ~1.8 KB/s baseline, (c) pulling a node's link mid-collection recovers within seconds without disturbing other nodes.

---

# Full-codebase review — open issue register (#6+)

**Reviewed:** 2026-08-21, branch `fix/Firmware_Fixes` @ `999cba4`. Scope: Master superloop, BLE central client, SD recorder/stager and collection coordinator; Node superloop, recording app, W25Q flash and upload sender; record protocol, ESOX v4 format, browser BLE contract; host tests. Every P1 below was confirmed by direct source read, not just search; `file:line` references are to this commit.

## Verified-fix audit

All four in-code fixes claimed above are present and correctly wired:

| Fix | Verified at |
|---|---|
| ICM FIFO armed at both node record-start sites, register-poll fallback | `NODE_RECORDING_APP.h:260` (delayed), `:499` (prepared/immediate); fallback `ICM45686_STM32.h:152-159` |
| `service_bno()` bounded drain + two extra superloop call points | `HUB_SENSOR_TEST_APP.h:133-138`; call sites `Master main.c:3000` (after `MX_APPE_Process`) and `:3061` (after `local_record_collect`); budget 8-packets-capturing / 2-idle, INT-gated |
| ACK serviced in the same iteration it was queued | `service_reliable_control` at `Master main.c:3004`, honors the post-manifest defer counter (`MASTER_TRAINING_CSV_COORDINATOR.h:272-279`) |
| Stager 4 KB RAM write buffer | `MASTER_NODE_SESSION_STAGER.h:548-550`; flush on full `:148-151`, validation open `:173`, shutdown/abandon `:327` |
| Targeted reconnect during discovery hold | `exo_hub_central_client.c:2090-2102` (request), `:1273-1311` (hold-branch execution), cap 3 + 250 ms settle `:74-75`, budget reset on connect `:1578-1579` / hold release `:2147-2148` |

All 9 Python invariant tests pass on this branch (re-run 2026-08-21).

## P1 issues

### #6 — Node flash erase runs inside the BLE command context and blocks the whole node *(gates 10-min goal)*
The Master drives nodes via Prepare→Commit (`Master main.c:1783-1784, 1860-1861`); the node's `prepare_recording()` erases the whole session region sector-by-sector synchronously inside the GATT-write dispatch (`NODE_RECORDER.h:68-79`, erase at `:73`; 4 KB sectors via `W25Q256_FLASH.h:125-136`). Erase time scales with session length: a 60 s session (~141 sectors) blocks ≈ 6 s typical (45 ms typ / 400 ms max per sector); a **10-min session (~1400 sectors) blocks ~60 s+**. While blocked the node runs nothing — no `MX_APPE_Process()`, no logging, no start heartbeats (1 Hz with a 5 s extend window, `Node main.c:846-857`). The legacy delayed-start path additionally latches `recording_started_ms_` *before* the erase and arms BNO/ICM capture only after it (`NODE_RECORDING_APP.h:243-263`), so those sessions also lose the first seconds of the window.
**Direction:** chunked background erase in the node superloop (N sectors per `process()` tick, heartbeats between chunks), or a pre-erased region pool. Verify against the Master's start-heartbeat budget.

### #7 — Master finalize/archive is a multi-pass full-file inline block
`local_record_collect` → `MasterSdSessionRecorder::finalize()` CRC-reads the whole payload (`MASTER_SD_SESSION_RECORDER.h:556-572`), then `archive_to_index()` copies the full file 256 B at a time (`:287-303`), validates (`:312`), renames (`:329`), re-opens and re-validates (`:337-350`) — all inline in the superloop with no yield points. At 10-min-session size (~5.8 MB) that is ≥3 full-file passes (~17 MB of I/O) with **zero BLE event dispatch**: incoming node chunks queue on the coprocessor and can overflow, ACKs stall, browser GATT operations time out. The fix pattern already exists — validation already runs chunked at 256 B per `service()` call (`MASTER_NODE_SESSION_STAGER.h:233-262`).
**Direction:** convert archive copy + validation to chunked state-machine steps like `step_validation`.

### #8 — Reliable-control frames can be silently dropped; callers ignore the failure
`queue_frame()` rejects a lower-priority frame while one is pending (NackRange=3 vs ManifestAck=4 / VerifyOk=5, `MASTER_NODE_RELIABLE_CONTROL.h:230-247`); every caller discards the result (`(void)` at `MASTER_TRAINING_CSV_COORDINATOR.h:242-250, 262`); the 300 ms window re-arm itself refuses while a frame is still pending (`:142-152`); a same-priority NACK silently overwrites a queued NACK. A corrupt/gap chunk arriving in the same BLE batch as the ManifestAck produces no recovery request; recovery then waits on the 300 ms re-arm or degrades to the 30 s stall abandon — the same failure shape as session-1 issue #4.
**Direction:** propagate queue failures (set a retry flag), or coalesce (let a NACK replace a pending ACK), and make `rearm_ack_window` robust to a busy queue.

### #9 — Abandoned/failed node stage files are never unlinked and poison run indexes
`NodeSessionStageOperation::Unlink` and `unlink_fn` exist in the stager vtable (`MASTER_NODE_SESSION_STAGER.h:63-67`) but no path ever calls them. `abandon_active_node` → `stager_.shutdown()` closes but keeps the truncated file (`:322-345`), and `index_occupied` treats any existing `R####N#.BIN` as a consumed index (`MASTER_BINARY_SESSION_INDEX.h:79-100`). Session 1 left exactly this artifact (0-byte `R0001N1.BIN`): every abandoned source permanently skips a run index and leaves a truncated file that looks like a final archive until fully parsed.
**Direction:** unlink the stage file on abandon/failure (the node's flash copy is untouched — no data is lost), or stage under a `.TMP` name and rename on success like the Master's own recorder does.

### #10 — SD-full / rename-failure latches and blocks all new Master sessions
`start()` refuses while `archive_required_` is set (`MASTER_SD_SESSION_RECORDER.h:24-30`); `reset()` cannot clear the flag if `service_archive_cleanup()` fails (`:396-404`); the 3 inline archive retries (`Master main.c:2740-2752`) leave it set on exhaustion, and later cleanup retries only the last recorded path. One SD-full event therefore refuses every subsequent StartRecord until a cleanup happens to succeed.
**Direction:** background cleanup retry with backoff + a cleanup/free-space status surfaced to the browser; distinguish "SD full — space needed" from transient I/O errors.

### #11 — No node crash recovery; finalize header rewrite has a corruption window
Power loss mid-record leaves `completion_flag = 0` and the header is never examined on boot (`NODE_RECORDING_APP.h:53-75`); the reserved `kRecoverySectorSize` (`:430-431`) is referenced nowhere in the tree. At finalize the header is rewritten in place (`NODE_RECORDER.h:138`), which the W25Q driver implements as read-sector → erase-sector → rewrite (`driver_w25qxx.c:8441-8468`) — and sector 0 also holds the first ~4 KB of BNO payload. Power loss inside that window corrupts both the header and already-stored payload; there is no boot-time salvage.
**Direction (needs a design spike):** finalize into a spare sector copy and swap pointers, or boot-scan for `completion_flag==0` with valid payload CRCs to offer salvage; at minimum make the header rewrite atomic (page-aligned header region).

### #12 — Node flash write failure at finalize is a permanent dead-end
`try_finalize_recording()` returns without clearing `record_finalize_pending_` while batches remain pending (`NODE_RECORDING_APP.h:775-777`); `flush_one_*` retries the same batch forever on write failure (`:668-706`); there is no retry cap or give-up path, BNO service is skipped while stuck (`:236`), and only Cancel (which also needs the flash) or a reboot clears the state.
**Direction:** bounded retries with escalation — log, fail the session cleanly, preserve what was captured for salvage.

### #13 — Transfer pacing ceiling ~22 KB/s sits just above the ≥18 KB/s target *(gates 10-min goal)*
`kNodeRecordChunkGapMs = 8` and `kNodeRecordBurstLimit = 1` (`Node main.c:304-305`, enforced `:696-703`) cap the link at 180 B / 8 ms ≈ **22.5 KB/s absolute**, before credit-round-trip effects (Master re-advertises credit-8 windows after each accepted chunk, `MASTER_TRAINING_CSV_COORDINATOR.h:262`). Even a perfect ACK path cannot exceed the ceiling; at 10-min-session size (~5.8 MB/node) that is ≥4.4 min/node at best. See also the caveat under Issue 5.
**Direction:** measure the `9a808c7` fix on hardware first; if short of target, tune burst limit (≥4) / gap (2-4 ms) / credit (16-24), then re-verify chunk-loss behavior at the new pacing.

## P2 issues

| # | Component | Issue | Evidence |
|---|---|---|---|
| #14 | Node upload | NackRange / VerifyFail / legacy ChunkAck chunk indices are unvalidated — an out-of-range cursor silently ends the upload (node goes mute, RecordDone not re-sent) | `Node main.c:1282, 1346, 635`; sender termination `:705-710` |
| #15 | Node ICM | 16-bit TMST wraps every 1.049 s; a stall ≥1.05 s yields a wrong (compressed) `offset_us` with no detection (sequence gaps still visible) | `ICM45686_STM32.h:204-221` |
| #16 | Node ICM | `read_fifo_samples()` returns 0 for both "FIFO empty" and "I²C error"; the finalize tail-drain treats a transient error as "caught the tail" and silently drops the ICM tail | `ICM45686_STM32.h:174-178`, `NODE_RECORDING_APP.h:762-771` |
| #17 | Node | A new StartRecord erases a retained ReadyForUpload session with no guard — next session started before pulling destroys the previous data | `NODE_RECORDING_APP.h:342-345`, `NODE_RECORDER.h:56-60` |
| #18 | Node flash | **FIXED 2026-08-22 (2nd pass)** — see "Resolved" section: public `w25qxx_write_no_check` + `write_pre_erased()` fast path. Original: driver re-reads the full 4 KB sector before every write although the region is pre-erased | `driver_w25qxx.c`, `W25Q256_FLASH.h`, `NODE_RECORDER.h` |
| #19 | Node DIAG | Boot flash self-test erases a sector inside the session region (default profile unaffected) | `Node main.c:1490`, `W25Q256_FLASH.h:190` |
| #20 | Master | Dead stale-ACK escape: the guard at `:4324-4332` returns on exactly the condition the escape at `:4338-4350` requires — unreachable code | `Master main.c:4324-4350` |
| #21 | Master | `ErrorStoredCanResume` is a dead-end phase — nothing retries or exits it except full reset; it also pins `g_ble_record_transfer_mode` and blocks node uploads | `Master main.c:3357-3362, 1160-1166, 3406-3415` |
| #22 | Master | Live-stream master switch not restored after a session — comment claims automatic resume, but only the gate clears; the 0xA0 switch stays off until the browser re-sends it | `Master main.c:2026, 2129, 3406-3415, 3519-3523` |
| #23 | Master | `ValidateNode` has no stall coverage in `service_finalize` (safe today — local and always progresses; future-proofing) | `MASTER_TRAINING_CSV_COORDINATOR.h:380-405` |
| #24 | Master | **FIXED 2026-08-22 (2nd pass)** — see "Resolved" section: boot-cached last index + persisted `RUNIDX.BIN` marker. Original: O(index×5) `f_stat` scan inside the BLE write handler + crash-window stranding | `MASTER_BINARY_SESSION_INDEX.h`, `Master main.c` |
| #25 | Protocol/browser | Browser NACK payload packs 12 B vs the 14 B C struct (`flags` field missing) and the node never reads the `CrcMismatch` flag the Master sets | `Exoskeleton.html:3106-3112`, `Node main.c:1268-1278`, `MASTER_TRAINING_CSV_COORDINATOR.h:249-250` |
| #26 | Protocol | Credit defaults disagree: protocol header 16, Master grants 8, browser presets 16/24, node re-arm 16 — functional (sanitized) but misleading for tuning | `BLE_RECORD_PROTOCOL.h:29`, `MASTER_TRAINING_CSV_COORDINATOR.h:66`, `Exoskeleton.html:1181-1228` |
| #27 | Master | **FIXED 2026-08-22 (2nd pass)** — see "Resolved" section: 8-deep RAM chunk queue, staging flushed from the superloop. Original: stager 4 KB flush + duplicate read-back in BLE notification context stalled dispatch for all 5 links | `MASTER_TRAINING_CSV_COORDINATOR.h` |
| #28 | Master | After a source resolves, a browser-relayed ManifestAck can re-grant chunk-0 credit to that node, restarting chunks into a coordinator that now ignores every frame | `Master main.c:4300-4306, 4367-4378`, `MASTER_TRAINING_CSV_COORDINATOR.h:230` |
| #29 | Master | End-to-end CRC32 mismatch after a full upload discards the entire staged file (StageError) with no selective re-pull — pairs with the Issue-#4 re-pull follow-up | `MASTER_NODE_SESSION_STAGER.h:264-281, 507-524` |

## Minor issues (P3)

- `start_timestamp_us` in StartSession/StartRecord is a relative lead time, not a timestamp, and is truncated to `uint32` ms on the node — rename or widen (`NODE_RECORDING_APP.h:240`, `Exoskeleton.html:2216`).
- Node-id set command decodes `payload[length-1]` on the blepipe lane but `payload[1]` on the legacy lane (`Node main.c:1152` vs `:2092`).
- BNO/ICM/TOUCH pins are configured `GPIO_MODE_IT_RISING` with no EXTI handler or NVIC enable — harmless while polled, a hang hazard if the IRQ is ever enabled (`Node gpio.c:62-66`).
- Legacy ChunkAck replenishes credit without clearing the retransmit cursor (`Node main.c:634-637`).
- Dead opcodes: ListSessions / FetchSession / EraseSession defined but unreferenced anywhere (`BLE_RECORD_PROTOCOL.h:11-16`).
- `MasterNodeTransferWindow::chunk_size_` is stored but never enforced; a zero-length chunk is silently `Ignore`d (`MASTER_NODE_TRANSFER_WINDOW.h:38, 53-91`).

## Verified non-defects (explicitly checked, sound)

- **32-bit tick wraparound:** all inspected deltas use unsigned subtraction or explicit casts (`MASTER_TRAINING_CSV_COORDINATOR.h:387-401`, node upload timers, central-client backoff timers).
- **Ingest buffer bounds:** blepipe decode verifies version/length/CRC; reliable-frame ingest checks exact `payload_len`; node upload reads are bounds-checked twice (`SESSION_TRANSFER.h:24-68`).
- **Stop delivery:** Master retry/ack-mask sync + idempotent node stop handler + duration-gate self-heal — a lost Stop cannot wedge a node.
- **ESOX v4 consistency:** 88/56/20-byte layouts agree exactly across C static_asserts, the Python converter, and the browser decoder; both `blepipe_proto.c` copies are byte-identical; all little-endian.
- **Concurrency model:** Master and Node are single-context (no RTOS) — sensor/flash/BLE all run in the main loop; no ISR races found (with the EXTI-config caveat above).
- **Window math:** `MasterNodeTransferWindow::inspect()` sequential decisions are off-by-one-free; Master-side credit accounting is absolute (no drift).

## Test-infrastructure gap

- The C++ host tests (transfer window, reliable control, stager, live queue, formatters) cannot execute on the dev box — `run_host_syntax_suite.ps1` is syntax-only and requires the ARM toolchain; behavioral regressions in window/credit logic are only catchable in CubeIDE on Windows.
- No runner/CI wiring even for the Python suite (all 9 files pass, re-run by hand 2026-08-21).
- `blepipe_proto.c` is compiled everywhere but never unit-tested (CRC, lane allow-list, trim path).
- **Recommendation:** a host-runnable g++ target (the FatFs/BLE stubs already exist under `HostTests/FatFsStub`) plus one `run_all` script — near-zero firmware risk and immediate regression coverage for the upcoming #8/#9/#13 fixes.

## Staged fix roadmap (smallest-first; one commit + patch per issue; hardware checkpoint after each stage)

1. **#8** — reliable-control queue-failure propagation (small; removes a known silent-loss path).
2. **#9** — unlink abandoned node stage files (small; restores run-index hygiene).
3. **Issue-#4 follow-up** — re-pull command for `failed_source_mask` nodes (medium).
4. **#10** — SD-full latch recovery (medium).
5. **#12** — node finalize bounded retry (small-medium).
6. **#13** — pacing tune, *after* hardware throughput measurement of the `9a808c7` fix (config + verification).
7. **#6** — node background/chunked erase (largest node change; **gates 10-min sessions**).
8. **#7** — Master chunked archive (largest Master change).
9. **#11** — node crash recovery / atomic finalize header (design spike first).
10. **P2 batch (#14-#29)** after the P1s, cheapest first (#14, #20, #22, #26 are near-trivial).

---

# Fix campaign applied 2026-08-22 (branch `fix/Firmware_Fixes`)

The roadmap above was executed end-to-end. Every fix is in code with Python invariant guards (all 9 test files re-run green after each commit); C++ host tests updated in sync but **not compiled anywhere yet** — build in CubeIDE and run them before hardware bring-up.

## Commit map

| Commit | Issues fixed |
|---|---|
| `9cd8783` nack_slot_fix | #8 — NACK owns a dedicated slot in `MasterNodeReliableControl`, serviced before ACK windows; can never be rejected by a queued ManifestAck/VerifyOk |
| `4d5f228` stage_unlink_on_abandon | #9 — `abandon_and_unlink()` (guarded by `stage_started_`/validated) wired into coordinator abandon/finalize/shutdown; validated archives are never unlinked |
| `b47381f` failed_source_repull | Issue-#4 follow-up — `RetrySource 0x10` browser→Master command, `coordinator.retry_failed_source()`, retained-RecordDone requeue, node resumes from its flash copy; "Retry Failed Nodes" button in the browser (Error/Incomplete states) |
| `66a1d6f` chunked_archive_and_recovery | #7 + #10 — Master finalize (payload CRC) and archive (copy/validate/rename/re-validate) advance in 512 B superloop steps (`Finalizing`/`Archiving` phases); latched `archive_required_` self-heals via `service_archive_recovery()` with 5 s backoff, also from the `Finished` phase |
| `ca6cc38` node_finalize_bounded_retry | #12 — 8-failure write streak drops pending batches (counted once); 3 failed finalize attempts latch `finalize_failed_` and leave the node responsive instead of wedging in Recording |
| `557e343` node_upload_burst_tune | #13 — `kNodeRecordBurstLimit` 1→4 (ceiling ~22 → ~90 KB/s; credit-8 round-trips remain the effective limiter). **Measure on hardware; tune credit/gap next if short of ≥10×** |
| `3b298ad` node_background_erase | #6 — chunked 4 KB-sector eraser serviced from `process()` (2 sectors/tick), prepare/start/commit buffered behind it, post-ack erase runs in background, pre-erased regions skip the erase entirely. First-boot/long-layout sessions start once the erase covers the region (delayed start, not a blocked node) |
| `2adf6ab` recovery_sector_header | #11 — session headers live in the reserved recovery sector (slot A in-progress, slot B finalized via single page program — the payload erase window is gone); `recover_after_boot()` boots a finalized session straight to ReadyForUpload (survives crashes incl. mid-upload) |
| `3ca45ab` node_p2_robustness | #14 (validated inbound chunk indices), #15 (TMST wrap → nominal-period substitution), #16 (I²C-error ≠ empty-FIFO at the tail, 5-retry budget), #17 (plain StartRecord can no longer wipe a retained ReadyForUpload session), #19 (DIAG self-test moved to the recovery sector), #18-half (SPI1 /16→/4 = 8 MHz), P3 node-id lane alignment + legacy-ACK retx release |
| `2fb2218` master_protocol_p2 | #20 (stale-ACK escape now reachable), #21 (bounded 8-retry SD read failures instead of instant dead-end), #22 (live-stream switch latched and restored at collection end), #23 (ValidateNode stall coverage), #25 (browser NACK packs the 14-B flags field), #26 (protocol default credit = 8, matches the grant), #28 (resolved sources no longer re-granted ManifestAck credit), #29 (CRC-mismatch StageError marks the source failed and is re-pullable in binary-only runs), P3 dead opcodes removed + transfer-window chunk-size bound enforced |
| `77b7927` review_repull_wedge_fix | Review finding P1: retained-done `valid` flag must survive replay or the re-pull wedges the scheduler; review P3: `start_erase_pending_` cleared on abort |
| `52e547b` master_link_overflow_patch | Master stopped linking: `.bss` +1880 B and FLASH +497 B over region caps. RAM region restored to full 128K SRAM1 (`0xFFF8 → 0x1FFF8`); 4 unused statics dropped from `main.c`; ~50 `EXO_LOG` literals compressed (every field kept) → links clean with 539 B flash margin. Details below |
| `200bad5` node_build_fix_patch | First Node compile of the campaign exposed that #16 accessed private `Icm45686Stm32::last_read_status_`; added a public `last_read_status()` getter and switched the call site. Also dropped an unused `node_blepipe_send_record_payload` and fixed a narrowing warning in Node `main.c`. Node links clean (79.3 KB flash / 17.9 KB RAM used — no memory pressure); Master rebuild byte-identical |

## Linker memory budget (Master, after `52e547b`)

Both project linker scripts have carried undersized MEMORY caps since the repo's first commit — `FLASH = 124K`, `RAM = 64K−8` — on an STM32WB55CCU6 (256 KB flash, 128 KB SRAM1). Nothing documents those numbers as deliberate (no bootloader/OTA layout anywhere), but they were harmless until this campaign added ~7.7 KB of statics (the 4 KB stager write buffer from the throughput fix, ~3.6 KB BNO85 queue depth 64) and the code growth pushed the image past both caps.

- **RAM**: `LENGTH = 0x1FFF8` restores all of SRAM1. The CPU2-shared areas already live in SRAM2 (`MAPPING_TABLE`, `MB_MEM1/2` @ `0x20030000`), so the upper 64 KB is pure application RAM; `_estack` now sits at `0x2001FFF8`.
- **FLASH**: the 124K cap was *kept* on purpose — the FUS + wireless stack binaries occupy the top of internal flash on STM32WB, so free user flash < 256 K and 124 K is the only value known-good on this hardware. Instead the image was trimmed ≥1 KB: dead statics (`master_ble_session_id`, `send_record_lane_v3`, `rad_to_mdeg`, `clamp_i16` — already GC'd, zero binary change) and wording compression of the longest log literals in `main.c` / `exo_hub_central_client.c` (format specifiers and field order untouched; no desktop tool parses these tags).

| Region | Limit | Used | Margin |
|---|---|---|---|
| FLASH (cap kept at 124K) | 126,976 B | 126,437 B | **539 B** |
| RAM (`.data`+`.bss`, now 128K−8) | 131,064 B | 65,872 B | **~65 KB** |

**Watch items:** (1) ~~`Firmware/Node/*.ld` still carries the same undersized caps~~ — **fixed 2026-08-22**: `Node/STM32WB55CCUX_FLASH.ld` RAM `0xFFF8 → 0x1FFF8` (mirrors `52e547b`); FLASH cap kept at 124K. (2) The 539 B Master flash margin is thin; if it shrinks again, verify the true free window above the app with STM32CubeProgrammer (find where the installed FUS/stack begins) and raise the 124K cap deliberately rather than trimming more logs.

## Deferred with rationale

- **P3 `start_timestamp_us` naming** — wontfix: pure naming (relative lead, not a timestamp); a rename touches browser + firmware + tests for zero behavior change.
- **P3 latent EXTI config** — wontfix: pins are CubeMX-generated (`gpio.c`), NVIC is not enabled, and the BNO INT is polled; documented here as a hazard for anyone enabling the IRQ.

## Resolved 2026-08-22 (second pass: deferred items closed)

- **#24 (run-index allocation)** — fixed: `MasterBinarySessionIndex` now warms a boot-time cache (`init_cache()` after FATFS init in `main.c`): reads a persisted 8-byte marker `/SESSIONS/RUNIDX.BIN` (magic "INDX" + last index); if missing/corrupt, runs the full occupancy scan once and rewrites the marker. `allocate()` is now a handful of `f_stat`s from `cached_last_+1`, persists the marker at hand-out, and falls back lazily if the card was absent at boot. Crash semantics: an index is never re-issued once handed out (stranded indexes possible after a crash between allocation and archive, matching the designed fix). The `test_binary_first_invariants.py` allocator guard was refined: `FA_CREATE_ALWAYS` allowed only for the RUNIDX.BIN marker, never payload files.
- **#27 (SD I/O inside BLE dispatch)** — fixed via the RAM-queue refactor: `MasterTrainingCsvCoordinator::on_node_reliable_frame` (BLE ingest context) now validates + commits the transfer window + ACKs and copies the ≤180 B chunk into an 8-deep `PendingChunk` RAM queue (≈1.6 KB); the actual `stager_.accept_chunk` staging — including the every-4 KB flush and duplicate read-backs — runs in `drain_pending_chunks()` from the superloop `service()`. Queue-full falls back to synchronous staging so no chunk is lost; shutdown/abandon/finalize paths reset the queue. The ValidateNode transition happens only after the queue drains. Remaining BLE-context SD work: the 8-byte marker write in `allocate()` (tiny) and the phone-write-handler shutdown flush (superloop-adjacent, unchanged).
- **#18 (driver RMW re-read)** — fixed: public `w25qxx_write_no_check()` added to the vendored driver (wraps the existing static page-programmer); `SessionFlash::write_pre_erased()` (default falls back to `write()`), overridden in `W25Q256Flash` to verify just the target span is 0xFF (64 B chunks — no 4 KB sector RMW) and page-program directly, falling back to the safe RMW path if not blank. `NodeRecorder::append_samples()` uses the fast path (payload regions are pre-erased by the background eraser); header/finalize/diagnostic writes keep the safe path.
- **Node linker RAM cap** — `Node/STM32WB55CCUX_FLASH.ld` RAM `0xFFF8 → 0x1FFF8` (full 128K SRAM1, mirrors `52e547b` on Master); `_RAM.ld` untouched; `size_budget.md` updated.

All four await CubeIDE compilation + the hardware verification checklist below (no ARM compiler on the dev box — syntax suite could not run).

## Verification checklist (unchanged gates, now with the new code)

1. **CubeIDE build** — Master + Node compile clean; run the C++ host tests (`test_master_node_reliable_control`, `test_master_node_session_stager_sequential`, `test_master_node_transfer_window`, `master_sd_session_recorder_test`, `ble_session_control_test`).
2. **Hardware throughput** — confirm collection ≥10× the ~1.8 KB/s baseline (burst-4 + same-iteration ACKs should land ~15-40 KB/s; if short, raise credit toward 16-24 next).
3. **Erase timing** — first-boot 10-min session: node prepare accepts in <100 ms, RecordDone/prepared status flows during the background erase, capture starts when it completes; second session on the same node starts instantly (pre-erased region).
4. **Crash recovery** — power-cycle a node after finalize (mid-upload): on boot it logs the salvaged session and re-advertises RecordDone.
5. **Re-pull** — stall a node mid-collection (shield it), let the run finish, press "Retry Failed Nodes" in the browser: the session re-uploads and the run completes.
6. **SD-full** — fill the card, confirm the archive failure is reported, free space, and confirm the next session starts without a reboot.
