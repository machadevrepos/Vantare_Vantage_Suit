# DATASET_ACQUISITION branch — fresh project context

> **CURRENT STATE — consolidated 2026-09-10.** Read this block first; everything below it
> (verified 2026-09-07) remains accurate as deep background on firmware/BLE/dataset
> plumbing, but the *direction* has pivoted since.
>
> **Focus: Coach Assist Motion Engine (Milestone 4)** — deterministic host-side kinematics
> in `host/live_tool/js/motion-engine.js`; contract + scope limits in
> `docs/architecture/motion-engine-contract.md`. Calibration maps sensors to **body
> segments** (N4 = upper arm, N2 = forearm, N3 = elbow validation + haptic site; master
> IMU = torso reference only), never to exercises; bicep curl is only the first
> validation target. First live validation **PASSED 2026-09-10**: calibration first try,
> 8 clean curl episodes (elbow 0.3→142.5°), upper-arm deviation ≤8.5°, sync gate fired
> only on real transport stalls; the motion path runs with no model loaded.
>
> **Roadmap from here:** pose battery → re-strap repeatability → yaw-drift logging
> (`yawDriftHintDeg` still UI-only, not in the NDJSON log) → hand the motion packet
> contract to the 3D/app team.
>
> **Status of older threads:**
> - Transport: **solved** — 25 Hz qualification PASS, 100% of 121 windows, ~8.9 KB/s
>   (fix header + §3 below).
> - AI model: **deprioritized by design.** The constant-output issue (header below) and
>   the session==class / mounting-leak diagnosis (§4.2) stay open but are off the
>   critical path. Model V2 + Phase 0 plan parked at
>   `docs/superpowers/specs/2026-09-07-bicep-curl-model-v2-design.md`.
> - Throughput: live wall ~9.8 KB/s shared; record upload measured ~7.2 KB/s per node
>   with root cause diagnosed (§3.6) — fix only if a measured blocker demands it.
> - P1 (Node notification-completion 0x08) still open; P2/P3 list in §5.
> - RS485: dormant on the `wt-work` worktree (wire transport HW-proven, ~37 commits, not pushed).
> - Parallel track, **awaiting user adoption**: 12-node architecture (WB55 hard cap =
>   6 central + 2 peripheral per radio; recommended WBA65-master PoC gate; node PCBs
>   orderable now, master choice does not gate them). Mobile app (Flutter) not started —
>   biggest contract gap; the client timeline has lapsed and a re-baseline conversation
>   is owed.
>
> **How to resume in a new chat:** AGENTS.md loads automatically and points here. Prior
> sessions do not need reopening — their durable results live in this doc,
> `docs/architecture/firmware-issues-and-fixes.md`, `docs/architecture/motion-engine-contract.md`,
> or agent memory. The old Claude-import deep-dive on `exo_hub_central_client.cpp` is
> fully folded into §3/§5.

Verified 2026-09-07 against `DATASET_ACQUISITION` @ `1820ad0` ("25Hz_with_qualification_fail").
Originally produced by a four-agent review sweep (project architecture, BLE subsystem,
dataset/live-model pipeline, firmware risk review); **re-verified 2026-09-07 by a second
three-agent sweep** (BLE subsystem deep-check, live-model/dataset deep-check, independent
firmware risk re-review) whose corrections and new findings are folded in below. All
`file:line` references are against this checkout. Working focus going forward: **BLE
communication improvements and live model testing**.

> **Fixes applied 2026-09-07 (25 Hz qualification — `interp_span` failure).** Root cause: transport
> timing jitter, not lost data (forward path was clean — `fwdF=0 pend=0 pciMs=15`). Four changes,
> source-only, awaiting a target build:
> 1. **Browser `interp_span` gate** `1.5T → 2.5T` soft, budget `2% → 10%` of window, + `4T` hard cap; staleness `3T → 4T`; qualification inter-packet gate now on the frame `time_ms` spacing (`maxSrcGapMs`), not browser arrival (`host/live_tool/js/ml-preprocessing.js`, `main.js`, `ble-protocol.js`).
> 2. **Node live-queue admit gate** `interval*3/4 (30 ms) → interval/2 (20 ms)` — the 30 ms admit vs 36 ms bundle pacer left the queue empty at ~1.3 ticks/s, skipping a bundle → ~1.5–2T gap (`node_live_sample_queue.h:50`).
> 3. **Node bundle reschedule** on empty-queue skip / backpressure: `now + interval → now + interval/2` (`Node/Core/Src/main.cpp:531,543,577`).
> 4. **Master B1 `time_ms` stamped at leaf-notification ingest**, not forward time — decouples the browser grid from TX-pool + Chrome coalescing jitter (`hub_leaf_ble_manager.h` `LiveSample::recv_ms`, `Master/Core/Src/main.cpp` `send_ble_v2_sample_status` / `exo_hub_leaf_stream_ingest`). Design §7.1.
> 5. **Master live diag frame suppressed while the live forward queue is backed up** (forced every 5 s so telemetry survives) — a 244 B status LOG frame on the shared browser TX pool was stealing forwarder slots (`master_blepipe_send_live_diag`).
>
> **Second build (08:34 run):** all six per-stream gates + skew PASS (source gaps 100–140 ms); `valid windows` still 0% but the reason flipped — window 1 `interp_span_n2s1`, then **155/156 `loss_n2s1`**. The per-window loss gate used the B1 `sequence` delta, which is a Master-wide counter (~250 "missing"/window at 27 Hz); it was just masked by `interp_span` failing first. **Fix 6:** loss gate rewritten as a sample-count deficit (`spanIntervals − realIntervals > maxMissing`), not sequence continuity. Interp budget raised to 12%.
>
> **Third build (08:58 run) — qualification PASS: 100% of 121 windows.** All six streams PASS (source gaps 83–100 ms, arrival 130–148 ms), skew 4.0 ms, 8.9 KB/s. Fix 5 worked — n4 source gap dropped 140→83 ms. **Transport is solved.**
>
> **Open — model output is constant.** Every one of ~140 predictions across a 73 s run is byte-identical: `classId 1 (incomplete_range)`, probs `[0.214, 0.633, 0.153]`, to 15 decimals. Either the run was performed holding still, or the v1_1 model is degenerate for this mounting (the session==class training confound, §4.2 — fix is more sessions, not code) or the feature vector is constant (preprocessing/decode). Needs: what the wearer was doing during the run, + a feature-vector dump for 3–5 windows to tell "constant features" from "model collapse".
>
> Independent of firmware: the Chrome tab must stay foregrounded for the 60 s qualification (early runs had `page was suspended` ×3). Still open: leaf links at `st=7 Degraded`.

---

## 1. Project truth

| Item | Value | Evidence |
|---|---|---|
| Master MCU | STM32WB55CCU6 (`Master.ioc`); physical silicon is 1 MB G-grade (SFSA=0xD0 measured), .ioc deliberately left CC-grade; linker budgets 768 K flash / 192 K RAM | `firmware/common/build/linker/STM32WB55_FLASH.ld:47-52` |
| Node MCU | STM32WB55CCU6 (`Node.ioc`), same linker script | |
| BLE stack | STM32_WPAN (CPU2 coprocessor via FUS), Cube package STM32Cube FW_WB **V1.24.0**, HAL **V1.14.7** | `Master.ioc`, HAL sources |
| RTOS | **None.** Single-context superloop per side; `UTIL_Sequencer` only pumps BLE/System-HCI events via `MX_APPE_Process()` | `Core/Src/app_entry.cpp:318,550` both sides |
| Watchdog | **No IWDG** (disabled in both `stm32wbxx_hal_conf.h`). All "watchdogs" in BLE paths are software timers | reviewer sweep |
| Build | STM32CubeIDE managed projects; current artifacts: `firmware/Master/Debug/Master.elf`, `firmware/Node/Debug/Node.elf` (Release stale/absent). Root `CMakeLists.txt` builds host tests only (`host/tests/cpp`) | |
| Tests | `firmware/Master/tests` (11 cpp + 10 ps1 source-invariants), `firmware/Node/tests` (1 cpp + 4 ps1), `host/tests/python` (15 pytest), `host/tests/cpp` (14 ctest) | |
| Transport | **BLE only.** RS485 was removed (ble-only cleanup complete); RS485 strings survive only in legacy utility comments (`exo/utils/neoway.h`, `exo/sensors/can.h:7`) | |
| Sources | C++ (`.cpp`) incl. CubeMX-generated files; shared app logic is header-only under `firmware/common/inc/exo/` so host tests compile it | |
| VERSION | 1.0.0 (root `VERSION`; no FW-internal version string). Live-tool page build string `LIVE_TOOL_BUILD` (`host/live_tool/js/live-inference.js:17`) = `"2026-09-03.12"` | |
| Git | Branch `DATASET_ACQUISITION` in sync with origin; second worktree `Vantare_Vantage_Suit_worktree` (branch `wt-work`) exists but is **dormant — current work is on the main checkout** | |

## 2. System architecture

- **Master ("HUB0001")** — dual BLE role:
  - GATT **client/central** to up to 4 sensor Nodes: `firmware/Master/Core/Src/ble/exo_hub_central_client.cpp`.
  - GATT **server/peripheral** to one Web Bluetooth browser link: `app_ble.cpp`, `custom_stm.cpp`, `custom_app.cpp`.
  - `CFG_BLE_NUM_LINK = 6` (4 nodes + browser + 1 spare): `Master/Core/Inc/app_conf.h:197`.
  - Own IMUs (BNO085 @ I2C3 0x4B, ICM45686 @ I2C1 0x69) + SD/FATFS recording. Main app: `Core/Src/main.cpp` (5341 lines, superloop at :3444).
- **Node ("L00xx")** — peripheral, `CFG_BLE_NUM_LINK = 2` (`Node/Core/Inc/app_conf.h:192`). Two IMUs each:
  - **BNO085** (sh2/SHTP over I2C): game-rotation anchor 100 Hz + linear-accel/gravity/gyro aux 50 Hz; float quaternion SI units; `Bno85SampleV3` 56 B (`exo/types/recording_types.h:113-128`).
  - **ICM-45686**: 200 Hz accel+gyro hardware FIFO for recording (5 ms register-poll fallback); raw int16; `Icm45686SampleV4` 20 B. Scaling done on the consumer side (browser JS / CSV formatter): accel `raw*4/32768` g, gyro `raw*2000/32768` dps (`host/desktop_tool/vantage_bin_to_csv.py:277-278`, byte-identical to `host/live_tool/js/ble-protocol.js:224-229`).
  - Storage: W25Q256 32 MB NOR SPI flash, ESOX v4 binary session format; erases only after Master ACK. Main app: `Core/Src/main.cpp` (2558 lines, superloop at :2080).
- **Shared header-only library** `firmware/common/inc/exo/`: `protocol/` (blepipe, record protocol, reliable control, transfer window, stream v2, tuning), `ble/` (central client, leaf bridge/manager, link tune, upload pump, notification gate), `sensors/` (bno85/icm45686 drivers, formatters), `recording/`, `storage/`, `actuator/` (haptic), `types/`, `utils/`.
- **Vendor/generated boundary**: `Drivers/` (HAL), `Middlewares/ST/STM32_WPAN`, FatFs, `Utilities/{lpm,sequencer}`, `third_party/{w25qxx,icm45686-driver,sh2}`, CubeMX peripheral init + `custom_stm.cpp` attribute tables (note: `custom_stm.cpp` is WPAN-template code — changes there are owned deviations, see §5 P1).
- **No RTOS tasks**: everything is cooperative services in the superloop. Master services: BLE process → own-IMU acquisition → SD finalize/archive (stepped) → training-CSV coordinator + chunk drain → live preview passthrough (round-robin) → recovery queues → power/touch. Node services: BLE process → sensor capture → recording app (flash batches, background 4 KB-sector eraser) → live-sample sender / upload pump → haptics → link tune → power/touch.
- **Concurrency model**: leaf-RX ACI callbacks and `drain_leaf_stream_passthrough` both run in the `MX_APPE_Process` sequencer context — single-threaded cooperative, so `mutable` peek-time mutation of round-robin cursors is safe. HAL_GetTick wrap is handled everywhere via `(uint32_t)(a-b)` / `(int32_t)(a-b)` forms.

## 3. BLE communication subsystem

### 3.1 GATT service (identical layout Master-browser and Node)
Custom "BLEPipe" service, UUID base `3F8810xx-B4A5-4F7C-9B60-98E0B5C8A000`:

| Char | UUID | Props | Use |
|---|---|---|---|
| PipeDataTx | …1001 | notify | data lane (244 B) |
| PipeControlRx | …1002 | write/write-nr | commands in |
| PipeControlTx | …1003 | notify+indicate | command ACKs |
| PipeStatusTx | …1004 | read+notify | status/log/recovery |
| PipeConfigRw | …1005 | read+write | config |

- Char-creation call sites: Master `Master/Core/Src/ble/custom_stm.cpp:562-678`, Node `Node/Core/Src/ble/custom_stm.cpp:523-644` (UUID macros are at `:115-120` — **that is the line the earlier draft mis-cited for the event masks**).
- GATT event-mask arg per char (vendor values from `ble_defs.h:390-393`: ATTRIBUTE_WRITE 0x01, WRITE_REQ_AND_WAIT 0x02, READ_REQ_AND_WAIT 0x04, **NOTIFICATION_COMPLETION 0x08**):
  - Master **PipeDataTx = 0x0F** (`custom_stm.cpp:571`, includes the 0x08 completion bit — load-bearing for browser-side flow control). All other Master chars = 0x07 (`:597,623,649,675`).
  - **Node — every char = 0x07** (`Node/Core/Src/ble/custom_stm.cpp:529,555,581,607,633`); the 0x08 bit is absent on Node PipeDataTx (P1, §5).
  - Attribute-table divergence to record with the P1 fix: Node PipeDataTx is `CHAR_VALUE_LEN_VARIABLE` (`custom_stm.cpp:531`) vs Master `CHAR_VALUE_LEN_CONSTANT` (`:573`); both size 244. Both are edits outside USER CODE regions.
- Link config both sides: MTU 247 (`app_conf.h:218`), DLE suggested 251 / `0x0848` (`link_tune_state.h:45-46`), 2M PHY requested per link (`:47`), `CFG_BLE_MBLOCK_EXTRA = 16` extra controller TX buffers (Master `app_conf.h:247`), adv intervals `0x0080`–`0x00A0` = 80–100 ms (`app_conf.h:67-68`).

### 3.2 blepipe envelope + opcodes
20 B header (`BLEPIPE_HDR_LEN`) + payload ≤222 B (`244 − 20 − 2`) + CRC16-CCITT (`exo/protocol/blepipe_proto.h:11-15,86-96`). JS mirror `host/live_tool/js/ble-protocol.js:80-92,138-186` (the C struct `blepipe_hdr_t` is physically 18 B; JS writes 2 zero pad bytes at offset 18-19 to fill `HDR_LEN=20`).

Message types (`blepipe_proto.h:32-58`) — **fuller list than the earlier draft**: `0x01` LEAF_SAMPLE, `0x02` HUB_AGGREGATE, `0x03` RAW_FORWARD (live bundle marker), `0x10`-`0x13` command/ack/nack family (`0x12` ACK, `0x13` NACK), `0x20`-`0x25` status/topology/log family (`0x21` TOPOLOGY, `0x22` **LINK_STATS**, `0x24` LOG), `0x30` TIME_SYNC, `0x31` CONFIG_SET, `0x32` STREAM_CONTROL, `0x40`-`0x44` CONFIG_READ/WRITE/ROUTING_TABLE/DEVICE_INFO/STREAM_PROFILE. (It is **not** a contiguous `0x10`-`0x25` block.)

Payload opcodes (payload[0]):

| Opcode | Meaning |
|---|---|
| `0xA0` / `0xA1` | start / stop live stream (arms fast link timing; `0xA1` also clears the live-priority latch) |
| `0xA2` | set stream interval ms (**Master clamps 10–100**, `Master/Core/Src/main.cpp:228-229`, applied :2452-2455 / :4725-4728; **Node queue clamps 40–80**, `node_live_sample_queue.h:22-23,131-139`) |
| `0xA3`–`0xA7` | ERM %, buzzer %, RGB, touch test, Node-owned bounded haptic pulse |
| `0xA8` | Master own-IMU preview on/off (~2.6 KB/s overhead when on); `0xA8 0` also sets the live-priority latch |
| `0xB0`/`0xB1` | provision / query node id |
| `0xB2` | force rediscovery |
| `0xB3` | **reset all recording/transfer state to Idle + erase session data** |
| `0xB4` | discovered-nodes report |
| `0xB5` | transfer tuning wire — fast-interval byte in 1.25 ms units, supported set `{24,12,9,6}` = 30 / 15 / 11.25 / 7.5 ms, default 12 (15 ms), `kBulkFastInterval = 24` (30 ms) (`record_transfer_tuning.h:20-39`) |
| `0x01`–`0x10` | record session control — Start / RecordDone / Chunk / Ack / Prepare(`0x0B`) / Commit(`0x0C`) / Abort / Stop(`0x0E`) / StartSession(`0x0F`) / RetrySource(`0x10`) (`ble_record_protocol.h:8-22`) |
| `0xE1`/`0xE2` | Master→browser ACK/report markers |

De-facto protocol spec = the headers in `firmware/common/inc/exo/protocol/` + byte-exact JS mirror `host/live_tool/js/ble-protocol.js`. Cross-check in the 2026-09-07 sweep found **no byte-level drift** on blepipe header, B1 v2 envelope, message types, or sensor ids.

### 3.3 Live streaming data path (25 Hz contract)
1. Sensors run at native 100/200 Hz; `NodeLiveSampleQueue` (**depth 16** — `node_recording_app.h:1179`, `NodeLiveSampleQueue<kMaxLivePayload,16U>`; was `12U` before this HEAD commit, design doc still says 8) produces the 25 Hz live contract. Interval clamp [40, 80] ms; **3/4-interval decimation admit-gate** `gate_ms = interval_ms_*3/4` (`node_live_sample_queue.h:50-56`, introduced this HEAD commit — BNO 100 Hz jitter was dropping ~1-in-12 against the strict 40 ms grid). **Software gating, not a hardware timer.**
2. On queue overflow the ring **drops the OLDEST and keeps the freshest 16** (`node_live_sample_queue.h:60-67` — "a recent gap is far better than throttling"). *(Earlier draft said "drop-freshest" — that is backwards.)*
3. Node sends one bundled notification per `interval×0.9` (floor 18 ms → ~36 ms at the 40 ms contract): `[0x03][bno_len][56 B BNO][icm_len][20 B ICM]` as `BLEPIPE_MSG_LEAF_SAMPLE` (`Node/Core/Src/main.cpp:365,499-501,543-559`). The bundle sender keeps only the freshest sample per sensor (`:511-555`) so a reconnect backlog cannot replay-storm. Upload pump active ⇒ live preview suppressed (`node_upload_pump.h:165` `live_preview_suppressed()`).
4. Master splits bundle (every field length-checked, uint16 offset math, `push_leaf_sample` rejects payload > 96 — `exo_hub_central_client.cpp:1471-1492`, `hub_leaf_ble_manager.h:71-74`), ingests into per-(node,sensor) slots: `live_slot_index = (node_id-1)*2 + (sensor_id-1)`, **node_id hard-range 1–4** (`hub_leaf_ble_manager.h:394,436-438`), depth 8, drop counted (`:399-414`).
5. Master forwards to browser as **B1 v2 envelopes** (14 B header, `exo/protocol/ble_stream_v2.h:19-31`), burst ≤8 per pass (`kLiveForwardBurstMax = 8`, `Master/Core/Src/main.cpp:1011`) until `INSUFFICIENT_RESOURCES`, gated by `BleNotificationGate` (10 ms watchdog, `notification_gate.h:33`) — `Master/Core/Src/main.cpp:1013-1069`. Node→master LINK_STATS re-emitted to the page ~1 Hz for SWO-free debugging.
6. **Master re-stamps every forwarded frame** with its own `HAL_GetTick()` *at forward time* (after burst/queue/gate delay) and a **single global `g_ble_sequence` counter shared across all streams** — `Master/Core/Src/main.cpp:343,990`. The node's own `time_ms` / `offset_us` (BNO) / `sequence` (ICM) **are present in the live payload** (`ble-protocol.js:213,231`) **but the live pipeline discards them** — `onNotify` passes the Master tick as `nodeTimeMs` into `TimeBase` (`ble-protocol.js:691-704`). Consequence: cross-node skew is structurally un-measurable on the current wire (see §4.3). Forwarding the already-present `offset_us`/`sequence` is a cheap BLE-comms improvement (§6).

### 3.4 Flow control (what actually exists — no token handshake)
- `BleNotificationGate` (`exo/ble/notification_gate.h`): one notification in flight per characteristic, reopened by notification-complete / TX-pool events, 10 ms watchdog safety net. Watchdog is **recovery-only**; credit exhaustion sets `send_ready_=false` *without* arming the watchdog (correct — not a lost-event condition). Used by Node live sender and Master forwarder.
- `NodeUploadPump` (`exo/ble/node_upload_pump.h`): credit pump for record upload, event-driven with **750 ms recovery watchdog** (`:38`), burst **64** foreground (`Node/Core/Src/main.cpp:326`). `OtherFailure` stops the pump and preserves the cursor for external resume.
- Reliable record transfer: sender credit **24** (`ble_record_protocol.h:41`), Master grants **8**/ACK window (`master_node_reliable_control.h:210`, sanitize cap 24 at `:217`), ACK batching **16 chunks / 750 ms** (`master_training_csv_coordinator.h:72-73`), ACK keepalive **300 ms** (`master_node_reliable_control.h:22`), NACK priority slot, retry 20 ms; sequential window validation `MasterNodeTransferWindow` (uint64 offset/size math, final-chunk exact-size check); browser-side ack/pause/resume reflected via `HubLeafBleManager`.
- **Two different chunk sizes — do not conflate:**
  - **Node→Master reliable transfer: 192 B payload, ceiling 197 B** (`244 − 22 blepipe − 25 reliable header`) — larger values are *silently dropped* by the stack (`ble_record_protocol.h:26-35`, static_assert `Node/Core/Src/main.cpp:317-319`).
  - **Master local (SD→browser) transfer: 180 B chunks** — `kRecordChunkPayloadBytes = 180` (`Master/Core/Src/main.cpp:230`); `offset = chunk_index * 180` (`:3909`). This is the path with the uint32-wrap P2s (§5).
- Live-priority latch `g_live_stream_priority` (set by `0xA8`-off + `0xA0`): while live, node RecordDone is ACKed but ignored so uploads can't steal the link (`Master/Core/Src/main.cpp:1510-1526, 4483-4491`), and the `!g_live_stream_priority` gate at `:3822` blocks the whole node-upload scheduler. Cleared only by explicit `0xA1` / `0xA8`-on / `0xB3` — **not** by browser disconnect (§5 new P2).

### 3.5 Connection management
- Scan 40 ms / 30 ms (`EXO_HUB_SCAN_INTERVAL 0x0040` / `EXO_HUB_SCAN_WINDOW 0x0030`, `exo_hub_central_client.cpp:67-68`); **while-connected `0x00A0`/`0x0010` = 100 ms / 10 ms** (`:69` — *earlier draft's "160 ms" was the raw decimal of 0xA0, not milliseconds*); 5 s scan windows (`:96`); targeted reconnect (direct connect, 250 ms settle `:91`, max 3 attempts `:92`) under `discovery_hold` during sessions/transfers; established links never torn down by the hold.
- First central link requests 7.5–10 ms interval (`0x0006/0x0008`, shapes WB scheduler); subsequent links 30–50 ms (`0x0018/0x0028`) — `exo_hub_central_client.cpp:71-75`.
- `LinkTuneState` (`exo/ble/link_tune_state.h`): one serialized LL procedure across all leaves (`active_` slot, `:189-216`, lowest slot index first); DLE → PHY → interval state machine; **states `0 NeedDle … 6 Ready, 7 Degraded, 8 Failed`** (`:18-28`); fallback ladder on STM32WB rejections 0x84/0x85/0x86 (`:79-81,245-262`), capped at `kMaxTransientAttempts = 4` then parked in `Degraded`; bulk interval 30 ms + parking idle leaves at 180 ms (`kParkedInterval 0x0090`) during transfers; **live CE budget 1.25–5 ms** (`kLiveMinCeLength 0x0002` / `kLiveMaxCeLength 0x0008`, `:66-67`) — without it the WB delivers ~1 packet/event/leaf ≈ 31 samples/s < 50/s produced.
  - Numeric fallback floor at level ≥ 3 is **15–30 ms** (`interval_min()` → 15 ms, `interval_max()` → 30 ms, `:741-778`); the "7.5–30 ms" in the stale narrative comment (`:744-746`) is only reachable if the browser explicitly sets the `0xB5` fast-interval byte to 6.
- **CPU2 radio firmware upgraded in place (2026-09-08):** FUS 1.2.0.0 → 2.2.0.0 and BLE stack v1.13.3.2 → v1.24-era on all boards; binary from `STM32Cube_FW_WB_V1.24.0\Projects\STM32WB_Copro_Wireless_Binaries\` (note `Projects\`, not `Middlewares\`). `FUS_STATE_ERR_UNKNOWN` after a FUS flash clears on full power-cycle. v1.14+ CPU2 stacks **removed the legacy `hci_le_connection_update` opcode** (returns `0x01` Unknown Command) — interval changes go through vendor cmd `aci_gap_start_connection_update` (OGF 0x3F, OCF 0x009E, same 7 params) with legacy-HCI fallback (`exo_hub_central_client.cpp:538`); one firmware image works across pre/post-v1.14 stacks.
- Browser link: Master asks the central OS stack for 10–15 ms via `aci_l2cap_connection_parameter_update_req(h, 8, 12, 0, 500)` — **best-effort, can be ignored by Chrome/OS** (`Master/Core/Src/ble/app_ble.cpp:1049-1063`).
- Upload intent survives reconnect (`TransferLinkRearmState`, `link_tune_state.h:866-923`).
- Fragility note: never call `hci_le_set_event_mask` on Master — kills advertising (`app_ble.cpp:376-383`).

### 3.6 Throughput history / known ceilings
- Node→Master upload: 1.8 KB/s (pre-fix) → pacing ceiling 22.5 KB/s → burst-4 ~90 KB/s ceiling → today event-driven pump (burst 64). Goal ≥30 KiB/s per node (`docs/superpowers/plans/2026-08-25-node-master-transfer-throughput.md`). **P1 (§5) is the current cap on this path** — the missing per-flush completion wake means the pump cannot top up the TX pool before it drains.
- Live link budget ~9.8 KB/s shared ⇒ 0xA8 turns off master-own stream (2.6 KB/s) during live inference.
- **2026-09-08 record-upload measurement: ~4.5 → ~7.2 KB/s per node** after the interval fallback ladder + burst-limit changes (3 nodes, 113 s → 73 s wall). Remaining gap to the ~98 KB/s ST-example ceiling is diagnosed, **not implemented**: (a) node TX pump refills **one** buffer per `ACI_GATT_TX_POOL_AVAILABLE` event — 1-deep pipeline, radio idle ~97% (NODE2 counters: 574 accepted / 637 `INSUFFICIENT_RESOURCES` / 699 pool events); this is P1 (§5) observed directly; (b) connection interval deterministically lands at **20 ms** — the BlueNRG controller only accepts intervals that are integer subdivisions of the current anchor (15 ms requests always `0x84`, ladder lands on 40÷2); (c) CE length min/max = 0 in connect params is an open suspect. Node-side TP flood test mode exists for isolating the radio path (`TP1`/`TP0` raw writes to PipeControlRx, `node_throughput_test_process()` in `Node/Core/Src/main.cpp`).
- Documented next step if 30 KiB/s unreachable: **capability-negotiated L2CAP CoC bulk lane** (throughput plan :115-117).

## 4. Dataset acquisition & live model testing

### 4.1 Recording sessions (binary-first)
- All session control over BLE from the desktop webpage (`host/desktop_tool/Exoskeleton.html`): StartRecord / PrepareRecord(`0x0B`) / CommitPreparedRecord(`0x0C`) / StopRecord(`0x0E`) / RecordDone (`exo/protocol/ble_record_protocol.h:8-22`). No hardware button.
- Node records **ESOX v4** (magic `0x584F5345` "ESOX", 88 B header — `static_assert(sizeof(SessionHeader)==88)` `recording_types.h:9,11,151`; `EXO_SAMPLE_FORMAT_VERSION 4U`), CRC32s, loss flags, to W25Q flash; on RecordDone uploads chunks; Master stages to SD `/SESSIONS/R####N#.BIN` (single-digit node id, `master_node_session_stager.h:435-449`); master's own IMUs → `/SESSIONS/R####M.BIN` (`master_sd_session_recorder.h:596`); intermediate `/SESSIONS/MREC.BIN` (`:12`); run index `/SESSIONS/RUNIDX.BIN` (`master_binary_session_index.h:252`).
- Offline conversion `host/desktop_tool/vantage_bin_to_csv.py` validates: magic, version==4, `completion_flag==0xA5`, node_id 0–4, sensor_mask ↔ payload/count consistency, `payload_size == count*sample_size`, `captured_count == sample_count`, `attempted >= captured`, **physical file size == logical size (explicit reject of sparse Master archives)**, header CRC32 (header with CRC field zeroed), streamed payload CRC32, BNO timestamps monotonic+finite, **ICM timestamps monotonic AND strict `sequence` continuity from 0** (`:122-218`). Filename regex `^R\d{4}(?:M|N[1-4])\.BIN$` (`:32`). → per-sensor 16-col CSVs + `*_metadata.json`.
- Dataset: **6 sessions × 3 classes** — correct(0) / incomplete_range(1) / elbow_movement(2); nodes **N2=wrist_distal_forearm, N3=elbow_region, N4=upper_arm_near_shoulder** (`dataset/dataset_config.json:7-17`, mirrored `host/live_tool/model/model_contract.json:4-13`); `included_nodes [2,3,4]`, `excluded_sources ["MASTER"]`. 41 MB committed; `converted_csv/`, `Session_output/` gitignored.

### 4.2 The bicep-curl model
- RandomForest (500 trees, depth 14) → ONNX via skl2onnx. Input `features` float32 [batch, **576**] = **72 channels × 8 statistics**; 72 = per node (13 BNO + 6 ICM + 4 magnitudes = 23) × 3 nodes + 3 inter-node relative quaternion angles (`host/live_tool/js/ml-preprocessing.js:28-52`, `buildChannelNames`). Windows of **50 samples @ 25 Hz** (2.0 s), stride **12** (0.48 s) (`model_contract.json:44-49`; contract gate `host/live_tool/js/live-inference.js:44-65`).
- 8 stats, order `mean, std, min, max, range, iqr, rms, mean_abs_diff` (`ml-preprocessing.js:46`, `pipeline/vantare_live_pipeline.py:38`, `model_contract.json:52-61`); IQR uses numpy linear-percentile.
- Models: v1 50 Hz (fold accuracy 0.967), **v1_1 25 Hz (0.953)** — deployed in `host/live_tool/model/`.
  - ⚠ **Fold accuracy is structurally inflated.** `dataset_config.json:26-29` is a 2-fold session hold-out: each class has exactly 2 sessions, one per fold side, so every fold trains on one session per class and tests on the *other* session of the same class — the number cannot separate "learned the movement" from "learned this recording". Fix = more sessions with varied mounting, not threshold tuning.
- **Inference runs in the browser only** — **ONNX Runtime Web 1.20.1**, WASM EP, `numThreads = 1` (`live-inference.js:10,102-104`). Model was *trained* against onnxruntime 1.29.0 / skl2onnx 1.20.0 / sklearn 1.6.1 / numpy 2.1.3 (`model_contract.json:112-120`); parity between train and runtime rests on `numThreads=1` + nearest-sample decimation, **not** on matching ORT versions. Zero ML in firmware — firmware only transports samples and relays haptic commands.

### 4.3 Live test session + qualification harness
- Start sequence (`host/live_tool/js/ble-protocol.js:545-552`, from `main.js:336`): `0xA8 0` (master-own off) → `0xA2 40` → `0xA0` start.
- Browser health gates, **T = 1000/25 = 40 ms** (`main.js:476-477`): staleness **> 3T (120 ms)** (`:502-503`), cross-node skew **> 0.5T (20 ms)** (`:509-511`) ⇒ DEGRADED (inference disabled, haptics disarmed, `:532-542`); sustained rate **< 0.9×25 Hz** *after* `elapsedMs > 10000` **AND `health.received > 250`** (`:524-525`) ⇒ rate-contract violation.
- **Qualification** ("Run 60 s Qualification", `main.js:583-640`): `scheduled ≈ 125 = 60/0.48`. PASS requires:
  - `validPct ≥ 95` (`:594`);
  - each of **six streams** — keys `n2s1,n2s2,n3s1,n3s2,n4s1,n4s2` = NODE_IDS `[2,3,4]` × SENSOR `{BNO=1, ICM=2}` (`main.js:600-602`, `ble-protocol.js:419-421`) — with `rate ≥ 23.75 Hz` **and** `loss ≤ 1.0 %` **and** `maxGap ≤ 3×periodMs (120 ms)` (`:613`);
  - `maxSkew ≤ 0.5×periodMs (20 ms)` (`:627`).
  - It is a transport/rate gate — **not** rep counting (V1 has no rep counting by design).
- **"loss" here is an arrival-deficit, not sequence-gap loss.** Because the Master stamps every relayed frame with one global sequence counter (§3.3.6), `StreamHealth` deliberately computes `lossFraction = max(0, 1 − received/(spanS·25))` (`ble-protocol.js:264-270,304-309`). A stream can report "0.00 % loss" while genuinely dropping packets, as long as its surviving average rate ≥ 25 Hz.
- **The 20 ms skew gate cannot meaningfully fail on the current wire.** All streams already share the Master clock, so `skewMs` (`ble-protocol.js:370-391`) measures only differential Master→browser jitter and returns `null` unless all three model nodes have history. In the FAIL run below it printed "PASS 0.0 ms" **while N2 was absent and skew was never computed** — a false pass.
- **A fully-absent stream is invisible to every runtime health gate.** Staleness and rate loops early-out on `if (!health) continue` (`main.js:501-502,520-521`); a stream that never delivers a sample never creates a `StreamHealth` entry, and `maybeEmit()` needs all six streams to have ≥ 2 samples before it forms a window (`ml-preprocessing.js:210-211`). With N2 gone the session sits in `warming_up` **forever** and never goes DEGRADED — only the explicit 60 s Qualification surfaces the problem.
- **Last recorded run FAILED** (`Output/output.md`, 2026-09-03 — the only run log in `Output/`, page build `2026-09-03.11`; the `.11 → .12` diff since is build-string only, no behavioral JS change):
  - `0.0 % valid windows of 0` (`:267`); verdict "FAIL — do not proceed" (`:265`).
  - **N2 streamed nothing at all** — `n2s1/n2s2: no data` (`:268-269`).
  - N3 fine — `n3s1 25.76 Hz / 0 % / 94 ms gap`, `n3s2 25.76 / 118 ms` (`:270-271`).
  - N4 marginal FAIL — `n4s1 25.65 / 118 ms` PASS, `n4s2 25.65 Hz / max inter-packet 145 ms > 120 ms gate` FAIL (`:272-273`).
  - Skew "PASS 0.0 ms" (`:274`) — false pass (see above). Throughput 6.8 KB/s (`:275`).
  - Master-side transport detail from the same log: congestion only in the first ~7 s (`fwdF` INSUFFICIENT_RESOURCES failures plateau at 3251; per-node forward drops `mdrp` plateau n3=218/n4=134), stable after; `wdog=1` (Master live-TX gate watchdog fired once); `nc` (Master notification-complete count) climbs steadily — the Master *has* the 0x08 bit.
  - **n3/n4 "passing" links ran in `LinkTuneState = Degraded` (st=7), never Ready (6), at iv=20 ms** — the passing streams were themselves not on a clean fast link.

### 4.4 Documented firmware gaps (from `host/live_tool/README.md:69-80` + design)
- Nodes do **not** report congestion/decimation counters or effective interval on the wire — `NodeLiveSampleQueue::congested()` returns constant `false` (`node_live_sample_queue.h:109`), `coalesced()` returns 0 (`:107`), and the queue's `decimated_`/`dropped_` counters (`:105,155-157`) never leave the Node. The Master *did* gain a per-node forward-drop counter this HEAD commit (`live_dropped_for_node`, `hub_leaf_ble_manager.h:171-178`) but that is Master-side, so the design §6.2 requirement (Node reports its own rate loss) is still unmet — the browser infers rate loss heuristically.
- Master does not implement the design-§7.1 **per-sample leaf-ingest receive-stamp**; instead it drops the node timestamp entirely and re-stamps at egress (§3.3.6). Skew estimation is therefore arrival-based and, per §4.3, structurally degenerate.
- No separate live firmware profile (`EXO_LIVE_INFERENCE_BUILD` / `_Live.bin`, `README.md:75-80`); live tool runs against recording firmware with streaming enabled.
- Design/code drift: design says NodeLiveSampleQueue depth 8 + node-side congestion demotion; code has depth 16 (12 before this commit) and `congested()` constant false — demotion now lives only Master-side (10→20 ms).
- `LIVE3` SWO diagnostic hard-codes nodes 3 and 4 (`Master/Core/Src/main.cpp:2390-2404`); `LIVE`/`LIVE2` do track n2 (`:2341,:2365-2368`).

## 5. Firmware risk review (current tree, re-verified 2026-09-07)

All four prior findings **CONFIRMED** at their cited lines. Four additional issues found (3× P2, plus P3s). Severity: P0 none, P1 one, P2 six, P3 three.

### P1 — Node BLE characteristics missing `GATT_NOTIFY_NOTIFICATION_COMPLETION` (0x08)
- `Node/Core/Src/ble/custom_stm.cpp:529,555,581,607,633` create every characteristic with event mask 0x07; Master sets 0x0F on PipeDataTx (`Master/Core/Src/ble/custom_stm.cpp:571`). The Node-side plumbing is otherwise complete and therefore dead: VS-event case `custom_stm.cpp:400-411` → `Custom_APP_NotificationComplete` `custom_app.cpp:325-338` → counters read at `Node/Core/Src/main.cpp:470-474` (live gate) and `:802-807` (upload pump).
- **Consequence split (refined this sweep):**
  - **Upload path — the real cost.** The per-flush completion wake is lost, so the pump cannot top up the TX pool before it fully drains → throughput ceiling below what the 20-buffer pool allows. The `aci_gatt_tx_pool_available` event *still* fires after each `INSUFFICIENT_RESOURCES` and wakes the pump (`node_upload_pump.h:94-102`, `Node/Core/Src/main.cpp:808-812`), so under backpressure the pump cycles burst-64 → pool-full → tx-pool-wake; the 750 ms watchdog is the **exception path**, not the steady-state clock — *unless* a tx-pool event is also lost, then 750 ms dominates (still "explicitly forbidden by `node_upload_pump.h:8-13`" as an intent statement).
  - **Live path — NOT blocked.** `g_node_live_tx_gate` reopens via its 10 ms watchdog ⇒ ~100 sends/s ceiling, comfortably above the ~28 bundles/s live cadence. Live 25 Hz does not depend on P1.
  - LINK_STATS `notification_complete_count` (sourced `Node/Core/Src/main.cpp:1022,1069`) always reads 0 — silently broken diagnostic.
- Fix: add the 0x08 mask bit on Node PipeDataTx (mirror Master), fix `CHAR_VALUE_LEN_VARIABLE`→`CONSTANT` in the same edit, record as an owned deviation to WPAN-template code. **Not verifiable by host tests**; needs a target check that `Custom_APP_NotificationCompleteCount()` climbs during stream/upload + before/after resume-latency comparison. ⚠ Re-test the live path after this change — waking *both* consumers on every flush is exactly the shape of the prior `live-inference-no-node-data` regression (separate `g_node_upload_seen_*` vs `g_node_live_seen_*` shadows make it *correct as written*, `Node/Core/Src/main.cpp:470-479,802-812`).

### P2 — Master local transfer accepts unvalidated host chunk index (uint32 wrap) — **ACK path**
- `Master/Core/Src/main.cpp:4991-4995` moves `g_local_stream_cursor_chunk` to the browser-supplied `ack.next_chunk_index` with only a later `offset >= g_local_session_size` guard (`:3910`). `offset = chunk_index * 180` (`:3909`) wraps for index ≥ ~23.86 M; a wrapped-low value passes the guard and `g_local_session_recorder.read()` serves wrong-offset SD bytes as a mislabeled chunk. End-of-transfer CRC is the only catch. Cursor only advances on `>`, so one bad frame parks it. Fix: bound-check against `reliable_total_chunks()` (Node does this at `Node/Core/Src/main.cpp:719-725`) + uint64 offset math.

### P2 — Master local transfer, **NACK / recovery path** — same wrap (new this sweep)
- `Master/Core/Src/main.cpp:5057-5061` queues browser `nack.first_chunk_index` (uint32, wire) → `process_recovery_queue` (`:1200-1203`) copies it into `g_local_retx_cursor_chunk` with no bound → same `offset = chunk_index * 180` wrap in the burst loop (`:3908-3910`), serving wrong-offset bytes as a `kRecordFlagRetransmit` chunk. Same fix.

### P2 — Node `AckWindow` skip-ahead accepts unvalidated `next_chunk_index` (new this sweep)
- `Node/Core/Src/main.cpp:1612-1614` is the **one** reliable-control path on the Node that moves `g_node_upload_next_chunk` *without* `node_upload_chunk_index_valid()` — its three siblings all validate (legacy ChunkAck `:784-790`, NackRange `:1637`, retransmit-request `:1707`). Injection vector: the Master forwards browser-originated reliable frames to the node verbatim via `forward_remote_record_control` (`Master/Core/Src/main.cpp:5008`), sanitizing only `credit`, not `next_chunk_index`. A browser `AckWindow` with `next_chunk_index` past EOF for an active node upload sets the cursor beyond the file → `offset >= g_node_upload_total_size` (`:878`) → node sets `g_node_upload_active=false` and **goes mute; the upload silently terminates** until session-stall timeout — exactly the failure the guard comment (`:716-718`) warns about. Fix: add `node_upload_chunk_index_valid(ack.next_chunk_index)` before `:1614`.

### P2 — Browser disconnect leaves the live-priority latch stuck (new this sweep)
- `g_live_stream_priority` / `g_ble_stream_enabled` are cleared only by explicit `0xA1` / `0xA8`-on (`Master/Core/Src/main.cpp:2438-2448, 4705-4716`; setter `master_exit_live_priority_session` `:1528-1531`). The phone-disconnect handler (`Master/Core/Src/ble/app_ble.cpp:490-497`) clears `g_phone_connected` / `g_phone_connection_handle` and **nothing else**. After a tab-close / crash / BLE drop mid-stream (a documented common failure, §4.3), the latch stays `true`: every subsequent Node `RecordDone` is ACKed-and-dropped (`main.cpp:4483-4491`) and the `!g_live_stream_priority` gate (`:3822`) blocks the whole node-upload scheduler — **no node upload can ever start again** until stream control or a `0xB3` reset. The forwarder also keeps spinning against the dead link (`:1033-1036`). Fix: on phone disconnect call `master_exit_live_priority_session()` + clear `g_ble_stream_enabled`, or add a "server disconnected > N ms" watchdog.

### P2 — `ble_v2_pack` length math in `uint8_t`
- `exo/protocol/ble_stream_v2.h:38`: `const uint8_t total = static_cast<uint8_t>(sizeof(BleV2EnvelopeHeader) + payload_len)` wraps for `payload_len ≥ 242`, defeating the `total > out_cap` guard → OOB `memcpy(out+14, payload, payload_len)`. **`out_cap` is also `uint8_t`** (`:34`) — a second latent wrap for any future buffer ≥ 256. Unreachable today (only caller `Master/Core/Src/main.cpp:990` with `push_leaf_sample` capping payload at 96, real payloads 56/20). Fix: compute in uint16_t, reject `payload_len > 255 − header`.

### P2 — Master→node control write falls back write-without-resp → write-with-resp on any failure
- `exo_hub_central_client.cpp:1128-1142`: `status != BLE_STATUS_SUCCESS` (no discrimination) retries via `aci_gatt_write_char_value` and overwrites `status` unconditionally (`:1141`). On `INSUFFICIENT_RESOURCES` a with-response write doesn't create pool capacity, and on a **teardown / `NOT_ALLOWED` race** it occupies the single ATT client-procedure slot for that connection until the ~30 s ATT transaction timeout — during which `LinkTuneState` DLE/PHY/interval commands *and* reliable-control frames to that node all fail. More serious than "muddies backpressure semantics." Fix: status-filter or drop the fallback; let reliable-control keepalive/retry (300 ms) handle pool-full.

### P3 — `reliable_total_chunks()` returns `uint16_t`
- `Master/Core/Src/main.cpp:287-290`; `RecordReliableManifestPayload::total_chunks` uint16 (`ble_record_protocol.h:221`). A session file > 65535 × 180 B ≈ **11.8 MB** overflows the advertised chunk count (Master local IMU sessions and 32 MB node flash can exceed this). Transfer still completes (byte-offset driven) but the manifest count and any browser progress UI are wrong. Fix: widen to uint32 or assert a size ceiling.

### P3 — Node does not sanitize wire credit
- `Node/Core/Src/main.cpp:1578,1616`: `g_node_upload_credit = ack.credit == 0U ? kRecordReliableDefaultCredit : ack.credit` — accepts 1..255 directly (Master caps at 24 via `sanitize_receiver_credit` `:727-731`) and **inverts a genuine zero-credit hold into full credit**. Per-pass `kNodeRecordForegroundBurstLimit` bounds immediate damage. Fix: mirror `MasterNodeReliableControl::sanitize_credit`; don't treat 0 as "full".

### P3 — Master live forwarder: backpressure path doesn't rotate the round-robin
- `Master/Core/Src/main.cpp:1062`: on `BUSY` / `INSUFFICIENT_RESOURCES` the loop `break`s without `leaf_ble_manager.on_live_sample_send_result(false,…)`, so `selected_live_index_` / `next_preview_source_` stay cached on the same slot (`hub_leaf_ble_manager.h:117-128`). Next pass retries the same (node,sensor). Not hard starvation (a fresh `push_leaf_sample` or later success re-rotates), but under sustained whole-link backpressure one node's slot is retried ahead of peers. Worth a 3–4-node load check; consider rotating source on backpressure too.

### Checked and sound (verified this sweep — safe to not re-audit)
- **Single-context concurrency model** — leaf-RX ACI callbacks and `drain_leaf_stream_passthrough` both in `MX_APPE_Process` context; `mutable` peek-time cursor mutation safe.
- **`MasterNodeReliableControl`** — `chunk_offset_valid_` / `chunk_byte_offset_` uint64 + clamp; credit sanitized 1..24; independent priority slots for NACK/ACK/manifest; 20 ms bounded retry; saturating attempt counters; **no credit leak / double-count**.
- **`MasterNodeTransferWindow`** — uint64 offset/size arithmetic, strict sequential validation, final-chunk exact-size check, `commit()` re-validates monotonicity.
- **Node inbound-index validation** on legacy ChunkAck (`main.cpp:784-790`), NackRange (`:1637`), retransmit-request (`:1707`) — all guarded. (AckWindow is the sole gap — new P2 above.)
- **Node upload burst loop** (`main.cpp:875-940`) — `offset >= total_size` guard, correct `remaining`/`chunk_size` math for in-range cursors, `encoded_len > 255U` truncation guard (`:432`).
- **Node→Master live bundle split** (`exo_hub_central_client.cpp:1471-1492`) — every field length-checked, uint16 offset math, `push_leaf_sample` rejects > 96.
- **`NodeLiveSampleQueue`** — `% Capacity` ring math, drop-oldest-with-counter on full, wrap-safe 3/4-interval decimation gate, `payload_len <= MaxPayload` enforced at `offer()`, per-sensor gate arrays sized correctly.
- **`BleNotificationGate` / `NodeUploadPump`** — watchdog is recovery-only; credit exhaustion does not arm the watchdog (correct); `OtherFailure` stops pump + preserves cursor.
- **`HubLeafBleManager`** — `on_ble_reliable_ack_window` / `on_ble_chunk_ack` monotonic (`>=`) guards, `owns_transfer_` session+source gating, round-robin advances on `discard_next_live_sample`. (`on_ble_reliable_nack_range` rewinds `next_chunk_index_` unbounded — low impact, local bookkeeping only; widen with the P2 fixes.)
- **Global `g_ble_sequence` uint16 shared across all forwarded streams** — intentional; JS measures loss by arrival deficit not sequence gaps (`ble-protocol.js:264-270`); wrap harmless.
- **blepipe / reliable header decode** both sides — `sizeof(hdr)+payload_len > length`, proto-version + magic checks present (`Master/Core/Src/main.cpp:4857-4874`, `Node/Core/Src/main.cpp:1546-1554`).
- **HAL_GetTick wrap** — wrap-safe subtraction forms throughout live/transfer paths.
- **SD recorder** — sector alignment, double CRC, bounded finalize (from prior sweep, not re-examined — nothing observed contradicts it).
- **Live pacing math** — 9/10 interval, stale-repeat 400 ms, master re-stamp (from prior sweep).

## 6. Improvement hooks (agreed focus: BLE communication + live model testing)

1. **P1 one-liner first** — add the 0x08 mask bit to Node PipeDataTx. Biggest single unlock for **upload throughput** (the live 25 Hz path does *not* depend on it — corrected this sweep). Re-test both paths for the known dual-wake regression.
2. **N2 no-data is a link-establishment failure, not a browser bug** (§6a below) — this is the blocking field failure for the model (wrist node required). Triage first.
3. **Node rate/congestion telemetry** — put the Node's existing `decimated_` / `dropped_` / effective-interval counters on the wire (LINK_STATS or live frames) so the browser stops guessing the rate contract (design §6.2).
4. **Forward the node timestamp** — `offset_us` (BNO) / `sequence` (ICM) are already in the live payload and thrown away (§3.3.6). Forwarding them gives real cross-node skew instead of the currently-degenerate arrival-time estimate (design §7.1), and makes the 20 ms skew gate meaningful.
5. **Clear the live-priority latch on browser disconnect** (§5 new P2) — otherwise a mid-stream tab close bricks node uploads until `0xB3`.
6. **n3/n4 stream in `LinkTuneState = Degraded`** even on "passing" runs — the fast link is not actually being achieved. Investigate the fallback ladder / CE-budget division across 3 live leaves + browser link before trusting any qualification PASS.
7. **Browser-link interval is best-effort** — if qualification still fails on staleness after the above, consider L2CAP CoC (documented next step) or application-level pacing matched to the actually-granted interval.
8. **Bound-check every wire-supplied chunk index** (§5 three P2s: Master ACK path, Master NACK path, Node AckWindow) — use uint64 offset math throughout.
9. **Dataset** — more sessions with varied mounting to break the session==class coincidence before trusting accuracy numbers (§4.2).
10. **Chunk ceilings are hard invariants** — Node→Master 192 B payload / 197 B ceiling; Master local 180 B. MTU/DLE/PHY constants likewise. Any framing change must re-derive them.

### 6a. The N2 "wrist node streamed nothing" failure — investigation state

**Not a browser-side or protocol-mapping cause.** The browser accepts `node_id 2` everywhere (`NODE_IDS=[2,3,4]` `ble-protocol.js:121`; `onNotify` `:678`; preprocessing, `modelStreamKeys()` `main.js:466-469`, qualification loop `:600-602`). The `n2s1/n2s2` labels are sensor ids (1=BNO85, 2=ICM45686), not a node mismatch.

**N2 never established a BLE central link with the Master.** From `Output/output.md`: every `LIVE` line `rx=…(n2=0 …)` (`Master/Core/Src/main.cpp:2341`, `live_rx_for_node(2)=0` all run); every `LIVE2` line `n2[iv=0.00 st=255 rt=0]` (`st=255` = "no link entry" sentinel, `rt=0` = zero fast-timing retries attempted); `Topology: NODE3, NODE4` — the discovered-node report never includes NODE2.

**Ranked firmware hypotheses** (all consistent with "one node totally silent, others fine"):

- **H1 (most likely) — the wrist node's W25Q256 flash failed to enumerate.** `NodeRecordingApp::begin()` returns `false` on `flash_.begin()` failure / capacity < `kMinimumSupportedFlashSize` / `!flash_layout_valid()` (`node_recording_app.h:53-72`). On `false`, `process()` early-returns every superloop iteration (`:270-273`) — and `process()` is the **only** place `bno85_.service()` runs and `live_queue_.offer()` is called (`:288,324-333,698-729`). The node still advertises, connects, services control/haptics/touch — looks healthy on the link, streams nothing. Per-unit (bad SPI solder / bad part on that one board) — explains why only the wrist node died. Boot-log discriminators: `"Node recording: not ready"`, `"W25Q init dbg: … jedec=…"`, `"Node ID runtime fallback=%u (flash unavailable)"` (`Node/Core/Src/main.cpp:2019,2040,2043-2049`).
- **H2 — the wrist node self-identifies as node_id 1 (or 0).** `#define EXO_NODE_ID 1U` is the compile default (`Node/Core/Src/main.cpp:108-109`); `node_id_cache()` also defaults to `kNodeIdMin=1` (`node_runtime_config.h:64`); `current_node_id()` returns the build default on any settings-sector read failure without persisting (`:184-196`). Master copies `src_id` verbatim (`exo_hub_central_client.cpp:1152-1155`). Browser `DISPLAY_SOURCE_IDS` includes 1 (raw graph moves) but `NODE_IDS` excludes it → n2s1/n2s2 get zero samples even with a perfect RF link. Discriminator: `"Node ID runtime=%u (default=%u provisioned=%u)"` (`:2030-2033`).
- **H3 — settings-sector base offset wrong.** `settings_sector_base() = flash_capacity_bytes() − 4096`, and `flash_capacity_bytes()` defaults to **2 MB** (`node_runtime_config.h:11,46-53`); it only becomes 32 MB after `set_flash_capacity(...)` which runs *only inside* `if (node_recording_ready)` (`Node/Core/Src/main.cpp:2023-2024`). Flaky-but-not-dead flash → id read from the wrong offset → comes back `Blank` → re-provisioned to `EXO_NODE_ID` (=1).
- **H4 (weaker) — two nodes on the same id.** Would show N3 at double rate; `output.md` shows N3 at a clean 25.76 Hz, so unlikely for this run, but `node_runtime_config.h:169-171` documents a past incident of it.
- **Adv-name parse has no clamp** — `exo_parse_leaf_name_id` (`exo_hub_central_client.cpp:671-699`) takes the last two digits of the `L00xx` advertised name as the id; a node named `L0021` presents id 21 and is silently rejected by `push_leaf_sample`. If N2 never appears in the `0xB4` discovered-nodes report at all, look here.

**Not the cause:** CE-budget division (degrades *rate* on a leaf, can't zero it — N4's 145 ms gap is consistent with CE pressure, N2's total silence is not); LinkTune starvation (`issue_next` is capped at 4 attempts then parks, can't monopolize the arbiter); targeted-reconnect (capped at 3, doesn't tear down established links).

**Recommended triage:** SWO boot-log capture from the wrist node — the three lines `"Node recording: …"`, `"Node ID runtime=… (default=… provisioned=…)"`, `"Node flash capacity=…"` distinguish H1/H2/H3 in one shot.

## 7. Explicitly not present (avoid re-searching)
No RTOS, no IWDG, no RS485, no downstream token handshake (grep "token" hits only legacy Modbus code), no rep counting, no on-MCU inference, no local training scripts (Colab only), no TODO/FIXME markers in app code, no button-based session control (all BLE), no newer live-run log than `Output/output.md`, no `EXO_LIVE_INFERENCE_BUILD` profile.
