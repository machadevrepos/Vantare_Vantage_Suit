Firmware fixes: Node linker RAM fix + deferred issues #24, #27, #18

## 1. Node linker RAM fix (mirrors Master commit 52e547b)
- `Firmware/Node/STM32WB55CCUX_FLASH.ld:48`: change `RAM` region `LENGTH = 0xFFF8` → `0x1FFF8` (full 128K SRAM1; CPU2-shared areas are in SRAM2 @ 0x20030000, unaffected).
- Leave the `_RAM.ld` debug script untouched (same as Master fix). FLASH cap stays 124K.
- Update `Firmware/Node/size_budget.md` with the new SRAM budget numbers.

## 2. Issue #18 — skip 4 KB re-read per flash write (Node recording throughput)
- `Firmware/LIBRARY/CUSTOM/w25qxx/src/driver_w25qxx.c:8358`: `a_w25qxx_write_no_check` already exists but is `static`. Add a thin public wrapper (e.g. `w25qxx_write_no_check`) following the file's existing public-API conventions.
- Expose it through `W25Q256_FLASH.h:142-147` as a `write_pre_erased(offset, data, len)` method with a debug/parity-check option (verify destination is 0xFF before programming, cheap read of just the target range rather than a full 4 KB sector RMW).
- Switch call sites where the target region is guaranteed pre-erased by the background chunked-erase path: `NODE_RECORDER.h:383` `append_samples()` and `NODE_RECORDING_APP.h:479`.
- Header/finalize writes that are NOT guaranteed pre-erased keep the safe `write()` path.

## 3. Issue #24 — O(n) run-index scan + crash-window stranding
- `Firmware/Master/Core/Inc/MASTER_BINARY_SESSION_INDEX.h`:
  - Add a boot-time cache of the last allocated index: scan `R####`/legacy files once at startup (outside BLE context), remember `last_index`, and persist an 8-byte marker file (e.g. `/SESSIONS/RUNIDX.BIN`) after each allocation.
  - `allocate()` becomes: marker read → marker index+1 → occupancy check (`f_stat` only for the candidate index, not a 1..9999 loop) → fallback to full scan only if the marker is missing/corrupt.
  - Crash-window semantics: index claimed at prepare persists the marker immediately, so a crash mid-run strands the index (file may be missing) but never re-issues it — matching the designed fix in the issues doc.
- Verify boot ordering: cache init must happen after SD mount and before BLE services start (check `main.c` init sequence).

## 4. Issue #27 — move SD I/O out of BLE dispatch context
- `Firmware/Master/Core/Inc/MASTER_NODE_SESSION_STAGER.h` + call sites in `Master/Core/Src/main.c` (~lines 1370-1392 phone-write handler, 1728-1768 collection begin):
  - Add a small pending-work queue/flag set: BLE handler paths only enqueue (flush request, shutdown request, index allocate request) and return.
  - Existing superloop already calls `service_reliable_control()` per iteration — add `stager_.service()` there to perform the actual 4 KB flushes (`flush_write_buffer()` :435), shutdown/abandon flushes (:327-329), and duplicate read-back validation (`duplicate_matches()` :458-493) outside the notification context.
  - Keep synchronous behavior for correctness-critical paths (final file validation/close) only if they already run in superloop context.
- This uses the RAM freed by the Node/Master linker state (65 KB free on Master) if a queue buffer is needed.

## 5. Verification
- Run the 9 Python host tests in `Firmware/HostTests/` (they cover the converter invariants; issues #24/#27 touch only firmware C++, but run them as regression).
- Syntax-check the modified headers via the existing PowerShell syntax-only suite if available; full CubeIDE build is deferred to hardware verification per project convention.
- Update `Docs/FIRMWARE_ISSUES_AND_FIXES.md`: mark #24/#27/#18 resolved, record the Node linker fix, refresh memory-budget tables.
- Commit each fix separately with the project's `applied ..._patch` / descriptive style.

## Notes
- Master 539 B flash margin: per the doc's own guidance, the 124K cap is the only known-good value; we will NOT trim more logs or raise the cap without STM32CubeProgrammer hardware verification. Only watch it.
- Deferred-by-design items stay deferred: #18's parity scan is kept cheap (read only target range), P3 items untouched.