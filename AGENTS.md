# AGENTS.md — Vantare Vantage Suit

Wearable exosuit prototype: one STM32WB55 **Master** hub + up to 4 sensor **Nodes**
(BLE hub-leaf mesh), IMU session recording (Master SD / Node W25Q256 flash), live
streaming to host tools, haptic actuation, and browser-based desktop tooling.
Client-facing summary of end requirements: `docs/architecture/project-conversation.pdf`
and `docs/hardware/milestones.pdf`.

## Project truth

| Item | Value |
|---|---|
| MCU (both sides) | STM32WB55CCU6; physical silicon is 1 MB G-grade — linker budgets 768 K flash / 192 K RAM (`firmware/common/build/linker/STM32WB55_FLASH.ld`) |
| BLE stack | STM32_WPAN on CPU2 coprocessor; Cube package STM32Cube FW_WB V1.24.0, HAL V1.14.7 |
| Link config | MTU 247, DLE 251, 2M PHY per link (`exo/ble/link_tune_state.h`) |
| RTOS | None — single-context superloop per side; `UTIL_Sequencer` pumps BLE/HCI events |
| Watchdog | No IWDG; all "watchdogs" in BLE paths are software timers |
| Transport | BLE only (RS485 removed; that work lives on the dormant `wt-work` branch/worktree) |
| Languages | C++ (`.cpp`) incl. CubeMX-generated files; shared app logic is header-only under `firmware/common/inc/exo/` so host tests compile it |
| Version | Root `VERSION` (semver); live-tool page build string `LIVE_TOOL_BUILD` in `host/live_tool/js/live-inference.js` |

## Current direction (2026-09-09) — Coach Assist, not AI Coach

The rollout is coach-controlled movement tracking; the ML model is a **future**
version and is off the critical path. The immediate target is Milestone 4:
live suit → neutral-pose calibration → generic arm kinematics → clean motion
packet for the 3D/app team. Calibration maps each sensor to a **body segment**,
never to an exercise; bicep curl is only the first validation target.

- `host/live_tool/js/motion-engine.js` — deterministic kinematics, independent of
  the model path by design. `host/live_tool/js/rep-analyzer.js` — per-rep verdicts
  against a coach target. Contract, scope limits, measured thresholds and field
  results: `docs/architecture/motion-engine-contract.md`.
- Serve the live tool with `python host/live_tool/serve.py` (caching disabled);
  a red STALE BUILD banner means the browser is running cached modules.
- Explicitly out of scope now: model retraining, AI-selected haptics, a polished
  full-body avatar, MCU/radio redesign (the 12-node master study is a separate,
  later timeline and must not reopen firmware work without a measured blocker).

## Repository layout

- `firmware/Master/`, `firmware/Node/` — CubeIDE managed projects (`.ioc` at roots)
- `firmware/common/inc/exo/` — header-only shared modules: `protocol/`, `ble/`, `sensors/`, `recording/`, `storage/`, `actuator/`, `types/`, `utils/`
- `firmware/third_party/` — vendored: w25qxx, icm45686-driver, sh2 (treat as external)
- `host/live_tool/` — browser live-inference tool (Web Bluetooth, ONNX Runtime Web) + deployed model + `model_contract.json`
- `host/desktop_tool/` — recording/commissioning webpage + `vantage_bin_to_csv.py` converter
- `host/tests/` — `python/` (pytest), `cpp/` (CMake/ctest); `firmware/{Master,Node}/tests` — source-invariant scripts
- `dataset/` — training sessions (see `dataset/README.md` workflow); `scripts/qa_session.py`, `scripts/build_dataset_config.py`
- `docs/architecture/dataset-acquisition-branch-context.md` — **authoritative current-state context**; `docs/architecture/motion-engine-contract.md` — **current focus**, Coach Assist motion data contract + scope limits; `docs/superpowers/specs/2026-09-07-bicep-curl-model-v2-design.md` — model V2 + Phase 0 plan (**deprioritized**, see below)

## Verification (this PC)

- The user compiles (STM32CubeIDE, headless-capable) and flashes all hardware themselves.
  Agents verify edits by inspection; never build or flash.
- No cmake/host compiler installed: run single-TU host tests with `zig c++`.
- Firmware TU syntax check: CubeIDE `arm-none-eabi-g++ -fsyntax-only` with the project's include paths.
- `python -m pytest host/tests/python -v` works; the cmake/ctest path in `host/tests/run_tests.ps1` does not (no cmake).
- Sequential CubeIDE builds only (workspace lock).

## Hard invariants — do not break

- Chunk ceilings are framing-derived: Node→Master reliable payload 192 B (197 B ceiling); Master local transfer chunk 180 B. MTU/DLE/PHY constants likewise; re-derive before any framing change.
- GATT event masks: Master PipeDataTx 0x0F, Node chars 0x07 (planned P1: add 0x08 to Node PipeDataTx + `CHAR_VALUE_LEN_CONSTANT`).
- `live_slot_index` assumes node_id hard-range 1–4; live queue interval clamp [40, 80] ms.
- ESOX v4 session format: magic "ESOX", 88 B header, `EXO_SAMPLE_FORMAT_VERSION 4`.
- Never call `hci_le_set_event_mask` on the Master — it kills advertising.
- ICM scaling happens consumer-side: accel `raw*4/32768` g, gyro `raw*2000/32768` dps — keep browser JS and Python converter byte-identical.

## Working agreements

- Generated/vendor code (`Drivers/`, `Middlewares/`, `custom_stm.cpp` WPAN-template regions, `third_party/`) is edited only as documented, owned deviations; never regenerate `.ioc` casually.
- Dataset work follows `dataset/README.md`; every session is converted + `qa_session.py`-checked before it is trusted.
- Branch model: `DATASET_ACQUISITION` = current focus (dataset + live model, BLE-only). `wt-work` = RS485 wired transport (dormant). `main` = stable.
