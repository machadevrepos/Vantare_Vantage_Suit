# Vantare Live Inference Tool

Browser application for live bicep-curl classification over Web Bluetooth.
Implements the design in
`docs/superpowers/specs/2026-09-01-live-bicep-curl-inference-design.md`.

**The model is V1: experimental and personalized (six sessions, one person).
It is not a general exercise-assessment or safety system, and the notebook's
0.967 accuracy is not a generalization estimate (design Section 15).**

## Running

```bash
cd host/live_tool
python -m http.server 8080
# open http://localhost:8080 in Chrome/Edge (Web Bluetooth requires a secure context)
```

The page must be served over `localhost` (or another secure context); opening
`index.html` as a raw file is not supported.

## Model artifacts (required before inference works)

The app loads `model/model.onnx`, `model/model_contract.json`, and
`model/feature_names.json` at startup and **refuses to start inference unless
the contract matches the qualified 25 Hz live rate** (design Sections 3, 11).
It never resamples a lower-rate stream onto a higher-rate grid.

The checked-in `host/notebooks/` artifacts are the 50 Hz V1 training outputs
and will NOT pass this gate — that is deliberate. To produce the live
artifacts:

1. Run `host/notebooks/Vantare_Bicep_Curl_Training_ONNX_v1_1.ipynb` in Colab
   (dataset upload per `dataset/README.md`). It implements design Section
   11.3: 25 Hz target, 50-sample windows, 12-sample (0.48 s) stride,
   nearest-sample decimation, unchanged channels/features/hyperparameters.
2. Copy into `host/live_tool/model/`:
   - `vantare_bicep_curl_v1_1.onnx` → `model.onnx`
   - `model_contract_v1_1.json` → `model_contract.json`
   - `feature_names_v1_1.json` → `feature_names.json` (must be byte-identical
     to the V1 `feature_names.json`; the notebook asserts this)

## ONNX Runtime Web

`vendor/ort/` holds a version-pinned ONNX Runtime Web build served locally
(no CDN). The version is recorded in `js/live-inference.js`
(`ORT_VERSION_PINNED`) and shown in the UI; the WASM execution provider runs
single-threaded (`numThreads = 1`) so no cross-origin isolation headers are
needed. If `vendor/ort/` is empty, populate it with the pinned version's
`ort.min.js` and the matching `ort-wasm-simd-threaded.wasm` from the
onnxruntime-web npm distribution before serving.

## Firmware expectations

The current firmware already supports the live path used here:

- `0xA2` set-interval + `0xA0` start / `0xA1` stop (blepipe `MSG_STREAM_CONTROL`)
  enable live B1 preview streaming independent of SD recording
  (Master `main.cpp` `g_ble_stream_enabled`, Node `node_stream_enabled`).
- `0xA3` SET_ERM targeted per node drives the motor (blepipe `MSG_COMMAND`).

- `0xA7` bounded haptic pulse (blepipe `MSG_COMMAND`, payload
  `[0xA7][intensity%][dur_lo][dur_hi][evt b0..b3]`). The Node validates the
  bounds, owns the stop deadline, and ignores an event id at or below the last
  one it executed. Logic lives in
  `firmware/common/inc/exo/actuator/node_haptic_pulse.h` and is covered by
  `host/tests/cpp/test_node_haptic_pulse.cpp`.

Not yet implemented in firmware (design Sections 6.1/6.2/6.3):

- Node congestion/decimation counters and effective-interval reporting. The
  browser compensates by treating a sustained effective rate below contract
  as a rate-contract violation, but without counters it cannot distinguish
  demotion from loss.
- Separate `EXO_LIVE_INFERENCE_BUILD` profiles and `_Live.bin` outputs, and the
  Master-side live behavior (SD-write rejection, per-node stream health
  reporting, Master receive-stamping for the Section 7.1 time base). The tool
  currently runs against the recording firmware with streaming enabled, and
  estimates cross-node skew from browser arrival time, which carries Web
  Bluetooth notification jitter.

## Verification status

Per the project's verification policy, all of the following remain
**hardware-dependent and unverified until exercised on the target**:
BLE transport behavior against the real Master, effective stream rates and
congestion behavior, motor ring-down (sets the Section 9.1 blanking margin,
default 250 ms pending measurement), cross-node skew (Section 10 gate,
20 ms), and classifier behavior on a real wearer.

Verified on the host today:

```bash
node host/tests/scripts/test_live_preprocessing.mjs        # 10 analytic fixtures
python -m unittest discover -s host/tests/python -p "test_live_*.py"
```

The parity suite (`host/tests/python/test_live_preprocessing_parity.py`)
implements the Section 12 Python-to-browser test: a deterministic six-stream
fixture is run through the pure-Python reference in
`host/tests/python/reference_preprocessing.py` and through the browser port,
and all 576 features must agree within 1e-4. `test_fixture_is_discriminating`
asserts the fixture separates nearest-sample decimation from interpolation, so
the suite cannot silently pass if both sides regress together.

The session replay parity test (Section 12) still needs a recorded session and
has not been written.

The firmware C++ test `host/tests/cpp/test_node_haptic_pulse.cpp` is
registered with ctest but has **not been compiled here** — this machine has no
C++ toolchain. Build it before trusting the pulse logic:

```bash
cmake -S host/tests/cpp -B host/tests/cpp/build && cmake --build host/tests/cpp/build && ctest --test-dir host/tests/cpp/build --output-on-failure
```

## Live visualization

Every source the Master relays is charted, including the Master itself
(`node_id` 0) and Node 1 in four-node builds. Only N2/N3/N4 feed the model:
display-only sources never gate inference, so a missing or stale Master never
degrades a session.

- One card per source, ordered MASTER first, with a per-card signal selector
  (BNO gyro / linear accel / quaternion, ICM accel / gyro).
- Rendering is decoupled from BLE callbacks: notifications only append to a
  preallocated ring and a fixed render loop draws, so a bursty link changes the
  trace and never the frame rate. A source going stale keeps its history.
- Charts fill whenever samples arrive, including raw streaming driven from the
  Stream Control panel with no inference session running.
- `window.vantare` is a debug handle to the app for bench replay without
  hardware; see the comment at the bottom of `js/main.js`.

The fonts are loaded from Google Fonts with full system fallbacks, so the page
still renders correctly with no network access.

## Firmware control surface

The page drives the command set the firmware actually implements:

| Command | Control |
| --- | --- |
| `0xA0` / `0xA1` / `0xA2` | start / stop / interval (Stream Control) |
| `0xA3` | ERM percent, per target |
| `0xA4` | passive buzzer percent |
| `0xA5` | RGB mask, and bit7 to release the actuator override |
| `0xA6` | touch feedback self-test |
| `0xA7` | bounded haptic pulse (Node-owned stop timer) |
| `0xB2` / `0xB4` | rediscover nodes / node report |
| `0xB3` | reset retained sessions (destructive, behind a confirm) |

Node provisioning (`0xB0` set node id) is deliberately absent: re-provisioning a
node mid-test would silently invalidate the placement contract the model depends
on. Use the desktop tool for that.

## Files

- `index.html` — page
- `js/ble-protocol.js` — Web Bluetooth transport, blepipe, B1 decode, per-stream health, node time base
- `js/ml-preprocessing.js` — window assembly, 72-channel synchronization, 576 features (port of `pipeline/vantare_live_pipeline.py`)
- `pipeline/vantare_live_pipeline.py` — authoritative Python pipeline; the notebook exports this file verbatim
- `js/live-inference.js` — contract validation gate + ONNX Runtime Web engine
- `js/haptic-controller.js` — manual arming, confidence/consecutive/cooldown gate, blanking intervals
- `js/session-log.js` — two-tier session log (full-fidelity events + byte-budgeted raw-sample rings)
- `js/charts.js` — canvas strip charts, preallocated rings, per-source auto-scaling
- `js/ui.js`, `js/main.js` — rendering, state machine, health gates, qualification harness
