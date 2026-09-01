# Live Bicep-Curl Inference V1 Design

Date: 2026-09-01
Status: Finalized for implementation. Revised 2026-09-02 after checking the
design against the firmware protocol headers, the live sample queue, and the
trained model artifacts.

**Read first if you are starting execution.** Three decisions in this document
differ from the original draft and change the order of work:

1. The live contract rate is **25 Hz, not 50 Hz**. This is settled by arithmetic
   over the existing packet formats plus a firmware interval clamp
   (Section 11.1), not by a bench test. The model must be retrained before
   inference integration (Section 11.3).
2. The Node live queue can **halve its own sample rate mid-session** under
   congestion without losing a packet (Section 6.2). The browser must detect
   this, so the firmware must report it.
3. The notebook's 0.967 accuracy is **not a generalization estimate** and must
   not be used as one (Section 15).

## 1. Purpose

This specification defines the first live inference path for the Vantare Vantage Suit. Three wearable nodes stream sensor data during bicep curls:

- N2: wrist or distal forearm
- N3: elbow region
- N4: upper arm near the shoulder

A browser application receives all three nodes through the Master PCB, reproduces the preprocessing used during model training, runs the exported bicep-curl ONNX model locally in the browser, displays a classification roughly twice per second, and optionally sends bounded haptic feedback through the Master.

The V1 model recognizes only:

1. `correct`
2. `incomplete_range`
3. `elbow_movement`

This is an experimental, personalized classifier trained from six one-person sessions. It is not a general exercise-assessment or safety system. The design preserves the raw evidence needed to evaluate it in real use and retrain it later.

## 2. Scope

### Included

- Separate live-inference firmware build profiles for Master and Node.
- Continuous live BNO85 and ICM45686 forwarding from N2, N3, and N4.
- One Web Bluetooth connection from the browser to the Master.
- Browser-side synchronization, feature extraction, ONNX inference, visualization, health monitoring, session logging, and haptic decisions.
- Class-specific haptic routing to N2 and N3.
- Validation against the Colab/Python preprocessing and ONNX results.

### Excluded from V1

- Training or retraining on the embedded devices or local PC.
- Replacing the existing recording firmware or data-collection webpage.
- Direct browser connections to each Node.
- On-device neural-network inference.
- Automatic repetition counting.
- User-independent accuracy claims.
- Recognition of rest, transitions, shoulder lifting, shoulder swinging, or momentum errors.
- Background inference when the live page is suspended or closed.

## 3. Existing Contracts

The design builds on the repository's existing topology and protocols:

- Browser connects to the Master over Web Bluetooth.
- Master connects to and relays samples from Nodes.
- The live wire format is the per-sample BLE V2 envelope in
  `firmware/common/inc/exo/protocol/ble_stream_v2.h`: frame id `0xB1`, version,
  `node_id`, `sensor_id`, `sequence`, `time_ms`, `payload_len`, one reserved
  byte — 14 bytes total — followed by one sensor payload. One envelope carries
  exactly one sample from one sensor on one node.
- The payload structs are those selected by `EXO_SAMPLE_FORMAT_VERSION` in
  `firmware/common/inc/exo/types/recording_types.h`. The default build uses
  `Bno85SampleV3` (56 bytes, float fields) and `Icm45686SampleV4` (20 bytes,
  int16 fields plus offset and sequence).
- The existing browser decoder understands these compact BNO and ICM samples.
- Master can broadcast stream-control commands and forward targeted actuator commands.

Note: `docs/architecture/four-node-live-preview-training-csv-design.md`
describes a per-source "preview bundle" carrying both sensors behind
present-flags. The firmware does not implement that. The per-sample B1 envelope
above is authoritative for this design, and the bundle description in that
document is stale and should be corrected separately.

The existing recording behavior remains the baseline and must not regress.

### Model contract

The authoritative artifacts are:

- `host/notebooks/vantare_bicep_curl_v1.onnx`
- `host/notebooks/model_contract.json`
- `host/notebooks/feature_names.json`

The ONNX input is `features`, `float32`, shape `[batch_size, 576]`.

The outputs are:

- `label`: `int64` predicted class identifier
- `probabilities`: floating-point probabilities ordered as `correct`, `incomplete_range`, `elbow_movement`

The preprocessing contract as trained (V1) is:

- target rate: 50 Hz
- window length: 2 seconds
- samples per window: 100
- prediction stride: 0.5 seconds
- synchronized channels: 72
- features: 576

**Stride must be an integer number of samples.** At 50 Hz a 0.5-second stride is
exactly 25 samples. At 25 Hz it would be 12.5 samples, which is not a valid
window offset. The 25 Hz contract therefore uses a **12-sample stride (0.48
seconds)**, giving about 2.08 predictions per second. Training and live
preprocessing must use the same integer stride; nothing else in the contract
changes.

**Contract status.** The 50 Hz figure is a property of the offline training
pipeline, which resampled full-rate flash recordings. Section 11 shows it is not
achievable over the live BLE transport. Live deployment therefore requires a
retrained V1.1 model at 25 Hz with 50 samples per window and a 12-sample stride.
Window duration, channel count, feature count, and feature ordering are
unchanged; only the sample rate, the samples per window, and the integer stride
differ. Until
`model_contract.json` and the ONNX artifact are regenerated at 25 Hz, the live
application must refuse to start inference rather than resample a 25 Hz stream
onto a 50 Hz grid.

Each channel contributes, in the exact order recorded by `feature_names.json`, these eight statistics: mean, standard deviation, minimum, maximum, range, interquartile range, root mean square, and mean absolute difference.

The 72 synchronized channels comprise the BNO quaternion, linear acceleration, gravity, and gyroscope values; ICM accelerometer and gyroscope values; four per-node vector magnitudes; and three inter-node relative angles. Quaternion normalization, raw ICM scaling, vector-magnitude calculation, relative-angle calculation, resampling, and feature ordering must match the training notebook exactly.

## 4. Approaches Considered

### Recommended: shared source with separate build profiles

Keep the current Master and Node source trees and add a live-inference build profile selected at compile time. Common drivers, BLE transport, protocols, and actuator control remain shared. Recording-specific services are excluded or inactive only in the live build.

This avoids source divergence while producing visibly separate live binaries.

### Rejected: duplicate both firmware trees

Copying the complete firmware would provide immediate separation, but bug fixes and protocol changes would need to be maintained twice. The copies would drift quickly, especially around generated MCU files and shared BLE code.

### Rejected: change the existing recording build in place

Turning the current build into a live-only system would risk the proven recording workflow and make switching between capture and inference modes harder to verify.

## 5. System Architecture

```text
N2 wrist ─────┐
N3 elbow ─────┼─ BLE ─> Master ─ Web Bluetooth ─> Live browser application
N4 shoulder ──┘                                      │
                                                     ├─ synchronization
                                                     ├─ 2 s contract-rate window
                                                     ├─ 576 ordered features
                                                     ├─ ONNX Runtime Web
                                                     ├─ UI and session log
                                                     └─ bounded haptic command
                                                               │
                         N2 or N3 motor <─ BLE <─ Master <──────┘
```

The browser owns inference and the high-level feedback decision. The Master remains a transport and coordination device. Nodes remain sensor and actuator endpoints. This keeps the embedded workload predictable and makes model replacement possible without reflashing the suit.

## 6. Firmware Design

### 6.1 Build profiles

Add the compile-time build flag `EXO_LIVE_INFERENCE_BUILD` without copying the firmware source trees.

The build outputs are named distinctly:

- `VantageMaster_Live.bin`
- `VantageNode_Live.bin`

The existing recording outputs and their defaults remain unchanged.

### 6.2 Live Node behavior

The live Node build shall:

- initialize BNO85, ICM45686, BLE transport, timing, and actuator support;
- stream compact samples without requiring flash recording to be enabled;
- accept the requested live interval down to the qualified live minimum (Section 11);
- expose sequence and timing information needed for browser health checks;
- omit flash erase, recording, persistence, and upload work;
- implement a bounded motor pulse that always turns itself off locally.

The existing live queue is currently coupled to flash/recording configuration. The implementation must separate sensor-to-live-queue delivery from flash writes so that disabling storage does not disable live streaming.

Two properties of `exo::NodeLiveSampleQueue`
(`firmware/common/inc/exo/recording/node_live_sample_queue.h`) constrain this
work and must be handled explicitly rather than discovered on the bench:

- `clamp_interval()` clamps every requested interval into
  `[kMinimumPreviewIntervalMs, kCongestedPreviewIntervalMs]` = `[40 ms, 80 ms]`.
  A requested 20 ms interval is silently raised to 40 ms, so 50 Hz per sensor
  cannot be requested at all in the current firmware. Any change to these
  constants is a transport change and requires re-running Section 11.
- The queue is a real bounded FIFO (depth 8) with a per-sensor decimation gate,
  not a latest-value slot. On overflow it sets `congested_` and demotes
  `effective_interval_ms_` to 80 ms for the remainder of the recovery window.
  The effective sample rate can therefore halve mid-session without any packet
  being lost on the air.

The live Node build shall expose `decimated_` and `dropped_` counters and the
current `effective_interval_ms_` to the Master, and the Master shall forward
them to the browser. The browser treats a congestion demotion as a rate-contract
violation (Section 10), not as ordinary packet loss.

### 6.3 Live Master behavior

The live Master build shall:

- connect to N2, N3, and N4;
- accept one browser connection;
- start, stop, and configure live forwarding for all three Nodes;
- relay B1 sample envelopes without writing to SD;
- report connection and stream-health status, including each Node's effective live interval, decimated count, and dropped count;
- route validated haptic pulses to the specified Node;
- reject recording, erase, and file-transfer operations in the live build.

### 6.4 Haptic pulse command

Add or formalize a command with these semantic fields:

- target node
- intensity percentage
- duration in milliseconds
- event identifier

V1 defaults are 50% intensity and 250 ms duration. Firmware accepts intensities from 1% through 100% and durations from 50 ms through 500 ms; values outside those bounds are rejected. The Node owns the stop timer, so a lost follow-up packet cannot leave the motor running. Event identifiers are monotonically increasing within a live session. Each Node remembers the highest identifier it has executed and ignores any command whose identifier is less than or equal to that value, so both an exact retransmission and a reordered stale command are suppressed. Starting a new live session resets the identifier state.

The browser may issue this command only while haptics are armed and data is healthy.

## 7. Browser Application Design

Create a separate modular application under `host/live_tool/`. The existing `host/desktop_tool/Exoskeleton.html` remains untouched.

### `index.html`

Provides the live-session page and loads the modules and model assets. The application must be served from `localhost` or another secure context; opening it as a raw local file is not the supported execution path.

### `ble-protocol.js`

- Connects only to the Master.
- Sends session, stream-rate, and haptic commands.
- Parses B1 envelopes and compact BNO/ICM payloads.
- Converts raw ICM readings using the same scale factors as the training data.
- Emits normalized sample records with node, sensor, sequence, session time, and decoded fields.
- Tracks per-stream packet counts, gaps, rates, and staleness.

### `ml-preprocessing.js`

- Maintains bounded buffers for the six node/sensor streams.
- Establishes and maintains the cross-node time base described in Section 7.1.
- Builds the common time grid at the contract rate.
- Interpolates only when the source samples surrounding a target time are within the maximum interpolation span in the Section 10 gate table (`1.5T`, 60 ms at 25 Hz).
- Normalizes quaternions and calculates magnitudes and relative angles exactly as training did.
- Produces two-second windows at the contract sample count (50 at 25 Hz) with the contract integer stride (12 samples at 25 Hz).
- Calculates the 576 features in the exact `feature_names.json` order.
- Refuses to produce a vector if a required input is missing or unhealthy.
- Refuses to produce a vector for any window overlapping a haptic blanking
  interval (Section 9).

### 7.1 Cross-node time base

Three of the 72 channels are inter-node relative angles, so any skew between the
N2, N3, and N4 clocks corrupts precisely the channels that distinguish
`elbow_movement`. The design treats time alignment as a first-class requirement
rather than an implementation detail.

- Each B1 envelope carries the originating Node's `time_ms`. These are
  independent millisecond counters, not a shared clock.
- The Master shall stamp each relayed envelope with its own receive time so the
  browser can observe Node-clock-to-Master-clock offset and drift per stream.
- On session start the browser establishes a per-node offset to the Master time
  base and continuously re-estimates it, rejecting outliers caused by transport
  jitter rather than tracking them.
- The browser shall expose the current estimated skew and drift rate per node in
  the UI and the session log.
- The alignment method used here must be compared against the alignment used to
  build the training CSVs. If the training pipeline aligned sources by a
  different rule, the live and training preprocessing are not equivalent no
  matter how closely the feature arithmetic agrees, and the training alignment
  must be reproduced or the model retrained against the live rule.

### `live-inference.js`

- Loads a vendored, version-pinned ONNX Runtime Web using the WASM execution
  provider. The runtime version and the SIMD/threading flags are recorded in the
  repository and in the session log, because they change floating-point
  accumulation order and therefore the parity tolerance in Section 12. The
  runtime and its `.wasm` assets are served locally; no CDN dependency.
- Reads the `label` output as a `BigInt64Array` and converts explicitly.
- Loads the model and contract artifacts once.
- Verifies the input name, input width, output names, class order, feature count, and expected channels before enabling a session.
- Runs one inference per stride step (0.48 s at 25 Hz) after the two-second warm-up.
- Produces the label, all three probabilities, inference latency, and source-window time range.

### `haptic-controller.js`

- Starts disarmed for every session.
- Can be manually armed only when the model is valid, N2-N4 are connected, all six sensor streams are healthy, and valid inference is running.
- Disarms on degradation, disconnection, page suspension, session stop, or model error.
- Applies the approved confidence, consecutive-window, target, and cooldown rules.
- Generates unique event identifiers.

### `session-log.js`

Records decoded incoming samples, stream-health events, prediction probabilities, decisions, inference latency, skew estimates, and haptic events. It provides downloadable logs for diagnosing live behavior and preparing future datasets.

Retention is governed by an explicit memory budget rather than by wall-clock
time alone. Six streams at the contract rate produce on the order of 10^5 sample
records per minute, which cannot be held as live JavaScript objects for ten
minutes. The log therefore keeps two tiers:

- Predictions, probabilities, latency, health transitions, decisions, and haptic
  events are retained in full for the whole session; these are low-rate and are
  the primary diagnostic record.
- Raw decoded samples are retained in preallocated typed-array ring buffers with
  a fixed byte budget (default 64 MB, configurable), which is the actual
  retention limit; the equivalent duration is displayed to the user rather than
  assumed.

Before the oldest records are overwritten, the UI warns the user to download the current log. This does not restore embedded flash/SD recording.

### `ui.js`

Displays:

- Master and node connection state;
- BNO and ICM effective rate and packet gaps for each node;
- session state and warm-up progress;
- current class and three probabilities;
- prediction and haptic timeline;
- inference latency;
- estimated cross-node skew and drift;
- haptic armed/disarmed status and any active blanking interval;
- clear reasons whenever inference is unavailable.

## 8. Live Data Flow

1. User serves and opens the live page.
2. Page loads ONNX Runtime, the model, the model contract, and feature names.
3. User connects the page to the Master.
4. Master confirms N2, N3, and N4 connectivity.
5. User starts a live session. The browser sends a common session start and requests the qualified 40 ms interval.
6. Nodes stream BNO and ICM samples through the Master without flash or SD writes, reporting their effective interval and decimation counters.
7. Browser decodes, timestamps, health-checks, synchronizes, and buffers samples.
8. After a complete healthy two-second window exists, and provided the window intersects no haptic blanking interval, preprocessing emits 576 `float32` features.
9. ONNX inference returns a class and three probabilities.
10. UI updates and the session log records the result.
11. If manually armed, the haptic controller evaluates the result and may send one bounded pulse.
12. Stopping the session clears buffers, disarms haptics, stops forwarding, and leaves the log available for download.

## 9. Classification and Feedback Rules

The browser uses the model contract's thresholding policy:

- probability threshold: 0.70
- required consecutive incorrect windows: 2
- haptic cooldown: 2 seconds

Class mapping is:

| Class | UI result | Haptic result |
|---|---|---|
| `correct` | Correct curl | No vibration |
| `incomplete_range` | Incomplete range of motion | Pulse N2 wrist motor |
| `elbow_movement` | Elbow moving forward/backward | Pulse N3 elbow motor |

An incorrect class triggers only when the same incorrect class reaches at least 0.70 probability in two consecutive inference windows. A class change resets the consecutive count. `correct`, low confidence, invalid data, or degraded state resets the incorrect count. No feedback is sent while disarmed or during cooldown.

Because V1 has no rest or transition class, the user explicitly starts and stops the active exercise set. Haptics never arm automatically.

### 9.1 Haptic blanking

The motors driven for feedback sit on N2 and N3 — the same nodes whose IMUs feed
the classifier. A 250 ms pulse plus mechanical ring-down injects energy into the
accelerometer and gyroscope channels, and because windows are two seconds long
and the stride is under 0.5 seconds, a single pulse falls inside four or more subsequent
inference windows. The 2-second cooldown limits repetition but does not address
contamination.

The browser shall therefore mark a blanking interval on the pulsed node running
from pulse start to pulse end plus a ring-down margin (default 250 ms,
configurable). Any inference window whose time range intersects a blanking
interval on any required node is invalid and is skipped and logged, exactly like
any other invalid window. The bench test in Section 12 shall measure the actual
ring-down duration on hardware and the margin shall be set from that
measurement, not from the default.

## 10. Health and Failure Handling

The application state machine is:

```text
Disconnected -> Connecting -> Ready -> Warming Up -> Inferring
                                 ^         |              |
                                 |         v              v
                                 |      Stopped        Degraded
                                 |                        |
                                 +------------------------+
                                   recovery re-enters
                                   Warming Up with an
                                   empty window buffer
```

Inference becomes unavailable and haptics disarm when any of these occurs:

- N2, N3, or N4 disconnects;
- a required BNO or ICM stream becomes stale;
- timestamps move backward or cannot be aligned;
- packet loss or gaps make the current window invalid;
- the window lacks any required channel;
- the model or contract fails validation;
- inference returns non-finite values or unexpected dimensions;
- a Node reports a congestion demotion of its effective live interval, so the
  stream no longer meets the model contract rate;
- estimated cross-node clock skew exceeds the skew gate below;
- the browser page is suspended or hidden long enough to interrupt timely processing;
- the session is stopped.

In `Degraded`, the page may continue displaying and logging incoming data, but the result is `Result unavailable` and haptic commands are prohibited. Recovery requires fresh healthy data and a new complete two-second window. Haptics remain disarmed after recovery until the user arms them again.

The health gates are defined relative to the qualified contract interval `T`
(40 ms at the 25 Hz contract) so that they stay mutually consistent if the rate
changes:

| Gate | Threshold | At 25 Hz (`T` = 40 ms) |
|---|---|---|
| Stream stale | no packet for `3T` | 120 ms |
| Max interpolation span | `1.5T` | 60 ms |
| Missing packets per window | more than 2% of expected | 1 of 50 |
| Cross-node skew | estimated offset error above `0.5T` | 20 ms |

The previous draft paired a 200 ms staleness limit with a 2%-per-window loss
limit; those disagree by roughly 5x, so a stream could be reported healthy while
every window failed validation. The table above replaces both with one scale.

A candidate inference window is also invalid when any required value is
non-finite, when it intersects a haptic blanking interval (Section 9.1), or when
any contributing Node reported a congestion demotion covering that time range.

These thresholds are health gates rather than substitute samples: invalid
windows are skipped and logged. The browser never fabricates samples to satisfy
a gate.

## 11. Sampling-Rate Budget and Qualification

### 11.1 Byte budget

The required rate can be settled from the existing packet definitions before any
firmware is written. Browser-bound bytes per sample are the 14-byte B1 envelope
plus the payload:

| Stream | Payload | Bytes on the wire |
|---|---:|---:|
| BNO85 (`Bno85SampleV3`) | 56 | 70 |
| ICM45686 (`Icm45686SampleV4`) | 20 | 34 |

One node streaming both sensors at rate `R` costs `104R` bytes per second;
three nodes cost `312R`:

| Per-sensor rate | Three-node payload throughput |
|---:|---:|
| 50 Hz | ~15.6 KB/s |
| 25 Hz | ~7.8 KB/s |

The demonstrated Master-to-browser throughput on this hardware is approximately
9 KB/s. The 50 Hz contract needs roughly 1.7x that and is not reachable;
25 Hz fits with thin margin. Quantizing the BNO payload to int16 would reduce
50 Hz to about 9.9 KB/s, which is still above the measured ceiling and would
also invalidate the existing decoder and training-data scaling, so it is not
pursued for V1.

Independently, `NodeLiveSampleQueue::clamp_interval()` floors the live interval
at 40 ms (Section 6.2), so 50 Hz cannot even be requested without changing a
firmware constant.

**Conclusion: the live contract rate for V1 is 25 Hz**, 40 ms interval, two-second
windows of 50 samples, a 12-sample (0.48 s) stride, 72 channels, 576 features. The model
must be retrained in Colab at 25 Hz before inference integration proceeds. This
is a planned step, not a contingency.

### 11.2 Qualification run

Before the model is integrated, the live build runs a 60-second qualification
test at the 40 ms interval with all six streams active. The test reports
per-stream effective rate, sequence gaps, packet loss, longest stale interval,
cross-node skew and drift, congestion demotions, valid-window percentage, and
transport throughput.

Qualification passes when, over the 60-second run:

- every stream averages at least 23.75 samples per second;
- sequence loss is no more than 1% per stream;
- no stale interval exceeds 120 ms;
- no Node reports a congestion demotion;
- estimated cross-node skew stays within 20 ms;
- at least 95% of scheduled inference windows are valid.

If 25 Hz also fails to qualify, the measured stable rate is used and the model is
retrained again at that rate. Under no circumstance does the implementation claim
compatibility by resampling a lower-rate stream onto a higher-rate grid.

### 11.3 Retrain procedure

This is the concrete work behind step 3 of Section 14. It is a re-run of the
existing notebook with a different resampling rate, not new modelling work.

1. Work in `host/notebooks/Vantare_Bicep_Curl_Training_ONNX.ipynb`. Keep the
   current 50 Hz outputs; write the new ones under a `v1_1` name so the V1
   artifacts remain available for comparison.
2. Change the resampling target from 50 Hz to 25 Hz, the window length from 100
   samples to 50, and the stride from 25 samples to 12. Leave the channel
   construction, feature list, feature ordering, and model type untouched.
3. Build the 25 Hz features by **decimating** the full-rate recordings the way
   the live path will see them — take the sample nearest each 40 ms grid point
   rather than averaging or low-pass filtering the full-rate data. Averaging
   would produce cleaner features than the live stream can ever deliver and
   would silently inflate the scores.
4. Re-run the same two complete-session-held-out folds (train 1/3/5 test 2/4/6,
   then the reverse). Do not introduce a random row-level split; sessions must
   stay whole, per `dataset/README.md`.
5. Regenerate `model_contract.json` with `target_hz: 25`, `window_samples: 50`,
   `stride_seconds: 0.48`, and a `stride_samples: 12` field. `synchronized_channels`
   stays 72 and `features` stays 576 — the feature count is per-channel
   statistics and does not depend on the sample rate.
6. Regenerate `feature_names.json` and confirm it is byte-identical to the 50 Hz
   version. If it is not, the channel construction changed and something in step
   2 went further than intended.
7. Re-export ONNX and re-run the notebook's own ONNX agreement check. The V1
   run achieved label agreement 1.0 and a maximum probability difference of
   about 5.8e-6; the new run should be in the same range.
8. Record the new fold metrics in `metrics.json` alongside the V1 numbers.
   Expect the 25 Hz scores to be **lower** than the 50 Hz scores. That is the
   correct and expected outcome of halving the input rate, not a bug to tune
   away. A large drop is informative; a score that stays identical or improves
   is a signal that something is wrong with the decimation in step 3.

Do not tune thresholds, add features, or change the model family during this
retrain. The only variable being changed is the sample rate. Any other change
invalidates the comparison against the V1 metrics and against the parity tests
in Section 12.

## 12. Verification Strategy

### Firmware tests

- Live samples enter the forwarding queue when flash is disabled.
- The live build's minimum preview interval matches the qualified contract interval, and a requested interval is not silently clamped to a different value.
- Queue decimation and congestion counters and the effective interval reach the browser.
- Live build performs no flash or SD writes.
- Start, stop, and interval commands reach N2-N4.
- Master preserves node, sensor, sequence, and timestamp identity while relaying.
- Haptic commands route only to the requested node.
- Motor stops locally at the duration bound.
- Duplicate, stale, or reordered event identifiers do not produce extra pulses.
- Invalid target, intensity, or duration is rejected.
- Recording build tests continue to pass unchanged.

### Browser unit tests

- B1 and compact sensor fixtures decode to expected physical units.
- Buffering is bounded and ordered.
- Gap, staleness, and timestamp failures invalidate a window.
- Quaternion normalization, interpolation, magnitudes, and relative angles match reference fixtures.
- The generated feature vector has 576 finite values in the exact required order.
- Confidence, consecutive-window, cooldown, and manual-arm rules behave deterministically.
- Any degraded condition disarms haptics.

### Python-to-browser parity test

A fixed captured sensor fixture is processed through the Colab/Python preprocessing and through the browser preprocessing. Feature values must agree within an absolute or relative tolerance of `1e-4`. For the same feature vectors, browser and Python ONNX execution must select identical labels and each class probability must differ by no more than `1e-4`.

### Session replay parity test

The fixture test above validates arithmetic only. Because both sides are handed
an already-aligned array, it is structurally blind to time-base, ordering, and
buffering defects — which Section 7.1 identifies as the largest accuracy risk.

A second test therefore replays a complete recorded session as a stream of B1
envelopes into the browser transport and preprocessing path, with realistic
packet ordering and arrival timing, and compares the resulting per-window labels
and probabilities against the notebook's output for the same session. Labels
must match on at least 99% of windows, and probability differences must stay
within `1e-3`. Any window the browser invalidates must be reported with its
reason rather than counted as a mismatch.

### Training-to-live distribution check

The training features were computed by resampling full-rate flash recordings
(BNO at 100 Hz, ICM at 200 Hz). Live features will be computed from a 25 Hz
decimated, jittered stream. Identical feature code does not imply identical
feature distributions: standard deviation, interquartile range, and mean
absolute difference are especially sensitive to the source rate.

Section 11.3 already requires the retrain to build its features by decimation
for this reason. This check confirms it actually happened: compare the feature
distributions (mean, std, IQR, mean-abs-diff per channel) between the retrained
notebook features and features computed from a real live capture of the same
movements. Systematic differences — live std consistently higher, or live
mean-abs-diff consistently lower — indicate the notebook is still smoothing
where the live path cannot, and the retrain must be redone before the real-world
test is meaningful.

### End-to-end bench test

Run N2, N3, and N4 for at least 60 seconds. Verify stream rates and gaps, prediction cadence at the contract stride after warm-up, visible health state, bounded memory behavior, correct log contents, and haptic auto-off.

The bench test also measures two values the design currently assumes: the motor
ring-down duration on each node, which sets the haptic blanking margin
(Section 9.1), and the observed cross-node skew and drift, which confirm the
20 ms skew gate is achievable in practice.

### Real-world V1 test

Perform labeled sets of correct, incomplete-range, and elbow-movement curls that were not part of the training recordings. Review predictions by set and by time, confusion matrix, per-class precision/recall/F1, confidence distribution, false haptic events, missed feedback events, and end-to-end latency. These results, not the notebook score alone, determine whether V1 is useful on the wearable.

The pass/fail bar is fixed before the run so the result is not graded after the
fact. V1 is considered useful on the wearable when, over at least 60 labeled
repetitions covering all three classes:

- per-class recall is at least 0.70 for each of the three classes;
- false haptic events during `correct` repetitions are at most 1 per minute;
- at least 80% of repetitions that should trigger feedback do so within 1.5
  seconds of the relevant portion of the movement;
- end-to-end latency from movement to haptic pulse stays below 1 second.

Missing the bar is a result, not a failure of the exercise: it directs work
toward more sessions and a V2 model rather than toward tuning thresholds until
the numbers move.

## 13. Acceptance Criteria

- Existing recording firmware and data-collection webpage still operate independently.
- Live firmware performs no Node flash or Master SD recording.
- The browser uses one BLE connection to the Master and receives N2-N4 BNO and ICM data.
- The model is retrained at the 25 Hz live contract rate (Section 11) and sampling-rate qualification passes at that rate before inference integration continues.
- No stream is ever resampled onto a grid faster than its measured rate.
- A valid result is produced once per contract stride step after a two-second warm-up.
- Every inference window contains all required channels from all three nodes.
- Cross-node time base is estimated, displayed, logged, and gated (Section 7.1).
- Windows overlapping a haptic pulse and its measured ring-down are invalidated (Section 9.1).
- A Node congestion demotion degrades the session rather than silently halving the effective rate.
- Browser features and ONNX outputs pass the Python parity tolerances.
- The session replay parity test passes on a full recorded session.
- The ONNX Runtime Web version and execution-provider flags are pinned and recorded.
- `correct` never intentionally triggers vibration.
- `incomplete_range` targets N2 and `elbow_movement` targets N3.
- Incorrect feedback requires two consecutive predictions at or above 70% for the same class.
- A two-second cooldown limits repeated feedback.
- Haptics start disarmed, require manual arming, auto-stop locally, and disarm on every degraded condition.
- The page clearly reports unavailable results and their reason.
- Downloaded logs contain samples, health events, predictions, probabilities, latency, decisions, skew estimates, and haptic events, within a stated memory budget.
- The live wearable evaluation is reported separately from the six-session notebook evaluation, against the numeric bar in Section 12.

## 14. Delivery Sequence

Steps 1 and 2 are independent of each other and of the Colab work in step 3, so
the retrain can proceed in parallel with firmware changes.

1. Decouple Node live forwarding from flash recording, expose the queue
   decimation/congestion counters, and establish live build profiles.
2. Add bounded haptic-pulse semantics, the identifier rule, and firmware tests.
3. Retrain in Colab at the 25 Hz live contract rate by the procedure in
   Section 11.3, and regenerate `model_contract.json` and `feature_names.json`.
4. Build the modular live browser transport, time-base estimation, and health
   monitoring. This includes the qualification harness, which is a subset of it.
5. Run the 60-second qualification test (Section 11.2). Do not proceed until it
   passes or the contract rate is revised again.
6. Port preprocessing with Python-to-browser golden fixtures, then add the
   session replay parity test.
7. Integrate the pinned ONNX Runtime Web and validate model I/O parity.
8. Add the inference UI, logging, and manually armed haptic controller.
9. Run bench tests, measure motor ring-down and cross-node skew, and set the
   blanking margin from measurement.
10. Run the real-world labeled curl test against the Section 12 numeric bar.

The earlier draft placed throughput qualification before the browser transport
that performs the measurement; steps 4 and 5 above correct that ordering.

## 15. Safety and Interpretation

The haptic motor is feedback, not a protective control. Communication loss, inference errors, or misclassification must fail silent by stopping and disarming vibration. The live UI must identify the model as V1 and experimental.

### How to read the existing notebook metrics

`host/notebooks/metrics.json` reports a mean accuracy of about 0.967 across two
complete-session-held-out folds. That number should not be carried into any
claim about live performance, for a specific and checkable reason.

The dataset has six sessions, two per class, and class identity is therefore
perfectly aligned with session identity. Each fold trains on exactly one session
per class and tests on exactly one session per class. Anything that is constant
within a session but not caused by the movement — strap position, sensor
re-mount orientation, the day's mounting angle — is available to the model as a
shortcut that happens to predict the label.

The confusion matrix shows the fingerprint of exactly that. `correct` and
`incomplete_range` confuse each other in both directions, which is what a real
movement classifier looks like when two classes are genuinely similar. But
`elbow_movement` is separated perfectly: 212 of 212, with zero errors in either
direction. A perfect column from two sessions of one person is much more likely
to reflect a session-level cue than a genuinely unmistakable movement.

The practical consequences for this design:

- The 0.967 figure is not a generalization estimate and must not appear in any
  user-facing or external description of V1.
- The real-world labeled test in Section 12, against its pre-declared numeric
  bar, is the only evidence that counts for the wearable.
- If live `elbow_movement` performance falls well below its notebook score while
  `correct` and `incomplete_range` roughly hold, that is confirmation of the
  shortcut rather than a live-path defect, and the fix is more sessions with
  varied sensor mounting, not threshold tuning.
- A future V2 needs more sessions per class, recorded across different mounting
  occasions, so that session identity and class identity stop coinciding.

Beyond this, the metrics say nothing about different users, sensor placements,
fatigue levels, curl speeds, body types, or environmental conditions. Those
require new labeled sessions and a future model version.
