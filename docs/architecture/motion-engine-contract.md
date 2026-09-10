# Motion Engine — data contract for the 3D / app team

Status: **validated on hardware 2026-09-10.** Neutral + hinge calibration and
signed flexion all confirmed on a real arm; see "Field results" below. Remaining
gap: an isolated yaw-drift measurement.
Implementation: `host/live_tool/js/motion-engine.js`
Tests: `host/tests/python/test_motion_engine.py` (19 tests, all passing)
Source requirement: *Coach Assist Requirements, Milestone Alignment & Movement
Tracking Plan* — sections 6, 7 (step 3/5) and 8.

## Purpose

Give the 3D/app team body-motion data instead of sensor data. Nothing in this
contract exposes BLE packets, STM32 structures, BNO/ICM registers or model
features, and none of it depends on the ML model — the Motion Engine runs
whether or not a model is loaded.

## The packet

One packet per motion tick, 25 Hz, matching the qualified live grid. This runs
on its own timer rather than the render tick — the render tick also fires on
haptic state changes, which produced an irregular 2–258 ms cadence that was
unusable for measuring rep tempo.

```js
{
  timestamp_ms: 1234567,
  upper_arm_orientation: { qx, qy, qz, qw } | null,   // N4
  forearm_orientation:   { qx, qy, qz, qw } | null,   // N2
  elbow_relative_rotation_deg: 87.4 | null,   // unsigned composite
  elbow_flexion_deg:            87.1 | null,   // signed, about the measured hinge
  elbow_off_axis_deg:            2.3 | null,   // how un-hinge-like the motion is
  elbow_off_axis_excess_deg:     2.3 | null,   // off-axis minus the drift estimate
  upper_arm_deviation_deg:      12.1 | null,
  health: { n2, n3, n4, synchronized, calibrated },   // all booleans
  diagnostics: { ... }                                // ours, not part of the contract
}
```

### Field meanings

| Field | Meaning |
|---|---|
| `upper_arm_orientation` | Rotation of the upper arm **since the neutral calibration pose**. Identity at neutral. |
| `forearm_orientation` | Same, for the forearm. |
| `elbow_relative_rotation_deg` | Rotation between the two segments, referenced to neutral. `0` = calibration pose. Unsigned, range `[0, 180]`. Kept for continuity; **prefer `elbow_flexion_deg`**. |
| `elbow_flexion_deg` | Signed rotation about the measured hinge axis — the real joint angle. `null` until a hinge calibration has run. |
| `elbow_off_axis_deg` | The part of the motion that was *not* about the hinge. Near zero on a clean rep; large on compound movement. |
| `elbow_off_axis_excess_deg` | Off-axis with the drift estimate removed. **Threshold on this, not the raw value** — drift inflates the raw term by ~10°/min. |
| `upper_arm_deviation_deg` | How far the upper arm has moved from its neutral pose. |
| `health.n2/n3/n4` | That node produced a sample within the last 250 ms. |
| `health.synchronized` | Device-time skew between N2 and N4 is within 60 ms. |
| `health.calibrated` | A neutral pose has been captured and accepted. |

### Rules for the consumer

- **Never treat a value as valid without checking `health`.** Angles are `null`
  when uncalibrated or when a required node is stale. Render a degraded state;
  do not hold the last good pose and present it as live.
- The packet is emitted even when unhealthy, deliberately — so a consumer can
  distinguish "degraded" from "the feed stopped."
- Quaternions are `null` before calibration, never identity-as-a-placeholder.

## Why the angles are trustworthy on the first wear

The strap orientation of each PCB is unknown and different every time the suit
is worn. Writing `M_n` for that unknown constant mount rotation and `q_n(t)` for
the reported quaternion, the true segment orientation is `q_n(t) * M_n`.

Calibration stores the averaged neutral quaternion `q_ref[n]` and everything is
reported relative to it:

```
D_n(t) = conj(q_ref[n]) * q_n(t)
```

The segment-frame equivalent is `conj(M_n) * D_n(t) * M_n` — a conjugation, and
rotation *angle* is invariant under conjugation. So the reported angles are
exactly the true angles regardless of strap position. `test_motion_engine.py`
proves this by generating sensor data through deliberately awkward mount
rotations the engine is never told about and requiring exact recovery.

## Limits — read before mapping bones

These are real and currently unresolved. They are also carried in-band in
`diagnostics` so they cannot be forgotten.

1. **Axes are in the sensor's neutral frame, not the anatomical frame**
   (`diagnostics.axisFrame === "sensor_neutral"`). Angle magnitude is correct;
   the *axis* the avatar rotates about is off by the residual mount rotation.
   Fix: a two-pose calibration (neutral + full flexion) to observe the flexion
   axis and populate `MotionEngine.mountCorrection`. The seam exists in the code
   and is applied automatically once filled. Scheduled after numeric validation.

2. **No trunk reference** (`diagnostics.trunkReferenced === false`).
   `upper_arm_deviation_deg` is measured from the calibration pose, not from the
   torso, so it degrades if the athlete rotates their torso mid-rep. The final
   suit adds lower-back and neck nodes plus a chest-mounted Master; the
   `MOTION_ROLES.torso` placeholder is where that lands.

3. **Yaw drifts.** The BNO085 runs `SH2_GAME_ROTATION_VECTOR` — no magnetometer
   — so each sensor's heading is arbitrary at power-up and drifts independently.
   Calibration cancels the inter-sensor offset at t0 but not afterwards.
   Elbow flexion is largely protected (drift common to both sensors cancels
   exactly out of the relative rotation; there is a test for this). Absolute
   upper-arm heading is not protected. `diagnostics.yawDriftHintDeg` exposes the
   observable part; it is reported, never silently corrected.

4. **~~Elbow rotation is unsigned.~~ RESOLVED** by the hinge calibration: run a
   range calibration (a few slow full reps) and `elbow_flexion_deg` becomes a
   signed angle about the joint's measured axis, with `elbow_off_axis_deg`
   carrying the leftover. Until that calibration runs, only the unsigned
   composite is available. Note this does *not* yet fix limit 1 — the exported
   quaternion axes are still in the sensor's neutral frame.

5. **Arm only.** Three nodes support an instrumented arm model, not full-body
   reconstruction.

## Field results — session 2026-09-10T04:31

First run on a real arm. `tmp/vantage_live_2026-09-10T04-31-43-629Z.ndjson`,
705 motion frames, calibration accepted after a 1505 ms hold on N4/N2/N3.

| Phase | Peak `elbow_deg` | Max `upper_dev_deg` | Duration | Hinge-axis spread |
|---|---|---|---|---|
| Reps 1–4 (clean) | 139.7 / 132.0 / 128.6 / 128.5 | 7.5–9.4 | 1.76–2.01 s | mean 1.9°, max 6.3° |
| Reps 5–6 (compound) | 176.0 / 179.3 | 68.1 / 51.2 | 3.8 / 5.7 s | mean 60.2°, max 175.7° |
| Reps 7–8 (partial) | 108.2 / 112.2 | 12.6 / 17.0 | ~1.2 s | mean 3.0°, max 10.0° |

Reads on this:

- **Clean reps are coach-grade.** Four consecutive curls peaked within an 11°
  band with the upper arm held inside 10°. The measurement is repeatable.
- **The hinge axis is observable.** During clean reps the rotation axis holds to
  ±2–3°, and the partial reps land 5.6° away from the same axis. This is direct
  evidence that the two-pose/range calibration for signed flexion will work.
- **The unsigned composite angle breaks down on compound motion.** Reps 5–6 read
  176–179°, which is not elbow flexion — the axis spread of 60° shows the motion
  was not a hinge rotation at all, and the unsigned angle folded flexion together
  with forearm rotation. 179.9° is at the `[0, 180]` limit, past which readings
  wrap back down. Limit 4 above is therefore not theoretical: it fails exactly
  when form is worst, which is when the number matters most.
- **Drift.** At rest, `elbow_deg` went from 0.76° to 6.44° over ~30 s
  (`upper_dev_deg` 0.61° → 15.58°, but that is confounded with the arm not
  returning to an identical posture). An isolated drift test — calibrate, hold
  still 2 minutes — is still needed.
- **Transport.** 685/705 frames synchronized; worst skew 92 ms. Two `degraded`
  events fired during warm-up, before calibration.

## Field results — session 2026-09-10T04:58 (nine clean reps)

`tmp/vantage_live_2026-09-10T04-58-14-538Z.ndjson`. Neutral accepted after a
1555 ms hold; nine consecutive curls, no deliberate faults.

| Metric | Result |
|---|---|
| Peak elbow | mean **133.1°**, sd **5.0°**, range 125.7–142.5 |
| Max upper-arm deviation | mean 6.9°, worst 8.5° |
| Rep duration | mean 2.22 s, sd 0.59 s |
| Hinge-axis spread (9 reps, 30 s) | mean **2.3°**, p95 5.2°, max 13.0° |
| Skew | median 11.8 ms, p95 36.5 ms, max 132.2 ms |

- The axis held to 2.3° across nine reps and thirty seconds. That is what
  justified building the hinge calibration rather than deferring it.
- The recovered axis differs by 15.6° from the previous session's. Expected and
  correct: the axis lives in the sensor frame, so a re-strap moves it, while the
  *angles* survived the remount. Re-run the hinge calibration after every
  re-strap, exactly as for the neutral pose.
- Between-rep resting elbow rose 2.3° → ~11° over 28 s and then plateaued, while
  `upper_dev_deg` showed no matching trend. A sensor drift would move both and
  keep growing; a plateau in one signal looks more like the athlete stopping
  short of full extension as the set goes on — a real coaching cue, not an
  artifact. **Not yet proven either way**: the isolated drift test (calibrate,
  hold still two minutes) is still outstanding.

## Field results — session 2026-09-10T05:45 (signed flexion, validated)

`tmp/vantage_live_2026-09-10T05-45-31-556Z.ndjson`. First session on the
complete pipeline: 928 motion frames at a clean **25.0 Hz**, neutral accepted
after 1522 ms, hinge axis accepted from 40 samples at **3.13° mean spread**
(max 7.63°). Nine curls.

| Metric | Result |
|---|---|
| Peak flexion | mean **131.6°**, sd **4.0°**, range 122.9–137.3 |
| Max off-axis per rep | mean 7.8°, worst 12.1° |
| Max upper-arm deviation | mean 5.8°, worst 8.1° |
| Rep duration | mean 1.72 s, sd 0.30 s |
| Off-axis, whole set | median 4.5°, p95 7.9°, max 12.1° |
| Skew | median 12.3 ms, p95 45.9 ms; 554/564 frames synchronized |

- **Signed flexion is tighter than the unsigned composite** it replaces (sd 4.0°
  vs 5.0° on the previous set), because the off-axis component no longer leaks
  into the peak. Signed and unsigned differ by at most 6.2° on clean reps, which
  is exactly the off-axis term being removed.
- **Off-axis gives a usable baseline for compound motion.** A clean rep sits
  under ~8°; the compound reps in the 04:31 session had 60°+ axis incoherence.
  A threshold somewhere around 20–25° should separate them — to be confirmed
  against a deliberately-bad set.
- **Motion survived a model-path degradation.** At 21.0 s the inference session
  degraded (`N2 clock skew 20.6 ms exceeds 20 ms`, the model's stricter 20 ms
  gate) while motion output continued uninterrupted. That independence is the
  design intent, now demonstrated rather than asserted.

### Recommendation for the 3D team

**Drive the elbow bone with the scalar `elbow_flexion_deg` about the rig's own
elbow axis — do not apply `forearm_orientation` as a raw quaternion.** Limit 1
below (axes in the sensor's neutral frame) then does not affect the elbow at
all, because a scalar angle carries no frame. The shoulder still needs the full
mount correction, so treat upper-arm orientation as provisional until then.

## Field results — 2026-09-10 drift and random-movement tests

Two targeted tests, both of which changed the implementation.

### Drift (05:59, 60 s of held stillness)

| Metric | Drift rate | Residual noise |
|---|---|---|
| Elbow flexion | **−4.1°/min** | sd 0.44° |
| Off-axis | **+9.9°/min** | sd 0.97° |
| Upper-arm deviation | +0.03°/min | sd 0.55° |
| Unsigned elbow | +10.3°/min | sd 0.71° |

Residual noise under 1° means this is genuine drift, not sensor noise. Each
segment's own deviation is flat, so the drift is in the *relative* orientation
— N2 and N4 pulling apart, exactly the magnetometer-free GRV behaviour.

The important part: **the hinge projection isolates it.** Drift lands almost
entirely in the off-axis term, leaving flexion drifting at only −4.1°/min. But
it also means any fixed off-axis threshold decays over a session, which is why
the gate thresholds `elbow_off_axis_excess_deg` rather than the raw value.

Practical consequence: flexion is good for roughly 2–3 minutes per calibration
(~8–12° of accumulated error). `diagnostics.recalibrationRecommended` flags
when the estimate passes 15°. `rezeroFromRest()` clears it from a rest
observation without redoing the neutral pose or the hinge axis.

### Random movement (06:02)

| Phase | Off-axis (median / p95 / max) | Max upper-arm dev |
|---|---|---|
| Curl-like opening | 11.7 / 28.2 / 90.4 | 21.7 |
| Random movement | **104.1 / 155.4 / 178.7** | 135.1 |

Off-axis separates compound motion from clean reps by an enormous margin
(clean curls: median 4.5°, max 12.1°).

It also exposed a defect: **26% of random-movement frames reported
|flexion| > 150°**, which an elbow physically cannot do. Swing-twist about a
fixed axis is only meaningful while the motion is roughly about that axis; past
that the twist term runs to ±180° as the quaternion's scalar part nears zero.
The engine was reporting those confidently.

Fixed with two guards, validated by replaying both sessions:

| Gate | Effect |
|---|---|
| `elbow_off_axis_excess_deg > 35°` | catches 78% of impossible frames |
| `\|flexion\| > 150°` (anatomical limit) | catches the remainder |
| **Combined** | **0 impossible values survive**, 96% of random-movement frames withheld, and all clean-curl frames retained (peaks 137–140°) |

When flexion is withheld, `diagnostics.flexionValid` is `false` — distinct from
`null` meaning no hinge calibration has run. The unsigned angle, off-axis term
and segment quaternions continue to be reported.

## Calibration behaviour

Press **Calibrate Neutral Pose**, then hold still. The engine requires ~1.5 s of
contiguous stillness with:

- gyro magnitude below 0.2 rad/s on every node, and
- orientation spread under 3° across the hold, and
- both required nodes (N2, N4) fresh and contributing.

Any violation discards the whole accumulated window and restarts the hold — a
reference averaged across a twitch would blend two poses, which is worse than
asking the wearer to hold again. The attempt fails after 15 s.

Then press **Calibrate Hinge** and perform a few slow full reps. The engine
collects the elbow rotation axis from frames past 40° of flexion and accepts it
once 40 samples agree to within 15°. The direction of the reps defines positive
flexion, so the sign is meaningful without assuming which arm is instrumented or
how the PCB sits. Incoherent motion is rejected rather than averaged into a
meaningless axis. Clearing the neutral calibration also clears the hinge axis,
because the axis is expressed relative to that neutral reference.

## Rep layer (Coach Assist correction loop)

`host/live_tool/js/rep-analyzer.js` consumes these packets and produces per-rep
verdicts. It inherits the Motion Engine's independence from the model path.

Segmentation is hysteresis on `elbow_flexion_deg` (open above 30°, close below
20°), with segments shorter than 0.4 s or longer than 6.0 s rejected as *not a
rep* — they never reach the coach's rep count.

Coach-adjustable target, with defaults measured on this rig:

| Check | Default | Basis |
|---|---|---|
| `romTargetDeg` | 120° | clean sets peaked 131.6° ± 4.0° |
| `upperArmToleranceDeg` | 15° | clean 6.4–9.8°, bad 18.9–103.5° |
| `maxOffHingeFraction` | 0.10 | clean 0%, gross swinging 25–90% |

**Which metric carries the judgement.** From the bad-form set (2026-09-10T06:43):

| Metric | Clean reps | Bad reps | Verdict |
|---|---|---|---|
| Upper-arm deviation | 6.4–9.8° | 18.9–103.5° | **clean split** |
| Off-axis excess | 6.1–23.8° | 17.2–91.7° | overlaps |
| Peak flexion | 132.8–137.6° | 96.2–149.3° | overlaps |

Upper-arm deviation is the discriminator, with a 15° threshold in the middle of
a wide empty gap. Peak flexion catches short reps but cannot stand alone — a
swung rep often reaches *full* range precisely because it was swung. Off-axis
excess is deliberately **not** used as a form grade despite being the obvious
candidate: its clean and bad ranges overlap, so thresholding it would both
false-alarm on good reps and miss real faults. It serves as a validity signal
instead, through the off-hinge fraction.

Replaying the recorded bad-form session through the analyzer reproduces the
manual per-rep analysis: 3/10 correct, `upper_arm_movement` on all seven
faulted reps, and the two prolonged flailing segments rejected as uncountable.

## Logging

Each frame is written to the session log as the `motion` stream, columns in
`MotionEngine.LOG_COLUMNS`, so recorded runs can be replayed and compared.
Missing values are `NaN`, which serializes to JSON `null` — not a sentinel
number, because `-1` is a legitimate signed flexion angle.
