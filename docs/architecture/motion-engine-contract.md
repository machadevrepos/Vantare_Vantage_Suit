# Motion Engine — data contract for the 3D / app team

Status: **validated on hardware 2026-09-10.** Neutral + hinge calibration and
signed flexion all confirmed on a real arm; see "Field results" below. The
three-pose anatomical calibration (below) is implemented and host-verified;
**it has not yet had its physical acceptance run** — that is the documented
handoff at the end of this document. Remaining gap: an isolated yaw-drift
measurement.
Implementation: `host/live_tool/js/motion-engine.js`,
`host/live_tool/js/anatomical-calibration.js` (pure TRIAD solver),
`host/live_tool/js/arm-avatar.js` (nested display rig)
Tests: `host/tests/python/test_motion_engine.py` (32 tests, all passing),
`host/tests/scripts/test_anatomical_calibration.mjs`,
`host/tests/scripts/test_arm_avatar.mjs`,
`host/tests/scripts/replay_arm_avatar.mjs` (session-log replay audit)
Source requirement: *Coach Assist Requirements, Milestone Alignment & Movement
Tracking Plan* — sections 6, 7 (step 3/5) and 8. Three-pose design:
`docs/superpowers/specs/2026-09-10-three-pose-anatomical-arm-calibration-design.md`.

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

1. **~~Axes are in the sensor's neutral frame, not the anatomical frame~~ RESOLVED**
   by the three-pose anatomical calibration (section below): two held
   straight-arm directions observe each sensor's mount rotation, and the engine
   installs per-node corrections so `upper_arm_orientation` /
   `forearm_orientation` land in the anatomical arm frame
   (`diagnostics.axisFrame === "anatomical"`). Until that workflow runs, the
   axis-frame flag honestly reads `sensor_neutral` and only angle *magnitudes*
   are trustworthy.

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
   observable part; it is reported, never silently corrected. The anatomical
   avatar makes this limit **visible**: it renders absolute segment
   orientations, and the measured drift concentrates in the N2-vs-N4
   inter-sensor offset, so expect the forearm to wander (~10°/min in the
   05:59 stillness test) while flexion stays protected. Recalibrate when
   `diagnostics.recalibrationRecommended` flags.

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

**After the three-pose workflow** (`diagnostics.axisFrame === "anatomical"`):
consume `upper_arm_orientation` and `forearm_orientation` directly and nest the
forearm as `conjugate(upper) * forearm` at the elbow pivot, exactly as
`arm-avatar.js` does — the mount rotation is already removed and the axes are
anatomical. **Before it** (`sensor_neutral`): fall back to the previous
guidance — drive the elbow bone with the scalar `elbow_flexion_deg` about the
rig's own elbow axis, and treat upper-arm orientation as provisional, because a
scalar angle carries no frame and survives the unknown mount. Check the flag
per session; it changes with calibration state, not with the binary.

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

## Three-pose anatomical calibration

The neutral pose removes each sensor's *reference* but not its *mount*: a
sensor strapped 30° about the arm still reads 30°-off axes. One more held pose
cannot separate mount rotation from movement, but two non-collinear directions
can — a pure TRIAD solver turns the two observed rotation axes into a per-node
mount correction.

**Axis convention (right arm only):** `+X` wearer-right, `+Y` down the neutral
arm, `+Z` forward. Directional targets: side axis `[0, 0, -1]`, forward axis
`[1, 0, 0]`. N4 is the upper arm, N2 the forearm; N3 stays auxiliary.

**Workflow (ordered UI steps):**

1. **Neutral** — arms hanging, palms facing the thighs, ~1.5 s of stillness.
   Starting a new neutral calibration invalidates every later stage.
2. **Right-side raise** — straight arm held ~90° out to the right, palm down,
   still, for the 1 s capture window.
3. **Forward raise** — straight arm held ~90° forward, thumb up (the natural
   no-twist end state; "palm down" here would force a forearm rotation into
   the capture). After this capture both mount corrections are solved and
   installed atomically: a failure on either node leaves `mountCorrection`
   empty and preserves the last completed stage.
4. **Elbow hinge** — needs only the neutral pose (its math is independent of
   the anatomical solve); still required for signed flexion and rep verdicts,
   no longer required for the avatar.

**Capture validation thresholds.** A transient fault — motion, a low raise,
a bent elbow, unsynchronized data — restarts the 1 s capture window and tells
the wearer what to fix; every restart is audited as an
`motion_anatomical_hold_restarted` event with its reason. Only the 15 s
timeout or a solver-level failure ends the attempt, and the timeout message
carries the reason.

| Check | Threshold | On fault |
|---|---|---|
| Stillness | gyro < 0.2 rad/s, orientation spread ≤ 3° | restart window |
| Raise magnitude (per node) | 60–120° from neutral | restart window |
| Segment mismatch (straight elbow) | \|N4 angle − N2 angle\| ≤ 15° | restart window |
| Freshness / sync | both nodes fresh, device-time skew ≤ 60 ms | restart window |
| Solver | axes 60–120° apart, matrix finite, orthonormal, det ≈ +1 | fail attempt (reports the measured separation) |

**Why there is no elbow-relative capture gate** (2026-09-11 field lesson).
An inter-sensor quantity such as `conj(q_upper) · q_fore` looks like the
perfect straight-elbow check — mount rotations cancel out of it — but each
BNO085 Game Rotation Vector also carries its own arbitrary power-up heading,
and those do **not** cancel once the upper arm rotates: a perfectly straight
arm reads roughly 1° of phantom bend per 1° of N2-vs-N4 heading offset, which
rejected every real side raise in the 04:44 session. Every gate that runs
during a directional capture is therefore conjugation-invariant (magnitudes,
axis angles). Twist remains undetectable until the gravity-vector cross-check
exists.

**Why the solve window is 60–120°, not tighter** (second 2026-09-11 field
lesson). An 80–100° window — chosen to match the ≤10° acceptance criterion —
blocked the next session entirely (05:14, "raises were not independent"):
people raise sideways in the scapular plane, 20–30° forward of pure lateral,
landing at 60–75° of observed separation. The TRIAD solve is well-conditioned
anywhere in 60–120°; the trade-off is plane fidelity IF the demo raise uses a
different plane than the capture. Mitigations: the failure message and the
`anatomical_failed` event report the measured separation, the success quality
carries `axisSeparationDeg` per node (≈90° is the target to check), and the
acceptance run instructs deliberate in-plane raises. A bent-elbow side raise
presents the same geometry as a scapular raise and is therefore accepted too
— the upper arm's mount stays exact, the forearm's absorbs the bend and shows
it in the render; closing that gap needs the gravity-vector cross-check, not
a tighter window.

**Pose conventions.** The neutral pose is arms hanging, palms facing the
thighs. From there the natural no-twist raises are **side → palm down** and
**forward → thumb up**; the UI instructions say exactly that, because asking
for any other hand orientation forces a forearm rotation into the capture.

**What capture validation still cannot see: whole-arm twist.** If the arm
rotates about its own long axis during a raise, N4 and N2 rotate together:
mismatch 0°, every gate passes, and the captured axis is contaminated. The
solver's determinant/orthogonality check cannot catch it — TRIAD always
produces a proper rotation. The designed remedy is a **gravity-vector
cross-check**: the BNO already streams `gravity_x/y/z`, which is drift-free
and heading-free and exposes twist, mirrored poses and off-plane raises
independently of the quaternion path. Not implemented yet; the acceptance
run must watch for it.

**Pre-existing, separate from calibration** (2026-09-11 review): the engine's
own `elbow_relative_rotation_deg` — and the signed flexion, off-axis and
hinge-axis quantities built on it — uses the same inter-sensor product, so it
carries the same heading contamination whenever the upper arm *moves*. The
hardware-validated curls are unaffected (upper arm still); swing-type
movements will read a contaminated elbow angle. Once anatomical calibration
is installed, `conj(upper_corrected) · fore_corrected` is the heading-free
replacement; switching the internal computation to it is its own change.

**Diagnostics added:** `anatomicalState`
(`none / side_capturing / side_ready / forward_capturing / calibrated / failed`),
`anatomicalMessage` (next instruction or rejection reason), `anatomicalQuality`
(per-node capture angles, spreads, solver determinant/orthogonality), and
`axisFrame`: `"anatomical"` only when both corrections are installed. Events
`anatomical_side_started / _side_complete / _forward_started / anatomical_complete /
anatomical_failed` go to the session log as Tier 1 `motion_anatomical_*` events
carrying the accepted axes, mount quaternions, capture angles and solver
quality.

### Display kinematics (arm-avatar.js)

The rig renders the nested transform, not a scalar elbow:

```text
q_forearm_relative = conjugate(q_upper_anatomical) * q_forearm_anatomical
```

`q_upper_anatomical` drives the upper arm at the shoulder pivot,
`q_forearm_relative` the forearm at the elbow pivot, and the hand stays a child
of the forearm — so a 90° elbow bend looks identical in every shoulder
direction, and a rigid straight-arm raise leaves the elbow at identity. While
`axisFrame !== "anatomical"` the rig shows
`anatomical_calibration_required` and renders no directional pose. Scalar
`elbow_flexion_deg` remains in numeric diagnostics and rep analysis only.

**Latency policy:** the 540°/s display rate cap is gone. Adaptive
interpolation uses a ~15 ms time constant during deliberate movement, relaxing
up to 60 ms only for sub-degree stationary jitter; shortest-hemisphere
interpolation is mandatory. A missed or unsynchronized frame holds the last
valid pose for **120 ms**, then the rig reports `tracking_unavailable` while
*retaining* the last transform — dropout never writes identity or zero.

### Replay audit

`node host/tests/scripts/replay_arm_avatar.mjs <session.ndjson>` replays a
recorded session through the production avatar path at 60 Hz and prints JSON
metrics: cadence, invalid/dropout frames, per-tick visual-step distribution,
added display lag, `axisFrame`, and — when `motion_anatomical_*` events are
present — the calibrated side/forward direction errors against the anatomical
targets. A legacy log without directional events reports
`axisFrame: "sensor_neutral"` and `directionValidation: "unavailable"`;
missing calibration is reported, never repaired.

### Physical acceptance run (handoff)

After a fresh four-step calibration, record the wearer and screen together and
perform, slowly first, then at normal speed:

1. neutral hold;
2. right-side raise to ~90° and return;
3. forward raise to ~90° and return;
4. elbow curl with the upper arm still;
5. side raise with elbow flexion;
6. one slow circular shoulder movement.

Acceptance: held side/forward display directions within 10° of the instructed
plane; no zero-angle snap on an isolated dropped frame; added display latency
≤ 40 ms median / 80 ms p95; shoulder and elbow pivots stay connected with no
transform flips. A failed criterion is reported as a prototype limitation, not
smoothed away. Do not claim client-demo readiness before this run.

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
