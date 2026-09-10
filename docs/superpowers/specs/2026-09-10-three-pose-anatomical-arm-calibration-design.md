# Three-Pose Anatomical Arm Calibration Design

**Date:** 2026-09-10

**Status:** Proposed for implementation

**Scope:** Right-arm, three-node Coach Assist prototype

**Critical path:** Neutral calibration → anatomical frame calibration → generic arm kinematics → low-latency 3D display

## 1. Outcome

The live tool shall map N4 and N2 orientations into a stable right-arm anatomical frame before they reach the 3D avatar. A guided neutral, right-side raise, and forward raise calibration shall replace the current direct use of sensor-neutral quaternion axes.

After calibration:

- N4 drives the upper-arm orientation.
- N2 drives the forearm orientation relative to N4.
- The hand inherits the forearm orientation because there is no hand node.
- N3 remains an auxiliary elbow-area sensor for health and later validation; it is not treated as a separate rigid body.
- Rendering remains deterministic and independent of the ML classifier.

The calibration is session-only. It must be repeated whenever a node or strap is moved.

## 2. Evidence and root cause

The 2026-09-10 11:03 recording contains 1,142 calibrated motion frames over 45.64 seconds. The motion packet cadence was 40 ms median, 50 ms p95, and 53 ms maximum. N4's largest raw interval was 120 ms. Transport timing is therefore not the primary cause of the directional failure.

The strongest upper-arm movement was 79.2 degrees at 79.776 seconds. Its sensor-neutral rotation axis was approximately `[-0.330, 0.944, -0.016]`. The avatar's neutral arm also lies along local positive Y. Applying that quaternion directly rotates mainly around the length of the avatar arm: the rendered arm tilts only 24.35 degrees from vertical instead of reproducing the approximately 79–90 degree right-side raise.

This matches the documented Motion Engine limitation. Neutral calibration preserves rotation magnitude but leaves the axis in each sensor's neutral frame. `MotionEngine.mountCorrection` is present but is not populated. More filtering cannot correct the wrong coordinate frame, and the present 60 Hz smoother adds perceptible delay while preserving the wrong direction.

## 3. Physical assumptions

- N4 is rigidly strapped to the lateral upper arm, slightly below the shoulder joint.
- N2 is rigidly strapped to the forearm near the wrist.
- N3 is near the elbow and may cross soft tissue; it is not assumed to be a stable third bone segment.
- The wearer faces the display during calibration.
- The elbow remains straight and the wrist does not rotate during the two directional poses.
- This version supports the right arm only. Left-arm mirroring is a later explicit extension.

If N4 is placed on the torso, on top of the acromion, or on loose clothing, the upper arm cannot be reconstructed correctly and calibration must be rejected operationally rather than hidden in software.

## 4. Coordinate convention

The avatar anatomical frame uses:

- `+X`: wearer's right.
- `+Y`: down along the neutral arm.
- `+Z`: forward from the wearer.

For a right arm beginning at the side:

- A right-side raise rotates the neutral `+Y` arm direction toward `+X`, about anatomical `-Z`.
- A forward raise rotates `+Y` toward `+Z`, about anatomical `+X`.

The existing neutral delta for node `n` is:

```text
D_sensor[n](t) = conjugate(q_reference[n]) * q_sensor[n](t)
```

The corrected anatomical delta remains the existing seam:

```text
D_anatomical[n](t) = conjugate(M[n]) * D_sensor[n](t) * M[n]
```

`M[n]` is the mount rotation from the anatomical segment frame into the sensor-neutral frame.

## 5. Guided calibration

### 5.1 Pose sequence

1. **Neutral** — stand upright, right arm relaxed straight down, elbow extended, palm facing the body. Use the existing neutral-pose capture.
2. **Right side** — hold the whole straight arm out to the right, approximately horizontal, without leaning or rotating the torso.
3. **Forward** — return to neutral, then hold the whole straight arm directly forward, approximately horizontal, without rotating the torso.

Each directional capture requires one second of accepted stillness. The UI displays the active instruction, progress, and an actionable rejection reason.

### 5.2 Captured values

For N4 and N2 independently, each directional pose records an averaged neutral-relative quaternion and its folded, consistently signed rotation axis:

```text
side_axis_sensor[n]
forward_axis_sensor[n]
```

The target anatomical axes are:

```text
side_axis_anatomical    = [0, 0, -1]
forward_axis_anatomical = [1, 0,  0]
```

### 5.3 Solving the mount rotation

For each node, form an orthonormal source basis with a TRIAD/Gram-Schmidt construction:

```text
s1 = normalize(side_axis_sensor)
s2 = normalize(forward_axis_sensor - dot(forward_axis_sensor, s1) * s1)
s3 = cross(s1, s2)
```

Form the matching anatomical target basis from the two target axes:

```text
t1 = side_axis_anatomical
t2 = forward_axis_anatomical
t3 = cross(t1, t2)
```

The vector rotation from sensor-neutral axes into anatomical axes is:

```text
R_map = [t1 t2 t3] * transpose([s1 s2 s3])
```

Convert `R_map` to quaternion `Q_map`. Because the Motion Engine applies `conjugate(M) * D * M`, store:

```text
M = conjugate(Q_map)
```

The implementation must check that the resulting matrix is finite, orthonormal, and has determinant approximately `+1` before accepting it.

## 6. Capture validation and failure handling

A directional pose is rejected unless all conditions hold:

- N4 and N2 are fresh and synchronized.
- The required stillness window completes without reset.
- Per-node orientation spread is at most 3 degrees.
- Per-node rotation from neutral is between 60 and 120 degrees.
- N4 and N2 rotation magnitudes differ by no more than 15 degrees, indicating a straight elbow and a rigid whole-arm pose.
- The observed side and forward axes are separated by 60–120 degrees after sign normalization.
- The reconstructed mount matrix passes the determinant and orthogonality checks.

Rejection keeps the previous completed calibration stage and explains whether the problem was movement, bent elbow, insufficient raise, excessive raise, unsynchronized data, or non-independent poses. It never installs a partial or guessed correction.

`Clear Calibration` clears neutral, hinge, anatomical captures, and all mount corrections together. Starting a new neutral calibration also invalidates every later stage.

## 7. Motion packet and avatar mapping

The public motion packet shape remains compatible. Its existing orientation fields change frame only after successful anatomical calibration:

- `upper_arm_orientation`: corrected N4 anatomical delta.
- `forearm_orientation`: corrected N2 anatomical delta.
- `diagnostics.axisFrame`: `"anatomical"` only when both required corrections are installed; otherwise `"sensor_neutral"`.
- Add anatomical calibration state, quality, and rejection reason under `diagnostics`.

The avatar computes the nested forearm transform as:

```text
q_forearm_relative = conjugate(q_upper_anatomical) * q_forearm_anatomical
```

It applies:

- `q_upper_anatomical` to the upper-arm object at the shoulder pivot.
- `q_forearm_relative` to the forearm object at the elbow pivot.
- The hand as a child of the forearm.

Before anatomical calibration, the avatar must not pretend to provide directional tracking. It shows a clear `Anatomical calibration required` state. Numeric angle diagnostics can continue because their magnitude remains valid in the sensor-neutral frame.

The existing hinge-calibrated scalar flexion remains available to Coach Assist analysis and as a diagnostic comparison. It is no longer the sole visual forearm transform once anatomical calibration succeeds.

## 8. Low-latency display policy

The 25 Hz sensor and motion contracts remain unchanged. The avatar continues painting on `requestAnimationFrame`, but the previous heavy smoothing policy is replaced:

- Quaternion hemisphere continuity is mandatory.
- Use a short adaptive interpolation time constant: approximately 15 ms during deliberate movement and up to 60 ms only for sub-degree stationary jitter.
- Remove the 540 degrees/second rate cap; it created catch-up delay after large changes.
- Hold the last valid pose for at most 120 ms during a missing update.
- After 120 ms, freeze and display a degraded-data indicator. Do not extrapolate an unmeasured pose for the client demonstration.
- Blend from the held pose when data returns; never reset a joint to zero because synchronization briefly failed.

Target added visual latency is at most 40 ms median and 80 ms p95 when the browser tab is foregrounded. These are display-path targets, separate from BLE transport and packet timestamps.

## 9. UI flow

The Motion Engine panel presents one ordered workflow:

1. `1. Calibrate Neutral`
2. `2. Capture Right-Side Raise`
3. `3. Capture Forward Raise`
4. `4. Calibrate Elbow Hinge`

Buttons are enabled only when their prerequisites are complete. The panel displays:

- current stage;
- stillness progress;
- measured pose angle for N4 and N2;
- separation between side and forward axes;
- final anatomical calibration quality;
- whether the avatar is using `sensor-neutral` or `anatomical` axes.

Calibration events and accepted axes/mount quaternions are added to the NDJSON event stream so a physical test can be audited. Raw sensor and motion streams remain unchanged.

## 10. Implementation boundaries

Expected implementation areas:

- `host/live_tool/js/anatomical-calibration.js` — pure TRIAD math, validation, and capture result types.
- `host/live_tool/js/motion-engine.js` — capture state, stillness accumulation, mount installation, diagnostics, and reset behavior.
- `host/live_tool/js/arm-avatar.js` — corrected upper and relative-forearm quaternion smoothing/rendering.
- `host/live_tool/js/main.js` and `host/live_tool/index.html` — ordered controls, status, and event logging.
- Host tests for calibration math, state transitions, avatar hierarchy, and latency/dropout behavior.

Explicitly out of scope:

- firmware, BLE rate, MTU, DLE, or PHY changes;
- model retraining or classifier changes;
- automatic exercise recognition;
- torso-relative tracking;
- left-arm mirroring;
- persistent mount calibration across sessions;
- wrist articulation without a hand-mounted sensor;
- full-body avatar work.

## 11. Verification

### 11.1 Automated tests

- Recover known anatomical rotations from synthetic sensor data using several deliberately awkward mount rotations.
- Confirm side raise maps the neutral arm direction toward anatomical `+X`.
- Confirm forward raise maps toward anatomical `+Z`.
- Confirm arbitrary combined upper-arm rotations survive the correction.
- Recover the relative forearm transform from independently mounted N4 and N2 sensors.
- Reject insufficient, excessive, moving, bent-elbow, unsynchronized, collinear, and non-finite captures.
- Confirm clearing or replacing neutral calibration invalidates mount correction.
- Confirm a short dropout holds the last pose and a longer dropout changes health without zeroing the pose.
- Confirm display filtering meets the latency targets on deterministic fixtures.
- Run the complete host Python and JavaScript test suites.

### 11.2 Browser test

- Confirm the current build string and no stale-build banner.
- Complete all four calibration steps using live hardware.
- Confirm status reports `axis frame: anatomical`.
- Observe side, forward, overhead, diagonal, circular, and elbow movements.
- Confirm the shoulder and elbow pivots remain connected and no transform flips occur.

### 11.3 Client-video acceptance test

Record the wearer and screen together after a fresh calibration. Perform slowly first, then at normal speed:

1. neutral hold;
2. right-side raise to approximately 90 degrees and return;
3. forward raise to approximately 90 degrees and return;
4. elbow curl with upper arm still;
5. side raise with elbow flexion;
6. one slow circular shoulder movement.

Acceptance criteria:

- Neutral display direction is within 5 degrees of the expected down direction.
- Side and forward display directions are each within 10 degrees of the instructed plane at the held pose.
- Display elevation is within 10 degrees of the measured segment rotation magnitude.
- No zero-angle snap occurs during an isolated missed/unsynchronized frame.
- Added display latency is at most 40 ms median and 80 ms p95 in recorded timestamp diagnostics.
- Any failed criterion is reported as a remaining prototype limitation; it is not hidden by recording selection or stronger smoothing.

## 12. Remaining limitations

Three-pose calibration removes constant sensor-mount rotation but cannot create information the current rig does not measure. Without a torso node, upper-arm direction remains relative to the calibration pose rather than to a moving trunk. The magnetometer-free Game Rotation Vector can still accumulate yaw drift. The hand follows the forearm because no hand sensor exists. These limits must remain visible in the UI and client explanation.
