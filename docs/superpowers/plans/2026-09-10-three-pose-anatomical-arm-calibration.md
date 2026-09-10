# Three-Pose Anatomical Arm Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Calibrate N4 and N2 into a right-arm anatomical coordinate frame and render low-latency upper-arm and forearm orientations that reproduce side, forward, combined, and elbow movements.

**Architecture:** A pure TRIAD solver converts two observed, non-collinear sensor-neutral rotation axes into per-node mount corrections. `MotionEngine` owns the guided capture state and installs those corrections; `ArmAvatar` consumes corrected upper and forearm quaternions and renders their nested relative transform on `requestAnimationFrame` with only light adaptive filtering.

**Tech Stack:** Browser ES modules, JavaScript quaternion/vector math, CSS 3D transforms, Node.js test runners, Python `unittest`/pytest fixture harness.

**Spec:** `docs/superpowers/specs/2026-09-10-three-pose-anatomical-arm-calibration-design.md`

## Global Constraints

- Right arm only: `+X` wearer-right, `+Y` down the neutral arm, `+Z` forward.
- Directional targets are side axis `[0, 0, -1]` and forward axis `[1, 0, 0]`.
- N4 is upper arm, N2 is forearm, and N3 remains auxiliary.
- Directional captures are session-only and invalidated by a new neutral calibration.
- Keep the 25 Hz / 40 ms motion, logging, and analysis contract unchanged.
- Do not change firmware, BLE framing, MTU/DLE/PHY, ESOX, ML preprocessing, model artifacts, or haptic policy.
- Do not claim torso-relative tracking, left-arm support, wrist articulation, or drift correction.
- Use TDD for every behavior change and run the failing test before production edits.

---

### Task 1: Pure Anatomical TRIAD Solver

**Files:**
- Create: `host/live_tool/js/anatomical-calibration.js`
- Create: `host/tests/scripts/test_anatomical_calibration.mjs`

**Interfaces:**
- Consumes: source side/forward axes as finite three-number arrays.
- Produces: `solveMountCorrection(sourceSide, sourceForward, options?)` returning `{ ok, mount, quality, reason }`; successful `mount` is an internal `[w, x, y, z]` quaternion.
- Produces: `rotateVectorByQuaternion(q, vector)` for independent solver and integration assertions.

- [ ] **Step 1: Write failing solver tests**

Cover identity, deliberately awkward known mount rotations, correct right/forward target directions, collinear axes, non-finite values, and determinant/orthogonality quality. Derive expected target vectors as literals.

```js
const solved = solveMountCorrection(sourceSide, sourceForward);
assert.equal(solved.ok, true);
const map = [solved.mount[0], -solved.mount[1], -solved.mount[2], -solved.mount[3]];
assertVectorClose(rotateVectorByQuaternion(map, sourceSide), [0, 0, -1]);
assertVectorClose(rotateVectorByQuaternion(map, sourceForward), [1, 0, 0]);
assert.equal(solveMountCorrection([0, 1, 0], [0, 2, 0]).ok, false);
```

- [ ] **Step 2: Run the solver test and verify red**

Run: `node host/tests/scripts/test_anatomical_calibration.mjs`

Expected: FAIL because `anatomical-calibration.js` or `solveMountCorrection` does not exist.

- [ ] **Step 3: Implement the minimal pure solver**

Implement normalization, dot/cross products, Gram-Schmidt TRIAD construction, `R_map = T * transpose(S)`, rotation-matrix-to-quaternion conversion, `mount = conjugate(Q_map)`, and quality checks. Return explicit reasons: `non_finite_axis`, `zero_axis`, `axes_not_independent`, or `invalid_rotation`.

```js
export function solveMountCorrection(sourceSide, sourceForward, {
  targetSide = [0, 0, -1],
  targetForward = [1, 0, 0],
  minSeparationDeg = 60,
  maxSeparationDeg = 120,
} = {}) { /* TRIAD solution */ }
```

- [ ] **Step 4: Run the solver test and verify green**

Run: `node host/tests/scripts/test_anatomical_calibration.mjs`

Expected: PASS with every synthetic mount mapping both observed axes to the literal anatomical targets.

- [ ] **Step 5: Commit the solver**

```powershell
git add host/live_tool/js/anatomical-calibration.js host/tests/scripts/test_anatomical_calibration.mjs
git commit -m "feat(motion): add anatomical mount solver"
```

### Task 2: Guided Directional Capture in MotionEngine

**Files:**
- Modify: `host/live_tool/js/motion-engine.js`
- Modify: `host/tests/scripts/run_motion_fixture.mjs`
- Modify: `host/tests/python/test_motion_engine.py`

**Interfaces:**
- Consumes: `solveMountCorrection()` from Task 1.
- Produces: `beginSideCalibration(nowMs)`, `beginForwardCalibration(nowMs)`, `updateAnatomicalCalibration(nowMs)`.
- Produces diagnostics: `anatomicalState`, `anatomicalMessage`, `anatomicalQuality`, and existing `axisFrame`.
- Emits events: `anatomical_side_started`, `anatomical_side_complete`, `anatomical_forward_started`, `anatomical_complete`, and `anatomical_failed`.

- [ ] **Step 1: Extend the fixture vocabulary and write failing integration tests**

Add fixture operations `beginSide` and `beginForward`. Generate sensor reports from known anatomical segment poses composed with independent awkward N4 and N2 mounts. Require side and forward corrected vectors, arbitrary combined rotations, and relative forearm recovery.

```python
builder.begin_side()
builder.hold_pose(side_upper, side_forearm, duration_ms=1100)
builder.begin_forward()
builder.hold_pose(forward_upper, forward_forearm, duration_ms=1100)
builder.frame("anatomical")
self.assertEqual(frame["diagnostics"]["axisFrame"], "anatomical")
```

Add separate tests for moving capture, 45-degree under-raise, bent-elbow magnitude mismatch, collinear side/forward axes, synchronization loss, and neutral recalibration clearing installed corrections.

- [ ] **Step 2: Run focused MotionEngine tests and verify red**

Run: `py -3 -m pytest host/tests/python/test_motion_engine.py -v`

Expected: FAIL because the fixture runner and MotionEngine do not recognize directional calibration operations.

- [ ] **Step 3: Implement the directional capture state machine**

Add `ANATOMICAL_STATE` values `none`, `side_capturing`, `side_ready`, `forward_capturing`, `calibrated`, and `failed`. Capture one second of still N4/N2 neutral-relative quaternions, average them sign-safely, and validate:

```js
60 <= rotationAngleDeg && rotationAngleDeg <= 120
Math.abs(n4AngleDeg - n2AngleDeg) <= 15
spreadDeg <= 3
requiredFresh && synchronized
```

After the forward capture, solve and atomically install corrections for both N4 and N2 only when both solutions succeed. A failure leaves `mountCorrection` empty and preserves the last completed stage.

- [ ] **Step 4: Add corrected diagnostics and reset semantics**

Set `axisFrame` to `"anatomical"` only when both required node corrections exist. Include capture angles, axis separation, solver quality, state, and message. Make `beginCalibration()` and `clearCalibration()` clear side/forward captures, hinge calibration, and mount corrections.

- [ ] **Step 5: Run focused tests and verify green**

Run: `py -3 -m pytest host/tests/python/test_motion_engine.py -v`

Expected: PASS, including existing mount-independent angle and hinge tests.

- [ ] **Step 6: Commit MotionEngine calibration**

```powershell
git add host/live_tool/js/motion-engine.js host/tests/scripts/run_motion_fixture.mjs host/tests/python/test_motion_engine.py
git commit -m "feat(motion): calibrate arm axes anatomically"
```

### Task 3: Full Upper-Arm and Forearm Avatar Kinematics

**Files:**
- Modify: `host/live_tool/js/arm-avatar.js`
- Modify: `host/live_tool/index.html`
- Modify: `host/tests/scripts/test_arm_avatar.mjs`

**Interfaces:**
- Consumes: corrected `frame.upper_arm_orientation`, `frame.forearm_orientation`, and `frame.diagnostics.axisFrame`.
- Produces: smoothed `{ shoulder, forearmRelative, state }` and CSS variables `--upper-matrix`, `--forearm-matrix`.

- [ ] **Step 1: Write failing nested-kinematics tests**

Require `forearmRelative = conjugate(upper) * forearm`, identity at a rigid straight-arm raise, a 90-degree elbow transform independent of upper-arm direction, shortest-hemisphere interpolation, and no directional rendering while `axisFrame !== "anatomical"`.

```js
const pose = armPoseForMotion(anatomicalFrame);
assertQuaternionClose(pose.forearmRelative, qAxisAngle([0, 0, -1], 90));
assert.equal(armPoseForMotion(sensorNeutralFrame).state, "anatomical_calibration_required");
```

- [ ] **Step 2: Run avatar tests and verify red**

Run: `node host/tests/scripts/test_arm_avatar.mjs`

Expected: FAIL because the avatar exposes only shoulder quaternion plus scalar elbow flexion.

- [ ] **Step 3: Implement relative-forearm rendering**

Compute the nested forearm quaternion from corrected segment quaternions. Extend `ArmPoseSmoother` to maintain upper and relative-forearm targets independently. Apply `--forearm-matrix` at the elbow pivot and let the existing hand remain a child of that forearm transform.

Replace the scalar-only transform:

```css
.arm-forearm {
  transform: var(--forearm-matrix) translateZ(4px);
}
```

Retain scalar elbow flexion only in numeric diagnostics and rep analysis.

- [ ] **Step 4: Retune display filtering and dropout behavior**

Remove the 540 degrees/second cap. Use a time-aware quaternion interpolation constant near 15 ms for movement and up to 60 ms for sub-degree jitter. Reduce the valid-pose hold to 120 ms. After that threshold, retain the last transform but expose `tracking_unavailable`; never write identity or zero as a dropout response.

- [ ] **Step 5: Run avatar tests and verify green**

Run: `node host/tests/scripts/test_arm_avatar.mjs`

Expected: PASS for anatomical gating, upper/forearm hierarchy, latency, quaternion sign continuity, and dropout hold.

- [ ] **Step 6: Commit avatar kinematics**

```powershell
git add host/live_tool/js/arm-avatar.js host/live_tool/index.html host/tests/scripts/test_arm_avatar.mjs
git commit -m "feat(live-tool): render anatomical arm segments"
```

### Task 4: Ordered Calibration UI and Audit Events

**Files:**
- Modify: `host/live_tool/index.html`
- Modify: `host/live_tool/js/ui.js`
- Modify: `host/live_tool/js/main.js`
- Modify: `host/live_tool/js/session-log.js` only if event serialization cannot carry nested quality fields unchanged
- Modify: `host/tests/python/test_live_tool_invariants.py`

**Interfaces:**
- Consumes: MotionEngine methods and diagnostics from Task 2.
- Produces controls `calibrateBtn`, `sideCalibrateBtn`, `forwardCalibrateBtn`, `hingeBtn`; renders sequential enablement and calibration quality.

- [ ] **Step 1: Write failing UI/invariant tests**

Require all four control IDs, their ordered labels, main.js event bindings, `axisFrame` display, and a build-string change. Assert behavior-bearing integration strings and method connections, not cosmetic prose.

- [ ] **Step 2: Run invariant tests and verify red**

Run: `py -3 -m pytest host/tests/python/test_live_tool_invariants.py -v`

Expected: FAIL because side/forward controls and bindings do not exist.

- [ ] **Step 3: Implement the ordered workflow**

Add:

```html
<button id="calibrateBtn">1. Calibrate Neutral</button>
<button id="sideCalibrateBtn">2. Capture Right-Side Raise</button>
<button id="forwardCalibrateBtn">3. Capture Forward Raise</button>
<button id="hingeBtn">4. Calibrate Elbow Hinge</button>
```

Bind the new controls in `Ui` and `App`. Enable each only after its prerequisite. Display active instruction, N4/N2 capture angles, side-forward separation, quality, failure reason, and `axis frame: anatomical|sensor-neutral`.

- [ ] **Step 4: Preserve NDJSON auditability**

Continue routing MotionEngine events through `onMotionEvent()`. Verify accepted axes, mount quaternions, capture angles, and solver quality are serialized in `motion_anatomical_*` events without modifying raw sample streams or `MotionEngine.LOG_COLUMNS`.

- [ ] **Step 5: Bump and expose the live build**

Increment `LIVE_TOOL_BUILD` in `host/live_tool/js/live-inference.js`. Update the avatar note to say directional tracking is available only after the three-pose workflow.

- [ ] **Step 6: Run invariant and component tests**

Run:

```powershell
py -3 -m pytest host/tests/python/test_live_tool_invariants.py host/tests/python/test_motion_engine.py -v
node host/tests/scripts/test_anatomical_calibration.mjs
node host/tests/scripts/test_arm_avatar.mjs
```

Expected: all pass.

- [ ] **Step 7: Commit the UI workflow**

```powershell
git add host/live_tool/index.html host/live_tool/js/ui.js host/live_tool/js/main.js host/live_tool/js/live-inference.js host/tests/python/test_live_tool_invariants.py
git commit -m "feat(live-tool): guide anatomical arm calibration"
```

### Task 5: Replay, Browser, and Physical Handoff Verification

**Files:**
- Modify: `docs/architecture/motion-engine-contract.md`
- Create: `host/tests/scripts/replay_arm_avatar.mjs`
- Modify: `host/tests/python/test_live_tool_invariants.py`

**Interfaces:**
- Consumes: an NDJSON path and the same smoothing/kinematics functions used by the browser.
- Produces: JSON metrics for cadence, invalid frames, visual-step distribution, added display lag, axis frame, and calibrated direction errors when calibration events are available.

- [ ] **Step 1: Write the replay CLI test before its implementation**

Use a temporary deterministic NDJSON fixture containing neutral, side, forward, combined, dropout, and recovery frames. Invoke the real CLI and assert literal direction/latency bounds from its JSON output.

```python
completed = subprocess.run(
    ["node", str(REPLAY_RUNNER), str(fixture_path)],
    check=True, capture_output=True, text=True,
)
metrics = json.loads(completed.stdout)
self.assertEqual(metrics["axisFrame"], "anatomical")
self.assertLessEqual(metrics["sideDirectionErrorDeg"], 10.0)
self.assertLessEqual(metrics["medianAddedLatencyMs"], 40.0)
```

- [ ] **Step 2: Run the replay test and verify red**

Run: `py -3 -m pytest host/tests/python/test_live_tool_invariants.py -v`

Expected: FAIL because `replay_arm_avatar.mjs` is missing.

- [ ] **Step 3: Implement the replay CLI**

The CLI must import production calibration/avatar functions, replay at 60 Hz, and report rather than silently repair missing calibration information. A legacy log without directional calibration events must report `axisFrame: sensor_neutral` and `directionValidation: unavailable`.

- [ ] **Step 4: Document the final contract and procedure**

Update the contract with the three-pose state machine, axis convention, validation thresholds, forearm-relative formula, latency policy, and the exact six-movement client-video procedure from the approved spec.

- [ ] **Step 5: Run complete verification**

Run:

```powershell
node host/tests/scripts/test_anatomical_calibration.mjs
node host/tests/scripts/test_arm_avatar.mjs
py -3 -m pytest host/tests/python -q
node --check host/live_tool/js/anatomical-calibration.js
node --check host/live_tool/js/arm-avatar.js
node --check host/live_tool/js/motion-engine.js
node --check host/live_tool/js/main.js
git diff --check
```

Expected: zero failures and zero syntax/diff errors.

- [ ] **Step 6: Browser smoke test**

Serve with `py -3 host/live_tool/serve.py`, hard-refresh `http://localhost:8080/`, confirm the new build string, inspect the four-step workflow, and confirm no browser console errors. Do not claim physical success from this test.

- [ ] **Step 7: Commit replay and documentation**

```powershell
git add host/tests/scripts/replay_arm_avatar.mjs docs/architecture/motion-engine-contract.md
git commit -m "test(motion): add anatomical avatar replay audit"
```

- [ ] **Step 8: Hand off the physical acceptance run**

Ask the user to perform a fresh neutral/side/forward/hinge calibration and record the six prescribed movements. Review the new NDJSON and synchronized video before claiming client-demo readiness.
