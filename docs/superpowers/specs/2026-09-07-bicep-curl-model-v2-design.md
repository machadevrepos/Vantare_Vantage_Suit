# Bicep-Curl Model V2 — Design

Date: 2026-09-07
Status: Draft. Phase 0 (proof of integration) starts now; Phase 1+ pending the
Phase 0 result.

**Read first.** The V1_1 model works as an *integration* proof — the 2026-09-07
live run showed transport, preprocessing, ONNX inference, and the haptic gate all
running end to end (241 distinct probability vectors over 296 predictions). But
its predictions are not usable: confidence rarely clears the 0.70 haptic
threshold and `correct` vs `incomplete_range` do not separate. Two root causes,
both fixable:

1. **Session identity == class identity.** Six sessions, two per class. The
   RandomForest learned "which recording is this", not "what movement is this".
   Any unseen session (i.e. every live session) lands in a no-man's-land.
2. **Mounting-dependent features.** Of the 72 synchronized channels, the raw
   per-node quaternions and gravity vectors encode how the straps sat on the
   arm. Only the three inter-node relative angles are mount-invariant. A new
   strap-up moves the whole feature distribution.

This document defines the target model (V2) and a cheap Phase 0 that de-risks the
whole effort before the expensive tooling is built.

---

## 1. Target output shape (V2)

Form coaching needs more than one softmax label per 2-second window. Real form
breaks down as several faults at once, a fixed window straddles rep boundaries,
and "incomplete_range" throws away *how* incomplete.

V2 is **rep-segmented, multi-output**:

| Output | Type | Use |
|---|---|---|
| `rep_count` | running integer | displayed; the unit of feedback |
| `rom_fraction` | regression 0–1 per rep | spoken/visual score; "82% range" |
| `elbow_drift` | binary per rep | distinct haptic pattern A |
| `torso_swing` | binary per rep | distinct haptic pattern B |
| `no_eccentric_control` | binary per rep | distinct haptic pattern C |
| `wrist_curl` | binary per rep | distinct haptic pattern D |
| `rep_valid` | binary per rep | gates everything; rejects fidgets/adjustments |

Start with `rom_fraction` + `elbow_drift` + one more fault; add the rest once the
pipeline holds. A per-rep form score for the spoken/visual readout is derived
from `rom_fraction` and the fault flags.

Single arm only for now (3 nodes: N2 wrist / N3 elbow / N4 upper arm). A
bilateral build (N1 + a second set) is out of scope until more PCBs exist.

## 2. Rep segmentation

A bicep-curl rep is a monotonic flexion then extension of the elbow, ~2–4 s,
with a clean signature in the **N3↔N2 relative angle** (the existing
`relative_angle_n2_n3_deg` channel). Segmentation:

- Track the relative angle; a rep is a local-minimum → local-maximum → next
  local-minimum cycle (extended arm → contracted → extended), with a minimum
  angular excursion and a minimum/maximum duration to reject twitches and rests.
- `rom_fraction` = observed peak-to-peak excursion / a per-wearer calibration
  excursion captured at session start (three deliberate full reps).
- Runs in `ml-preprocessing.js` (live) and the offline pipeline, from the same
  code, under the parity test.

Features are then computed **per rep** (the 8 statistics over each channel across
the rep's samples, plus rep duration, plus phase-split stats: concentric half vs
eccentric half). This replaces the fixed 50-sample / stride-12 windowing for the
fault heads. `rep_count` and `rom_fraction` fall directly out of segmentation.

## 3. Ground truth — video annotation

"Distinct haptic pattern per fault" means the model will tell a wearer "your
elbow is drifting". A wrong label there is worse than no label, which rules out:

- **Session-level labels** (folder = class): re-creates root cause 1 exactly —
  the label is perfectly correlated with the recording.
- **Real-time keypress marking**: ~300–500 ms reaction lag, misses subtle
  faults, needs an observer every session, inter-observer drift.

V2 ground truth is **per-rep video annotation**:

1. Film every session — one phone on a tripod, side-on (ROM, torso swing,
   elbow-forward). A front angle is added later for elbow flare if needed.
2. **Sync**: three firm taps on the N2 (wrist) sensor at session start, in
   frame. Sharp triple spike in N2 accel magnitude + visible/audible on video;
   align the first spike. Crystal drift over a 3-minute session is <0.1 %.
3. **Auto rep segmentation** (Section 2) so the annotator never hunts for reps.
4. **Annotation tool** — a local single-file HTML page (no deps, like the live
   tool): loads a session's CSVs + video, shows the synced signal with rep
   boundaries, one checkbox row of fault flags + a ROM number + valid/invalid
   per rep. Exports `labels.json` per session (~2–4 min per session).
5. **Labeling rubric** — a written spec (what counts as `elbow_drift`, how to
   eyeball `rom_fraction`, etc.) so 4+ wearers/annotators are consistent.

Recording protocol is still structured — mostly-clean sessions plus a few
fault-emphasis sessions for coverage — but **every rep is video-labeled
regardless of what the session was "for"**, so the labels stay honest when
execution wanders.

## 4. Data volume and wearers (V2)

- 4+ wearers.
- Per wearer: several mostly-clean sessions **with a real re-mount between every
  session**, plus fault-emphasis sessions. ~15–25 reps/session.
- Target ~60–100 sessions total, ~1500+ annotated reps, class-balanced enough
  that each fault head has ≥300 positive reps.
- Per-session metadata: `wearer_id`, `mount_attempt`, `date`, `video_file`,
  `notes`.

## 5. Validation

- **Grouped CV by session** (windows/reps from one session never span
  train/test) — this is already how V1 folds work; keep it.
- **Leave-one-wearer-out (LOWO)** as the headline metric — the honest estimate
  of live performance on a new person.
- Per-fault precision/recall/PR-AUC, not accuracy. A fault head that never fires
  scores 90 %+ accuracy and is useless.
- **Probability calibration** (isotonic, fit on held-out folds) so the haptic
  threshold means what it says.
- A **feature-leakage check**: train a classifier to predict `session_id` from
  the features; any feature with high session-discriminability *and* high fault
  importance is a memorization risk — drop or replace it.
- One genuinely fresh live session per iteration as the reality check.

## 6. Code changes (V2)

- **Firmware**: none. It still streams the six sensor lanes.
- **`ml-preprocessing.js`**: rep segmentation; per-rep features; keep the fixed
  window path only if a window model is still wanted alongside.
- **`live-inference.js` / model**: multi-output ONNX (or one small model per
  head); per-head thresholds.
- **`haptic-controller.js`**: distinct pulse patterns per fault head; rep-count
  and score readout.
- **Notebook**: multi-head training, grouped + LOWO CV, calibration, leakage
  check, the annotation `labels.json` as the training target.
- **`dataset_config.json`**: v2 schema (Section 8).
- **New**: annotation tool, rep segmenter (shared JS/Python), labeling rubric,
  session-QA script.

---

## 7. Phase 0 — Proof of live model integration (do this first)

**Goal.** Before building the annotation tool, rep segmenter, and multi-head
notebook, prove that fixing the two root causes with the *existing* single-class
pipeline produces a model that behaves correctly on live hardware — confident,
class-correct, and firing the haptic on a real fault. If it does, the V2
investment is justified. If it doesn't, we learn why cheaply.

**Keep from V1**: the 3-class softmax, the 50-sample / stride-12 window, the
RandomForest, the ONNX contract, the existing notebook and browser pipeline.
Nothing about the live tool or firmware changes.

**Change for Phase 0**:

1. **More sessions, disciplined re-mount.** ~8–10 sessions per class (up from 2),
   1–2 wearers for the pilot. **Take the rig off and re-strap between every
   session** — this alone breaks most of root cause 1 even with session-level
   labels. ~2 min of continuous reps per session.
2. **Session-level labels are acceptable for Phase 0** — the wearer does a whole
   session of one class. Accept ~5–10 % label noise; the point is integration,
   not final accuracy.
3. **Feature de-leaking.** Add a notebook cell that ranks the 72 channels by how
   well they predict `session_id`, and train a second model on a reduced set
   (drop raw quaternions and raw gravity; keep relative angles, linear-accel /
   gyro magnitudes, gravity-frame-projected accel if cheap). Compare LOWO / held-
   out-session metrics between full and reduced features.
4. **Held-out fresh session** — record one extra session per class on a
   different mount (ideally a different day / wearer) that is *never* in training,
   and report metrics on it. That number is the Phase 0 success criterion.
5. **Threshold tuning** — pick the haptic `probabilityThreshold` from the
   held-out PR curve rather than assuming 0.70.

**Phase 0 taxonomy.** Keep `correct` / `incomplete_range` / `elbow_movement` so
the contract and notebook are unchanged. Make `incomplete_range` unambiguous
(clear half-ROM reps, not marginal ones) so it is learnable from session labels.

**Phase 0 success criteria**:

- Held-out-fresh-session macro-F1 ≥ 0.75 (V1_1 has no comparable honest number).
- On a live run of deliberate reps: the predicted class matches the intended
  class ≥ 70 % of windows, and confidence clears the tuned threshold on the
  fault classes for ≥ 2 consecutive windows at least once per fault → haptic
  fires on N2 (`incomplete_range`) and N3 (`elbow_movement`).
- Feature de-leaking measurably improves the held-out number (evidence that
  root cause 2 is real and worth fixing properly in V2).

**Phase 0 deliverables**:

- [done] `dataset_config.json` v2 schema + `scripts/build_dataset_config.py`
  (regenerate from folders + per-session `session.json`; `--check` for CI).
- [done] `scripts/qa_session.py`: rate, gap, duration, rep-count sanity,
  outlier-vs-same-class distance. (The converter already does CRC.)
- [done] `dataset/README.md` -- the add-a-session workflow.
- [done] Phase 0 collection protocol (Section 9).
- [pending real data] Notebook changes:
  1. cell 7: `assert len(session_entries) == 6` -> `>= 6`;
     `assert len(folds) == 2` -> `>= 2`.
  2. cell 9: `assert len(profiles) == 36` -> `== 6 * len(session_entries)`.
  3. new cells after cell 21: feature-vs-`session_id` leakage rank; a
     reduced-feature model (drop raw quats + raw gravity); `held_out_sessions`
     eval; `lowo_folds` eval; haptic threshold from the held-out PR curve.
  These are done once ~6-10 new sessions exist, so they can be checked against
  real files in Colab.

---

## 8. `dataset_config.json` v2 schema

```json
{
  "dataset_name": "vantare_bicep_curl_v2",
  "dataset_version": 2,
  "task": "bicep_curl_form_classification",
  "phase": 0,
  "included_nodes": [2, 3, 4],
  "node_placements": { "2": "wrist_distal_forearm", "3": "elbow_region", "4": "upper_arm_near_shoulder" },
  "sensors_per_node": ["BNO85", "ICM45686"],
  "class_mapping": { "0": "correct", "1": "incomplete_range", "2": "elbow_movement" },
  "wearers": { "W1": {"notes": "..."}, "W2": {"notes": "..."} },
  "sessions": [
    {
      "session_id": "S007",
      "folder": "correct/S007",
      "class_id": 0, "label": "correct",
      "wearer_id": "W1",
      "mount_attempt": 3,
      "date": "2026-09-08",
      "held_out": false,
      "video_file": "correct/S007/video.mp4",
      "notes": "seated, moderate tempo"
    }
  ],
  "folds": { "strategy": "group_by_session_plus_lowo" },
  "expected_files_per_session": 9
}
```

`folds` is computed by the notebook from `session_id` / `wearer_id` /
`held_out`, not hand-listed.

## 9. Phase 0 collection protocol

Per wearer (1–2 for the pilot, then 4+ for V2):

1. Strap N2 (wrist, ~2 cm proximal to the styloid), N3 (over the lateral
   epicondyle), N4 (mid-upper-arm, lateral). Note `mount_attempt`.
2. Start recording. Do **3 deliberate full-ROM reps** (this is the ROM
   calibration reference and, later, the video sync if filming).
3. Do ~2 min of continuous reps of the session's class:
   - `correct`: full ROM, elbow pinned to the ribs, no torso movement, ~2 s up /
     ~2 s down.
   - `incomplete_range`: clearly partial — stop ~halfway, both directions.
   - `elbow_movement`: let the elbow drift forward/up each rep (front-delt
     cheat).
4. Stop. Convert (`vantage_bin_to_csv.py`), drop into `dataset/<class>/S0NN/`,
   run `scripts/qa_session.py`.
5. **Fully un-strap.** Re-strap for the next session — deliberately slightly
   different rotation/position. This is the point.
6. Repeat for ~8–10 sessions per class, cycling classes.
7. Record 1 extra session per class as `held_out: true` on a fresh mount /
   different day.

Total Phase 0: ~30 training + 3 held-out sessions, ~1–2 hours of wearer time
plus conversion.

## 10. Open questions

- Which faults for V2 beyond `elbow_drift` — needs the coaching goal firmed up.
- Front-camera angle: needed for `elbow_drift` labeling, or is side enough?
- Annotation labor: one trained annotator with the rubric, or crowd it across
  wearers? Affects rubric strictness.
- Does the Master's own IMU (currently `excluded_sources`) help detect
  `torso_swing` if the Master unit is torso-mounted? Revisit for V2.
