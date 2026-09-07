# Bicep-curl training dataset

Layout:

```
dataset/
  <class>/<session_folder>/
    R<NNNN>N2_BNO85.csv   R<NNNN>N2_ICM45686.csv   R<NNNN>N2_metadata.json
    R<NNNN>N3_*           R<NNNN>N4_*
    session.json                 (per-session metadata; optional but expected)
  dataset_config.json            (generated -- do not hand-edit)
  README.md
```

`<class>` is `correct`, `incomplete_range`, or `elbow_movement` (Phase 0 taxonomy;
see `docs/superpowers/specs/2026-09-07-bicep-curl-model-v2-design.md`). The class
folder sets the class id. `<NNNN>` is the SD run number from
`vantage_bin_to_csv.py`; it must be unique across the whole dataset.

## Adding a session

1. Record with the desktop tool, one class per session, ~2 min of continuous
   reps preceded by 3 deliberate full-ROM reps. **Re-strap the rig between every
   session** (that is what stops the model memorizing recordings).
2. Convert: `python host/desktop_tool/vantage_bin_to_csv.py <R<NNNN>N*.BIN> -o <tmp>`.
3. Move the 9 files into `dataset/<class>/session_NN/`.
4. Write `dataset/<class>/session_NN/session.json`:
   ```json
   {
     "wearer_id": "W1",
     "mount_attempt": 3,
     "date": "2026-09-08",
     "held_out": false,
     "video_file": null,
     "notes": "seated, moderate tempo"
   }
   ```
   Set `"held_out": true` for the fresh-mount / fresh-day sessions that must
   never appear in training.
5. QA it: `python scripts/qa_session.py dataset/<class>/session_NN --compare dataset/<class>`
   -- fix or discard anything that FAILs; investigate WARNings.
6. Regenerate the config: `python scripts/build_dataset_config.py`
7. Commit the CSVs, `session.json`, and `dataset_config.json` together.

CI (`scripts/build_dataset_config.py --check`) fails if `dataset_config.json` is
stale relative to the folder tree.

## Folds

`dataset_config.json` carries three, all generated from `session.json`:

- `validation_folds` -- grouped k-fold by session (windows from one session
  never span train/test). The notebook's cross-val number.
- `held_out_sessions` -- train on everything else, report on these. The honest
  "new mount / new day" number.
- `lowo_folds` -- leave-one-wearer-out, once there are >=2 wearers. The honest
  "new person" number, and the one that matters for a general model.
