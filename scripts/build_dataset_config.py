#!/usr/bin/env python3
"""Regenerate dataset/dataset_config.json from the folder tree.

The training notebook (host/notebooks/Vantare_Bicep_Curl_Training_ONNX_v1_1.ipynb)
reads dataset/dataset_config.json: the session list plus validation folds. Hand-
maintaining it does not scale past the original six sessions, so this script
rebuilds it from what is on disk:

    dataset/<class_name>/<session_folder>/
        R<NNNN>N2_BNO85.csv   R<NNNN>N2_ICM45686.csv   R<NNNN>N2_metadata.json
        R<NNNN>N3_*           R<NNNN>N4_*
        session.json                      (optional, per-session metadata)

<class_name> is one of correct / incomplete_range / elbow_movement and sets the
class id (index into that tuple). The session number NNNN is read from the file
names (it is what the notebook keys on). session.json may carry:

    {"wearer_id": "W1", "mount_attempt": 3, "date": "2026-09-08",
     "held_out": false, "notes": "seated, moderate tempo"}

Folds are generated, not hand-listed:
  * validation_folds  -- grouped k-fold by session over the non-held-out
                         sessions (this is the cross-val estimate cell 17 uses).
  * held_out_sessions -- sessions flagged held_out:true; the notebook trains on
                         everything else and reports on these as the honest
                         "fresh mount / fresh day" number.
  * lowo_folds        -- leave-one-wearer-out, one fold per wearer, when two or
                         more wearers are present among the non-held-out sessions.

Usage:
    python scripts/build_dataset_config.py                 # write dataset/dataset_config.json
    python scripts/build_dataset_config.py --check         # exit 1 if out of date
    python scripts/build_dataset_config.py --folds 5       # k for the grouped k-fold
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

CLASS_NAMES = ("correct", "incomplete_range", "elbow_movement")
NODES = (2, 3, 4)
SENSORS = ("BNO85", "ICM45686")
STEM_RE = re.compile(r"^R(\d{3,5})N([234])_BNO85\.csv$")

REPO_ROOT = Path(__file__).resolve().parents[1]
DATASET_ROOT = REPO_ROOT / "dataset"
CONFIG_PATH = DATASET_ROOT / "dataset_config.json"


def discover_sessions(dataset_root: Path) -> list[dict]:
    sessions: list[dict] = []
    for class_id, class_name in enumerate(CLASS_NAMES):
        class_dir = dataset_root / class_name
        if not class_dir.is_dir():
            continue
        for session_dir in sorted(p for p in class_dir.iterdir() if p.is_dir()):
            stems = sorted(
                STEM_RE.match(p.name).group(1)
                for p in session_dir.iterdir()
                if STEM_RE.match(p.name)
            )
            numbers = sorted({int(s) for s in stems})
            if len(numbers) != 1:
                raise SystemExit(
                    f"{session_dir}: expected exactly one R<NNNN> session number, found {numbers or 'none'}"
                )
            number = numbers[0]
            missing = [
                f"R{number:04d}N{node}_{sensor}.csv"
                for node in NODES
                for sensor in SENSORS
                if not (session_dir / f"R{number:04d}N{node}_{sensor}.csv").is_file()
            ] + [
                f"R{number:04d}N{node}_metadata.json"
                for node in NODES
                if not (session_dir / f"R{number:04d}N{node}_metadata.json").is_file()
            ]
            if missing:
                raise SystemExit(f"{session_dir}: missing required files: {', '.join(missing)}")

            meta_path = session_dir / "session.json"
            meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}

            sessions.append(
                {
                    "session": number,
                    "session_id": f"S{number:04d}",
                    "folder": f"{class_name}/{session_dir.name}",
                    "class_id": class_id,
                    "label": class_name,
                    "wearer_id": str(meta.get("wearer_id", "W0")),
                    "mount_attempt": int(meta.get("mount_attempt", 0)),
                    "date": str(meta.get("date", "")),
                    "held_out": bool(meta.get("held_out", False)),
                    "video_file": str(meta["video_file"]) if meta.get("video_file") else None,
                    "notes": str(meta.get("notes", "")),
                }
            )

    if not sessions:
        raise SystemExit(f"no sessions found under {dataset_root}/<class>/<session>/")
    seen = {}
    for s in sessions:
        if s["session"] in seen:
            raise SystemExit(
                f"session number {s['session']} used twice: {seen[s['session']]} and {s['folder']}"
            )
        seen[s["session"]] = s["folder"]
    return sorted(sessions, key=lambda s: s["session"])


def grouped_kfold(sessions: list[dict], k: int) -> list[dict]:
    """Assign whole sessions to k folds, round-robin within each class so every
    fold has a spread of classes. Returns notebook-shaped fold dicts."""
    train_pool = [s for s in sessions if not s["held_out"]]
    per_class_min = min(
        (sum(1 for s in train_pool if s["class_id"] == c) for c in range(len(CLASS_NAMES))),
        default=0,
    )
    k = max(2, min(k, per_class_min)) if per_class_min >= 2 else 1
    if k < 2:
        return []
    fold_of: dict[int, int] = {}
    for c in range(len(CLASS_NAMES)):
        members = [s["session"] for s in train_pool if s["class_id"] == c]
        for i, num in enumerate(members):
            fold_of[num] = i % k
    folds = []
    for f in range(k):
        test = sorted(n for n, fi in fold_of.items() if fi == f)
        train = sorted(n for n in fold_of if fold_of[n] != f)
        if test and train:
            folds.append(
                {"name": chr(ord("A") + f), "train_sessions": train, "test_sessions": test}
            )
    return folds


def lowo_folds(sessions: list[dict]) -> list[dict]:
    train_pool = [s for s in sessions if not s["held_out"]]
    wearers = sorted({s["wearer_id"] for s in train_pool})
    if len(wearers) < 2:
        return []
    out = []
    for w in wearers:
        test = sorted(s["session"] for s in train_pool if s["wearer_id"] == w)
        train = sorted(s["session"] for s in train_pool if s["wearer_id"] != w)
        classes_in_test = {s["class_id"] for s in train_pool if s["wearer_id"] == w}
        if train and len(classes_in_test) == len(CLASS_NAMES):
            out.append({"wearer": w, "train_sessions": train, "test_sessions": test})
    return out


def build_config(sessions: list[dict], k: int) -> dict:
    wearers = sorted({s["wearer_id"] for s in sessions})
    return {
        "dataset_name": "vantare_bicep_curl_v2",
        "dataset_version": 2,
        "phase": 0,
        "task": "bicep_curl_form_classification",
        "generated_by": "scripts/build_dataset_config.py",
        "included_nodes": list(NODES),
        "excluded_sources": ["MASTER"],
        "node_placements": {
            "2": "wrist_distal_forearm",
            "3": "elbow_region",
            "4": "upper_arm_near_shoulder",
        },
        "sensors_per_node": list(SENSORS),
        "class_mapping": {str(i): name for i, name in enumerate(CLASS_NAMES)},
        "wearers": wearers,
        "sessions": sessions,
        "validation_folds": grouped_kfold(sessions, k),
        "held_out_sessions": sorted(s["session"] for s in sessions if s["held_out"]),
        "lowo_folds": lowo_folds(sessions),
        "expected_files_per_session": 9,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--folds", type=int, default=2, help="k for the grouped k-fold (default 2)")
    parser.add_argument("--check", action="store_true", help="exit 1 if the on-disk config is stale")
    args = parser.parse_args(argv)

    sessions = discover_sessions(DATASET_ROOT)
    config = build_config(sessions, args.folds)
    rendered = json.dumps(config, indent=2) + "\n"

    if args.check:
        current = CONFIG_PATH.read_text(encoding="utf-8") if CONFIG_PATH.is_file() else ""
        if current != rendered:
            print(f"{CONFIG_PATH} is out of date; run scripts/build_dataset_config.py", file=sys.stderr)
            return 1
        print(f"{CONFIG_PATH} is up to date ({len(sessions)} sessions).")
        return 0

    CONFIG_PATH.write_text(rendered, encoding="utf-8")
    by_class = {name: sum(1 for s in sessions if s["label"] == name) for name in CLASS_NAMES}
    held = sum(1 for s in sessions if s["held_out"])
    print(f"wrote {CONFIG_PATH}")
    print(f"  sessions: {len(sessions)}  ({', '.join(f'{n}={c}' for n, c in by_class.items())})")
    print(f"  wearers: {config['wearers']}")
    print(f"  held-out sessions: {config['held_out_sessions'] or 'none'}")
    print(f"  grouped folds: {[f['name'] for f in config['validation_folds']] or 'none (need >=2 sessions/class)'}")
    print(f"  LOWO folds: {[f['wearer'] for f in config['lowo_folds']] or 'none (need >=2 wearers)'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
