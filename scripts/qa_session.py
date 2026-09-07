#!/usr/bin/env python3
"""Sanity-check a converted training session before it enters the dataset.

The offline converter (host/desktop_tool/vantage_bin_to_csv.py) already verifies
CRCs, timestamp monotonicity, and ICM sequence continuity. This adds the checks
that matter for *training data quality*: is the rate right, are there gaps big
enough to corrupt a 2 s window, does the recording actually contain reps, and is
this session an outlier versus other sessions of the same class (a sign of a
mislabeled folder or a bad mount).

Usage:
    python scripts/qa_session.py dataset/correct/session_07
    python scripts/qa_session.py dataset/correct/session_07 --compare dataset/correct
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path

NODES = (2, 3, 4)
SENSORS = ("BNO85", "ICM45686")
EXPECT_HZ = {"BNO85": 100.0, "ICM45686": 200.0}  # native recording rates
MIN_DURATION_S = 45.0
MAX_GAP_MS = 120.0          # a gap past 3 grid periods at 25 Hz corrupts windows
MIN_REPS = 8


def _session_number(session_dir: Path) -> int:
    for p in session_dir.iterdir():
        name = p.name
        if name.startswith("R") and name.endswith("N2_BNO85.csv"):
            return int(name[1 : name.index("N")])
    raise SystemExit(f"{session_dir}: no R<NNNN>N2_BNO85.csv found")


def _read_column(path: Path, column: str) -> tuple[list[float], list[float]]:
    ts: list[float] = []
    vals: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if "timestamp_us" not in (reader.fieldnames or []) or column not in (reader.fieldnames or []):
            raise SystemExit(f"{path.name}: missing timestamp_us or {column}")
        for row in reader:
            try:
                ts.append(float(row["timestamp_us"]) / 1e6)
                vals.append(float(row[column]))
            except (TypeError, ValueError):
                continue
    return ts, vals


def _smooth(x: list[float], w: int) -> list[float]:
    if w < 2 or len(x) < w:
        return x
    out = []
    acc = sum(x[:w])
    for i in range(len(x)):
        if i >= w:
            acc += x[i] - x[i - w]
        out.append(acc / min(i + 1, w))
    return out


def _count_reps(_ts: list[float], flexion_rate: list[float]) -> int:
    """Rep count from the elbow flexion-axis angular rate (N3 gyro_x): it swings
    positive on the way up and negative on the way down, ~one full cycle per
    rep. Count positive-going zero crossings of the smoothed signal with an
    amplitude gate. Catches 'no reps / wrong recording', not a precise count."""
    if len(flexion_rate) < 50:
        return 0
    s = _smooth(flexion_rate, 15)
    amp = statistics.pstdev(s)
    if amp < 0.15:  # rad/s; a real curl swings ~1-3 rad/s
        return 0
    gate = amp * 0.5
    reps = 0
    state = -1
    for v in s:
        if state <= 0 and v > gate:
            state = 1
            reps += 1
        elif state >= 0 and v < -gate:
            state = -1
    return reps


def qa_one(session_dir: Path) -> dict:
    number = _session_number(session_dir)
    report: dict = {"session": number, "folder": str(session_dir), "problems": [], "warnings": []}

    for node in NODES:
        for sensor in SENSORS:
            path = session_dir / f"R{number:04d}N{node}_{sensor}.csv"
            if not path.is_file():
                report["problems"].append(f"missing {path.name}")
                continue
            col = "linear_accel_x_mps2" if sensor == "BNO85" else "accel_x_g"
            ts, _ = _read_column(path, col)
            if len(ts) < 100:
                report["problems"].append(f"{path.name}: only {len(ts)} rows")
                continue
            duration = ts[-1] - ts[0]
            rate = (len(ts) - 1) / duration if duration > 0 else 0.0
            gaps_ms = [(ts[i] - ts[i - 1]) * 1000 for i in range(1, len(ts))]
            max_gap = max(gaps_ms)
            big_gaps = sum(1 for g in gaps_ms if g > MAX_GAP_MS)
            report[f"N{node}_{sensor}"] = {
                "rows": len(ts),
                "duration_s": round(duration, 1),
                "rate_hz": round(rate, 1),
                "max_gap_ms": round(max_gap, 1),
                "gaps_over_120ms": big_gaps,
            }
            if duration < MIN_DURATION_S:
                report["problems"].append(f"{path.name}: {duration:.0f}s < {MIN_DURATION_S:.0f}s minimum")
            expect = EXPECT_HZ[sensor]
            if rate < expect * 0.6:
                report["warnings"].append(f"{path.name}: {rate:.0f} Hz, expected ~{expect:.0f}")
            if max_gap > MAX_GAP_MS * 3:
                report["problems"].append(f"{path.name}: {max_gap:.0f} ms gap (windows spanning it are lost)")
            elif big_gaps > 0:
                report["warnings"].append(f"{path.name}: {big_gaps} gaps over {MAX_GAP_MS:.0f} ms")

    # Rough rep count: the N3 (elbow) forearm-flexion axis gyro alternates sign
    # once per rep. Crossings of its running integral about the midline, halved.
    n3 = session_dir / f"R{number:04d}N3_BNO85.csv"
    if n3.is_file():
        # Which body axis the elbow flexion lands on depends on strap rotation,
        # so take the highest-variance gyro axis rather than a fixed one.
        axes = {ax: _read_column(n3, ax)[1] for ax in ("gyro_x_radps", "gyro_y_radps", "gyro_z_radps")}
        flexion = max(axes.values(), key=lambda v: statistics.pstdev(v) if v else 0.0)
        ts, _ = _read_column(n3, "gyro_x_radps")
        reps = _count_reps(ts, flexion)
        report["reps_estimate"] = reps
        if reps < MIN_REPS:
            report["warnings"].append(
                f"elbow signal shows ~{reps} reps (< {MIN_REPS}); is this the right recording / class?"
            )

    return report


def compare_class(session_dir: Path, class_dir: Path) -> list[str]:
    """Flag a session whose per-channel means are far from its class siblings."""
    number = _session_number(session_dir)
    channels = [("N3_BNO85", "gyro_x_radps"), ("N2_BNO85", "linear_accel_x_mps2"), ("N4_BNO85", "gravity_z_mps2")]
    peers = [d for d in class_dir.iterdir() if d.is_dir() and d != session_dir]
    if len(peers) < 2:
        return []
    out: list[str] = []
    for tag, col in channels:
        node, sensor = tag.split("_")
        def series_mean(sd: Path) -> float | None:
            n = _session_number(sd)
            p = sd / f"R{n:04d}{node}_{sensor}.csv"
            if not p.is_file():
                return None
            _, v = _read_column(p, col)
            return statistics.fmean(v) if v else None
        this = series_mean(session_dir)
        peer_means = [m for m in (series_mean(d) for d in peers) if m is not None]
        if this is None or len(peer_means) < 2:
            continue
        mu = statistics.fmean(peer_means)
        sd = statistics.pstdev(peer_means) or 1e-6
        z = abs(this - mu) / sd
        if z > 3.0:
            out.append(f"{tag}.{col}: this session mean {this:.2f} is {z:.1f}sigma from class peers ({mu:.2f}+-{sd:.2f})")
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--compare", type=Path, help="class directory to compare against (e.g. dataset/correct)")
    parser.add_argument("--json", action="store_true", help="emit the report as JSON")
    args = parser.parse_args(argv)

    if not args.session_dir.is_dir():
        raise SystemExit(f"not a directory: {args.session_dir}")

    report = qa_one(args.session_dir)
    if args.compare:
        report["class_outliers"] = compare_class(args.session_dir, args.compare)
        report["warnings"].extend(report["class_outliers"])

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"session {report['session']}  ({report['folder']})")
        for key, val in report.items():
            if isinstance(val, dict):
                print(f"  {key}: {val}")
        if report.get("reps_estimate") is not None:
            print(f"  reps_estimate: {report['reps_estimate']}")
        for w in report["warnings"]:
            print(f"  WARN  {w}")
        for p in report["problems"]:
            print(f"  FAIL  {p}")
        if not report["problems"] and not report["warnings"]:
            print("  OK")

    return 1 if report["problems"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
