#!/usr/bin/env python3
"""Summarize Vantare session CSVs: rows, duration, sample rate, per-column stats.

Usage:
    python host/scripts/analyze_session.py <csv-or-directory> [...]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def iter_csvs(targets: list[Path]) -> list[Path]:
    files: list[Path] = []
    for target in targets:
        if target.is_dir():
            files.extend(sorted(target.rglob("*.csv")))
        elif target.suffix.lower() == ".csv":
            files.append(target)
        else:
            print(f"skipping non-csv: {target}", file=sys.stderr)
    return files


def analyze(path: Path) -> None:
    import csv

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if not header:
            print(f"{path.name}: empty")
            return
        rows = 0
        first_ts = last_ts = None
        ts_idx = None
        for name in ("timestamp_us", "timestamp", "t_us"):
            if name in header:
                ts_idx = header.index(name)
                break
        for row in reader:
            rows += 1
            if ts_idx is not None and ts_idx < len(row):
                try:
                    ts = int(row[ts_idx])
                except ValueError:
                    continue
                if first_ts is None:
                    first_ts = ts
                last_ts = ts

    duration_s = (last_ts - first_ts) / 1e6 if first_ts is not None and last_ts is not None else float("nan")
    rate_hz = rows / duration_s if duration_s and duration_s > 0 and not math.isnan(duration_s) else float("nan")
    print(f"{path.name}: cols={len(header)} rows={rows} duration={duration_s:.3f}s rate={rate_hz:.1f}Hz")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("targets", nargs="+", type=Path, help="CSV files or directories to scan")
    args = parser.parse_args(argv)

    files = iter_csvs(args.targets)
    if not files:
        print("no CSV files found", file=sys.stderr)
        return 1
    for path in files:
        try:
            analyze(path)
        except OSError as exc:
            print(f"{path}: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
