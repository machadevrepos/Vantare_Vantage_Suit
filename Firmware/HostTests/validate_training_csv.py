#!/usr/bin/env python3
"""Validate that a training CSV contains every expected recording source and sensor."""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path


def parse_sources(value: str) -> set[int]:
    try:
        sources = {int(part.strip(), 10) for part in value.split(",") if part.strip()}
    except ValueError as exc:
        raise argparse.ArgumentTypeError("sources must be comma-separated integers") from exc
    if not sources or any(source < 0 or source > 4 for source in sources):
        raise argparse.ArgumentTypeError("sources must be a non-empty subset of 0,1,2,3,4")
    return sources


def validate(path: Path, expected_sources: set[int]) -> int:
    counts: Counter[tuple[int, str]] = Counter()
    source_rows: Counter[int] = Counter()

    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required_columns = {"source_node_id", "sensor_id"}
        missing_columns = required_columns.difference(reader.fieldnames or [])
        if missing_columns:
            print(
                f"ERROR: {path} is missing columns: {', '.join(sorted(missing_columns))}",
                file=sys.stderr,
            )
            return 2

        for line_number, row in enumerate(reader, start=2):
            try:
                source = int(row["source_node_id"], 10)
            except (TypeError, ValueError):
                print(
                    f"ERROR: invalid source_node_id at CSV line {line_number}",
                    file=sys.stderr,
                )
                return 2
            sensor = (row["sensor_id"] or "").strip().upper()
            source_rows[source] += 1
            counts[(source, sensor)] += 1

    failed = False
    for source in sorted(expected_sources):
        label = "MASTER" if source == 0 else f"NODE{source}"
        rows = source_rows[source]
        bno_rows = counts[(source, "BNO85")]
        icm_rows = counts[(source, "ICM45686")]
        print(f"{label}: rows={rows} BNO85={bno_rows} ICM45686={icm_rows}")
        if rows == 0:
            print(f"ERROR: {label} is absent from the CSV", file=sys.stderr)
            failed = True
        if bno_rows == 0:
            print(f"ERROR: {label} has no BNO85 rows", file=sys.stderr)
            failed = True
        if icm_rows == 0:
            print(f"ERROR: {label} has no ICM45686 rows", file=sys.stderr)
            failed = True

    unexpected = sorted(set(source_rows).difference(expected_sources))
    if unexpected:
        print(f"INFO: additional sources present: {unexpected}")

    if failed:
        return 1

    print("CSV source coverage passed")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--sources",
        type=parse_sources,
        required=True,
        help="expected source IDs, for example 0,1,2",
    )
    args = parser.parse_args()

    if not args.csv_path.is_file():
        parser.error(f"file does not exist: {args.csv_path}")
    return validate(args.csv_path, args.sources)


if __name__ == "__main__":
    raise SystemExit(main())
