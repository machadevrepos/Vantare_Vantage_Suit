#!/usr/bin/env python3
"""Validate source coverage, schema-v2 features, timing, and quality metadata."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from pathlib import Path

SCHEMA_VERSION = 2
SENSORS = ("BNO85", "ICM45686")
COMMON_COLUMNS = {
    "schema_version", "session_id", "row_sequence", "source_node_id",
    "source_label", "sensor_id", "sensor_sequence", "session_time_us",
    "sample_delta_us", "effective_sample_rate_hz", "source_target_rate_hz",
    "source_attempted_count", "source_captured_count", "source_dropped_count",
    "source_loss_flags", "source_payload_crc32", "timestamp_quality_flags",
}
BNO_RAW_COLUMNS = {
    "bno_qx", "bno_qy", "bno_qz", "bno_qw",
    "bno_linear_x_mps2", "bno_linear_y_mps2", "bno_linear_z_mps2",
    "bno_gravity_x_mps2", "bno_gravity_y_mps2", "bno_gravity_z_mps2",
    "bno_gyro_x_radps", "bno_gyro_y_radps", "bno_gyro_z_radps",
}
ICM_RAW_COLUMNS = {
    "icm_accel_x_raw", "icm_accel_y_raw", "icm_accel_z_raw",
    "icm_gyro_x_raw", "icm_gyro_y_raw", "icm_gyro_z_raw",
    "icm_accel_x_g", "icm_accel_y_g", "icm_accel_z_g",
    "icm_gyro_x_dps", "icm_gyro_y_dps", "icm_gyro_z_dps",
}
DERIVED_COLUMNS = {
    "bno_roll_deg", "bno_pitch_deg", "bno_yaw_deg",
    "bno_linear_accel_magnitude_mps2", "bno_gravity_magnitude_mps2",
    "bno_gyro_magnitude_radps", "icm_accel_magnitude_g",
    "icm_gyro_magnitude_dps",
}
REQUIRED_COLUMNS = COMMON_COLUMNS | BNO_RAW_COLUMNS | ICM_RAW_COLUMNS | DERIVED_COLUMNS


def parse_sources(value: str) -> set[int]:
    try:
        sources = {int(part.strip(), 10) for part in value.split(",") if part.strip()}
    except ValueError as exc:
        raise argparse.ArgumentTypeError("sources must be comma-separated integers") from exc
    if not sources or any(source < 0 or source > 4 for source in sources):
        raise argparse.ArgumentTypeError("sources must be a non-empty subset of 0,1,2,3,4")
    return sources


def parse_int(row: dict[str, str], column: str, line_number: int) -> int:
    try:
        return int((row.get(column) or "").strip(), 10)
    except ValueError as exc:
        raise ValueError(f"invalid {column} at CSV line {line_number}") from exc


def finite_fields(row: dict[str, str], columns: set[str]) -> bool:
    for column in columns:
        value = (row.get(column) or "").strip()
        if not value:
            return False
        try:
            if not math.isfinite(float(value)):
                return False
        except ValueError:
            return False
    return True


def validate(path: Path, expected_sources: set[int]) -> int:
    counts: Counter[tuple[int, str]] = Counter()
    source_rows: Counter[int] = Counter()
    representative: dict[tuple[int, str], bool] = {}
    previous_time: dict[tuple[int, str], int] = {}
    errors: list[str] = []

    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        missing_columns = REQUIRED_COLUMNS.difference(set(reader.fieldnames or []))
        if missing_columns:
            print(
                f"ERROR: {path} is missing schema-v2 columns: "
                f"{', '.join(sorted(missing_columns))}",
                file=sys.stderr,
            )
            return 2

        expected_row_sequence = 0
        for line_number, row in enumerate(reader, start=2):
            try:
                schema = parse_int(row, "schema_version", line_number)
                source = parse_int(row, "source_node_id", line_number)
                row_sequence = parse_int(row, "row_sequence", line_number)
                sensor_sequence = parse_int(row, "sensor_sequence", line_number)
                session_time = parse_int(row, "session_time_us", line_number)
                quality = parse_int(row, "timestamp_quality_flags", line_number)
                target_rate = parse_int(row, "source_target_rate_hz", line_number)
                attempted = parse_int(row, "source_attempted_count", line_number)
                captured = parse_int(row, "source_captured_count", line_number)
                dropped = parse_int(row, "source_dropped_count", line_number)
                parse_int(row, "source_loss_flags", line_number)
                parse_int(row, "source_payload_crc32", line_number)
            except ValueError as exc:
                errors.append(str(exc))
                continue

            sensor = (row.get("sensor_id") or "").strip().upper()
            if schema != SCHEMA_VERSION:
                errors.append(f"CSV line {line_number}: schema_version={schema}, expected {SCHEMA_VERSION}")
            if source < 0 or source > 4:
                errors.append(f"CSV line {line_number}: source_node_id={source} is outside 0..4")
                continue
            label = "MASTER" if source == 0 else f"NODE{source}"
            if (row.get("source_label") or "").strip().upper() != label:
                errors.append(f"CSV line {line_number}: source label does not match source {source}")
            if sensor not in SENSORS:
                errors.append(f"CSV line {line_number}: unsupported sensor_id={sensor!r}")
                continue
            if row_sequence != expected_row_sequence:
                errors.append(f"CSV line {line_number}: row_sequence={row_sequence}, expected {expected_row_sequence}")
            expected_row_sequence = row_sequence + 1
            if sensor_sequence != counts[(source, sensor)]:
                errors.append(
                    f"CSV line {line_number}: {label}/{sensor} sensor_sequence="
                    f"{sensor_sequence}, expected {counts[(source, sensor)]}"
                )
            if target_rate <= 0 or attempted < captured or captured <= 0 or dropped < 0:
                errors.append(f"CSV line {line_number}: invalid source capture metadata")

            key = (source, sensor)
            delta_text = (row.get("sample_delta_us") or "").strip()
            rate_text = (row.get("effective_sample_rate_hz") or "").strip()
            if key not in previous_time:
                if delta_text or rate_text:
                    errors.append(f"CSV line {line_number}: first {label}/{sensor} row has timing deltas")
                previous_time[key] = session_time
            elif session_time <= previous_time[key]:
                if quality == 0:
                    errors.append(f"CSV line {line_number}: non-monotonic timestamp lacks quality flag")
                if delta_text or rate_text:
                    errors.append(f"CSV line {line_number}: invalid timestamp has derived timing values")
            else:
                expected_delta = session_time - previous_time[key]
                if quality != 0:
                    if delta_text or rate_text:
                        errors.append(f"CSV line {line_number}: flagged timestamp has derived timing values")
                else:
                    try:
                        delta = int(delta_text, 10)
                        rate = float(rate_text)
                    except ValueError:
                        errors.append(f"CSV line {line_number}: missing or invalid timing features")
                    else:
                        expected_rate = 1_000_000.0 / expected_delta
                        if delta != expected_delta or not math.isfinite(rate) or rate <= 0.0:
                            errors.append(f"CSV line {line_number}: incorrect timing features")
                        elif not math.isclose(rate, expected_rate, rel_tol=1e-5, abs_tol=1e-5):
                            errors.append(f"CSV line {line_number}: effective sample rate mismatch")
                    previous_time[key] = session_time

            if sensor == "BNO85":
                representative[key] = representative.get(key, False) or finite_fields(
                    row,
                    BNO_RAW_COLUMNS | {
                        "bno_roll_deg", "bno_pitch_deg", "bno_yaw_deg",
                        "bno_linear_accel_magnitude_mps2",
                        "bno_gravity_magnitude_mps2", "bno_gyro_magnitude_radps",
                    },
                )
            else:
                representative[key] = representative.get(key, False) or finite_fields(
                    row,
                    ICM_RAW_COLUMNS | {"icm_accel_magnitude_g", "icm_gyro_magnitude_dps"},
                )
            source_rows[source] += 1
            counts[key] += 1

    failed = bool(errors)
    for message in errors[:25]:
        print(f"ERROR: {message}", file=sys.stderr)
    if len(errors) > 25:
        print(f"ERROR: {len(errors) - 25} additional validation errors", file=sys.stderr)

    for source in sorted(expected_sources):
        label = "MASTER" if source == 0 else f"NODE{source}"
        rows = source_rows[source]
        bno_rows = counts[(source, "BNO85")]
        icm_rows = counts[(source, "ICM45686")]
        print(f"{label}: rows={rows} BNO85={bno_rows} ICM45686={icm_rows}")
        if rows == 0:
            print(f"ERROR: {label} is absent from the CSV", file=sys.stderr)
            failed = True
        for sensor, sensor_rows in (("BNO85", bno_rows), ("ICM45686", icm_rows)):
            if sensor_rows == 0:
                print(f"ERROR: {label} has no {sensor} rows", file=sys.stderr)
                failed = True
            elif not representative.get((source, sensor), False):
                print(
                    f"ERROR: {label}/{sensor} has no row with complete finite raw and derived features",
                    file=sys.stderr,
                )
                failed = True

    unexpected = sorted(set(source_rows).difference(expected_sources))
    if unexpected:
        print(f"INFO: additional sources present: {unexpected}")
    if failed:
        return 1
    print("CSV schema-v2 source, timing, and feature validation passed")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--sources", type=parse_sources, required=True,
        help="expected source IDs, for example 0,1,2,3,4",
    )
    args = parser.parse_args()
    if not args.csv_path.is_file():
        parser.error(f"file does not exist: {args.csv_path}")
    return validate(args.csv_path, args.sources)


if __name__ == "__main__":
    raise SystemExit(main())
