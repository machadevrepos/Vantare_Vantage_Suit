#!/usr/bin/env python3
"""Convert Vantare Vantage v4 session BIN files to validated CSV exports.

The converter treats firmware-generated BIN as authoritative. It refuses partial,
sparse legacy Master archives, CRC-corrupt files, and unsupported sample versions.
Only Python's standard library is required.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import re
import struct
import sys
import zlib
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator

SESSION_MAGIC = 0x584F5345  # "ESOX"
SESSION_VERSION = 4
SESSION_COMPLETE = 0xA5
SENSOR_BNO85 = 0x01
SENSOR_ICM45686 = 0x02
KNOWN_SENSOR_MASK = SENSOR_BNO85 | SENSOR_ICM45686
HEADER_SIZE = 88
BNO_SAMPLE_SIZE = 56
ICM_SAMPLE_SIZE = 20
HEADER_CRC_OFFSET = 84
SESSION_FILENAME_RE = re.compile(r"^R\d{4}(?:M|N[1-4])\.BIN$", re.IGNORECASE)

LOSS_FLAGS = {
    0x00000001: "bno_read",
    0x00000002: "icm_read",
    0x00000004: "bno_buffer",
    0x00000008: "icm_buffer",
    0x00000010: "bno_write",
    0x00000020: "icm_write",
}

HEADER_STRUCT = struct.Struct("<IHHIQIIBBHIIIIHHIIIIIIIII")
BNO_STRUCT = struct.Struct("<I13f")
ICM_STRUCT = struct.Struct("<II6h")
assert HEADER_STRUCT.size == HEADER_SIZE
assert BNO_STRUCT.size == BNO_SAMPLE_SIZE
assert ICM_STRUCT.size == ICM_SAMPLE_SIZE


class SessionFormatError(ValueError):
    pass


@dataclasses.dataclass(frozen=True)
class SessionHeader:
    magic: int
    version: int
    node_id: int
    session_id: int
    start_timestamp_us: int
    requested_duration_ms: int
    actual_duration_ms: int
    sensor_mask: int
    completion_flag: int
    reserved: int
    bno85_sample_count: int
    icm45686_sample_count: int
    bno85_payload_size: int
    icm45686_payload_size: int
    bno85_target_rate_hz: int
    icm45686_target_rate_hz: int
    bno85_attempted_count: int
    icm45686_attempted_count: int
    bno85_captured_count: int
    icm45686_captured_count: int
    bno85_dropped_count: int
    icm45686_dropped_count: int
    loss_flags: int
    payload_crc32: int
    header_crc32: int

    @property
    def logical_size(self) -> int:
        return HEADER_SIZE + self.bno85_payload_size + self.icm45686_payload_size

    @property
    def source_label(self) -> str:
        return "MASTER" if self.node_id == 0 else f"NODE{self.node_id}"


def _crc32(data: bytes, crc: int = 0) -> int:
    return zlib.crc32(data, crc) & 0xFFFFFFFF


def _decode_header(raw: bytes) -> SessionHeader:
    if len(raw) != HEADER_SIZE:
        raise SessionFormatError(f"short SessionHeader: {len(raw)} bytes, expected {HEADER_SIZE}")
    return SessionHeader(*HEADER_STRUCT.unpack(raw))


def _loss_names(flags: int) -> list[str]:
    names = [name for bit, name in LOSS_FLAGS.items() if flags & bit]
    unknown = flags & ~sum(LOSS_FLAGS.keys())
    if unknown:
        names.append(f"unknown_0x{unknown:08X}")
    return names


def validate_session(path: Path) -> SessionHeader:
    physical_size = path.stat().st_size
    with path.open("rb") as stream:
        raw_header = stream.read(HEADER_SIZE)
        header = _decode_header(raw_header)

        if header.magic != SESSION_MAGIC:
            raise SessionFormatError(f"bad magic 0x{header.magic:08X}; expected ESOX")
        if header.version != SESSION_VERSION:
            raise SessionFormatError(
                f"unsupported session version {header.version}; this tool is synchronized to firmware v{SESSION_VERSION}"
            )
        if header.completion_flag != SESSION_COMPLETE:
            raise SessionFormatError(
                f"session is not finalized (completion_flag=0x{header.completion_flag:02X})"
            )
        if not 0 <= header.node_id <= 4:
            raise SessionFormatError(f"invalid node_id {header.node_id}")
        if header.sensor_mask & ~KNOWN_SENSOR_MASK:
            raise SessionFormatError(f"unknown sensor mask bits 0x{header.sensor_mask:02X}")
        if header.bno85_payload_size != header.bno85_sample_count * BNO_SAMPLE_SIZE:
            raise SessionFormatError("BNO85 payload size/count mismatch")
        if header.icm45686_payload_size != header.icm45686_sample_count * ICM_SAMPLE_SIZE:
            raise SessionFormatError("ICM45686 payload size/count mismatch")
        expected_mask = 0
        if header.bno85_sample_count:
            expected_mask |= SENSOR_BNO85
        if header.icm45686_sample_count:
            expected_mask |= SENSOR_ICM45686
        if header.sensor_mask != expected_mask:
            raise SessionFormatError(
                f"sensor_mask 0x{header.sensor_mask:02X} does not match payloads (expected 0x{expected_mask:02X})"
            )
        if header.bno85_captured_count != header.bno85_sample_count:
            raise SessionFormatError("BNO85 captured_count does not match sample_count")
        if header.icm45686_captured_count != header.icm45686_sample_count:
            raise SessionFormatError("ICM45686 captured_count does not match sample_count")
        if header.bno85_attempted_count < header.bno85_captured_count:
            raise SessionFormatError("BNO85 attempted_count is less than captured_count")
        if header.icm45686_attempted_count < header.icm45686_captured_count:
            raise SessionFormatError("ICM45686 attempted_count is less than captured_count")
        if physical_size != header.logical_size:
            if physical_size > header.logical_size and header.node_id == 0:
                raise SessionFormatError(
                    f"non-canonical/sparse Master archive: physical={physical_size} logical={header.logical_size}; "
                    "re-record with canonical archive firmware"
                )
            raise SessionFormatError(
                f"file size mismatch: physical={physical_size} logical={header.logical_size}"
            )

        header_for_crc = bytearray(raw_header)
        header_for_crc[HEADER_CRC_OFFSET:HEADER_CRC_OFFSET + 4] = b"\0\0\0\0"
        actual_header_crc = _crc32(header_for_crc)
        if actual_header_crc != header.header_crc32:
            raise SessionFormatError(
                f"header CRC mismatch: stored=0x{header.header_crc32:08X} calculated=0x{actual_header_crc:08X}"
            )

        payload_crc = 0
        remaining = header.bno85_payload_size + header.icm45686_payload_size
        while remaining:
            chunk = stream.read(min(65536, remaining))
            if not chunk:
                raise SessionFormatError("unexpected EOF while validating payload")
            payload_crc = _crc32(chunk, payload_crc)
            remaining -= len(chunk)
        if payload_crc != header.payload_crc32:
            raise SessionFormatError(
                f"payload CRC mismatch: stored=0x{header.payload_crc32:08X} calculated=0x{payload_crc:08X}"
            )
    return header


def _iter_bno(stream: BinaryIO, header: SessionHeader) -> Iterator[tuple]:
    stream.seek(HEADER_SIZE)
    last_offset = -1
    for index in range(header.bno85_sample_count):
        raw = stream.read(BNO_SAMPLE_SIZE)
        if len(raw) != BNO_SAMPLE_SIZE:
            raise SessionFormatError(f"short BNO85 sample at index {index}")
        sample = BNO_STRUCT.unpack(raw)
        if sample[0] < last_offset:
            raise SessionFormatError(f"BNO85 timestamp moved backwards at index {index}")
        if not all(math.isfinite(v) for v in sample[1:]):
            raise SessionFormatError(f"BNO85 contains non-finite value at index {index}")
        last_offset = sample[0]
        yield sample


def _iter_icm(stream: BinaryIO, header: SessionHeader) -> Iterator[tuple]:
    stream.seek(HEADER_SIZE + header.bno85_payload_size)
    last_offset = -1
    expected_sequence = 0
    for index in range(header.icm45686_sample_count):
        raw = stream.read(ICM_SAMPLE_SIZE)
        if len(raw) != ICM_SAMPLE_SIZE:
            raise SessionFormatError(f"short ICM45686 sample at index {index}")
        sample = ICM_STRUCT.unpack(raw)
        offset_us, sequence = sample[:2]
        if offset_us < last_offset:
            raise SessionFormatError(f"ICM45686 timestamp moved backwards at index {index}")
        if sequence != expected_sequence:
            raise SessionFormatError(
                f"ICM45686 sequence gap at sample {index}: got {sequence}, expected {expected_sequence}"
            )
        last_offset = offset_us
        expected_sequence += 1
        yield sample


def _effective_rate(count: int, samples: Iterable[tuple]) -> float:
    # Kept for callers that already materialized offsets; converter uses _rate_from_bounds below.
    values = list(samples)
    if count < 2 or len(values) < 2:
        return 0.0
    span = values[-1][0] - values[0][0]
    return 0.0 if span <= 0 else (len(values) - 1) * 1_000_000.0 / span


def _rate_from_bounds(count: int, first_offset: int | None, last_offset: int | None) -> float:
    if count < 2 or first_offset is None or last_offset is None or last_offset <= first_offset:
        return 0.0
    return (count - 1) * 1_000_000.0 / (last_offset - first_offset)


def _write_bno_csv(path: Path, source: Path, header: SessionHeader) -> tuple[int | None, int | None]:
    columns = [
        "sample_index", "offset_us", "timestamp_us",
        "quat_i", "quat_j", "quat_k", "quat_real",
        "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
        "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
        "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
    ]
    first = last = None
    with source.open("rb") as stream, path.open("w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        writer.writerow(columns)
        for index, sample in enumerate(_iter_bno(stream, header)):
            offset = sample[0]
            if first is None:
                first = offset
            last = offset
            writer.writerow([index, offset, header.start_timestamp_us + offset, *[f"{v:.9g}" for v in sample[1:]]])
    return first, last


def _write_icm_csv(path: Path, source: Path, header: SessionHeader) -> tuple[int | None, int | None]:
    columns = [
        "sample_index", "offset_us", "timestamp_us", "sequence",
        "accel_x_raw", "accel_y_raw", "accel_z_raw",
        "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
        "accel_x_g", "accel_y_g", "accel_z_g",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    ]
    first = last = None
    with source.open("rb") as stream, path.open("w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        writer.writerow(columns)
        for index, sample in enumerate(_iter_icm(stream, header)):
            offset, sequence, ax, ay, az, gx, gy, gz = sample
            if first is None:
                first = offset
            last = offset
            writer.writerow([
                index, offset, header.start_timestamp_us + offset, sequence,
                ax, ay, az, gx, gy, gz,
                f"{ax * 4.0 / 32768.0:.9g}", f"{ay * 4.0 / 32768.0:.9g}", f"{az * 4.0 / 32768.0:.9g}",
                f"{gx * 2000.0 / 32768.0:.9g}", f"{gy * 2000.0 / 32768.0:.9g}", f"{gz * 2000.0 / 32768.0:.9g}",
            ])
    return first, last


def _write_metadata(path: Path, source: Path, header: SessionHeader,
                    bno_bounds: tuple[int | None, int | None],
                    icm_bounds: tuple[int | None, int | None]) -> dict:
    bno_rate = _rate_from_bounds(header.bno85_sample_count, *bno_bounds)
    icm_rate = _rate_from_bounds(header.icm45686_sample_count, *icm_bounds)
    metadata = dataclasses.asdict(header)
    metadata.update({
        "input_file": source.name,
        "source_label": header.source_label,
        "validated": True,
        "format": "ESOX/v4 canonical contiguous",
        "loss_flag_names": _loss_names(header.loss_flags),
        "bno85_effective_rate_hz": round(bno_rate, 6),
        "icm45686_effective_rate_hz": round(icm_rate, 6),
        "bno85_rate_ratio": round(bno_rate / header.bno85_target_rate_hz, 6) if header.bno85_target_rate_hz else None,
        "icm45686_rate_ratio": round(icm_rate / header.icm45686_target_rate_hz, 6) if header.icm45686_target_rate_hz else None,
    })
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def convert_one(source: Path, output_dir: Path) -> dict:
    header = validate_session(source)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = source.stem
    bno_path = output_dir / f"{stem}_BNO85.csv"
    icm_path = output_dir / f"{stem}_ICM45686.csv"
    meta_path = output_dir / f"{stem}_metadata.json"
    bno_bounds = _write_bno_csv(bno_path, source, header)
    icm_bounds = _write_icm_csv(icm_path, source, header)
    metadata = _write_metadata(meta_path, source, header, bno_bounds, icm_bounds)
    return {"header": header, "metadata": metadata, "bno": bno_path, "icm": icm_path, "meta": meta_path}


def discover_inputs(items: list[Path]) -> list[Path]:
    found: list[Path] = []
    for item in items:
        if item.is_dir():
            found.extend(sorted(
                p for p in item.iterdir()
                if p.is_file() and SESSION_FILENAME_RE.match(p.name)
            ))
        else:
            found.append(item)
    unique: list[Path] = []
    seen = set()
    for path in found:
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(path)
    return unique


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate Vantare Vantage ESOX/v4 BIN recordings and export BNO85/ICM45686 CSV files."
    )
    parser.add_argument("inputs", nargs="+", type=Path, help="BIN file(s) or directories containing R####M/N#.BIN")
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("converted_csv"), help="output directory")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    inputs = discover_inputs(args.inputs)
    if not inputs:
        print("error: no .BIN inputs found", file=sys.stderr)
        return 2
    failures = 0
    for source in inputs:
        try:
            result = convert_one(source, args.output_dir)
            h: SessionHeader = result["header"]
            m = result["metadata"]
            loss = ",".join(m["loss_flag_names"]) or "none"
            print(
                f"OK {source.name}: {h.source_label} session={h.session_id} "
                f"BNO={h.bno85_sample_count} ({m['bno85_effective_rate_hz']:.2f} Hz) "
                f"ICM={h.icm45686_sample_count} ({m['icm45686_effective_rate_hz']:.2f} Hz) loss={loss}"
            )
        except (OSError, SessionFormatError) as exc:
            failures += 1
            print(f"ERROR {source}: {exc}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())