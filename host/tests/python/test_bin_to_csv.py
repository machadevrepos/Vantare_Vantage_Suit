#!/usr/bin/env python3
from __future__ import annotations
import importlib.util
import json
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT / 'host' / 'desktop_tool' / 'vantage_bin_to_csv.py'
spec = importlib.util.spec_from_file_location('vantage_bin_to_csv', MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def make_bin(path: Path, *, node_id=0, corrupt_payload=False, sparse=False):
    bno = mod.BNO_STRUCT.pack(0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0, 0.0, 0.0, 9.81, 0.1, 0.2, 0.3)
    icm0 = mod.ICM_STRUCT.pack(0, 0, 100, -200, 300, 400, -500, 600)
    icm1 = mod.ICM_STRUCT.pack(5000, 1, 101, -201, 301, 401, -501, 601)
    payload = bno + icm0 + icm1
    payload_crc = zlib.crc32(payload) & 0xffffffff
    values = [
        mod.SESSION_MAGIC, mod.SESSION_VERSION, node_id, 42, 300000,
        1000, 10, mod.SENSOR_BNO85 | mod.SENSOR_ICM45686, mod.SESSION_COMPLETE, 0,
        1, 2, len(bno), len(icm0) + len(icm1), 100, 200,
        1, 2, 1, 2, 0, 0, 0, payload_crc, 0,
    ]
    raw = bytearray(mod.HEADER_STRUCT.pack(*values))
    raw[mod.HEADER_CRC_OFFSET:mod.HEADER_CRC_OFFSET+4] = struct.pack('<I', zlib.crc32(raw) & 0xffffffff)
    if corrupt_payload:
        payload = payload[:-1] + bytes([payload[-1] ^ 0x55])
    if sparse:
        data = bytes(raw) + bno + (b'\0' * 1024) + icm0 + icm1
    else:
        data = bytes(raw) + payload
    path.write_bytes(data)


class ConverterTests(unittest.TestCase):
    def test_valid_v4_exports_csv_and_metadata(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source = root / 'R0042M.BIN'
            make_bin(source)
            out = root / 'csv'
            result = mod.convert_one(source, out)
            self.assertTrue(result['bno'].exists())
            self.assertTrue(result['icm'].exists())
            meta = json.loads(result['meta'].read_text())
            self.assertTrue(meta['validated'])
            self.assertEqual(meta['source_label'], 'MASTER')
            self.assertAlmostEqual(meta['icm45686_effective_rate_hz'], 200.0)
            self.assertIn('accel_x_g', result['icm'].read_text().splitlines()[0])

    def test_rejects_corrupt_payload(self):
        with tempfile.TemporaryDirectory() as td:
            source = Path(td) / 'R0042N2.BIN'
            make_bin(source, node_id=2, corrupt_payload=True)
            with self.assertRaisesRegex(mod.SessionFormatError, 'payload CRC mismatch'):
                mod.validate_session(source)

    def test_rejects_sparse_master(self):
        with tempfile.TemporaryDirectory() as td:
            source = Path(td) / 'R0042M.BIN'
            make_bin(source, sparse=True)
            with self.assertRaisesRegex(mod.SessionFormatError, 'non-canonical/sparse Master archive'):
                mod.validate_session(source)

    def test_directory_discovery_ignores_non_session_bin_files(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            session = root / 'R0042N2.BIN'
            make_bin(session, node_id=2)
            (root / 'HUBTEST.BIN').write_bytes(b'LOG1')
            self.assertEqual(mod.discover_inputs([root]), [session])

    def test_rejects_icm_sequence_gap(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source = root / 'R0042N2.BIN'
            make_bin(source, node_id=2)
            data = bytearray(source.read_bytes())
            seq_off = mod.HEADER_SIZE + mod.BNO_SAMPLE_SIZE + mod.ICM_SAMPLE_SIZE + 4
            data[seq_off:seq_off+4] = struct.pack('<I', 9)
            payload_crc = zlib.crc32(data[mod.HEADER_SIZE:]) & 0xffffffff
            data[80:84] = struct.pack('<I', payload_crc)
            data[84:88] = b'\0\0\0\0'
            data[84:88] = struct.pack('<I', zlib.crc32(data[:88]) & 0xffffffff)
            source.write_bytes(data)
            header = mod.validate_session(source)
            with source.open('rb') as stream:
                with self.assertRaisesRegex(mod.SessionFormatError, 'sequence gap'):
                    list(mod._iter_icm(stream, header))


if __name__ == '__main__':
    unittest.main()