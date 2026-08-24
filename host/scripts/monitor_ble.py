#!/usr/bin/env python3
"""Scan for the Master's BLE peripheral and print advertised data.

Requires the optional `bleak` package (pip install bleak).

Usage:
    python host/scripts/monitor_ble.py [--name-contains VANTARE] [--seconds 10]
"""

from __future__ import annotations

import argparse
import asyncio
import sys


async def scan(name_contains: str, seconds: float) -> int:
    try:
        from bleak import BleakScanner
    except ImportError:
        print("bleak is required: pip install bleak", file=sys.stderr)
        return 2

    devices = await BleakScanner.discover(timeout=seconds)
    matches = [d for d in devices if name_contains.lower() in (d.name or "").lower()]
    if not matches:
        print(f"no devices matching '{name_contains}' found ({len(devices)} total)")
        return 1
    for device in matches:
        print(f"{device.address}  {device.name}  rssi={device.rssi}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name-contains", default="VANTARE", help="filter advertised names")
    parser.add_argument("--seconds", type=float, default=10.0, help="scan window")
    args = parser.parse_args()
    try:
        return asyncio.run(scan(args.name_contains, args.seconds))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
