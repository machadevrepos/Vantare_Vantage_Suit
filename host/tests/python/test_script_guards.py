#!/usr/bin/env python3
"""Pytest wrapper that runs the script-style firmware invariant guards.

The invariant guards are standalone scripts exiting non-zero on violation;
this keeps them in the default `pytest` run alongside unit tests.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

GUARDS = [
    "test_acquisition_demo_invariants.py",
    "test_binary_first_invariants.py",
    "test_ble_commissioning_path_invariants.py",
    "test_ble_only_cleanup.py",
    "test_remote_transfer_lifecycle_source.py",
    "test_repair_invariants.py",
    "test_stop_delivery_invariants.py",
]

HERE = Path(__file__).resolve().parent


@pytest.mark.parametrize("script", GUARDS)
def test_script_guard(script: str) -> None:
    result = subprocess.run(
        [sys.executable, str(HERE / script)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        f"{script} failed:\n{result.stdout}\n{result.stderr}"
    )
