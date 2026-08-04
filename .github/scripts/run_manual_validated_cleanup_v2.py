#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import apply_validated_firmware_cleanup as cleanup
import run_manual_validated_cleanup as manual


_original_compile_test = cleanup.compile_test


def compile_test(source: str, extra: list[str] | None = None,
                 suffix: str = '') -> Path:
    includes = list(extra or [])
    target_include = ['-I', 'Firmware/Master/FATFS/Target']
    if target_include[1] not in includes:
        includes.extend(target_include)
    return _original_compile_test(source, extra=includes, suffix=suffix)


cleanup.compile_test = compile_test
raise SystemExit(manual.main())
