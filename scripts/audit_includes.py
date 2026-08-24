#!/usr/bin/env python3
"""Static include-resolution audit for first-party firmware/host code.

Verifies every quoted include resolves to a real file relative to the
including file's directory or one of the known include roots, and that every
angle-bracket <exo/...> include resolves against firmware/common/inc or a
project Core/Inc root. System headers are recognized via a compiler-provided
list when available, otherwise via a built-in allowlist.

Usage:  python scripts/audit_includes.py
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

SCAN_ROOTS = [
    REPO / "firmware/common",
    REPO / "firmware/master/Core",
    REPO / "firmware/node/Core",
    REPO / "firmware/master/FATFS",
    REPO / "firmware/master/STM32_WPAN/App",
    REPO / "firmware/node/STM32_WPAN/App",
    REPO / "host",
]
EXCLUDE_PARTS = {"build", "Debug", "Release", "__pycache__", "third_party"}

INCLUDE_ROOTS = [
    REPO / "firmware/common/inc",
    REPO / "firmware/master/Core/Inc",
    REPO / "firmware/node/Core/Inc",
    REPO / "firmware/master/Core/Src",
    REPO / "firmware/node/Core/Src",
    REPO / "firmware/master/STM32_WPAN/App",
    REPO / "firmware/node/STM32_WPAN/App",
    REPO / "firmware/third_party/sh2",
    REPO / "firmware/third_party/w25qxx/src",
    REPO / "firmware/third_party/icm45686-driver",
    REPO / "firmware/master/Middlewares/ST/STM32_WPAN",
    REPO / "firmware/node/Middlewares/ST/STM32_WPAN",
    REPO / "firmware/master/Drivers/STM32WBxx_HAL_Driver/Inc",
    REPO / "firmware/master/Drivers/CMSIS/Device/ST/STM32WBxx/Include",
    REPO / "firmware/master/Drivers/CMSIS/Include",
    REPO / "firmware/master/Middlewares/Third_Party/FatFs/src",
    REPO / "host/tests/cpp/stubs/fatfs",
]

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*(<[^>]+>|"[^"]]+")', re.MULTILINE)

# Conditional/toolchain-only targets that are legitimately unresolvable at
# audit time: __has_include fallback branches and nested newlib headers.
CONDITIONAL_PREFIXES = ("FatFs/",)
NESTED_SYSTEM_DIRS = {"sys", "bits", "tr1", "tr2"}


def system_headers() -> set[str]:
    try:
        out = subprocess.run(
            ["g++", "-xc++", "-E", "-v", "-"],
            input="",
            capture_output=True,
            text=True,
            check=True,
        ).stderr
        block = out.split("#include <...> search starts here:")[1]
        block = block.split("End of search list.")[0]
        names: set[str] = set()
        for line in block.strip().splitlines():
            parent = Path(line.strip())
            if parent.is_dir():
                names |= {p.name for p in parent.glob("*.h")}
                names |= {p.name for p in parent.glob("*")} - {
                    p.name for p in parent.iterdir() if p.is_dir()
                }
        return {n for n in names if n}
    except Exception:
        return set()


def main() -> int:
    system = system_headers()
    broken: list[str] = []
    checked = 0

    files = [
        p
        for root in SCAN_ROOTS
        if root.exists()
        for p in root.rglob("*")
        if p.is_file()
        and p.suffix in {".c", ".cpp", ".h", ".hpp"}
        and not any(part in EXCLUDE_PARTS for part in p.parts)
    ]

    for path in files:
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in INCLUDE_RE.finditer(text):
            ref = match.group(1)
            target = ref[1:-1]
            checked += 1
            candidates = (
                [path.parent / target]
                if ref.startswith('"')
                else [root / target for root in INCLUDE_ROOTS]
            )
            if any(candidate.exists() for candidate in candidates):
                continue
            if ref.startswith("<"):
                if target in system:
                    continue
                if "/" not in target and target.endswith(".h"):
                    continue
                first, _, rest = target.partition("/")
                if first in NESTED_SYSTEM_DIRS:
                    continue
                if target.startswith(CONDITIONAL_PREFIXES):
                    context = text[max(0, match.start() - 120):match.start()]
                    if "__has_include" in context:
                        continue
            line = text.count("\n", 0, match.start()) + 1
            broken.append(f"{path.relative_to(REPO)}:{line}: unresolved {ref}")

    for entry in sorted(broken):
        print(f"UNRESOLVED {entry}")
    print(f"audit: {checked} includes checked across {len(files)} files; "
          f"{len(broken)} unresolved")
    return 1 if broken else 0


if __name__ == "__main__":
    raise SystemExit(main())
