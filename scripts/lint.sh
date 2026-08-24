#!/usr/bin/env bash
# Lints C/C++ sources with cppcheck.
set -euo pipefail

command -v cppcheck >/dev/null || { echo 'cppcheck not found; install cppcheck' >&2; exit 2; }

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cppcheck \
    --enable=warning,performance,portability \
    --std=c++17 \
    --inline-suppr \
    --error-exitcode=1 \
    --suppress='*:*/third_party/*' \
    --suppress='*:*/STM32_WPAN/*' \
    --suppress='*:*/Middlewares/*' \
    --suppress='*:*/Drivers/*' \
    --suppress='*:*/Utilities/*' \
    -I "$repo_root/firmware/common/inc" \
    "$repo_root/firmware/common/src" \
    "$repo_root/host/tests/cpp"
