#!/usr/bin/env bash
# Repo bootstrap: verify toolchain + python deps for host tests.
set -euo pipefail

echo '== Python =='
python3 --version

echo '== Python test deps =='
python3 -m pip install --quiet pytest

echo '== CMake =='
command -v cmake >/dev/null || echo 'WARNING: cmake not found; C++ host tests unavailable' >&2

echo '== C++ compiler =='
command -v g++ >/dev/null || echo 'WARNING: g++ not found; install a C++17 toolchain' >&2

echo 'setup complete.'
