#!/usr/bin/env bash
# Runs the full host test suite: Python invariant tests + C++ module tests.
#
#   ./host/tests/run_tests.sh          # everything
#   ./host/tests/run_tests.sh py       # python only
#   ./host/tests/run_tests.sh cpp      # c++ only
set -euo pipefail

mode="${1:-all}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

run_python() {
    echo '== Python invariant tests =='
    python3 -m pytest "$repo_root/host/tests/python" -v
}

run_cpp() {
    echo '== C++ module tests =='
    cmake -S "$repo_root/host/tests/cpp" -B "$repo_root/host/tests/cpp/build"
    cmake --build "$repo_root/host/tests/cpp/build" -j
    ctest --test-dir "$repo_root/host/tests/cpp/build" --output-on-failure
}

case "$mode" in
    py)  run_python ;;
    cpp) run_cpp ;;
    all) run_python; run_cpp ;;
    *) echo "usage: $0 [all|py|cpp]" >&2; exit 2 ;;
esac
