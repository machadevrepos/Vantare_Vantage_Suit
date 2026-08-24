#!/usr/bin/env bash
# Formats all first-party C/C++ sources with clang-format.
set -euo pipefail

command -v clang-format >/dev/null || { echo 'clang-format not found; install LLVM' >&2; exit 2; }

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

find "$repo_root/firmware" "$repo_root/host" -type f \( \
    -name '*.cpp' -o -name '*.h' -o -name '*.cc' -o -name '*.hpp' \) \
    ! -path '*/build/*' ! -path '*/Debug/*' ! -path '*/Release/*' \
    ! -path '*/third_party/*' ! -path '*/STM32_WPAN/*' ! -path '*/Middlewares/*' \
    ! -path '*/Drivers/*' ! -path '*/Utilities/*' ! -path '*/FATFS/*' \
    -print0 | xargs -0 -n1 clang-format -i --style=file

echo 'format complete.'
