#!/usr/bin/env bash
# Batch-convert session BIN recordings to CSV via the desktop tool converter.
#
#   ./host/scripts/convert_bin_to_csv.sh data/sessions out/csv
set -euo pipefail

input="${1:-data/sessions}"
output="${2:-data/converted_csv}"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
converter="$script_dir/../desktop_tool/vantage_bin_to_csv.py"

exec python3 "$converter" "$input" -o "$output"
