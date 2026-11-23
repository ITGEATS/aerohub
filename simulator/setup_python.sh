#!/usr/bin/env bash
# Prepare the repository Python environment for the simulator manager and clients.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in
  ""|--airsim-only) ;;
  *) echo "Usage: simulator/setup_python.sh [--airsim-only]" >&2; exit 2 ;;
esac
if (( $# > 1 )); then
  echo "Usage: simulator/setup_python.sh [--airsim-only]" >&2
  exit 2
fi
bash "$root/setup_python.sh"
python="$root/.venv/bin/python"
if [[ "${1:-}" != --airsim-only ]]; then
  "$python" "$root/simulator/projectairsim/setup_client.py"
fi
