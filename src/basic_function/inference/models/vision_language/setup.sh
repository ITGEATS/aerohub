#!/usr/bin/env bash
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if (( $# )); then
  echo 'Cloud model preparation accepts no CUDA flags; no local model artifact is provided.' >&2
  exit 2
fi
command -v uv >/dev/null || { echo 'Install uv first' >&2; exit 1; }
uv sync --locked --project "$model_dir" --python 3.10
# No API calls during preparation. Credentials are validated at worker startup.
