#!/usr/bin/env bash
# Prepare shared host tools in the repository environment.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if (( $# )); then
  echo "Usage: bash setup_python.sh" >&2
  exit 2
fi
# Preserve the configured editable SDK and its optional dependencies.
UV_PROJECT_ENVIRONMENT="$root/.venv" uv sync --project "$root" --locked --inexact
