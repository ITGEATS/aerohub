#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
model_dir="$repo_dir/src/basic_function/inference/models/grounding_dino"
[[ -x "$model_dir/.venv/bin/python" ]] || { echo 'Run grounding_dino/setup.sh --check-cuda first' >&2; exit 1; }
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
exec .venv/bin/python -m uvicorn serve:app --host "${INFERENCE_HOST:-127.0.0.1}" --port "${INFERENCE_PORT:-8003}" --workers 1 --limit-concurrency 16
