#!/usr/bin/env bash
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  '') device=cpu ;;
  --check-cuda) device=cuda:0 ;;
  *) echo "Usage: $0 [--check-cuda]" >&2; exit 2 ;;
esac
(( $# <= 1 )) || exit 2
command -v uv >/dev/null || { echo 'Install uv first' >&2; exit 1; }
python3 "$model_dir/prepare_artifacts.py"
UV_PROJECT_ENVIRONMENT="$model_dir/.venv" uv sync --locked --project "$model_dir"
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
GROUNDING_DINO_DEVICE="$device" .venv/bin/python -c 'from adapter import Predictor; import json; p=Predictor(); print(json.dumps(p.metadata)); p.close(); p.close()'
