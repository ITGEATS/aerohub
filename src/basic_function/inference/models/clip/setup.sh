#!/usr/bin/env bash
# Prepare the clip worker: verify/fetch the HF openai/clip-vit-large-patch14
# port into ignored artifacts/ (sha256-pinned, never silently overwritten),
# sync the locked uv environment, load the default weights and warm up.
# --check-cuda adds a real GPU check. Starts no services.
# Idempotent: correct files are never rewritten.
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
check_cuda=false
case "${1:-}" in
  '') ;;
  --check-cuda) check_cuda=true ;;
  -h|--help) echo "Usage: bash $0 [--check-cuda]"; exit 0 ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Unexpected extra arguments' >&2; exit 2; fi
command -v uv >/dev/null || { echo 'Install uv first' >&2; exit 1; }
host_python=""
for candidate in python3 python; do
  if command -v "$candidate" >/dev/null 2>&1 && "$candidate" -c 'import sys' >/dev/null 2>&1; then
    host_python="$candidate"; break
  fi
done
if [ -z "$host_python" ]; then echo 'A working Python 3 is required to run prepare_artifacts.py' >&2; exit 1; fi

"$host_python" "$model_dir/prepare_artifacts.py"

export UV_PROJECT_ENVIRONMENT="$model_dir/.venv"
uv sync --locked --project "$model_dir"
if [ -f "$model_dir/.venv/Scripts/python.exe" ]; then venv_python="$model_dir/.venv/Scripts/python.exe"
else venv_python="$model_dir/.venv/bin/python"; fi

export PYTHONDONTWRITEBYTECODE=1
cd "$model_dir"
echo "CPU load/warmup check (preparation check only)"
CLIP_DEVICE=cpu "$venv_python" -c 'from adapter import Predictor; p=Predictor(); print(p.metadata["model_id"], p.metadata["device"], "warmup_seconds=%.1f" % p.warmup_seconds); p.close()'
if "$check_cuda"; then
  echo "CUDA check: real GPU load and forward warmup"
  CLIP_DEVICE=0 "$venv_python" -c 'import logging; logging.basicConfig(level=logging.INFO); from adapter import Predictor; p=Predictor(); print(p.metadata["device"], "warmup_seconds=%.2f" % p.warmup_seconds, p.metadata.get("cuda_memory_mib")); p.close()'
fi
