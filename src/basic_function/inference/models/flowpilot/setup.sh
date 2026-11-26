#!/usr/bin/env bash
# Prepare only this model's environment and weights; never start ROS or a server.
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$model_dir/../../../../.." && pwd)"
check_cuda=false
case "${1:-}" in
  "") ;;
  --check-cuda) check_cuda=true ;;
  -h|--help)
    echo "Usage: bash $0 [--check-cuda]"
    echo "Prepare locked dependencies and verified weights; validate CUDA load/warmup."
    echo "--check-cuda additionally runs one real GPU inference with a fixed observation."
    exit 0 ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo "Too many arguments" >&2; exit 2; fi
command -v uv >/dev/null || { echo "Install uv before preparing FlowPilot" >&2; exit 1; }
upstream="$repo_dir/src/tool/navigation/planner/flowpilot/upstream"
expected_revision=f29f1bfee1399c96e5fd384276d5d7a09876286b
if [[ ! -f "$upstream/inference/runtime.py" ]]; then
  echo "Initialize algorithm source first: git submodule update --init src/tool/navigation/planner/flowpilot/upstream" >&2
  echo "(or: git clone https://github.com/flyingbitac/FlowPilot.git src/tool/navigation/planner/flowpilot/upstream && git -C src/tool/navigation/planner/flowpilot/upstream checkout $expected_revision)" >&2
  exit 1
fi
if [[ "$(git -C "$upstream" rev-parse HEAD)" != "$expected_revision" ]]; then
  echo "FlowPilot source differs from the supported revision $expected_revision; preserving checkout" >&2
  exit 1
fi
if [[ -n "$(git -C "$upstream" status --porcelain)" ]]; then
  echo "FlowPilot upstream has local changes; preserving checkout:" >&2
  git -C "$upstream" status --short >&2
  exit 1
fi
cd "$model_dir"
export UV_PROJECT_ENVIRONMENT="$model_dir/.venv"
# Standard-library download and checksum preparation; URLs and revisions live in prepare.py.
uv run --no-project --python 3.11 python prepare.py
uv sync --locked --python 3.11 --project "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
echo "Validating prepared weights and CUDA load/warmup (not GPU acceptance)"
.venv/bin/python - <<'PY'
from adapter import Predictor
predictor = Predictor()
print('CUDA preparation passed:', predictor.metadata)
predictor.close()
PY
if "$check_cuda"; then
  echo "Validating one real CUDA inference with a fixed observation"
  .venv/bin/python - <<'PY'
import numpy as np
import torch
from adapter import Predictor
predictor = Predictor()
try:
    request = dict(request_id='prepare-check',
                   depth_m=np.full((96, 160), 5., dtype=np.float32).tolist(),
                   state_goal=[0.] * 25, speed_mps=2.)
    result = predictor.infer(request)
    print('CUDA inference passed:', torch.cuda.get_device_name(), sorted(result))
finally:
    predictor.close()
PY
fi
echo "FlowPilot prepared. Start with: cd $model_dir && .venv/bin/python -m uvicorn serve:app --host 127.0.0.1 --port 18082"
