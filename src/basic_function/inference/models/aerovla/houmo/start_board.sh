#!/system/bin/sh
set -eu
runtime_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
export PYTHONPATH="$runtime_dir${PYTHONPATH:+:$PYTHONPATH}"
# The board Houmo runtime (tcim_lite python package + shared libraries) is a
# documented board prerequisite at /data/local/houmo (see README); the packed
# runtime vendors FastAPI/Starlette/Uvicorn wheels instead.
export TCIM_PYTHON_PATH="${TCIM_PYTHON_PATH:-/data/local/houmo/python}"
export LD_LIBRARY_PATH="${TCIM_PYTHON_PATH:+$TCIM_PYTHON_PATH/../../lib:}/data/local/houmo/lib:/sys_prod/robot/out/lib:/sys_prod/robot/install/lib:/usr/lib:/lib"
export AEROVLA_INFER_DIR="${AEROVLA_INFER_DIR:-$runtime_dir/aerovla_infer}"
cd "$runtime_dir"
exec /sys_prod/robot/out/bin/python3 -m uvicorn serve:app \
  --host "${INFERENCE_HOST:-127.0.0.1}" \
  --port "${INFERENCE_PORT:-8090}" \
  --workers 1 --limit-concurrency 16 --h11-max-incomplete-event-size 16777216
