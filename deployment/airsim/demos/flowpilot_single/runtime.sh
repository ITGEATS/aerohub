#!/usr/bin/env bash
# Environments are prepared in the image entrypoint, never mounted host overlays.
set -eo pipefail
cd /repo/src/basic_function/inference/models/flowpilot
env PYTHONPATH=/repo/src/basic_function/inference/runtime /opt/flowpilot/model-venv/bin/python -B -m uvicorn serve:app --host 127.0.0.1 --port 18082 &
worker=$!
supervisor=
cleanup() {
  trap - EXIT INT TERM
  for child in "$supervisor" "$worker"; do
    [[ -n "$child" ]] || continue
    kill -INT "$child" 2>/dev/null || true
    for attempt in $(seq 1 50); do kill -0 "$child" 2>/dev/null || break; sleep .2; done
    if kill -0 "$child" 2>/dev/null; then kill -TERM "$child" 2>/dev/null || true; sleep 2; fi
    if kill -0 "$child" 2>/dev/null; then kill -KILL "$child" 2>/dev/null || true; fi
    wait "$child" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for attempt in $(seq 1 180); do
  kill -0 "$worker" || exit 1
  if python3 -c "import urllib.request; urllib.request.build_opener(urllib.request.ProxyHandler({})).open('http://127.0.0.1:18082/health/ready',timeout=2)" 2>/dev/null; then
    sleep .2
    kill -0 "$worker" || exit 1
    cd /repo
    python3 deployment/airsim/demos/flowpilot_single/compute.py &
    supervisor=$!
    wait "$supervisor"
    exit $?
  fi
  sleep 2
done
echo 'worker readiness timeout' >&2
exit 1
