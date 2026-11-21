#!/usr/bin/env bash
set -euo pipefail
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check-cuda ) ]]; then
    printf 'Usage: %s [--check-cuda]\n' "$0" >&2
    exit 2
fi
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
upstream="$(realpath -e "${YOPO_TRACKER_UPSTREAM:?Set YOPO_TRACKER_UPSTREAM to the locked Tracker checkout}")"
cd "$model_dir"
revision=d313f8610195dc986caceff7a3616f63d7e6f31f
[[ "$(git -C "$upstream" rev-parse HEAD)" == "$revision" ]]
git -C "$upstream" diff --quiet "$revision" -- YOPO
source_weight="$upstream/YOPO/saved/YOPO_1/epoch50.pth"
read -r digest _ < weights.sha256
printf '%s  %s\n' "$digest" "$source_weight" | sha256sum -c -
mkdir -p artifacts
if [[ ! -e artifacts/epoch50.pth ]]; then
    cp "$source_weight" artifacts/epoch50.pth
fi
printf '%s  artifacts/epoch50.pth\n' "$digest" | sha256sum -c -
export UV_PROJECT_ENVIRONMENT="$model_dir/.venv"
uv sync --locked
YOPO_TRACKER_UPSTREAM="$upstream" YOPO_TRACKER_WEIGHTS="$model_dir/artifacts/epoch50.pth" \
    .venv/bin/python -B -c 'from serve import Predictor; print(Predictor().metadata)'
