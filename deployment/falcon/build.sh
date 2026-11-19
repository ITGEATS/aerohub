#!/usr/bin/env bash
set -euo pipefail

FALCON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FALCON_SOURCE="$FALCON_ROOT/src/tool/navigation/planner/falcon/upstream"
FALCON_REVISION=b447c1b660ab2305c023a38121fb4feeea44e47b
FALCON_IMAGE=${FALCON_IMAGE:-aerohub/falcon:ros2-humble}
FALCON_BASE_IMAGE=${FALCON_BASE_IMAGE:-onboard_computer:v0.1.1}
FALCON_WORKSTATION_IMAGE=${FALCON_WORKSTATION_IMAGE:-aerohub/falcon-workstation:ros2-humble}
FALCON_WORKSTATION_BASE_IMAGE=${FALCON_WORKSTATION_BASE_IMAGE:-osrf/ros:humble-desktop-full-jammy}

test "$(git -C "$FALCON_SOURCE" rev-parse HEAD)" = "$FALCON_REVISION" || {
  echo "FALCON submodule is not at the locked Fork revision" >&2
  exit 1
}
test -z "$(git -C "$FALCON_SOURCE" status --porcelain)" || {
  echo "FALCON submodule must be clean before a reproducible build" >&2
  exit 1
}

proxy_args=()
for name in HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy; do
  if [[ -n "${!name:-}" ]]; then
    proxy_args+=(--build-arg "$name=${!name}")
  fi
done

FALCON_SOURCE_DIGEST=$(python3 \
  "$FALCON_ROOT/deployment/falcon/source_manifest.py" --root "$FALCON_ROOT")

docker build \
  --network host \
  "${proxy_args[@]}" \
  --build-arg "BASE_IMAGE=$FALCON_BASE_IMAGE" \
  --build-arg "FALCON_BUILD_JOBS=${FALCON_BUILD_JOBS:-2}" \
  --build-arg "FALCON_SOURCE_DIGEST=$FALCON_SOURCE_DIGEST" \
  --build-arg "FALCON_REVISION=$FALCON_REVISION" \
  --tag "$FALCON_IMAGE" \
  --file "$FALCON_ROOT/deployment/falcon/Dockerfile" \
  "$FALCON_ROOT"

docker build \
  --network host \
  "${proxy_args[@]}" \
  --build-arg "ONBOARD_SOURCE_IMAGE=$FALCON_BASE_IMAGE" \
  --build-arg "WORKSTATION_BASE_IMAGE=$FALCON_WORKSTATION_BASE_IMAGE" \
  --build-arg "FALCON_SOURCE_DIGEST=$FALCON_SOURCE_DIGEST" \
  --build-arg "FALCON_REVISION=$FALCON_REVISION" \
  --tag "$FALCON_WORKSTATION_IMAGE" \
  --file "$FALCON_ROOT/deployment/falcon/Dockerfile.workstation" \
  "$FALCON_ROOT"

for image in "$FALCON_IMAGE" "$FALCON_WORKSTATION_IMAGE"; do
  docker image inspect "$image" --format '{{.Id}} {{index .Config.Labels "io.aerohub.source.digest"}}'
done
