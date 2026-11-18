#!/usr/bin/env bash
set -euo pipefail

image=${ROG_MAP_IMAGE:-aerohub/rog-map:ros2-humble}
namespace=${ROG_MAP_NAMESPACE:-drone0}
domain=${ROS_DOMAIN_ID:-0}
container=${ROG_MAP_CONTAINER:-aerohub-rog-map-$namespace}

docker image inspect "$image" >/dev/null
exec docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=rog-map-runtime \
  --label "aerohub.namespace=$namespace" \
  --env "ROS_DOMAIN_ID=$domain" \
  --env "ROG_MAP_NAMESPACE=$namespace" \
  "$image"
