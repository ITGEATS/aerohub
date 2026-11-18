#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image=${ROG_MAP_IMAGE:-aerohub/rog-map:ros2-humble}
base_image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.1}
args=(--network host --file "$root/deployment/rog_map/Dockerfile")
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n ${!variable:-} ]]; then args+=(--build-arg "$variable=${!variable}"); fi
done

docker image inspect "$base_image" >/dev/null
docker build "${args[@]}" --build-arg "BASE_IMAGE=$base_image" \
  --tag "$image" "$root"
docker run --rm --entrypoint bash "$image" -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/rog_map_ws/install/setup.bash
  ros2 pkg prefix rog_map
  ros2 pkg prefix uav_rog_map_interfaces
  ros2 pkg prefix uav_rog_map
  ros2 pkg prefix as2_navigation_inputs
  test ! -e /opt/ros/noetic
  test ! -e /opt/rog_map_ws/devel
'
docker image inspect "$image" --format '{{.Id}}'
