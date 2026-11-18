#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image=${FAST_LIVO2_IMAGE:-aerohub/fast-livo2:ros2-humble}
base_image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.1}
args=(--network host --file "$root/deployment/fast_livo2/Dockerfile")
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n ${!variable:-} ]]; then args+=(--build-arg "$variable=${!variable}"); fi
done

docker image inspect "$base_image" >/dev/null
docker build "${args[@]}" --build-arg "BASE_IMAGE=$base_image" \
  --tag "$image" "$root"
docker run --rm --entrypoint bash "$image" -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/fast_livo2_ws/install/setup.bash
  ros2 pkg prefix fast_livo
  ros2 pkg prefix uav_fast_livo2_ros2
  ros2 pkg prefix uav_fast_livo2
  test ! -e /opt/ros/noetic
  test ! -e /opt/fast_livo2_ws/devel
'
docker image inspect "$image" --format '{{.Id}}'
