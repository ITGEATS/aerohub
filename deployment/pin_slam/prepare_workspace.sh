#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image=${PIN_SLAM_IMAGE:-aerohub/pin-slam:ros2-humble}
base_image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.1}
wheelhouse=${PIN_SLAM_WHEELHOUSE:-$root/log/workspaces/pin_slam/wheelhouse}

test -d "$wheelhouse" || { echo "PIN-SLAM wheelhouse is missing: $wheelhouse" >&2; exit 2; }
args=(--network host --file "$root/deployment/pin_slam/Dockerfile"
      --build-context "pin_wheels=$wheelhouse")
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n ${!variable:-} ]]; then args+=(--build-arg "$variable=${!variable}"); fi
done
docker image inspect "$base_image" >/dev/null
docker build "${args[@]}" --build-arg "BASE_IMAGE=$base_image" \
  --tag "$image" "$root"
docker run --rm --gpus all --entrypoint bash "$image" -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/pin_ws/install/setup.bash
  ros2 pkg prefix uav_pin_slam_ros2
  ros2 pkg prefix uav_pin_slam
  /opt/pin_venv/bin/python -c "import torch; assert torch.cuda.is_available()"
  test ! -e /opt/ros/noetic
  test ! -e /opt/pin_ws/devel
'
docker image inspect "$image" --format '{{.Id}}'
