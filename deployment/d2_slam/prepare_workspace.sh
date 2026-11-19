#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${D2_SLAM_IMAGE:-aerohub/d2-slam:ros2-humble-native}"
base_image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.2}"
jobs="${D2_SLAM_BUILD_JOBS:-1}"

if [[ ! "${jobs}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'D2_SLAM_BUILD_JOBS must be a positive integer\n' >&2
  exit 2
fi

docker image inspect "${base_image}" >/dev/null
proxy_args=()
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n "${!variable:-}" ]]; then
    proxy_args+=(--build-arg "${variable}=${!variable}")
  fi
done
docker build --network host --file "${repo_root}/deployment/d2_slam/Dockerfile" \
  --build-arg "BASE_IMAGE=${base_image}" \
  --build-arg "D2_BUILD_JOBS=${jobs}" \
  "${proxy_args[@]}" \
  --tag "${image}" "${repo_root}"

docker run --rm --entrypoint bash "${image}" -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/d2_slam_ws/install/setup.bash
  for package in d2vins d2pgo d2comm uav_d2_slam_ros2 uav_d2_slam; do
    ros2 pkg prefix "${package}"
  done
  test ! -e /opt/ros/noetic
  test ! -e /opt/d2_slam_ws/devel
  ! grep -RIlE "/home/|/repo|/workspace|catkin_ws" \
    /opt/d2_slam_ws/install /opt/d2_slam/config
'

docker image inspect "${image}" --format '{{.Id}}'
