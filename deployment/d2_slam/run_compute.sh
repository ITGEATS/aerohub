#!/usr/bin/env bash
set -euo pipefail

image="${D2_SLAM_IMAGE:-aerohub/d2-slam:ros2-humble-native}"
vehicle=""
vehicle_index=""
main_id="1"
output=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --vehicle) vehicle=$2; shift 2 ;;
    --vehicle-index) vehicle_index=$2; shift 2 ;;
    --main-id) main_id=$2; shift 2 ;;
    --output) output=$2; shift 2 ;;
    --without-as2-plugin) shift ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ -z "${vehicle}" || ! "${vehicle_index}" =~ ^[0-9]+$ ||
      ! "${main_id}" =~ ^[0-9]+$ || -z "${output}" ]]; then
  printf 'vehicle, numeric vehicle-index/main-id, and output are required\n' >&2
  exit 2
fi

docker image inspect "${image}" >/dev/null
mkdir -p "${output}"
container="aerohub-d2-slam-${vehicle}"
docker_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "${docker_pid}" ]]; then
    docker stop --timeout 8 "${container}" >/dev/null 2>&1 || true
    wait "${docker_pid}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 143' INT TERM

docker run --rm --init --network host --ipc host \
  --name "${container}" \
  --label aerohub.task=d2-slam \
  --label "aerohub.vehicle=${vehicle}" \
  --shm-size 2g \
  --env "D2_SLAM_NAMESPACE=${vehicle}" \
  --env "D2_SLAM_SELF_ID=${vehicle_index}" \
  --env "D2_SLAM_MAIN_ID=${main_id}" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-92}" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --volume "${output}:/runtime/output" \
  "${image}" &
docker_pid=$!

set +e
wait "${docker_pid}"
status=$?
set -e
exit "${status}"
