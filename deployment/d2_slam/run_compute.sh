#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="${D2_SLAM_WORKSPACE:-${repo_root}/log/workspaces/d2_slam}"
image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}"
vehicle=""
vehicle_index=""
main_id="1"
output=""
start_as2_plugin=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --vehicle) vehicle=$2; shift 2 ;;
    --vehicle-index) vehicle_index=$2; shift 2 ;;
    --main-id) main_id=$2; shift 2 ;;
    --output) output=$2; shift 2 ;;
    --without-as2-plugin) start_as2_plugin=0; shift ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ -z "${vehicle}" || ! "${vehicle_index}" =~ ^[0-9]+$ || ! "${main_id}" =~ ^[0-9]+$ || -z "${output}" ]]; then
  printf 'vehicle, numeric vehicle-index/main-id, and output are required\n' >&2
  exit 2
fi
for required in \
  "${workspace}/ros1/bin/roscore" \
  "${workspace}/catkin_ws/devel/.private/d2vins/lib/d2vins/d2vins_node" \
  "${workspace}/ros2/install/setup.bash" \
  "${workspace}/onnxruntime-linux-x64-gpu-1.18.1/lib/libonnxruntime.so"; do
  if [[ ! -e "${required}" ]]; then
    printf 'D2SLAM workspace artifact is missing: %s\n' "${required}" >&2
    exit 2
  fi
done
for container_link in \
  "${workspace}/catkin_ws/devel/setup.bash" \
  "${workspace}/catkin_ws/devel/lib/d2vins/d2vins_node"; do
  if [[ ! -e "${container_link}" && ! -L "${container_link}" ]]; then
    printf 'D2SLAM container-relative catkin entry is missing: %s\n' "${container_link}" >&2
    exit 2
  fi
done

mkdir -p "${output}"
container="aerohub-d2-slam-${vehicle}"
master_port=$((12111 + vehicle_index))
docker_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "${docker_pid}" ]]; then
    docker stop --timeout 8 "${container}" >/dev/null 2>&1 || true
    if kill -0 "${docker_pid}" 2>/dev/null; then
      kill -TERM "${docker_pid}" 2>/dev/null || true
    fi
    wait "${docker_pid}" 2>/dev/null || true
  fi
  exit "${status}"
}

terminate() {
  exit 143
}

trap cleanup EXIT
trap terminate INT TERM

docker run --rm --init --network host --ipc host --gpus all \
  --name "${container}" \
  --label aerohub.task=d2-slam \
  --label "aerohub.vehicle=${vehicle}" \
  --shm-size 2g \
  --env "D2_VEHICLE=${vehicle}" \
  --env "D2_VEHICLE_INDEX=${vehicle_index}" \
  --env "D2_MAIN_ID=${main_id}" \
  --env "D2_START_AS2_PLUGIN=${start_as2_plugin}" \
  --env "D2SLAM_LOG_LEVEL=${D2SLAM_LOG_LEVEL:-info}" \
  --env "ROS_MASTER_URI=http://127.0.0.1:${master_port}" \
  --env ROS_DOMAIN_ID=92 \
  --env ROS_LOCALHOST_ONLY=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --volume "${repo_root}:/repo:ro" \
  --volume "${repo_root}:${repo_root}:ro" \
  --volume "${workspace}:/workspace:ro" \
  --volume "${workspace}:${workspace}:ro" \
  --volume "${output}:/runtime" \
  --entrypoint /repo/deployment/d2_slam/compute_entrypoint.sh \
  "${image}" &
docker_pid=$!

set +e
wait "${docker_pid}"
status=$?
set -e
exit "${status}"
