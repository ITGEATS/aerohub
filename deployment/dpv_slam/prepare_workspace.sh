#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image=${DPV_SLAM_IMAGE:-aerohub/dpv-slam:ros2-humble}
base_image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.1}
artifact_root=${DPV_SLAM_ARTIFACT_ROOT:-$root/log/workspaces/dpv_slam}
wheelhouse=$artifact_root/wheelhouse
cuda_packages=$artifact_root/cuda-packages
for directory in "$wheelhouse" "$cuda_packages"; do
  test -d "$directory" || { echo "DPV-SLAM build artifacts missing: $directory" >&2; exit 2; }
done
args=(--network host --file "$root/deployment/dpv_slam/Dockerfile"
      --build-context "dpv_wheels=$wheelhouse"
      --build-context "dpv_cuda_packages=$cuda_packages")
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n ${!variable:-} ]]; then args+=(--build-arg "$variable=${!variable}"); fi
done
docker image inspect "$base_image" >/dev/null
docker build "${args[@]}" --build-arg "BASE_IMAGE=$base_image" \
  --tag "$image" "$root"
docker run --rm --gpus all --entrypoint bash "$image" -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/dpv_ws/install/setup.bash
  cuda_wheel_libs=$(find /opt/dpv/python/nvidia -type d -name lib -print | paste -sd:)
  export LD_LIBRARY_PATH="$cuda_wheel_libs:/opt/dpv/python/torch/lib:/opt/dpv/cuda/lib:/opt/dpv/cuda/lib64:${LD_LIBRARY_PATH:-}"
  ros2 pkg prefix uav_dpv_slam_ros2
  ros2 pkg prefix uav_dpv_slam
  python3 -c "import cuda_corr, cuda_ba, lietorch_backends, torch; assert torch.cuda.is_available()"
  test ! -e /opt/ros/noetic
  test ! -e /opt/dpv_ws/devel
'
docker image inspect "$image" --format '{{.Id}}'
