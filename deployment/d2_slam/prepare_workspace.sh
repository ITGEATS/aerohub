#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="${D2_SLAM_WORKSPACE:-${repo_root}/log/workspaces/d2_slam}"
image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}"
expected_image_id="sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9"
jobs="${D2_SLAM_BUILD_JOBS:-4}"

micromamba_url="https://micro.mamba.pm/api/micromamba/linux-64/2.10.0"
micromamba_sha256="9a97976244929bbae28fb1fae504e74a3e7e6d1932870e3a03321f01b7fbd07b"
lcm_url="https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.tar.gz"
lcm_sha256="149d7076369d261e6adbb25d713dc9e30aeba415f4fc62bb41e748b2eb229b46"
onnx_url="https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-gpu-1.18.1.tgz"
onnx_sha256="d947af0e4311fd38012ad69dea4983e73ce5f1754da0d5b7a118603dd87b197d"

d2_slam_commit="f5ea552bf1198de1332c78a3915948cd350546ad"
ceres_commit="488a28fa3247844977b64884a9f7d0daa8ab933d"
opengv_commit="5dda2a3dc005554fb31d7e619fbbb862d6768b1b"
swarm_msgs_commit="7dfbe609e8150f8bda5ac1a5a57b3a929f0a499b"

if [[ "${1:-}" == "--inside" ]]; then
  shift
  workspace=/workspace
  repo_root=/repo

  fetch_checked() {
    local url=$1
    local output=$2
    local expected=$3
    mkdir -p "$(dirname "${output}")"
    if [[ ! -f "${output}" ]] || ! echo "${expected}  ${output}" | sha256sum --check --status; then
      rm -f "${output}.partial"
      curl --fail --location --retry 5 --retry-delay 2 \
        --output "${output}.partial" "${url}"
      echo "${expected}  ${output}.partial" | sha256sum --check --status
      mv "${output}.partial" "${output}"
    fi
  }

  mkdir -p "${workspace}"/{bin,build,cache,downloads,install,mamba-root}
  fetch_checked "${micromamba_url}" "${workspace}/cache/micromamba.tar.bz2" "${micromamba_sha256}"
  if [[ ! -x "${workspace}/bin/micromamba" ]]; then
    tar -xjf "${workspace}/cache/micromamba.tar.bz2" -C "${workspace}" bin/micromamba
  fi
  [[ "$("${workspace}/bin/micromamba" --version)" == "2.10.0" ]]

  export MAMBA_ROOT_PREFIX="${workspace}/mamba-root"
  if [[ ! -x "${workspace}/ros1/bin/roscore" ]]; then
    "${workspace}/bin/micromamba" create -y -p "${workspace}/ros1" \
      --file "${repo_root}/deployment/d2_slam/locks/ros1-linux-64.lock"
  fi
  if [[ ! -e "${workspace}/cuda11/lib/libcudnn.so" ]]; then
    "${workspace}/bin/micromamba" create -y -p "${workspace}/cuda11" \
      --file "${repo_root}/deployment/d2_slam/locks/cuda11-linux-64.lock"
  fi

  fetch_checked "${onnx_url}" \
    "${workspace}/downloads/onnxruntime-linux-x64-gpu-1.18.1.tgz" "${onnx_sha256}"
  if [[ ! -e "${workspace}/onnxruntime-linux-x64-gpu-1.18.1/lib/libonnxruntime.so" ]]; then
    tar -xzf "${workspace}/downloads/onnxruntime-linux-x64-gpu-1.18.1.tgz" \
      -C "${workspace}"
  fi

  fetch_checked "${lcm_url}" "${workspace}/downloads/lcm-1.4.0.tar.gz" "${lcm_sha256}"
  if [[ ! -e "${workspace}/install/lib/lcm/cmake/lcmConfig.cmake" ]]; then
    rm -rf "${workspace}/build/lcm-1.4.0"
    tar -xzf "${workspace}/downloads/lcm-1.4.0.tar.gz" -C "${workspace}/build"
    "${workspace}/ros1/bin/cmake" \
      -S "${workspace}/build/lcm-1.4.0" -B "${workspace}/build/lcm-build" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${workspace}/install" \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DLCM_ENABLE_TESTS=OFF -DLCM_ENABLE_EXAMPLES=OFF \
      -DLCM_ENABLE_JAVA=OFF -DLCM_ENABLE_PYTHON=OFF
    "${workspace}/ros1/bin/cmake" --build "${workspace}/build/lcm-build" --parallel "${jobs}"
    "${workspace}/ros1/bin/cmake" --install "${workspace}/build/lcm-build"
  fi

  if [[ ! -e "${workspace}/install/lib/cmake/Ceres/CeresConfig.cmake" ]]; then
    "${workspace}/ros1/bin/cmake" \
      -S "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/ceres_solver" \
      -B "${workspace}/build/ceres" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_INSTALL_PREFIX="${workspace}/install" -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF -DMINIGLOG=OFF -DCUDA=OFF
    "${workspace}/ros1/bin/cmake" --build "${workspace}/build/ceres" --parallel "${jobs}"
    "${workspace}/ros1/bin/cmake" --install "${workspace}/build/ceres"
  fi

  if [[ ! -e "${workspace}/install/lib/cmake/opengv-1.0/opengvConfig.cmake" ]]; then
    "${workspace}/ros1/bin/cmake" \
      -S "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/opengv" \
      -B "${workspace}/build/opengv" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_INSTALL_PREFIX="${workspace}/install" -DBUILD_TESTS=OFF
    "${workspace}/ros1/bin/cmake" --build "${workspace}/build/opengv" --parallel "${jobs}"
    "${workspace}/ros1/bin/cmake" --install "${workspace}/build/opengv"
  fi

  mkdir -p "${workspace}/catkin_ws/src"
  ln -sfn "${repo_root}/src/basic_function/state_estimator/d2_slam/upstream" \
    "${workspace}/catkin_ws/src/D2SLAM"
  for package in swarm_msgs swarmcomm_msgs swarmtal_msgs; do
    ln -sfn "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs/${package}" \
      "${workspace}/catkin_ws/src/${package}"
  done

  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH="${workspace}/ros1/bin:${PATH}"
  source "${workspace}/ros1/setup.bash"
  export LD_LIBRARY_PATH="${workspace}/ros1/lib:${workspace}/cuda11/lib:${workspace}/onnxruntime-linux-x64-gpu-1.18.1/lib:${workspace}/install/lib:${LD_LIBRARY_PATH:-}"
  catkin_metadata="${workspace}/catkin_ws/build/.catkin_tools.yaml"
  if [[ -f "${catkin_metadata}" ]] && \
      ! grep -Fxq "workspace: ${workspace}/catkin_ws" "${catkin_metadata}"; then
    rm -rf "${workspace}/catkin_ws/build" \
      "${workspace}/catkin_ws/devel" "${workspace}/catkin_ws/logs"
  fi
  "${workspace}/ros1/bin/catkin" config --workspace "${workspace}/catkin_ws" \
    --extend "${workspace}/ros1" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
      -DCMAKE_INCLUDE_PATH="${workspace}/install/include" \
      -DCMAKE_LIBRARY_PATH="${workspace}/install/lib" \
      -DCeres_DIR="${workspace}/install/lib/cmake/Ceres" \
      -Dopengv_DIR="${workspace}/install/lib/cmake/opengv-1.0" \
      -Dlcm_DIR="${workspace}/install/lib/lcm/cmake" \
      -DONNXRUNTIME_LIB_DIR="${workspace}/onnxruntime-linux-x64-gpu-1.18.1/lib" \
      -DONNXRUNTIME_INC_DIR="${workspace}/onnxruntime-linux-x64-gpu-1.18.1/include" \
      -DUSE_CUDA=OFF \
      -DCMAKE_EXE_LINKER_FLAGS="-L${workspace}/ros1/lib" \
      -DCMAKE_SHARED_LINKER_FLAGS="-L${workspace}/ros1/lib" \
      -DCMAKE_MODULE_LINKER_FLAGS="-L${workspace}/ros1/lib"
  "${workspace}/ros1/bin/catkin" build --workspace "${workspace}/catkin_ws" \
    --no-status --summarize --jobs "${jobs}" camera_models swarmtal_msgs
  "${workspace}/ros1/bin/catkin" build --workspace "${workspace}/catkin_ws" \
    --no-status --summarize --jobs "${jobs}" swarm_msgs swarmcomm_msgs
  "${workspace}/ros1/bin/catkin" build --workspace "${workspace}/catkin_ws" \
    --no-status --summarize --jobs "${jobs}" d2common
  "${workspace}/ros1/bin/catkin" build --workspace "${workspace}/catkin_ws" \
    --no-status --summarize --jobs "${jobs}" d2comm d2frontend d2pgo
  "${workspace}/ros1/bin/catkin" build --workspace "${workspace}/catkin_ws" \
    --no-status --summarize --jobs "${jobs}" d2vins

  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
  unset LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  ros2_cache="${workspace}/ros2/build/uav_d2_slam/CMakeCache.txt"
  if [[ -f "${ros2_cache}" ]] && \
      ! grep -Fxq "CMAKE_CACHEFILE_DIR:INTERNAL=${workspace}/ros2/build/uav_d2_slam" \
        "${ros2_cache}"; then
    rm -rf "${workspace}/ros2/build" \
      "${workspace}/ros2/install" "${workspace}/ros2/log"
  fi
  colcon --log-base "${workspace}/ros2/log" build \
    --base-paths "${repo_root}/src/basic_function/state_estimator/d2_slam/ros2_adapter" \
      "${repo_root}/src/basic_function/state_estimator/d2_slam/plugin" \
      "${repo_root}/simulator/airsim/sensors" \
    --build-base "${workspace}/ros2/build" --install-base "${workspace}/ros2/install" \
    --merge-install --parallel-workers "${jobs}" \
    --packages-select uav_as2_sensor_adapter uav_d2_slam_ros2 uav_d2_slam \
    --cmake-clean-cache
  exit 0
fi

if [[ ! "${jobs}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'D2_SLAM_BUILD_JOBS must be a positive integer\n' >&2
  exit 2
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  printf 'This lock set currently supports x86_64 only\n' >&2
  exit 2
fi
for tool in docker git; do
  command -v "${tool}" >/dev/null || { printf 'Missing required tool: %s\n' "${tool}" >&2; exit 2; }
done

require_commit() {
  local path=$1
  local expected=$2
  local actual
  actual=$(git -C "${path}" rev-parse HEAD)
  if [[ "${actual}" != "${expected}" ]]; then
    printf 'Unexpected source commit for %s: %s (expected %s)\n' \
      "${path}" "${actual}" "${expected}" >&2
    exit 2
  fi
}
require_commit "${repo_root}/src/basic_function/state_estimator/d2_slam/upstream" "${d2_slam_commit}"
if [[ -n "$(git -C "${repo_root}/src/basic_function/state_estimator/d2_slam/upstream" status --porcelain)" ]]; then
  printf 'D2SLAM Fork submodule must be clean\n' >&2
  exit 2
fi
require_commit "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/ceres_solver" "${ceres_commit}"
require_commit "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/opengv" "${opengv_commit}"
require_commit "${repo_root}/src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs" "${swarm_msgs_commit}"

actual_image_id=$(docker image inspect "${image}" --format '{{.Id}}')
if [[ "${actual_image_id}" != "${expected_image_id}" ]]; then
  printf 'Unexpected onboard image ID: %s (expected %s)\n' \
    "${actual_image_id}" "${expected_image_id}" >&2
  exit 2
fi

mkdir -p "${workspace}"
container="aerohub-d2-prepare-$$"
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
terminate() { exit 143; }
trap cleanup EXIT
trap terminate INT TERM

proxy_args=()
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n "${!variable:-}" ]]; then
    proxy_args+=(--env "${variable}=${!variable}")
  fi
done

docker run --rm --init --network host --name "${container}" \
  --label aerohub.task=d2-slam-prepare \
  --volume "${repo_root}:/repo:ro" \
  --volume "${workspace}:/workspace" --volume "${workspace}:${workspace}" \
  "${proxy_args[@]}" --entrypoint /repo/deployment/d2_slam/prepare_workspace.sh \
  "${image}" --inside &
docker_pid=$!
set +e
wait "${docker_pid}"
status=$?
set -e
exit "${status}"
