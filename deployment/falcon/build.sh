#!/usr/bin/env bash
set -euo pipefail

FALCON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FALCON_SOURCE="$FALCON_ROOT/src/tool/navigation/planner/falcon/upstream"
FALCON_BASELINE=312eb4d32c6c7af1a482f94a0a204aa2bb150cca
FALCON_IMAGE=onboard_computer:v0.1.0
FALCON_WORKSTATION_IMAGE=osrf/ros:humble-desktop-full-jammy

test "$(git -C "$FALCON_SOURCE" rev-parse HEAD)" != "$FALCON_BASELINE" || {
  echo "FALCON Fork adaptation commit has not been created yet" >&2
  exit 1
}
test -z "$(git -C "$FALCON_SOURCE" status --porcelain)" || {
  echo "FALCON submodule must be clean before a reproducible build" >&2
  exit 1
}

mkdir -p "$FALCON_ROOT/log/falcon"
proxy_args=()
for name in HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy; do
  if [[ -n "${!name:-}" ]]; then proxy_args+=(--env "$name=${!name}"); fi
done
docker run --rm --init --network host "${proxy_args[@]}" \
  --name aerohub-falcon-build \
  --entrypoint bash \
  -v "$FALCON_ROOT:/workspace" \
  "$FALCON_IMAGE" -lc '
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  libnlopt-cxx-dev=2.7.1-3build1 \
  libgoogle-glog-dev=0.5.0+really0.4.0-2 \
  libdw-dev=0.186-1ubuntu0.1
mkdir -p /workspace/log/falcon/deps
packages="libnlopt-cxx-dev libnlopt-dev libnlopt-cxx0 libnlopt0 libgoogle-glog-dev libgoogle-glog0v5 libgflags-dev libgflags2.2 libunwind-dev libunwind8 libdw-dev libdw1 libelf-dev libelf1"
cd /workspace/log/falcon/deps
find . -maxdepth 1 -type f -name "*.deb" -delete
for package in $packages; do
  version=$(dpkg-query -W "$package" | cut -f2)
  apt-get download "$package=$version"
done
sha256sum ./*.deb | sed '\''s#  \./#  #'\'' | sort -k2 > /workspace/log/falcon/dependencies.resolved.sha256
diff -u /workspace/deployment/falcon/dependencies.sha256 /workspace/log/falcon/dependencies.resolved.sha256
set +u
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
set -u
cd /workspace
colcon --log-base log/falcon/colcon build \
  --base-paths \
    as2/aerostack2/as2_msgs \
    src/tool/navigation/interfaces \
    src/behavior/follow_path \
    src/tool/navigation/planner/falcon/upstream/falcon_planner/falcon_ros2 \
    src/tool/navigation/planner/falcon/ros2_adapter \
  --build-base log/falcon/build \
  --install-base log/falcon/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon --log-base log/falcon/test-log test \
  --build-base log/falcon/build \
  --install-base log/falcon/install \
  --packages-select uav_falcon_adapter
colcon test-result --test-result-base log/falcon/build --verbose
'

docker run --rm --init --network none \
  --name aerohub-falcon-workstation-build \
  --entrypoint bash \
  -v "$FALCON_ROOT:/workspace" \
  "$FALCON_WORKSTATION_IMAGE" -lc '
set -eo pipefail
python3 /workspace/deployment/falcon/prepare_workstation.py \
  --source /workspace/as2/aerostack2/as2_msgs \
  --output /workspace/log/falcon/workstation
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base log/falcon/workstation/log build \
  --base-paths log/falcon/workstation/src/as2_msgs \
  --build-base log/falcon/workstation/build \
  --install-base log/falcon/workstation/install \
  --cmake-args -DBUILD_TESTING=OFF
'
