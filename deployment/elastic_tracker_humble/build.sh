#!/usr/bin/env bash
# Build a matching ROS 2 Humble overlay in the onboard tool environment.
set -eo pipefail
repo=$(cd "${1:-$(dirname "${BASH_SOURCE[0]}")/../..}" && pwd)
workspace=${2:-"$repo/log/elastic-tracker/build"}
source /root/aerostack2_ws/install/setup.bash
mkdir -p "$workspace"
cd "$workspace"
component="$repo/src/tool/navigation/planner/elastic_tracker"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
colcon build --executor sequential --base-paths \
  "$component/ros2_adapter" \
  "$repo/src/basic_function/inference/interfaces" "$repo/src/basic_function/inference/ros2_adapter" \
  "$repo/src/tool/navigation/interfaces" "$repo/src/tool/object_detection" "$repo/src/tool/target_tracking" \
  "$repo/src/tool/system/motion_resource" \
  "$repo/src/behavior/follow_path" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
