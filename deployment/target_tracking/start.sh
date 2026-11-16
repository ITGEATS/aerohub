#!/usr/bin/env bash
# Run inside the fixed compute container with /opt/ros/humble sourced.
set -e
repo=$(cd "$(dirname "$0")/../.." && pwd)
workspace=${TRACKING_WORKSPACE:-"$repo/log/yopov2-tracking/build"}
source "$workspace/install/setup.bash"
exec ros2 run uav_target_tracking target_tracking --ros-args \
 -r "__ns:=/${UAV_NAMESPACE:-drone0}" --params-file "$repo/deployment/target_tracking/params.yaml" "$@"
