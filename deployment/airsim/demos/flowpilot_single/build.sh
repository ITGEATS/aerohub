#!/usr/bin/env bash
# Optional development overlay. Delivery builds use deployment/inference/flowpilot/Dockerfile.
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
work="${FLOWPILOT_ROS_WORKSPACE:-$repo/src/tool/navigation/planner/flowpilot/workspace}"
[[ -n "${ROS_DISTRO:-}" ]] || { echo 'Source matching Humble and AS2 first' >&2; exit 1; }
mkdir -p "$work"
cd "$work"
exec colcon build --base-paths "$repo/src/tool/navigation/interfaces" "$repo/src/tool/navigation/python_api" "$repo/src/basic_function/inference/interfaces" "$repo/src/basic_function/inference/ros2_adapter" "$repo/src/tool/navigation/planner/flowpilot/ros2_adapter" --packages-select uav_planner_interfaces uav_planner_python_api uav_inference_interfaces uav_inference uav_flowpilot_planner --event-handlers console_direct+
