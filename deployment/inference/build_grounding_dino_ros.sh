#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export INFERENCE_ROS_WORKSPACE="${INFERENCE_ROS_WORKSPACE:-$repo_dir/log/grounding-dino-inference/ros_ws}"
exec bash "$repo_dir/deployment/inference/build_ros.sh"
