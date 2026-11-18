#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/pin_ws/install/setup.bash
mkdir -p /runtime/output

exec /opt/pin_venv/bin/python \
  /opt/pin_ws/install/lib/uav_pin_slam_ros2/pin_slam_node \
  --ros-args -r "__ns:=/${PIN_SLAM_NAMESPACE}" \
  --params-file /opt/pin_slam/config/algorithm.yaml \
  -p "use_sim_time:=${PIN_SLAM_USE_SIM_TIME}"
