#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/d2_slam_ws/install/setup.bash
mkdir -p /runtime/output

exec ros2 launch uav_d2_slam_ros2 d2_slam.launch.py \
  namespace:="${D2_SLAM_NAMESPACE}" \
  self_id:="${D2_SLAM_SELF_ID}" \
  main_id:="${D2_SLAM_MAIN_ID}" \
  config:="${D2_SLAM_CONFIG}" \
  lcm_uri:="${D2_SLAM_LCM_URI}"
