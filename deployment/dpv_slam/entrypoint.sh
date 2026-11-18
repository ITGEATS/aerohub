#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/dpv_ws/install/setup.bash
cuda_wheel_libs=$(find /opt/dpv/python/nvidia -type d -name lib -print | paste -sd:)
export LD_LIBRARY_PATH="${cuda_wheel_libs}:/opt/dpv/python/torch/lib:/opt/dpv/cuda/lib:/opt/dpv/cuda/lib64:${LD_LIBRARY_PATH:-}"
mkdir -p /runtime/output
exec ros2 run uav_dpv_slam_ros2 dpv_slam_node --ros-args \
  -r "__ns:=/${DPV_SLAM_NAMESPACE}" \
  --params-file /opt/dpv/config/algorithm.yaml \
  -p "use_sim_time:=${DPV_SLAM_USE_SIM_TIME}"
