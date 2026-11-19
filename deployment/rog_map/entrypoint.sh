#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/rog_map_ws/install/setup.bash

namespace=${ROG_MAP_NAMESPACE:-drone0}
exec ros2 launch uav_rog_map rog_map.launch.py \
  namespace:="${namespace}" \
  global_frame:="${namespace}/odom" \
  sensor_frame:="${namespace}/Lidar"
