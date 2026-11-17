#!/usr/bin/env bash
set -euo pipefail
: "${AEROHUB_MOTION_LOCK_DIR:?Set a persistent shared motion directory for this vehicle}"
exec ros2 run uav_system_tool system_tool --ros-args -r __ns:="/${UAV_NAMESPACE:-drone0}" -p use_sim_time:="${USE_SIM_TIME:-true}"
