#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
if [[ -f /opt/px4_ws/install/setup.bash ]]; then source /opt/px4_ws/install/setup.bash; fi
source /opt/flowpilot/ros_ws/install/setup.bash
export AS2_MODULES_PATH="/opt/flowpilot/ros_ws/install/uav_planner_python_api/share/uav_planner_python_api/modules${AS2_MODULES_PATH:+:$AS2_MODULES_PATH}"
export AS2_CONFIG_FILE=/repo/deployment/airsim/config/as2/config.yaml
export AS2_FLEET_FILE=/repo/deployment/airsim/demos/flowpilot_single/fleet.yaml
cd /repo
role=${1:-compute}
shift || true
case "$role" in
  compute) exec bash deployment/airsim/demos/flowpilot_single/runtime.sh "$@" ;;
  sensor) exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args \
    -p settings_file:=/config/settings.json -p rpc_host:="${AIRSIM_HOST:?}" -p rpc_port:="${AIRSIM_RPC_PORT:?}" \
    -p vehicle_name:=PX4 -p vehicle_namespace:=drone0 -p camera_rate_hz:=15.0 -p use_sim_time:=true ;;
  overhead) exec python3 -m deployment.swarm.overhead_view --settings /config/settings.json --host "${AIRSIM_HOST:?}" --state-file /evidence/overhead.json --primary-vehicle-camera PX4 ;;
  preflight) exec python3 deployment/airsim/demos/flowpilot_single/preflight.py "$@" ;;
  mission) exec python3 deployment/airsim/demos/flowpilot_single/mission.py "$@" ;;
  check) exec python3 deployment/airsim/demos/flowpilot_single/sensor_health.py ;;
  parity) exec /opt/flowpilot/model-venv/bin/python deployment/airsim/demos/flowpilot_single/check_parity.py --cuda ;;
  ros-smoke) exec python3 deployment/inference/tests/flowpilot_ros_smoke.py ;;
  *) exec "$role" "$@" ;;
esac
