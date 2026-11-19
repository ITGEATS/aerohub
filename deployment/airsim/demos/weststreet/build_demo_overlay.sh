#!/usr/bin/env bash
# Build task interfaces/clients in an independent workspace over the fixed image.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
work=${1:-"$repo/log/weststreet-demo/client-workspace"}
mkdir -p "$work"
work=$(cd "$work" && pwd)
python3 "$repo/deployment/airsim/demos/super_forest/prepare_navigation_dependencies.py" "$work"
docker run --rm --init --network none --entrypoint bash \
  -v "$repo:/repo:ro" -v "$work:/work" onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
cd /work
python3 - <<"PYDEPS"
import json
from pathlib import Path
import zipfile
for item in json.loads(Path("/repo/deployment/airsim/demos/super_forest/navigation_dependencies.json").read_text()):
    with zipfile.ZipFile(Path("/work/wheels")/item["filename"]) as wheel:
        wheel.extractall("/work/python_deps")
PYDEPS
export PYTHONPATH="/work/python_deps${PYTHONPATH:+:$PYTHONPATH}"
colcon build --executor sequential --base-paths \
  /repo/as2/aerostack2/as2_python_api \
  /repo/src/tool/system/interfaces /repo/src/tool/system/python_api \
  /repo/src/tool/navigation/interfaces /repo/src/tool/navigation/python_api \
  /repo/src/tool/target_tracking/interfaces /repo/src/tool/target_tracking/python_api \
  --event-handlers console_direct+
source /work/install/setup.bash
export AS2_MODULES_PATH=/work/install/uav_system_python_api/share/uav_system_python_api/modules:/work/install/uav_planner_python_api/share/uav_planner_python_api/modules:/work/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=193 PYTHONDONTWRITEBYTECODE=1
python3 - <<"PYTEST"
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
rclpy.init()
drone = DroneInterfaceBase("demo_client_check", use_sim_time=True)
try:
    for module in ("system", "navigation", "target_tracking"):
        drone.load_module(module)
    print("Three Tool modules loaded; no motion requested", flush=True)
finally:
    drone.shutdown()
    drone.destroy_node()
    rclpy.shutdown()
PYTEST
'
