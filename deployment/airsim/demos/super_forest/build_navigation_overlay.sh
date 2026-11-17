#!/usr/bin/env bash
# Keep the existing simulation image; compile only the navigation overlay.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/super-navigation/workspace"}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
python3 "$repo/deployment/airsim/demos/super_forest/prepare_navigation_dependencies.py" "$workspace"
docker run --rm --network none --entrypoint /bin/bash \
  -v "$repo:/repo:ro" -v "$workspace:/work" \
  "${SUPER_APP_IMAGE:-uav/apps-super-forest:migration-main-behavior}" -lc '
set -eo pipefail
source /opt/super_ws/install/setup.bash
source /opt/super_app_ws/install/setup.bash
cd /work
python3 - <<"PYDEPS"
import json, pathlib, zipfile
for item in json.loads(pathlib.Path("/repo/deployment/airsim/demos/super_forest/navigation_dependencies.json").read_text()):
    with zipfile.ZipFile(pathlib.Path("/work/wheels") / item["filename"]) as wheel:
        wheel.extractall("/work/python_deps")
PYDEPS
export PYTHONPATH="/work/python_deps${PYTHONPATH:+:$PYTHONPATH}"
MAKEFLAGS=-j2 colcon build --merge-install --parallel-workers 2 --base-paths \
  /repo/as2/aerostack2/as2_python_api \
  /repo/src/tool/navigation/interfaces /repo/src/tool/navigation/framework \
  /repo/src/behavior/follow_path /repo/src/tool/navigation/python_api \
  /repo/src/tool/navigation/planner/super/ros2_adapter/bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > build.log 2>&1
source install/setup.bash
export AS2_MODULES_PATH=/work/install/share/uav_planner_python_api/modules
ROS_DOMAIN_ID=194 python3 /repo/src/tool/navigation/planner/super/ros2_adapter/bridge/tests/navigation_interface.py
python3 -m unittest discover -s /repo/src/tool/navigation/planner/super/ros2_adapter/bridge/tests -p "test_*.py" -v
python3 -m unittest discover -s /repo/src/tool/navigation/framework/tests -v
'
