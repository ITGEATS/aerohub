#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/system-tool/workspace"}
docker run --rm --init --network none --entrypoint bash \
 -v "$repo:/repo:ro" -v "$work:/work:ro" onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/repo/src/tool/system/ros2_adapter:/repo/src/tool/system/motion_resource:/repo/src/tool/system/python_api${PYTHONPATH:+:$PYTHONPATH}
python3 -m unittest discover -s /repo/src/tool/system/tests -p test_ros.py -v
'
