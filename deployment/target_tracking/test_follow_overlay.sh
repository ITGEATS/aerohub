#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/target-tracking-follow/workspace"}
query=${2:-"$repo/log/object-detection-query/workspace"}
docker run --rm --init --network none --entrypoint bash \
 -v "$repo:/repo:ro" -v "$work:/work:ro" -v "$query:/query-work:ro" \
 onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /query-work/install/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/query-work/python_deps${PYTHONPATH:+:$PYTHONPATH}
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1
python3 -m unittest discover -s /repo/src/tool/target_tracking/tests -p test_facing.py -v
python3 -m unittest discover -s /repo/src/tool/target_tracking/tests -p test_measurement_filter.py -v
python3 /repo/src/tool/target_tracking/tests/test_core.py
python3 /repo/src/tool/target_tracking/tests/test_ros.py
python3 /repo/src/tool/target_tracking/tests/test_follow_ros.py
python3 /repo/src/tool/target_tracking/tests/test_stream_ros.py
'
