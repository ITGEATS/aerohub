#!/usr/bin/env bash
# Actual generated ROS messages, TF2 and DDS. Deterministic inference, no scene.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/object-detection-query/workspace"}
docker run --rm --init --network host --name object-detection-query-tests \
 -e ROS_DOMAIN_ID="${QUERY_TEST_DOMAIN:-231}" -e ROS_LOCALHOST_ONLY=1 \
 -v "$repo:/repo:ro" -v "$work:/work:ro" \
 --entrypoint bash onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/work/python_deps${PYTHONPATH:+:$PYTHONPATH}
export AS2_MODULES_PATH=/work/install/uav_object_detection_python_api/share/uav_object_detection_python_api/modules:/work/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules
python3 /repo/deployment/inference/tests/test_yolo_contract.py
python3 /repo/src/tool/object_detection/tests/test_box_depth.py
python3 /repo/src/tool/object_detection/tests/test_projection.py
python3 /repo/src/tool/object_detection/tests/test_snapshot_ros.py
python3 /repo/src/tool/object_detection/tests/test_backend_ros.py
python3 /repo/src/tool/target_tracking/tests/test_core.py
python3 /repo/src/tool/target_tracking/tests/test_ros.py
'
