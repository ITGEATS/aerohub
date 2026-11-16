#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
output=${1:-"$repo/log/yopov2-tracking/build"}
mkdir -p "$output"
output=$(cd "$output" && pwd)
docker run --rm --init --network none -v "$repo:/repo:ro" -v "$output:/work" \
 --entrypoint bash onboard_computer:v0.1.1 -lc '
set -e
export ROS_DOMAIN_ID=217
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
cd /work
colcon --log-base /work/colcon-log build --base-paths /repo/src/tool/target_tracking /repo/src/tool/object_detection/interfaces /repo/src/basic_function/inference/interfaces --build-base /work/build --install-base /work/install --executor sequential
source /work/install/setup.bash
cmp /repo/deployment/airsim/demos/target_tracking/reference/yopo_tracker/YOPO/policy/target_ekf.py /work/install/uav_target_tracking/share/uav_target_tracking/upstream/target_ekf.py
cmp /repo/deployment/airsim/demos/target_tracking/reference/yopo_tracker/LICENSE /work/install/uav_target_tracking/share/uav_target_tracking/upstream/LICENSE
python3 /repo/src/tool/target_tracking/tests/test_core.py -v
python3 /repo/src/tool/target_tracking/tests/test_ros.py -v
python3 -c "from uav_target_tracking.node import TrackingNode; import rclpy; rclpy.init(); n=TrackingNode(); n.destroy_node(); rclpy.shutdown()"
'
