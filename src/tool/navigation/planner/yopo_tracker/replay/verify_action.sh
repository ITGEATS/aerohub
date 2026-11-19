#!/usr/bin/env bash
# Compile the existing execution packages and run the isolated ROS fixture.
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)"
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/yopo-tracker/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1

cd "${TMPDIR:-/tmp}"
colcon --log-base yopo-action-log build --base-paths \
  "$repo/src/tool/navigation/interfaces" "$repo/src/behavior/follow_path" \
  --build-base yopo-action-build --install-base yopo-action-install \
  --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source yopo-action-install/local_setup.bash

follow_path=$(ros2 pkg prefix uav_follow_path_execution)
as2_motion=$(ros2 pkg prefix --share as2_behaviors_motion)
"$follow_path/lib/uav_follow_path_execution/follow_path_extended_node" --ros-args \
  --params-file "$as2_motion/follow_path_behavior/config/config_default.yaml" \
  -r __ns:=/drone0 -p plugin_name:=follow_path_plugin_position \
  -p follow_path_speed:=1.0 -p follow_path_threshold:=0.2 -p tf_timeout_threshold:=0.1 &
behavior=$!
trap 'kill "$behavior" 2>/dev/null || true; wait "$behavior" 2>/dev/null || true' EXIT

replay="$repo/src/tool/navigation/planner/yopo_tracker/replay"
if [[ "${1:-}" == --selected-fixture ]]; then
  fixture=$(mktemp -d)
  python3 "$replay/make_test_bag.py" "$fixture/bag" --frame drone0/odom
  /opt/vision/bin/python -u "$replay/verify_selected.py" /opt/yopo-tracker/upstream "$fixture/bag" --expect-empty
  /opt/vision/bin/python -u "$replay/verify_selected.py" /opt/yopo-tracker/upstream "$fixture/bag" --fixture-heads
elif [[ "${1:-}" == --selected ]]; then
  shift
  /opt/vision/bin/python -u "$replay/verify_selected.py" /opt/yopo-tracker/upstream "$@"
else
  /opt/vision/bin/python -u "$replay/check_action.py" /opt/yopo-tracker/upstream
fi
