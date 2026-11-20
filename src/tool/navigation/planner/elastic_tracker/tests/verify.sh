#!/usr/bin/env bash
# Fixed onboard tool image; checkout mounted read-only at /workspace.
# Separate writable build volume at /build; network disabled. No simulator/arming.
set -eo pipefail
root=${ELASTIC_REPOSITORY:-/workspace}
overlay=${ELASTIC_BUILD_WORKSPACE:-/build}
component="$root/src/tool/navigation/planner/elastic_tracker"
if [[ ${ELASTIC_SKIP_BUILD:-0} != 1 ]]; then
  bash "$root/deployment/elastic_tracker_humble/build.sh" "$root" "$overlay"
fi
source /root/aerostack2_ws/install/setup.bash
source "$overlay/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
python3 - <<'PY'
from uav_planner_interfaces.action import FollowTimedPath
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_target_tracking_interfaces.action import TrackObject
from uav_target_tracking.core import load_ekf
from uav_object_detection.node import ObjectDetectionTool
from uav_elastic_tracker_adapter.native_execution import NativeAdapter
from uav_elastic_tracker_adapter.elastic_tracking_bridge import TargetStateBridge
assert DetectSnapshot.Request().query == '' and not TrackObject.Goal().follow
load_ekf()
print('PASS: actual ROS interfaces, shared detector, locked EKF and bridge imported')
PY
for test in test_core test_execution_client test_rpc_freshness test_rolling_handoff test_native_rpc test_direct_native test_target_state_bridge; do
 python3 -m unittest discover -s "$component/tests" -p "$test.py"
done
python3 "$component/tests/check_layout.py"
python3 "$component/tests/native_replay.py"
python3 "$component/tests/native_handoff_timing.py" --require-clean --output "$overlay/direct-handoff-$(date +%s%N)"
# Synthetic inputs and fake controller/platform, with compiled standard execution.
python3 "$component/tests/native_ros_smoke.py" --launch-only
python3 "$root/src/behavior/follow_path/tests/ros_smoke.py"
