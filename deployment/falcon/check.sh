#!/usr/bin/env bash
set -euo pipefail

FALCON_IMAGE=${FALCON_IMAGE:-aerohub/falcon:ros2-humble}
FALCON_WORKSTATION_IMAGE=${FALCON_WORKSTATION_IMAGE:-aerohub/falcon-workstation:ros2-humble}

docker run --rm --init --network none \
  --name aerohub-falcon-check \
  "$FALCON_IMAGE" shell -lc '
set -eo pipefail
test ! -e /opt/ros/noetic
! command -v roscore
! command -v roslaunch
! command -v catkin_make
test -d /opt/falcon_ws/install
export FALCON_CONFIG_SOURCE=/opt/falcon/config/upstream
python3 /opt/falcon/runtime/prepare_params.py \
  --source /opt/falcon/config/upstream \
  --runtime-dir /tmp/falcon-runtime \
  --output /tmp/falcon.params.yaml
python3 -m pytest -q --junitxml=/tmp/falcon-pytest.xml \
  /opt/falcon/tests/adapter \
  /opt/falcon/tests/deployment/test_prepare_params.py
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

root = ET.parse(Path("/tmp/falcon-pytest.xml")).getroot()
totals = {name: sum(int(node.attrib.get(name, 0)) for node in root.iter("testsuite"))
          for name in ("failures", "errors", "skipped")}
if any(totals.values()):
    raise SystemExit(f"pytest matrix is incomplete: {totals}")
PY
timeout 10 ros2 run falcon_ros2 falcon_exploration_node \
  --ros-args --params-file /tmp/falcon.params.yaml \
  -r /odom_world:=/falcon/input/odometry \
  -r /voxel_mapping/depth_image:=/falcon/input/depth \
  -r /transformer/sensor_pose_topic:=/falcon/input/sensor_pose \
  >/tmp/falcon-node.log 2>&1 &
node_pid=$!
trap '\''kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true'\'' EXIT
sleep 3
kill -0 "$node_pid"
ros2 node list | grep -Fx /exploration_node
'

docker run --rm --init --network none \
  --name aerohub-falcon-workstation-check \
  "$FALCON_WORKSTATION_IMAGE" shell -lc '
set -eo pipefail
test ! -e /opt/ros/noetic
! command -v roscore
! command -v roslaunch
! command -v catkin_make
test -d /opt/falcon_workstation_ws/install
! ros2 pkg prefix falcon_ros2 >/dev/null 2>&1
python3 -c "from as2_msgs.action import Land, Takeoff; from as2_msgs.msg import PlatformInfo, TrajectorySetpoints"
python3 -m pytest -q --junitxml=/tmp/falcon-workstation-pytest.xml \
  /opt/falcon/tests/test_mission.py \
  /opt/falcon/tests/test_run.py \
  /opt/falcon/tests/test_prepare_workstation.py
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

root = ET.parse(Path("/tmp/falcon-workstation-pytest.xml")).getroot()
totals = {name: sum(int(node.attrib.get(name, 0)) for node in root.iter("testsuite"))
          for name in ("failures", "errors", "skipped")}
if any(totals.values()):
    raise SystemExit(f"workstation pytest matrix is incomplete: {totals}")
PY
'
