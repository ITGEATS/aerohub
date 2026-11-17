#!/usr/bin/env bash
set -euo pipefail

FALCON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FALCON_IMAGE=onboard_computer:v0.1.0

docker run --rm --init --network host \
  --name aerohub-falcon-check \
  --entrypoint bash \
  -v "$FALCON_ROOT:/workspace" \
  "$FALCON_IMAGE" -lc '
set -eo pipefail
cd /workspace/log/falcon/deps
sha256sum -c /workspace/deployment/falcon/dependencies.sha256
dependency_root=/tmp/falcon-deps-root
rm -rf "$dependency_root"
mkdir -p "$dependency_root"
for package in ./*.deb; do dpkg-deb -x "$package" "$dependency_root"; done
export LD_LIBRARY_PATH="$dependency_root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
set +u
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /workspace/log/falcon/install/setup.bash
set -u
cd /workspace
python3 deployment/falcon/prepare_params.py \
  --source src/tool/navigation/planner/falcon/upstream/falcon_planner \
  --runtime-dir log/falcon/check/runtime \
  --output log/falcon/check/falcon.params.yaml
PYTHONPATH="/workspace/deployment/falcon:${PYTHONPATH:-}" python3 -m pytest -q \
  src/tool/navigation/planner/falcon/ros2_adapter/tests \
  deployment/falcon/tests
timeout 10 ros2 run falcon_ros2 falcon_exploration_node \
  --ros-args --params-file log/falcon/check/falcon.params.yaml \
  -r /odom_world:=/falcon/input/odometry \
  -r /voxel_mapping/depth_image:=/falcon/input/depth \
  -r /transformer/sensor_pose_topic:=/falcon/input/sensor_pose \
  > log/falcon/check/node.log 2>&1 &
node_pid=$!
trap '\''kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true'\'' EXIT
sleep 3
kill -0 "$node_pid"
ros2 node list | grep -Fx /exploration_node
'
