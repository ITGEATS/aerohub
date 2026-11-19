#!/usr/bin/env bash
set -euo pipefail

image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.2}

test "$(docker image inspect "$image" --format '{{index .Config.Labels "io.aerohub.role"}}')" = compute-base
docker run --rm --network none --entrypoint bash "$image" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/uav_execution_ws/install/setup.bash
test "$(dpkg-query -W -f=\${Version} python3-rosdep)" = 0.27.0-1
python3 -c "import msgpack; from px4_msgs.msg import VehicleStatus; assert VehicleStatus.MESSAGE_VERSION == 4"
ros2 pkg prefix as2_platform_pixhawk
ros2 pkg prefix as2_state_estimator
ros2 pkg prefix as2_motion_controller
ros2 pkg prefix as2_behaviors_motion
ros2 pkg prefix uav_planner_interfaces
ros2 pkg prefix uav_follow_path_execution
! command -v px4
! command -v MicroXRCEAgent
! test -e /opt/ros/noetic
for package in d2vins dpvo fast_livo pin_slam rog_map falcon_ros2; do
  ! ros2 pkg prefix "$package" >/dev/null 2>&1
done
'

printf 'onboard_computer contract passed: %s\n' "$image"
