#!/usr/bin/env bash
set -eo pipefail

: "${D2_VEHICLE:?D2_VEHICLE is required}"
: "${D2_VEHICLE_INDEX:?D2_VEHICLE_INDEX is required}"
: "${D2_MAIN_ID:?D2_MAIN_ID is required}"
: "${ROS_MASTER_URI:?ROS_MASTER_URI is required}"
: "${D2_START_AS2_PLUGIN:=1}"
if [[ "${D2_START_AS2_PLUGIN}" != 0 && "${D2_START_AS2_PLUGIN}" != 1 ]]; then
  printf 'D2_START_AS2_PLUGIN must be 0 or 1\n' >&2
  exit 2
fi

workspace=/workspace
runtime=/runtime
mkdir -p "${runtime}/output"

children=()
start_process() {
  local name=$1
  shift
  setsid "$@" > "${runtime}/${name}.log" 2>&1 &
  children+=("$!")
}

stop_processes() {
  local pid
  for pid in "${children[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in "${children[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap stop_processes EXIT INT TERM

start_process roscore bash -lc '
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH=/workspace/ros1/bin:${PATH}
  export LD_LIBRARY_PATH=/workspace/ros1/lib:${LD_LIBRARY_PATH:-}
  source /workspace/ros1/setup.bash
  exec roscore -p "${ROS_MASTER_URI##*:}"'

for _ in $(seq 1 200); do
  if env PATH="${workspace}/ros1/bin:${PATH}" \
      LD_LIBRARY_PATH="${workspace}/ros1/lib:${LD_LIBRARY_PATH:-}" \
      "${workspace}/ros1/bin/rosparam" list >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
env PATH="${workspace}/ros1/bin:${PATH}" \
  LD_LIBRARY_PATH="${workspace}/ros1/lib:${LD_LIBRARY_PATH:-}" \
  "${workspace}/ros1/bin/rosparam" list >/dev/null

start_process ros1-proxy bash -lc '
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH=/workspace/ros1/bin:${PATH}
  export LD_LIBRARY_PATH=/workspace/ros1/lib:${LD_LIBRARY_PATH:-}
  source /workspace/ros1/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  exec python3 /repo/src/basic_function/state_estimator/d2_slam/ros1_proxy/scripts/d2_slam_ros1_proxy.py \
    _socket_path:=/runtime/bridge.sock \
    _odometry_topic:=/d2vins/odometry \
    _landmarks_topic:=/d2vins/point_cloud'

start_process d2-slam bash -lc '
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  export PATH=/workspace/ros1/bin:/workspace/cuda11/bin:${PATH}
  export LD_LIBRARY_PATH=/workspace/ros1/lib:/workspace/cuda11/lib:/workspace/onnxruntime-linux-x64-gpu-1.18.1/lib:/workspace/install/lib:${LD_LIBRARY_PATH:-}
  source /workspace/ros1/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  exec roslaunch /repo/deployment/d2_slam/config/d2_slam.launch \
    self_id:=${D2_VEHICLE_INDEX} main_id:=${D2_MAIN_ID} output_path:=/runtime/output'

start_process ros2-adapter bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /workspace/ros2/install/setup.bash
  exec ros2 run uav_d2_slam_ros2 d2_slam_bridge --ros-args \
    -p vehicle_namespace:=${D2_VEHICLE} \
    -p socket_path:=/runtime/bridge.sock \
    -p use_sim_time:=true'

if [[ "${D2_START_AS2_PLUGIN}" == 1 ]]; then
  start_process as2-plugin bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/aerostack2_ws/install/setup.bash
    source /workspace/ros2/install/setup.bash
    exec ros2 launch uav_d2_slam d2_slam.launch.py \
      namespace:=${D2_VEHICLE} use_sim_time:=true'
fi

while true; do
  for pid in "${children[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}"
      exit $?
    fi
  done
  sleep 0.5
done
