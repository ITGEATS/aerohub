ARG BASE_IMAGE=onboard_computer:v0.1.2
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG FALCON_BUILD_JOBS=2

RUN apt-get update && apt-get install -y --no-install-recommends \
      libnlopt-cxx-dev=2.7.1-3build1 \
      libgoogle-glog-dev=0.5.0+really0.4.0-2 \
      libdw-dev=0.186-1ubuntu0.1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/falcon_ws/src
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/ falcon_planner/
COPY src/tool/navigation/interfaces/ uav_planner_interfaces/
COPY src/behavior/follow_path/ uav_follow_path_execution/
COPY src/tool/navigation/planner/falcon/ros2_adapter/ uav_falcon_adapter/

WORKDIR /opt/falcon_ws
RUN source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && export CMAKE_BUILD_PARALLEL_LEVEL="${FALCON_BUILD_JOBS}" \
    && colcon build --merge-install --executor sequential \
      --base-paths \
        src/falcon_planner/falcon_ros2 \
        src/uav_planner_interfaces \
        src/uav_follow_path_execution \
        src/uav_falcon_adapter \
      --packages-select falcon_ros2 uav_planner_interfaces \
        uav_follow_path_execution uav_falcon_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      libnlopt-cxx0=2.7.1-3build1 \
      libgoogle-glog0v5=0.5.0+really0.4.0-2 \
      libdw1=0.186-1ubuntu0.1 \
    && rm -rf /var/lib/apt/lists/*

ARG FALCON_SOURCE_DIGEST=unknown
ARG FALCON_REVISION=unknown
LABEL org.opencontainers.image.title="AeroHub FALCON native ROS 2 runtime" \
      org.opencontainers.image.description="Self-contained ROS 2 Humble FALCON exploration runtime" \
      io.aerohub.source.digest="${FALCON_SOURCE_DIGEST}" \
      io.aerohub.falcon.revision="${FALCON_REVISION}"

COPY --from=builder /opt/falcon_ws/install/ /opt/falcon_ws/install/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/exploration_manager/config/ \
  /opt/falcon/config/upstream/exploration_manager/config/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/voxel_mapping/config/ \
  /opt/falcon/config/upstream/voxel_mapping/config/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/pathfinding/config/ \
  /opt/falcon/config/upstream/pathfinding/config/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/fast_planner/config/ \
  /opt/falcon/config/upstream/fast_planner/config/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/exploration_preprocessing/config/ \
  /opt/falcon/config/upstream/exploration_preprocessing/config/
COPY src/tool/navigation/planner/falcon/upstream/falcon_planner/exploration_utils/config/ \
  /opt/falcon/config/upstream/exploration_utils/config/

COPY deployment/falcon/compute_runtime.py /opt/falcon/runtime/compute_runtime.py
COPY deployment/falcon/prepare_params.py /opt/falcon/runtime/prepare_params.py
COPY deployment/airsim/as2/run.py /opt/falcon/runtime/run_as2.py
COPY deployment/airsim/config/as2/config.yaml /opt/falcon/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/falcon/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/falcon/runtime/fastdds_udp.xml
COPY deployment/falcon/tests/test_prepare_params.py /opt/falcon/tests/deployment/test_prepare_params.py
COPY src/tool/navigation/planner/falcon/ros2_adapter/tests/ /opt/falcon/tests/adapter/
COPY deployment/falcon/entrypoint.sh /usr/local/bin/aerohub-falcon

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/falcon/runtime/fastdds_udp.xml \
    PYTHONPATH=/opt/falcon/runtime

RUN chmod +x /usr/local/bin/aerohub-falcon \
    && printf '%s\n' "${FALCON_SOURCE_DIGEST}" > /opt/falcon/source-digest \
    && test ! -e /opt/ros/noetic \
    && test ! -e /opt/falcon_ws/devel \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/falcon_ws/install /opt/falcon/config /opt/falcon/runtime \
         > /tmp/host-path-hits; then \
         cat /tmp/host-path-hits; exit 1; \
       fi

ENTRYPOINT ["/usr/local/bin/aerohub-falcon"]
