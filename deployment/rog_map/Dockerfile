ARG BASE_IMAGE=onboard_computer:v0.1.1
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/rog_map_ws/src
COPY src/tool/navigation/planner/super/ros2_adapter/native/rog_map/ rog_map/
COPY src/basic_function/perception/rog_map/interfaces/ uav_rog_map_interfaces/
COPY src/basic_function/perception/rog_map/ros2_adapter/ uav_rog_map/
COPY src/basic_function/perception/navigation_inputs/ as2_navigation_inputs/
COPY simulator/airsim/sensors/ uav_as2_sensor_adapter/

WORKDIR /opt/rog_map_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --executor sequential \
      --packages-up-to uav_rog_map as2_navigation_inputs uav_as2_sensor_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="AeroHub ROG-Map ROS 2 runtime"
RUN apt-get update && apt-get install -y --no-install-recommends python3-msgpack \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/rog_map_ws/install/ /opt/rog_map_ws/install/
COPY deployment/rog_map/acceptance.py /opt/rog_map/runtime/acceptance.py
COPY deployment/airsim/as2/run.py /opt/rog_map/runtime/run_as2.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/rog_map/runtime/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/rog_map/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/rog_map/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/rog_map/runtime/fastdds_udp.xml
COPY deployment/rog_map/entrypoint.sh /usr/local/bin/aerohub-rog-map
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/rog_map/runtime/fastdds_udp.xml
RUN chmod +x /usr/local/bin/aerohub-rog-map \
    && test ! -e /opt/ros/noetic \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/rog_map_ws/install /opt/rog_map/runtime \
         > /tmp/host-path-hits; then cat /tmp/host-path-hits; exit 1; fi
ENTRYPOINT ["/usr/local/bin/aerohub-rog-map"]
