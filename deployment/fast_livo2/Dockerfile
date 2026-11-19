ARG BASE_IMAGE=onboard_computer:v0.1.2
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY src/basic_function/state_estimator/fast_livo2/dependencies/sophus/ /opt/sophus/
RUN cmake -S /opt/sophus -B /opt/sophus-build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/fast-livo-deps \
      -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_INSTALL=ON \
    && cmake --build /opt/sophus-build --parallel 2 \
    && cmake --install /opt/sophus-build

WORKDIR /opt/fast_livo2_ws/src
COPY src/basic_function/state_estimator/fast_livo2/ros2_upstream/ fast_livo/
COPY src/basic_function/state_estimator/fast_livo2/dependencies/rpg_vikit/vikit_common/ vikit_common/
COPY src/basic_function/state_estimator/fast_livo2/dependencies/rpg_vikit/vikit_ros/ vikit_ros/
COPY src/basic_function/state_estimator/fast_livo2/ros2_adapter/livox_ros_driver2/ livox_ros_driver2/
COPY src/basic_function/state_estimator/fast_livo2/ros2_adapter/uav_fast_livo2_ros2/ uav_fast_livo2_ros2/
COPY src/basic_function/state_estimator/fast_livo2/plugin/ uav_fast_livo2/
COPY simulator/airsim/sensors/ uav_as2_sensor_adapter/
RUN mkdir -p fast_livo/Log

WORKDIR /opt/fast_livo2_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --executor sequential \
      --packages-select livox_ros_driver2 vikit_common vikit_ros fast_livo \
        uav_fast_livo2_ros2 uav_fast_livo2 uav_as2_sensor_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH=/opt/fast-livo-deps -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="AeroHub FAST-LIVO2 native ROS 2 runtime"
COPY --from=builder /opt/fast-livo-deps/ /opt/fast-livo-deps/
COPY --from=builder /opt/fast_livo2_ws/install/ /opt/fast_livo2_ws/install/
COPY deployment/fast_livo2/airsim.yaml /opt/fast_livo2/config/airsim.yaml
COPY deployment/fast_livo2/camera_airsim.yaml /opt/fast_livo2/config/camera.yaml
COPY deployment/fast_livo2/plugin.params.yaml /opt/fast_livo2/config/plugin.yaml
COPY deployment/fast_livo2/preflight_probe.py /opt/fast_livo2/runtime/preflight_probe.py
COPY deployment/fast_livo2/airsim_mission.py /opt/fast_livo2/runtime/airsim_mission.py
COPY deployment/fast_livo2/tests/adapter_integration.py /opt/fast_livo2/tests/adapter_integration.py
COPY simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py /opt/fast_livo2/runtime/airsim_rpc.py
COPY deployment/airsim/as2/run.py /opt/fast_livo2/runtime/run_as2.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/fast_livo2/runtime/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/fast_livo2/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/fast_livo2/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/fast_livo2/runtime/fastdds_udp.xml
COPY deployment/fast_livo2/run_algorithm.sh /usr/local/bin/aerohub-fast-livo2
ENV FAST_LIVO2_WORKSPACE_CONTAINER=/opt/fast_livo2_ws \
    FAST_LIVO2_CONFIG=/opt/fast_livo2/config/airsim.yaml \
    FAST_LIVO2_CAMERA_CONFIG=/opt/fast_livo2/config/camera.yaml \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fast_livo2/runtime/fastdds_udp.xml
RUN chmod +x /usr/local/bin/aerohub-fast-livo2 \
    && test ! -e /opt/ros/noetic \
    && test ! -e /opt/fast_livo2_ws/devel \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/fast_livo2_ws/install /opt/fast_livo2/config \
         /opt/fast_livo2/runtime > /tmp/path-hits; then \
         cat /tmp/path-hits; exit 1; fi
ENTRYPOINT ["/usr/local/bin/aerohub-fast-livo2"]
