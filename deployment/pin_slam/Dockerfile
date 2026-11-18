# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=onboard_computer:v0.1.1
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN python3 -m venv --without-pip --system-site-packages /opt/pin_venv
COPY --from=pin_wheels . /opt/pin_wheelhouse/
COPY deployment/pin_slam/requirements.lock /opt/pin_slam/requirements.lock
COPY deployment/pin_slam/bootstrap-wheels.sha256 /opt/pin_slam/bootstrap-wheels.sha256
RUN cd /opt/pin_wheelhouse \
    && sha256sum --check /opt/pin_slam/bootstrap-wheels.sha256 \
    && PYTHONPATH=/opt/pin_wheelhouse/pip-24.3.1-py3-none-any.whl \
      /opt/pin_venv/bin/python -m pip install --no-index --find-links /opt/pin_wheelhouse \
      pip==24.3.1 setuptools==75.6.0 wheel==0.45.1 \
    && /opt/pin_venv/bin/python -m pip install --no-index \
      --find-links /opt/pin_wheelhouse --require-hashes \
      --requirement /opt/pin_slam/requirements.lock

WORKDIR /opt/pin_ws/src
COPY src/basic_function/state_estimator/pin_slam/ros2_adapter/ uav_pin_slam_ros2/
COPY src/basic_function/state_estimator/pin_slam/plugin/ uav_pin_slam/
COPY simulator/airsim/sensors/ uav_as2_sensor_adapter/
WORKDIR /opt/pin_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --executor sequential \
      --packages-select uav_pin_slam_ros2 uav_pin_slam uav_as2_sensor_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="AeroHub PIN-SLAM native ROS 2 runtime"
RUN apt-get update && apt-get install -y --no-install-recommends python3-msgpack \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/pin_venv/ /opt/pin_venv/
COPY --from=builder /opt/pin_ws/install/ /opt/pin_ws/install/
COPY src/basic_function/state_estimator/pin_slam/upstream/ /opt/pin_slam/upstream/
COPY deployment/pin_slam/airsim.yaml /opt/pin_slam/config/airsim.yaml
COPY deployment/pin_slam/algorithm.params.yaml /opt/pin_slam/config/algorithm.yaml
COPY src/basic_function/state_estimator/pin_slam/plugin/config/estimator.yaml /opt/pin_slam/config/estimator.yaml
COPY deployment/pin_slam/airsim_truth_bridge.py /opt/pin_slam/runtime/airsim_truth_bridge.py
COPY deployment/pin_slam/shadow_validation.py /opt/pin_slam/runtime/shadow_validation.py
COPY deployment/pin_slam/gate_result.py /opt/pin_slam/runtime/gate_result.py
COPY deployment/pin_slam/validation.yaml /opt/pin_slam/runtime/validation.yaml
COPY deployment/state_estimator/canonical_freshness.py /opt/pin_slam/runtime/canonical_freshness.py
COPY deployment/airsim/as2/run.py /opt/pin_slam/runtime/run_as2.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/pin_slam/runtime/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/pin_slam/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/pin_slam/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/pin_slam/runtime/fastdds_udp.xml
COPY deployment/pin_slam/entrypoint.sh /usr/local/bin/aerohub-pin-slam
ENV PIN_SLAM_ROOT=/opt/pin_slam/upstream \
    PYTHONPATH=/opt/pin_slam/upstream \
    PIN_SLAM_NAMESPACE=drone0 \
    PIN_SLAM_USE_SIM_TIME=true \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/pin_slam/runtime/fastdds_udp.xml
RUN chmod +x /usr/local/bin/aerohub-pin-slam \
    && test ! -e /opt/ros/noetic \
    && test ! -e /opt/pin_ws/devel \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/pin_ws/install /opt/pin_slam/config /opt/pin_slam/runtime \
         > /tmp/path-hits; then \
         cat /tmp/path-hits; exit 1; fi
ENTRYPOINT ["/usr/local/bin/aerohub-pin-slam"]
