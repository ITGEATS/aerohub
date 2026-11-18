# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=onboard_computer:v0.1.1
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=dpv_wheels . /opt/dpv/wheelhouse/
COPY --from=dpv_cuda_packages . /opt/dpv/cuda-packages/
COPY deployment/dpv_slam/python-wheels.sha256 /opt/dpv/python-wheels.sha256
COPY deployment/dpv_slam/cuda-build-packages.sha256 /opt/dpv/cuda-build-packages.sha256
RUN cd /opt/dpv/wheelhouse \
    && awk '{print $1 "  /opt/dpv/wheelhouse/" $2}' /opt/dpv/python-wheels.sha256 \
      | sha256sum --check \
    && cd /opt/dpv/cuda-packages \
    && awk '{print $1 "  /opt/dpv/cuda-packages/" $2}' /opt/dpv/cuda-build-packages.sha256 \
      | sha256sum --check \
    && mkdir -p /opt/dpv/python /opt/dpv/cuda \
    && for package in /opt/dpv/cuda-packages/*.tar.bz2; do \
         tar -xjf "$package" -C /opt/dpv/cuda; \
       done \
    && PYTHONPATH=/opt/dpv/wheelhouse/pip-24.3.1-py3-none-any.whl \
      python3 -m pip install --upgrade --no-index --no-deps \
      --target /opt/dpv/python /opt/dpv/wheelhouse/*.whl

COPY src/basic_function/state_estimator/dpv_slam/upstream/ /opt/dpv/upstream/
COPY deployment/dpv_slam/build_extensions.py /opt/dpv/build_extensions.py
RUN export CUDA_HOME=/opt/dpv/cuda \
           PATH=/opt/dpv/python/bin:/opt/dpv/python/ninja/data/bin:/opt/dpv/cuda/bin:$PATH \
           LD_LIBRARY_PATH=/opt/dpv/cuda/lib:/opt/dpv/cuda/lib64 \
           PYTHONPATH=/opt/dpv/python:/opt/dpv/upstream \
           TORCH_CUDA_ARCH_LIST=8.9 MAX_JOBS=2 DPVO_SOURCE=/opt/dpv/upstream \
    && mkdir -p /opt/dpv/extensions \
    && cd /opt/dpv/extensions \
    && python3 /opt/dpv/build_extensions.py build_ext --build-lib /opt/dpv/python

WORKDIR /opt/dpv_ws/src
COPY src/basic_function/state_estimator/dpv_slam/ros2_adapter/ uav_dpv_slam_ros2/
COPY src/basic_function/state_estimator/dpv_slam/plugin/ uav_dpv_slam/
COPY simulator/airsim/sensors/ uav_as2_sensor_adapter/
WORKDIR /opt/dpv_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --executor sequential \
      --packages-select uav_dpv_slam_ros2 uav_dpv_slam uav_as2_sensor_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="AeroHub DPV-SLAM native ROS 2 runtime"
RUN apt-get update && apt-get install -y --no-install-recommends python3-msgpack \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/dpv/python/ /opt/dpv/python/
COPY --from=builder /opt/dpv/cuda/ /opt/dpv/cuda/
COPY --from=builder /opt/dpv/upstream/ /opt/dpv/upstream/
COPY --from=builder /opt/dpv_ws/install/ /opt/dpv_ws/install/
COPY deployment/dpv_slam/dpvo-airsim.yaml /opt/dpv/config/dpvo-airsim.yaml
COPY deployment/dpv_slam/models/dpvo.pth /opt/dpv/models/dpvo.pth
COPY deployment/dpv_slam/algorithm.params.yaml /opt/dpv/config/algorithm.yaml
COPY src/basic_function/state_estimator/dpv_slam/plugin/config/estimator.yaml /opt/dpv/config/estimator.yaml
COPY deployment/dpv_slam/airsim_truth_bridge.py /opt/dpv/runtime/airsim_truth_bridge.py
COPY deployment/dpv_slam/shadow_validation.py /opt/dpv/runtime/shadow_validation.py
COPY deployment/dpv_slam/gate_result.py /opt/dpv/runtime/gate_result.py
COPY deployment/dpv_slam/validation.yaml /opt/dpv/runtime/validation.yaml
COPY deployment/state_estimator/canonical_freshness.py /opt/dpv/runtime/canonical_freshness.py
COPY deployment/airsim/as2/run.py /opt/dpv/runtime/run_as2.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/dpv/runtime/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/dpv/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/dpv/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/dpv/runtime/fastdds_udp.xml
COPY deployment/dpv_slam/entrypoint.sh /usr/local/bin/aerohub-dpv-slam
ENV PYTHONPATH=/opt/dpv/python:/opt/dpv/upstream \
    PATH=/opt/dpv/python/bin:/opt/dpv/cuda/bin:$PATH \
    LD_LIBRARY_PATH=/opt/dpv/python/torch/lib:/opt/dpv/cuda/lib:/opt/dpv/cuda/lib64 \
    DPV_SLAM_NAMESPACE=drone0 DPV_SLAM_USE_SIM_TIME=true \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/dpv/runtime/fastdds_udp.xml
RUN chmod +x /usr/local/bin/aerohub-dpv-slam \
    && test ! -e /opt/ros/noetic \
    && test ! -e /opt/dpv_ws/devel \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/dpv_ws/install /opt/dpv/config /opt/dpv/runtime \
         > /tmp/path-hits; then \
         cat /tmp/path-hits; exit 1; fi
ENTRYPOINT ["/usr/local/bin/aerohub-dpv-slam"]
