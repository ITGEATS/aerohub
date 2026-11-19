ARG BASE_IMAGE=onboard_computer:v0.1.2
FROM ${BASE_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG D2_BUILD_JOBS=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl libopenblas-dev ninja-build unzip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/d2_sources
RUN curl --fail --location --retry 10 --retry-all-errors --retry-delay 2 \
      --output lcm-1.4.0.tar.gz \
      https://codeload.github.com/lcm-proj/lcm/tar.gz/refs/tags/v1.4.0 \
    && echo "149d7076369d261e6adbb25d713dc9e30aeba415f4fc62bb41e748b2eb229b46  lcm-1.4.0.tar.gz" \
      | sha256sum --check --status \
    && tar -xzf lcm-1.4.0.tar.gz \
    && cmake -S lcm-1.4.0 -B build-lcm -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/d2-deps \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DLCM_ENABLE_TESTS=OFF -DLCM_ENABLE_EXAMPLES=OFF \
      -DLCM_ENABLE_JAVA=OFF -DLCM_ENABLE_PYTHON=OFF \
    && cmake --build build-lcm --parallel "${D2_BUILD_JOBS}" \
    && cmake --install build-lcm

RUN curl --fail --location --retry 10 --retry-all-errors --retry-delay 2 \
      --output faiss-v1.7.3.tar.gz \
      https://codeload.github.com/facebookresearch/faiss/tar.gz/refs/tags/v1.7.3 \
    && echo "dda1b031957789e6a6f3b37d48ad135b9557a33c5013c1c245ef4bced2d787de  faiss-v1.7.3.tar.gz" \
      | sha256sum --check --status \
    && tar -xzf faiss-v1.7.3.tar.gz \
    && cmake -S faiss-1.7.3 -B build-faiss -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/d2-deps \
      -DBUILD_SHARED_LIBS=ON -DFAISS_ENABLE_GPU=OFF \
      -DFAISS_ENABLE_PYTHON=OFF -DFAISS_ENABLE_C_API=OFF \
      -DFAISS_OPT_LEVEL=generic -DBUILD_TESTING=OFF \
    && cmake --build build-faiss --parallel "${D2_BUILD_JOBS}" --target faiss \
    && cmake --install build-faiss

RUN curl --fail --location --retry 10 --retry-all-errors --retry-delay 2 \
      --output microsoft.ml.onnxruntime.1.18.1.nupkg \
      https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/1.18.1/microsoft.ml.onnxruntime.1.18.1.nupkg \
    && echo "e1fdeb5359407c948569905b4eb76af50b8bdb38b2bd3f2ad6516fe4a7186d6a  microsoft.ml.onnxruntime.1.18.1.nupkg" \
      | sha256sum --check --status \
    && mkdir -p /opt/onnxruntime/include /opt/onnxruntime/lib \
      /opt/onnxruntime/share/licenses \
    && unzip -q microsoft.ml.onnxruntime.1.18.1.nupkg \
      'build/native/include/*' \
      'runtimes/linux-x64/native/libonnxruntime.so' \
      LICENSE ThirdPartyNotices.txt -d onnxruntime-nuget \
    && cp -a onnxruntime-nuget/build/native/include/. /opt/onnxruntime/include/ \
    && cp onnxruntime-nuget/runtimes/linux-x64/native/libonnxruntime.so \
      /opt/onnxruntime/lib/ \
    && cp onnxruntime-nuget/LICENSE onnxruntime-nuget/ThirdPartyNotices.txt \
      /opt/onnxruntime/share/licenses/

COPY src/basic_function/state_estimator/d2_slam/dependencies/ceres_solver/ ceres/
COPY src/basic_function/state_estimator/d2_slam/dependencies/opengv/ opengv/
RUN cmake -S ceres -B build-ceres -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/d2-deps \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
      -DBUILD_SHARED_LIBS=OFF -DMINIGLOG=ON \
      -DCUDA=OFF -DCXSPARSE=OFF -DSUITESPARSE=OFF -DLAPACK=ON \
    && cmake --build build-ceres --parallel "${D2_BUILD_JOBS}" \
    && cmake --install build-ceres \
    && cmake -S opengv -B build-opengv -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/d2-deps \
      -DBUILD_TESTS=OFF -DBUILD_SHARED_LIBS=OFF \
    && cmake --build build-opengv --parallel "${D2_BUILD_JOBS}" \
    && cmake --install build-opengv

WORKDIR /opt/d2_slam_ws/src
COPY src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs/swarm_msgs/ swarm_msgs/
COPY src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs/swarmcomm_msgs/ swarmcomm_msgs/
COPY src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs/swarmtal_msgs/ swarmtal_msgs/
COPY src/basic_function/state_estimator/d2_slam/upstream/camera_models/ camera_models/
COPY src/basic_function/state_estimator/d2_slam/upstream/d2comm/ d2comm/
COPY src/basic_function/state_estimator/d2_slam/upstream/d2common/ d2common/
COPY src/basic_function/state_estimator/d2_slam/upstream/d2frontend/ d2frontend/
COPY src/basic_function/state_estimator/d2_slam/upstream/d2pgo/ d2pgo/
COPY src/basic_function/state_estimator/d2_slam/upstream/d2vins/ d2vins/
COPY src/basic_function/state_estimator/d2_slam/upstream/config/ config/
COPY src/basic_function/state_estimator/d2_slam/upstream/models/ models/
COPY src/basic_function/state_estimator/d2_slam/ros2_adapter/ uav_d2_slam_ros2/
COPY src/basic_function/state_estimator/d2_slam/plugin/ uav_d2_slam/
COPY simulator/airsim/sensors/ uav_as2_sensor_adapter/

WORKDIR /opt/d2_slam_ws
RUN source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && export CMAKE_BUILD_PARALLEL_LEVEL="${D2_BUILD_JOBS}" MAKEFLAGS="-j${D2_BUILD_JOBS}" \
    && colcon build --merge-install --executor sequential \
      --packages-select camera_models swarm_msgs swarmcomm_msgs swarmtal_msgs \
        d2common d2comm d2frontend d2pgo d2vins \
        uav_d2_slam_ros2 uav_d2_slam uav_as2_sensor_adapter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DCMAKE_PREFIX_PATH=/opt/d2-deps \
        -DONNXRUNTIME_INC_DIR=/opt/onnxruntime/include \
        -DONNXRUNTIME_LIB_DIR=/opt/onnxruntime/lib

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="AeroHub D2SLAM native ROS 2 runtime" \
      org.opencontainers.image.description="Self-contained ROS 2 Humble D2SLAM runtime"

RUN apt-get update && apt-get install -y --no-install-recommends \
      libopenblas0 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/d2-deps/ /opt/d2-deps/
COPY --from=builder /opt/onnxruntime/ /opt/onnxruntime/
COPY --from=builder /opt/d2_slam_ws/install/ /opt/d2_slam_ws/install/
COPY deployment/d2_slam/config/ /opt/d2_slam/config/
COPY deployment/d2_slam/ground_probe.py /opt/d2_slam/runtime/ground_probe.py
COPY deployment/d2_slam/flight_validation.py /opt/d2_slam/runtime/flight_validation.py
COPY deployment/d2_slam/validation.yaml /opt/d2_slam/runtime/validation.yaml
COPY simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py /opt/d2_slam/runtime/airsim_rpc.py
COPY deployment/airsim/as2/run.py /opt/d2_slam/runtime/run_as2.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/d2_slam/runtime/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/d2_slam/runtime/as2_config.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/d2_slam/runtime/pid.yaml
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/d2_slam/runtime/fastdds_udp.xml
COPY deployment/d2_slam/entrypoint.sh /usr/local/bin/aerohub-d2-slam

ENV LD_LIBRARY_PATH=/opt/onnxruntime/lib:/opt/d2-deps/lib \
    D2_ONNX_INTRA_OP_THREADS=3 \
    D2_SLAM_NAMESPACE=drone0 \
    D2_SLAM_SELF_ID=0 \
    D2_SLAM_MAIN_ID=0 \
    D2_SLAM_CONFIG=/opt/d2_slam/config/d2_slam.yaml \
    D2_SLAM_LCM_URI=udpm://224.0.0.251:7667?ttl=1 \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/d2_slam/runtime/fastdds_udp.xml

RUN chmod +x /usr/local/bin/aerohub-d2-slam \
    && test ! -e /opt/ros/noetic \
    && test ! -e /opt/d2_slam_ws/devel \
    && if grep -RIlE '/home/|/repo|/workspace|catkin_ws' \
         /opt/d2_slam_ws/install /opt/d2_slam/config /opt/d2_slam/runtime \
         > /tmp/host-path-hits; then \
         cat /tmp/host-path-hits; exit 1; \
       fi

ENTRYPOINT ["/usr/local/bin/aerohub-d2-slam"]
