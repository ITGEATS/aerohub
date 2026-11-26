# Requires BuildKit with named build contexts and RUN mounts.
ARG UV_IMAGE
ARG ONBOARD_IMAGE
FROM ${UV_IMAGE} AS uv
FROM ${ONBOARD_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=uv /uv /usr/local/bin/uv
RUN apt-get update && apt-get install -y --no-install-recommends python3-scipy python3-msgpack python3-yaml python3-numpy \
    && rm -rf /var/lib/apt/lists/* \
    && chmod o+x /root
WORKDIR /repo
COPY src/basic_function/inference/interfaces src/basic_function/inference/interfaces
COPY src/basic_function/inference/ros2_adapter src/basic_function/inference/ros2_adapter
COPY src/basic_function/inference/runtime src/basic_function/inference/runtime
COPY src/basic_function/inference/models/flowpilot src/basic_function/inference/models/flowpilot
COPY src/tool/navigation/interfaces src/tool/navigation/interfaces
COPY src/tool/navigation/python_api src/tool/navigation/python_api
COPY src/tool/navigation/planner/flowpilot src/tool/navigation/planner/flowpilot
COPY src/behavior/follow_path src/behavior/follow_path
COPY simulator/airsim/sensors simulator/airsim/sensors
COPY deployment/airsim/as2 deployment/airsim/as2
COPY deployment/airsim/config/as2 deployment/airsim/config/as2
COPY deployment/airsim/demos/flowpilot_single deployment/airsim/demos/flowpilot_single
COPY deployment/swarm/overhead_view.py deployment/swarm/view.py deployment/swarm/
COPY deployment/inference/flowpilot/entrypoint.sh /opt/flowpilot/entrypoint.sh
COPY deployment/inference/tests/test_flowpilot_*.py deployment/inference/tests/
COPY deployment/inference/tests/flowpilot* deployment/inference/tests/
COPY --from=flowpilot_weights / /models/
RUN test -f src/tool/navigation/planner/flowpilot/upstream/inference/runtime.py \
    && source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && if [[ -f /opt/px4_ws/install/setup.bash ]]; then source /opt/px4_ws/install/setup.bash; fi \
    && mkdir -p /opt/flowpilot/ros_ws && cd /opt/flowpilot/ros_ws \
    && colcon build --base-paths /repo/src/basic_function/inference/interfaces /repo/src/basic_function/inference/ros2_adapter /repo/src/tool/navigation/interfaces /repo/src/tool/navigation/python_api /repo/src/tool/navigation/planner/flowpilot/ros2_adapter /repo/src/behavior/follow_path /repo/simulator/airsim/sensors \
       --packages-select uav_inference_interfaces uav_inference uav_planner_interfaces uav_planner_python_api uav_flowpilot_planner uav_follow_path_execution uav_as2_sensor_adapter \
       --cmake-args -DBUILD_TESTING=OFF
ENV UV_PROJECT_ENVIRONMENT=/opt/flowpilot/model-venv
RUN --mount=type=bind,from=uv,target=/flowpilot-tools,rw \
    if [[ -x /flowpilot-tools/python/bin/python3.11 ]]; then \
      cp -a /flowpilot-tools/python /opt/flowpilot/python; fp_python=/opt/flowpilot/python/bin/python3.11; \
    else fp_python=3.11; fi \
    && if [[ -d /flowpilot-tools/uv-cache/archive-v0 ]]; then \
      UV_CACHE_DIR=/flowpilot-tools/uv-cache UV_LINK_MODE=copy uv sync --locked --offline --python "$fp_python" --project /repo/src/basic_function/inference/models/flowpilot; \
    else uv sync --locked --python "$fp_python" --project /repo/src/basic_function/inference/models/flowpilot; fi \
    && chmod +x /opt/flowpilot/entrypoint.sh
RUN cd /repo/src/basic_function/inference/models/flowpilot \
    && /opt/flowpilot/model-venv/bin/python -c "from weights import verify_weights; verify_weights('/models', 'weights.sha256')"
RUN source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && source /opt/flowpilot/ros_ws/install/setup.bash \
    && python3 -c "from uav_inference.flowpilot_node import FlowPilotService; from uav_flowpilot_planner.node import FlowPilotPlanner; from uav_planner_interfaces.action import NavigateToPose" \
    && python3 -B -m unittest discover -s deployment/inference/tests -p test_flowpilot_contract.py
ENV FLOWPILOT_WEIGHTS_DIR=/models AS2_EXECUTION_BACKEND=follow_path AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=0.5
ENTRYPOINT ["/opt/flowpilot/entrypoint.sh"]
CMD ["compute"]
