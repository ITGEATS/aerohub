ARG UV_CACHE_IMAGE=scratch
FROM ${UV_CACHE_IMAGE} AS uv_cache_seed
FROM ghcr.io/astral-sh/uv:0.9.9@sha256:f6e3549ed287fee0ddde2460a2a74a2d74366f84b04aaa34c1f19fec40da8652 AS uv
FROM ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9 AS standalone
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/yopo-tracker
ENV UV_PYTHON_DOWNLOADS=never UV_LINK_MODE=copy UV_PROJECT_ENVIRONMENT=/opt/yopo-tracker/venv

COPY --from=uv /uv /usr/local/bin/uv
COPY pyproject.toml uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv,sharing=locked \
    --mount=type=bind,from=uv_cache_seed,target=/seed \
    if [ -d /seed/root/.cache/uv ]; then cp -an /seed/root/.cache/uv/. /root/.cache/uv/; fi && \
    uv venv --system-site-packages --python /usr/bin/python3 "$UV_PROJECT_ENVIRONMENT" && \
    uv sync --locked --no-dev --no-install-project --python /usr/bin/python3

COPY upstream/Controller/src/utils/quadrotor_msgs /opt/yopo-tracker/interfaces/quadrotor_msgs
COPY planner_interfaces /opt/yopo-tracker/interfaces/uav_planner_interfaces
COPY inference_interfaces /opt/yopo-tracker/interfaces/uav_inference_interfaces
COPY inference_ros /opt/yopo-tracker/interfaces/uav_inference
RUN source /opt/ros/humble/setup.bash && \
    colcon --log-base /tmp/yopo-build-log build --base-paths interfaces \
      --build-base /tmp/yopo-build --install-base /opt/yopo-tracker/install \
      --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

COPY upstream/YOPO upstream/YOPO
COPY upstream/LICENSE upstream/LICENSE
RUN if [[ -d upstream/YOPO/saved ]]; then \
      echo 'a625490468cb8734867e51d2af2be68a8182bcbbbb1e0401b1d2a32e6ccd5f7f  upstream/YOPO/saved/YOPO_1/epoch50.pth' | sha256sum -c -; \
    fi

COPY ros2_adapter ./ros2_adapter
COPY entrypoint.sh brake-calibration.sitl.json ./
ENV YOPO_TRACKER_PYTHON=/opt/yopo-tracker/venv/bin/python
ENTRYPOINT ["bash", "/opt/yopo-tracker/entrypoint.sh"]

# Local verification can share the existing CUDA worker's image layers.
FROM uv_cache_seed AS reuse
WORKDIR /opt/yopo-tracker
COPY --from=standalone /opt/yopo-tracker/install ./install
COPY --from=standalone /opt/yopo-tracker/upstream ./upstream
COPY --from=standalone /opt/yopo-tracker/venv/lib/python3.10/site-packages/ruamel ./addons/ruamel
COPY --from=standalone /opt/yopo-tracker/venv/lib/python3.10/site-packages/_ruamel_yaml*.so ./addons/
COPY ros2_adapter ./ros2_adapter
COPY entrypoint.sh brake-calibration.sitl.json ./
ENV PYTHONPATH=/opt/yopo-tracker/addons:/usr/lib/python3/dist-packages \
    YOPO_TRACKER_PYTHON=/opt/vision/bin/python
ENTRYPOINT ["bash", "/opt/yopo-tracker/entrypoint.sh"]

FROM standalone AS release
