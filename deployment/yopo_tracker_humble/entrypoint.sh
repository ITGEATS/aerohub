#!/usr/bin/env bash
set -eo pipefail

cd /opt/yopo-tracker
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/yopo-tracker/install/local_setup.bash

module=ros2_adapter.bridge.live_node
if [[ "${1:-}" == --native-diagnostics ]]; then
    shift
    module=ros2_adapter.native.native_node
fi
exec "$YOPO_TRACKER_PYTHON" -u -m "$module" "$@"
