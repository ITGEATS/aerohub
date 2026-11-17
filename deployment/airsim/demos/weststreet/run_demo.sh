#!/usr/bin/env bash
# Run inside the workstation role after deploying the matching onboard Tools.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
: "${DEMO_CLIENT_OVERLAY:?Set the built client workspace install directory}"
set +u
source "$DEMO_CLIENT_OVERLAY/setup.bash"
set -u
export PYTHONPATH="$DEMO_CLIENT_OVERLAY/../python_deps${PYTHONPATH:+:$PYTHONPATH}"
for package in uav_system_python_api uav_planner_python_api uav_target_tracking_python_api; do
  prefix=$(ros2 pkg prefix "$package")
  export AS2_MODULES_PATH="$prefix/share/$package/modules${AS2_MODULES_PATH:+:$AS2_MODULES_PATH}"
done
exec python3 "$repo/deployment/airsim/demos/weststreet/demo.py" "$@"
