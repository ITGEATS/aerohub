#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source "${SPF_OVERLAY:-$repo/log/spf/pc-e2e/ros_ws/install}/setup.bash"
set -u
export AS2_CONFIG_FILE="$repo/deployment/airsim/config/as2/config.yaml"
export AS2_CONTROLLER_CONFIG_FILE="$repo/deployment/airsim/demos/spf_language_nav/controller.yaml"
export AS2_FLEET_FILE="${SPF_FLEET_FILE:?set SPF_FLEET_FILE}"
export AS2_EXECUTION_BACKEND=follow_path
export AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=0.5
unset AS2_TF_SCOPE
exec python3 "$repo/deployment/airsim/demos/spf_language_nav/spf_compute.py"
