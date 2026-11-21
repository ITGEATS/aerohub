#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source "${CITYNAV_OVERLAY:-$repo/log/citynav/pc-e2e/ros_ws/install}/setup.bash"
set -u
export AS2_CONFIG_FILE="$repo/deployment/airsim/config/as2/config.yaml"
export AS2_CONTROLLER_CONFIG_FILE="$repo/deployment/airsim/demos/citynav_blocks/controller.yaml"
export AS2_FLEET_FILE="${CITYNAV_FLEET_FILE:?set CITYNAV_FLEET_FILE}"
export AS2_EXECUTION_BACKEND=follow_path
export AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=0.5
unset AS2_TF_SCOPE
exec python3 "$repo/deployment/airsim/demos/citynav_blocks/citynav_compute.py"
