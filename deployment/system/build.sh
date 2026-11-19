#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/system-tool/workspace"}
mkdir -p "$work"
work=$(cd "$work" && pwd)
docker run --rm --init --network none --entrypoint bash \
 -v "$repo:/repo:ro" -v "$work:/work" onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
cd /work
colcon build --executor sequential --base-paths /repo/src/tool/system --event-handlers console_direct+
'
