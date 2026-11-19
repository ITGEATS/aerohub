#!/usr/bin/env bash
# New schema and every Tracking consumer, over a read-only matching detector overlay.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/target-tracking-follow/workspace"}
query=${2:-"$repo/log/object-detection-query/workspace"}
mkdir -p "$work"
work=$(cd "$work" && pwd)
query=$(cd "$query" && pwd)
docker run --rm --init --network none --entrypoint bash \
 -v "$repo:/repo:ro" -v "$work:/work" -v "$query:/query-work:ro" \
 onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /query-work/install/setup.bash
export PYTHONPATH=/query-work/python_deps${PYTHONPATH:+:$PYTHONPATH}
cd /work
colcon build --executor sequential --base-paths \
 /repo/src/tool/navigation/interfaces /repo/src/tool/target_tracking --event-handlers console_direct+
'
