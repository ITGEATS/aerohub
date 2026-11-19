#!/usr/bin/env bash
# Rebuild every affected schema and consumer together in the fixed existing image.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-"$repo/log/object-detection-query/workspace"}
mkdir -p "$work"
work=$(cd "$work" && pwd)
python3 "$repo/deployment/airsim/demos/super_forest/prepare_navigation_dependencies.py" "$work"
docker run --rm --init --network none --name object-detection-query-build \
 --entrypoint bash -v "$repo:/repo:ro" -v "$work:/work" \
 onboard_computer:v0.1.1 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
python3 - <<"PYDEPS"
import json,pathlib,zipfile
for item in json.loads(pathlib.Path("/repo/deployment/airsim/demos/super_forest/navigation_dependencies.json").read_text()):
    with zipfile.ZipFile(pathlib.Path("/work/wheels") / item["filename"]) as archive:
        archive.extractall("/work/python_deps")
PYDEPS
cd /work
colcon build --executor sequential --base-paths \
 /repo/as2/aerostack2/as2_python_api \
 /repo/src/basic_function/inference/interfaces /repo/src/basic_function/inference/ros2_adapter \
 /repo/src/tool/navigation/interfaces \
 /repo/src/tool/object_detection /repo/src/tool/target_tracking \
 --event-handlers console_direct+
'
