#!/usr/bin/env bash
set -eo pipefail
cd /opt/yopo-following
source /opt/ros/humble/setup.bash
source install/local_setup.bash
exec /usr/bin/python3 following_action.py "$@"
