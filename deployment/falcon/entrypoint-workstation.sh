#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/falcon_workstation_ws/install/setup.bash

case "${1:-mission}" in
  mission)
    shift || true
    exec python3 /opt/falcon/runtime/mission.py "$@"
    ;;
  shell)
    shift || true
    exec bash "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
