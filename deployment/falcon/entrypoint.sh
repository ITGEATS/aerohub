#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/falcon_ws/install/setup.bash

case "${1:-compute}" in
  compute)
    shift || true
    exec python3 /opt/falcon/runtime/compute_runtime.py "$@"
    ;;
  shell)
    shift
    exec bash "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
