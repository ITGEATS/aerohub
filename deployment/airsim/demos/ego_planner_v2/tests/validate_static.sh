#!/usr/bin/env bash
set -eo pipefail
cd /input
python3 -B deployment/airsim/demos/ego_planner_v2/tests/check_delivery.py /input
python3 -B -m unittest discover -s deployment/robopi/tests -p 'test_layout.py'
python3 -B -m unittest discover -s deployment/robopi/tests -p 'test_ego_layout.py'
python3 -B deployment/airsim/demos/ego_planner_v2/robopi/test_offline.py
for skill in uav-install-robopi-runtime uav-access-robopi-with-hdc uav-cross-compile-ros-for-ohos uav-package-pc-runtime uav-build-pc-runtime uav-validate-pc-runtime; do
  python3 -B /skill-validator.py /input/skill/$skill
done
for file in deployment/airsim/demos/ego_planner_v2/*.sh deployment/airsim/demos/ego_planner_v2/pc/*.sh deployment/airsim/demos/ego_planner_v2/tests/*.sh tools/robopi/build_lib.sh; do
  bash -n "$file"
done
python3 -B - <<'PY'
from pathlib import Path
import yaml
for p in [Path('cards/planner/ego-planner-v2.yaml'),Path('src/tool/navigation/planner/ego_planner_v2/component.yaml')]:
    data=yaml.safe_load(p.read_text());assert data['id']
    print('YAML valid:',p)
PY
