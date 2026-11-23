"""Check package uniqueness in the Elastic build scope."""
from collections import defaultdict
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[6]
COMPONENT = Path(__file__).resolve().parent.parent
BUILD_ROOTS = (
    COMPONENT / 'ros2_adapter',
    ROOT / 'src/basic_function/inference/interfaces',
    ROOT / 'src/basic_function/inference/ros2_adapter',
    ROOT / 'src/tool/navigation/interfaces',
    ROOT / 'src/tool/object_detection',
    ROOT / 'src/tool/target_tracking',
    ROOT / 'src/tool/system/motion_resource',
    ROOT / 'src/behavior/follow_path',
)
packages = defaultdict(list)
for base in BUILD_ROOTS:
    for path in sorted(base.rglob('package.xml')):
        name = ET.parse(path).getroot().findtext('name')
        packages[name].append(str(path.relative_to(ROOT)))
duplicates = {name: paths for name, paths in packages.items() if len(paths) > 1}
assert not duplicates, duplicates
assert {'elastic_tracker_planning', 'uav_target_tracking', 'uav_follow_path_execution'} <= packages.keys()
assert not (COMPONENT / 'ros1_proxy').exists()
print(json.dumps({'build_package_count': len(packages), 'duplicates': duplicates}, indent=2))
