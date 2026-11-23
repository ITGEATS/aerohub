"""Resolve the host inputs to the WestStreet Tool deployment."""
import json
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from simulator.manager.resources import resolve_resource


SCENE_RESOURCES = {
    'base_profile': 'projectairsim_course_profile',
    'ue_root': 'ue5_root',
    'project': 'projectairsim_weststreet_project',
}

DEFAULT_RUNTIME_CONFIG = Path(__file__).with_name('runtime.local.json')

OVERLAY_SETUP = {
    'navigation_workspace': 'install/setup.bash',
    'client_workspace': 'install/setup.bash',
    'system_workspace': 'install/setup.bash',
    'query_workspace': 'install/setup.bash',
    'tracking_workspace': 'install/setup.bash',
    'perception_workspace': 'install/setup.bash',
    'workstation_workspace': 'ws/install/setup.bash',
}


def load_runtime(path, *, require_acceptance=False):
    path = path.expanduser().resolve()
    config = json.loads(path.read_text())
    required = set(OVERLAY_SETUP)
    if require_acceptance:
        required.add('acceptance_workspace')
    if not isinstance(config, dict):
        raise ValueError(f'{path}: expected an object with runtime paths')
    missing = required - config.keys()
    if missing:
        raise ValueError(f"{path}: missing {', '.join(sorted(missing))}")

    paths = {key: resolve_resource(resource) for key, resource in SCENE_RESOURCES.items()
             if key not in config}
    for key in required | (config.keys() & {'acceptance_workspace', *SCENE_RESOURCES}):
        value = config[key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{path}: {key} must be a nonempty path')
        paths[key] = (path.parent / Path(value).expanduser()).absolute()

    inputs = {
        'base_profile': paths['base_profile'],
        'project': paths['project'],
        'ue_root': paths['ue_root'] / 'Engine/Binaries/Linux/UnrealEditor',
        **{key: paths[key] / setup for key, setup in OVERLAY_SETUP.items()},
    }
    if require_acceptance:
        acceptance = paths['acceptance_workspace']
        inputs['acceptance_manifest'] = acceptance / 'restored-source.json'
        inputs['acceptance_overlay'] = acceptance / 'install/local_setup.bash'
    for key, file in inputs.items():
        if not file.is_file():
            raise ValueError(f'{path}: {key} requires {file}')
    return paths
