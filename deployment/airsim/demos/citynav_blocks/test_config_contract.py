"""Offline contract tests for the citynav_blocks Demo configuration files.

Runs on the development host with plain pytest (no ROS, no Docker): the
settings/planner/controller files must agree with the interface contract
(src/tool/navigation/planner/citynav/README.md §5, log/citynav/contract/
result-20261117.md) and with the parameters the two ROS packages actually
declare, so a rename in one place cannot silently drift the Demo.
"""
import json
from pathlib import Path
import re

import yaml

APP = Path(__file__).resolve().parent
REPO = APP.parents[3]
PLANNER_NODE = REPO / ('src/tool/navigation/planner/citynav/ros2_adapter/'
                       'uav_citynav_planner/node.py')
CITY_CAMS = ['CityLeft', 'CitySlightlyLeft', 'CityFront',
             'CitySlightlyRight', 'CityRight']


def test_settings_five_eye_array():
    settings = json.loads((APP / 'settings.json').read_text())
    vehicle = settings['Vehicles']['PX4']
    assert vehicle['EnableCollisions'] is True
    assert set(vehicle['Cameras']) == set(CITY_CAMS)
    for cam in CITY_CAMS:
        camera = vehicle['Cameras'][cam]
        assert camera['X'] == 1, 'forward boom like FrontCamera convention'
        types = {c['ImageType']: c for c in camera['CaptureSettings']}
        assert set(types) == {0, 1}, 'rgb + DepthPlanar only'
        assert 2 not in types, 'contract forbids DepthPerspective'
        for capture in types.values():
            assert (capture['Width'], capture['Height']) == (256, 256)
            assert capture['FOV_Degrees'] == 90
    yaws = {cam: vehicle['Cameras'][cam]['Yaw'] for cam in CITY_CAMS}
    assert yaws == {'CityLeft': -90, 'CitySlightlyLeft': -45, 'CityFront': 0,
                    'CitySlightlyRight': 45, 'CityRight': 90}


def test_planner_yaml_matches_declared_parameters():
    params = yaml.safe_load((APP / 'planner.yaml').read_text())
    planner = params['/drone0/citynav_planner']['ros__parameters']

    declared = set(re.findall(r'declare_parameter\("([^"]+)"',
                              PLANNER_NODE.read_text(encoding='utf-8')))
    declared |= {'use_sim_time'}  # rclpy built-in
    unknown = set(planner) - declared
    assert not unknown, 'planner.yaml sets parameters the node does not declare: %s' % unknown

def test_camera_names_unified_across_roles():
    params = yaml.safe_load((APP / 'planner.yaml').read_text())
    planner = params['/drone0/citynav_planner']['ros__parameters']
    assert list(planner['view_cameras']) == CITY_CAMS


def test_locked_contract_values():
    params = yaml.safe_load((APP / 'planner.yaml').read_text())
    planner = params['/drone0/citynav_planner']['ros__parameters']
    assert planner['base_velocity'] == 1.0
    assert planner['base_yaw_rate'] == 30.0
    assert planner['min_altitude_m'] == 1.2
    assert planner['max_result_age_s'] == 90.0      # D21
    assert planner['view_freshness_s'] == 2.0
    assert planner['llm_model_name'] == 'vlm'
    assert 'grounding_url' not in planner and 'clip_url' not in planner
    inference = yaml.safe_load((APP / 'inference.yaml').read_text())
    workers = inference['/drone0/image_text_inference']['ros__parameters']
    assert workers['grounded_sam_endpoint'] == 'http://__HOST__:8100'
    assert workers['clip_endpoint'] == 'http://__HOST__:8101'
    assert planner['memory_graph_path'] == '/evidence/citynav_memory.json'


def test_controller_yaml_is_pid_speed_controller_tuning():
    params = yaml.safe_load((APP / 'controller.yaml').read_text())
    tuning = params['/**']['ros__parameters']['pid_speed_controller']
    # SPF-validated values retained; the 2026-11-18 z-oscillation study
    # (log/citynav/z-limcycle) proved the swing is PX4-EKF-driven and
    # gain-independent, so the demo mitigates via takeoff headroom +
    # velocity settle gate instead of retuning.
    assert tuning['position_control']['kp']['z'] == 1.0
    assert tuning['position_control']['ki']['z'] == 0.01
    assert tuning['trajectory_control']['kp']['x'] == 0.4


def test_compute_scripts_point_at_citynav_paths():
    script = (APP / 'citynav_compute.sh').read_text()
    assert 'citynav_blocks/controller.yaml' in script
    assert 'citynav_blocks/citynav_compute.py' in script
    assert 'AS2_EXECUTION_BACKEND=follow_path' in script
    supervisor = (APP / 'citynav_compute.py').read_text()
    assert 'uav_citynav_planner' in supervisor
    assert 'citynav_planner' in supervisor
