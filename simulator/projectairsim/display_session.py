#!/usr/bin/env python3
"""Attach display observations to a scene; --load explicitly loads the given scene."""
import argparse
import json
from pathlib import Path
import sys
import time
from urllib.request import Request, ProxyHandler, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulator.projectairsim.image_windows import ImageWindow, ImageWindows
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient, DisplayRPC

COMPONENTS = {
    0: ('RGB', 'SceneCaptureComponent', 'scene_camera'),
    1: ('DepthPlanar', 'DepthPlanarCaptureComponent', 'depth_planar_camera'),
    2: ('DepthPerspective', 'DepthPerspectiveCaptureComponent', 'depth_camera'),
    3: ('Segmentation', 'SegmentationCaptureComponent', 'segmentation_camera'),
    4: ('DepthVis', 'DepthVisCaptureComponent', 'depth_vis_camera'),
    5: ('DisparityNormalized', 'DisparityNormalizedCaptureComponent', 'disparity_normalized_camera'),
    6: ('SurfaceNormals', 'SurfaceNormalsCaptureComponent', 'surface_normals_camera'),
}


def views_from_scene(scene, *, include_captured_views=False):
    """Adapt locked native sensor captures to public UE component identities."""
    views, images = [], []
    for actor in scene.get('actors', []):
        if actor.get('type') != 'robot':
            continue
        robot = actor['robot-config']
        if not isinstance(robot, dict):
            raise ValueError('Expected expanded native robot configuration')
        for sensor in robot.get('sensors', []):
            if sensor.get('type') != 'camera' or not sensor.get('enabled', False):
                continue
            for capture in sensor.get('capture-settings', []):
                kind = capture['image-type']
                if kind not in COMPONENTS:
                    continue
                label, component, topic = COMPONENTS[kind]
                identity = dict(vehicle_id=actor['name'], camera_id=sensor['id'],
                                image_type=label, component=component)
                if capture.get('streaming-enabled', False) or (include_captured_views and capture.get('capture-enabled',False)):
                    views.append(identity)
                if capture.get('capture-enabled', False):
                    source = f"/Sim/{scene['id']}/robots/{actor['name']}/sensors/{sensor['id']}/{topic}"
                    images.append(ImageWindow(f"{actor['name']} · {sensor['id']} · {label}", source))
    return views, images



def main():
    from projectairsim import ProjectAirSimClient, World
    from projectairsim.utils import load_scene_config_as_dict
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--load', action='store_true')
    parser.add_argument('--topic-port', type=int, default=8989)
    parser.add_argument('--service-port', type=int, default=8990)
    parser.add_argument('--display-port', type=int, default=8991)
    parser.add_argument('--no-windows', action='store_true')
    args = parser.parse_args()
    scene, _ = load_scene_config_as_dict(args.scene.name, str(args.scene.resolve().parent))
    client = ProjectAirSimClient(port_topics=args.topic_port, port_services=args.service_port)
    # This session owns the topic connection; do not run alongside a bridge owning Pair0.
    client.connect()
    client.socket_services.recv_timeout = 3000
    windows = ImageWindows(window_size=(480, 270))
    telemetry_open = build_opener(ProxyHandler({})).open
    try:
        if args.load:
            world = World(client, args.scene.name, sim_config_path=str(args.scene.resolve().parent),
                          delay_after_load_sec=1)
            scene = world.get_configuration()
        views, images = views_from_scene(scene)
        rpc = ProjectAirSimExtensionClient(port=args.display_port)
        rpc.call('RegisterViews', views=views)
        # Code-level initial choice; runtime callers may replace/close this list.
        selected = [item for item in images if 'FrontCamera' in item.source
                    and ('scene_camera' in item.source or '/depth_camera' in item.source)][:2]
        if not args.no_windows:
            windows.show(selected)
        for item in images:
            client.subscribe(item.source,
                lambda _, image, source=item.source: windows.receive(source, image))
        vehicles = [a['name'] for a in scene['actors'] if a.get('type') == 'robot']
        print('Display session ready. Runtime HUD control: POST /aerohub/rpc.', flush=True)
        print('Main-window Tab/G remain observation shortcuts; no interactive terminal commands.', flush=True)
        while True:
            for vehicle in vehicles:
                try:
                    state = client.request({'method':f"/Sim/{scene['id']}/robots/{vehicle}/GetGroundTruthKinematics",
                                            'params':{}, 'version':1.0})
                    velocity = state['twist']['linear']
                    speed = sum(velocity[a]**2 for a in ('x','y','z'))**.5
                    payload = json.dumps({'vehicle':vehicle, 'speed_mps':speed}).encode()
                    with telemetry_open(Request(f'http://127.0.0.1:{args.display_port}/aerohub/telemetry',
                                          data=payload, headers={'Content-Type':'application/json'}), timeout=2) as response:
                        response.read()
                except (OSError, RuntimeError) as error:
                    print(f'{vehicle}: {error}', flush=True)
            time.sleep(.1)
    finally:
        windows.close()
        client.disconnect()


if __name__ == '__main__':
    main()
