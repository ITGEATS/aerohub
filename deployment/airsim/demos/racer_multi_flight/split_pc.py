"""PC placement checks for the RACER development deployment."""
import json
from pathlib import Path
import subprocess
import sys


def resolve_placement(repo, fleet, image):
    from deployment.swarm.topology import load_config, resolve_topology

    config = load_config(repo / 'deployment/swarm/config.example.yaml')
    vehicles = fleet['vehicles']
    config['deployment']['stage'] = 'pc_simulation'
    config['coordination'] = {'mode': 'decentralized'}
    config['fleet']['vehicle_count'] = len(vehicles)
    config['nodes']['onboard_computer']['image'] = image
    first = vehicles[0]
    config['airsim']['ports'].update(px4_instance_base=first['px4_instance'],
                                     px4_system_id_base=first['px4_system_id'],
                                     xrce_agent_base=first['xrce_port'])
    topology = resolve_topology(config)
    selected = topology['onboard_computer_vehicles']
    if topology['board_vehicle'] is not None or len(selected) != len(vehicles):
        raise ValueError('expected one PC compute container per fleet member')
    for v, target in zip(vehicles, selected):
        identity = (v['index'], v['namespace'], v['airsim'], v['px4_instance'], v['px4_system_id'], v['xrce_port'])
        resolved = tuple(target[k] for k in ('index','name','airsim_name','px4_instance','px4_system_id','xrce_agent_port'))
        if resolved != identity:
            raise ValueError(f'swarm placement differs from RACER vehicle {identity}')
    topology['scene_settings_source'] = 'Selected RACER settings; swarm topology supplies compute placement only'
    return topology


def check_initial_poses(repo, local, fleet):
    sys.path[:0] = [str(repo/'simulator/airsim/sensors'), str(repo/'deployment/robopi/vendor')]
    from uav_as2_sensor_adapter.rpc import Rpc

    rpc = Rpc(local.get('airsim_probe_host', '127.0.0.1'), local['rpc_port'], 5)
    settings = json.loads(Path(local['settings']).read_text(encoding='utf-8'))
    if json.loads(rpc.call('getSettingsString')) != settings:
        raise ValueError('live AirSim settings differ from the selected file')
    names = {v['airsim'] for v in fleet['vehicles']}
    if set(rpc.call('listVehicles')) != names:
        raise ValueError('live AirSim fleet differs from selected fleet')
    for name in names:
        pos = rpc.call('simGetVehiclePose', name)['position']
        if max(abs(pos[k]) for k in ('x_val', 'y_val')) >= 0.25:
            raise ValueError(f'{name} is displaced from its initial position; check scene session')


def verify_processes(deployment):
    rows = {}
    members = [v['namespace'] for v in deployment.fleet['vehicles']]
    for ns in members:
        role = 'native-'+ns
        name = deployment.prefix+'-'+role
        out = deployment.root/role
        info = json.loads(subprocess.check_output(['docker','inspect',name], text=True))[0]
        workers = json.loads((out/'worker-topology.json').read_text())
        processes = subprocess.check_output(['docker','top',name,'-eo','pid,args'], text=True)
        (out/'processes.log').write_text(processes, encoding='utf-8')
        nodes = []
        for line in processes.splitlines():
            fields = line.split()
            if len(fields) > 1 and Path(fields[1]).name == 'exploration_node':
                nodes.extend(arg for arg in fields[2:] if arg.startswith('__node:='))
        if (not info['State']['Running'] or info['Image'] != deployment.images['algorithm']
                or info['Config']['Labels'].get('aerohub.racer.owner') != deployment.prefix
                or len(workers) != 1 or workers[0]['identity'] != ns or not workers[0]['pid']
                or nodes != ['__node:=racer_'+ns]):
            raise RuntimeError(f'unexpected worker identity in {name}; see {out}')
        rows[ns] = dict(container=info['Name'], id=info['Id'], image=info['Image'], worker=workers[0])
    if len({r['id'] for r in rows.values()}) != len(members):
        raise RuntimeError('native workers share a container')
    (deployment.root/'verified-placement.json').write_text(json.dumps(rows, indent=2))
