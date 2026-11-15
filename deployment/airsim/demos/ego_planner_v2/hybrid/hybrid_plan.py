"""Generate EGO artifacts using the shared swarm schema; no device operations."""
import copy
import hashlib
import importlib.util
import importlib
import json
from pathlib import Path
import sys


def shared_modules(root):
    package=Path(root)/'deployment/swarm'
    spec=importlib.util.spec_from_file_location('ego_shared_swarm',package/'__init__.py',
        submodule_search_locations=[str(package)])
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return importlib.import_module('ego_shared_swarm.topology'),importlib.import_module('ego_shared_swarm.view')


def resolve(config, snapshot, shared_root):
    topology_module,view=shared_modules(shared_root)
    topology=topology_module.resolve_topology(config)
    if topology['deployment_stage']!='hybrid_board' or topology['vehicle_count']!=2 or topology['board_vehicle']['name']!='drone0' or topology['coordination_mode']!='decentralized':
        raise ValueError('This acceptance entry requires decentralized RoboPi drone0 + PC drone1')
    settings=copy.deepcopy(snapshot)
    fleet={}
    for vehicle in topology['vehicles']:
        name=vehicle['name'];i=vehicle['index']
        if name!=f'drone{i}' or vehicle['airsim_name']!=f'Drone{i}':raise ValueError('Unsupported vehicle naming')
        source=snapshot['Vehicles'][vehicle['airsim_name']]
        if source['VehicleSysID']!=vehicle['px4_system_id']:raise ValueError('PX4 system ID mismatch')
        for key,base in [('TcpPort','tcp_base'),('ControlPortLocal','control_local_base'),('ControlPortRemote','control_remote_base')]:
            if source[key]!=config['airsim']['ports'][base]+i:raise ValueError('Snapshot port mismatch: '+key)
        vehicle['initial_xy_m']=[source['X'],source['Y']]
        vehicle['origin_ned_m']=[source[k] for k in ('X','Y','Z')]
        fleet[name]=dict(namespace=name,px4_origin_in_fleet_ned=vehicle['origin_ned_m'])
    if set(snapshot['Vehicles'])!={'Drone0','Drone1'}:raise ValueError('Expected exact two-vehicle snapshot')
    settings=view.apply_overhead_view(settings,config['airsim']['camera'])
    return dict(topology=topology,settings=settings,
        ego=dict(task={'task_id':'assigned_per_session'},vehicles=list(fleet),planner_ids={'drone0':0,'drone1':1},
            deployment=fleet,peer_body_mask_radius_m=.75,
            limits={'velocity':.23,'acceleration':.35,'tolerance':.05},
            planner={'parameters':{'valid_from_lead':.4,'sample_dt':.1}}),
        shared_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(shared_root)/'deployment/swarm').glob('*.py')})


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('config','snapshot','shared_root','output'):parser.add_argument('--'+key.replace('_','-'),type=Path,required=True)
    args=parser.parse_args()
    result=resolve(json.loads(args.config.read_text()),json.loads(args.snapshot.read_text()),args.shared_root)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
