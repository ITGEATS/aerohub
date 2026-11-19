#!/usr/bin/env python3
"""SDK-free client for simulated runner start/stop/reset (not UAV control)."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient

class SceneControlClient(ProjectAirSimExtensionClient):
    def __init__(self,host='127.0.0.1',port=19892,*,timeout=3):
        super().__init__(host=host,scene_port=port,timeout=timeout)
    def call(self,method,**params):return self.call_scene(method,**params)
    def state(self):return self.get_actor_state('runner')
    def control(self,event):
        if event not in ('start','stop','reset'):raise ValueError('Expected start/stop/reset')
        return self.control_actor('runner',event)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('event',choices=['state','start','stop','reset']);p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=19892)
    a=p.parse_args();client=SceneControlClient(a.host,a.port)
    print(json.dumps(client.state() if a.event=='state' else client.control(a.event),indent=2))
