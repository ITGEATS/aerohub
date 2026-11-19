#!/usr/bin/env python3
"""Real scene RPC check for the runner ramp; no UAV commands or truth feedback."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=19892)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    client=ProjectAirSimExtensionClient(scene_port=args.port)
    report=dict(passed=False,samples=[])
    try:
        initial=client.reset_actor()
        assert initial['state']=='paused'
        start=client.start_actor()
        duration=start['startup_ramp_seconds']
        assert duration>0 and start['starting'] and start['play_rate']==0
        began=time.monotonic();repeated=False
        while time.monotonic()-began<duration+1.:
            state=client.get_actor_state()
            state['elapsed_wall_s']=time.monotonic()-began
            report['samples'].append(state)
            if not repeated and state['play_rate']>.1:
                before=state['play_rate'];again=client.start_actor()
                assert again['play_rate']>=before,'Repeated start restarted ramp'
                repeated=True
            time.sleep(.04)
        samples=report['samples'];rates=[s['play_rate']for s in samples]
        assert repeated and all(b>=a for a,b in zip(rates,rates[1:])), 'Ramp not monotonic'
        assert any(.1<r<.9 for r in rates) and rates[-1]==1 and not samples[-1]['starting']
        cruise=[s['speed_mps']for s in samples if s['play_rate']==1]
        assert len(cruise)>=5 and abs(statistics.median(cruise)-5.)<.5,'Cruise speed not 5m/s'
        stopped=client.stop_actor();time.sleep(.2);frozen=client.get_actor_state()
        xyz=lambda s:[s['location_ue_cm'][k]for k in ('x','y','z')]
        assert math.dist(xyz(stopped),xyz(frozen))<.1 and frozen['state']=='paused'
        resumed=client.start_actor();assert resumed['starting'] and resumed['play_rate']==0
        client.reset_actor();reset=client.get_actor_state()
        assert not reset['starting'] and reset['state']=='paused' and abs(reset['time_seconds'])<.001
        report.update(passed=True,cruise_speed_median_mps=statistics.median(cruise),
            startup_ramp_seconds=duration,checks=['smooth rates','5m/s actual cruise','idempotent start','stop freeze','resume ramp','reset'])
    finally:
        client.stop_actor()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items()if k!='samples'}))


if __name__=='__main__':main()
