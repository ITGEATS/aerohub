"""Plot the actual recorded scene trajectories; offline, no simulator calls."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/evidence/mission-short')
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
starts=[]
for i in (0,1):
    rows=[json.loads(line) for line in (root/f'drone{i}/flight.jsonl').read_text().splitlines()]
    data=[r for r in rows if 'scene_pose' in r]
    time=np.array([r['sim'] for r in data]);starts.append(time[0])
    xyz=np.array([[r['scene_pose']['position']['y_val']-4*i,r['scene_pose']['position']['x_val'],-r['scene_pose']['position']['z_val']] for r in data])
    color=('tab:blue','tab:orange')[i];label=('RoboPi Drone0','PC Drone1')[i]
    axes[0].plot(xyz[:,0],xyz[:,1],color=color,label=label)
    axes[0].scatter(xyz[0,0],xyz[0,1],color=color,marker='o')
    axes[0].scatter(xyz[-1,0],xyz[-1,1],color=color,marker='x')
    axes[1].plot(time-min(starts),xyz[:,2]-xyz[0,2],color=color,label=label)
axes[0].set(xlabel='World ENU east (m)',ylabel='World ENU north (m)',title='Actual scene track: short execution')
axes[0].axis('equal')
axes[1].set(xlabel='Simulation time since client observation (s)',ylabel='Height above initial scene position (m)',title='Takeoff, short execution, native landing')
for ax in axes:ax.grid(alpha=.3);ax.legend()
fig.savefig(root/'flight-tracks.png',dpi=160)
