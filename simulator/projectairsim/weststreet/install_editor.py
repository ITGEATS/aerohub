#!/usr/bin/env python3
"""Install existing matching UE plugins into a street project; no Cook/package."""
import argparse,configparser,json,shutil
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
for name in ['ue-root','project','native-project','hud-package','evidence']:p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--display-port',type=int,default=19891)
a=p.parse_args();root=a.project.resolve().parent;a.evidence.mkdir(parents=True,exist_ok=True)
engine_id=json.loads((a.ue_root/'Engine/Binaries/Linux/UnrealEditor.modules').read_text())['BuildId']
for name,source in [('ProjectAirSim',a.native_project.parent/'Plugins/ProjectAirSim'),('Drone',a.native_project.parent/'Plugins/Drone'),('AerohubDisplay',a.hud_package)]:
 manifest=source/'Binaries/Linux/UnrealEditor.modules'
 if manifest.exists() and json.loads(manifest.read_text())['BuildId']!=engine_id:raise RuntimeError('UE BuildId mismatch: '+name)
 target=root/'Plugins'/name
 if target.exists():raise RuntimeError('Refusing to overwrite installed plugin: '+str(target))
 shutil.copytree(source,target,symlinks=True,ignore=shutil.ignore_patterns('Intermediate','Saved','HostProject'))
project=json.loads(a.project.read_text());(a.evidence/'project-before.json').write_text(json.dumps(project,indent=2))
plugins=project.setdefault('Plugins',[])
for name in ['ProjectAirSim','Drone','AerohubDisplay']:
 match=next((entry for entry in plugins if entry['Name']==name),None)
 if match:match['Enabled']=True
 else:plugins.append({'Name':name,'Enabled':True})
a.project.write_text(json.dumps(project,indent=2)+'\n')
path=root/'Config/DefaultEngine.ini';text=path.read_text();(a.evidence/'DefaultEngine-before.ini').write_text(text)
section='[/Script/EngineSettings.GameMapsSettings]'
start=text.index(section)+len(section);end=text.find('\n[',start)
if end<0:end=len(text)
lines=text[start:end].splitlines()
values={'GameDefaultMap':'/Game/Aerohub/WestStreetCircuit','EditorStartupMap':'/Game/Aerohub/WestStreetCircuit','GlobalDefaultGameMode':'/Script/ProjectAirSim.ProjectAirSimGameMode'}
lines=[line for line in lines if not any(line.startswith(key+'=') for key in values)]
text=text[:start]+'\n'+'\n'.join(lines+[key+'='+value for key,value in values.items()])+'\n'+text[end:]
path.write_text(text)
config=root/'Saved/Aerohub/display.json';config.parent.mkdir(parents=True,exist_ok=True)
if config.exists():shutil.copy2(config,a.evidence/'display-before.json')
config.write_text(json.dumps({'enabled':True,'vehicle':'Drone1','speed_max_mps':20,'port':a.display_port,'global_camera_tag':'WestRunnerGlobalFollow'},indent=2))
print(json.dumps({'project':str(a.project),'ue_build_id':engine_id,'plugins':['ProjectAirSim','Drone','AerohubDisplay'],'display_port':a.display_port}))
