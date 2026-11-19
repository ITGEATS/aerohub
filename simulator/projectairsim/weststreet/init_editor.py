"""Initialize street native GameMode and a trusted local editor request service."""
import json,os,runpy,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_INTEGRATION_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
u.EditorLoadingAndSavingUtils.load_map('/Game/Aerohub/WestStreetCircuit')
world=u.EditorLevelLibrary.get_editor_world()
world.get_world_settings().set_editor_property('default_game_mode',u.load_class(None,'/Script/ProjectAirSim.ProjectAirSimGameMode'))
u.EditorLoadingAndSavingUtils.save_current_level()
u.get_editor_subsystem(u.LevelEditorSubsystem).eject_pilot_level_actor()
last=None;busy=False

def tick(delta):
 global last,busy
 if busy:return
 path=out/'editor-request.json'
 if not path.exists():return
 job=json.loads(path.read_text())
 if job['id']==last:return
 last=job['id'];busy=True
 try:runpy.run_path(job['script'],run_name='__main__')
 except Exception:(out/'editor-request-error.txt').write_text(traceback.format_exc())
 finally:busy=False
handle=u.register_slate_post_tick_callback(tick)
(out/'editor-ready.json').write_text(json.dumps({'map':world.get_name(),'native_game_mode':str(world.get_world_settings().get_editor_property('default_game_mode'))}))
