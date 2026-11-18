"""Create an independent 2m-right junction candidate; preserve source assets."""
import os,runpy,traceback,json
from pathlib import Path
import unreal as u
root=Path(__file__).resolve().parent
out=Path(os.environ['WEST_SMOOTH_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
os.environ['WEST_ROUTE_VARIANT']='shift-right-2m'
try:
 runpy.run_path(str(root/'prepare_smooth_turn.py'),run_name='__main__')
 world=u.EditorLevelLibrary.get_editor_world()
 seq=u.load_asset('/Game/Aerohub/WestShiftRight2mMotion')
 actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
 controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
 data=controller.get_editor_property('binding_overrides').get_editor_property('binding_data')
 if len(data)!=1:raise RuntimeError('Expected one fresh runner override, got '+str(len(data)))
 (out/'generated-bindings.json').write_text(json.dumps(dict(map=world.get_name(),sequence=seq.get_path_name(),override_count=len(data),data=[x.export_text()for x in data]),indent=2))
except Exception:(out/'generation-error.txt').write_text(traceback.format_exc())
finally:u.SystemLibrary.execute_console_command(u.EditorLevelLibrary.get_editor_world(),'QUIT_EDITOR')
