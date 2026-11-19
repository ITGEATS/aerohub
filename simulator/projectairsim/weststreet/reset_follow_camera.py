"""Reset the existing follow camera and leave editor navigation unpiloted."""
import json,os,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_RUNNER_OUTPUT'])
try:
 editor=u.get_editor_subsystem(u.LevelEditorSubsystem)
 if editor.is_in_play_in_editor():raise RuntimeError('Stop Play before changing saved camera')
 editor.eject_pilot_level_actor()
 actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
 runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
 camera=next(a for a in actors if a.get_actor_label()=='West_Circuit_FollowCamera')
 offset=u.Vector(160,-600,320)
 camera.set_actor_relative_location(offset,False,False)
 camera.set_actor_relative_rotation(u.MathLibrary.find_look_at_rotation(offset,u.Vector(0,170,100)),False,False)
 camera.camera_component.set_editor_property('field_of_view',75)
 loc=camera.get_actor_location();rot=camera.get_actor_rotation()
 target=runner.get_actor_location()+u.Vector(0,0,100)
 d=target-loc;f=rot.get_forward_vector()
 dot=(d.x*f.x+d.y*f.y+d.z*f.z)/d.length()
 if dot<.95:raise RuntimeError('Camera is not looking at runner: '+str(dot))
 u.EditorLoadingAndSavingUtils.save_current_level()
 u.EditorLevelLibrary.set_level_viewport_camera_info(loc,rot)
 (out/'camera-reset.json').write_text(json.dumps({'target_forward_dot':dot,'offset_cm':[160,-600,320],'editor_pilot':str(editor.get_pilot_level_actor()),'status':'saved; actual Play screenshot verification pending'},indent=2))
except Exception:
 (out/'camera-reset-error.txt').write_text(traceback.format_exc());raise
