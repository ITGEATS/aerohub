"""Persist a sequence camera following the runner; original straight map is untouched."""
import json,os,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_RUNNER_OUTPUT'])
try:
 subsystem=u.get_editor_subsystem(u.EditorActorSubsystem)
 actors=subsystem.get_all_level_actors()
 if u.EditorLevelLibrary.get_editor_world().get_name()!='WestStreetCircuit':raise RuntimeError('Wrong map')
 runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
 controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
 existing=[a for a in actors if a.get_actor_label()=='West_Circuit_FollowCamera']
 if existing:raise RuntimeError('Camera already exists')
 camera=subsystem.spawn_actor_from_class(u.CameraActor,runner.get_actor_location())
 camera.set_actor_label('West_Circuit_FollowCamera')
 camera.attach_to_actor(runner,'None',u.AttachmentRule.KEEP_RELATIVE,u.AttachmentRule.KEEP_RELATIVE,u.AttachmentRule.KEEP_RELATIVE,False)
 offset=u.Vector(160,-600,320)
 camera.set_actor_relative_location(offset,False,False)
 camera.set_actor_relative_rotation(u.MathLibrary.find_look_at_rotation(offset,u.Vector(0,170,100)),False,False)
 camera.camera_component.set_editor_property('field_of_view',75)
 seq=controller.get_sequence();binding=seq.add_possessable(camera)
 track=seq.add_master_track(u.MovieSceneCameraCutTrack)
 section=track.add_section();section.set_range(0,json.loads((out/'circuit-prepared.json').read_text())['playback_frames'])
 bid=u.MovieSceneObjectBindingID();bid.set_editor_property('guid',binding.get_id())
 section.set_camera_binding_id(bid)
 controller.set_binding(bid,[camera],False)
 u.EditorAssetLibrary.save_loaded_asset(seq)
 u.EditorLoadingAndSavingUtils.save_current_level()
 u.get_editor_subsystem(u.LevelEditorSubsystem).eject_pilot_level_actor()
 u.EditorLevelLibrary.set_level_viewport_camera_info(camera.get_actor_location(),camera.get_actor_rotation())
 (out/'follow-camera.json').write_text(json.dumps({'camera':camera.get_name(),'offset_cm':[160,-600,320],'status':'saved; automatic camera-cut runtime verification pending'}))
except Exception:
 (out/'follow-camera-error.txt').write_text(traceback.format_exc());raise
