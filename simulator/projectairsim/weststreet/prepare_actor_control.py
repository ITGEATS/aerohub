"""UE editor commandlet: persist runner tags and paused-on-start playback."""
import json,os
from pathlib import Path
import unreal as u
u.EditorLoadingAndSavingUtils.load_map('/Game/Aerohub/WestStreetCircuit')
actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
for actor,tag in [(runner,'WestRunner'),(controller,'WestRunnerSequence')]:
 tags=list(actor.tags)
 if tag not in [str(t) for t in tags]:tags.append(tag)
 actor.set_editor_property('tags',tags)
settings=controller.get_editor_property('playback_settings');settings.auto_play=False
controller.set_editor_property('playback_settings',settings)
runner.set_actor_location(u.Vector(400,-4000,-15.053299),False,False)
runner.set_actor_rotation(u.Rotator(0,0,0),False)
mesh=runner.skeletal_mesh_component
mesh.set_editor_property('pause_anims',True)
data=mesh.get_editor_property('animation_data');data.saved_playing=False;mesh.set_editor_property('animation_data',data)
u.EditorLoadingAndSavingUtils.save_current_level()
Path(os.environ['WEST_FLIGHT_OUTPUT'],'paused-map.json').write_text(json.dumps({'map':'/Game/Aerohub/WestStreetCircuit','auto_play':False,'runner_tag':'WestRunner','sequence_tag':'WestRunnerSequence'}))
