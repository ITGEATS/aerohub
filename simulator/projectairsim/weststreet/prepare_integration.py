"""Prepare the accepted circuit for native ProjectAirSim and HUD-owned Global."""
import json,os,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ.get('WEST_INTEGRATION_OUTPUT',os.environ['WEST_RUNNER_OUTPUT']))
out.mkdir(parents=True,exist_ok=True)
try:
 editor=u.get_editor_subsystem(u.LevelEditorSubsystem)
 if editor.is_in_play_in_editor():raise RuntimeError('Stop Play before preparing integration')
 if u.EditorLevelLibrary.get_editor_world().get_name()!='WestStreetCircuit':raise RuntimeError('Wrong map')
 camera=next(a for a in u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors() if a.get_actor_label()=='West_Circuit_FollowCamera')
 tags=list(camera.tags)
 if 'WestRunnerGlobalFollow' not in [str(t) for t in tags]:tags.append('WestRunnerGlobalFollow')
 camera.set_editor_property('tags',tags)
 seq=u.load_asset('/Game/Aerohub/WestCircuitMotion')
 cuts=[t for t in seq.get_master_tracks() if isinstance(t,u.MovieSceneCameraCutTrack)]
 for track in cuts:seq.remove_master_track(track)
 u.EditorAssetLibrary.save_loaded_asset(seq)
 # Already-loaded native plugin is optional during initial preparation.
 if hasattr(u,'ProjectAirSimGameMode'):
  u.EditorLevelLibrary.get_editor_world().get_world_settings().set_editor_property('default_game_mode',u.ProjectAirSimGameMode.static_class())
 u.EditorLoadingAndSavingUtils.save_current_level()
 editor.eject_pilot_level_actor()
 (out/'map-integration.json').write_text(json.dumps({'map':'/Game/Aerohub/WestStreetCircuit','global_camera_tag':'WestRunnerGlobalFollow','removed_camera_cut_tracks':len(cuts)}))
except Exception:
 (out/'map-integration-error.txt').write_text(traceback.format_exc());raise
