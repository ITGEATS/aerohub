"""Observe real HUD camera ownership and one native drone; issue no vehicle commands."""
import json,os,time,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_INTEGRATION_OUTPUT'])
samples=[];last=0;start=time.monotonic()
def tick(delta):
 global last
 try:
  if time.monotonic()-start>90:raise RuntimeError('Integration editor validation timeout')
  if time.monotonic()-last<.5:return
  last=time.monotonic()
  w=u.EditorLevelLibrary.get_game_world()
  if not w:return
  pc=u.GameplayStatics.get_player_controller(w,0)
  actors=u.GameplayStatics.get_all_actors_of_class(w,u.Actor)
  follow=[a for a in actors if a.actor_has_tag('WestRunnerGlobalFollow')]
  drones=[a for a in actors if a.get_name()=='Drone1']
  runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
  view=pc.get_view_target()
  expected=follow[0]
  error=(view.get_actor_location()-expected.get_actor_location()).length()
  rp=runner.get_actor_location()
  samples.append({'runner_position_cm':[rp.x,rp.y,rp.z],'view_target':view.get_name(),'follow_camera':expected.get_name(),'position_error_cm':error,'tag_matches':len(follow),'native_drones':len(drones),'hud_class':pc.get_hud().get_class().get_name()})
  if len(samples)>=12:
   travel=(u.Vector(*samples[-1]['runner_position_cm'])-u.Vector(*samples[0]['runner_position_cm'])).length()
   result={'samples':samples,'runner_displacement_cm':travel,'passed':travel>100 and all(s['position_error_cm']<50 and s['native_drones']==1 and s['tag_matches']==1 and s['hud_class']=='AerohubHUD' for s in samples)}
   (out/'editor-validation.json').write_text(json.dumps(result,indent=2))
   u.unregister_slate_post_tick_callback(handle)
 except Exception:
  (out/'editor-validation-error.txt').write_text(traceback.format_exc());u.unregister_slate_post_tick_callback(handle)
handle=u.register_slate_post_tick_callback(tick)
