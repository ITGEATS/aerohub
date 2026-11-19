"""Run with editor -ExecCmds='py ...'; retain window and evidence."""
import json,os,time,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_RUNNER_OUTPUT'])
for name in ['review-error.txt','motion-validation.json']:
 if (out/name).exists():
  (out/name).rename(out/(name+'.previous-'+str(int(time.time()))))
u.EditorLoadingAndSavingUtils.load_map('/Game/Aerohub/WestStreetRunner')
actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
camera=next(a for a in actors if a.get_actor_label()=='West_Runner_ReviewCamera')
u.get_editor_subsystem(u.LevelEditorSubsystem).pilot_level_actor(camera)
start=time.monotonic()
stage=0
samples=[]
last=0
capture=None
busy=False

def tick(delta):
 global stage,last,capture,busy
 if busy:return
 busy=True
 try:
  elapsed=time.monotonic()-start
  if elapsed>600: raise RuntimeError('Review timeout')
  if stage==0 and elapsed>45:
   stage=1
   capture=u.AutomationLibrary.take_high_res_screenshot(1280,720,str(out/'runner-street.png'),camera=camera,delay=1)
  elif stage==1 and capture.is_task_done():
   stage=2
   u.get_editor_subsystem(u.LevelEditorSubsystem).editor_play_simulate()
  elif stage==2:
   world=u.EditorLevelLibrary.get_game_world()
   if not world:return
   found=[a for a in u.GameplayStatics.get_all_actors_of_class(world,u.SkeletalMeshActor) if a.get_actor_label()=='West_Runner_5mps']
   if not found:return
   t=u.GameplayStatics.get_time_seconds(world)
   if t-last<0.2:return
   a=found[0];p=a.get_actor_location()
   foot=a.skeletal_mesh_component.get_socket_location('foot_l')
   samples.append({'time':t,'position_cm':[p.x,p.y,p.z],'foot_left_relative_cm':[foot.x-p.x,foot.y-p.y,foot.z-p.z]})
   last=t
   if t>20:
    speeds=[]
    for x,y in zip(samples,samples[1:]):
     dt=y['time']-x['time']
     if dt>0 and not (x['time']<16<y['time']):
      speeds.append(sum((b-a)**2 for a,b in zip(x['position_cm'],y['position_cm']))**0.5/dt/100)
    result={'passed':bool(speeds) and all(abs(v-5)<0.1 for v in speeds),'samples':samples,'speed_min_mps':min(speeds),'speed_max_mps':max(speeds),'mode':'editor simulate; not packaged runtime'}
    (out/'motion-validation.json').write_text(json.dumps(result,indent=2))
    u.get_editor_subsystem(u.LevelEditorSubsystem).editor_request_end_play()
    u.unregister_slate_post_tick_callback(handle)
    u.log('WEST_RUNNER_REVIEW_COMPLETE')
 except Exception:
  (out/'review-error.txt').write_text(traceback.format_exc())
  u.unregister_slate_post_tick_callback(handle)
 finally:
  busy=False
handle=u.register_slate_post_tick_callback(tick)
