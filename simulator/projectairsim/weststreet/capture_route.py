"""Capture static route review views via SceneCapture2D without taking desktop focus.
The editor runner is temporarily positioned for inspection, then restored; not saved.
"""
import json,math,os,time,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_RUNNER_OUTPUT'])
world=u.EditorLevelLibrary.get_editor_world()
actors=u.get_editor_subsystem(u.EditorActorSubsystem)
runner=next(a for a in actors.get_all_level_actors() if a.get_actor_label()=='West_Runner_5mps')
original=runner.get_actor_transform()
plan=json.loads((out/'circuit-prepared.json').read_text())
queue=[(25,'junction'),(65,'plaza-entry'),(73,'obstacles'),(80,'return')]
pending=None;busy=False;capture_actor=None;captures=[]
def cleanup():
 runner.set_actor_transform(original,False,False)
 if capture_actor:actors.destroy_actor(capture_actor)
def tick(delta):
 global pending,busy,capture_actor
 if busy:return
 busy=True
 try:
  if pending:
   target,label,ready=pending
   if time.monotonic()<ready:return
   u.RenderingLibrary.export_render_target(world,target,str(out),'route-review-'+label+'.png')
   captures.append(label);pending=None
   (out/'route-review-captures.json').write_text(json.dumps({'mode':'static editor placements, not runtime screenshots','views':captures}))
   if not queue:
    cleanup();u.unregister_slate_post_tick_callback(handle)
   return
  t,label=queue.pop(0)
  i=min(range(len(plan['times'])),key=lambda k:abs(plan['times'][k]-t))
  point=plan['points_cm'][i];nxt=plan['points_cm'][i+1]
  yaw=math.atan2(nxt[1]-point[1],nxt[0]-point[0]);p=u.Vector(*point)
  runner.set_actor_location(p,False,False)
  runner.set_actor_rotation(u.Rotator(0,math.degrees(yaw)-90,0),False)
  loc=u.Vector(p.x-850*math.cos(yaw)+650*math.sin(yaw),p.y-850*math.sin(yaw)-650*math.cos(yaw),p.z+1000)
  rot=u.MathLibrary.find_look_at_rotation(loc,u.Vector(p.x,p.y,p.z+85))
  if capture_actor is None:capture_actor=actors.spawn_actor_from_class(u.SceneCapture2D,loc,rot)
  else:
   capture_actor.set_actor_location(loc,False,False);capture_actor.set_actor_rotation(rot,False)
  comp=capture_actor.get_editor_property('capture_component2d')
  target=u.RenderingLibrary.create_render_target2d(world,1280,720,u.TextureRenderTargetFormat.RTF_RGBA8)
  comp.set_editor_property('texture_target',target)
  comp.set_editor_property('capture_source',u.SceneCaptureSource.SCS_FINAL_COLOR_LDR)
  comp.set_editor_property('fov_angle',65)
  post=comp.get_editor_property('post_process_settings')
  post.set_editor_property('override_auto_exposure_method',True)
  post.set_editor_property('auto_exposure_method',u.AutoExposureMethod.AEM_MANUAL)
  post.set_editor_property('override_auto_exposure_apply_physical_camera_exposure',True)
  post.set_editor_property('auto_exposure_apply_physical_camera_exposure',False)
  post.set_editor_property('override_auto_exposure_bias',True)
  post.set_editor_property('auto_exposure_bias',0.0)
  comp.set_editor_property('post_process_settings',post)
  comp.capture_scene()
  pending=(target,label,time.monotonic()+1)
 except Exception:
  cleanup()
  (out/'route-review-capture-error.txt').write_text(traceback.format_exc())
  u.unregister_slate_post_tick_callback(handle)
 finally:busy=False
handle=u.register_slate_post_tick_callback(tick)
