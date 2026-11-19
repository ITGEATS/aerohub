"""Validate one complete live circuit in the current editor and capture key views."""
import bisect,json,math,os,time,traceback
from pathlib import Path
import unreal as u
out=Path(os.environ['WEST_RUNNER_OUTPUT'])
plan=json.loads((out/'circuit-prepared.json').read_text())
if (out/'circuit-validation.json').exists():
 (out/'circuit-validation.json').rename(out/('circuit-validation.previous-'+str(int(time.time()))+'.json'))
editor=u.get_editor_subsystem(u.LevelEditorSubsystem)
if editor.is_in_play_in_editor():raise RuntimeError('Stop existing simulation first')
if u.EditorLevelLibrary.get_editor_world().get_name()!='WestStreetCircuit':
 u.EditorLoadingAndSavingUtils.load_map('/Game/Aerohub/WestStreetCircuit')
editor.eject_pilot_level_actor()
start=time.monotonic();samples=[];last=-1;busy=False;task=None;images=[]
views=[]  # Capture separate scene views; do not block runtime validation on a background viewport.
editor.editor_play_simulate()

def expected(t):
    t=t%(plan['playback_frames']/30)
    times=plan['times'];pts=plan['points_cm']
    i=max(0,min(bisect.bisect_right(times,t)-1,len(times)-2))
    a,b=pts[i:i+2];alpha=min(1,max(0,(t-times[i])/(times[i+1]-times[i])))
    return [x+(y-x)*alpha for x,y in zip(a,b)]

def tick(delta):
 global last,busy,task
 if busy:return
 busy=True
 try:
    if time.monotonic()-start>900:raise RuntimeError('Runtime circuit review timeout')
    world=u.EditorLevelLibrary.get_game_world()
    if not world:return
    people=[a for a in u.GameplayStatics.get_all_actors_of_class(world,u.SkeletalMeshActor) if a.get_actor_label()=='West_Runner_5mps']
    if len(people)!=1:raise RuntimeError('Expected exactly one runner')
    runner=people[0];t=u.GameplayStatics.get_time_seconds(world)
    p=runner.get_actor_location();xyz=[p.x,p.y,p.z]
    controller=next(a for a in u.GameplayStatics.get_all_actors_of_class(world,u.LevelSequenceActor) if a.get_actor_label()=='West_Runner_Autoplay')
    qt=controller.get_editor_property('sequence_player').get_current_time()
    sequence_time=(qt.time.frame_number.value+qt.time.sub_frame)*qt.rate.denominator/qt.rate.numerator
    if t-last>=.2:
        foot=runner.skeletal_mesh_component.get_socket_location('foot_l')
        samples.append({'time':t,'sequence_time':sequence_time,'position_cm':xyz,'yaw':runner.get_actor_rotation().yaw,'foot_relative_cm':[foot.x-p.x,foot.y-p.y,foot.z-p.z],'expected_error_cm':math.dist(xyz,expected(sequence_time))})
        last=t
    if task and task.is_task_done():task=None
    if views and t>=views[0][0] and task is None:
        _,label=views.pop(0)
        yaw=math.radians(runner.get_actor_rotation().yaw+90)
        # Offset side/behind the moving actor so its route remains visible.
        loc=u.Vector(p.x-1000*math.cos(yaw)+700*math.sin(yaw),p.y-1000*math.sin(yaw)-700*math.cos(yaw),p.z+600)
        rot=u.MathLibrary.find_look_at_rotation(loc,u.Vector(p.x,p.y,p.z+85))
        u.EditorLevelLibrary.set_level_viewport_camera_info(loc,rot)
        path=str(out/('circuit-'+label+'.png'));images.append(path)
        task=u.AutomationLibrary.take_high_res_screenshot(1280,720,path,delay=.1)
    if t>plan['playback_frames']/30+5 and not task:
        speeds=[math.dist(a['position_cm'],b['position_cm'])/(b['time']-a['time'])/100 for a,b in zip(samples,samples[1:])]
        errors=[s['expected_error_cm'] for s in samples]
        straight_speeds=[]
        for a,b in zip(samples,samples[1:]):
            dt=b['sequence_time']-a['sequence_time']
            yaw_delta=abs((b['yaw']-a['yaw']+180)%360-180)
            if dt>0 and yaw_delta<1:
                straight_speeds.append(math.dist(a['position_cm'],b['position_cm'])/dt/100)
        # Absolute time alignment may differ by <= one game frame; position error is recorded.
        result={'mode':'UE editor Simulate, not packaged runtime','samples':samples,'duration_seconds':t,'speed_min_mps':min(speeds),'speed_max_mps':max(speeds),'speed_mean_mps':sum(speeds)/len(speeds),'max_expected_error_cm':max(errors),'images':images,'full_cycle_observed':True,'straight_speed_min_mps':min(straight_speeds),'straight_speed_max_mps':max(straight_speeds),'measurement':'position and evaluated sequence time; world-time chord speeds retained as diagnostics; turning chords are shorter than path arc','passed':max(errors)<50 and bool(straight_speeds) and all(abs(v-5)<.1 for v in straight_speeds)}
        (out/'circuit-validation.json').write_text(json.dumps(result,indent=2))
        editor.editor_request_end_play()
        u.unregister_slate_post_tick_callback(handle)
        u.log('WEST_CIRCUIT_REVIEW_COMPLETE')
 except Exception:
    (out/'circuit-review-error.txt').write_text(traceback.format_exc())
    u.unregister_slate_post_tick_callback(handle)
 finally:busy=False
handle=u.register_slate_post_tick_callback(tick)
