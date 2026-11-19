"""Create a collision-probed street/plaza circuit in the existing UE editor."""
import heapq,json,math,os,time,traceback
from pathlib import Path
import unreal as u

out=Path(os.environ['WEST_RUNNER_OUTPUT'])
out.mkdir(parents=True,exist_ok=True)

def prepare():
    editor=u.get_editor_subsystem(u.LevelEditorSubsystem)
    if editor.is_in_play_in_editor():
        raise RuntimeError('Stop Play/Simulate before editing the route')
    world=u.EditorLevelLibrary.get_editor_world()
    if world.get_name() not in ['WestStreetRunner','WestStreetCircuit']:
        raise RuntimeError('Open the reviewed runner map first; refusing to modify another map')
    actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
    runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
    controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
    ignores=[runner]
    trace=u.TraceTypeQuery.TRACE_TYPE_QUERY1
    checked=0
    ground_cache={}
    def ground(x,y):
        key=(round(x,2),round(y,2))
        if key not in ground_cache:
            hit=u.SystemLibrary.line_trace_single(world,u.Vector(x,y,250),u.Vector(x,y,-150),trace,True,ignores,u.DrawDebugTrace.NONE,True)
            info=hit.to_tuple() if hit else None
            ok=bool(info and info[0])
            if ok:
                info=hit.to_tuple()
                p=info[5]
                ground_cache[key]=p.z if -60<p.z<55 else None
            else:ground_cache[key]=None
        return ground_cache[key]
    def clear(a,b):
        nonlocal checked
        checked+=1
        za,zb=ground(*a),ground(*b)
        if za is None or zb is None:return False
        if abs(zb-za)>35:return False
        hit=u.SystemLibrary.capsule_trace_single(world,u.Vector(a[0],a[1],za+100),u.Vector(b[0],b[1],zb+100),35,80,trace,True,ignores,u.DrawDebugTrace.NONE,True)
        return hit is None or not hit.to_tuple()[0]
    # Main street -> eastbound at northern junction -> eastern street -> plaza -> return.
    checkpoints=[(400,-4000),(400,6400),(12200,6400),(12200,-1000),
                 (10000,-1000),(9000,-500),(8000,-1000),(6900,-1100),
                 (6200,-500),(5400,-1200),(4600,-700),(3800,-1300),
                 (3000,-700),(2100,-1200),(400,-1200),(400,-4000)]
    grid=100
    planned=[checkpoints[0]]
    astar_counts=[]
    for start,goal in zip(checkpoints,checkpoints[1:]):
        if clear(start,goal):planned.append(goal);astar_counts.append(0);continue
        start=tuple(round(v/grid) for v in start);goal=tuple(round(v/grid) for v in goal)
        q=[(0,start)];cost={start:0};prev={};closed=set()
        while q:
            _,node=heapq.heappop(q)
            if node in closed:continue
            closed.add(node)
            if node==goal:break
            if len(closed)>18000:raise RuntimeError('Route search exceeded limit')
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nxt=(node[0]+dx,node[1]+dy)
                if not(-5<=nxt[0]<=130 and -50<=nxt[1]<=75):continue
                nc=cost[node]+math.hypot(dx,dy)
                if nc>=cost.get(nxt,float('inf')):continue
                a=tuple(v*grid for v in node);b=tuple(v*grid for v in nxt)
                if not clear(a,b):continue
                cost[nxt]=nc;prev[nxt]=node
                heapq.heappush(q,(nc+math.dist(nxt,goal),nxt))
        if goal not in closed:raise RuntimeError('No collision-clear route to '+str(goal))
        path=[goal]
        while path[-1]!=start:path.append(prev[path[-1]])
        path=[tuple(v*grid for v in n) for n in reversed(path)]
        # Visibility simplification retains actual capsule-clear shortcuts.
        i=0
        while i<len(path)-1:
            k=len(path)-1
            while k>i+1 and not clear(path[i],path[k]):k-=1
            planned.append(path[k]);i=k
        astar_counts.append(len(closed))
    # Chamfer corners, retaining only independently collision-clear bends.
    smooth=[planned[0]]
    for i in range(1,len(planned)-1):
        a,b,c=planned[i-1:i+2]
        d1=math.dist(a,b);d2=math.dist(b,c)
        r=min(250,d1*.25,d2*.25)
        entry=tuple(b[k]+(a[k]-b[k])*r/d1 for k in range(2))
        exit=tuple(b[k]+(c[k]-b[k])*r/d2 for k in range(2))
        curve=[]
        for n in range(9):
            t=n/8
            curve.append(tuple((1-t)**2*entry[k]+2*(1-t)*t*b[k]+t*t*exit[k] for k in range(2)))
        if all(clear(x,y) for x,y in zip(curve,curve[1:])):smooth.extend(curve)
        else:smooth.append(b)
    smooth.append(planned[-1])
    # Dense samples validate the entire physical corridor, not just waypoints.
    route=[]
    for a,b in zip(smooth,smooth[1:]):
        for n in range(max(1,math.ceil(math.dist(a,b)/50))):
            t=n/max(1,math.ceil(math.dist(a,b)/50))
            route.append(tuple(a[k]*(1-t)+b[k]*t for k in range(2)))
    route.append(smooth[-1])
    for a,b in zip(route,route[1:]):
        if not clear(a,b):raise RuntimeError('Final route capsule collision at '+str(a))
    points=[(x,y,ground(x,y)+1) for x,y in route]
    asset_path='/Game/Aerohub/WestCircuitMotion'
    if u.EditorAssetLibrary.does_asset_exist(asset_path):raise RuntimeError('Circuit asset already exists; review before replacing')
    seq=u.AssetToolsHelpers.get_asset_tools().create_asset('WestCircuitMotion','/Game/Aerohub',u.LevelSequence,u.LevelSequenceFactoryNew())
    seq.set_display_rate(u.FrameRate(30,1))
    times=[0.0]
    for a,b in zip(points,points[1:]):times.append(times[-1]+math.dist(a,b)/500)
    end=math.ceil(times[-1]*30)
    seq.set_playback_start(0);seq.set_playback_end(end)
    binding=seq.add_possessable(runner)
    section=binding.add_track(u.MovieScene3DTransformTrack).add_section();section.set_range(0,end)
    channels=section.get_channels()
    for i,v in [(3,0),(4,0),(6,1),(7,1),(8,1)]:channels[i].set_default(v)
    previous=None
    for i,(t,p) in enumerate(zip(times,points)):
        nxt=points[(i+1)%len(points)] if i<len(points)-1 else points[1]
        # Sprint asset faces +Y at actor yaw 0.
        yaw=math.degrees(math.atan2(nxt[1]-p[1],nxt[0]-p[0]))-90
        if previous is not None:
            while yaw-previous>180:yaw-=360
            while yaw-previous<-180:yaw+=360
        previous=yaw
        frame=u.FrameNumber(int(t*30));sub=t*30-int(t*30)
        for index,value in [(0,p[0]),(1,p[1]),(2,p[2]),(5,yaw)]:
            channels[index].add_key(frame,value,sub_frame=sub,interpolation=u.MovieSceneKeyInterpolation.LINEAR)
    animation=binding.add_track(u.MovieSceneSkeletalAnimationTrack).add_section();animation.set_range(0,end)
    params=animation.get_editor_property('params');params.animation=u.load_asset('/Game/AnimStarterPack/Sprint_Fwd_Rifle');params.play_rate=1
    animation.set_editor_property('params',params)
    controller.set_sequence(seq)
    bid=u.MovieSceneObjectBindingID();bid.set_editor_property('guid',binding.get_id());controller.set_binding(bid,[runner],False)
    runner.set_actor_location(u.Vector(*points[0]),False,False)
    u.EditorAssetLibrary.save_loaded_asset(seq)
    if not u.EditorLoadingAndSavingUtils.save_map(world,'/Game/Aerohub/WestStreetCircuit'):raise RuntimeError('Could not save circuit map')
    result={'map':'/Game/Aerohub/WestStreetCircuit','sequence':asset_path,'speed_mps':5,'length_m':times[-1]*5,'duration_seconds':times[-1],'playback_frames':end,'checkpoints_cm':checkpoints,'points_cm':points,'times':times,'capsule_radius_cm':35,'capsule_half_height_cm':80,'collision_queries':checked,'astar_expansions':astar_counts,'status':'prepared; runtime validation pending'}
    (out/'circuit-prepared.json').write_text(json.dumps(result,indent=2))
    u.log('WEST_CIRCUIT_PREPARED')

try:prepare()
except Exception:
    (out/'circuit-error.txt').write_text(traceback.format_exc())
    raise

# Reuse this editor for subsequent review scripts without restarting it.
last_request=None
busy=False
def service(delta):
    global last_request,busy
    if busy:return
    request=out/'editor-request.json'
    if not request.exists():return
    job=json.loads(request.read_text())
    if job['id']==last_request:return
    last_request=job['id'];busy=True
    try:
        import runpy
        runpy.run_path(job['script'],run_name='__main__')
    except Exception:(out/'editor-request-error.txt').write_text(traceback.format_exc())
    finally:busy=False
service_handle=u.register_slate_post_tick_callback(service)
