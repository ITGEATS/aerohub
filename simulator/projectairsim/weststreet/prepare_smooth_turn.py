"""Explicit editor-only generation; never opens/stops/starts a map or PIE.

Run only after coordination and external-project write approval. Requires
WEST_SMOOTH_BASELINE pointing to the reviewed circuit-prepared.json, and
WEST_SMOOTH_OUTPUT pointing to an ignored log directory. Creates independent
map/sequence assets; refuses overwrite. No automatic runtime validation.
"""
import json
import math
import os
import sys
from pathlib import Path
import unreal as u

sys.path.insert(0, str(Path(__file__).resolve().parent))
from smooth_turn import candidate_from_baseline, profile, sample, shifted_first_junction, delayed_first_junction

DELAYED=os.environ.get('WEST_ROUTE_VARIANT')=='shift-right-2m-late-4m'
SHIFTED=os.environ.get('WEST_ROUTE_VARIANT')=='shift-right-2m'
MAP='/Game/Aerohub/WestStreetShiftRight2m' if SHIFTED else '/Game/Aerohub/WestStreetSmoothTurn'
if DELAYED:MAP='/Game/Aerohub/WestStreetShiftRight2mLate4m'
SEQUENCE='/Game/Aerohub/WestShiftRight2mMotion' if SHIFTED else '/Game/Aerohub/WestSmoothTurnMotion'
if DELAYED:SEQUENCE='/Game/Aerohub/WestShiftRight2mLate4mMotion'


def prepare():
    output=Path(os.environ['WEST_SMOOTH_OUTPUT'])
    output.mkdir(parents=True,exist_ok=True)
    def stage(name, **details):
        (output/'generation-stage.json').write_text(json.dumps(dict(stage=name, **details)))
    stage('preflight', pid=os.getpid())
    editor=u.get_editor_subsystem(u.LevelEditorSubsystem)
    if editor.is_in_play_in_editor():
        raise RuntimeError('PIE active: coordinate stop separately; script will not stop it')
    world=u.EditorLevelLibrary.get_editor_world()
    if world.get_name()!='WestStreetCircuit':
        raise RuntimeError('Open reviewed WestStreetCircuit manually before running')
    if any(u.EditorAssetLibrary.does_asset_exist(p) for p in (MAP,SEQUENCE)):
        raise RuntimeError('Independent output assets exist; refusing overwrite')
    baseline=json.loads(Path(os.environ['WEST_SMOOTH_BASELINE']).read_text())
    original=[(p[0]/100,p[1]/100) for p in baseline['points_cm']]
    points=candidate_from_baseline(original)
    if SHIFTED or DELAYED:points=shifted_first_junction(points,2.)
    if DELAYED:points=delayed_first_junction(points,4.)
    plan=profile(points)
    actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
    runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
    controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
    trace=u.TraceTypeQuery.TRACE_TYPE_QUERY1
    def ground(p):
        hit=u.SystemLibrary.line_trace_single(world,u.Vector(p[0]*100,p[1]*100,250),
            u.Vector(p[0]*100,p[1]*100,-150),trace,True,[runner],u.DrawDebugTrace.NONE,True)
        info=hit.to_tuple() if hit else None
        if not info or not info[0] or not -60<info[5].z<55:
            raise RuntimeError('No reviewed-height ground at '+str(p))
        return info[5].z+1
    # Validate before touching bindings/assets; never silently fall back to sharp bends.
    stage('ground_probe', points=len(points))
    grounded=[(p[0]*100,p[1]*100,ground(p)) for p in points]
    stage('capsule_probe', segments=len(grounded)-1)
    for a,b in zip(grounded,grounded[1:]):
        if abs(a[2]-b[2])>35:raise RuntimeError('Ground discontinuity')
        hit=u.SystemLibrary.capsule_trace_single(world,u.Vector(a[0],a[1],a[2]+99),
            u.Vector(b[0],b[1],b[2]+99),35,80,trace,True,[runner],u.DrawDebugTrace.NONE,True)
        if hit and hit.to_tuple()[0]:raise RuntimeError('Rounded corridor blocked at '+str(a))
    # Save-as changes editor world package, not source map. Do not save source.
    stage('save_independent_map')
    if not u.EditorLoadingAndSavingUtils.save_map(world,MAP):
        raise RuntimeError('Independent map save failed')
    # Save-as duplicates the world: bind only actors reloaded from the new map.
    u.EditorLoadingAndSavingUtils.load_map(MAP)
    world=u.EditorLevelLibrary.get_editor_world()
    actors=u.get_editor_subsystem(u.EditorActorSubsystem).get_all_level_actors()
    runner=next(a for a in actors if a.get_actor_label()=='West_Runner_5mps')
    controller=next(a for a in actors if a.get_actor_label()=='West_Runner_Autoplay')
    seq=u.AssetToolsHelpers.get_asset_tools().create_asset(SEQUENCE.rsplit('/',1)[1],
        '/Game/Aerohub',u.LevelSequence,u.LevelSequenceFactoryNew())
    fps=30
    seq.set_display_rate(u.FrameRate(fps,1))
    tick_rate=seq.get_tick_resolution()
    ticks_per_second=tick_rate.numerator/tick_rate.denominator
    end=math.ceil(plan['duration_s']*fps)
    # Tiny uniform time stretch closes loop exactly on frame boundary.
    stretch=(end/fps)/plan['duration_s']
    seq.set_playback_start(0);seq.set_playback_end(end)
    binding=seq.add_possessable(runner)
    section=binding.add_track(u.MovieScene3DTransformTrack).add_section();section.set_range(0,end)
    channels=section.get_channels()
    for i,v in [(3,0),(4,0),(6,1),(7,1),(8,1)]:channels[i].set_default(v)
    anim=u.load_asset('/Game/AnimStarterPack/Sprint_Fwd_Rifle')
    anim_duration=anim.get_editor_property('sequence_length')
    animation=binding.add_track(u.MovieSceneSkeletalAnimationTrack)
    previous_yaw=0.
    stage('sequence_keys', frames=end)
    samples=[sample(plan,(f/fps)/stretch) for f in range(end+1)]
    for f,(p,distance) in enumerate(samples):
        nxt=samples[f+1][0] if f<end else samples[1][0]
        yaw=math.degrees(math.atan2(nxt[1]-p[1],nxt[0]-p[0]))-90
        while yaw-previous_yaw>180:yaw-=360
        while yaw-previous_yaw<-180:yaw+=360
        previous_yaw=yaw
        for index,value in [(0,p[0]*100),(1,p[1]*100),(2,ground(p)),(5,yaw)]:
            channels[index].add_key(u.FrameNumber(f),value,interpolation=u.MovieSceneKeyInterpolation.LINEAR)
        if f==end:break
        # One frame section: animation phase and rate follow physical distance,
        # not wall time. UE API/section-boundary continuity requires actual validation.
        sec=animation.add_section();sec.set_range(f,f+1)
        params=sec.get_editor_property('params');params.animation=anim
        params.play_rate=(samples[f+1][1]-distance)*fps/5.
        params.first_loop_start_frame_offset=u.FrameNumber(round(((distance/5.)%anim_duration)*ticks_per_second))
        sec.set_editor_property('params',params)
    # New map controller only: SetSequence does not clear inherited overrides.
    controller.reset_bindings()
    controller.set_sequence(seq)
    bid=u.MovieSceneObjectBindingID();bid.set_editor_property('guid',binding.get_id())
    controller.set_binding(bid,[runner],False)
    settings=controller.get_editor_property('playback_settings');settings.auto_play=False
    controller.set_editor_property('playback_settings',settings)
    u.EditorAssetLibrary.save_loaded_asset(seq)
    # Initial save_map above is Save As; final save must target the existing
    # new-world package explicitly (save_map again rejects the existing name).
    if not u.EditorLoadingAndSavingUtils.save_packages([world.get_outermost()], False):
        raise RuntimeError('Final independent map package save failed')
    plan.update(map=MAP,sequence=SEQUENCE,junction_delay_ue_y_m=4. if DELAYED else 0.,lateral_shift_ue_x_m=-2. if SHIFTED or DELAYED else 0.,radius_m=4.,straight_mps=5.,turn_mps=2.75,
                acceleration_mps2=1.25,animation_reference_mps=5.,frame_stretch=stretch,
                status='generated; UE motion/animation and flight validation NOT executed')
    (output/'smooth-turn-prepared.json').write_text(json.dumps(plan,indent=2))
    stage('generated', frames=end, map=MAP, sequence=SEQUENCE)
    u.log('WEST_SMOOTH_TURN_PREPARED')


if __name__=='__main__':
    prepare()
