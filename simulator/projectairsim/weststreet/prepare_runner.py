"""UE 5.2 editor Python: create an independent street map and looping runner."""
import json
import os
from pathlib import Path
import unreal as u

out = Path(os.environ['WEST_RUNNER_OUTPUT'])
out.mkdir(parents=True, exist_ok=True)
sequence_path = '/Game/Aerohub/WestRunnerMotion'
if u.EditorAssetLibrary.does_asset_exist(sequence_path):
    raise RuntimeError('Runner sequence already exists; refusing to overwrite')
actors = u.get_editor_subsystem(u.EditorActorSubsystem)
u.EditorLoadingAndSavingUtils.load_map('/Game/Downtown_West/Maps/Demo_Environment')
# Explicit dependencies persist in the new level and sequence for cooking.
mesh = u.load_asset('/Game/Scanned3DPeoplePack/RP_Character/rp_nathan_rigged_003_ue4/rp_nathan_rigged_003_ue4')
anim = u.load_asset('/Game/AnimStarterPack/Sprint_Fwd_Rifle')
skel = mesh.get_editor_property('skeleton')
compatible = list(skel.get_editor_property('compatible_skeletons'))
source_skel = anim.get_editor_property('skeleton')
if source_skel not in compatible:
    compatible.append(source_skel)
    skel.set_editor_property('compatible_skeletons', compatible)
    u.EditorAssetLibrary.save_loaded_asset(skel)

runner = actors.spawn_actor_from_class(u.SkeletalMeshActor, u.Vector(375, -4000, -15))
runner.set_actor_label('West_Runner_5mps')
comp = runner.skeletal_mesh_component
comp.set_mobility(u.ComponentMobility.MOVABLE)
comp.set_skeletal_mesh_asset(mesh)
comp.set_collision_profile_name('PhysicsActor')
comp.set_animation_mode(u.AnimationMode.ANIMATION_SINGLE_NODE)
# Sequencer drives actual displacement; root motion must not move the actor twice.
comp.set_editor_property('animation_data', u.SingleAnimationPlayData(anim_to_play=anim, saved_looping=True, saved_playing=True))

asset_tools = u.AssetToolsHelpers.get_asset_tools()
seq = asset_tools.create_asset('WestRunnerMotion','/Game/Aerohub',u.LevelSequence,u.LevelSequenceFactoryNew())
seq.set_display_rate(u.FrameRate(30,1))
seq.set_playback_start(0)
seq.set_playback_end(960)
binding = seq.add_possessable(runner)
track = binding.add_track(u.MovieScene3DTransformTrack)
section = track.add_section()
section.set_range(0,960)
channels = section.get_channels()
# 80 m in 16 s, reverse for another 16 s; instantaneous reversal at each end.
keys = [(0,-4000,0),(479,3983.333333,0),(480,4000,180),(959,-3983.333333,180),(960,-4000,180)]
for index,value in [(0,375),(2,-15),(3,0),(4,0),(6,1),(7,1),(8,1)]:
    channels[index].set_default(value)
for frame,y,yaw in keys:
    channels[1].add_key(u.FrameNumber(frame),y,interpolation=u.MovieSceneKeyInterpolation.LINEAR)
    channels[5].add_key(u.FrameNumber(frame),yaw,interpolation=u.MovieSceneKeyInterpolation.CONSTANT)
animation_track = binding.add_track(u.MovieSceneSkeletalAnimationTrack)
animation_section = animation_track.add_section()
animation_section.set_range(0,960)
params = animation_section.get_editor_property('params')
params.animation = anim
params.play_rate = 1.0
animation_section.set_editor_property('params',params)
seq_actor = actors.spawn_actor_from_class(u.LevelSequenceActor,u.Vector())
seq_actor.set_actor_label('West_Runner_Autoplay')
seq_actor.set_sequence(seq)
binding_id=u.MovieSceneObjectBindingID()
binding_id.set_editor_property('guid',binding.get_id())
seq_actor.set_binding(binding_id, [runner], False)
settings=seq_actor.get_editor_property('playback_settings')
settings.auto_play=True
settings.loop_count=u.MovieSceneSequenceLoopCount(-1)
seq_actor.set_editor_property('playback_settings',settings)
cam=actors.spawn_actor_from_class(u.CameraActor,u.Vector(1150,-4750,400))
cam.set_actor_label('West_Runner_ReviewCamera')
cam.set_actor_rotation(u.MathLibrary.find_look_at_rotation(cam.get_actor_location(),u.Vector(375,-3400,80)),False)
cam.camera_component.set_editor_property('field_of_view',65.0)
u.EditorAssetLibrary.save_loaded_asset(seq)
if not u.EditorLoadingAndSavingUtils.save_map(u.EditorLevelLibrary.get_editor_world(),'/Game/Aerohub/WestStreetRunner'):
    raise RuntimeError('Could not save runner map')
result={'map':'/Game/Aerohub/WestStreetRunner','sequence':sequence_path,'actor':runner.get_name(),'mesh':mesh.get_path_name(),'animation':anim.get_path_name(),'speed_mps':5,'route_cm':[[375,-4000,-15],[375,4000,-15]],'cycle_seconds':32,'turns':'instantaneous reversal','animation_play_rate':1,'status':'prepared; visual and runtime validation pending'}
(out/'prepared.json').write_text(json.dumps(result,indent=2))
