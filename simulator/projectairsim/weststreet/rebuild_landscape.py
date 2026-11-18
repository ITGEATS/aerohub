"""Run in a rendered UE editor; rebuild and save the selected map's landscape.

WESTSTREET_REPAIR_OUTPUT selects the evidence directory. The editor must open
the intended map before executing this script. No screen messages are disabled.
"""
import json
import os
import traceback
from pathlib import Path

import unreal

output = Path(os.environ['WESTSTREET_REPAIR_OUTPUT'])
output.mkdir(parents=True, exist_ok=True)
frames = 0


def tick(delta):
    global frames
    frames += 1
    if frames < 90:
        return
    unreal.unregister_slate_post_tick_callback(handle)
    try:
        world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        unreal.SystemLibrary.execute_console_command(world, 'DumpTextureStreamingStats')
        report = {'map': world.get_path_name()}
        landscapes = [a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
                      if isinstance(a, unreal.LandscapeProxy)]
        report['landscapes'] = [a.get_path_name() for a in landscapes]
        report['before'] = unreal.AerohubSceneControlWorld.outdated_landscape_materials(world)
        # A loaded editor may have already refreshed transient render tasks.
        # Force a save so their physical material hashes/collision data persist.
        for actor in landscapes:
            actor.modify()
        report['after_build'] = unreal.AerohubSceneControlWorld.rebuild_landscape(world)
        report['saved'] = unreal.EditorLoadingAndSavingUtils.save_packages([world.get_outermost()], False)
        report['after_save'] = unreal.AerohubSceneControlWorld.outdated_landscape_materials(world)
        report['success'] = report['saved'] and report['after_save'] == 0
        (output / 'landscape-rebuild.json').write_text(json.dumps(report, indent=2))
        if not report['success']:
            raise RuntimeError('Landscape rebuild/save failed: ' + str(report))
    except Exception:
        (output / 'landscape-error.txt').write_text(traceback.format_exc())
    unreal.SystemLibrary.quit_editor()


handle = unreal.register_slate_post_tick_callback(tick)
