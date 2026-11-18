#!/usr/bin/env python3
"""Incrementally build the scene plugin against an existing Linux UE project.

The native ProjectAirSim editor plugin must already be built with this engine.
--install backs up and updates only AerohubSceneControl, never other plugins.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ue-root', type=Path, required=True)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    project, output = args.project.resolve(), args.output.resolve()
    engine = args.ue_root.resolve() / 'Engine'
    native = project.parent / 'Plugins/ProjectAirSim'
    native_binary = native / 'Binaries/Linux/libUnrealEditor-ProjectAirSim.so'
    if not native_binary.is_file():
        raise RuntimeError('Build the matching native ProjectAirSim editor plugin first')
    host = output / 'HostProject'
    plugins = host / 'Plugins'
    plugins.mkdir(parents=True, exist_ok=True)
    link = plugins / 'ProjectAirSim'
    if link.exists() and link.resolve() != native.resolve():
        raise RuntimeError('Output already references a different ProjectAirSim plugin')
    if not link.exists():
        link.symlink_to(native, target_is_directory=True)
    source = repo / 'simulator/projectairsim/unreal/AerohubSceneControl'
    built = plugins / 'AerohubSceneControl'
    shutil.copytree(source, built, dirs_exist_ok=True)
    host_project = host / 'HostProject.uproject'
    host_project.write_text(json.dumps({'FileVersion': 3, 'Plugins': [
        {'Name': name, 'Enabled': True} for name in ('ProjectAirSim', 'AerohubSceneControl')]}))
    command = [str(engine / 'Build/BatchFiles/Linux/Build.sh'), 'UnrealEditor', 'Linux', 'Development',
               '-Project=' + str(host_project), '-Module=AerohubSceneControl', '-NoHotReloadFromIDE',
               '-MaxParallelActions=2', '-log=' + str(output / 'build.log')]
    (output / 'build-command.json').write_text(json.dumps(command, indent=2))
    subprocess.run(command, check=True)
    binary = built / 'Binaries/Linux/libUnrealEditor-AerohubSceneControl.so'
    # Modular editor builds must import the existing singleton, not define one.
    symbols = subprocess.check_output(['nm', '-D', '-C', str(binary)], text=True)
    clock_symbols = [line for line in symbols.splitlines() if 'SimClock::Get(' in line]
    if len(clock_symbols) != 1 or not clock_symbols[0].lstrip().startswith('U '):
        raise RuntimeError('Scene plugin must import the existing ProjectAirSim clock singleton')
    manifest = {'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                'native_binary_sha256': hashlib.sha256(native_binary.read_bytes()).hexdigest(),
                'clock_symbol': clock_symbols[0].strip(), 'installed': False}
    if args.install:
        if subprocess.run(['pgrep', '-x', 'UnrealEditor'], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('Close existing UnrealEditor processes before installing the plugin')
        target = project.parent / 'Plugins/AerohubSceneControl'
        backup = output / 'installed-before'
        if target.exists() and not backup.exists():
            shutil.copytree(target, backup, ignore=shutil.ignore_patterns('Intermediate'))
        shutil.copytree(source, target, dirs_exist_ok=True)
        (target / 'Binaries/Linux').mkdir(parents=True, exist_ok=True)
        shutil.copy2(binary, target / 'Binaries/Linux')
        build_id = json.loads((engine / 'Binaries/Linux/UnrealEditor.modules').read_text())['BuildId']
        (target / 'Binaries/Linux/UnrealEditor.modules').write_text(json.dumps({
            'BuildId': build_id, 'Modules': {'AerohubSceneControl': binary.name}}, indent=2))
        manifest['installed'] = True
    (output / 'plugin-build.json').write_text(json.dumps(manifest, indent=2))
    print('Scene control plugin built' + (' and installed' if args.install else ''))


if __name__ == '__main__':
    main()
