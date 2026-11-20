"""Build the Humble migration image from an exact upstream Git tree."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

UPSTREAM_URL = 'https://github.com/TJU-Aerial-Robotics/YOPO-Tracker.git'
UPSTREAM_COMMIT = 'd313f8610195dc986caceff7a3616f63d7e6f31f'
UPSTREAM_PATHS = ['LICENSE', 'YOPO', 'Controller/src/utils/quadrotor_msgs']
DEPLOYMENT_FILES = ['Dockerfile', 'pyproject.toml', 'uv.lock', 'entrypoint.sh', 'brake-calibration.sitl.json']


def run(*args):
    subprocess.run(list(map(str, args)), check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, help='Local Git repository containing the locked commit')
    parser.add_argument('--tag', help='Override the image tag for the selected build mode')
    parser.add_argument('--uv-cache-image', help='Optional local build image with an existing uv cache')
    parser.add_argument('--offline', action='store_true', help='Disable network access in build steps')
    parser.add_argument('--reuse-worker', action='store_true',
                        help='Build a local verification image sharing the specified CUDA worker layers')
    parser.add_argument('--with-native-baseline', action='store_true',
                        help='Include the checkpoint for standalone native diagnostics and replay')
    args = parser.parse_args()
    if args.reuse_worker and not args.uv_cache_image:
        parser.error('--reuse-worker requires --uv-cache-image')
    if args.offline and args.upstream is None:
        parser.error('--offline requires a local --upstream Git repository')
    deployment = Path(__file__).resolve().parent
    repo = deployment.parents[1]
    adapter = repo / 'src/tool/navigation/planner/yopo_tracker/ros2_adapter'

    with tempfile.TemporaryDirectory(prefix='yopo-tracker-build-') as directory:
        temporary = Path(directory)
        source = args.upstream
        if source is None:
            source = temporary / 'upstream-git'
            run('git', 'init', '--bare', source)
            run('git', '-C', source, 'fetch', '--no-tags', '--depth=1', UPSTREAM_URL, UPSTREAM_COMMIT)

        archive = temporary / 'upstream.tar'
        run('git', '-C', source, 'archive', '--format=tar', '-o', archive,
            UPSTREAM_COMMIT, *UPSTREAM_PATHS)
        context = temporary / 'context'
        (context / 'upstream').mkdir(parents=True)
        exclusions = [] if args.with_native_baseline else ['--exclude=YOPO/saved']
        run('tar', *exclusions, '-xf', archive, '-C', context / 'upstream')
        for file in DEPLOYMENT_FILES:
            shutil.copy2(deployment / file, context)
        for layer in ('native', 'bridge'):
            shutil.copytree(adapter / layer, context / 'ros2_adapter' / layer,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        shutil.copytree(repo / 'src/tool/navigation/interfaces', context / 'planner_interfaces')
        inference = repo / 'src/basic_function/inference'
        shutil.copytree(inference / 'interfaces', context / 'inference_interfaces')
        shutil.copytree(inference / 'ros2_adapter', context / 'inference_ros',
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        target = 'reuse' if args.reuse_worker else 'standalone'
        default_tag = 'uav/yopo-tracker:humble-native'
        if args.with_native_baseline:
            default_tag += '-baseline'
        if args.reuse_worker:
            default_tag += '-local'
        build_args = ['--target', target]
        if args.offline:
            build_args.extend(['--pull=false', '--network=none'])
        if args.uv_cache_image:
            build_args.extend(['--build-arg', 'UV_CACHE_IMAGE=' + args.uv_cache_image])
        run('docker', 'build', *build_args, '--tag', args.tag or default_tag, '--label',
            'org.opencontainers.image.revision=' + UPSTREAM_COMMIT, context)


if __name__ == '__main__':
    main()
