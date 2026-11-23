"""Prepare the model-owned Vision Language environment for ROS containers."""
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MODEL = Path('src/basic_function/inference/models/vision_language')
ENVIRONMENT = MODEL / '.venv'
CONTAINER_ENVIRONMENT = Path('/repo') / ENVIRONMENT


def prepare(image):
    uv = shutil.which('uv')
    if uv is None:
        raise RuntimeError('Install uv to prepare the Vision Language model environment')
    environment = ROOT / ENVIRONMENT
    environment.mkdir(exist_ok=True)
    subprocess.run([
        'docker', 'run', '--rm', '--user', f'{os.getuid()}:{os.getgid()}',
        '-e', 'HOME=/tmp', '-e', 'UV_CACHE_DIR=/tmp/uv-cache', '-e', 'UV_LINK_MODE=copy',
        '-e', f'UV_PROJECT_ENVIRONMENT={CONTAINER_ENVIRONMENT}',
        '-v', f'{ROOT}:/repo:ro',
        '-v', f'{environment}:{CONTAINER_ENVIRONMENT}',
        '-v', f'{uv}:/usr/local/bin/uv:ro',
        '--entrypoint', '/bin/bash', image, '-ec',
        r'''
build=$(mktemp -d)
mkdir -p "$build/models/vision_language" "$build/runtime"
cp -a /repo/src/basic_function/inference/runtime/. "$build/runtime/"
cp /repo/src/basic_function/inference/models/vision_language/{pyproject.toml,uv.lock} "$build/models/vision_language/"
uv sync --locked --no-editable --reinstall-package aerohub-inference-runtime \
    --python /usr/bin/python3 --project "$build/models/vision_language"
"$UV_PROJECT_ENVIRONMENT/bin/python" -c 'import uvicorn, aerohub_inference'
''',
    ], check=True)
