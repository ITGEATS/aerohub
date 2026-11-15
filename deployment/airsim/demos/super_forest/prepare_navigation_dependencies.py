"""Prepare locked CPython 3.10 wheels for the existing Humble image."""
import hashlib
import json
from pathlib import Path
import sys
from urllib.request import urlopen


def prepare(workspace):
    wheels = workspace / 'wheels'
    wheels.mkdir(parents=True, exist_ok=True)
    dependencies = json.loads(Path(__file__).with_name('navigation_dependencies.json').read_text())
    for dependency in dependencies:
        target = wheels / dependency['filename']
        if not target.exists():
            with urlopen(dependency['url'], timeout=60) as response:
                data = response.read()
            if hashlib.sha256(data).hexdigest() != dependency['sha256']:
                raise RuntimeError('Wheel checksum mismatch: ' + target.name)
            target.write_bytes(data)
        if hashlib.sha256(target.read_bytes()).hexdigest() != dependency['sha256']:
            raise RuntimeError('Wheel checksum mismatch: ' + target.name)


if __name__ == '__main__':
    prepare(Path(sys.argv[1]))
