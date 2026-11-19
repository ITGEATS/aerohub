"""Verify local snapshot and link artifacts; never download or overwrite weights."""
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def verify(source):
    manifest = json.loads((ROOT / 'artifacts.json').read_text())
    for name, expected in manifest['files'].items():
        path = source / name
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        if digest.hexdigest() != expected:
            raise ValueError('artifact SHA256 mismatch: ' + name)
    return manifest


def prepare():
    destination = ROOT / 'artifacts' / 'snapshot'
    supplied = os.environ.get('GROUNDING_DINO_SOURCE')
    source = Path(supplied).expanduser().resolve() if supplied else destination
    verify(source)
    if destination.exists() or destination.is_symlink():
        verify(destination)
        if supplied and destination.resolve() != source:
            raise ValueError('existing snapshot points elsewhere; remove explicitly before changing source')
    else:
        destination.parent.mkdir(exist_ok=True)
        destination.symlink_to(source, target_is_directory=True)
    print(json.dumps(dict(verified=True, files=list(verify(destination)['files']))))


if __name__ == '__main__':
    prepare()
