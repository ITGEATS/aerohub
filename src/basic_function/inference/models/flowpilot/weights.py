"""Verify externally supplied weight data against the model's pinned manifest."""
import hashlib
from pathlib import Path


def verify_weights(directory, manifest):
    directory = Path(directory).resolve()
    checked = 0
    for line in Path(manifest).read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        expected, name = line.split()
        if Path(name).name != name or len(expected) != 64:
            raise ValueError('invalid weight manifest record')
        path = directory / name
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        if digest.hexdigest() != expected:
            raise ValueError('weight checksum mismatch: ' + name)
        checked += 1
    if not checked:
        raise ValueError('empty weight manifest')
    return directory
