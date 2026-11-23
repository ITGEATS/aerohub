"""WestStreet Deploy port defaults and retained-session lookup."""
import json
from pathlib import Path

PORTS = json.loads(Path(__file__).with_name('deployment.json').read_text())['ports']


def session_ports(directory):
    profile = json.loads((directory / 'profile.json').read_text())
    scene = json.loads((directory / 'scene-control.json').read_text())
    return {
        **PORTS,
        **profile['ports'],
        'hud': profile['display']['port'],
        'scene': scene['port'],
    }
