import json
import socket
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hud_port import choose_hud_port
import run_super


class HudPortTest(unittest.TestCase):
    def test_live_listener_is_not_interrupted(self):
        with socket.socket() as existing:
            existing.bind(('0.0.0.0', 0))
            existing.listen()
            preferred = existing.getsockname()[1]
            selected = choose_hud_port(preferred)
            self.assertNotEqual(selected, preferred)
            with socket.socket() as probe:
                probe.bind(('0.0.0.0', selected))
            self.assertEqual(existing.getsockname()[1], preferred)

    def test_time_wait_falls_back_without_retry(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            preferred = listener.getsockname()[1]
            with socket.create_connection(('127.0.0.1', preferred)) as client:
                server, _ = listener.accept()
                server.close()  # server initiates close and owns TIME_WAIT
                self.assertEqual(client.recv(1), b'')
        self.assertNotEqual(choose_hud_port(preferred), preferred)

    def test_launch_profile_uses_selected_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root/'profile.json'
            profile.write_text(json.dumps({'display': {'port': 9011}}))
            binary = root/'Blocks'
            binary.touch()
            with patch.object(run_super, 'PROFILE', profile), patch.object(run_super, 'RUNTIME', root/'runtime'), patch.dict(run_super.os.environ, {'COURSE_BINARY': str(binary)}), patch.object(run_super, 'choose_hud_port', return_value=23456), patch.object(run_super, 'run'):
                run_super.manager('start')
                actual = json.loads((root/'runtime/hud-profile.json').read_text())
                self.assertEqual(actual['display']['port'], 23456)
                self.assertEqual(run_super.HUD_PORT, 23456)


if __name__ == '__main__':
    unittest.main()
