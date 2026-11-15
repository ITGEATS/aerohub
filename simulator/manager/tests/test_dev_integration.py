"""Integration boundaries for old AirSim profiles and unified manager recovery."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from simulator.manager import manager, resources

ROOT = Path(__file__).resolve().parents[3]


class DevIntegrationTest(unittest.TestCase):
    def test_local_resources_override_shared_defaults(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            entries = [
                ('config/resources.yaml', '/shared/default'),
                ('xdg/aerohub/resources.yaml', '/user/install'),
                ('simulator/airsim/config/resources.local.yaml', '/legacy/install'),
                ('config/resources.local.yaml', '/new/install'),
            ]
            for relative, value in entries:
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f'resources:\n  airsim_nh:\n    path: {value}\n')
            with patch.object(resources, '__file__', str(root/'simulator/manager/resources.py')), patch.dict(
                    os.environ, {'XDG_CONFIG_HOME': str(root/'xdg')}):
                self.assertEqual(resources.resolve_resource('airsim_nh'), Path('/new/install'))
                (root/'config/resources.local.yaml').unlink()
                self.assertEqual(resources.resolve_resource('airsim_nh'), Path('/legacy/install'))
                (root/'simulator/airsim/config/resources.local.yaml').unlink()
                self.assertEqual(resources.resolve_resource('airsim_nh'), Path('/user/install'))

    def test_airsim_recovery_ignores_missing_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for action in ('status', 'stop'):
                command = [sys.executable, str(ROOT/'simulator/manager/manager.py'), action,
                           '--profile', str(root/'missing.json'), '--run-dir', str(root/'session')]
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(result.returncode, 1 if action == 'status' else 0, result.stderr)
                self.assertNotIn('No such file', result.stderr)
            self.assertEqual(json.loads((root/'session/state.json').read_text())['phase'], 'stopped')

    def test_projectairsim_stop_uses_saved_state_without_loading_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'state.json').write_text(json.dumps({
                'backend': 'projectairsim', 'owns_host': False, 'phase': 'ready', 'auxiliaries': []}))
            argv = ['manager', 'stop', '--profile', str(root/'missing.yaml'), '--run-dir', str(root)]
            with patch.object(sys, 'argv', argv), patch.object(manager, 'load_profile') as load, patch.object(
                    manager.os, 'killpg') as kill:
                self.assertEqual(manager.main(), 0)
                load.assert_not_called()
                kill.assert_not_called()
            self.assertEqual(json.loads((root/'state.json').read_text())['phase'], 'stopped')

    def test_legacy_scene_profiles_dispatch_to_same_airsim_check(self):
        profiles = [ROOT/'simulator/airsim/config/nh.json',
                    *sorted((ROOT/'simulator/airsim/scenarios').glob('*/profile.json'))]
        with tempfile.TemporaryDirectory() as tmp:
            for profile in profiles:
                with self.subTest(profile=profile):
                    results = []
                    for entry in ('simulator/manager/manager.py', 'simulator/airsim/manager/manager.py'):
                        result = subprocess.run([sys.executable, str(ROOT/entry), 'check', '--profile',
                            str(profile), '--binary', '/bin/true', '--run-dir', tmp],
                            capture_output=True, text=True, env={**os.environ, 'AIRSIM_RENDER_MODE': 'profile'})
                        self.assertEqual(result.returncode, 0, result.stderr)
                        results.append(json.loads(result.stdout))
                    self.assertEqual(results[0], results[1])
