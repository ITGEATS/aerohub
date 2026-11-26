"""Check delivery planning, fail-closed config and external weight integrity."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


deploy = load('flowpilot_delivery', 'deployment/airsim/demos/flowpilot_single/run.py')
weights = load('flowpilot_weight_check', 'src/basic_function/inference/models/flowpilot/weights.py')
verdict = load('flowpilot_verdict', 'deployment/airsim/demos/flowpilot_single/result_checks.py')


class DeliveryTest(unittest.TestCase):
    def config(self):
        return dict(prefix='fp-test', domain=187, image='flowpilot:test', flight_image='flight_board:test',
                    airsim_host='127.0.0.1', rpc_port=45471, evidence='evidence',
                    goal=[0.,12.,2.], timeout=90.)

    def test_runtime_mounts_are_data_only(self):
        c = self.config()
        c['evidence'] = '/external/evidence'
        commands = list(deploy.start_commands(c))
        self.assertEqual(len(commands), 5)
        for cmd in commands:
            for i, arg in enumerate(cmd):
                if arg == '-v':
                    parts = cmd[i+1].rsplit(':', 2)
                    target = parts[-2] if parts[-1] in ('ro', 'rw') else parts[-1]
                    self.assertIn(target, ['/evidence', '/config/settings.json'])
        self.assertIn('overhead', commands[3])

    def test_invalid_config_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'config.json'
            for k, v in [('domain', True), ('prefix', 'bad/path'), ('rpc_port', 41471), ('goal', [float('nan'), 0, 0])]:
                c = self.config(); c[k] = v
                path.write_text(json.dumps(c), encoding='utf-8')
                with self.assertRaises(ValueError): deploy.load_config(path)

    def test_flight_dry_run_is_gated_and_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'config.json';path.write_text(json.dumps(self.config()), encoding='utf-8')
            result = subprocess.run([sys.executable, str(ROOT/'deployment/airsim/demos/flowpilot_single/run.py'),
                                     '--config', str(path), '--dry-run', '--case', 'case1', 'flight'],
                                    capture_output=True, text=True, check=True)
            commands = [json.loads(line) for line in result.stdout.splitlines()]
            self.assertEqual(commands[0][commands[0].index('flowpilot:test')+1], 'preflight')
            self.assertEqual(commands[1][commands[1].index('flowpilot:test')+1], 'mission')
            self.assertFalse((Path(folder)/'evidence').exists())

    def test_weight_integrity_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);(root/'model.pt').write_bytes(b'expected')
            manifest = root/'manifest';manifest.write_text(hashlib.sha256(b'expected').hexdigest()+'  model.pt\n')
            weights.verify_weights(root, manifest)
            (root/'model.pt').write_bytes(b'corrupt')
            with self.assertRaises(ValueError):weights.verify_weights(root,manifest)

    def test_weight_manifest_cannot_escape_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);manifest=root/'manifest';manifest.write_text('a'*64+'  ../outside\n')
            with self.assertRaises(ValueError):weights.verify_weights(root,manifest)

    def test_fault_verdict_requires_injection_and_complete_recovery(self):
        result=dict(fault_injected=True,cancel_ack=True,expected_cancel=True,stopped=True,
                    land_success=True,disarmed=True,offboard_released=True,
                    max_effective_revision=2,collision=False,success=False)
        self.assertTrue(verdict.cancel_case_passed(result))
        for key in ('cancel_ack','land_success','offboard_released','fault_injected'):
            bad=dict(result);bad[key]=False
            self.assertFalse(verdict.cancel_case_passed(bad))
        result['max_effective_revision']=True
        self.assertFalse(verdict.cancel_case_passed(result))


if __name__ == '__main__':
    unittest.main()
