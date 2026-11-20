"""Placement selection and evidence compatibility, without starting a mission."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from evaluate import cleanup_checks
from split_pc import verify_processes

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('flight_for_split_test', HERE/'run.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


class SplitPlacement(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name)
        self.root = self.folder/'run'
        self.config = {'run_root': str(self.root), 'release': str(self.folder/'release'), 'project': 'test-split'}

    def deployment(self):
        path = self.folder/'local.json'
        path.write_text(json.dumps(self.config))
        return entry.FlightDeployment(path)

    def test_new_default_and_opt_out(self):
        self.assertTrue(self.deployment().split_native)
        self.config['native_placement'] = 'shared'
        self.assertFalse(self.deployment().split_native)
        self.config['native_placement'] = 'unknown'
        with self.assertRaises(ValueError):
            self.deployment()

    def test_resume_uses_recorded_layout(self):
        self.root.mkdir()
        (self.root/'local.json').write_text(json.dumps(self.config))
        self.assertFalse(self.deployment().split_native)
        (self.root/'swarm-topology.json').write_text('{}')
        self.assertTrue(self.deployment().split_native)
        self.config['native_placement'] = 'shared'
        with self.assertRaises(ValueError):
            self.deployment()

    def test_stored_cleanup_rejects_a_failed_recorder(self):
        self.root.mkdir()
        local = dict(self.config, native_placement='per_vehicle', diagnostics={'enabled': True})
        (self.root/'local.json').write_text(json.dumps(local))
        (self.root/'compose.json').write_text('{"services": {}}')
        (self.root/'core-final.json').write_text('[]')
        names = ['drone0','drone1']
        roles = ['native-'+n for n in names]+['camera-'+n for n in names]+['adapter','inputs','observer','overhead','diagnostics']
        states = [dict(name='test-split-'+role, State=dict(Running=False,ExitCode=0)) for role in roles]
        path = self.root/'cleanup-result.json'
        fleet = {'vehicles':[{'namespace':n} for n in names]}
        path.write_text(json.dumps(states))
        self.assertTrue(cleanup_checks(self.root,fleet)['pass'])
        states[-1]['State']['ExitCode'] = 1
        path.write_text(json.dumps(states))
        self.assertFalse(cleanup_checks(self.root,fleet)['pass'])

    def test_wrong_worker_in_an_owned_container_is_rejected(self):
        d = self.deployment()
        self.root.mkdir()
        for v in d.fleet['vehicles']:
            ns = v['namespace']
            out = self.root/('native-'+ns)
            out.mkdir()
            (out/'worker-topology.json').write_text(json.dumps([dict(identity=ns,pid=1)]))

        def inspect(args, text):
            name = args[2]
            if args[1] == 'top':
                return 'PID COMMAND\n7 /install/exploration_node --ros-args -r __node:=racer_drone0\n'
            return json.dumps([dict(Name='/'+name, Id=name, Image=d.images['algorithm'],
                                    State={'Running':True}, Config={'Labels':{'aerohub.racer.owner':d.prefix}})])

        with patch('split_pc.subprocess.check_output',side_effect=inspect):
            with self.assertRaises(RuntimeError):
                verify_processes(d)
