"""Launch plans and executable wiring; no ROS, workers, scenes or flight."""
import ast
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]


class DemoConfigTests(unittest.TestCase):
    def plan(self, script, *args):
        return json.loads(subprocess.check_output([sys.executable, str(ROOT/script),
            '--output', str(ROOT/'log/never-started-dry-run'), '--dry-run', *args], text=True))

    def test_tracking_ground_plan(self):
        plan = self.plan('deployment/target_tracking/airsim/run.py')
        self.assertEqual((plan['backend'], plan['model_id'], plan['port'], plan['query']),
                         ('grounding_dino', 'grounding-dino-tiny', 8003, 'car'))
        self.assertEqual(plan['workspace'], 'log/object-detection-query/workspace')
        self.assertFalse(plan['worker_owned'])
        self.assertTrue(plan['observation_only'])

    def test_mountains_plan(self):
        plan = self.plan('deployment/target_tracking/airsim/mountains/run.py', '--scene-package', '/missing')
        self.assertEqual((plan['backend'], plan['model_id'], plan['port'], plan['query']),
                         ('grounding_dino', 'grounding-dino-tiny', 8003, 'car'))
        self.assertEqual(plan['worker_start'], 'deployment/inference/start_grounding_dino.sh')
        self.assertTrue(plan['observation_only'])

    def test_hover_is_explicit(self):
        plan = self.plan('deployment/target_tracking/airsim/mountains/run_hover.py', '--scene-package', '/missing')
        self.assertFalse(plan['observation_only'])
        self.assertEqual(plan['query'], 'car')

    def test_shared_yolo_default(self):
        plan = self.plan('deployment/object_detection/airsim/run.py')
        self.assertEqual((plan['backend'], plan['model_id'], plan['port']), ('yolo', 'yolo26n-seg', 8002))

    def test_all_probe_nodes_and_params(self):
        params = (ROOT/'deployment/target_tracking/params.yaml').read_text()
        for entry in ('detector_backend: grounding_dino', 'query: car', 'endpoint: http://127.0.0.1:8003',
                      'timeout_s: 10.0', 'snapshot_timeout: 1.0', 'max_capture_age: 0.5',
                      'box_threshold: 0.30', 'text_threshold: 0.25'):
            self.assertIn(entry, params)
        for name in ('probe.py', 'mountains/probe.py', 'mountains/hover_probe.py'):
            source = (ROOT/'deployment/target_tracking/airsim'/name).read_text()
            tree = ast.parse(source)
            calls = [n.func.id for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
            self.assertEqual(calls.count('GroundingDinoNode'), 1)
            self.assertNotIn('YoloNode', calls)
            self.assertIn('/repo/deployment/target_tracking/params.yaml', source)
            self.assertNotIn('.detector.detect', source)

    def test_ground_query_propagation(self):
        source = (ROOT/'deployment/target_tracking/airsim/mountains/probe.py').read_text()
        self.assertIn("'--query',a.query", source)
        launch = (ROOT/'deployment/target_tracking/airsim/mountains/run.py').read_text()
        self.assertIn("command+=' --query '+shlex.quote(args.mission_query)", launch)
        self.assertNotIn('diagnose_classes.py', launch)
        self.assertNotIn('/tracking/install', launch)


if __name__ == '__main__':
    unittest.main()
