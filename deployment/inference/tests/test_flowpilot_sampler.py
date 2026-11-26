"""Compare the FlowPilot reference sampler to the real FollowTimedPath header."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
PUBLIC = Path(os.environ.get('AEROHUB_REPO', ROOT))
sys.path.insert(0, str(ROOT / 'src/tool/navigation/planner/flowpilot/ros2_adapter'))
from uav_flowpilot_planner.curve import Segment


class SamplerTest(unittest.TestCase):
    def test_interpolation_matches_behavior_between_points(self):
        controls = np.random.default_rng(21).normal(size=(8, 3))
        segment = Segment(controls, [3., 2., 1.], np.eye(3), 10., 10.215, np.pi-.03)
        lines = [f'{segment.start} {len(segment.times)}']
        for elapsed in segment.times:
            pva, yaw = segment.sample(segment.start + elapsed)
            lines.append(' '.join(map(str, [elapsed, *pva.ravel(), yaw])))
        epochs = np.r_[segment.start-.1, np.linspace(segment.start, segment.end, 111), segment.end+.1]
        lines.extend(map(str, epochs))
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'sampler'
            subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(PUBLIC / 'src/behavior/follow_path/include'),
                            str(Path(__file__).with_name('flowpilot_sampler.cpp')), '-o', str(binary)], check=True)
            output = subprocess.check_output([str(binary)], input='\n'.join(lines), text=True)
        actual = np.array([list(map(float, row.split())) for row in output.splitlines()])
        expected = np.array([np.r_[segment.execution_sample(t)[0].ravel(), segment.execution_sample(t)[1]]
                             for t in epochs])
        np.testing.assert_allclose(actual[:, :9], expected[:, :9], atol=1e-7, rtol=1e-7)
        delta = actual[:, 9] - expected[:, 9]
        np.testing.assert_allclose(np.arctan2(np.sin(delta), np.cos(delta)), 0., atol=1e-10)


if __name__ == '__main__':
    unittest.main()
