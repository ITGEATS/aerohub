"""Compare a fixed native quintic with the default FollowTimedPath sampler."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream.resolve() / 'YOPO'))
    from policy.poly_solver import Poly5Solver, wrap_to_pi

    repo = Path(__file__).resolve().parents[6]
    sampler = repo / 'src/tool/navigation/planner/yopo/ros2_adapter/tests/behavior_math.cpp'
    duration = 1.5
    # Fixed nonzero boundary derivatives exercise acceleration and velocity ordering.
    poly = Poly5Solver(np.array([1., -2., 1.]), [.1, 0., .4], [.4, -.5, 0.],
                       [3., -1., .2], [-.1, .2, .3], [.2, .4, -.3], duration)

    def expected(t):
        return np.r_[poly.get_position(t), poly.get_velocity(t), poly.get_acceleration(t),
                     wrap_to_pi(3.05 + .3 * t)]

    knots = [0., .5, duration]
    queries = np.linspace(0., duration, 151)
    lines = [f'{len(knots)} {len(queries)}']
    lines.extend(' '.join(map(str, [t, *expected(t)])) for t in knots)
    lines.extend(map(str, queries))
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / ('sample.exe' if sys.platform == 'win32' else 'sample')
        subprocess.run(['g++', '-std=c++17', '-I' + str(repo / 'src/behavior/follow_path/include'),
                        str(sampler), '-o', str(binary)], check=True)
        output = subprocess.check_output([str(binary)], input='\n'.join(lines), text=True)
    actual = np.array([line.split() for line in output.splitlines()], dtype=float)
    reference = np.array([expected(t) for t in queries])
    np.testing.assert_allclose(actual[:, :9], reference[:, :9], atol=1e-8, rtol=1e-7)
    np.testing.assert_allclose(wrap_to_pi(actual[:, 9] - reference[:, 9]), 0., atol=1e-10)
    print(f'PASS: {len(queries)} PVA/yaw samples, default FollowTimedPath')


if __name__ == '__main__':
    main()
