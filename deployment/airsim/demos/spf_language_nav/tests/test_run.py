import importlib.util
from pathlib import Path
import shlex
import tempfile
from types import SimpleNamespace
import unittest


spec = importlib.util.spec_from_file_location('spf_run', Path(__file__).resolve().parents[1] / 'run.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class MissionArguments(unittest.TestCase):
    def test_language_and_case_are_literal_shell_arguments(self):
        args = SimpleNamespace(case="case with spaces", instruction='靠近 "red ball"; $(touch /tmp/unwanted) `echo x`',
                               navigation_timeout=60, timeout=100, fault='none',
                               target=[0, 10, 1.5], spawn=[0, 0, 0])
        with tempfile.TemporaryDirectory() as tmp:
            script = runner.mission_script(Path(tmp), args, '127.0.0.1')
        argv = shlex.split(script.split(' && exec ', 1)[1])
        self.assertEqual(argv[argv.index('--instruction') + 1], args.instruction)
        self.assertEqual(argv[argv.index('--output') + 1], '/evidence/' + args.case)
        self.assertEqual(argv[argv.index('--spawn') + 1:], ['0', '0', '0'])


if __name__ == '__main__':
    unittest.main()
