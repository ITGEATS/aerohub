import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import reset_sim
import run_super


class ResetTest(unittest.TestCase):
    def test_live_mission_and_armed_px4_can_be_abandoned(self):
        flight='aerohub-yopo-course-flight-20261116-155759-997920'
        compute=flight.replace('-flight-','-compute-')
        active='\n'.join([run_super.PX4,flight,compute,'unrelated-flight-1'])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'log/projectairsim').mkdir(parents=True)
            with patch.object(reset_sim,'ROOT',root), patch.object(run_super,'capture',return_value=active), patch.object(run_super,'run') as run, patch.object(run_super,'manager') as manager, patch.object(run_super,'status',side_effect=AssertionError('reset must not require armed feedback')):
                reset_sim.reset()
                calls=[call.args[0] for call in run.call_args_list]
                self.assertEqual(calls[0],['docker','stop','--timeout','3',run_super.PX4])
                self.assertEqual(calls[1],['docker','stop','--timeout','2',flight,compute])
                manager.assert_called_once_with('stop')

    def test_scope_does_not_match_other_demos(self):
        self.assertFalse(reset_sim.OWNED.fullmatch('unrelated-flight-1'))
        self.assertFalse(reset_sim.OWNED.fullmatch('aerohub-yopo-course-flight-foreign'))


if __name__=='__main__':
    unittest.main()
