import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/tool/system/ros2_adapter'), str(ROOT/'src/tool/system/motion_resource')]
from uav_system_tool.policy import validate
from uav_motion_resource.lease import MotionLease


class PolicyTests(unittest.TestCase):
    def test_parameters(self):
        self.assertEqual(validate(1,3.,.5,60.), '')
        self.assertEqual(validate(2,0.,.5,90.), '')
        for values in ((0,3,.5,60), (1,float('nan'),.5,60), (1,.1,.5,60),
                       (1,3,0,60), (2,0,.5,float('inf')), (2,0,.5,0)):
            self.assertTrue(validate(*values))


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def test_exclusion_and_confirmed_handoff(self):
        a = MotionLease('/drone0', self.temp.name)
        b = MotionLease('drone0/', self.temp.name)
        self.assertTrue(a.acquire('system.takeoff'))
        self.assertFalse(a.acquire('again'))
        self.assertFalse(b.acquire('navigation.super'))
        a.release(True)
        self.assertTrue(b.acquire('navigation.super'))
        b.release(True)

    def test_unconfirmed_stop_is_sticky(self):
        a = MotionLease('/drone0', self.temp.name)
        self.assertTrue(a.acquire('navigation.super'))
        a.release(False)
        self.assertFalse(MotionLease('/drone0', self.temp.name).acquire('system.land'))

    def test_vehicle_isolation(self):
        a = MotionLease('/drone0', self.temp.name)
        b = MotionLease('/drone1', self.temp.name)
        self.assertTrue(a.acquire('takeoff'))
        self.assertTrue(b.acquire('takeoff'))
        a.release(True)
        b.release(True)

    def test_process_death_keeps_blocked_record(self):
        script = "from uav_motion_resource.lease import MotionLease; import os,sys; a=MotionLease('/drone0',sys.argv[1]); assert a.acquire('navigation.super'); os._exit(0)"
        env = dict(os.environ, PYTHONPATH=str(ROOT/'src/tool/system/motion_resource'))
        subprocess.run([sys.executable,'-c',script,self.temp.name],check=True,env=env)
        self.assertFalse(MotionLease('/drone0',self.temp.name).acquire('system.takeoff'))

    def test_cross_process_exclusion(self):
        a = MotionLease('/drone0',self.temp.name)
        self.assertTrue(a.acquire('takeoff'))
        script = "from uav_motion_resource.lease import MotionLease; import sys; assert not MotionLease('/drone0',sys.argv[1]).acquire('navigation.super')"
        env = dict(os.environ,PYTHONPATH=str(ROOT/'src/tool/system/motion_resource'))
        subprocess.run([sys.executable,'-c',script,self.temp.name],check=True,env=env)
        a.release(True)


if __name__=='__main__':
    unittest.main()
