"""ROS imports required; pure geometry and capture-transition policy tests."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_pointing import CameraPointing, pointing_angles


class PointingTests(unittest.TestCase):
    def test_vision_position_not_actor_identity_controls_angles(self):
        pitch,yaw=pointing_angles((0,0,3),0.,(10,10,1))
        self.assertLess(pitch,0.)
        self.assertAlmostEqual(yaw,-45.)
        self.assertEqual(pointing_angles((0,0,0),0.,(1,0,100))[0],20.)
        with self.assertRaises(ValueError):pointing_angles((0,0,0),0.,(.1,0,0))

    def test_mount_history_rejects_ambiguous_and_unconfirmed_capture(self):
        controller=CameraPointing.__new__(CameraPointing)
        controller.error='';controller.before=None
        controller.history=[(None,0,{'Yaw':0}),(100,150,{'Yaw':20})]
        self.assertEqual(controller.origin_at(99),{'Yaw':0})
        for stamp in (100,120,150):self.assertIsNone(controller.origin_at(stamp))
        self.assertEqual(controller.origin_at(151),{'Yaw':20})
        controller.before=170
        self.assertIsNone(controller.origin_at(180))
        self.assertEqual(controller.origin_at(160),{'Yaw':20})
        controller.error='RPC timeout'
        self.assertIsNone(controller.origin_at(160))
