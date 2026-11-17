import math
import sys
from pathlib import Path
import unittest
from simulator.projectairsim.sensor_contract import streams, extrinsic
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'projectairsim'))
from gimbal import interpolate, stabilized_transform
from uav_as2_sensor_adapter.conversion import multiply


def rotate(q,v):
    return multiply(multiply(q,tuple(v)+(0.,)),(-q[0],-q[1],-q[2],q[3]))[:3]


class GimbalTests(unittest.TestCase):
    def test_combined_body_tilt_preserves_horizontal_view_and_yaw(self):
        origin=dict(X=.6,Y=0.,Z=-.8,Roll=0.,Pitch=0.,Yaw=0.)
        body=extrinsic(dict(Roll=23.,Pitch=-35.,Yaw=70.))[1]
        xyz,optical=stabilized_transform(body,origin)
        forward=rotate(multiply(body,optical),(0.,0.,1.))
        self.assertAlmostEqual(forward[2],0.,places=8)
        self.assertAlmostEqual(math.atan2(forward[1],forward[0]),math.radians(-70.))
        world_offset=rotate(body,xyz)
        expected=rotate(extrinsic(dict(Yaw=70.))[1],(.6,0.,.8))
        for a,b in zip(world_offset,expected):self.assertAlmostEqual(a,b)

    def test_slerp_shortest_arc_and_invalid_rotation(self):
        self.assertEqual(interpolate((0.,0.,0.,1.),(0.,0.,0.,-1.),.5),(0.,0.,0.,1.))
        with self.assertRaises(ValueError):stabilized_transform((0.,0.,0.,0.),{})

    def test_inventory_marks_only_supported_mode_dynamic(self):
        camera=dict(id='Front',type='camera',enabled=True,**{'parent-link':'Frame',
            'gimbal':{'lock-roll':True,'lock-pitch':True,'lock-yaw':False},
            'capture-settings':[{'image-type':0,'capture-enabled':True}]})
        scene=dict(id='West',actors=[dict(name='Drone1',type='robot',**{'robot-config':dict(links=[dict(name='Frame')],sensors=[camera])})])
        result=streams(scene,dict(vehicles=[dict(name='Drone1',namespace='drone0')]))
        self.assertIsNotNone(result[0]['gimbal_origin'])
        self.assertTrue(result[0]['pose_source'].endswith('/robots/Drone1/actual_pose'))
        camera['gimbal']['lock-yaw']=True
        with self.assertRaises(ValueError):streams(scene,dict(vehicles=[dict(name='Drone1',namespace='drone0')]))
