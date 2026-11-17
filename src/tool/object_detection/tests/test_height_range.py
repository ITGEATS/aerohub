"""Independent pinhole fixtures for explicit person-height ranging."""
import unittest
import numpy as np
from uav_object_detection.height_range import locate_known_height


class HeightRangeTests(unittest.TestCase):
    def setUp(self):
        self.k = [320., 320., 320., 240.]
        self.r = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        self.t = np.array([0.,0.,1.1])

    def fixture(self, rotation, center=(8.,0.,.9), height=1.7):
        center = np.asarray(center)
        top = center + np.array([0.,0.,height/2])
        bottom = center - np.array([0.,0.,height/2])
        uv = []
        for world in (top,bottom):
            x,y,z = rotation.T @ (world-self.t)
            uv.append([320+320*x/z,240+320*y/z])
        return [uv[0][0]-15,uv[0][1],uv[1][0]+15,uv[1][1]]

    def test_level_distance_and_center(self):
        box = self.fixture(self.r)
        point, report = locate_known_height(box,self.k,(640,480),self.r,self.t,1.7)
        np.testing.assert_allclose(point,[8.,0.,.9],atol=1e-9)
        self.assertAlmostEqual(report['optical_z_m'],320*1.7/(box[3]-box[1]))

    def test_pitched_camera_matches_independent_projection(self):
        for angle in (-.3,-.15,.15,.3):
            c,s = np.cos(angle),np.sin(angle)
            r = self.r @ np.array([[1.,0.,0.],[0.,c,-s],[0.,s,c]])
            point,_ = locate_known_height(self.fixture(r),self.k,(640,480),r,self.t,1.7)
            np.testing.assert_allclose(point,[8.,0.,.9],atol=1e-9)

    def test_height_error_scales_range(self):
        box = self.fixture(self.r)
        p,_ = locate_known_height(box,self.k,(640,480),self.r,self.t,1.87)
        self.assertAlmostEqual(p[0],8.8)

    def test_truncation_small_and_nonfinite_rejected(self):
        for box in ([300,0,340,200],[300,200,340,480],[0,200,340,300],
                    [300,200,340,210],[300,float('nan'),340,300]):
            with self.assertRaises(ValueError):
                locate_known_height(box,self.k,(640,480),self.r,self.t,1.7)

    def test_invalid_height_rotation_and_overhead_rejected(self):
        box = self.fixture(self.r)
        for height,r in [(0,self.r),(-1,self.r),(float('nan'),self.r),(1.7,np.eye(3)),(1.7,self.r*2)]:
            with self.assertRaises(ValueError):
                locate_known_height(box,self.k,(640,480),r,self.t,height)


if __name__ == '__main__':
    unittest.main(verbosity=2)
