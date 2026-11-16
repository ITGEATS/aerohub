"""Conservative box foreground geometry, explicit synthetic depth."""
import unittest
import numpy as np
from uav_object_detection.box_depth import locate_box


class BoxDepthTest(unittest.TestCase):
    def setUp(self):
        self.depth=np.full((40,40),8.,float)
        self.depth[10:30,10:30]=2.
        self.k=[20.,20.,20.,20.]

    def locate(self, depth):
        return locate_box(depth,[0.,0.,40.,40.],self.k,np.eye(3),np.array([3.,1.,0.]))

    def test_surface_and_nonidentity_transform(self):
        point,diagnostic=self.locate(self.depth)
        np.testing.assert_allclose(point,[2.95,.95,2.])
        self.assertEqual(diagnostic['valid_pixels'],400)
        self.assertEqual(diagnostic['depth_m'],2.)

    def test_flat_background_missing_competing_and_no_center(self):
        cases=[]
        cases.append(np.full((40,40),8.))
        d=self.depth.copy();d[10:30,10:30]=np.nan;cases.append(d)
        d=self.depth.copy();d[10:30,10:20]=4.;cases.append(d)
        d=self.depth.copy();d[15:25,15:25]=8.;cases.append(d)
        for depth in cases:
            with self.subTest(depth=depth[20,20]):
                with self.assertRaises(ValueError):self.locate(depth)


if __name__=='__main__':unittest.main(verbosity=2)
