import math
import unittest

from uav_falcon_adapter.sensor_pose import compose, interpolate, rotate


class SensorPoseTests(unittest.TestCase):
    def test_interpolation_uses_measurement_timestamp(self):
        before = (1.0, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
        after = (3.0, (4.0, 2.0, 1.0), (0.0, 0.0, 1.0, 0.0))
        position, orientation = interpolate(before, after, 2.0)
        self.assertEqual(position, (2.0, 1.0, 1.0))
        forward = rotate(orientation, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(forward[0], 0.0, places=6)
        self.assertAlmostEqual(forward[1], 1.0, places=6)

    def test_camera_mount_is_composed_in_body_frame(self):
        half = math.sqrt(0.5)
        position, _ = compose(
            (10.0, 20.0, 3.0), (0.0, 0.0, half, half),
            (1.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5))
        self.assertAlmostEqual(position[0], 10.0, places=6)
        self.assertAlmostEqual(position[1], 21.0, places=6)
        self.assertAlmostEqual(position[2], 3.0, places=6)


if __name__ == "__main__":
    unittest.main()
