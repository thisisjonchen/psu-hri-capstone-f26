"""Hardware-free checks for optical-to-UWB ray/plane math."""
import math
import unittest

import numpy as np

from localization.apriltag_geometry import camera_rotation, project_floor_tag


class AprilTagGeometryTests(unittest.TestCase):
    K = [300, 0, 160, 0, 300, 120, 0, 0, 1]

    def project(self, pixel=(160, 120), height=1.0, quaternion=(0, 0, 0, 1),
                pitch=60, offset=(0, 0, 0), plane=0):
        return project_floor_tag(pixel, self.K, [0, 0, 0, 0, 0],
            (0, 0, height), quaternion, offset, camera_rotation(pitch),
            plane_z=plane, max_distance=5.0)

    def test_center_ray_at_sixty_degrees_lands_forward(self):
        result = self.project()
        np.testing.assert_allclose(result, (0, 1/math.sqrt(3), 0), atol=1e-7)

    def test_vertical_camera_points_directly_below_camera(self):
        np.testing.assert_allclose(self.project(pitch=90, offset=(.1, .2, -.1)),
                                   (.1, .2, 0), atol=1e-7)

    def test_yaw_and_height_change_intersection(self):
        # +90 degrees around map Z rotates the forward ray toward map -X.
        s = math.sqrt(.5)
        np.testing.assert_allclose(self.project(height=2, quaternion=(0, 0, s, s)),
                                   (-2/math.sqrt(3), 0, 0), atol=1e-7)
        np.testing.assert_allclose(self.project(height=1, plane=.5),
                                   (0, .5/math.sqrt(3), .5), atol=1e-7)

    def test_image_right_corresponds_to_body_right(self):
        p = self.project(pixel=(190, 120))
        self.assertGreater(p[0], 0)
        self.assertGreater(p[1], 0)

    def test_cannot_intersect_upwards_or_distant_plane(self):
        with self.assertRaises(ValueError):
            self.project(height=.5, plane=1)
        with self.assertRaises(ValueError):
            project_floor_tag((160,120), self.K, [], (0,0,5), (0,0,0,1),
                              (0,0,0), camera_rotation(60), max_distance=2)


if __name__ == '__main__':
    unittest.main()
