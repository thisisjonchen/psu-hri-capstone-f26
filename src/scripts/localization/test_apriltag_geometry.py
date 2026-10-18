"""Hardware-free checks for optical-to-UWB geometry and mapper publication."""
import math
import copy
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

import numpy as np

from localization.apriltag_geometry import camera_rotation, project_floor_tag
from localization.test_ros_node import Pose, Stamp


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


class AprilTagMappingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.maps = []
        ros = ModuleType('rospy')
        ros.get_param = lambda name, default: default
        ros.Time = NS(now=lambda: Stamp(self.now))
        ros.Subscriber = ros.loginfo = ros.logwarn = ros.logwarn_throttle = mock.Mock()
        ros.Publisher = lambda *args, **kwargs: NS(
            publish=lambda msg: self.maps.append(copy.deepcopy(msg)))
        modules = {'rospy': ros}
        for package, messages in [
                ('geometry_msgs', {'Point': NS, 'PoseWithCovarianceStamped': Pose}),
                ('pidrone_pkg', {'TagDetection': NS, 'TagMap': lambda: NS(
                    header=NS(), tag_ids=[], zones=[], positions=[], sample_counts=[])}),
                ('sensor_msgs', {'CameraInfo': NS}),
                ('std_msgs', {'Bool': NS, 'UInt32': NS})]:
            modules[package] = ModuleType(package)
            modules[package + '.msg'] = ModuleType(package + '.msg')
            for name, value in messages.items():
                setattr(modules[package + '.msg'], name, value)
        patch = mock.patch.dict(sys.modules, modules)
        patch.start()
        self.addCleanup(patch.stop)
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'apriltag_mapping_node')
        self.node = runpy.run_path(script)['AprilTagMappingNode']()
        self.node.generation_callback(NS(data=1))
        self.node.ready_callback(NS(data=True))
        self.node.camera_callback(NS(width=320, height=240, K=AprilTagGeometryTests.K,
                                     D=[0] * 5, distortion_model='plumb_bob'))

    def observe(self, image_stamp, pose_stamp):
        pose = Pose()
        pose.header.frame_id, pose.header.stamp = 'map', Stamp(pose_stamp)
        pose.pose.pose.position = NS(x=1, y=2, z=1)
        pose.pose.pose.orientation.w = 1
        self.node.pose_callback(pose)
        message = NS(header=NS(stamp=Stamp(image_stamp)), camera_online=True,
                     image_width=320, image_height=240, tag_ids=[0], zones=['PICKUP'],
                     center_x=[160], center_y=[120])
        self.node.detection_callback(message)
        return message

    def test_delayed_detections_publish_coordinates_once_per_image(self):
        # Pose updates can overtake image processing; older poses also remain valid.
        for index, pose_offset in enumerate((-.03, -.03, -.15)):
            self.now = 100 + index * .1
            message = self.observe(self.now - .1, self.now + pose_offset)
            self.node.detection_callback(message)
            self.assertEqual(self.node.tags[0]['count'], index + 1)
            self.assertEqual(self.maps[-1].tag_ids, [0] if index == 2 else [])
        result = self.maps[-1]
        self.assertEqual(result.header.frame_id, 'map')
        self.assertEqual(result.sample_counts, [3])
        point = result.positions[0]
        np.testing.assert_allclose((point.x, point.y, point.z),
                                   (1, 2 + 1/math.sqrt(3), 0), atol=1e-7)
        # Losing readiness pauses sampling without erasing measured landmarks.
        self.node.ready_callback(NS(data=False))
        self.now += .1
        self.observe(self.now - .1, self.now)
        self.assertEqual(self.node.tags[0]['count'], 3)
        self.node.generation_callback(NS(data=2))
        self.assertEqual(self.maps[-1].tag_ids, [])
        self.assertIsNone(self.node.pose)

    def test_timing_fix_does_not_accept_stale_future_or_zero_stamps(self):
        for image_stamp, pose_stamp in ((99.5, 100), (99.9, 99.5),
                                       (100.1, 100), (99.9, 100.1)):
            self.node.pose = None
            self.observe(image_stamp, pose_stamp)
            self.assertEqual(self.node.tags, {})
        # Cached poses must expire even if callbacks stop arriving.
        self.observe(99.95, 99.95)
        self.now = 100.4
        self.observe(100.35, 99.95)
        self.assertEqual(self.node.tags[0]['count'], 1)
        self.now = .1
        self.node.pose = None
        self.observe(0, 0)
        self.assertIsNone(self.node.pose)
        self.assertEqual(self.node.tags[0]['count'], 1)


if __name__ == '__main__':
    unittest.main()
