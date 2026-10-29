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
import cv2

from localization.apriltag_geometry import (camera_rotation, project_floor_tag,
                                            floor_tag_yaw_offset, quaternion_matrix)
from localization.test_fusion import quaternion
from localization.test_ros_node import Pose, Stamp


class AprilTagGeometryTests(unittest.TestCase):
    K = [300, 0, 160, 0, 300, 120, 0, 0, 1]

    def project(self, pixel=(160, 120), height=1.0, quaternion=(0, 0, 0, 1),
                pitch=60, offset=(0, 0, 0), plane=0):
        return project_floor_tag(pixel, self.K, [0, 0, 0, 0, 0],
            (0, 0, height), quaternion, offset, camera_rotation(pitch),
            plane_z=plane, max_distance=5.0)

    def test_off_center_pinhole_ray_lands_right_and_forward(self):
        np.testing.assert_allclose(self.project(pixel=(190, 120)),
                                   (.2/math.sqrt(3), 1/math.sqrt(3), 0), atol=1e-7)

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

    def test_cannot_intersect_upwards_or_distant_plane(self):
        with self.assertRaises(ValueError):
            self.project(height=.5, plane=1)
        with self.assertRaises(ValueError):
            project_floor_tag((160,120), self.K, [], (0,0,5), (0,0,0,1),
                              (0,0,0), camera_rotation(60), max_distance=2)

    def test_yaw_reference_recovers_offset_with_tilt_mounting_distortion_and_wrap(self):
        # Independently render a square whose printed top is world +Y.
        points = np.array([[-.2, .2, 0], [.2, .2, 0], [.2, -.2, 0], [-.2, -.2, 0]])
        mounting = camera_rotation(60, 12, -8)
        k = np.array(self.K, dtype=float).reshape(3, 3)
        for yaw in (0, .7, math.pi-.01, -math.pi+.01):
            for model in ('plumb_bob', 'equidistant'):
                actual = quaternion(.08, -.12, yaw)
                biased = quaternion(.08, -.12, yaw+.25)
                rotation = quaternion_matrix(actual).dot(mounting)
                camera = points.dot(rotation) - np.array([0, 0, 1]).dot(rotation)
                distortion = np.array([-.1, .02, 0, 0, 0] if model == 'plumb_bob' else [-.05, .01, 0, 0])
                if model == 'equidistant':
                    pixels = cv2.fisheye.projectPoints(camera.reshape(1, -1, 3),
                        np.zeros(3), np.zeros(3), k, distortion)[0].reshape(4, 2)
                else:
                    pixels = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, distortion)[0].reshape(4, 2)
                self.assertAlmostEqual(floor_tag_yaw_offset(pixels, k, distortion, biased,
                                                            mounting, model), -.25, places=5)
        with self.assertRaises(ValueError):
            floor_tag_yaw_offset(np.zeros((4, 2)), k, [], (0, 0, 0, 1), mounting)


class AprilTagMappingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.maps = []
        ros = ModuleType('rospy')
        # These expectations are worked out for a 60 degree mounting; set it explicitly.
        params = {'~camera_pitch_down_deg': 60.0}
        ros.get_param = lambda name, default: params.get(name, default)
        ros.Time = NS(now=lambda: Stamp(self.now))
        ros.Subscriber = ros.loginfo = ros.logwarn = ros.logwarn_throttle = mock.Mock()
        ros.Publisher = lambda topic, *args, **kwargs: NS(
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

    def observe(self, image_stamp, pose_stamp, pixel=(160, 120), size=(320, 240)):
        pose = Pose()
        pose.header.frame_id, pose.header.stamp = 'map', Stamp(pose_stamp)
        pose.pose.pose.position = NS(x=1, y=2, z=1)
        pose.pose.pose.orientation.w = 1
        self.node.pose_callback(pose)
        message = NS(header=NS(stamp=Stamp(image_stamp)), camera_online=True,
                     image_width=size[0], image_height=size[1], tag_ids=[0], zones=['PICKUP'],
                     center_x=[pixel[0]], center_y=[pixel[1]])
        self.node.detection_callback(message)
        return message

    def test_delayed_detections_publish_coordinates_once_per_image(self):
        # Pose updates can overtake image processing; older poses also remain valid.
        for index, pose_offset in enumerate((-.03, -.03, -.15)):
            self.now = 100 + index * .1
            message = self.observe(self.now - .1, self.now + pose_offset)
            published = len(self.maps)
            self.node.detection_callback(message)
            self.assertEqual(len(self.maps), published)
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
        published = len(self.maps)
        self.observe(self.now - .1, self.now)
        self.assertEqual(len(self.maps), published)
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
        self.assertEqual(len(self.node.tags[0]['samples']), 1)
        self.now = .1
        self.node.pose = None
        self.observe(0, 0)
        self.assertIsNone(self.node.pose)
        self.assertEqual(len(self.node.tags[0]['samples']), 1)

    def test_equidistant_camera_maps_an_off_center_tag(self):
        # CameraInfo reported by Drone 2. Generate a pixel independently using
        # theta_d = theta * (1 + k1*theta^2 + ... + k4*theta^8).
        k = [154.1638736, 0, 118.8348587, 0, 155.2131826, 153.5541936, 0, 0, 1]
        d = [-.04846490008, .03309016118, -.03411731679, .009736962306]
        self.node.camera_callback(NS(width=240, height=320, K=k, D=d,
                                     distortion_model='equidistant'))
        x, y = .5, .25
        radius = math.hypot(x, y)
        theta = math.atan(radius)
        distorted = theta * (1 + sum(c * theta**(2*(i+1)) for i, c in enumerate(d)))
        pixel = (k[2] + k[0]*x*distorted/radius, k[5] + k[4]*y*distorted/radius)
        for i in range(3):
            self.now = 100 + i*.1
            self.observe(self.now, self.now, pixel=pixel, size=(240, 320))
        result = self.maps[-1]
        self.assertEqual(result.tag_ids, [0])
        point = result.positions[0]
        down = math.sqrt(3)/2 + y/2
        np.testing.assert_allclose((point.x, point.y, point.z),
                                   (1 + x/down, 2 + (.5-y*math.sqrt(3)/2)/down, 0), atol=1e-7)

        # A malformed calibration update pauses observations without losing the map.
        self.node.camera_callback(NS(width=240, height=320, K=k, D=d + [0],
                                     distortion_model='equidistant'))
        self.now += .1
        published = len(self.maps)
        self.observe(self.now, self.now, pixel=pixel, size=(240, 320))
        self.assertEqual(len(self.maps), published)

    def test_moved_tag_replaces_old_position_after_consistent_distinct_frames(self):
        self.node.rotation = camera_rotation(90)

        def sample(x=160):
            self.now += .1
            return self.observe(self.now, self.now, pixel=(x, 120))

        def mapped_x():
            return self.maps[-1].positions[0].x

        for _ in range(12):
            sample()
        self.assertAlmostEqual(mapped_x(), 1)
        self.assertEqual(self.maps[-1].sample_counts, [3])
        sample(10)  # A lone outlier must not move the marker.
        self.assertAlmostEqual(mapped_x(), 1)
        for _ in range(3):
            sample()
        self.assertAlmostEqual(mapped_x(), 1)

        message = sample(310)  # Move 0.5 m, beyond max_tag_jump.
        for _ in range(3):
            self.node.detection_callback(message)
        sample(310)
        self.assertAlmostEqual(mapped_x(), 1)
        sample(310)
        self.assertAlmostEqual(mapped_x(), 1.5)
        # Small movements also replace history rather than averaging forever.
        for _ in range(3):
            sample(280)
        self.assertAlmostEqual(mapped_x(), 1.4)

        sample(10)
        self.now += 1  # A gap discards unfinished confirmation, not the marker.
        sample(10)
        sample(10)
        self.assertAlmostEqual(mapped_x(), 1.4)
        sample(10)
        self.assertAlmostEqual(mapped_x(), .5)


if __name__ == '__main__':
    unittest.main()
