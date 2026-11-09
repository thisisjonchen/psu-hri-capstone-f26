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

from localization.apriltag_geometry import camera_rotation, project_floor_tag, quaternion_matrix
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


class AprilTagMappingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.maps, self.payload_points = [], []
        ros = ModuleType('rospy')
        # Exercise the mapper's default 45 degree camera mounting.
        params = {'~payload_plane_z': .25}
        ros.get_param = lambda name, default: params.get(name, default)
        ros.Time = NS(now=lambda: Stamp(self.now))
        ros.Subscriber = ros.loginfo = ros.logwarn = ros.logwarn_throttle = mock.Mock()
        reports = {'/pidrone/apriltag/map': self.maps,
                   '/pidrone/payload/map_point': self.payload_points}
        ros.Publisher = lambda topic, *args, **kwargs: NS(
            publish=lambda msg: reports[topic].append(copy.deepcopy(msg)))
        modules = {'rospy': ros}
        for package, messages in [
                ('geometry_msgs', {'Point': NS, 'PoseWithCovarianceStamped': Pose}),
                ('pidrone_pkg', {'PayloadDetection': NS,
                    'PayloadMapPoint': lambda: NS(header=NS(), point=NS(), valid=False),
                    'TagDetection': NS, 'TagMap': lambda: NS(
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
                                     D=[0] * 5, distortion_model='plumb_bob',
                                     header=NS(frame_id='camera')))

    def set_pose(self, pose_stamp, position=(1, 2, 1), yaw=0, sign=1):
        pose = Pose()
        pose.header.frame_id, pose.header.stamp = 'map', Stamp(pose_stamp)
        pose.pose.pose.position = NS(x=position[0], y=position[1], z=position[2])
        pose.pose.pose.orientation = NS(x=0, y=0, z=sign*math.sin(yaw/2), w=sign*math.cos(yaw/2))
        self.node.pose_callback(pose)

    def observe(self, image_stamp, pose_stamp, pixel=(160, 120), size=(320, 240)):
        self.set_pose(pose_stamp)
        message = NS(header=NS(stamp=Stamp(image_stamp)), camera_online=True,
                     image_width=size[0], image_height=size[1], tag_ids=[0], zones=['PAYLOAD'],
                     center_x=[pixel[0]], center_y=[pixel[1]])
        self.node.detection_callback(message)
        return message

    def test_payload_projects_at_configured_height_and_invalidates_without_fresh_input(self):
        self.set_pose(99.85)
        self.set_pose(99.98)
        detection = NS(header=NS(stamp=Stamp(99.9), frame_id='camera'), camera_online=True,
                       detected=True, image_width=320, image_height=240, center_x=190, center_y=120)
        self.node.payload_callback(detection)
        result, = self.payload_points
        self.assertTrue(result.valid)
        self.assertEqual((result.header.frame_id, result.header.stamp.to_sec(), result.generation),
                         ('map', 99.9, 1))
        np.testing.assert_allclose((result.point.x, result.point.y, result.point.z),
                                   (1 + .075*math.sqrt(2), 2.75, .25), atol=1e-7)
        self.node.payload_callback(detection)
        self.assertEqual(len(self.payload_points), 1)

        self.now = 100.4
        detection.header.stamp = Stamp(100.38)
        self.node.payload_callback(detection)  # The fused pose has expired.
        self.assertFalse(self.payload_points[-1].valid)
        self.assertEqual(self.payload_points[-1].header.stamp.to_sec(), 99.9)
        self.set_pose(100.36)
        self.set_pose(self.now)
        self.node.payload_callback(detection)
        self.assertTrue(self.payload_points[-1].valid)
        detection.detected = False  # Height gate can withdraw the same frame.
        self.node.payload_callback(detection)
        self.assertFalse(self.payload_points[-1].valid)
        self.assertEqual(self.maps[-1].tag_ids, [])

    def test_delayed_detections_publish_coordinates_once_per_image(self):
        # Use the image-time midpoint during translation and yaw, even with
        # opposite quaternion signs on the bracketing poses.
        for index in range(3):
            self.now = 100 + index * .1
            self.set_pose(self.now - .08, position=(.9, 2, 1), yaw=-.2)
            self.set_pose(self.now - .02, position=(1.1, 2, 1), yaw=.2, sign=-1)
            message = self.observe(self.now - .05, self.now - .02)
            published = len(self.maps)
            self.node.detection_callback(message)
            self.assertEqual(len(self.maps), published)
            self.assertEqual(self.maps[-1].tag_ids, [0] if index == 2 else [])
        result = self.maps[-1]
        self.assertEqual(result.header.frame_id, 'map')
        self.assertEqual(result.sample_counts, [3])
        point = result.positions[0]
        np.testing.assert_allclose((point.x, point.y, point.z),
                                   (1, 3, 0), atol=1e-7)
        # Losing readiness pauses sampling without erasing measured landmarks.
        self.node.ready_callback(NS(data=False))
        self.now += .1
        published = len(self.maps)
        self.observe(self.now - .1, self.now)
        self.assertEqual(len(self.maps), published)
        self.node.generation_callback(NS(data=2))
        self.assertEqual(self.maps[-1].tag_ids, [])
        self.assertIsNone(self.node.pose)

    def test_unbracketed_image_can_retry_and_history_resets_on_readiness_loss(self):
        message = self.observe(99.95, 99.9)
        self.assertEqual(self.node.tags, {})
        self.set_pose(99.98)
        self.node.detection_callback(message)
        self.assertEqual(len(self.node.tags[0]['samples']), 1)
        self.node.ready_callback(NS(data=False))
        self.node.ready_callback(NS(data=True))
        self.set_pose(100)
        message.header.stamp = Stamp(99.99)
        self.node.detection_callback(message)
        self.assertEqual(len(self.node.tags[0]['samples']), 1)
        self.node.generation_callback(NS(data=2))
        self.assertEqual(len(self.node.poses), 0)

    def test_timing_fix_does_not_accept_stale_future_or_zero_stamps(self):
        for image_stamp, pose_stamp in ((99.5, 100), (99.9, 99.5),
                                       (100.1, 100), (99.9, 100.1)):
            self.node.pose = None
            self.node.poses.clear()
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
        down = (1 + y) / math.sqrt(2)
        np.testing.assert_allclose((point.x, point.y, point.z),
                                   (1 + x/down, 2 + (1-y)/(1+y), 0), atol=1e-7)

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

    def test_measured_square_recovers_tag_center_above_floor(self):
        self.node.rotation = camera_rotation(90)
        self.node.tag_sizes[0] = .1
        # A 10 cm tag spans 60 px at 0.5 m depth with a 300 px focal length.
        # Its center is 60 px right of the optical axis, i.e. 10 cm right.
        for _ in range(3):
            self.now += .1
            self.set_pose(self.now)
            self.node.detection_callback(NS(
                header=NS(stamp=Stamp(self.now)), camera_online=True,
                image_width=320, image_height=240, tag_ids=[0], zones=['PAYLOAD'],
                center_x=[220], center_y=[120],
                corner_x=[190, 250, 250, 190], corner_y=[150, 150, 90, 90]))
        point, = self.maps[-1].positions
        np.testing.assert_allclose((point.x, point.y, point.z), (1.1, 2, .5), atol=1e-7)


if __name__ == '__main__':
    unittest.main()
