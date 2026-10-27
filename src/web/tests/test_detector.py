"""Exercise onboard tag and yellow-payload detectors with mocked ROS transport."""
import copy
import runpy
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

SCRIPT = Path(__file__).parents[2] / 'scripts' / 'PALS' / 'apriltag_detection_drone'
sys.path.insert(0, str(SCRIPT.parents[1]))


class TagDetection:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 100.0))
        self.camera_online = False
        self.tag_ids = []
        self.zones = []
        self.center_x = []
        self.center_y = []
        self.image_width = 0
        self.image_height = 0


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.messages = []
        self.references = []
        self.ros = ModuleType('rospy')
        self.ros.get_param = lambda name, default: default
        self.ros.Time = SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: self.now))
        self.ros.Publisher = lambda topic, *args, **kwargs: SimpleNamespace(
            publish=lambda msg: (self.references if topic.endswith('reference_corners')
                                 else self.messages).append(copy.deepcopy(msg)))
        self.ros.Subscriber = Mock()
        self.ros.loginfo = Mock()
        self.ros.logwarn_throttle = Mock()
        modules = {'rospy': self.ros}
        for package, message in [('sensor_msgs', 'CompressedImage'), ('pidrone_pkg', 'TagDetection')]:
            modules[package] = ModuleType(package)
            modules[package + '.msg'] = ModuleType(package + '.msg')
            setattr(modules[package + '.msg'], message, TagDetection if message == 'TagDetection' else object)
        modules['pidrone_pkg.msg'].TagCorners = SimpleNamespace
        self.mock_ros = patch.dict(sys.modules, modules)
        self.mock_ros.start()
        self.addCleanup(self.mock_ros.stop)
        self.clock = patch('time.time', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = runpy.run_path(str(SCRIPT))['AprilTagNode']()

    def frame(self, tag=None):
        image = np.full((320, 240), 255, dtype=np.uint8)
        if tag is not None:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            image[80:240, 40:200] = cv2.aruco.generateImageMarker(dictionary, tag, 160)
        ok, encoded = cv2.imencode('.jpg', image)
        self.assertTrue(ok)
        return SimpleNamespace(data=encoded.tobytes(), header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda stamp=self.now: stamp)))

    def test_real_camera_tags_publish_zones_and_clear_when_the_tag_leaves(self):
        self.ros.Subscriber.assert_called_once()
        self.assertEqual(self.ros.Subscriber.call_args.args[0], '/raspicam_node/image/compressed')
        for tag, zone in [(0, 'PICKUP'), (1, 'DROP_OFF'), (2, 'BORDER'), (3, 'UNKNOWN')]:
            self.node.image_callback(self.frame(tag))
            self.node.step()
            self.assertTrue(self.messages[-1].camera_online)
            self.assertEqual(self.messages[-1].tag_ids, [tag])
            self.assertEqual(self.messages[-1].zones, [zone])
            self.assertEqual(len(self.messages[-1].center_x), 1)
            self.assertEqual(len(self.messages[-1].center_y), 1)
            self.assertEqual((self.messages[-1].image_width, self.messages[-1].image_height), (240, 320))
        self.node.image_callback(self.frame())
        self.node.step()
        self.assertTrue(self.messages[-1].camera_online)
        self.assertEqual(self.messages[-1].tag_ids, [])

    def test_missing_stale_and_invalid_images_publish_offline_without_stale_tags(self):
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)

        old_frame = self.frame(0)
        self.now += 0.6
        self.node.image_callback(old_frame)
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.node.image_callback(self.frame(0))
        self.node.step()
        self.now += 0.6
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.assertEqual(self.messages[-1].tag_ids, [])
        invalid = self.frame()
        invalid.data = b'not an image'
        self.node.image_callback(invalid)
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.node.image_callback(self.frame(1))
        self.now += 0.6
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)

    def test_latest_frame_replaces_pending_work_instead_of_building_a_backlog(self):
        self.node.image_callback(self.frame(0))
        self.node.image_callback(self.frame(1))
        self.node.step()
        self.assertEqual(self.messages[-1].tag_ids, [1])
        self.assertIsNone(self.node.pending)

    def test_reference_tracks_printed_top_and_is_only_published_for_new_valid_tag5_frames(self):
        # Official AprilRobotics orientation: OpenCV's generator uses a different
        # printed orientation. Keep a 10x10 reference rather than guessing an axis.
        source = cv2.imread(str(Path(__file__).parent / 'fixtures/tag36_11_00005.png'), 0)
        self.assertIsNotNone(source)
        for turns in range(4):
            self.now += .1
            image = np.full((320, 240), 255, dtype=np.uint8)
            image[60:260, 20:220] = cv2.resize(np.rot90(source, turns), (200, 200),
                                             interpolation=cv2.INTER_NEAREST)
            message = self.frame()
            message.data = cv2.imencode('.jpg', image)[1].tobytes()
            self.node.image_callback(message)
            self.node.step()
            reference = self.references[-1]
            points = np.column_stack((reference.corner_x, reference.corner_y))
            top = points[:2].mean(axis=0) - points[2:].mean(axis=0)
            top /= np.linalg.norm(top)
            np.testing.assert_allclose(top, [(0, -1), (-1, 0), (0, 1), (1, 0)][turns], atol=.01)
            self.assertEqual(reference.tag_id, 5)
            self.assertEqual(reference.header.stamp.to_sec(), self.now)
            self.assertEqual((reference.image_width, reference.image_height), (240, 320))
            count = len(self.references)
            self.node.step()
            self.assertEqual(len(self.references), count)
        count = len(self.references)
        for tag in (0, None):
            self.now += .1
            self.node.image_callback(self.frame(tag))
            self.node.step()
            self.assertEqual(len(self.references), count)
        self.now += .6
        self.node.step()
        self.assertEqual(len(self.references), count)
        # An uncertain decode can remain visible in the UI, but cannot correct yaw.
        self.node.detector = SimpleNamespace(detect=lambda _: [SimpleNamespace(
            tag_id=5, center=(120, 160), hamming=1, decision_margin=100)])
        self.node.image_callback(self.frame())
        self.node.step()
        self.assertEqual(len(self.references), count)


class PayloadDetection:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 0.0))
        self.camera_online = self.detected = False
        for field in ('image_width', 'image_height', 'bbox_x', 'bbox_y', 'bbox_width',
                      'bbox_height', 'center_x', 'center_y', 'area'):
            setattr(self, field, 0)


class PayloadDetectorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.messages = []
        self.params = {}
        ros = ModuleType('rospy')
        ros.get_param = lambda name, default: self.params.get(name, default)
        ros.Time = SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: self.now))
        ros.Publisher = lambda *args, **kwargs: SimpleNamespace(
            publish=lambda msg: self.messages.append(copy.deepcopy(msg)))
        ros.Subscriber = Mock()
        ros.loginfo = ros.logwarn_throttle = Mock()
        modules = {'rospy': ros}
        for package, name, cls in [('sensor_msgs', 'CompressedImage', object),
                                   ('pidrone_pkg', 'PayloadDetection', PayloadDetection)]:
            modules[package] = ModuleType(package)
            modules[package + '.msg'] = ModuleType(package + '.msg')
            setattr(modules[package + '.msg'], name, cls)
        patcher = patch.dict(sys.modules, modules)
        patcher.start()
        self.addCleanup(patcher.stop)
        node_class = runpy.run_path(str(SCRIPT.with_name('payload_detection_drone')))['PayloadNode']
        node_class.step.__globals__['monotonic'] = lambda: self.now
        self.node_class = node_class
        self.node = node_class()

    def frame(self, regions=(), stamp=None):
        image = np.zeros((320, 240, 3), dtype=np.uint8)
        for x, y, w, h, color in regions:
            image[y:y+h, x:x+w] = color
        data = cv2.imencode('.jpg', image)[1].tobytes()
        timestamp = self.now if stamp is None else stamp
        return SimpleNamespace(data=data, header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda: timestamp), frame_id='camera'))

    def observe(self, regions=()):
        self.now += .1
        self.node.image_callback(self.frame(regions))
        self.node.step()
        return self.messages[-1]

    def test_yellow_region_reports_pixels_and_clears_when_absent(self):
        yellow = (0, 255, 255)
        msg = self.observe([(40, 80, 80, 100, yellow), (170, 20, 40, 50, (255, 0, 0)),
                            (190, 270, 5, 5, yellow)])
        self.assertTrue(msg.camera_online and msg.detected)
        self.assertEqual((msg.image_width, msg.image_height), (240, 320))
        self.assertAlmostEqual(msg.center_x, 79.5, delta=2)
        self.assertAlmostEqual(msg.center_y, 129.5, delta=2)
        self.assertAlmostEqual(msg.bbox_x, 40, delta=2)
        self.assertAlmostEqual(msg.bbox_y, 80, delta=2)
        self.assertAlmostEqual(msg.bbox_width, 80, delta=3)
        self.assertAlmostEqual(msg.bbox_height, 100, delta=3)
        self.assertGreater(msg.area, 7000)
        self.assertEqual(msg.header.stamp.to_sec(), self.now)
        self.assertEqual(msg.header.frame_id, 'camera')
        for regions in [[], [(20, 30, 60, 70, (0, 0, 255))], [(40, 80, 5, 5, yellow)]]:
            msg = self.observe(regions)
            self.assertTrue(msg.camera_online)
            self.assertFalse(msg.detected)
            self.assertEqual((msg.area, msg.bbox_width, msg.center_x), (0, 0, 0))

    def test_association_ignores_a_larger_distractor_and_expires_across_a_gap(self):
        yellow = (0, 255, 255)
        self.observe([(20, 20, 30, 30, yellow)])
        msg = self.observe([(25, 25, 30, 30, yellow), (150, 220, 70, 80, yellow)])
        self.assertAlmostEqual(msg.center_x, 39.5, delta=2)
        msg = self.observe([(150, 220, 70, 80, yellow)])
        self.assertTrue(msg.camera_online)
        self.assertFalse(msg.detected)  # No cached/predicted coordinates on a miss.
        self.now += .6
        msg = self.observe([(150, 220, 70, 80, yellow)])
        self.assertTrue(msg.detected)
        self.assertAlmostEqual(msg.center_x, 184.5, delta=2)

    def test_missing_invalid_and_replayed_images_never_renew_observation_time(self):
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        yellow = [(40, 80, 80, 100, (0, 255, 255))]
        msg = self.observe(yellow)
        measured_at = msg.header.stamp.to_sec()
        self.now += .1
        self.node.step()
        self.assertTrue(self.messages[-1].detected)
        self.assertEqual(self.messages[-1].header.stamp.to_sec(), measured_at)
        self.now += .5
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.assertFalse(self.messages[-1].detected)
        self.assertEqual(self.messages[-1].header.stamp.to_sec(), measured_at)
        for offset in (-1, .1, None, 0):
            self.observe(yellow)
            frame = self.frame(yellow, stamp=0 if offset is None else self.now + offset)
            self.node.image_callback(frame)
            self.node.step()
            self.assertFalse(self.messages[-1].camera_online)
            self.assertFalse(self.messages[-1].detected)
        frame = self.frame(yellow)
        frame.data = b'invalid jpeg'
        self.now += .1
        self.node.image_callback(frame)
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.now += .1
        self.node.image_callback(self.frame(yellow))
        self.now += .6
        self.node.step()
        self.assertFalse(self.messages[-1].camera_online)
        self.now += .1
        self.node.image_callback(self.frame(yellow))
        latest = self.frame()
        self.node.image_callback(latest)
        self.node.step()
        self.assertTrue(self.messages[-1].camera_online)
        self.assertFalse(self.messages[-1].detected)
        self.assertIsNone(self.node.pending)

    def test_invalid_thresholds_fail_at_startup(self):
        for name, value in [('~hsv_lower', [40, 100, 80]), ('~rate', float('nan')),
                            ('~hsv_upper', [180, 255, 255]), ('~min_area_fraction', 0)]:
            self.params.clear()
            self.params[name] = value
            if name == '~hsv_lower':
                self.params['~hsv_upper'] = [20, 255, 255]
            with self.assertRaises(ValueError):
                self.node_class()


if __name__ == '__main__':
    unittest.main()
