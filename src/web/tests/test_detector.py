"""Exercise the onboard node with real encoded tags and mocked ROS transport."""
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


class TagDetection:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 100.0))
        self.camera_online = False
        self.tag_ids = []
        self.zones = []


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.messages = []
        self.ros = ModuleType('rospy')
        self.ros.get_param = lambda name, default: default
        self.ros.Time = SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: self.now))
        self.ros.Publisher = Mock(return_value=SimpleNamespace(
            publish=lambda msg: self.messages.append(copy.deepcopy(msg))))
        self.ros.Subscriber = Mock()
        self.ros.loginfo = Mock()
        self.ros.logwarn_throttle = Mock()
        modules = {'rospy': self.ros}
        for package, message in [('sensor_msgs', 'CompressedImage'), ('pidrone_pkg', 'TagDetection')]:
            modules[package] = ModuleType(package)
            modules[package + '.msg'] = ModuleType(package + '.msg')
            setattr(modules[package + '.msg'], message, TagDetection if message == 'TagDetection' else object)
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


if __name__ == '__main__':
    unittest.main()
