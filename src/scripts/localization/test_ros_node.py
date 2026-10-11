"""Exercise real node callbacks/publication using ROS message/transport doubles."""
import copy
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest import mock

from localization.uwb_locator import Position, Anchor, AnchorSample


class Stamp(object):
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class Pose(object):
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.pose = SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0, y=0, z=0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=0)), covariance=[0] * 36)


class Diagnostics(object):
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.status = []


class Layout(object):
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.anchor_ids = []
        self.positions = []


class Status(object):
    OK, WARN, ERROR = 0, 1, 2


class FakeLocator(object):
    def __init__(self, **kwargs):
        self.port = kwargs['port']
        self.running = False
        self.position = None
        self.error = None
        self.starts = 0
        self.anchor_sample = None
        self.stale_after = kwargs['stale_after']

    def start(self):
        self.starts += 1
        self.running = True
        self.error = None

    def stop(self):
        self.running = False
        self.position = None


class NodeTest(unittest.TestCase):
    def setUp(self):
        self.now, self.ros_now = 10, 100
        self.params = {'~port': 'fake'}
        self.messages = {}
        self.ros = ModuleType('rospy')
        self.ros.get_param = lambda name, default: self.params.get(name, default)
        self.ros.get_name = lambda: '/uwb_localization'
        self.ros.Time = SimpleNamespace(now=lambda: Stamp(self.ros_now), from_sec=Stamp)
        self.ros.Subscriber = mock.Mock()
        self.ros.on_shutdown = mock.Mock()
        self.ros.logwarn_throttle = mock.Mock()

        def publisher(topic, *args, **kwargs):
            self.messages[topic] = []
            return SimpleNamespace(publish=lambda msg: self.messages[topic].append(copy.deepcopy(msg)))

        self.ros.Publisher = publisher
        modules = {'rospy': self.ros}
        for package, messages in [
                ('diagnostic_msgs', {'DiagnosticArray': Diagnostics, 'DiagnosticStatus': Status,
                                     'KeyValue': SimpleNamespace}),
                ('geometry_msgs', {'PoseWithCovarianceStamped': Pose, 'Point': SimpleNamespace}),
                ('pidrone_pkg', {'AnchorLayout': Layout}),
                ('sensor_msgs', {'Imu': object, 'Range': object})]:
            modules[package] = ModuleType(package)
            modules[package + '.msg'] = ModuleType(package + '.msg')
            for name, value in messages.items():
                setattr(modules[package + '.msg'], name, value)
        patched = mock.patch.dict(sys.modules, modules)
        patched.start()
        self.addCleanup(patched.stop)
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'uwb_localization_node')
        node_globals = runpy.run_path(script)
        self.node_class = node_globals['UWBLocalizationNode']
        # Class methods retain the run_path globals dictionary.
        self.node_class.step.__globals__['monotonic'] = lambda: self.now
        self.node_class.range_callback.__globals__['monotonic'] = lambda: self.now
        self.node_class.__init__.__globals__['UWBLocator'] = FakeLocator
        self.node = self.node_class()
        self.node.locator.running = True

    @property
    def poses(self):
        return self.messages['pidrone/localization/pose']

    @property
    def health(self):
        return self.messages['pidrone/localization/status'][-1].status[0]

    def sensors(self, distance=0.5, range_stamp=None):
        stamp = self.ros_now if range_stamp is None else range_stamp
        self.node.range_callback(SimpleNamespace(
            range=distance, min_range=0.05, max_range=4, header=SimpleNamespace(stamp=Stamp(stamp))))
        self.node.imu_callback(SimpleNamespace(
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1), orientation_covariance=[0] * 9,
            header=SimpleNamespace(stamp=Stamp(self.ros_now))))
        self.node.locator.position = Position(1, 2, 999, 90, self.now)

    def test_publication_uses_tof_covariance_and_original_stamp_only_once(self):
        self.sensors(range_stamp=99.95)
        self.node.step()
        msg = self.poses[-1]
        self.assertEqual((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), (1, 2, 0.5))
        self.assertEqual(msg.header.stamp.to_sec(), 99.95)
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertEqual(msg.pose.pose.orientation.w, 1)
        self.assertAlmostEqual(msg.pose.covariance[0], 0.15 ** 2)
        self.assertAlmostEqual(msg.pose.covariance[14], 0.03 ** 2)
        self.assertEqual(msg.pose.covariance[35], 1e6)
        self.node.step()
        self.assertEqual(len(self.poses), 1)
        self.assertEqual(self.health.level, 0)
        self.assertEqual(self.health.values[1].value, 'ToF')
        xy = self.messages['pidrone/localization/uwb'][-1]
        self.assertEqual(xy.header.stamp.to_sec(), 100)
        self.assertEqual(xy.pose.covariance[14], 1e6)
        self.assertEqual(len(self.messages['pidrone/localization/uwb']), 1)

    def test_missing_invalid_or_stale_tof_suppresses_pose_and_publishes_health(self):
        self.node.step()
        self.assertEqual(self.poses, [])
        self.assertNotEqual(self.health.level, 0)
        self.sensors(distance=8)
        self.node.step()
        self.assertEqual(self.poses, [])
        self.assertEqual(len(self.messages['pidrone/localization/uwb']), 1)
        self.sensors()
        self.now, self.ros_now = 10.6, 100.6
        self.node.locator.position = Position(1, 2, 999, 90, self.now)
        self.node.step()
        self.assertEqual(self.poses, [])
        self.assertIn('ToF', self.health.message)

    def test_disconnect_attempts_reconnect_without_publishing_old_fix(self):
        self.sensors()
        self.node.locator.position = None
        self.node.locator.running = False
        self.node.step()
        self.assertEqual(self.node.locator.starts, 1)
        self.assertEqual(self.poses, [])
        self.node.locator.running = False
        self.node.step()
        self.assertEqual(self.node.locator.starts, 1)
        self.now += 6
        self.ros_now += 6
        self.node.step()
        self.assertEqual(self.node.locator.starts, 2)

    def test_unconfigured_node_never_opens_flight_controller_port(self):
        self.node.locator.port = ''
        self.node.locator.running = False
        self.node.step()
        self.assertEqual(self.node.locator.starts, 0)
        self.assertEqual(self.health.level, 2)
        self.assertIn('Set port', self.health.message)

    def test_ros_clock_reset_invalidates_cached_sensors(self):
        self.sensors()
        self.node.step()
        self.ros_now = 50
        self.node.step()
        self.assertEqual(len(self.poses), 1)
        self.assertNotEqual(self.health.level, 0)
        self.assertIsNone(self.node.combiner.range)
        self.sensors()
        self.now += 0.01
        self.ros_now += 0.01
        self.node.step()
        self.assertEqual(len(self.poses), 2)

    def test_anchor_reports_publish_without_tof_and_expire_without_renewing_old_stamps(self):
        self.node.locator.anchor_sample = AnchorSample((Anchor('0CA8', 0, 6, 2.25, 6.5),), 9.9)
        reports = self.messages['pidrone/localization/anchors']
        self.ros_now = 0
        self.node.step()
        self.assertEqual(reports, [])
        self.ros_now = 100
        self.node.step()
        self.assertEqual(reports[-1].header.frame_id, 'map')
        self.assertEqual(reports[-1].anchor_ids, ['0CA8'])
        self.assertAlmostEqual(reports[-1].header.stamp.to_sec(), 99.9)
        self.assertEqual(reports[-1].positions[0].z, 2.25)
        self.assertEqual(self.poses, [])
        self.node.step()
        self.assertEqual(len(reports), 1)
        self.now, self.ros_now = 10.6, 100.6
        self.node.step()
        self.assertEqual(reports[-1].anchor_ids, [])
        self.assertEqual(len(reports), 2)
        self.node.step()
        self.assertEqual(len(reports), 2)


if __name__ == '__main__':
    unittest.main()
