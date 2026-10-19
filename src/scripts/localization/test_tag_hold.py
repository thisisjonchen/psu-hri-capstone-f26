"""Tag hold with the real Drone/geometry code and mocked ROS transport."""
import copy
import os
import runpy
import sys
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from localization.test_ros_node import Stamp
from localization import test_routine_control as routine_helpers

PARAMS = {'/apriltag_mapping/camera_pitch_down_deg': 90.0,
          '/apriltag_mapping/camera_azimuth_deg': 0.0,
          '/apriltag_mapping/camera_image_roll_deg': 0.0,
          '/apriltag_mapping/camera_offset_body': [0, 0, 0]}
SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'tag_hold.py')


class TagHoldTest(unittest.TestCase):
    def setUp(self):
        self.routine = routine_helpers.RoutineControlTest()
        self.routine.setUp()
        self.addCleanup(self.routine.doCleanups)
        self.fusion, self.ros, self.drone = self.routine.fusion, self.routine.ros, self.routine.drone
        self.params = dict(PARAMS)
        # Like rospy: a missing parameter without a default raises KeyError.
        self.ros.get_param = lambda name, *default: (
            self.params[name] if name in self.params or not default else default[0])
        sys.modules['pidrone_pkg.msg'].TagDetection = NS
        sys.modules['sensor_msgs.msg'].CameraInfo = NS
        self.drone.height = .5
        self.drone.hover = mock.Mock()
        self.ros.loginfo, self.ros.logwarn = mock.Mock(), mock.Mock()
        self.hold = runpy.run_path(SCRIPT)['TagHold'](self.drone)
        # 100 px focal length: a 50 px shift at 0.5 m is 0.25 m on the floor.
        self.hold._camera_received(NS(K=[100, 0, 120, 0, 100, 160, 0, 0, 1], D=[0] * 5,
                                      width=240, height=320, distortion_model='plumb_bob'))

    def see(self, u=120, v=160, tag_ids=(3,)):
        self.hold._detection_received(NS(
            header=NS(stamp=Stamp(self.fusion.ros_now)), camera_online=True,
            tag_ids=list(tag_ids), center_x=[u] * len(tag_ids), center_y=[v] * len(tag_ids),
            image_width=240, image_height=320))

    def advance(self, seconds, x=1):
        self.fusion.ros_now += seconds
        pose = copy.deepcopy(self.drone.fused_pose)
        pose.header.stamp = Stamp(self.fusion.ros_now)
        pose.pose.pose.position.x = x
        self.drone._fused_received(pose)

    def test_corrects_toward_locked_offset_within_speed_limit(self):
        self.see()
        self.hold.acquire()
        self.assertEqual(self.hold.tag_id, 3)
        for u, v, expected in [(170, 160, (.05, 0)), (120, 110, (0, .05)), (239, 160, (.1, 0))]:
            self.advance(.1)
            self.see(u, v)
            command = self.hold.command()
            self.assertAlmostEqual(command.linear.x, expected[0])
            self.assertAlmostEqual(command.linear.y, expected[1])
        self.advance(.1)
        self.see()
        self.hold.detection.center_x = []
        self.assertIsNone(self.hold._new_detection())
        # A ROS clock rollback must not keep a cached tag correction fresh.
        self.fusion.ros_now -= 1
        self.assertEqual(self.hold.command().linear.x, 0)
        self.assertTrue(self.hold.lost)

    def test_lost_tag_preserves_acquired_hold_target_then_resumes_tag_offset(self):
        self.see()
        self.hold.acquire()
        self.advance(.3)
        self.hold.command()  # Same frame again: not a new measurement.
        self.advance(.3, x=1.4)
        self.hold.command()
        self.assertTrue(self.hold.lost)
        self.assertEqual(self.drone.xy_anchor, (1, 2))
        self.advance(.1, x=1.5)
        self.assertAlmostEqual(self.hold.command().linear.x, -.1)
        self.assertEqual(self.drone.xy_anchor, (1, 2))
        self.see(170, 160, tag_ids=(1, 3))
        self.assertAlmostEqual(self.hold.command().linear.x, .05)
        self.advance(.1)
        self.see(170, 160)
        self.hold.command()
        self.assertFalse(self.hold.lost)
        # One message per switch, none while either hold continues.
        switches = [call[0][0] for log in (self.ros.loginfo, self.ros.logwarn)
                    for call in log.call_args_list if call[0][0].startswith('Stabilization')]
        self.assertEqual(len(switches), 3)  # Lock, loss, reacquire.

    def test_no_tag_or_missing_calibration_fails(self):
        with self.assertRaises(RuntimeError):
            self.hold.acquire(timeout=0)
        del self.params['/apriltag_mapping/camera_offset_body']
        with self.assertRaises(KeyError):
            runpy.run_path(SCRIPT)['TagHold'](self.drone)


if __name__ == '__main__':
    unittest.main()
