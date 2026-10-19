"""Routine hold integration with the real fusion/PID code and mocked ROS transport."""
import copy
import math
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

from localization import test_fusion as fusion_helpers
from localization.test_fusion import quaternion, twist
from localization.test_ros_node import Pose, Stamp


class RoutineControlTest(unittest.TestCase):
    def setUp(self):
        self.fusion = fusion_helpers.FusionNodeTest()
        self.fusion.setUp()
        self.addCleanup(self.fusion.doCleanups)
        self.ros = self.fusion.ros
        self.ros.Timer = mock.Mock()
        self.ros.Duration = lambda seconds: seconds
        self.ros.is_shutdown = lambda: False
        sys.modules['pidrone_pkg.msg'].Battery = NS
        sys.modules['geometry_msgs.msg'].Pose = lambda: Pose().pose.pose
        sys.modules['geometry_msgs.msg'].Twist = twist
        sys.modules['std_msgs.msg'].Bool = lambda value: NS(data=value)
        self.nodes = ModuleType('rosnode')
        self.nodes.get_node_names = mock.Mock(return_value=[])
        patch = mock.patch.dict(sys.modules, {'rosnode': self.nodes})
        patch.start()
        self.addCleanup(patch.stop)
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'drone_controller.py')
        cls = runpy.run_path(script)['Drone']
        self.globals = cls.__init__.__globals__
        self.globals['time'] = NS(time=lambda: self.fusion.now, sleep=lambda _: None)
        self.drone = cls()
        self.drone.mode = 'ARMED'
        self.drone.mode_at = self.drone.height_at = self.fusion.now
        self.drone._fused_received(copy.deepcopy(self.fusion.messages['pidrone/localization/fused'][-1]))

    def test_hold_uses_fused_position_and_heading(self):
        drone = self.drone
        drone.lock_xy()
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertFalse(self.fusion.messages['/pidrone/position_control'][-1].data)
        for yaw, expected in [(0, (-.1, 0)), (math.pi / 2, (0, .1))]:
            pose = copy.deepcopy(drone.fused_pose)
            pose.pose.pose.position.x = 2
            q = pose.pose.pose.orientation
            q.x, q.y, q.z, q.w = quaternion(yaw=yaw)
            drone._fused_received(pose)
            command = drone._xy_hold_command()
            self.assertAlmostEqual(command.linear.x, expected[0])
            self.assertAlmostEqual(command.linear.y, expected[1])
            self.assertAlmostEqual(math.hypot(command.linear.x, command.linear.y), .1)
            self.assertEqual(command.linear.z, 0)
        drone.unlock_xy()
        self.assertIsNone(drone.xy_anchor)
        self.assertEqual(drone._xy_hold_command().linear.x, 0)

    def test_altitude_update_keeps_fused_anchor_and_existing_pid_feedback(self):
        drone = self.drone
        drone.lock_xy()
        controller = self.fusion.controller()
        controller.position_control_callback(self.fusion.messages['/pidrone/position_control'][-1])
        pose = copy.deepcopy(drone.fused_pose)
        pose.pose.pose.position.x += .2
        pose.pose.pose.position.z = 9  # Room Z does not replace the PID's ToF height.
        drone._fused_received(pose)
        command = drone._xy_hold_command()
        controller.desired_twist_callback(command)
        drone._set_altitude(.5)
        controller.desired_pose_callback(self.fusion.messages['/pidrone/desired/pose'][-1])
        controller.desired_twist_callback(drone._xy_hold_command())
        controller.calc_error()
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertEqual(controller.desired_position.z, .5)
        self.assertEqual(controller.current_position.z, .5)
        self.assertEqual(controller.pid_error.z, 0)
        self.assertAlmostEqual(controller.desired_velocity.x, -.04)
        self.assertFalse(controller.position_control)

    def test_relative_move_tracks_uwb_target_through_heading_change(self):
        drone = self.drone
        drone.mode = 'FLYING'
        drone.height = .5
        drone.battery_at = self.fusion.now
        pose = copy.deepcopy(drone.fused_pose)
        q = pose.pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion(yaw=math.pi/2)
        drone._fused_received(pose)
        commands = []

        def tick(seconds):
            command = self.fusion.messages['/pidrone/desired/twist'][-1]
            commands.append((command.linear.x, command.linear.y))
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
            drone.mode_at = drone.height_at = self.fusion.now
            pose.header.stamp = Stamp(self.fusion.ros_now)
            # The room target stays at (1, 2.2) even after the nose turns.
            pose.pose.pose.position.y += .1
            q.x, q.y, q.z, q.w = quaternion(yaw=0)
            drone._fused_received(pose)

        self.ros.sleep = tick
        drone.move(x=.2)
        self.assertEqual(len(commands), 2)
        self.assertGreater(commands[0][0], 0)
        self.assertAlmostEqual(commands[0][1], 0)
        self.assertAlmostEqual(commands[1][0], 0)
        self.assertGreater(commands[1][1], 0)
        final = self.fusion.messages['/pidrone/desired/twist'][-1]
        self.assertEqual((final.linear.x, final.linear.y), (0, 0))
        self.fusion.ros_now += 1
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.move(y=.2)

    def test_hold_rejects_invalid_or_different_frame_pose(self):
        self.drone.lock_xy()
        for change in ('invalid', 'frame'):
            pose = copy.deepcopy(self.fusion.messages['pidrone/localization/fused'][-1])
            if change == 'invalid':
                pose.pose.pose.position.x = float('nan')
            else:
                pose.header.frame_id = 'different_room'
            self.drone._fused_received(pose)
            with self.assertRaises(RuntimeError):
                self.drone._xy_hold_command()
            self.assertEqual(self.drone.xy_anchor, (1, 2))

    def test_fusion_gap_keeps_anchor_and_mode_through_altitude_changes_then_resumes(self):
        drone = self.drone
        drone.lock_xy()
        controller = self.fusion.controller()
        modes = self.fusion.messages['/pidrone/position_control']
        controller.position_control_callback(modes[-1])
        # A nonzero old error must not become an indefinite movement command.
        pose = copy.deepcopy(drone.fused_pose)
        pose.pose.pose.position.x += .2
        drone._fused_received(pose)
        self.assertAlmostEqual(drone._xy_hold_command().linear.x, -.04)
        self.fusion.ros_now += .7
        self.assertAlmostEqual(drone._xy_hold_command().linear.x, -.04)
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.fusion.ros_now += .4
        controller.local_ready_callback(NS(data=False))
        command = drone._xy_hold_command()
        self.assertEqual((command.linear.x, command.linear.y), (0, 0))
        self.assertFalse(modes[-1].data)
        controller.desired_twist_callback(command)
        controller.position_control_callback(modes[-1])
        mode_count = len(modes)

        # Altitude state remains available after UWB expires.
        state = copy.deepcopy(self.fusion.messages['pidrone/state'][-1])
        state.header.stamp = Stamp(self.fusion.ros_now)
        state.pose_with_covariance.pose.position.x += .05
        controller.current_state_callback(state)
        for sample in (pose, None, pose):
            drone._fused_received(sample)
            command = drone._xy_hold_command()
            controller.desired_twist_callback(command)
            self.assertEqual(len(modes), mode_count, 'do not switch hold modes')
        drone._set_altitude(.15)
        controller.desired_pose_callback(self.fusion.messages['/pidrone/desired/pose'][-1])
        controller.calc_error()
        self.assertFalse(controller.position_control)
        self.assertAlmostEqual(controller.desired_position.z, .15)
        self.assertEqual(drone.xy_anchor, (1, 2))

        pose.header.stamp = Stamp(self.fusion.ros_now)
        drone._fused_received(pose)
        command = drone._xy_hold_command()
        controller.position_control_callback(modes[-1])
        controller.desired_twist_callback(command)
        self.assertAlmostEqual(command.linear.x, -.04)
        self.assertFalse(controller.position_control)
        self.assertEqual(drone.xy_anchor, (1, 2))

    def test_initial_lock_requires_fresh_fusion(self):
        drone = self.drone
        self.fusion.ros_now += .7  # Grace is for an existing hold, not a new lock.
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.lock_xy()
        drone._fused_received(None)
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.lock_xy()

    def test_landing_with_stale_fusion_keeps_hold_until_disarm(self):
        drone = self.drone
        drone.lock_xy()
        drone.mode = 'FLYING'
        drone.height, drone.ground_height = .25, .05
        self.fusion.ros_now += 1
        self.globals['Mode'] = lambda name: NS(mode=name)
        drone.mode_pub.publish = drone._mode_received

        def tick(seconds):
            self.assertEqual(drone.xy_anchor, (1, 2))
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
            drone.mode_at = drone.height_at = self.fusion.now
            drone.height = max(.05, drone.height - .02)

        self.ros.sleep = tick
        drone.land(hold_xy=True)
        self.assertEqual(drone.mode, 'DISARMED')
        self.assertIsNone(drone.xy_anchor)
        self.assertEqual([msg.data for msg in self.fusion.messages['/pidrone/position_control']],
                         [False])

    def advance_until(self, release_at=None):
        started = self.fusion.now
        self.drone.voltage = 16.
        self.drone.battery_at = self.drone.mode_at = self.drone.height_at = started

        def sleep(seconds):
            self.fusion.now += seconds
            self.drone.battery_at = self.drone.mode_at = self.drone.height_at = self.fusion.now
            if release_at is not None and self.fusion.now - started >= release_at:
                self.drone._start_received(None)
        self.globals['time'] = NS(time=lambda: self.fusion.now, sleep=sleep)

    def test_synchronized_start_posts_ready_and_ignores_an_earlier_start(self):
        drone = self.drone
        drone.mode = 'DISARMED'
        drone._start_received(None)  # From a previous run: must not release this one.
        self.advance_until(release_at=.5)
        drone.wait_for_start()
        ready = self.fusion.messages['/pidrone/routine/ready']
        self.assertTrue(2 <= len(ready) <= 4)  # 5 Hz for 0.5 s

    def test_no_start_or_mode_change_aborts_before_arming(self):
        drone = self.drone
        drone.mode = 'DISARMED'
        self.advance_until()
        with self.assertRaisesRegex(RuntimeError, 'No synchronized start within 30 s'):
            drone.wait_for_start()
        drone.mode = 'ARMED'
        with self.assertRaisesRegex(RuntimeError, 'Mode changed to ARMED'):
            drone.wait_for_start()
        drone.mode, drone.height, drone.ground_height = 'DISARMED', .05, .05
        drone.mode_pub.publish = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'No synchronized start'):
            drone.takeoff()
        drone.mode_pub.publish.assert_not_called()
        drone.height_at = self.fusion.now - 2
        with self.assertRaisesRegex(RuntimeError, 'Downward range is missing/stale'):
            drone.wait_for_start()
        drone.mode_pub.publish.assert_not_called()

    def test_ready_rejects_a_competing_legacy_state_publisher(self):
        self.drone.mode = 'DISARMED'
        self.nodes.get_node_names.return_value = ['/state_estimator', '/fusion_localization']
        with self.assertRaisesRegex(RuntimeError, 'legacy /state_estimator'):
            self.drone.ready()


if __name__ == '__main__':
    unittest.main()
