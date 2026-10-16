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
        self.ros.wait_for_message = lambda *args, **kwargs: NS(mode='DISARMED')
        sys.modules['pidrone_pkg.msg'].Battery = NS
        sys.modules['geometry_msgs.msg'].Pose = lambda: Pose().pose.pose
        sys.modules['geometry_msgs.msg'].Twist = twist
        sys.modules['std_msgs.msg'].Bool = lambda value: NS(data=value)
        self.nodes = ModuleType('rosnode')
        self.nodes.get_node_names = mock.Mock(return_value=[])
        self.nodes.rosnode_ping = lambda *args, **kwargs: True
        patch = mock.patch.dict(sys.modules, {'rosnode': self.nodes})
        patch.start()
        self.addCleanup(patch.stop)
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'drone_controller.py')
        cls = runpy.run_path(script)['Drone']
        self.globals = cls.__init__.__globals__
        self.globals['time'] = NS(time=lambda: self.fusion.now, sleep=lambda _: None)
        self.drone = cls()
        self.drone.mode = 'ARMED'
        self.drone.mode_at = self.drone.height_at = self.drone.flow_at = self.fusion.now
        self.drone.flow_valid = True
        self.drone._fused_received(copy.deepcopy(self.fusion.messages['pidrone/localization/fused'][-1]))

    def test_hold_uses_fused_position_and_heading_instead_of_integrated_flow(self):
        drone = self.drone
        drone.lock_xy()
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertFalse(self.fusion.messages['/pidrone/position_control'][-1].data)
        # Even wildly different flow-integrated coordinates cannot move the hold anchor.
        drone.right_m, drone.forward_m = 100, -200
        for yaw, expected in [(0, (-.2, 0)), (math.pi / 2, (0, .2))]:
            pose = copy.deepcopy(drone.fused_pose)
            pose.pose.pose.position.x = 2
            q = pose.pose.pose.orientation
            q.x, q.y, q.z, q.w = quaternion(yaw=yaw)
            drone._fused_received(pose)
            command = drone._xy_hold_command()
            self.assertAlmostEqual(command.linear.x, expected[0])
            self.assertAlmostEqual(command.linear.y, expected[1])
            self.assertAlmostEqual(math.hypot(command.linear.x, command.linear.y), .2)
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
        self.assertAlmostEqual(controller.desired_velocity.x, -.1)
        self.assertFalse(controller.position_control)

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

    def test_fusion_gap_holds_local_position_through_altitude_changes_then_resumes(self):
        drone = self.drone
        drone.lock_xy()
        controller = self.fusion.controller()
        modes = self.fusion.messages['/pidrone/position_control']
        controller.position_control_callback(modes[-1])
        # A nonzero old error must not become an indefinite movement command.
        pose = copy.deepcopy(drone.fused_pose)
        pose.pose.pose.position.x += .2
        drone._fused_received(pose)
        self.assertAlmostEqual(drone._xy_hold_command().linear.x, -.1)
        self.fusion.ros_now += 1
        controller.local_ready_callback(NS(data=True))
        command = drone._xy_hold_command()
        self.assertEqual((command.linear.x, command.linear.y), (0, 0))
        self.assertTrue(modes[-1].data)
        controller.desired_twist_callback(command)
        controller.position_control_callback(modes[-1])
        held_xy = (controller.desired_position.x, controller.desired_position.y)
        mode_count = len(modes)

        # Local odometry remains live and detects drift during the UWB gap.
        state = copy.deepcopy(self.fusion.messages['pidrone/state'][-1])
        state.header.stamp = Stamp(self.fusion.ros_now)
        state.pose_with_covariance.pose.position.x += .05
        controller.current_state_callback(state)
        for sample in (pose, None, pose):
            drone._fused_received(sample)
            command = drone._xy_hold_command()
            controller.desired_twist_callback(command)
            self.assertEqual(len(modes), mode_count, 'do not recapture the local hold point')
        drone._set_altitude(.15)
        controller.desired_pose_callback(self.fusion.messages['/pidrone/desired/pose'][-1])
        controller.calc_error()
        self.assertTrue(controller.position_control)
        self.assertEqual((controller.desired_position.x, controller.desired_position.y), held_xy)
        self.assertAlmostEqual(controller.desired_position.z, .15)
        self.assertGreater(controller.position_error.planar_magnitude(), 0)
        self.assertEqual(drone.xy_anchor, (1, 2))

        pose.header.stamp = Stamp(self.fusion.ros_now)
        drone._fused_received(pose)
        command = drone._xy_hold_command()
        controller.position_control_callback(modes[-1])
        controller.desired_twist_callback(command)
        self.assertAlmostEqual(command.linear.x, -.1)
        self.assertFalse(controller.position_control)
        self.assertEqual(drone.xy_anchor, (1, 2))

    def test_initial_lock_requires_fresh_fusion(self):
        drone = self.drone
        self.fusion.ros_now += 1
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
            drone.mode_at = drone.height_at = drone.flow_at = self.fusion.now
            drone.height = max(.05, drone.height - .02)

        self.ros.sleep = tick
        drone.land(hold_xy=True)
        self.assertEqual(drone.mode, 'DISARMED')
        self.assertIsNone(drone.xy_anchor)
        self.assertEqual([msg.data for msg in self.fusion.messages['/pidrone/position_control']],
                         [False, True, False])

    def test_startup_reuses_nodes_and_launches_only_missing_uwb_and_fusion(self):
        self.drone.mode = 'DISARMED'
        running = ['/raspicam_node', '/vl53l1x', '/optical_flow_node',
                   '/pid_controller', '/flight_controller_node', '/apriltag_detection']
        self.nodes.get_node_names.side_effect = lambda: list(running)
        commands = []

        def launch(command, **kwargs):
            commands.append(command)
            name = '/uwb_localization' if any('uwb_localization.launch' in part for part in command) else '/fusion_localization'
            running.append(name)
            return NS(poll=lambda: None)

        log = mock.MagicMock()
        log.__enter__.return_value = NS(name='mock-routine.log')
        with mock.patch.object(self.globals['subprocess'], 'Popen', side_effect=launch), \
                mock.patch.object(self.globals['tempfile'], 'NamedTemporaryFile', return_value=log), \
                mock.patch.dict(os.environ, {'PALS_UWB_PORT': '/dev/test-tag'}):
            self.drone._start_missing_nodes()
            self.drone._start_missing_nodes()
        self.assertEqual(len(commands), 2)
        self.assertIn('port:=/dev/test-tag', commands[0])
        self.assertFalse(any('state_estimator' in part for command in commands for part in command))

    def test_startup_rejects_a_competing_legacy_state_publisher(self):
        self.drone.mode = 'DISARMED'
        self.nodes.get_node_names.return_value = ['/state_estimator', '/fusion_localization']
        with mock.patch.object(self.globals['subprocess'], 'Popen') as launch:
            with self.assertRaisesRegex(RuntimeError, 'legacy /state_estimator'):
                self.drone._start_missing_nodes()
            launch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
