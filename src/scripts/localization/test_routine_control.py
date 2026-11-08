"""Routine hold integration with the real fusion/PID code and mocked ROS transport."""
import copy
import contextlib
import io
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
        self.ros.logerr = mock.Mock()
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
        # Hold and relative moves use local odometry; reuse the fused numbers in its frame.
        local = copy.deepcopy(self.fusion.messages['pidrone/localization/fused'][-1])
        local.header.frame_id = 'odom'
        self.drone._local_received(local)

    def test_hold_uses_local_position_and_heading(self):
        drone = self.drone
        drone.lock_xy()
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertFalse(self.fusion.messages['/pidrone/position_control'][-1].data)
        for yaw, expected in [(0, (-.1, 0)), (math.pi / 2, (0, .1))]:
            pose = copy.deepcopy(drone.local_pose)
            pose.pose.pose.position.x = 2
            q = pose.pose.pose.orientation
            q.x, q.y, q.z, q.w = quaternion(yaw=yaw)
            drone._local_received(pose)
            command = drone._xy_hold_command()
            self.assertAlmostEqual(command.linear.x, expected[0])
            self.assertAlmostEqual(command.linear.y, expected[1])
            self.assertAlmostEqual(math.hypot(command.linear.x, command.linear.y), .1)
            self.assertEqual(command.linear.z, 0)
        drone.unlock_xy()
        self.assertIsNone(drone.xy_anchor)
        self.assertEqual(drone._xy_hold_command().linear.x, 0)

    def test_altitude_update_keeps_local_anchor_and_existing_pid_feedback(self):
        drone = self.drone
        drone.lock_xy()
        controller = self.fusion.controller()
        controller.position_control_callback(self.fusion.messages['/pidrone/position_control'][-1])
        pose = copy.deepcopy(drone.local_pose)
        pose.pose.pose.position.x += .2
        pose.pose.pose.position.z = 9  # Room Z does not replace the PID's ToF height.
        drone._local_received(pose)
        command = drone._xy_hold_command()
        controller.desired_twist_callback(command)
        drone._set_altitude(.5)
        controller.desired_pose_callback(self.fusion.messages['/pidrone/desired/pose'][-1])
        controller.desired_twist_callback(drone._xy_hold_command())
        controller.calc_error()
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertEqual(controller.desired_position.z, .5)
        self.assertEqual(controller.current_position.z, .5)
        self.assertAlmostEqual(controller.desired_velocity.x, -.06)
        self.assertFalse(controller.position_control)

    def test_relative_move_tracks_local_target_through_heading_change(self):
        drone = self.drone
        drone.mode = 'FLYING'
        drone.height = .5
        drone.battery_at = self.fusion.now
        pose = copy.deepcopy(drone.local_pose)
        q = pose.pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion(yaw=math.pi/2)
        drone._local_received(pose)
        commands = []
        positions = iter([(1.061, 2), (1, 2.9), (1, 3)])

        def tick(seconds):
            command = self.fusion.messages['/pidrone/desired/twist'][-1]
            commands.append((command.linear.x, command.linear.y))
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
            drone.mode_at = drone.height_at = self.fusion.now
            pose.header.stamp = Stamp(self.fusion.ros_now)
            # Keep the room target after turning, then approach with a sideways error.
            pose.pose.pose.position.x, pose.pose.pose.position.y = next(positions)
            q.x, q.y, q.z, q.w = quaternion(yaw=0)
            drone._local_received(pose)

        self.ros.sleep = tick
        drone.move(x=1, speed_limit=.06)
        self.assertEqual(len(commands), 3)
        self.assertGreater(commands[0][0], 0)
        self.assertAlmostEqual(commands[0][1], 0)
        self.assertLess(commands[1][0], 0)
        self.assertGreater(commands[1][1], .059)
        self.assertAlmostEqual(commands[1][0] / commands[1][1], -.061)
        self.assertAlmostEqual(commands[2][0], 0)
        self.assertGreater(commands[2][1], 0)
        self.assertLess(commands[2][1], commands[1][1])
        self.assertTrue(all(math.hypot(*command) <= .06 for command in commands))
        final = self.fusion.messages['/pidrone/desired/twist'][-1]
        self.assertEqual((final.linear.x, final.linear.y), (0, 0))
        commands[:] = []
        pose.pose.pose.position.x, pose.pose.pose.position.y = 1, 2
        q.x, q.y, q.z, q.w = quaternion(yaw=math.pi/2)
        drone._local_received(pose)
        positions = iter([(1.061, 2), (1, 2.9), (1, 3)])
        drone.move(x=1, speed_limit=None)
        self.assertAlmostEqual(commands[0][0], .5)
        self.assertAlmostEqual(commands[1][0] / commands[1][1], -.061)
        self.assertAlmostEqual(commands[2][1], .05)
        final = self.fusion.messages['/pidrone/desired/twist'][-1]
        self.assertEqual((final.linear.x, final.linear.y), (0, 0))
        self.fusion.ros_now += 1
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.move(y=.2)

        def stationary_tick(seconds):
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
            drone.mode_at = drone.height_at = drone.battery_at = self.fusion.now
            pose.header.stamp = Stamp(self.fusion.ros_now)
            drone._local_received(pose)
        self.ros.sleep = stationary_tick
        stationary_tick(0)
        with self.assertRaisesRegex(RuntimeError, 'No horizontal movement detected: 0.000 m'):
            drone.move_forward(.25, speed_limit=None)
        self.ros.logerr.assert_called_once()
        self.assertIn('No horizontal movement detected', str(self.ros.logerr.call_args.args[1]))
        final = self.fusion.messages['/pidrone/desired/twist'][-1]
        self.assertEqual((final.linear.x, final.linear.y), (0, 0))

    def test_hold_and_relative_moves_work_without_uwb_but_waypoints_require_it(self):
        drone = self.drone
        drone._fused_received(None)  # No UWB anchors at all.
        drone.lock_xy()
        self.assertEqual(drone.xy_frame, 'odom')
        pose = copy.deepcopy(drone.local_pose)
        pose.pose.pose.position.x += .2
        drone._local_received(pose)
        self.assertAlmostEqual(math.hypot(drone._xy_hold_command().linear.x,
                                          drone._xy_hold_command().linear.y), .06)
        drone.unlock_xy()
        drone.mode, drone.height, drone.battery_at = 'FLYING', .5, self.fusion.now
        with self.assertRaisesRegex(RuntimeError, 'Fused position is missing/stale'):
            drone.move_to(1, 2.2, 'map')

    def test_hold_rejects_invalid_or_different_frame_pose(self):
        self.drone.lock_xy()
        for change in ('invalid', 'frame'):
            pose = copy.deepcopy(self.drone.local_pose)
            if change == 'invalid':
                pose.pose.pose.position.x = float('nan')
            else:
                pose.header.frame_id = 'different_room'
            self.drone._local_received(pose)
            with self.assertRaises(RuntimeError):
                self.drone._xy_hold_command()
            self.assertEqual(self.drone.xy_anchor, (1, 2))

    def test_odometry_gap_keeps_anchor_and_mode_through_altitude_changes_then_resumes(self):
        drone = self.drone
        drone.lock_xy()
        controller = self.fusion.controller()
        modes = self.fusion.messages['/pidrone/position_control']
        controller.position_control_callback(modes[-1])
        # A nonzero old error must not become an indefinite movement command.
        pose = copy.deepcopy(drone.local_pose)
        pose.pose.pose.position.x += .2
        drone._local_received(pose)
        self.assertAlmostEqual(drone._xy_hold_command().linear.x, -.06)
        self.fusion.ros_now += .45  # Within the 0.5 s velocity coast limit.
        self.assertAlmostEqual(drone._xy_hold_command().linear.x, -.06)
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.fusion.ros_now += .1
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
            drone._local_received(sample)
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
        drone._local_received(pose)
        command = drone._xy_hold_command()
        controller.position_control_callback(modes[-1])
        controller.desired_twist_callback(command)
        self.assertAlmostEqual(command.linear.x, -.06)
        self.assertFalse(controller.position_control)
        self.assertEqual(drone.xy_anchor, (1, 2))

    def test_initial_lock_requires_fresh_local_position(self):
        drone = self.drone
        self.fusion.ros_now += .7  # Grace is for an existing hold, not a new lock.
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.lock_xy()
        drone._local_received(None)
        with self.assertRaisesRegex(RuntimeError, 'missing/stale'):
            drone.lock_xy()

    def test_landing_with_stale_local_position_keeps_hold_until_disarm(self):
        drone = self.drone
        drone.lock_xy()
        drone.mode = 'FLYING'
        drone.height, drone.ground_height = .25, .05
        self.fusion.ros_now += 1
        self.globals['Mode'] = lambda name: NS(mode=name)
        controller = self.fusion.controller()
        drone.landing_pub.publish = controller.landing_callback
        def mode_received(msg):
            drone._mode_received(msg)
            controller.current_mode_callback(msg)
        drone.mode_pub.publish = mode_received

        def tick(seconds):
            self.assertEqual(drone.xy_anchor, (1, 2))
            self.assertEqual(controller.altitude.landing_ground, .05)
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
            drone.mode_at = drone.height_at = self.fusion.now
            drone.height = max(.05, drone.height - .02)

        self.ros.sleep = tick
        drone.land(hold_xy=True)
        self.assertEqual(drone.mode, 'DISARMED')
        self.assertIsNone(drone.xy_anchor)
        self.assertIsNone(controller.altitude.landing_ground)
        self.assertEqual([msg.data for msg in self.fusion.messages['/pidrone/position_control']],
                         [False])

        controller.landing_callback(NS(data=.05))
        self.assertIsNone(controller.altitude.landing_ground, 'ignore landing requests while disarmed')
        controller.current_mode = 'FLYING'
        for ground in (float('nan'), -.1, .5):
            controller.landing_callback(NS(data=ground))
            self.assertIsNone(controller.altitude.landing_ground)
        controller.landing_callback(NS(data=.05))
        controller.desired_pose_callback(NS(position=NS(x=0, y=0, z=.2)))
        self.assertIsNone(controller.altitude.landing_ground, 'manual altitude input cancels descent')

        drone.mode = 'FLYING'
        drone.height = .25
        def stuck(seconds):
            self.fusion.now += seconds
            drone.mode_at = drone.height_at = self.fusion.now
        self.ros.sleep = stuck
        with self.assertRaisesRegex(RuntimeError, 'Landing was not confirmed'):
            drone.land()
        self.assertEqual(drone.mode, 'FLYING', 'a landing timeout must not disarm in the air')

    def test_finish_preserves_disarm_and_manual_landing_recovery(self):
        drone = self.drone
        with mock.patch.object(drone, 'land') as land, \
                mock.patch.object(drone, 'disarm') as disarm, \
                mock.patch.object(self.ros, 'sleep', create=True) as sleep:
            for mode in ('DISARMED', 'ARMED', None):
                drone.mode = mode
                drone.finish()
                disarm.assert_called_once_with()
                land.assert_not_called()
                disarm.reset_mock()

            drone.mode = 'FLYING'
            for hold_xy in (False, True):
                drone.finish(hold_xy=hold_xy)
                land.assert_called_once_with(hold_xy=hold_xy)
                land.reset_mock()
            disarm.assert_not_called()
            sleep.assert_not_called()

            land.side_effect = RuntimeError('range unavailable')

            def manual_recovery(seconds):
                disarm.assert_not_called()
                drone.heartbeat.shutdown.assert_not_called()
                # Stay alive through an intermediate mode until manual disarm.
                drone.mode = 'ARMED' if sleep.call_count == 1 else 'DISARMED'

            sleep.side_effect = manual_recovery
            drone.finish(hold_xy=True)
            land.assert_called_once_with(hold_xy=True)
            self.assertEqual(sleep.call_args_list, [mock.call(.5), mock.call(.5)])
            disarm.assert_not_called()
            drone.heartbeat.shutdown.assert_not_called()

    def test_heartbeat_stops_on_ros_shutdown_without_hiding_live_publish_failures(self):
        drone = self.drone
        self.ros.ROSException = type('ROSException', (Exception,), {})
        self.ros.on_shutdown.assert_any_call(drone.heartbeat.shutdown)
        callback = self.ros.Timer.call_args.args[1]
        with mock.patch.object(drone.heartbeat_pub, 'publish') as publish:
            callback(None)
            publish.assert_called_once()
            publish.reset_mock()
            self.ros.is_shutdown = lambda: True
            callback(None)
            publish.assert_not_called()
            self.ros.is_shutdown = lambda: False

            def close_during_publish(message):
                self.ros.is_shutdown = lambda: True
                raise self.ros.ROSException('publish() to a closed topic')

            publish.side_effect = close_during_publish
            callback(None)  # Shutdown raced the check; suppress only that error.
            self.ros.is_shutdown = lambda: False
            publish.side_effect = self.ros.ROSException('live transport failure')
            with self.assertRaisesRegex(self.ros.ROSException, 'live transport failure'):
                callback(None)
        drone.heartbeat.shutdown.assert_not_called()

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

    def test_takeoff_can_continue_after_one_second_without_changing_altitude_target(self):
        drone = self.drone
        drone.wait_for_start = mock.Mock()
        self.globals['Mode'] = lambda name: NS(mode=name)
        drone.mode_pub.publish = drone._mode_received
        for delay, height in ((1.0, None), (None, None), (None, .6)):
            with self.subTest(continue_after=delay, height=height):
                target = .25 if height is None else height
                drone.mode, drone.height, drone.ground_height = 'DISARMED', .05, .05
                drone.voltage = 16.
                drone.battery_at = drone.mode_at = drone.height_at = self.fusion.now
                heights = iter([.10] * 20 if delay else [.19, .21, .32] + [target] * 15)
                observed = []
                flying_seconds = [0.]

                def tick(seconds):
                    self.fusion.now += seconds
                    self.fusion.ros_now += seconds
                    drone.battery_at = drone.mode_at = drone.height_at = self.fusion.now
                    pose = copy.deepcopy(drone.local_pose)
                    pose.header.stamp = Stamp(self.fusion.ros_now)
                    drone._local_received(pose)
                    if drone.mode == 'FLYING':
                        flying_seconds[0] += seconds
                        drone.height = next(heights)
                        observed.append(drone.height)

                self.ros.sleep = tick
                drone.takeoff(hold_xy=True, continue_after=delay, height=height)
                self.assertEqual(drone.target_height, target)
                self.assertEqual(self.fusion.messages['/pidrone/desired/takeoff_height'][-1].data, target)
                self.assertEqual(drone.mode, 'FLYING')
                self.assertEqual(drone.xy_anchor, (1, 2))
                if delay is not None:
                    self.assertAlmostEqual(flying_seconds[0], 1.0)
                    self.assertEqual(drone.height, .10, 'do not wait for the old 0.20 m threshold')
                else:
                    self.assertEqual(drone.height, target)
                    self.assertGreaterEqual(observed.count(target), 11)

    def test_requested_takeoff_height_survives_mode_resets_without_canceling_ramp(self):
        for delivery in ('DISARMED', 'ARMED', 'FLYING'):
            with self.subTest(delivery=delivery):
                controller = self.fusion.controller()
                controller.current_position.z = .05
                for mode in ('DISARMED', 'ARMED', 'FLYING'):
                    controller.current_mode_callback(NS(mode=mode))
                    if mode == delivery:
                        controller.takeoff_height_callback(NS(data=.6))
                self.assertEqual(controller.desired_position.z, .6)
                ramp = controller.altitude.takeoff
                self.assertIsNotNone(ramp)
                for invalid in (0, -1, float('nan'), float('inf')):
                    controller.takeoff_height_callback(NS(data=invalid))
                self.assertEqual(controller.desired_position.z, .6)
                controller.altitude.step(.6, .05, 0, 1, ramp[0] + 1)
                self.assertAlmostEqual(controller.altitude.target, .15)
                controller.altitude.step(.6, .6, 0, 1, ramp[0] + 6)
                self.assertEqual(controller.altitude.target, .6)
                controller.current_mode_callback(NS(mode='DISARMED'))
                controller.current_mode_callback(NS(mode='ARMED'))
                self.assertEqual(controller.desired_position.z, controller.takeoff_height)

    def test_prototype_moves_after_timed_takeoff_and_lands_on_completion_or_failure(self):
        module = ModuleType('drone_controller')
        module.Drone = mock.Mock()
        drone = module.Drone.return_value
        drone.mode = 'FLYING'
        drone.finish = type(self.drone).finish.__get__(drone)
        self.ros.init_node = mock.Mock()
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'routines', 'prototype-1-forward.py')
        with mock.patch.dict(sys.modules, {'drone_controller': module}), \
                mock.patch.object(sys, 'argv', [script, '--fly', '--distance', '.3']):
            main = runpy.run_path(script)['main']
            main()
            self.assertEqual([call[0] for call in drone.mock_calls],
                             ['ready', 'takeoff', 'unlock_xy',
                              'move_forward', 'lock_xy', 'land'])
            drone.takeoff.assert_called_once_with(
                hold_xy=True, continue_after=1.0)
            drone.move_forward.assert_called_once_with(.3, speed_limit=.5, timeout=15.)
            drone.reset_mock()
            drone.move_forward.side_effect = RuntimeError('movement failed')
            with self.assertRaisesRegex(RuntimeError, 'movement failed'):
                main()
            drone.land.assert_called_once_with(hold_xy=True)

    def test_hover_height_precedes_timed_hold_and_preserves_landing_on_failure(self):
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'routines', 'hover_routine.py')
        for height, failed in [(None, False), (.5, False), (.5, True)]:
            with self.subTest(height=height, failed=failed):
                drone = mock.Mock(mode='FLYING', takeoff_height=.3)
                drone.finish = type(self.drone).finish.__get__(drone)
                if failed:
                    drone.takeoff.side_effect = RuntimeError('altitude did not settle')
                args = [script, '--fly', '--seconds', '10']
                if height is not None:
                    args += ['--height', str(height)]
                self.ros.init_node = mock.Mock()
                with mock.patch.dict(sys.modules, {'drone_controller': NS(Drone=lambda: drone)}), \
                        mock.patch.object(sys, 'argv', args):
                    main = runpy.run_path(script)['main']
                    if failed:
                        with self.assertRaisesRegex(RuntimeError, 'altitude did not settle'):
                            main()
                        drone.hover.assert_not_called()
                    else:
                        main()
                        drone.hover.assert_called_once_with(10.)
                expected = [mock.call.ready(), mock.call.takeoff(hold_xy=True, height=height)]
                if not failed:
                    expected.append(mock.call.hover(10.))
                expected.append(mock.call.land(hold_xy=True))
                self.assertEqual(drone.mock_calls, expected)

    def test_invalid_hover_height_is_rejected_before_initializing_flight(self):
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'routines', 'hover_routine.py')
        drone = mock.Mock()
        self.ros.init_node = mock.Mock()
        for height in ('0', '-.1', 'nan', 'inf'):
            with self.subTest(height=height), \
                    mock.patch.dict(sys.modules, {'drone_controller': NS(Drone=drone)}), \
                    mock.patch.object(sys, 'argv', [script, '--fly', '--height', height]), \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    runpy.run_path(script)['main']()
                self.assertEqual(error.exception.code, 2)
        self.ros.init_node.assert_not_called()
        drone.assert_not_called()

    def test_ready_uses_current_ground_height_and_rejects_legacy_publisher(self):
        self.drone.mode = 'DISARMED'
        self.nodes.get_node_names.return_value = ['/state_estimator', '/fusion_localization']
        with self.assertRaisesRegex(RuntimeError, 'legacy /state_estimator'):
            self.drone.ready()
        self.nodes.get_node_names.return_value = ['/fusion_localization']
        drone = self.drone
        for publisher in (drone.mode_pub, drone.twist_pub, drone.pose_pub, drone.landing_pub, drone.takeoff_pub,
                          drone.position_mode_pub, drone.heartbeat_pub):
            publisher.get_num_connections = lambda: 1
        drone.voltage, drone.battery_at = 16, self.fusion.now
        self.globals['time'].sleep = mock.Mock(side_effect=AssertionError('Unexpected readiness wait'))
        for height in (.065, .04, .063):
            drone.height = height
            drone.ready()
            self.assertEqual(drone.ground_height, height)
        self.globals['time'].sleep.assert_not_called()
        self.assertEqual(self.fusion.messages['/pidrone/desired/mode'], [])

    def test_absolute_waypoint_keeps_room_target_and_checks_reference_during_motion(self):
        drone = self.drone
        drone.mode, drone.height = 'FLYING', .5
        drone.battery_at = self.fusion.now
        pose = copy.deepcopy(drone.fused_pose)
        pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = quaternion(yaw=math.pi / 2)
        drone._fused_received(pose)
        commands = []
        def tick(seconds):
            command = self.fusion.messages['/pidrone/desired/twist'][-1]
            commands.append((command.linear.x, command.linear.y))
            pose.pose.pose.position.y += .1
            drone._fused_received(pose)
        self.ros.sleep = tick
        check = mock.Mock()
        drone.move_to(1, 2.2, 'map', check=check)
        self.assertTrue(all(x > 0 and abs(y) < 1e-8 for x, y in commands))
        self.assertGreaterEqual(check.call_count, 2)
        with self.assertRaisesRegex(RuntimeError, 'Waypoint frame'):
            drone.move_to(1, 2.2, 'other')
        with self.assertRaisesRegex(RuntimeError, 'room changed'):
            drone.move_to(1, 3, 'map', check=mock.Mock(side_effect=RuntimeError('room changed')))
        final = self.fusion.messages['/pidrone/desired/twist'][-1]
        self.assertEqual((final.linear.x, final.linear.y), (0, 0))


if __name__ == '__main__':
    unittest.main()
