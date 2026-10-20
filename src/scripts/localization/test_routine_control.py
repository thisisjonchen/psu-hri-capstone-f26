"""Routine hold integration with the real fusion/PID code and mocked ROS transport."""
import copy
import json
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
        sys.modules['std_msgs.msg'].String = NS
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
        drone.move(x=.2, speed_limit=.06)
        self.assertEqual(len(commands), 2)
        self.assertGreater(commands[0][0], 0)
        self.assertAlmostEqual(commands[0][1], 0)
        self.assertAlmostEqual(commands[1][0], 0)
        self.assertGreater(commands[1][1], 0)
        self.assertTrue(all(math.hypot(*command) <= .06 for command in commands))
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

    def advance_until(self, release_at=None, run=None):
        started = self.fusion.now
        self.drone.voltage = 16.
        self.drone.battery_at = self.drone.mode_at = self.drone.height_at = started

        def sleep(seconds):
            self.fusion.now += seconds
            self.drone.battery_at = self.drone.mode_at = self.drone.height_at = self.fusion.now
            if release_at is not None and self.fusion.now - started >= release_at:
                self.drone._start_received(None)
                if run is not None:
                    self.drone._stage_received(NS(data=json.dumps({'run': run, 'stage': 'start'})))
        self.globals['time'] = NS(time=lambda: self.fusion.now, sleep=sleep)

    def test_synchronized_start_posts_ready_and_ignores_an_earlier_start(self):
        drone = self.drone
        drone.mode = 'DISARMED'
        drone._start_received(None)  # From a previous run: must not release this one.
        self.advance_until(release_at=.5)
        drone.wait_for_start()
        ready = self.fusion.messages['/pidrone/routine/ready']
        self.assertTrue(2 <= len(ready) <= 4)  # 5 Hz for 0.5 s
        drone.routine_run = 'old-run'
        self.advance_until(release_at=.01)
        with self.assertRaisesRegex(RuntimeError, 'No synchronized start'):
            drone.wait_for_start(timeout=.1, require_run=True)
        self.assertFalse(drone.waiting_for_start)
        self.advance_until(release_at=.01, run='new-run')
        drone.wait_for_start(timeout=.1, require_run=True)
        self.assertEqual(drone.routine_run, 'new-run')

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
        self.advance_until(release_at=.01)
        with self.assertRaisesRegex(RuntimeError, 'bad waypoint plan'):
            drone.takeoff(preflight=mock.Mock(side_effect=RuntimeError('bad waypoint plan')))
        drone.mode_pub.publish.assert_not_called()

    def test_stage_wait_holds_position_and_requires_the_current_run_release(self):
        drone = self.drone
        drone.mode = 'FLYING'
        drone.lock_xy()
        drone.routine_run = 'current-run'
        self.advance_until()
        sleep = self.globals['time'].sleep
        started = self.fusion.now

        def tick(seconds):
            sleep(seconds)
            run = 'old-run' if self.fusion.now - started < .5 else 'current-run'
            drone._stage_received(NS(data=json.dumps({'run': run, 'stage': 'land'})))

        self.globals['time'].sleep = tick
        drone.wait_for_stage('land', timeout=1)
        self.assertGreaterEqual(self.fusion.now - started, .5)
        self.assertEqual(drone.xy_anchor, (1, 2))
        self.assertGreater(len(self.fusion.messages['/pidrone/desired/twist']), 1)
        ready = self.fusion.messages['/pidrone/routine/stage_ready'][-1]
        self.assertEqual(json.loads(ready.data), {'run': 'current-run', 'stage': 'land'})
        self.globals['time'].sleep = sleep
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            drone.wait_for_stage('forward', timeout=.1)
        drone.mode = 'DISARMED'
        with self.assertRaisesRegex(RuntimeError, 'Flight mode changed'):
            drone.wait_for_stage('land')

    def test_prototype_sequences_stages_and_lands_on_movement_failure(self):
        module = ModuleType('drone_controller')
        module.Drone = mock.Mock()
        drone = module.Drone.return_value
        drone.mode = 'FLYING'
        self.ros.init_node = mock.Mock()
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'routines', 'prototype-1-forward.py')
        with mock.patch.dict(sys.modules, {'drone_controller': module}), \
                mock.patch.object(sys, 'argv', [script, '--fly', '--distance', '.3']):
            main = runpy.run_path(script)['main']
            main()
            self.assertEqual([call[0] for call in drone.mock_calls],
                             ['ready', 'takeoff', 'wait_for_stage', 'unlock_xy',
                              'move_forward', 'lock_xy', 'wait_for_stage', 'land'])
            self.assertEqual(drone.wait_for_stage.call_args_list, [mock.call('forward'), mock.call('land')])
            drone.move_forward.assert_called_once_with(.3, speed_limit=.06, timeout=15.)
            drone.reset_mock()
            drone.move_forward.side_effect = RuntimeError('movement failed')
            with self.assertRaisesRegex(RuntimeError, 'movement failed'):
                main()
            drone.land.assert_called_once_with(hold_xy=True)
            drone.wait_for_stage.assert_called_once_with('forward')

    def test_ready_uses_current_ground_height_and_rejects_legacy_publisher(self):
        self.drone.mode = 'DISARMED'
        self.nodes.get_node_names.return_value = ['/state_estimator', '/fusion_localization']
        with self.assertRaisesRegex(RuntimeError, 'legacy /state_estimator'):
            self.drone.ready()
        self.nodes.get_node_names.return_value = ['/fusion_localization']
        drone = self.drone
        for publisher in (drone.mode_pub, drone.twist_pub, drone.pose_pub,
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

    def test_tag_path_freezes_distinct_targets_and_rejects_stale_or_changed_map(self):
        sys.modules['pidrone_pkg.msg'].TagDetection = NS
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'apriltag_path.py')
        namespace = runpy.run_path(script)
        cls = namespace['AprilTagPath']
        cls.prepare.__globals__['time'] = NS(time=lambda: self.fusion.now)
        anchors = [dict(name=str(i), x=x, y=y, z=2.) for i, (x, y) in
                   enumerate([(0., 0.), (3., 0.), (3., 6.), (0., 6.)])]
        self.drone.routine_plan = dict(ids=['drone1', 'drone2'], tag_ids=[1, 2], frame='map',
            generations={'drone1': 7, 'drone2': 7}, anchors=anchors,
            starts={'drone1': [1., 2.], 'drone2': [1.75, 2.]},
            points=[dict(id=1, x=1.5, y=3., stamp=100.), dict(id=2, x=1.5, y=4., stamp=100.)])
        settings = dict(enabled=True, min_distance=.6, max_distance=.9, nominal_separation_m=.75,
                        drone_id='drone1')
        self.fusion.params['/pidrone/coordination'] = settings
        with mock.patch.dict(os.environ, {'ROS_MASTER_URI': 'http://drone1.local:11311'}):
            path = cls(self.drone, [1, 2], 'both')
        path._generation(NS(data=7))
        path._layout(NS(header=NS(frame_id='map', stamp=Stamp(100.)), anchor_ids=[a['name'] for a in anchors],
                        positions=[NS(x=a['x'], y=a['y'], z=a['z']) for a in anchors]))
        path._detection(NS(header=NS(stamp=Stamp(100.)), camera_online=True, tag_ids=[1]))
        path.prepare()
        self.assertEqual(path.targets, [(1.125, 3.), (1.125, 4.)])
        path.own = settings['drone_id'] = 'drone2'
        self.drone.fused_pose.pose.pose.position.x = 1.75
        path.prepare()
        self.assertEqual(path.targets, [(1.875, 3.), (1.875, 4.)])
        path.mode = 'single'
        with self.assertRaisesRegex(RuntimeError, 'participants'):
            path.prepare()
        self.drone.routine_plan['ids'] = ['drone2']
        path.prepare()
        self.assertEqual(path.targets, [(1.5, 3.), (1.5, 4.)])
        self.drone.routine_plan['points'][0]['stamp'] = -30.
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            path.prepare()
        self.drone.routine_plan['points'][0]['stamp'] = 100.
        self.drone.routine_plan['points'][0]['x'] = .01
        with self.assertRaisesRegex(RuntimeError, 'boundary'):
            path.prepare()
        path._generation(NS(data=8))
        with self.assertRaisesRegex(RuntimeError, 'reference changed'):
            path.check()

    def test_apriltag_routine_sequences_checkpoints_and_lands_if_confirmation_fails(self):
        control, path_module = ModuleType('drone_controller'), ModuleType('apriltag_path')
        control.Drone, path_module.AprilTagPath = mock.Mock(), mock.Mock()
        path_module.tag_ids = lambda text: [int(i) for i in text.split(',')]
        drone, route = control.Drone.return_value, path_module.AprilTagPath.return_value
        drone.mode = 'FLYING'
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'routines', 'prototype-1-apriltag.py')
        with mock.patch.dict(sys.modules, {'drone_controller': control, 'apriltag_path': path_module}), \
                mock.patch.object(sys, 'argv', [script, '--fly', '--drones', 'both', '--tags', '1,2']):
            main = runpy.run_path(script)['main']
            main()
            drone.takeoff.assert_called_once_with(hold_xy=True, synchronized_stages=True,
                                                  preflight=route.prepare, check=route.check)
            self.assertEqual(route.visit.call_args_list, [mock.call(0), mock.call(1)])
            self.assertEqual([call.args[0] for call in drone.wait_for_stage.call_args_list], ['tag-0', 'tag-1', 'land'])
            drone.reset_mock()
            route.visit.side_effect = RuntimeError('tag not visible')
            with self.assertRaisesRegex(RuntimeError, 'tag not visible'):
                main()
            drone.land.assert_called_once_with(hold_xy=True)
            drone.wait_for_stage.assert_called_once_with('tag-0', check=route.check)

    def test_tag_checkpoint_requires_a_new_visible_frame_and_times_out_in_hold(self):
        sys.modules['pidrone_pkg.msg'].TagDetection = NS
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'apriltag_path.py')
        cls = runpy.run_path(script)['AprilTagPath']
        cls.visit.__globals__['time'] = NS(time=lambda: self.fusion.now)
        path = cls.__new__(cls)
        path.drone, path.check = mock.Mock(), mock.Mock()
        path.plan, path.targets, path.ids = {'frame': 'map'}, [(1., 3.)], [4]
        path.drone._hold_pose.return_value = self.drone.fused_pose
        def hover(seconds):
            self.fusion.now += seconds
            self.fusion.ros_now += seconds
        path.drone.hover.side_effect = hover
        path.detection = NS(camera_online=True, tag_ids=[4], header=NS(stamp=Stamp(self.fusion.ros_now)))
        with self.assertRaisesRegex(RuntimeError, 'not visible'):
            path.visit(0)  # A tag visible before arrival cannot confirm a checkpoint.
        path.drone.lock_xy.assert_called_once()
        path.drone.reset_mock()
        def fresh(seconds):
            hover(seconds)
            path.detection.header.stamp = Stamp(self.fusion.ros_now)
        path.drone.hover.side_effect = fresh
        path.visit(0)
        self.assertEqual(path.drone.hover.call_args_list, [mock.call(.1), mock.call(1.)])


if __name__ == '__main__':
    unittest.main()
