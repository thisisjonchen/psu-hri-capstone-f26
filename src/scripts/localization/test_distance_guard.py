"""Hardware-free distance control through the real PID and peer relay parser."""
import copy
import math
from pathlib import Path
import runpy
import unittest
from types import SimpleNamespace as NS

import numpy as np

from localization import test_fusion as helpers
from localization.distance_guard import DistanceGuard, automatic_coordination
from localization.test_ros_node import Stamp


class DistanceControlTest(unittest.TestCase):
    def test_radial_correction_preserves_route_motion_and_limits_speed(self):
        guard = DistanceGuard(.6, 2., 1.5, .4, .15)
        for gap, direction in ((.5, -1), (2.2, 1)):
            command, status = guard.filter((0, 0), (gap, 0), (0, 0), (0, .1), .12)
            self.assertGreater(command[0] * direction, 0)
            self.assertGreater(command[1], 0)
            self.assertLessEqual(math.hypot(*command), .15 + 1e-12)
            self.assertEqual(status, 'too_close' if direction < 0 else 'too_far')
        command, status = guard.filter((0, 0), (1.2, 0), (0, 0), (0, .1), .12)
        np.testing.assert_allclose(command, (0, .1))
        self.assertEqual(status, 'clear')
        command, status = guard.filter((0, 0), (.8, 0), (0, 0), (.15, 0), .12)
        self.assertTrue(0 < command[0] < .15, 'slow closing before reaching the minimum')
        self.assertEqual(status, 'limited')

    def test_invalid_or_uncertain_geometry_does_not_guess_an_escape_direction(self):
        guard = DistanceGuard(.6, 2., 1.5, .4, .15)
        for peer, margin in [((0, 0), .1), ((1, 0), 1), ((float('nan'), 0), .1)]:
            command, status = guard.filter((0, 0), peer, (0, 0), (.1, 0), margin)
            self.assertEqual(command, (0, 0))
            self.assertNotEqual(status, 'clear')
        for minimum, maximum, nominal in [(2, .6, 1.5), (.6, 2, 2), (.6, 2, float('nan'))]:
            with self.assertRaises(ValueError):
                DistanceGuard(minimum, maximum, nominal, .4, .15)
        command, status = guard.filter((0, 0), (1.5, 0), (0, 0), (.1, 0), .6)
        self.assertEqual((command, status), ((0, 0), 'uncertainty_too_large'))

    def test_follower_returns_to_nominal_even_with_opposing_route_requests(self):
        follower = DistanceGuard(1.2, 1.8, 1.5, .4, .15, follow_peer=True)
        for gap in (1.35, 1.65):
            command, status = follower.filter((gap, 0), (0, 0), (0, 0),
                                               (-.1 if gap < 1.5 else .1, .03), .06)
            self.assertGreater(command[0] * (1.5-gap), 0)
            self.assertAlmostEqual(command[1], .03)
            self.assertEqual(status, 'tracking_spacing')
        command, status = follower.filter((1.35, 0), (0, 0), (.14, 0), (0, .1), .06)
        self.assertGreater(command[0], 0)
        self.assertLessEqual(math.hypot(*command), .15 + 1e-12)
        self.assertEqual(status, 'limited')
        command, status = follower.filter((1.5, 0), (0, 0), (.08, .03), (.08, .03), .06)
        np.testing.assert_allclose(command, (.08, .03))
        self.assertEqual(status, 'clear')


class DistancePIDTest(unittest.TestCase):
    def setUp(self):
        self.fixture = helpers.FusionNodeTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.params['/pidrone/coordination'] = {
            'drone_id': 'drone2', 'peer_id': 'drone1', 'min_distance': .6, 'max_distance': 2.,
            'nominal_separation_m': 1.5}
        self.c = self.fixture.controller()
        self.c.room_generation_callback(NS(data=0))
        self.c.room_pose = copy.deepcopy(self.c.room_pose)
        self.c.room_pose.pose.covariance[0] = self.c.room_pose.pose.covariance[7] = .0004
        self.c.peer_state_callback.__globals__['monotonic'] = lambda: self.fixture.now

    def peer(self, x=2.2):
        pose = copy.deepcopy(self.c.room_pose.pose)
        pose.pose.position.x = x
        return NS(header=NS(frame_id='map', stamp=Stamp(self.fixture.ros_now)),
                  drone_id='drone1', recipient_id='drone2', recipient_generation=0,
                  pose=pose, velocity=NS(x=0, y=0, z=0))

    def test_automatic_identity_and_explicit_solo_configuration(self):
        geometry = {'nominal_separation_m': 1.5, 'payload_drop_m': .5}
        settings = {'enabled': True, 'min_distance': 1.2, 'max_distance': 1.8}
        for own, peer in [('drone1', 'drone2'), ('drone2', 'drone1')]:
            for suffix in ('', '.local'):
                config = automatic_coordination(geometry, settings, 'http://%s%s:11311' % (own, suffix))
                self.assertEqual((config['drone_id'], config['peer_id']), (own, peer))
                self.fixture.params['/pidrone/coordination'] = config
                controller = self.fixture.controller()
                self.assertEqual(controller.distance_guard.follow_peer, own == 'drone2')
                self.assertEqual(controller.distance_guard.nominal, 1.5)
                self.assertAlmostEqual(controller.distance_guard.speed_limit, .1 * 2. / 3. if own == 'drone1' else .1)
        for uri in ('', 'http://localhost:11311', 'http://container-id:11311'):
            with self.assertRaises(ValueError):
                automatic_coordination(geometry, settings, uri)
        settings['enabled'] = False
        self.fixture.params['/pidrone/coordination'] = automatic_coordination(geometry, settings, '')
        controller = self.fixture.controller()
        self.assertIsNone(controller.coordination_config)
        self.assertIsNone(controller.distance_guard)
        self.assertEqual(controller.coordination_status, 'disabled')

    def test_filters_route_velocity_after_body_conversion_without_replacing_command(self):
        # At map yaw +90 degrees, body +Y points map -X (away from this peer).
        q = self.c.room_pose.pose.pose.orientation
        q.x, q.y, q.z, q.w = helpers.quaternion(yaw=math.pi/2)
        self.c.peer_state_callback(self.peer(1.5))
        command = helpers.twist()
        command.linear.y = -.1  # Route asks to move toward the too-close peer.
        command.angular.z = .05
        self.c.desired_twist_callback(command)
        self.c.calc_error()
        self.assertGreater(self.c.pid_error.y, 0)
        self.assertEqual(self.c.desired_velocity.y, -.1)
        self.assertEqual(self.c.desired_yaw_velocity, .05)
        self.assertEqual(self.c.desired_position.z, .25)
        self.assertEqual(self.c.current_mode, 'FLYING')
        self.assertEqual(self.c.coordination_status, 'too_close')

    def test_blocked_forward_request_logs_filtered_velocity_for_both_drones(self):
        for own, other in [('drone1', 'drone2'), ('drone2', 'drone1')]:
            self.fixture.params['/pidrone/coordination'].update(
                drone_id=own, peer_id=other, min_distance=.85,
                max_distance=1.15, nominal_separation_m=1.)
            controller = self.fixture.controller()
            controller.room_generation_callback(NS(data=0))
            controller.room_pose = copy.deepcopy(controller.room_pose)
            controller.room_pose.pose.covariance[0] = .01
            controller.room_pose.pose.covariance[7] = .01
            peer = self.peer()
            peer.drone_id, peer.recipient_id = other, own
            controller.peer_state_callback(peer)
            command = helpers.twist()
            command.linear.y = .5
            controller.desired_twist_callback(command)
            controller.step()
            self.assertEqual(controller.coordination_status, 'uncertainty_too_large')
            self.assertAlmostEqual(controller.current_velocity.y + controller.pid_error.y / 100., 0)
            message = self.fixture.ros.loginfo_throttle.call_args[0][1]
            self.assertIn('desired=(0.000, 0.500)', message)
            self.assertIn('filtered=(0.000, 0.000) coordination=uncertainty_too_large', message)
            self.assertEqual(controller.current_mode, 'FLYING')

    def test_base_selection_is_latched_until_disarm_and_missing_selection_is_not_solo(self):
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.coordination_selection_callback(NS(data=False))
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.coordination_selection_callback(NS(data=True))  # Second drone connects after arming.
        self.c.current_mode_callback(NS(mode='FLYING'))
        command = helpers.twist()
        command.linear.x = .05
        self.c.desired_twist_callback(command)
        self.c.calc_error()
        self.assertGreater(self.c.pid_error.x, 0)
        self.assertEqual(self.c.coordination_status, 'solo')
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.coordination_selection_callback(NS(data=True))
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.coordination_selection_callback(NS(data=False))  # Peer/base disconnects.
        self.c.current_mode_callback(NS(mode='FLYING'))
        self.c.calc_error()
        self.assertTrue(self.c.coordination_active)
        self.assertEqual(self.c.coordination_status, 'peer_state_stale')
        self.assertEqual((self.c.pid_error.x, self.c.pid_error.y), (0, 0))
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.coordination_selection_callback(NS(data=False))
        self.fixture.now += 3
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.assertTrue(self.c.coordination_active, 'stale solo selection must not bypass the guard')

    def test_room_target_survives_stale_peer_and_resumes_without_recapture(self):
        target = NS(header=NS(frame_id='map', stamp=Stamp(self.fixture.ros_now)),
                    pose=NS(position=NS(x=1, y=3)))
        self.c.room_target_callback(target)
        peer = self.peer()
        self.c.peer_state_callback(peer)
        self.c.calc_error()
        self.assertGreater(self.c.pid_error.y, 0)
        self.assertLess(self.c.pid_error.x, 0, 'restore nominal spacing even inside the min/max band')
        # A frozen ROS clock cannot conceal a dead relay.
        self.fixture.now += .6
        self.c.peer_state_callback(peer)
        self.c.calc_error()
        self.assertEqual((self.c.pid_error.x, self.c.pid_error.y), (0, 0))
        self.assertEqual(self.c.coordination_status, 'peer_state_stale')
        np.testing.assert_array_equal(self.c.room_target, (1, 3))
        self.fixture.ros_now += .1
        peer = self.peer()
        self.c.peer_state_callback(peer)
        self.c.calc_error()
        self.assertGreater(self.c.pid_error.y, 0)
        self.assertLess(self.c.pid_error.x, 0)
        np.testing.assert_array_equal(self.c.room_target, (1, 3))

    def test_local_hold_cannot_bypass_filter_and_explicit_disarm_remains_available(self):
        self.c.peer_state_callback(self.peer(1.5))
        self.c.position_control = True
        self.c.desired_position.x = self.c.current_position.x + .5
        self.c.calc_error()
        self.assertLess(self.c.pid_error.x, 0)
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.calc_error()
        self.assertIsNone(self.c.peer_state)
        self.assertEqual(self.c.current_mode, 'DISARMED')
        self.assertEqual(self.c.coordination_status, 'inactive')

    def test_wrong_identity_reference_and_bad_covariance_do_not_enable_motion(self):
        for field, value in [('drone_id', 'drone2'), ('recipient_id', 'drone1')]:
            peer = self.peer()
            setattr(peer, field, value)
            self.c.peer_state_callback(peer)
            self.assertIsNone(self.c.peer_state)
        peer = self.peer()
        peer.recipient_generation = 9
        self.c.peer_state_callback(peer)
        self.c.calc_error()
        self.assertEqual(self.c.coordination_status, 'reference_mismatch')
        self.fixture.ros_now += .1
        peer = self.peer()
        peer.pose.covariance[0] = -1
        self.c.peer_state_callback(peer)
        self.c.calc_error()
        self.assertEqual(self.c.coordination_status, 'invalid_covariance')
        self.assertEqual((self.c.pid_error.x, self.c.pid_error.y), (0, 0))
        self.c.room_generation_callback(NS(data=1))
        self.assertIsNone(self.c.peer_state)


class PeerRelayTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[3] / 'tools' / 'peer_relay.py'
        self.module = runpy.run_path(str(path))

    def snapshot(self):
        header = {'stamp': {'secs': 100, 'nsecs': 0}, 'frame_id': 'map'}
        covariance = [0.] * 36
        covariance[0] = covariance[7] = .0004
        messages = {
            'pose': {'header': header, 'pose': {'covariance': covariance, 'pose': {
                'position': dict(x=1, y=2, z=.5),
                'orientation': dict(x=0, y=0, z=math.sqrt(.5), w=math.sqrt(.5))}}},
            'odom': {'header': {'stamp': {'secs': 99, 'nsecs': 950000000}, 'frame_id': 'odom'},
                     'child_frame_id': 'pidrone_body', 'twist': {'twist': {'linear': dict(x=.1, y=0)}}},
            'anchors': {'header': header, 'anchor_ids': ['1', '2', '3', '4'],
                        'positions': [dict(x=x, y=y, z=2) for x, y in ((0, 0), (3, 0), (3, 6), (0, 6))]},
            'ready': {'data': True}, 'mode': {'mode': 'FLYING'}, 'generation': {'data': 0}}
        return {key: (message, 10) for key, message in messages.items()}

    def test_peer_state_preserves_oldest_measurement_and_converts_velocity(self):
        snapshots = [self.snapshot(), self.snapshot()]
        messages = self.module['peer_messages'](snapshots, 100.1, 10.1)
        for index, message in enumerate(messages):
            self.assertEqual(self.module['stamp'](message['header']), 99.95)
            self.assertEqual(message['header']['frame_id'], 'map')
            self.assertAlmostEqual(message['velocity']['x'], 0)
            self.assertAlmostEqual(message['velocity']['y'], .1)
            self.assertEqual(message['recipient_id'], 'drone%d' % (index+1))
            self.assertEqual(message['drone_id'], 'drone%d' % (2-index))
        # Fresh network receipts must not refresh the age of old sensor data.
        snapshots = [{k: (v[0], 11) for k, v in s.items()} for s in snapshots]
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            self.module['peer_messages'](snapshots, 101, 11)

    def test_pair_requires_matching_maps_and_both_fresh_inputs(self):
        first, second = self.snapshot(), self.snapshot()
        second['anchors'][0]['positions'][0]['x'] += .1
        with self.assertRaisesRegex(ValueError, 'anchor map'):
            self.module['peer_messages']([first, second], 100.1, 10.1)
        second = self.snapshot()
        del second['pose']
        with self.assertRaisesRegex(ValueError, 'missing/stale'):
            self.module['peer_messages']([first, second], 100.1, 10.1)
        with self.assertRaisesRegex(ValueError, 'missing/stale'):
            self.module['peer_messages']([first, self.snapshot()], 100.1, 10.51)


if __name__ == '__main__':
    unittest.main()
