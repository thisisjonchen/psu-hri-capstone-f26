"""Hardware-free distance control through the real PID and peer relay parser."""
import copy
import math
from pathlib import Path
import runpy
import unittest
from unittest import mock
from types import SimpleNamespace as NS

import numpy as np

from localization import test_fusion as helpers
from localization.distance_guard import DistanceGuard, automatic_coordination
from localization.fusion import target_velocity
from localization.test_ros_node import Stamp
from localization.test_velocity_controller import SimulatedAxis


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
        for gap, margin, reason in [(1.5, .12, ''), (.72, .12, 'too_close'),
                                   (1.88, .12, 'too_far'), (1.5, .6, 'uncertainty_too_large')]:
            self.assertEqual(guard.preflight((0, 0), (gap, 0), margin), reason)

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
        self.c = self.guarded_controller()
        self.c.room_generation_callback(NS(data=0))
        self.c.room_pose = copy.deepcopy(self.c.room_pose)
        self.c.room_pose.pose.covariance[0] = self.c.room_pose.pose.covariance[7] = .0004
        self.c.peer_state_callback.__globals__['monotonic'] = lambda: self.fixture.now

    def guarded_controller(self):
        controller = self.fixture.controller()
        # Existing guard cases exercise a flight with protection explicitly enabled.
        controller.coordination_enabled = controller.coordination_active = True
        return controller

    def peer(self, x=2.2):
        pose = copy.deepcopy(self.c.room_pose.pose)
        pose.pose.position.x = x
        return NS(header=NS(frame_id='map', stamp=Stamp(self.fixture.ros_now)),
                  drone_id='drone1', recipient_id='drone2', recipient_generation=0,
                  pose=pose, velocity=NS(x=0, y=0, z=0))

    def test_paired_preflight_requires_fresh_comparable_positions_and_clearance(self):
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.coordination_selection_callback(NS(data=True))
        self.assertEqual(self.c.flight_readiness().reason, 'peer_state_stale')
        peer = self.peer()
        peer.header.stamp = Stamp(self.fixture.ros_now - .1)
        self.c.peer_preview_callback(peer)
        report = self.c.flight_readiness()
        self.assertTrue(report.ready)
        self.assertTrue(report.paired)
        self.assertEqual(report.header.stamp.to_sec(), peer.header.stamp.to_sec())
        self.assertIsNone(self.c.peer_state, 'preflight must not enable the flight guard')
        reports = []
        self.c.preflight_pub = NS(publish=reports.append)
        self.c.calc_error()
        for _ in range(5):
            self.c.calc_error()
        self.assertEqual(len(reports), 1)
        self.fixture.now += .11
        self.c.calc_error()
        self.assertEqual(len(reports), 2)
        self.assertEqual(reports[-1].header.stamp.to_sec(), peer.header.stamp.to_sec())
        peer.recipient_generation = 99
        self.c.calc_error()
        self.assertEqual(reports[-1].reason, 'reference_mismatch')
        self.assertEqual(len(reports), 3, 'report readiness changes without waiting')
        self.assertEqual(self.c.flight_readiness().reason, 'reference_mismatch')
        peer.recipient_generation = 0
        peer.pose.covariance[0] = .15 ** 2
        self.assertEqual(self.c.flight_readiness().reason, 'uncertainty_too_large')
        peer.pose.covariance[0] = .0004
        self.c.room_ready_callback(NS(data=False))
        self.assertEqual(self.c.flight_readiness().reason, 'local_position_unavailable')
        self.c.room_ready_callback(NS(data=True))
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.assertFalse(self.c.flight_readiness().ready, 'arming requires a new takeoff check')
        self.c.peer_preview_callback(peer)
        self.assertTrue(self.c.flight_readiness().ready)
        self.fixture.now += .51  # Let the relay time out without advancing ROS time.
        self.c.peer_preview_callback(peer)
        self.assertFalse(self.c.flight_readiness().ready)

    def test_solo_preflight_needs_current_selection_and_local_estimate_but_not_uwb(self):
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.room_ready_callback(NS(data=False))
        self.assertEqual(self.c.flight_readiness().reason, 'flight_setup_unavailable')
        self.c.coordination_selection_callback(NS(data=False))
        self.assertTrue(self.c.flight_readiness().ready)
        self.assertFalse(self.c.flight_readiness().paired)
        self.c.local_ready_callback(NS(data=False))
        self.assertEqual(self.c.flight_readiness().reason, 'local_estimate_unavailable')
        self.c.local_ready_callback(NS(data=True))
        self.fixture.now += 2.1
        self.assertEqual(self.c.flight_readiness().reason, 'flight_setup_unavailable')

    def test_guard_defaults_off_and_only_changes_while_disarmed(self):
        self.c = self.fixture.controller()
        self.c.peer_state_callback.__globals__['monotonic'] = lambda: self.fixture.now
        self.c.room_generation_callback(NS(data=0))
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.assertFalse(self.c.coordination_enabled)
        self.c.coordination_selection_callback(NS(data=False))
        self.c.room_ready_callback(NS(data=False))
        self.assertTrue(self.c.flight_readiness().ready, 'disabled guard needs no UWB or peer')
        self.assertFalse(self.c.flight_readiness().paired)
        self.c.local_ready_callback(NS(data=False))
        self.assertEqual(self.c.flight_readiness().reason, 'local_estimate_unavailable')
        self.c.local_ready_callback(NS(data=True))
        self.c.room_ready_callback(NS(data=True))
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.coordination_enabled_callback(NS(data=True))
        self.assertTrue(self.c.flight_readiness().ready)
        self.c.current_mode_callback(NS(mode='FLYING'))
        self.c.coordination_enabled_callback(NS(data=True))
        command = helpers.twist()
        command.linear.x = .05
        self.c.desired_twist_callback(command)
        self.c.peer_state_callback(self.peer(1.5))  # Too close, but explicitly disabled.
        for elapsed in (0, .6):
            self.fixture.now += elapsed
            self.c.calc_error()
            self.assertEqual(self.c.coordination_status, 'disabled')
            self.assertFalse(self.c.coordination_active)
            self.assertAlmostEqual(self.c.current_velocity.x + self.c.pid_error.x, .05)
            self.assertEqual(self.c.current_mode, 'FLYING')
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.assertFalse(self.c.coordination_enabled)
        self.c.coordination_enabled_callback(NS(data=True))
        self.c.coordination_selection_callback(NS(data=True))
        self.assertEqual(self.c.flight_readiness().reason, 'peer_state_stale')
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.current_mode_callback(NS(mode='FLYING'))
        self.c.coordination_enabled_callback(NS(data=False))  # Cannot disable midflight either.
        self.c.desired_twist_callback(command)
        self.c.peer_state_callback(self.peer(1.5))
        self.c.calc_error()
        self.assertEqual(self.c.coordination_status, 'too_close')
        self.assertLess(self.c.pid_error.x, 0)

    def test_automatic_identity_and_legacy_flag_cannot_disable_paired_protection(self):
        geometry = {'nominal_separation_m': 1.5, 'payload_drop_m': .5}
        settings = {'enabled': False, 'min_distance': 1.2, 'max_distance': 1.8}
        for own, peer in [('drone1', 'drone2'), ('drone2', 'drone1')]:
            for suffix in ('', '.local'):
                config = automatic_coordination(geometry, settings, 'http://%s%s:11311' % (own, suffix))
                self.assertEqual((config['drone_id'], config['peer_id']), (own, peer))
                self.fixture.params['/pidrone/coordination'] = config
                controller = self.guarded_controller()
                self.assertEqual(controller.distance_guard.follow_peer, own == 'drone2')
                self.assertEqual(controller.distance_guard.nominal, 1.5)
                self.assertAlmostEqual(controller.distance_guard.speed_limit, .1 * 2. / 3. if own == 'drone1' else .1)
        for uri in ('', 'http://localhost:11311', 'http://container-id:11311'):
            with self.assertRaises(ValueError):
                automatic_coordination(geometry, settings, uri)
        # Also check parameters left over from an older ROS session.
        config['enabled'] = False
        self.fixture.params['/pidrone/coordination'] = config
        controller = self.guarded_controller()
        self.assertIsNotNone(controller.distance_guard)
        controller.desired_velocity.x = .05
        controller.calc_error()
        self.assertEqual(controller.coordination_status, 'peer_state_stale')
        self.assertEqual((controller.pid_error.x, controller.pid_error.y), (0, 0))

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

    def test_blocks_uncertain_forward_request_for_both_drones(self):
        for own, other in [('drone1', 'drone2'), ('drone2', 'drone1')]:
            self.fixture.params['/pidrone/coordination'].update(
                drone_id=own, peer_id=other, min_distance=.85,
                max_distance=1.15, nominal_separation_m=1.)
            controller = self.guarded_controller()
            controller.room_generation_callback(NS(data=0))
            controller.room_pose = copy.deepcopy(controller.room_pose)
            controller.room_pose.pose.covariance[0] = .01
            controller.room_pose.pose.covariance[7] = .01
            peer = self.peer()
            peer.drone_id, peer.recipient_id = other, own
            peer.pose.pose.position.x = controller.room_pose.pose.pose.position.x + 1.
            peer.header.stamp = Stamp(self.fixture.ros_now - .1)
            controller.peer_state_callback(peer)
            command = helpers.twist()
            command.linear.y = .5
            controller.desired_twist_callback(command)
            controller.step()
            self.assertEqual(controller.coordination_status, 'uncertainty_too_large')
            self.assertAlmostEqual(controller.current_velocity.y + controller.pid_error.y, 0)
            self.assertEqual(controller.current_mode, 'FLYING')

    def test_pair_recovers_from_outward_trim_after_tension_is_removed(self):
        # Start at rest with learned outward lean balancing the string's pull.
        # Remove that pull, retaining fresh position/velocity feedback throughout.
        controls, plants = [], []
        anchors, positions = [-.5, .5], [-.5, .5]
        for index, direction in enumerate((-1, 1)):
            self.fixture.params['/pidrone/coordination'] = dict(
                drone_id='drone%d' % (index+1), peer_id='drone%d' % (2-index),
                min_distance=.85, max_distance=1.15, nominal_separation_m=1.)
            controller = self.guarded_controller()
            controller.peer_state_callback.__globals__['monotonic'] = lambda: self.fixture.now
            controller.room_generation_callback(NS(data=0))
            controller.current_state = copy.deepcopy(controller.current_state)
            controller.room_pose = copy.deepcopy(self.c.room_pose)
            controller.room_pose.pose.pose.orientation = NS(x=0., y=0., z=0., w=1.)
            controller.velocity.integral[0] = direction * .4
            plant = SimulatedAxis()
            plant.lean = math.atan2(direction * .4, 9.80665)
            controls.append(controller)
            plants.append(plant)

        for _ in range(20*60):
            self.fixture.now += 1/60.
            self.fixture.ros_now += 1/60.
            outputs = []
            for index, controller in enumerate(controls):
                pose = controller.room_pose
                pose.header.stamp = Stamp(self.fixture.ros_now)
                pose.pose.pose.position.x = positions[index]
                current = controller.current_state
                current.header.stamp = Stamp(self.fixture.ros_now)
                current.pose_with_covariance = copy.deepcopy(pose.pose)
                current.twist_with_covariance.twist.linear.x = plants[index].velocity
                controller.current_state_callback(current)
                peer = self.peer(positions[1-index])
                peer.drone_id, peer.recipient_id = 'drone%d' % (2-index), 'drone%d' % (index+1)
                peer.velocity.x = plants[1-index].velocity
                controller.peer_state_callback(peer)
                command = helpers.twist()
                command.linear.x, command.linear.y = target_velocity(
                    (anchors[index], 2.), (positions[index], 2.), 0., .3, .1)
                controller.desired_twist_callback(command)
                outputs.append(controller.step()[0])
            for index, plant in enumerate(plants):
                plant.step(outputs[index])
                positions[index] += plant.velocity / 60.

        for index, plant in enumerate(plants):
            self.assertLess(abs(plant.velocity), .03, 'still drifting after the string released')
            self.assertAlmostEqual(positions[index], anchors[index], delta=.05,
                                   msg='did not return to the hover position')

    def test_preflight_peer_cannot_activate_or_feed_flight_protection(self):
        self.c.current_mode_callback(NS(mode='DISARMED'))
        peer = self.peer()
        self.c.peer_preview_callback(peer)
        self.c.calc_error()
        self.assertIsNone(self.c.peer_state)
        self.assertEqual(self.c.coordination_status, 'inactive')
        self.c.coordination_selection_callback(NS(data=False))
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.current_mode_callback(NS(mode='FLYING'))
        self.c.peer_preview_callback(self.peer())
        self.c.calc_error()
        self.assertEqual(self.c.coordination_status, 'solo')
        self.assertIsNone(self.c.peer_state)
        self.assertIsNone(self.c.peer_preview)

    def test_solo_flight_enables_protection_on_peer_arrival_and_keeps_it_until_disarm(self):
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
        self.c.peer_state_callback(self.peer())  # Peer joins the solo flight.
        self.c.calc_error()
        self.assertTrue(self.c.coordination_active)
        self.assertNotEqual(self.c.coordination_status, 'solo')
        self.c.coordination_selection_callback(NS(data=False))  # Peer/base disconnects.
        self.fixture.now += .6
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
        # Let the relay time out without advancing ROS time.
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
        topics, messages = self.module['peer_messages'](snapshots, 100.1, 10.1)
        flight, preview = self.module['PEER_TOPIC'], self.module['PREVIEW_TOPIC']
        self.assertEqual(topics, [flight, flight])
        for index, message in enumerate(messages):
            self.assertEqual(self.module['stamp'](message['header']), 99.95)
            self.assertEqual(message['header']['frame_id'], 'map')
            self.assertAlmostEqual(message['velocity']['x'], 0)
            self.assertAlmostEqual(message['velocity']['y'], .1)
            self.assertEqual(message['recipient_id'], 'drone%d' % (index+1))
            self.assertEqual(message['drone_id'], 'drone%d' % (2-index))
        for modes, expected in [
                (('ARMED', 'ARMED'), [preview, preview]),
                (('FLYING', 'ARMED'), [flight, preview]),
                (('ARMED', 'FLYING'), [preview, flight]),
                (('FLYING', 'FLYING'), [flight, flight]),
                (('FLYING', 'DISARMED'), [preview, preview]),
                (('DISARMED', 'FLYING'), [preview, preview]),
                (('DISARMED', 'DISARMED'), [preview, preview])]:
            with self.subTest(modes=modes):
                for snapshot, mode in zip(snapshots, modes):
                    snapshot['mode'][0]['mode'] = mode
                topics, routed = self.module['peer_messages'](snapshots, 100.1, 10.1)
                self.assertEqual(topics, expected)
                self.assertEqual(routed, messages)
        snapshots[0]['mode'][0]['mode'] = 'UNKNOWN'
        with self.assertRaisesRegex(ValueError, 'known flight mode'):
            self.module['peer_messages'](snapshots, 100.1, 10.1)
        # Receiving an old measurement again must not make it fresh.
        snapshots = [{k: (v[0], 11) for k, v in s.items()} for s in snapshots]
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            self.module['peer_messages'](snapshots, 101, 11)

    def test_pair_requires_matching_maps_and_both_fresh_inputs(self):
        first, second = self.snapshot(), self.snapshot()
        first['generation'] = (first['generation'][0], 1)
        self.module['peer_messages']([first, second], 100.1, 10.1)
        missing = dict(first)
        del missing['generation']
        with self.assertRaisesRegex(ValueError, 'generation'):
            self.module['peer_messages']([missing, second], 100.1, 10.1)
        second['anchors'][0]['positions'][0]['x'] += .1
        with self.assertRaisesRegex(ValueError, 'anchor map'):
            self.module['peer_messages']([first, second], 100.1, 10.1)
        second = self.snapshot()
        del second['pose']
        with self.assertRaisesRegex(ValueError, 'missing/stale'):
            self.module['peer_messages']([first, second], 100.1, 10.1)
        with self.assertRaisesRegex(ValueError, 'missing/stale'):
            self.module['peer_messages']([first, self.snapshot()], 100.1, 10.51)

    def test_partial_reports_preserve_peer_telemetry_but_do_not_extend_layout_expiry(self):
        client = self.module['Client'](None)
        client.latest = self.snapshot()
        expected = self.module['peer_messages']([client.snapshot(), self.snapshot()], 100.1, 10.1)
        for count, elapsed, secs, nsecs in ((3, .1, 100, 0), (3, .1, 99, 990000000),
                                           (3, .1, 100, 100000000), (2, .2, 100, 200000000),
                                           (0, .3, 100, 300000000)):
            partial = copy.deepcopy(self.snapshot()['anchors'][0])
            partial['anchor_ids'] = partial['anchor_ids'][:count]
            partial['positions'] = partial['positions'][:count]
            partial['header']['stamp'] = dict(secs=secs, nsecs=nsecs)
            with mock.patch.object(self.module['time'], 'time', return_value=100 + elapsed), \
                    mock.patch.object(self.module['time'], 'monotonic', return_value=10 + elapsed):
                client.update('anchors', partial)
            self.assertEqual(self.module['peer_messages'](
                [client.snapshot(), self.snapshot()], 100 + elapsed, 10 + elapsed), expected)

        # Other telemetry keeps arriving; neither clock may be refreshed by partial anchors.
        snapshot = copy.deepcopy(client.snapshot())
        for key, (message, _) in list(snapshot.items()):
            if key != 'anchors':
                message = copy.deepcopy(message)
                if 'header' in message:
                    message['header']['stamp'] = dict(secs=100, nsecs=300000000)
                snapshot[key] = (message, 10.3)
        with self.assertRaisesRegex(ValueError, 'missing/stale anchors'):
            self.module['read_state'](snapshot, 100.49, 10.51)
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            self.module['read_state'](snapshot, 100.51, 10.49)

    def test_conflicting_partial_reports_require_a_new_complete_layout(self):
        partial = copy.deepcopy(self.snapshot()['anchors'][0])
        partial['anchor_ids'].pop()
        partial['positions'].pop()
        partial['header']['stamp']['nsecs'] = 100000000
        changes = [
            ('coordinates', lambda report: report['positions'][0].update(x=.1)),
            ('unknown ID', lambda report: report['anchor_ids'].__setitem__(0, '5')),
            ('duplicate ID', lambda report: report['anchor_ids'].__setitem__(0, '2')),
            ('missing position', lambda report: report['positions'].pop()),
            ('frame', lambda report: report['header'].update(frame_id='other')),
            ('future timestamp', lambda report: report['header']['stamp'].update(secs=101)),
        ]
        for name, change in changes:
            with self.subTest(name=name):
                client = self.module['Client'](None)
                client.latest = self.snapshot()
                report = copy.deepcopy(partial)
                change(report)
                with mock.patch.object(self.module['time'], 'time', return_value=100.2), \
                        mock.patch.object(self.module['time'], 'monotonic', return_value=10.2):
                    client.update('anchors', report)
                    with self.assertRaises(ValueError):
                        self.module['peer_messages']([client.snapshot(), self.snapshot()], 100.2, 10.2)
                    # A subsequent matching subset must not resurrect the invalidated layout.
                    client.update('anchors', partial)
                    with self.assertRaisesRegex(ValueError, 'four distinct'):
                        self.module['peer_messages']([client.snapshot(), self.snapshot()], 100.2, 10.2)
                    client.update('anchors', self.snapshot()['anchors'][0])
                self.module['peer_messages']([client.snapshot(), self.snapshot()], 100.2, 10.2)


if __name__ == '__main__':
    unittest.main()
