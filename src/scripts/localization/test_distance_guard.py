"""Hardware-free distance control through the real PID and peer relay parser."""
import math
from pathlib import Path
import runpy
import unittest
from types import SimpleNamespace as NS

import numpy as np

from localization import test_fusion as helpers
from localization.distance_guard import DistanceGuard


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
    """The PID no longer applies a distance guard to arming or motion."""

    def setUp(self):
        self.fixture = helpers.FusionNodeTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.c = self.fixture.controller()
        self.c.flight_readiness.__globals__['monotonic'] = lambda: self.fixture.now

    def test_paired_preflight_and_motion_ignore_peer_spacing(self):
        self.c.current_mode_callback(NS(mode='DISARMED'))
        self.c.coordination_selection_callback(NS(data=True))
        report = self.c.flight_readiness()
        self.assertTrue(report.ready, 'paired arming must not wait for peer data')
        self.assertTrue(report.paired)
        self.c.current_mode_callback(NS(mode='ARMED'))
        self.c.current_mode_callback(NS(mode='FLYING'))
        command = helpers.twist()
        command.linear.y = .05
        self.c.desired_twist_callback(command)
        self.c.calc_error()
        self.assertGreater(self.c.pid_error.y, 0)
        self.assertEqual(self.c.coordination_status, 'disabled')

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


if __name__ == '__main__':
    unittest.main()
