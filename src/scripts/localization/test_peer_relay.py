"""Hardware-free checks of the laptop peer relay parser."""
import asyncio
import json
import math
from pathlib import Path
import runpy
import unittest


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

    def test_handshake_messages_are_forwarded_to_the_other_drone(self):
        plan = {'plan_id': 3, 'action': 'takeoff', 'params': [.5], 'start_time': {'secs': 110, 'nsecs': 0}}

        class Socket:
            def __init__(self, packets):
                self.packets, self.sent = packets, []

            def __aiter__(self):
                return self

            async def __anext__(self):
                if not self.packets:
                    raise StopAsyncIteration
                return self.packets.pop(0)

            async def send(self, packet):
                self.sent.append(json.loads(packet))

        leader = self.module['Client'](Socket([json.dumps(
            {'op': 'publish', 'topic': '/pidrone/coordination/plan', 'msg': plan})]))
        follower = self.module['Client'](Socket([]))
        leader.peer, follower.peer = follower, leader
        asyncio.run(leader.receive())
        self.assertEqual(follower.socket.sent, [
            {'op': 'publish', 'topic': '/pidrone/coordination/peer_plan', 'msg': plan}])
        self.assertEqual(leader.socket.sent, [])


if __name__ == '__main__':
    unittest.main()
