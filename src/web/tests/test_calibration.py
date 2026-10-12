"""Calibration geometry, BLE workflow and grounded-write interlocks (no hardware)."""
import ast
import asyncio
import copy
import json
import math
import os
import socket
import sys
from pathlib import Path
import struct
import threading
import unittest
from contextlib import asynccontextmanager
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from calibration.anchor_calibration import (decode_info, decode_location, solve_layout,
    DATA_MODE, LOCATION, CLUSTER, SERVICE)
from calibration.calibration_guard import BridgeGuard, ANCHORS, ACK, OWNER, connect_bridge
from calibration.pans_ble import PansBLE
from calibration.calibration_service import AnchorCalibration, CalibrationJournal
from tempfile import TemporaryDirectory


POINTS = [(0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0)]
BOARDS = [dict(id='%04X' % (i+1), ble='00:00:00:00:00:%02X' % i, z=p[2], pan=0,
               address='%016X' % (i+1), initiator=i == 0, anchor=True, active=True)
          for i, p in enumerate(POINTS)]
ROWS = {a['id']: {b['id']: (math.sqrt(sum((x-y)**2 for x,y in zip(POINTS[i], POINTS[j]))), 100)
                 for j, b in enumerate(BOARDS) if i != j} for i, a in enumerate(BOARDS)}


def payload(row):
    return struct.pack('<BB', 1, len(row)) + b''.join(
        struct.pack('<HIB', int(key, 16), int(round(distance*1000)), quality)
        for key, (distance, quality) in row.items())


class CalibrationTest(unittest.TestCase):
    def test_device_info_accepts_legacy_zero_padding_but_rejects_malformed_fields(self):
        info = struct.pack('<Q', 0xDECA71FB90744D12) + bytes(21)
        mode, pan = b'\xc0\x80', b'\x00\x00'
        expected = dict(id='4D12', address='DECA71FB90744D12', anchor=True,
                        active=True, initiator=True, pan=0)
        for data in (info, info + bytes(16)):
            self.assertEqual(decode_info(data, mode, pan), expected)
            self.assertEqual(decode_info(data, mode, b''), dict(expected, pan=None))
        self.assertEqual(decode_info(info, mode, b'\x34\x12')['pan'], 0x1234)
        for data, flags, network in ((info[:22], mode, pan), (info + bytes(15), mode, pan),
                                     (info + bytes(15) + b'\x01', mode, pan),
                                     (info, mode[:1], pan), (info, mode, pan[:1])):
            with self.assertRaisesRegex(ValueError, 'info=.*mode=.*PAN='):
                decode_info(data, flags, network)

    def test_bridge_retries_resolved_addresses_after_timeout_and_closes_failed_handshake(self):
        url = 'ws://drone2.local:9090'
        addresses = [(socket.AF_INET6, socket.SOCK_STREAM, socket.IPPROTO_TCP, '', ('fe80::2', 9090, 0, 1)),
                     (socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP, '', ('192.0.2.2', 9090))]
        first, second = mock.Mock(), mock.Mock()
        first.connect.side_effect = TimeoutError('timed out')
        websocket = mock.Mock()
        with mock.patch('socket.getaddrinfo', return_value=addresses), \
                mock.patch('socket.socket', side_effect=[first, second]):
            bridge = connect_bridge(websocket, url)
        first.close.assert_called_once()
        second.connect.assert_called_once_with(addresses[1][4])
        second.settimeout.assert_called_once_with(5)
        websocket.create_connection.assert_called_once_with(url, socket=second, timeout=5)
        bridge.settimeout.assert_called_once_with(1)
        second.close.assert_not_called()
        websocket.create_connection.side_effect = RuntimeError('handshake failed')
        with mock.patch('socket.create_connection', return_value=second):
            with self.assertRaisesRegex(RuntimeError, 'handshake failed'):
                connect_bridge(websocket, url)
        second.close.assert_called_once()

    def test_shared_height_layout_from_ble_ranges_rejects_bad_data_and_corner_order(self):
        rows = {key: decode_location(payload(row))[1] for key, row in ROWS.items()}
        layout = solve_layout(BOARDS, rows)
        np.testing.assert_allclose([[p[k] for k in ('x', 'y', 'z')] for p in layout['positions']], POINTS, atol=1e-6)
        for bad in (b'', payload(ROWS['0001'])[:-1], b'\x03', payload({'0001': (0, 100)})):
            with self.assertRaises(ValueError):
                decode_location(bad)
        missing = copy.deepcopy(ROWS)
        del missing['0002']['0003']; del missing['0003']['0002']
        with self.assertRaisesRegex(ValueError, 'Missing good range'):
            solve_layout(BOARDS, missing)
        inconsistent = copy.deepcopy(ROWS)
        inconsistent['0001']['0002'] = (5, 100)
        with self.assertRaises(ValueError):
            solve_layout(BOARDS, inconsistent)
        with self.assertRaises(ValueError):
            solve_layout([BOARDS[i] for i in (0, 1, 3, 2)], ROWS)

    def test_guard_requires_both_fresh_disarmed_controllers_and_new_layout_reports(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner = 'coordinator'
        guard.participants = None
        guard.peers = [dict(socket=object(), mode='DISARMED', mode_at=10, locked=True, ack_at=10,
                            owner='coordinator', owner_at=10, url='ws://drone%d:9090' % i,
                            anchor_seq=1, anchor_stamp=None, anchors=None, anchors_at=0) for i in range(2)]
        guard.select([p['url'] for p in guard.peers])
        layout = solve_layout(BOARDS, ROWS)['positions']
        with mock.patch('calibration.calibration_guard.monotonic', return_value=10):
            guard.check(locked=True)
            guard.peers[1]['mode'] = 'ARMED'
            with self.assertRaises(ValueError): guard.check(locked=True)
            guard.peers[1]['mode'] = 'DISARMED'; guard.peers[1]['ack_at'] = 9
            with self.assertRaises(ValueError): guard.check(locked=True)
            guard.peers[1]['ack_at'] = 10
            report = dict(header=dict(stamp=dict(secs=10, nsecs=0), frame_id='map'),
                          anchor_ids=[p['id'] for p in layout], positions=layout)
            baseline = guard.sequences()
            for peer in guard.peers: guard.receive(peer, ANCHORS, report)
            guard.verify_layout(layout, baseline, lambda: guard.check(True))
            # Replaying the same report cannot advance acquisition or verification.
            baseline = guard.sequences()
            for peer in guard.peers: guard.receive(peer, ANCHORS, report)
            self.assertEqual(guard.sequences(), baseline)

    def test_competing_coordinator_cannot_claim_both_or_release_another_owner(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner, guard.claimed = 'ours', False
        guard.participants = None
        guard.peers = [dict(socket=object(), mode='DISARMED', mode_at=10, locked=False, ack_at=10,
                            owner='', owner_at=10, request='', url='ws://drone%d:9090' % i) for i in range(2)]
        guard.select([p['url'] for p in guard.peers])
        controllers = ['', 'other']
        def send(peer, message):
            i = guard.peers.index(peer)
            request = json.loads(message['msg']['data'])
            if request['action'] == 'acquire' and not controllers[i]:
                controllers[i] = request['owner']
            elif request['action'] == 'release' and controllers[i] == request['owner']:
                controllers[i] = ''
            guard.receive(peer, ACK, {'data': bool(controllers[i])})
            guard.receive(peer, OWNER, {'data': json.dumps(dict(owner=controllers[i], request=request['request']))})
        guard.send = send
        with mock.patch('calibration.calibration_guard.monotonic', return_value=10):
            with self.assertRaisesRegex(ValueError, 'another coordinator'):
                guard.set_locked(True)
            self.assertEqual(controllers, ['', 'other'])
            self.assertFalse(guard.claimed)
            controllers[1] = ''
            guard.receive(guard.peers[1], OWNER, {'data': json.dumps(dict(owner='', request=''))})
            guard.set_locked(True)
            guard.check(locked=True)
            self.assertEqual(controllers, ['ours', 'ours'])
            def stale(peer, message):
                guard.receive(peer, ACK, {'data': True})
                guard.receive(peer, OWNER, {'data': json.dumps(dict(owner='ours', request='old-request'))})
            guard.send = stale
            with mock.patch.object(guard.stop_event, 'wait', return_value=True):
                with self.assertRaisesRegex(ValueError, 'not acknowledged'):
                    guard.set_locked(False)
                # Failed re-acquisition must not release a lock protecting an incomplete save.
                with self.assertRaises(ValueError):
                    guard.set_locked(True)
            self.assertTrue(guard.claimed)
            self.assertEqual(controllers, ['ours', 'ours'])
            guard.send = send
            for peer in guard.peers: peer['owner_at'] = peer['ack_at'] = 10
            guard.set_locked(False)
            self.assertEqual(controllers, ['', ''])

    def test_dashboard_selects_one_or_both_without_dropping_calibration_participants(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner, guard.claimed, guard.participants = 'ours', False, None
        guard.peers = [dict(url='ws://drone1:9090', socket=object(), mode=None, mode_at=0,
                            owner=None, owner_at=0, locked=None, ack_at=0),
                       dict(url='ws://drone2:9090', socket=object(), mode='DISARMED', mode_at=10,
                            owner='', owner_at=10, locked=False, ack_at=10,
                            anchor_seq=1, anchor_stamp=None, anchors=None, anchors_at=0)]
        active = guard.peers[1]
        guard.select([active['url']])
        for invalid in (None, 'ws://drone2:9090', ['ws://other:9090'], [active['url']] * 2, [{}]):
            with self.assertRaises(ValueError): guard.select(invalid)
        def send(peer, message):
            self.assertIs(peer, active)  # An unselected bridge is reachable, but has no flight state.
            request = json.loads(message['msg']['data'])
            locked = request['action'] == 'acquire'
            guard.receive(peer, ACK, {'data': locked})
            guard.receive(peer, OWNER, {'data': json.dumps(dict(owner='ours' if locked else '', request=request['request']))})
        guard.send = send
        with mock.patch('calibration.calibration_guard.monotonic', return_value=10):
            guard.check()
            guard.set_locked(True)
            guard.check(locked=True)
            self.assertEqual(guard.participants, [active['url']])
            guard.select([])
            guard.check(locked=True)  # Disconnecting in the dashboard cannot bypass an existing lock.
            guard.select([active['url']])
            baseline = guard.sequences()
            layout = solve_layout(BOARDS, ROWS)['positions']
            guard.receive(active, ANCHORS, dict(header=dict(stamp=dict(secs=10, nsecs=0), frame_id='map'),
                                               anchor_ids=[p['id'] for p in layout], positions=layout))
            guard.verify_layout(layout, baseline, lambda: guard.check(True))
            active['socket'] = None
            with self.assertRaisesRegex(ValueError, 'cannot connect'): guard.check(True)
            with self.assertRaises(ValueError): guard.set_locked(False)
            self.assertEqual(guard.participants, [active['url']])
            active['socket'] = object()
            newcomer = guard.peers[0]
            guard.select([p['url'] for p in guard.peers])
            with self.assertRaisesRegex(ValueError, 'Fresh flight state'): guard.check(True)
            newcomer.update(socket=object(), mode='DISARMED', mode_at=10, owner='', owner_at=10, locked=False, ack_at=10)
            with self.assertRaisesRegex(ValueError, 'Every participating'): guard.check(True)
            guard.set_locked(False)  # Cancel releases the original participant without touching the newcomer.
            self.assertIsNone(guard.participants)
            guard.prepare()
            self.assertEqual(set(guard.participants), {p['url'] for p in guard.peers})
            guard.select([active['url']])
            newcomer['socket'] = None
            guard.prepare()  # A retry also retains the disconnected participant.
            with self.assertRaisesRegex(ValueError, 'cannot connect'): guard.check()


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.state_path = os.path.join(self.directory.name, 'calibration.json')
        self.guard = mock.Mock()
        self.guard.owner = 'laptop'
        self.guard.claimed = False
        self.guard.participants = None
        def prepare():
            self.guard.participants = ['ws://drone2.local:9090']
            return list(self.guard.participants)
        self.guard.prepare.side_effect = prepare
        def set_locked(value):
            self.guard.claimed = value
            if not value: self.guard.participants = None
        self.guard.set_locked.side_effect = set_locked
        self.ble = mock.Mock()
        self.ble.scan_warnings = []
        def call(action, guard, *args):
            if action == 'scan': return self.ble.scan(guard)
            if action == 'measure': return self.ble.measure(args[0], guard)
            if action == 'save': return self.ble.save(args[0], guard, args[1])
        self.ble.call.side_effect = call
        self.ble.scan.return_value, self.ble.measure.return_value = BOARDS, ROWS
        self.patch = mock.patch.multiple('calibration.calibration_service',
                                         BridgeGuard=mock.Mock(return_value=self.guard),
                                         PansBLE=mock.Mock(return_value=self.ble))
        self.patch.start(); self.addCleanup(self.patch.stop)
        self.node = self.create()
        self.node.run({'action': 'scan'})
        self.measure = {'action': 'measure', 'anchors': [dict(ble=b['ble']) for b in BOARDS]}

    def create(self):
        node = AnchorCalibration(['ws://drone2.local:9090'], self.state_path)
        self.addCleanup(lambda: None if node.stopping.is_set() else node.close())
        return node

    @property
    def params(self):
        with open(self.state_path) as stream:
            return {'~' + key: value for key, value in json.load(stream).items()}

    def test_partial_save_survives_restart_and_cannot_be_cancelled(self):
        self.node.run(self.measure)
        def fail(positions, guard, on_write):
            on_write(); raise ValueError('BLE write outcome unknown')
        self.ble.save.side_effect = fail
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.node.run(dict(action='cancel'))
        self.assertTrue(self.node.reserved)
        with self.assertRaisesRegex(ValueError, 'Another laptop calibration service'):
            CalibrationJournal(self.state_path)
        self.assertEqual(self.params['~participants'], ['ws://drone2.local:9090'])
        self.assertEqual(self.guard.set_locked.call_args_list, [mock.call(True)])
        self.node.close()
        self.node = self.create()
        self.assertTrue(self.node.dirty)
        self.assertEqual(len(self.node.boards), 4)
        self.node.run(self.measure)
        self.ble.save.side_effect = lambda positions, guard, on_write: on_write()
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.assertFalse(self.node.dirty)
        self.assertEqual(self.params['~participants'], [])
        self.guard.set_locked.assert_called_with(False)

    def test_cancel_during_ranging_releases_only_unwritten_layout(self):
        def cancel(boards, guard):
            self.node.cancel.set(); guard()
        self.ble.measure.side_effect = cancel
        self.node.run(self.measure)
        self.assertFalse(self.node.reserved)
        self.guard.set_locked.assert_called_with(False)
        self.ble.save.assert_not_called()
        # Recovered ownership without a preview must expose Cancel after a process restart.
        self.guard.claimed = True
        self.node.tick()
        self.assertTrue(self.node.reserved)
        self.node.run({'action': 'cancel'})
        self.assertFalse(self.node.reserved)

    def test_losing_ownership_does_not_leave_a_false_reservation_or_start_ranging(self):
        self.guard.set_locked.side_effect = ValueError('Another coordinator owns the anchors')
        self.node.run(self.measure)
        self.assertFalse(self.node.reserved)
        self.ble.measure.assert_not_called()
        self.ble.save.assert_not_called()

    def test_missing_tag_verification_keeps_flight_inhibited_and_expired_preview_cannot_write(self):
        self.node.run(self.measure)
        self.node.preview_at -= 121
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.ble.save.assert_not_called()
        self.assertFalse(self.node.reserved)
        self.node.run(self.measure)
        self.ble.save.side_effect = lambda positions, guard, on_write: on_write()
        self.guard.verify_layout.side_effect = ValueError('Second tag has not reported the layout')
        self.guard.set_locked.reset_mock()
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.assertTrue(self.node.dirty)
        self.assertTrue(self.node.reserved)
        self.guard.set_locked.assert_not_called()


class FlightInterlockTest(unittest.TestCase):
    def test_lock_is_ground_only_blocks_arming_and_preserves_disarm(self):
        # Execute the controller's real callbacks without opening the serial board.
        source = str(Path(__file__).resolve().parents[2] / 'scripts' / 'flight_controller_node.py')
        with open(source) as stream: tree = ast.parse(stream.read())
        kind = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'FlightController')
        kind.body = [m for m in kind.body if isinstance(m, ast.FunctionDef) and m.name in
                     ('desired_mode_callback', 'calibration_callback', 'calibration_owner_callback', 'update_command')]
        namespace = dict(rospy=NS(set_param=mock.Mock(), logwarn=mock.Mock()), json=json,
                         cmds=NS(disarm_cmd=[0], arm_cmd=[1], idle_cmd=[2]))
        exec(compile(ast.Module(body=[kind], type_ignores=[]), source, 'exec'), namespace)
        fc = namespace['FlightController']()
        fc.mode_lock, fc.curr_mode, fc.prev_mode, fc.calibrating = threading.RLock(), 'ARMED', 'DISARMED', False
        fc.calibration_owner, fc.calibration_request = '', ''
        fc.calibration_callback(NS(data=True))
        self.assertFalse(fc.calibrating)
        fc.desired_mode_callback(NS(mode='DISARMED'))
        fc.calibration_callback(NS(data=True))
        for mode in ('ARMED', 'FLYING'): fc.desired_mode_callback(NS(mode=mode))
        self.assertEqual(fc.curr_mode, 'DISARMED')
        fc.desired_mode_callback(NS(mode='DISARMED'))
        self.assertEqual(fc.command, [0])
        fc.calibration_callback(NS(data=False))
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'ARMED')
        def claim(owner, action):
            fc.calibration_owner_callback(NS(data=json.dumps(dict(owner=owner, action=action, request=owner + action))))
        claim('first', 'acquire')
        self.assertFalse(fc.calibrating)  # An armed controller cannot grant ownership.
        fc.desired_mode_callback(NS(mode='DISARMED'))
        claim('first', 'acquire')
        claim('second', 'acquire')
        claim('second', 'release')
        fc.calibration_callback(NS(data=False))
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.calibration_owner, 'first')
        self.assertTrue(fc.calibrating)
        self.assertEqual(fc.curr_mode, 'DISARMED')
        claim('first', 'release')
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'ARMED')


class LaptopBLETest(unittest.IsolatedAsyncioTestCase):
    async def test_scan_accepts_service_data_only_and_skips_tags_and_failed_connections(self):
        import sys
        from calibration.anchor_calibration import INFO, MODE, PAN
        ble = PansBLE.__new__(PansBLE)
        ble.devices = {}
        entries = {key: (NS(address=key), NS(service_uuids=[], service_data={SERVICE: bytes([mode])}))
                   for key, mode in [('tag', 0x02), ('failed', 0x82), ('anchor-uuid', 0x8a)]}
        scanner = mock.AsyncMock()
        scanner.discovered_devices_and_advertisement_data = entries
        client = mock.AsyncMock()
        info = {INFO: struct.pack('<Q', 1) + bytes(37), MODE: b'\xc0\x80', PAN: b''}
        client.read_gatt_char.side_effect = lambda key: info[key]
        def create(device, **kwargs):
            if device.address == 'failed':
                raise RuntimeError('connection rejected')
            return client
        fake = NS(BleakScanner=mock.Mock(return_value=scanner), BleakClient=create, BleakError=RuntimeError)
        with mock.patch.dict(sys.modules, {'bleak': fake}), mock.patch('calibration.pans_ble.asyncio.sleep', new=mock.AsyncMock()):
            boards = await ble._scan(lambda: None)
        self.assertEqual([(b['ble'], b['id']) for b in boards], [('anchor-uuid', '0001')])
        self.assertIsNone(boards[0]['pan'])
        self.assertNotIn('tag', ble.devices)
        self.assertIn('failed', ble.scan_warnings[0])
        client.disconnect.assert_awaited_once()

    async def test_pans_ranging_and_save_use_notifications_and_verify_readback_and_cleanup(self):
        from calibration.anchor_calibration import POSITION, encode_position
        ble = PansBLE.__new__(PansBLE)
        ble.loop, ble.timeout = asyncio.get_running_loop(), 1
        ble.verify = mock.AsyncMock()
        clients, modes, saved = {}, {}, {}
        for board in BOARDS:
            key = board['id']
            client = mock.AsyncMock()
            modes[key] = b'\x02'
            saved[key] = dict(board, x=POINTS[int(key, 16)-1][0], y=POINTS[int(key, 16)-1][1])
            def read(uuid, key=key):
                if uuid == DATA_MODE: return modes[key]
                if uuid == CLUSTER: return bytes(5)
                return b'\x00' + encode_position(saved[key]) if modes[key] == b'\x00' else payload(ROWS[key])
            def write(uuid, data, response, key=key):
                self.assertTrue(response)
                if uuid == DATA_MODE: modes[key] = bytes(data)
            client.read_gatt_char.side_effect, client.write_gatt_char.side_effect = read, write
            client.start_notify.side_effect = lambda uuid, callback, key=key: callback(None, payload(ROWS[key]))
            clients[key] = client
        @asynccontextmanager
        async def connect(board, guard):
            guard()
            yield clients[board['id']]
        ble.connect = connect
        for pans in ([None] * 4, [0, None, 0, None], [0] * 4):
            with self.subTest(pans=pans):
                boards = [dict(board, pan=pan) for board, pan in zip(BOARDS, pans)]
                rows = await ble._measure(boards, lambda: None)
                self.assertEqual(rows, ROWS)
                np.testing.assert_allclose(
                    [[p[k] for k in ('x', 'y', 'z')] for p in solve_layout(boards, rows)['positions']], POINTS)
        with self.assertRaisesRegex(ValueError, 'same PAN'):
            await ble._measure([dict(board, pan=pan) for board, pan in zip(BOARDS, [0, None, 1, None])], lambda: None)
        clients['0001'].stop_notify.assert_awaited_with(LOCATION)
        positions = solve_layout(BOARDS, rows)['positions']
        on_write = mock.Mock()
        await ble._save(positions, on_write, lambda: None)
        self.assertEqual(on_write.call_count, 4)
        for board in positions:
            clients[board['id']].write_gatt_char.assert_any_await(POSITION, encode_position(board), response=True)
        saved['0001']['x'] = 10
        with self.assertRaisesRegex(ValueError, 'failed verification'):
            await ble._save(positions, on_write, lambda: None)
        clients['0001'].start_notify.side_effect = ValueError('cancelled')
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            await ble._measure(BOARDS, lambda: None)
        self.assertEqual(set(modes.values()), {b'\x02'})


class LaptopCancellationTest(unittest.TestCase):
    def test_lost_flight_interlock_cancels_inflight_ble_and_waits_for_cleanup(self):
        ble = PansBLE()
        self.addCleanup(ble.close)
        started, cleaned = threading.Event(), threading.Event()
        async def stalled(guard):
            try:
                started.set()
                await asyncio.Event().wait()
            finally:
                cleaned.set()
        ble._scan = stalled
        def guard():
            if started.is_set(): raise ValueError('Flight interlock lost')
        with self.assertRaisesRegex(ValueError, 'Flight interlock lost'):
            ble.call('scan', guard)
        self.assertTrue(cleaned.is_set())
