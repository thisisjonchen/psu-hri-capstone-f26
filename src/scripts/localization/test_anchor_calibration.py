"""Calibration geometry, BLE workflow and grounded-write interlocks (no hardware)."""
import ast
import copy
import json
import math
import os
import runpy
import struct
import sys
import threading
import unittest
from contextlib import contextmanager
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

import numpy as np

from localization.anchor_calibration import (decode_location, solve_layout,
    DATA_MODE, LOCATION, CLUSTER, SERVICE)
from localization.calibration_guard import BridgeGuard, ANCHORS, ACK, OWNER
from localization.pans_ble import PansBLE


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
    def test_scan_start_failure_preserves_cause_when_helper_already_stopped_or_cleanup_fails(self):
        for helper in (None, object()):
            scanner = NS(_helper=helper, start=mock.Mock(side_effect=ValueError('Permission denied')),
                         stop=mock.Mock(side_effect=ValueError('Helper not started (did you call connect()?)')),
                         _stopHelper=mock.Mock(side_effect=ValueError('cleanup failed')))
            ble = PansBLE.__new__(PansBLE)
            ble.interface, ble.btle = 0, NS(Scanner=object)
            ble.bounded = lambda base, guard: lambda interface: scanner
            with self.assertRaisesRegex(ValueError, 'Cannot start Bluetooth scan on hci0: Permission denied'):
                ble.scan(lambda: None)
            scanner._stopHelper.assert_called_once()
            self.assertEqual(scanner.stop.call_count, int(helper is not None))

    def test_connection_retries_other_address_type_once_and_remembers_success(self):
        failed = NS(_helper=None, connect=mock.Mock(side_effect=RuntimeError('Failed public connection')),
                    disconnect=mock.Mock(), _stopHelper=mock.Mock())
        connected = NS(_helper=object(), connect=mock.Mock(), setMTU=mock.Mock(),
                       disconnect=mock.Mock(), _stopHelper=mock.Mock())
        ble = PansBLE.__new__(PansBLE)
        ble.interface, ble.btle = 0, NS(Peripheral=object, BTLEDisconnectError=RuntimeError)
        factory = mock.Mock(side_effect=[failed, connected])
        ble.bounded = lambda base, guard: factory
        board = dict(BOARDS[0], addr_type='public')
        with ble.connect(board, lambda: None) as peripheral:
            self.assertIs(peripheral, connected)
        failed.connect.assert_called_once_with(board['ble'], addrType='public', iface=0)
        connected.connect.assert_called_once_with(board['ble'], addrType='random', iface=0)
        self.assertEqual(board['addr_type'], 'random')
        connected.disconnect.assert_called_once()
        # Two failed address types must still report failure and clean up both attempts.
        connected.connect.side_effect = RuntimeError('Failed random connection')
        connected.disconnect.side_effect = ValueError('Helper not started')
        factory.reset_mock(side_effect=True)
        factory.side_effect = [failed, connected]
        with self.assertRaisesRegex(RuntimeError, 'Failed random connection'):
            with ble.connect(dict(BOARDS[0], addr_type='public'), lambda: None):
                self.fail('Cannot use a failed connection')
        self.assertEqual(factory.call_count, 2)

    def test_discovery_skips_tags_and_reports_unreadable_boards_without_losing_anchors(self):
        def entry(address, mode, connectable=True):
            return NS(addr=address, addrType='public', connectable=connectable,
                      getScanData=lambda: [(0x21, 'Service Data', SERVICE.replace('-', '') + '%02x00' % mode)])
        entries = [entry('tag', 0x02), entry('failed', 0x82), entry('busy', 0x82, False), entry('anchor', 0x82)]
        scanner = NS(_helper=None, start=mock.Mock(), getDevices=lambda: entries, stop=mock.Mock(), _stopHelper=mock.Mock())
        ble = PansBLE.__new__(PansBLE)
        ble.interface, ble.btle = 0, NS(Scanner=object, BTLEException=RuntimeError)
        ble.bounded = lambda base, guard: lambda interface: scanner
        attempted = []
        @contextmanager
        def connect(board, guard):
            attempted.append(board['ble'])
            if board['ble'] == 'failed': raise RuntimeError('Failed to connect')
            yield object()
        ble.connect, ble.inspect = connect, lambda p: BOARDS[0]
        with mock.patch('localization.pans_ble.monotonic', side_effect=[0, 6]):
            boards = ble.scan(lambda: None)
        self.assertEqual(attempted, ['failed', 'anchor'])
        self.assertEqual([b['id'] for b in boards], [BOARDS[0]['id']])
        self.assertEqual(len(ble.scan_warnings), 2)
        self.assertIn('failed', ble.scan_warnings[0])
        self.assertIn('busy', ble.scan_warnings[1])

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

    def test_ble_measurement_disables_notifications_restores_modes_and_reads_every_anchor(self):
        ble = PansBLE.__new__(PansBLE)
        ble.btle, ble.timeout = NS(DefaultDelegate=object), 1
        devices = {}
        for board in BOARDS:
            mode, cccd = mock.Mock(), mock.Mock()
            mode.read.return_value = b'\x02'
            location = mock.Mock()
            location.getHandle.return_value = 20
            location.getDescriptors.return_value = [cccd]
            location.read.return_value = payload(ROWS[board['id']])
            cluster = mock.Mock(); cluster.read.return_value = b'\x00\x0f\x00\x0f\x00'
            peripheral = NS(chars={DATA_MODE: mode, LOCATION: location, CLUSTER: cluster}, delegate=None)
            def delegate(value, p=peripheral): p.delegate = value
            def notify(timeout, p=peripheral, data=location.read.return_value): p.delegate.handleNotification(20, data)
            peripheral.withDelegate, peripheral.waitForNotifications = delegate, notify
            devices[board['id']] = peripheral
        @contextmanager
        def connect(board, guard):
            guard(); yield devices[board['id']]
        ble.connect = connect
        ble.verify = mock.Mock()
        ble.characteristic = lambda p, key: p.chars[key]
        rows = ble.measure(BOARDS, lambda: None)
        self.assertEqual(set(rows), set(ROWS))
        initiator = devices['0001']
        descriptor = initiator.chars[LOCATION].getDescriptors.return_value[0]
        self.assertEqual([c.args[0] for c in descriptor.write.call_args_list], [b'\x01\x00', b'\x00\x00'])
        self.assertEqual(initiator.chars[DATA_MODE].write.call_args_list[-1].args[0], b'\x02')
        initiator.waitForNotifications = mock.Mock(side_effect=ValueError('cancelled'))
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            ble.measure(BOARDS, lambda: None)
        self.assertEqual(descriptor.write.call_args_list[-1].args[0], b'\x00\x00')
        self.assertEqual(initiator.chars[DATA_MODE].write.call_args_list[-1].args[0], b'\x02')

    def test_guard_requires_both_fresh_disarmed_controllers_and_new_layout_reports(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner = 'coordinator'
        guard.participants = None
        guard.peers = [dict(socket=object(), mode='DISARMED', mode_at=10, locked=True, ack_at=10,
                            owner='coordinator', owner_at=10, url='ws://drone%d:9090' % i,
                            anchor_seq=1, anchor_stamp=None, anchors=None, anchors_at=0) for i in range(2)]
        layout = solve_layout(BOARDS, ROWS)['positions']
        with mock.patch('localization.calibration_guard.monotonic', return_value=10):
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
        with mock.patch('localization.calibration_guard.monotonic', return_value=10):
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

    def test_one_drone_can_lock_verify_and_unlock_but_participants_cannot_disappear(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner, guard.claimed, guard.participants = 'ours', False, None
        guard.peers = [dict(url='ws://drone1:9090', socket=None, mode=None, mode_at=0,
                            owner=None, owner_at=0, locked=None, ack_at=0),
                       dict(url='ws://drone2:9090', socket=object(), mode='DISARMED', mode_at=10,
                            owner='', owner_at=10, locked=False, ack_at=10,
                            anchor_seq=1, anchor_stamp=None, anchors=None, anchors_at=0)]
        active = guard.peers[1]
        def send(peer, message):
            self.assertIs(peer, active)  # Never send to the absent drone.
            request = json.loads(message['msg']['data'])
            locked = request['action'] == 'acquire'
            guard.receive(peer, ACK, {'data': locked})
            guard.receive(peer, OWNER, {'data': json.dumps(dict(owner='ours' if locked else '', request=request['request']))})
        guard.send = send
        with mock.patch('localization.calibration_guard.monotonic', return_value=10):
            guard.check()
            guard.set_locked(True)
            guard.check(locked=True)
            self.assertEqual(guard.participants, [active['url']])
            baseline = guard.sequences()
            layout = solve_layout(BOARDS, ROWS)['positions']
            guard.receive(active, ANCHORS, dict(header=dict(stamp=dict(secs=10, nsecs=0), frame_id='map'),
                                               anchor_ids=[p['id'] for p in layout], positions=layout))
            guard.verify_layout(layout, baseline, lambda: guard.check(True))
            active['socket'] = None
            with self.assertRaisesRegex(ValueError, 'Fresh flight state'): guard.check(True)
            with self.assertRaises(ValueError): guard.set_locked(False)
            self.assertEqual(guard.participants, [active['url']])
            active['socket'] = object()
            newcomer = guard.peers[0]
            newcomer.update(socket=object(), mode='DISARMED', mode_at=10, owner='', owner_at=10, locked=False, ack_at=10)
            with self.assertRaisesRegex(ValueError, 'Every participating'): guard.check(True)
            guard.set_locked(False)  # Cancel releases the original participant without touching the newcomer.
            self.assertIsNone(guard.participants)
            guard.prepare()
            self.assertEqual(set(guard.participants), {p['url'] for p in guard.peers})
            newcomer['socket'] = None
            guard.prepare()  # A retry also retains the disconnected participant.
            with self.assertRaisesRegex(ValueError, 'Fresh flight state'): guard.check()


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.params = {}
        ros = ModuleType('rospy')
        ros.get_param = lambda key, default: self.params.get(key, default)
        ros.set_param = lambda key, value: self.params.update({key: copy.deepcopy(value)})
        ros.Publisher = ros.Subscriber = ros.Timer = ros.Duration = ros.on_shutdown = mock.Mock()
        ros.is_shutdown = lambda: False
        ros.get_time = lambda: 10
        ros.logwarn = mock.Mock()
        std = ModuleType('std_msgs'); msg = ModuleType('std_msgs.msg'); msg.String = NS
        with mock.patch.dict(sys.modules, {'rospy': ros, 'std_msgs': std, 'std_msgs.msg': msg}):
            self.kind = runpy.run_path(os.path.join(os.path.dirname(__file__), '..', 'PALS', 'anchor_calibration_node'))['AnchorCalibrationNode']
        self.guard = mock.Mock()
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
        self.ble.scan.return_value, self.ble.measure.return_value = BOARDS, ROWS
        self.patch = mock.patch.dict(self.kind.run.__globals__, {'BridgeGuard': mock.Mock(return_value=self.guard),
                                                                'PansBLE': mock.Mock(return_value=self.ble)})
        self.patch.start(); self.addCleanup(self.patch.stop)
        self.node = self.kind()
        self.node.run({'action': 'scan'})
        self.measure = {'action': 'measure', 'anchors': [dict(ble=b['ble']) for b in BOARDS]}

    def test_review_then_save_verifies_both_tags_before_unlock(self):
        self.node.boards = [dict(board, z=2) for board in BOARDS]
        self.node.run(self.measure)
        self.assertEqual(self.node.state['stage'], 'preview')
        self.assertEqual([p['z'] for p in self.node.preview['positions']], [0]*4)
        self.ble.save.assert_not_called()
        self.guard.set_locked.assert_called_with(True)
        def save(positions, guard, on_write): on_write()
        self.ble.save.side_effect = save
        token = self.node.state['token']
        self.node.run(dict(action='apply', token=token))
        self.assertEqual(self.node.state['stage'], 'complete')
        self.guard.verify_layout.assert_called_once()
        self.guard.set_locked.assert_called_with(False)
        self.assertFalse(self.params['~incomplete_save'])
        self.assertFalse(self.node.dirty)
        self.node.run(dict(action='apply', token=token))
        self.assertEqual(self.ble.save.call_count, 1)

    def test_partial_save_survives_restart_and_cannot_be_cancelled(self):
        self.node.run(self.measure)
        def fail(positions, guard, on_write):
            on_write(); raise ValueError('BLE write outcome unknown')
        self.ble.save.side_effect = fail
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.node.run(dict(action='cancel'))
        self.assertTrue(self.node.reserved)
        self.assertEqual(self.params['~participants'], ['ws://drone2.local:9090'])
        self.assertEqual(self.guard.set_locked.call_args_list, [mock.call(True)])
        self.node = self.kind()
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
        self.node.tick(None)
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
        source = os.path.join(os.path.dirname(__file__), '..', 'flight_controller_node.py')
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
