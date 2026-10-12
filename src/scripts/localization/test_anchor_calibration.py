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
    DATA_MODE, LOCATION, CLUSTER)
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
        guard.peers = [dict(socket=object(), mode='DISARMED', mode_at=10, locked=True, ack_at=10,
                            owner='coordinator', owner_at=10,
                            anchor_seq=1, anchor_stamp=None, anchors=None, anchors_at=0) for _ in range(2)]
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
            for peer in guard.peers: guard.receive(peer, ANCHORS, report)
            guard.verify_layout(layout, [1, 1], lambda: guard.check(True))
            # Replaying the same report cannot advance acquisition or verification.
            for peer in guard.peers: guard.receive(peer, ANCHORS, report)
            with self.assertRaises(ValueError): guard.verify_layout(layout, [2, 2], lambda: None, timeout=0)

    def test_competing_coordinator_cannot_claim_both_or_release_another_owner(self):
        guard = BridgeGuard.__new__(BridgeGuard)
        guard.lock, guard.stop_event = threading.RLock(), threading.Event()
        guard.owner, guard.claimed = 'ours', False
        guard.peers = [dict(socket=object(), mode='DISARMED', mode_at=10, locked=False, ack_at=10,
                            owner='', owner_at=10, request='') for _ in range(2)]
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
        self.guard.set_locked.side_effect = lambda value: setattr(self.guard, 'claimed', value)
        self.ble = mock.Mock()
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
        self.assertEqual(self.guard.set_locked.call_args_list, [mock.call(True)])
        self.node = self.kind()
        self.assertTrue(self.node.dirty)
        self.assertEqual(len(self.node.boards), 4)
        self.node.run(self.measure)
        self.ble.save.side_effect = lambda positions, guard, on_write: on_write()
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.assertFalse(self.node.dirty)
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
