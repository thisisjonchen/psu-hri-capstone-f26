"""Calibration geometry, BLE workflow and save recovery (no hardware)."""
import asyncio
import copy
import json
import math
import os
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


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.state_path = os.path.join(self.directory.name, 'calibration.json')
        self.ble = mock.Mock()
        self.ble.scan_warnings = []
        def call(action, guard, *args):
            if action == 'scan': return self.ble.scan(guard)
            if action == 'measure': return self.ble.measure(args[0], guard)
            if action == 'save': return self.ble.save(args[0], guard, args[1])
        self.ble.call.side_effect = call
        self.ble.scan.return_value, self.ble.measure.return_value = BOARDS, ROWS
        self.patch = mock.patch.multiple('calibration.calibration_service',
                                         PansBLE=mock.Mock(return_value=self.ble))
        self.patch.start(); self.addCleanup(self.patch.stop)
        self.node = self.create()
        self.node.run({'action': 'scan'})
        self.measure = {'action': 'measure', 'anchors': [dict(ble=b['ble']) for b in BOARDS]}

    def create(self):
        node = AnchorCalibration(self.state_path)
        self.addCleanup(lambda: None if node.stopping.is_set() else node.close())
        return node

    def test_offline_room_setup_saves_without_drone_acknowledgments(self):
        self.node.close()
        with open(self.state_path, 'w') as stream:
            json.dump({'participants': ['ws://disconnected.local:9090'], 'owner': 'old-owner',
                       'boards': BOARDS}, stream)
        self.node = self.create()
        self.node.run(self.measure)
        self.assertEqual(self.node.state['stage'], 'preview')
        self.ble.save.side_effect = lambda positions, check, on_write: (check(), on_write())
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.assertEqual(self.node.state['stage'], 'complete')
        self.assertTrue(self.node.state['layout']['verified'])
        layout = self.node.status()['layout']
        self.node.close()
        self.node = self.create()
        self.assertEqual(self.node.status()['layout'], dict(layout, verified=False))
        self.assertEqual(self.node.boards, BOARDS)

    def test_cancel_retains_unverified_save_for_recovery(self):
        self.node.run(self.measure)
        def fail(positions, guard, on_write):
            on_write(); raise ValueError('BLE write outcome unknown')
        self.ble.save.side_effect = fail
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.node.run(dict(action='cancel'))
        self.assertTrue(self.node.dirty)
        self.assertIsNone(self.node.status()['layout'])
        self.assertIn('unverified', self.node.state['message'])
        with self.assertRaisesRegex(ValueError, 'Another laptop calibration service'):
            CalibrationJournal(self.state_path)
        self.node.close()
        self.node = self.create()
        self.assertTrue(self.node.dirty)
        self.assertIsNone(self.node.status()['layout'])
        self.assertEqual(len(self.node.boards), 4)
        self.node.run(self.measure)
        self.ble.save.side_effect = lambda positions, guard, on_write: on_write()
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.assertFalse(self.node.dirty)

    def test_expired_preview_cannot_write(self):
        self.node.run(self.measure)
        self.node.preview_at -= 121
        self.node.run(dict(action='apply', token=self.node.state['token']))
        self.ble.save.assert_not_called()


class LaptopBLETest(unittest.IsolatedAsyncioTestCase):
    async def test_scan_accepts_service_data_only_and_skips_tags_and_failed_connections(self):
        from calibration.anchor_calibration import INFO, MODE, PAN
        ble = PansBLE.__new__(PansBLE)
        ble.progress = mock.Mock()
        ble.devices = {'stale': object()}
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
        self.assertNotIn('stale', ble.devices)
        self.assertIn('failed', ble.scan_warnings[0])
        client.disconnect.assert_awaited_once()

    async def test_pans_ranging_and_save_use_notifications_and_verify_readback_and_cleanup(self):
        from calibration.anchor_calibration import POSITION, encode_position
        ble = PansBLE.__new__(PansBLE)
        ble.progress = mock.Mock()
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
        connections, active = [], set()
        @asynccontextmanager
        async def connect(board, guard):
            guard()
            connections.append(board['id'])
            active.add(board['id'])
            try:
                yield clients[board['id']]
            finally:
                active.remove(board['id'])
        ble.connect = connect
        boards = [dict(board, pan=pan) for board, pan in zip(BOARDS, [0, None, 0, None])]
        rows = await ble._measure(boards, lambda: None)
        self.assertEqual(rows, ROWS)
        self.assertEqual(connections, [board['id'] for board in boards])
        self.assertEqual(ble.verify.await_count, 4)
        with self.assertRaisesRegex(ValueError, 'same PAN'):
            await ble._measure([dict(board, pan=pan) for board, pan in zip(BOARDS, [0, None, 1, None])], lambda: None)
        clients['0001'].stop_notify.assert_awaited_with(LOCATION)
        positions = solve_layout(BOARDS, rows)['positions']
        on_write = mock.Mock(side_effect=lambda: self.assertEqual(len(active), 4))
        connections.clear()
        await ble._save(positions, on_write, lambda: None)
        self.assertEqual(connections, [board['id'] for board in positions])
        self.assertFalse(active)
        self.assertEqual(on_write.call_count, 4)
        for board in positions:
            clients[board['id']].write_gatt_char.assert_any_await(POSITION, encode_position(board), response=True)
        saved['0001']['x'] = 10
        with self.assertRaisesRegex(ValueError, 'failed verification'):
            await ble._save(positions, on_write, lambda: None)
        self.assertFalse(active)
        clients['0001'].start_notify.side_effect = ValueError('cancelled')
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            await ble._measure(BOARDS, lambda: None)
        self.assertEqual(set(modes.values()), {b'\x02'})

        # All anchors must pass preflight before the first coordinate write.
        ble.verify.side_effect = [None, ValueError('Anchor identity changed')]
        on_write.reset_mock()
        with self.assertRaisesRegex(ValueError, 'identity changed'):
            await ble._save(positions, on_write, lambda: None)
        on_write.assert_not_called()
        self.assertFalse(active)


class LaptopCancellationTest(unittest.TestCase):
    def test_cancellation_stops_inflight_ble_and_waits_for_cleanup(self):
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
            if started.is_set(): raise ValueError('Calibration cancelled')
        with self.assertRaisesRegex(ValueError, 'Calibration cancelled'):
            ble.call('scan', guard)
        self.assertTrue(cleaned.is_set())
