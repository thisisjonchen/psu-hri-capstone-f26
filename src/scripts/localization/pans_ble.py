"""DWM1001 PANS BLE auto-positioning, API guide sections 7.1 and 7.2."""
from __future__ import division

import struct
import uuid
import binascii
import sys
from contextlib import contextmanager

from localization.anchor_calibration import (SERVICE, MODE, PAN, DATA_MODE, LOCATION,
                                             INFO, POSITION, CLUSTER, decode_info,
                                             decode_location, encode_position)
from localization.uwb_locator import monotonic


class BluetoothTimeout(ValueError):
    pass


class PansBLE(object):
    def __init__(self, interface=0, timeout=30.):
        # Imported on demand so missing Bluetooth support produces a dashboard error.
        from bluepy import btle
        self.btle, self.interface, self.timeout = btle, int(interface), float(timeout)

    def bounded(self, base, guard):
        interface = self.interface
        class Bounded(base):
            check = staticmethod(guard)

            def _writeCmd(self, command):
                self.operation = command.split()[0]
                return base._writeCmd(self, command)

            def _waitResp(self, wantType, timeout=None):
                if timeout is not None:
                    self.check()
                    return base._waitResp(self, wantType, timeout)
                deadline = monotonic() + 5
                while monotonic() < deadline:
                    self.check()
                    response = base._waitResp(self, wantType, 0.25)
                    if response is not None:
                        return response
                raise BluetoothTimeout('Bluetooth timeout on hci%d, %s, command %s (waiting for %s); '
                                       'check BLE connections and range from the Pi'
                                       % (interface, getattr(self, 'context', 'scan'),
                                          getattr(self, 'operation', 'unknown'), wantType))
        return Bounded

    def cleanup(self, helper, stop):
        failed = sys.exc_info()[0] is not None
        cleanup_error = None
        helper.check = lambda: None
        try:
            # Bluepy stops its helper itself after some startup/management failures.
            if helper._helper is not None:
                stop()
        except Exception as error:
            cleanup_error = error
        finally:
            try:
                helper._stopHelper()
            except Exception as error:
                if cleanup_error is None:
                    cleanup_error = error
        if cleanup_error is not None and not failed:
            raise cleanup_error

    @contextmanager
    def connect(self, board, guard):
        preferred = board.get('addr_type', 'random')
        if preferred not in ('public', 'random'):
            raise ValueError('Unknown Bluetooth address type for %s' % board['ble'])
        address_types = [preferred, 'random' if preferred == 'public' else 'public']
        for index, address_type in enumerate(address_types):
            guard()
            peripheral = self.bounded(self.btle.Peripheral, guard)()
            peripheral.context = '%s (%s)' % (board['ble'], address_type)
            try:
                peripheral.connect(board['ble'], addrType=address_type, iface=self.interface)
            except Exception as error:
                self.cleanup(peripheral, peripheral.disconnect)
                if not isinstance(error, (self.btle.BTLEDisconnectError, BluetoothTimeout)) or index == len(address_types) - 1:
                    raise
            else:
                board['addr_type'] = address_type
                break
        try:
            peripheral.setMTU(128)  # Four-anchor range notifications exceed the default ATT payload.
            guard()
            yield peripheral
        finally:
            self.cleanup(peripheral, peripheral.disconnect)

    def characteristic(self, peripheral, uuid):
        items = peripheral.getCharacteristics(uuid=uuid)
        if len(items) != 1:
            raise ValueError('Missing PANS BLE characteristic %s; verify firmware' % uuid)
        return items[0]

    def inspect(self, peripheral):
        read = lambda uuid: self.characteristic(peripheral, uuid).read()
        return decode_info(read(INFO), read(MODE), read(PAN))

    def scan(self, guard):
        self.scan_warnings = []
        scanner = self.bounded(self.btle.Scanner, guard)(self.interface)
        try:
            try:
                scanner.start()
            except Exception as error:
                raise ValueError('Cannot start Bluetooth scan on hci%d: %s. Check adapter power and bluepy-helper permissions.'
                                 % (self.interface, error))
            deadline = monotonic() + 5
            while monotonic() < deadline:
                guard()
                scanner.process(0.25)
            entries = scanner.getDevices()
        finally:
            self.cleanup(scanner, scanner.stop)
        boards = []
        service_ids = (SERVICE.replace('-', ''), binascii.hexlify(uuid.UUID(SERVICE).bytes[::-1]).decode('ascii'))
        for entry in entries:
            guard()
            advertised = entry.getScanData()
            if not any(any(service in value.lower().replace('-', '') for service in service_ids)
                       for kind, _, value in advertised if kind in (0x06, 0x07, 0x21)):
                continue
            # PANS presence broadcasts include the anchor/tag bit after the 16-byte UUID.
            modes = [int(value[32:34], 16) for kind, _, raw in advertised
                     for value in [raw.lower().replace('-', '')] if kind == 0x21 and len(value) >= 36
                     and any(value.startswith(service) for service in service_ids)]
            if modes and not any(mode & 0x80 for mode in modes):
                continue
            if not entry.connectable:
                self.scan_warnings.append('%s is advertising but not connectable; close other BLE connections and retry' % entry.addr)
                continue
            board = {'ble': entry.addr, 'addr_type': entry.addrType}
            try:
                with self.connect(board, guard) as peripheral:
                    board.update(self.inspect(peripheral))
            except (self.btle.BTLEException, BluetoothTimeout) as error:
                guard()  # Cancellation or a lost flight interlock still aborts discovery.
                self.scan_warnings.append('%s: %s. Close other BLE connections, check range, and retry' % (entry.addr, error))
                continue
            if board['anchor']:
                boards.append(board)
        return sorted(boards, key=lambda b: b['id'])

    def verify(self, peripheral, board):
        actual = self.inspect(peripheral)
        if any(actual[key] != board[key] for key in ('address', 'id', 'pan', 'initiator')):
            raise ValueError('Anchor identity, role or network changed; scan again')
        if not actual['anchor'] or not actual['active']:
            raise ValueError('Anchor %s must have active UWB enabled' % board['id'])

    def measure(self, boards, guard):
        initiators = [b for b in boards if b['initiator']]
        if len(initiators) != 1 or len(set(b['pan'] for b in boards)) != 1:
            raise ValueError('Select one initiator and three anchors on the same PAN')
        initiator = initiators[0]
        for board in boards:
            with self.connect(board, guard) as peripheral:
                self.verify(peripheral, board)
        rows = {}
        with self.connect(initiator, guard) as peripheral:
            self.verify(peripheral, initiator)
            cluster = self.characteristic(peripheral, CLUSTER).read()
            if len(cluster) != 5 or struct.unpack('<B', cluster[:1])[0] != 0:
                raise ValueError('Selected initiator is not the network initiator (cluster seat must be 0)')
            data_mode = self.characteristic(peripheral, DATA_MODE)
            previous = data_mode.read()
            location = self.characteristic(peripheral, LOCATION)
            descriptors = location.getDescriptors(forUUID=0x2902)
            if len(descriptors) != 1:
                raise ValueError('PANS location notifications unavailable')
            cccd = descriptors[0]
            received = []
            btle = self.btle

            class Delegate(btle.DefaultDelegate):
                def handleNotification(self, handle, data):
                    if handle == location.getHandle():
                        received.append(data)

            peripheral.withDelegate(Delegate())
            try:
                data_mode.write(b'\x01', withResponse=True)
                guard()
                # PANS starts network inter-anchor ranging when the real initiator's
                # distance notifications are enabled. Serial lec does not trigger it.
                cccd.write(b'\x01\x00', withResponse=True)
                deadline = monotonic() + self.timeout
                while monotonic() < deadline:
                    guard()
                    peripheral.waitForNotifications(0.25)
                    while received:
                        payload = received.pop(0)
                        if not payload:
                            continue
                        _, ranges = decode_location(payload)
                        if all(b['id'] in ranges for b in boards if b != initiator):
                            rows[initiator['id']] = ranges
                            break
                    if initiator['id'] in rows:
                        break
                if initiator['id'] not in rows:
                    raise ValueError('No complete initiator ranges within %.0f s; check anchor line of sight' % self.timeout)
            finally:
                peripheral.check = lambda: None  # Cleanup still runs after cancellation.
                try:
                    cccd.write(b'\x00\x00', withResponse=True)
                finally:
                    data_mode.write(previous, withResponse=True)
        for board in boards:
            if board == initiator:
                continue
            with self.connect(board, guard) as peripheral:
                self.verify(peripheral, board)
                mode = self.characteristic(peripheral, DATA_MODE)
                previous = mode.read()
                try:
                    mode.write(b'\x01', withResponse=True)
                    guard()
                    _, rows[board['id']] = decode_location(self.characteristic(peripheral, LOCATION).read())
                finally:
                    peripheral.check = lambda: None
                    mode.write(previous, withResponse=True)
        return rows

    def read_position(self, peripheral):
        mode = self.characteristic(peripheral, DATA_MODE)
        previous = mode.read()
        try:
            mode.write(b'\x00', withResponse=True)
            position, _ = decode_location(self.characteristic(peripheral, LOCATION).read())
            if position is None or position['quality'] == 0:
                raise ValueError('PANS did not return the saved position')
            return position
        finally:
            peripheral.check = lambda: None
            mode.write(previous, withResponse=True)

    def save(self, positions, guard, on_write):
        for board in positions:
            with self.connect(board, guard) as peripheral:
                self.verify(peripheral, board)
        for board in positions:
            with self.connect(board, guard) as peripheral:
                self.verify(peripheral, board)
                guard()
                on_write()  # A failed/unknown write outcome must keep flight inhibited.
                self.characteristic(peripheral, POSITION).write(encode_position(board), withResponse=True)
                guard()
                saved = self.read_position(peripheral)
                if any(abs(saved[axis] - board[axis]) > 0.02 for axis in ('x', 'y', 'z')):
                    raise ValueError('Saved coordinates failed verification on %s' % board['id'])
