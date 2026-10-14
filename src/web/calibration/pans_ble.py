"""Laptop Bluetooth transport for the DWM1001 PANS calibration API."""
import asyncio
import logging
import sys
import threading
from contextlib import AsyncExitStack, asynccontextmanager

from calibration.anchor_calibration import (SERVICE, MODE, PAN, DATA_MODE, LOCATION,
                                             INFO, POSITION, CLUSTER, decode_info,
                                             decode_location, encode_position)


class PansBLE:
    def __init__(self, timeout=30, progress=None):
        self.timeout = timeout
        self.progress = progress or (lambda message: None)
        self.devices = {}
        self.scan_warnings = []
        # CoreBluetooth objects must stay on one event loop across requests.
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    def call(self, action, guard, *args):
        async def guarded():
            guard()
            task = asyncio.create_task(getattr(self, '_' + action)(*args, guard))
            try:
                while not task.done():
                    await asyncio.wait([task], timeout=0.2)
                    guard()
                return await task
            finally:
                if not task.done():
                    task.cancel()
                try:
                    await task
                except (Exception, asyncio.CancelledError):
                    pass
        return asyncio.run_coroutine_threadsafe(guarded(), self.loop).result()

    async def io(self, operation):
        try:
            return await asyncio.wait_for(operation, timeout=10)
        except asyncio.TimeoutError:
            raise TimeoutError('Laptop Bluetooth operation timed out; check anchor range and other BLE connections')

    async def cleanup(self, *operations):
        failed = sys.exc_info()[0] is not None
        first = None
        for operation in operations:
            try:
                await self.io(operation())
            except Exception as error:
                first = first or error
                logging.warning('Bluetooth cleanup: %s', error)
        if first is not None and not failed:
            raise first

    @asynccontextmanager
    async def connect(self, board, guard):
        from bleak import BleakClient, BleakScanner
        guard()
        device = self.devices.get(board['ble'])
        if device is None:
            device = await self.io(BleakScanner.find_device_by_address(
                board['ble'], timeout=5))
        if device is None:
            raise ValueError('Anchor %s is not visible from the laptop; scan again' % board.get('id', board['ble']))
        self.devices[board['ble']] = device
        client = BleakClient(device, timeout=10)
        try:
            await self.io(client.connect())
            guard()
            yield client
        finally:
            await self.cleanup(client.disconnect)

    async def inspect(self, client):
        values = [bytes(await self.io(client.read_gatt_char(key))) for key in (INFO, MODE, PAN)]
        try:
            return decode_info(*values)
        except ValueError:
            logging.warning('Unexpected PANS response: info=%s mode=%s PAN=%s',
                            *(value[:64].hex() for value in values))
            raise

    async def _scan(self, guard):
        from bleak import BleakScanner, BleakError
        self.scan_warnings = []
        self.devices.clear()
        self.progress('Discovering Bluetooth anchors...')
        scanner = BleakScanner()
        try:
            await self.io(scanner.start())
            for _ in range(25):
                guard()
                await asyncio.sleep(0.2)
            entries = scanner.discovered_devices_and_advertisement_data
        finally:
            await self.cleanup(scanner.stop)
        boards = []
        for device, advertisement in entries.values():
            guard()
            service_data = {key.lower(): value for key, value in advertisement.service_data.items()}
            if SERVICE not in service_data and SERVICE not in [key.lower() for key in advertisement.service_uuids]:
                continue
            presence = service_data.get(SERVICE)
            if presence and not presence[0] & 0x80:
                continue
            self.devices[device.address] = device
            board = {'ble': device.address}
            self.progress('Checking Bluetooth anchor %s; %d anchors found...' %
                          (device.address, len(boards)))
            try:
                async with self.connect(board, guard) as client:
                    board.update(await self.inspect(client))
            except (BleakError, TimeoutError, ValueError) as error:
                guard()
                self.scan_warnings.append('%s: %s' % (device.address, error))
                continue
            if board['anchor']:
                boards.append(board)
        return sorted(boards, key=lambda board: board['id'])

    async def verify(self, client, board):
        actual = await self.inspect(client)
        if any(actual[key] != board[key] for key in ('address', 'id', 'pan', 'initiator')):
            raise ValueError('Anchor identity, role or network changed; scan again')
        if not actual['anchor'] or not actual['active']:
            raise ValueError('Anchor %s must have active UWB enabled' % board['id'])

    async def _measure(self, boards, guard):
        initiators = [board for board in boards if board['initiator']]
        known_pans = {board['pan'] for board in boards if board['pan'] is not None}
        # An empty BLE PAN value is unknown, not zero. Live ranges must still
        # connect all six selected pairs before the coordinator offers a save.
        if len(initiators) != 1 or len(known_pans) > 1:
            raise ValueError('Select one initiator and three anchors on the same PAN')
        initiator = initiators[0]
        rows = {}
        # Verify each board in the same connection used to read its ranges.
        # Measurement does not write coordinates; save retains its full preflight.
        self.progress('Connecting to initiator %s (anchor 1 of 4)...' % initiator['id'])
        async with self.connect(initiator, guard) as client:
            await self.verify(client, initiator)
            cluster = await self.io(client.read_gatt_char(CLUSTER))
            if len(cluster) != 5 or cluster[0] != 0:
                raise ValueError('Selected initiator is not the network initiator (cluster seat must be 0)')
            previous = await self.io(client.read_gatt_char(DATA_MODE))
            received = asyncio.Queue()
            try:
                await self.io(client.write_gatt_char(DATA_MODE, b'\x01', response=True))
                guard()
                # Enabling the initiator's distance notifications starts PANS ranging.
                await self.io(client.start_notify(LOCATION, lambda _, data: received.put_nowait(bytes(data))))
                self.progress('Ranging from initiator %s; waiting up to %d seconds...' %
                              (initiator['id'], self.timeout))
                deadline = self.loop.time() + self.timeout
                while self.loop.time() < deadline:
                    guard()
                    try:
                        payload = await asyncio.wait_for(received.get(), max(0.001, min(0.2, deadline - self.loop.time())))
                    except asyncio.TimeoutError:
                        continue
                    if not payload:
                        continue
                    _, ranges = decode_location(payload)
                    if all(board['id'] in ranges for board in boards if board != initiator):
                        rows[initiator['id']] = ranges
                        break
                if initiator['id'] not in rows:
                    raise ValueError('No complete initiator ranges; check anchor line of sight')
            finally:
                await self.cleanup(lambda: client.stop_notify(LOCATION),
                                   lambda: client.write_gatt_char(DATA_MODE, previous, response=True))
        for board in boards:
            if board == initiator:
                continue
            self.progress('Reading distances from %s (anchor %d of 4)...' %
                          (board['id'], len(rows) + 1))
            async with self.connect(board, guard) as client:
                await self.verify(client, board)
                _, rows[board['id']] = decode_location(await self.read_location(client, b'\x01', guard))
        return rows

    async def read_location(self, client, mode, guard):
        previous = await self.io(client.read_gatt_char(DATA_MODE))
        try:
            await self.io(client.write_gatt_char(DATA_MODE, mode, response=True))
            guard()
            return bytes(await self.io(client.read_gatt_char(LOCATION)))
        finally:
            await self.cleanup(lambda: client.write_gatt_char(DATA_MODE, previous, response=True))

    async def _save(self, positions, on_write, guard):
        # Keep the four verified connections until readback finishes. No position
        # is written until every board passes preflight; all exits disconnect.
        async with AsyncExitStack() as connections:
            clients = []
            for index, board in enumerate(positions, 1):
                self.progress('Checking %s before saving (anchor %d of 4)...' % (board['id'], index))
                client = await connections.enter_async_context(self.connect(board, guard))
                await self.verify(client, board)
                clients.append(client)
            for index, (board, client) in enumerate(zip(positions, clients), 1):
                self.progress('Saving and verifying %s (anchor %d of 4)...' % (board['id'], index))
                await self.verify(client, board)
                guard()
                on_write()
                await self.io(client.write_gatt_char(POSITION, encode_position(board), response=True))
                guard()
                saved, _ = decode_location(await self.read_location(client, b'\x00', guard))
                if saved is None or not saved['quality'] or any(
                        abs(saved[axis] - board[axis]) > 0.02 for axis in ('x', 'y', 'z')):
                    raise ValueError('Saved coordinates failed verification on %s' % board['id'])

    def close(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()
        self.loop.close()
