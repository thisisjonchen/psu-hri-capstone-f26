"""Nonblocking DWM1001/PANS serial reader, compatible with ROS Kinetic Python 2.

Position timestamps are host monotonic receipt times, not UWB measurement times.
"""
from __future__ import print_function

import math
import re
import threading
from collections import namedtuple

import serial

try:
    from time import monotonic
except ImportError:  # Python 2.7 on the DD24's ROS Kinetic image
    from monotonic import monotonic

Position = namedtuple('Position', 'x y z quality timestamp')
Anchor = namedtuple('Anchor', 'id x y z distance')
AnchorSample = namedtuple('AnchorSample', 'anchors timestamp')


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


def parse_lep_line(line, timestamp=None):
    """Parse a PANS POS CSV record, rejecting invalid coordinates and quality."""
    i = line.find('POS,')
    if i < 0:
        return None
    parts = line[i:].strip().split(',')
    if len(parts) != 5:
        return None
    try:
        x, y, z = [float(v) for v in parts[1:4]]
        quality = int(parts[4])
    except ValueError:
        return None
    if not all(finite(v) for v in (x, y, z)) or not 0 <= quality <= 100:
        return None
    return Position(x, y, z, quality, monotonic() if timestamp is None else timestamp)


def parse_lec_line(line, timestamp=None):
    """Parse one complete DIST/AN/POS record; AN slot numbers are not identities.

    Returns (AnchorSample, Position or None). Coordinates and distances in LEC
    shell output are meters. A partial ranging set is valid; a malformed set is
    rejected as a whole so it cannot silently change the discovered room.
    """
    start = line.find('DIST,')
    if start < 0:
        return None
    parts = line[start:].strip().split(',')
    try:
        count = int(parts[1])
        if not 0 <= count <= 4 or len(parts) < 2 + 6 * count:
            return None
        anchors, ids, slots = [], set(), set()
        for index in range(count):
            slot, address, x, y, z, distance = parts[2 + 6 * index:8 + 6 * index]
            if not slot.startswith('AN') or not slot[2:].isdigit() or slot in slots:
                return None
            address = address.upper()
            if address.startswith('0X'):
                address = address[2:]
            if not 1 <= len(address) <= 4 or any(c not in '0123456789ABCDEF' for c in address):
                return None
            address = address.zfill(4)
            x, y, z, distance = [float(v) for v in (x, y, z, distance)]
            if address in ids or not all(finite(v) for v in (x, y, z, distance)) or distance < 0:
                return None
            ids.add(address)
            slots.add(slot)
            anchors.append(Anchor(address, x, y, z, distance))
        timestamp = monotonic() if timestamp is None else timestamp
        tail = parts[2 + 6 * count:]
        position = parse_lep_line(','.join(tail), timestamp) if tail else None
        if tail and (len(tail) != 5 or tail[0] != 'POS' or position is None):
            return None
        return AnchorSample(tuple(sorted(anchors, key=lambda a: a.id)), timestamp), position
    except (ValueError, IndexError):
        return None


class UWBLocator(object):
    """Keep the latest accepted fix; serial I/O and shell setup run in a worker.

    Resetting the tag once on connection establishes a known stream state. Sending
    lec again after a timeout would turn a slow-starting stream OFF.
    """
    def __init__(self, port=None, baud=115200, min_quality=50, stale_after=0.5):
        if not 0 <= min_quality <= 100 or not finite(stale_after) or stale_after <= 0:
            raise ValueError('Invalid minimum quality or stale timeout')
        self.port, self.baud = port, baud
        self.min_quality, self.stale_after = min_quality, stale_after
        self._serial = None
        self._thread = None
        self._stop = threading.Event()
        self._latest = None
        self._latest_anchors = None
        self._connected = False
        self._in_shell = False
        self.error = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive() and not self._stop.is_set()

    @property
    def connected(self):
        return self.running and self._connected

    @property
    def position(self):
        return self._latest if self.connected else None

    @property
    def anchor_sample(self):
        return self._latest_anchors if self.connected else None

    def is_fresh(self):
        p = self.position
        return p is not None and 0 <= monotonic() - p.timestamp <= self.stale_after

    def start(self):
        if self.running:
            return
        if not self.port:
            raise ValueError('An explicit UWB serial port is required')
        self.stop()
        self._stop.clear()
        self.error = None
        self._in_shell = False
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=1)
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def stop(self):
        self._stop.set()
        if self._thread is not None and self._thread.ident is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                # Closing interrupts a blocked read/write; don't start a second worker.
                if self._serial is not None:
                    self._serial.close()
                self._thread.join(timeout=2)
                if self._thread.is_alive():
                    raise RuntimeError('UWB serial worker did not stop')
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._thread = None
        self._latest = None
        self._latest_anchors = None
        self._connected = False

    def _enter_shell(self):
        self._serial.write(b'\r\r')
        if self._stop.wait(1):
            return False
        response = self._serial.read(self._serial.in_waiting or 1).decode('ascii', 'ignore')
        if 'dwm>' not in response:
            raise RuntimeError('No PANS shell prompt; verify the UWB tag serial port')
        self._in_shell = True
        self._serial.reset_input_buffer()
        return True

    def _run(self):
        try:
            if not self._enter_shell():
                return
            self._serial.write(b'reset\r')
            self._in_shell = False
            if self._stop.wait(3) or not self._enter_shell():
                return
            if not self._set_update_rate():
                return
            self._serial.write(b'lec\r')
            self._connected = True
            pending = b''
            while not self._stop.is_set():
                raw = self._serial.readline()
                pending += raw
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    text = line.decode('ascii', 'ignore')
                    if 'DIST,' in text:
                        record = parse_lec_line(text)
                        if record is None:
                            continue
                        self._latest_anchors, p = record
                    else:
                        p = parse_lep_line(text)
                    if p is not None and p.quality >= self.min_quality:
                        self._latest = p
                if len(pending) > 2048:
                    pending = b''
        except (serial.SerialException, OSError, ValueError, RuntimeError) as error:
            self.error = str(error)
        finally:
            self._connected = False
            self._latest = None
            self._latest_anchors = None
            try:
                if self._serial.is_open and self._in_shell:
                    self._serial.write(b'quit\r')
            except (serial.SerialException, OSError):
                pass
            finally:
                self._serial.close()

    def _set_update_rate(self):
        # PANS intervals are in 100 ms units: 10 Hz even when stationary.
        self._serial.write(b'aurs 1 1\r')
        deadline = monotonic() + 2
        response = ''
        while not self._stop.is_set() and monotonic() < deadline:
            response = (response + self._serial.readline().decode('ascii', 'ignore'))[-2048:]
            result = re.search(r'err code:\s*(-?\d+)\s*\r?\n', response)
            if result:
                if int(result.group(1)) != 0:
                    raise RuntimeError('PANS rejected 10 Hz update rate: error {}'.format(result.group(1)))
                return True
        if self._stop.is_set():
            return False
        raise RuntimeError('No PANS response while setting 10 Hz update rate')

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


if __name__ == '__main__':
    import sys
    import time
    if len(sys.argv) != 2:
        sys.exit('Usage: python -m localization.uwb_locator <UWB serial port>')
    with UWBLocator(sys.argv[1]) as locator:
        try:
            while locator.running:
                print(locator.position, 'fresh' if locator.is_fresh() else 'STALE')
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
