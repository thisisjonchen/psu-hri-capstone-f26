"""Reads the position of a DWM1001-DEV tag (stock PANS firmware) over USB serial.

Written for Python 3.5 (no f-strings or annotations), so it runs on Ubuntu 16.04's stock Python.
"""
import threading
import time
from collections import namedtuple

import serial

Position = namedtuple('Position', 'x y z quality timestamp')


def parse_lep_line(line, timestamp=None):
    """Parse a PANS `lep` line ("POS,x,y,z,qf") into a Position, or return None."""
    i = line.find('POS,')
    if i < 0:
        return None
    parts = line[i:].strip().split(',')
    if len(parts) < 5:
        return None
    try:
        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
        qf = int(parts[4])
    except ValueError:
        return None
    return Position(x, y, z, qf, time.monotonic() if timestamp is None else timestamp)


class UWBLocator(object):
    """Holds the latest world-frame (meters) position of this drone's tag.

    A daemon thread reads the serial stream; `position` and `is_fresh()` never block.
    """

    def __init__(self, port='/dev/ttyACM0', baud=115200, min_quality=50, stale_after=0.5):
        self.port = port
        self.baud = baud
        self.min_quality = min_quality
        self.stale_after = stale_after
        self._serial = None
        self._thread = None
        self._running = False
        self._latest = None

    @property
    def position(self):
        return self._latest

    def is_fresh(self):
        p = self._latest
        return p is not None and time.monotonic() - p.timestamp <= self.stale_after

    def read_z(self, uwb_z):
        # TODO: replace with ToF height. Passes the UWB z through until then.
        return uwb_z

    def start(self):
        self._serial = serial.Serial(self.port, self.baud, timeout=1)
        self._serial.write(b'\r\r')  # double Enter switches PANS into shell mode
        time.sleep(1)
        self._serial.reset_input_buffer()
        # `lep` toggles streaming, so if it was already on the first send turns it off.
        for _ in range(2):
            self._serial.write(b'lep\r')
            if self._wait_for_pos(2.0):
                break
        else:
            self._serial.close()
            raise RuntimeError('No POS data from {}; is the tag configured?'.format(self.port))
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.write(b'lep\r')
            finally:
                self._serial.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    def _wait_for_pos(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if 'POS,' in self._serial.readline().decode('ascii', 'ignore'):
                return True
        return False

    def _run(self):
        while self._running:
            try:
                raw = self._serial.readline()
            except serial.SerialException as e:
                print('UWBLocator: serial error on {}: {}'.format(self.port, e))
                self._running = False
                return
            p = parse_lep_line(raw.decode('ascii', 'ignore'))
            if p is not None and p.quality >= self.min_quality:
                self._latest = p._replace(z=self.read_z(p.z))


if __name__ == '__main__':
    import sys
    with UWBLocator(sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0') as loc:
        try:
            while True:
                print(loc.position, 'fresh' if loc.is_fresh() else 'STALE')
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
