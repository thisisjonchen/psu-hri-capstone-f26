"""Nonblocking DWM1001/PANS serial reader, compatible with ROS Kinetic Python 2.

Position timestamps are host monotonic receipt times, not UWB measurement times.
"""
from __future__ import print_function

import math
import logging
import re
import threading
from collections import namedtuple
from time import sleep

import serial

from localization.serial_port import ExclusiveSerial

try:
    from time import monotonic
except ImportError:  # Python 2.7 on the DD24's ROS Kinetic image
    from monotonic import monotonic

Position = namedtuple('Position', 'x y z quality timestamp')
Anchor = namedtuple('Anchor', 'id x y z distance')
AnchorSample = namedtuple('AnchorSample', 'anchors timestamp')
SERIAL_SILENCE_TIMEOUT = 30.0


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


RATE_PATTERNS = (
    r'\bnom\s*=\s*(\d+)\s+stat\s*=\s*(\d+)(?=[\s,])',
    r'upd\s+rate:\s*(\d+)\s*,\s*(\d+)\s*\(stat\)',
    r'upd_rate_norm=(\d+)\s+upd_rate_stat=(\d+)(?=[\s,])')


def parse_update_rate(response):
    for pattern in RATE_PATTERNS:
        match = re.search(pattern, response, re.I)
        if match:
            normal, stationary = [int(v) for v in match.groups()]
            if normal > 0 and stationary >= normal:
                return normal, stationary
    return None


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
        self.reader_state = 'stopped'
        self.last_line = ''
        self.last_serial_at = None
        self.last_record_at = None
        self.last_quality = None
        self.rejected_records = 0
        self.rate_verified = False
        self.rate_warning = ''
        self.update_rate = None
        self.rate_response = ''

    def diagnostics(self, now):
        def age(timestamp):
            return 'none' if timestamp is None else '{:.3f}'.format(max(0, now - timestamp))
        rates = self.update_rate
        return [('reader_state', self.reader_state),
                ('serial_age_s', age(self.last_serial_at)),
                ('serial_timeout_s', str(self.serial_timeout)),
                ('record_age_s', age(self.last_record_at)),
                ('last_reported_quality', str(self.last_quality) if self.last_quality is not None else 'none'),
                ('min_quality', str(self.min_quality)),
                ('rejected_records', str(self.rejected_records)),
                ('update_rate_verified', str(self.rate_verified)),
                ('update_rate_warning', self.rate_warning),
                ('normal_interval_100ms', str(rates[0]) if rates else 'unknown'),
                ('stationary_interval_100ms', str(rates[1]) if rates else 'unknown'),
                ('update_rate_response', self.rate_response),
                ('last_serial_line', self.last_line)]

    @property
    def serial_timeout(self):
        # Allow three stationary intervals when a slower rate could not be changed.
        return max(SERIAL_SILENCE_TIMEOUT, self.update_rate[1] * 0.3 if self.update_rate else 0)

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
        self.reader_state = 'opening'
        self.last_line = ''
        self.last_serial_at = self.last_record_at = self.last_quality = None
        self.rejected_records = 0
        self.rate_verified = False
        self.rate_warning = ''
        self.update_rate = None
        self.rate_response = ''
        try:
            self._serial = ExclusiveSerial(self.port, self.baud, timeout=0.1, write_timeout=1)
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()
        except Exception as error:
            self.stop()
            self.error = str(error)
            self.reader_state = 'error'
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
        self.reader_state = 'stopped'

    def _write_command(self, command):
        # PANS shell input can drop characters sent as a single USB/UART burst.
        # Finish the short command even during shutdown, including its terminator.
        for index in range(len(command)):
            self._serial.write(command[index:index + 1])
            sleep(0.02)

    def _enter_shell(self):
        self.reader_state = 'entering shell'
        self._write_command(b'\r\r')
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
            self.reader_state = 'resetting tag'
            self._write_command(b'reset\r')
            self._in_shell = False
            if self._stop.wait(3) or not self._enter_shell():
                return
            if not self._set_update_rate():
                return
            self._write_command(b'lec\r')
            self._connected = True
            self.reader_state = 'streaming'
            stream_started = monotonic()
            pending = b''
            while not self._stop.is_set():
                raw = self._serial.readline()
                if raw:
                    self.last_serial_at = monotonic()
                else:
                    last_received = self.last_serial_at if self.last_serial_at is not None else stream_started
                    if monotonic() - last_received >= self.serial_timeout:
                        raise RuntimeError('No UWB serial data for %.1f seconds; reconnecting tag' % self.serial_timeout)
                pending += raw
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    text = line.decode('ascii', 'ignore')
                    self.last_line = text.strip()[-512:]
                    complete = False
                    if 'DIST,' in text:
                        record = parse_lec_line(text)
                        if record is None:
                            self.rejected_records += 1
                            continue
                        self._latest_anchors, p = record
                        self.last_record_at = self._latest_anchors.timestamp
                        complete = len(self._latest_anchors.anchors) == 4
                    else:
                        p = parse_lep_line(text)
                        if p is not None:
                            self.last_record_at = p.timestamp
                        elif 'POS,' in text:
                            self.rejected_records += 1
                    if p is not None:
                        self.last_quality = p.quality
                    # Require four anchors in this record, not a cached layout.
                    # Partial records leave the accepted fix and its age unchanged.
                    if complete and p is not None and p.quality >= self.min_quality:
                        self._latest = p
                if len(pending) > 2048:
                    pending = b''
        except (serial.SerialException, OSError, ValueError, RuntimeError) as error:
            self.error = str(error)
            self.reader_state = 'error'
        finally:
            self._connected = False
            if self.reader_state != 'error':
                self.reader_state = 'stopped'
            self._latest = None
            self._latest_anchors = None
            try:
                if self._serial.is_open and self._in_shell:
                    self._write_command(b'quit\r')
            except (serial.SerialException, OSError):
                pass
            finally:
                self._serial.close()

    def _set_update_rate(self):
        self.reader_state = 'setting update rate'
        self._read_update_rate()
        # Read first to avoid unnecessary flash writes; verify after every change.
        for attempt in range(2):
            if self._stop.is_set():
                return False
            if self.update_rate == (1, 1):
                break
            response = self._shell_reply(b'aurs 1 1\r', r'err\s+code:\s*-?\d+(?=[\r\n,])')
            self.rate_response = 'aurs: ' + response.strip()[-256:]
            if self._stop.wait(0.5):
                return False
            self._read_update_rate()
        self.rate_verified = self.update_rate == (1, 1)
        if not self.rate_verified and not self._stop.is_set():
            reason = ('PANS reports nom=%d stat=%d' % self.update_rate if self.update_rate else
                      'No recognized PANS update-rate readback')
            self.rate_warning = reason + '; 10 Hz unconfirmed, reading the tag at its configured rate'
            logging.warning('UWB: %s', self.rate_warning)
        return not self._stop.is_set()

    def _read_update_rate(self):
        completed = '|'.join('(?:%s)' % pattern for pattern in RATE_PATTERNS)
        response = self._shell_reply(b'aurg\r', completed)
        self.update_rate = parse_update_rate(response)
        if self.update_rate is None and not self._stop.is_set():
            response = self._shell_reply(b'si\r', completed)
            self.update_rate = parse_update_rate(response)
        self.rate_response = (self.rate_response + '\nreadback: ' + response.strip())[-512:]

    def _shell_reply(self, command, completed):
        if self._stop.is_set():
            return ''
        # Only called before lec starts: discard leftovers from the previous prompt.
        self._serial.reset_input_buffer()
        self._write_command(command)
        deadline = monotonic() + 2
        response = ''
        while not self._stop.is_set() and monotonic() < deadline:
            raw = self._serial.readline().decode('ascii', 'ignore')
            response = (response + raw)[-2048:]
            if 'dwm>' in response or (not raw and re.search(completed, response, re.I)):
                break
        return response

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
        if locator.error:
            print('UWB reader failed:', locator.error)
