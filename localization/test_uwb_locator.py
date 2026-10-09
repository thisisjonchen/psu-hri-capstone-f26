import io
import sys
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

import serial

from localization import configure_board
from localization.uwb_locator import UWBLocator, parse_lep_line


class FakeSerial(object):
    """Stands in for serial.Serial. `lep` toggles the stream like PANS does."""

    def __init__(self, lines=(), streaming=False, fail_when_empty=False):
        self.lines = list(lines)
        self.streaming = streaming
        self.fail_when_empty = fail_when_empty
        self.writes = []
        self.is_open = True
        self.in_waiting = 0

    def __call__(self, *args, **kwargs):  # lets the instance replace the serial.Serial class
        return self

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def write(self, data):
        self.writes.append(data)
        if data == b'lep\r':
            self.streaming = not self.streaming

    def readline(self):
        if self.streaming and self.lines:
            return self.lines.pop(0)
        if self.fail_when_empty:
            raise serial.SerialException('device unplugged')
        time.sleep(0.01)
        return b''

    def read(self, n=1):
        return b''

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


def wait_until(cond, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return False


class ParseTest(unittest.TestCase):
    def test_plain(self):
        p = parse_lep_line('POS,1.23,4.56,0.78,92\r\n', timestamp=0)
        self.assertEqual((p.x, p.y, p.z, p.quality), (1.23, 4.56, 0.78, 92))

    def test_prompt_prefix(self):
        self.assertEqual(parse_lep_line('dwm> POS,1.0,2.0,0.0,80').x, 1.0)

    def test_malformed(self):
        for line in ['', 'dwm> ', 'POS,1.0,2.0', 'POS,a,b,c,d']:
            self.assertIsNone(parse_lep_line(line))


class FreshnessTest(unittest.TestCase):
    def test_fresh_and_stale(self):
        loc = UWBLocator(stale_after=0.5)
        self.assertFalse(loc.is_fresh())
        loc._latest = parse_lep_line('POS,1,2,0,90')
        self.assertTrue(loc.is_fresh())
        loc._latest = loc._latest._replace(timestamp=time.monotonic() - 1)
        self.assertFalse(loc.is_fresh())


class LocatorSerialTest(unittest.TestCase):
    # The first POS line is consumed by start() to confirm the stream is up.
    def run_locator(self, fake, **kwargs):
        with mock.patch('serial.Serial', fake):
            loc = UWBLocator(**kwargs)
            loc.start()
        return loc

    def test_start_reads_positions(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n', b'dwm> POS,1.5,2.5,0.0,90\r\n'])
        loc = self.run_locator(fake)
        self.assertTrue(wait_until(lambda: loc.position is not None))
        self.assertEqual((loc.position.x, loc.position.y), (1.5, 2.5))
        self.assertTrue(loc.is_fresh())
        self.assertEqual(fake.writes[:2], [b'\r\r', b'lep\r'])
        loc.stop()

    def test_retries_lep_if_stream_was_already_on(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n', b'POS,1,1,0,90\r\n'], streaming=True)
        loc = self.run_locator(fake)
        self.assertEqual(fake.writes.count(b'lep\r'), 2)
        self.assertTrue(wait_until(lambda: loc.position is not None))
        loc.stop()

    def test_raises_when_no_data(self):
        fake = FakeSerial()
        with mock.patch('serial.Serial', fake):
            with self.assertRaises(RuntimeError):
                UWBLocator().start()
        self.assertFalse(fake.is_open)

    def test_quality_filter(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n', b'POS,1,1,0,90\r\n', b'POS,9,9,0,10\r\n'])
        loc = self.run_locator(fake, min_quality=50)
        self.assertTrue(wait_until(lambda: not fake.lines))
        time.sleep(0.05)
        self.assertEqual(loc.position.x, 1.0)
        loc.stop()

    def test_read_z_hook_is_applied(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n', b'POS,1,1,0,90\r\n'])
        with mock.patch.object(UWBLocator, 'read_z', lambda self, z: 1.25):
            loc = self.run_locator(fake)
            self.assertTrue(wait_until(lambda: loc.position is not None))
        self.assertEqual(loc.position.z, 1.25)
        loc.stop()

    def test_serial_error_goes_stale(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n', b'POS,1,1,0,90\r\n'], fail_when_empty=True)
        with redirect_stdout(io.StringIO()):
            loc = self.run_locator(fake, stale_after=0.1)
            self.assertTrue(wait_until(lambda: not loc._thread.is_alive()))
        self.assertTrue(wait_until(lambda: not loc.is_fresh()))
        loc.stop()

    def test_stop_turns_stream_off_and_closes(self):
        fake = FakeSerial([b'POS,0,0,0,90\r\n'])
        loc = self.run_locator(fake)
        loc.stop()
        self.assertFalse(loc._thread.is_alive())
        self.assertEqual(fake.writes[-1], b'lep\r')
        self.assertFalse(fake.streaming)
        self.assertFalse(fake.is_open)


class ConfigureBoardTest(unittest.TestCase):
    def configure(self, *args):
        fake = FakeSerial()
        argv = ['configure_board.py', '--port', 'fake'] + list(args)
        with mock.patch('serial.Serial', fake), mock.patch.object(sys, 'argv', argv), \
                mock.patch('time.sleep'), redirect_stdout(io.StringIO()):
            configure_board.main()
        return fake.writes

    def test_initiator_anchor(self):
        self.assertEqual(self.configure('--anchor', 'A0'),
                         [b'\r\r', b'nis 0x1234\r', b'nmi\r', b'\r\r', b'aps 0 0 0\r', b'si\r'])

    def test_anchor_position_in_mm(self):
        writes = self.configure('--anchor', 'A2')
        self.assertIn(b'nma\r', writes)
        self.assertIn(b'aps 3000 6000 0\r', writes)

    def test_tag(self):
        self.assertEqual(self.configure('--tag'),
                         [b'\r\r', b'nis 0x1234\r', b'nmt\r', b'\r\r', b'aurs 1 1\r', b'si\r'])


if __name__ == '__main__':
    unittest.main()
