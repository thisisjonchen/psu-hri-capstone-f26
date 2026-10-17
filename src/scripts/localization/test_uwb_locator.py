"""Serial lifecycle and validation tests; no ROS or boards required."""
import time
import unittest
from collections import deque
from unittest import mock

import serial

from localization.uwb_locator import UWBLocator, parse_lep_line, parse_lec_line

LEC = 'DIST,4,AN0,1151,3,6,2.25,6.44,AN1,0CA8,0,6,2.25,6.50,AN2,111C,3,0,2.25,3.24,AN3,1150,0,0,2.25,3.19,POS,1,2,9,90\r\n'


class FakeSerial(object):
    def __init__(self, lines=(), streaming=False):
        self.lines = deque(lines)
        self.streaming = streaming
        self.writes = []
        self.byte_writes = []
        self.command = b''
        self.is_open = True
        self.fail = False
        self.in_waiting = 0
        self.prompt = b'dwm> '
        self.rate_reply = b'err code: 0\r\n'
        self.get_rate_reply = None
        self.update_rate = (1, 100)
        self.apply_after = 1
        self.rate_writes = 0
        self.info_reply = b'dwm> '
        self.replies = deque()

    def write(self, data):
        self.byte_writes.append(data)
        self.command += data
        if not self.command.endswith(b'\r'):
            return
        # The double-enter shell handshake is one logical command for assertions.
        if self.command == b'\r' and (not self.writes or self.writes[-1] == b'reset\r'):
            return
        data, self.command = self.command, b''
        self.writes.append(data)
        if data == b'reset\r' or data == b'quit\r':
            self.streaming = False
        if data == b'lec\r':
            self.streaming = not self.streaming
        if data == b'aurs 1 1\r':
            self.rate_writes += 1
            if self.apply_after is not None and self.rate_writes >= self.apply_after:
                self.update_rate = (1, 1)
            self.replies.append(self.rate_reply)
        if data == b'aurg\r':
            reply = self.get_rate_reply
            if reply is None:
                reply = ('aurg: nom=%d stat=%d\r\n' % self.update_rate).encode('ascii')
            self.replies.append(reply)
        if data == b'si\r':
            self.replies.append(self.info_reply)

    def readline(self):
        if self.fail:
            raise serial.SerialException('device unplugged')
        if self.replies:
            return self.replies.popleft()
        if self.streaming and self.lines:
            return self.lines.popleft()
        time.sleep(0.001)
        return b''

    def read(self, count):
        return self.prompt

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


def wait_until(condition):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.001)
    return False


class LocatorTest(unittest.TestCase):
    def setUp(self):
        pacing = mock.patch('localization.uwb_locator.sleep')
        self.sleep = pacing.start()
        self.addCleanup(pacing.stop)

    def start_locator(self, fake):
        loc = UWBLocator(port='fake')
        # Skip real hardware boot delays, retaining real reader threads and serial errors.
        loc._stop.wait = lambda timeout: loc._stop.is_set()
        self.addCleanup(loc.stop)
        with mock.patch('localization.uwb_locator.ExclusiveSerial', return_value=fake):
            loc.start()
        self.assertTrue(wait_until(lambda: loc.connected or not loc.running))
        return loc

    def test_parser_handles_prompt_and_rejects_nonfinite_and_bad_quality(self):
        p = parse_lep_line('dwm> POS,1.23,4.56,0.78,92\r\n', timestamp=100)
        self.assertEqual(tuple(p), (1.23, 4.56, 0.78, 92, 100))
        for line in ['', 'POS,1,2', 'POS,a,b,c,d', 'POS,nan,2,0,90',
                     'POS,1,inf,0,90', 'POS,1,2,-inf,90', 'POS,1,2,0,101',
                     'POS,1,2,0,-1', 'POS,1,2,0,90,unexpected']:
            self.assertIsNone(parse_lep_line(line), line)

    def test_lec_reports_hardware_ids_and_rejects_malformed_anchor_sets(self):
        sample, p = parse_lec_line('dwm> ' + LEC, timestamp=100)
        self.assertEqual([a.id for a in sample.anchors], ['0CA8', '111C', '1150', '1151'])
        self.assertEqual(sample.anchors[0].y, 6)
        self.assertEqual(sample.anchors[0].distance, 6.50)
        self.assertEqual(sample.timestamp, p.timestamp)
        self.assertEqual(p.x, 1)
        for line in [LEC.replace('DIST,4', 'DIST,5'), LEC.replace('1150', '1151'),
                     LEC.replace('1151', 'ZZZZ'), LEC.replace(',3,6,', ',nan,6,'),
                     LEC.replace(',2.25,6.44', ',2.25,-1'), LEC.replace('AN3', 'AN0'),
                     LEC.replace(',POS,', ',bad,POS,'), 'DIST,2,AN0,1151,0,0',
                     LEC.replace(',9,90', ',9,101')]:
            self.assertIsNone(parse_lec_line(line), line)
        partial, p = parse_lec_line('DIST,1,AN2,ca8,0,6,2.25,6.5', timestamp=101)
        self.assertEqual(partial.anchors[0].id, '0CA8')
        self.assertIsNone(p)
        self.assertEqual(parse_lec_line('DIST,0', timestamp=102)[0].anchors, ())

    def test_delayed_first_fix_does_not_toggle_stream_off_or_discard_first_fix(self):
        fake = FakeSerial(streaming=True)
        loc = self.start_locator(fake)
        self.assertIsNone(loc.position)
        fake.lines.append(LEC.encode('ascii'))
        self.assertTrue(wait_until(lambda: loc.position is not None))
        self.assertEqual((loc.position.x, loc.position.y), (1, 2))
        self.assertEqual(fake.writes.count(b'lec\r'), 1)
        self.assertEqual(len(loc.anchor_sample.anchors), 4)
        self.assertIn(b'reset\r', fake.writes)
        self.assertLess(fake.writes.index(b'reset\r'), fake.writes.index(b'aurs 1 1\r'))
        self.assertLess(fake.writes.index(b'aurs 1 1\r'), fake.writes.index(b'lec\r'))
        self.assertTrue(all(len(chunk) == 1 for chunk in fake.byte_writes))
        self.assertEqual(self.sleep.call_count, len(fake.byte_writes))
        self.assertTrue(all(call == mock.call(0.02) for call in self.sleep.call_args_list))

    def test_prolonged_serial_silence_closes_reader_for_reconnect(self):
        for lines in ([], [LEC.encode('ascii')]):
            with self.subTest(had_position=bool(lines)):
                fake = FakeSerial(lines)
                with mock.patch('localization.uwb_locator.SERIAL_SILENCE_TIMEOUT', 0.05):
                    loc = self.start_locator(fake)
                    self.assertTrue(wait_until(lambda: not loc.running))
                self.assertIn('No UWB serial data', loc.error)
                self.assertFalse(fake.is_open)
                self.assertIsNone(loc.position)
                self.assertEqual(fake.writes.count(b'lec\r'), 1)
        # Unconfirmed slow stationary updates must get time to arrive.
        loc.update_rate = (1, 100)
        self.assertGreaterEqual(loc.serial_timeout, 30)

    def test_rate_reply_variations_and_missing_ack_do_not_block_valid_stream(self):
        for reply, readback, info, apply_after, verified, writes in (
                (b'err code: 0\r', None, b'', 1, True, 1),
                (b'dwm> ', None, b'', 2, True, 2),
                (b'dwm> ', b'err code: 0, upd rate: 1, 1(stat)\r\n', b'', None, True, 0),
                (b'dwm> ', b'aurg: nom=1 stat=1\r\n', b'', None, True, 0),
                (b'dwm> ', b'dwm> ', b'cfg: upd_rate_norm=1 upd_rate_stat=1 label=DW1900\r\n', None, True, 0),
                (b'err code: 0\r\n', None, b'', None, False, 2),
                (b'err code: -1\r\n', None, b'', None, False, 2),
                (b'dwm> ', b'dwm> ', b'dwm> ', None, False, 2)):
            with self.subTest(reply=reply, readback=readback, apply_after=apply_after):
                fake = FakeSerial([LEC.encode('ascii')])
                fake.rate_reply, fake.get_rate_reply, fake.info_reply = reply, readback, info
                fake.apply_after = apply_after
                loc = self.start_locator(fake)
                self.assertTrue(wait_until(lambda: loc.position is not None))
                self.assertTrue(loc.is_fresh())
                self.assertEqual(loc.rate_verified, verified)
                self.assertEqual(bool(loc.rate_warning), not verified)
                self.assertIsNone(loc.error)
                self.assertEqual(fake.rate_writes, writes)
                self.assertEqual(fake.writes.count(b'lec\r'), 1)
                if readback is None:
                    self.assertEqual(loc.update_rate, (1, 1) if verified else (1, 100))
                if not verified and readback is None:
                    self.assertIn('nom=1 stat=100', loc.rate_warning)
                loc.stop()

        fake = FakeSerial()
        loc = UWBLocator(port='fake')
        loc._serial = fake
        loc._stop.wait = lambda timeout: False
        with mock.patch.object(loc, '_shell_reply', return_value='') as reply:
            self.assertTrue(loc._set_update_rate())
        self.assertEqual(sum(call.args[0] == b'aurs 1 1\r' for call in reply.call_args_list), 2)
        self.assertFalse(loc.rate_verified)
        self.assertIn('10 Hz unconfirmed', loc.rate_warning)
        with mock.patch('localization.uwb_locator.monotonic', side_effect=[0, 3]):
            self.assertEqual(loc._shell_reply(b'aurg\r', 'nom='), '')

        # A reconnect must repair a reverted tag, even after an earlier verified run.
        fake = FakeSerial([LEC.encode('ascii')])
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: loc.position is not None))
        loc.stop()
        replacement = FakeSerial([LEC.encode('ascii')])
        with mock.patch('localization.uwb_locator.ExclusiveSerial', return_value=replacement):
            loc.start()
        self.assertTrue(wait_until(lambda: loc.position is not None))
        self.assertEqual(replacement.rate_writes, 1)
        self.assertTrue(loc.rate_verified)

    def test_partial_lines_and_quality_filter(self):
        raw = LEC.encode('ascii')
        low_quality = LEC.replace('POS,1,2,9,90', 'POS,9,9,9,10')
        fake = FakeSerial([raw[:30], raw[30:], b'POS,unsupported\r\n', low_quality.encode('ascii')])
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: loc.last_quality == 10))
        self.assertEqual(loc.position.x, 1)
        self.assertEqual(len(loc.anchor_sample.anchors), 4)
        self.assertTrue(loc.is_fresh())
        details = dict(loc.diagnostics(time.monotonic()))
        self.assertEqual(details['reader_state'], 'streaming')
        self.assertEqual(details['last_reported_quality'], '10')
        self.assertEqual(details['last_serial_line'], low_quality.strip())
        self.assertEqual(details['rejected_records'], '1')
        self.assertNotEqual(details['serial_age_s'], 'none')
        self.assertNotEqual(details['record_age_s'], 'none')
        with mock.patch('localization.uwb_locator.monotonic', return_value=loc.position.timestamp + 1):
            self.assertFalse(loc.is_fresh())

    def test_position_requires_four_anchors_in_the_same_record(self):
        partial = LEC.replace('DIST,4', 'DIST,3').replace('AN3,1150,0,0,2.25,3.19,', '')
        partial = partial.replace('POS,1,2,9,90', 'POS,8,8,9,99')
        fake = FakeSerial([partial.encode('ascii')])
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: loc.last_quality == 99))
        self.assertIsNone(loc.position)
        self.assertEqual(len(loc.anchor_sample.anchors), 3)

        fake.lines.append(LEC.encode('ascii'))
        self.assertTrue(wait_until(lambda: loc.position is not None))
        accepted = loc.position
        # A cached complete layout cannot qualify a standalone POS message.
        fake.lines.append(b'POS,9,9,9,98\r\n')
        self.assertTrue(wait_until(lambda: loc.last_quality == 98))
        self.assertIs(loc.position, accepted)
        fake.lines.append(partial.encode('ascii'))
        self.assertTrue(wait_until(lambda: loc.last_quality == 99))
        self.assertEqual(len(loc.anchor_sample.anchors), 3)
        self.assertIs(loc.position, accepted)
        with mock.patch('localization.uwb_locator.monotonic', return_value=accepted.timestamp + 1):
            self.assertFalse(loc.is_fresh())

        fake.lines.append(LEC.replace('POS,1,2,9,90', 'POS,3,4,9,90').encode('ascii'))
        self.assertTrue(wait_until(lambda: loc.position.x == 3))
        self.assertGreater(loc.position.timestamp, accepted.timestamp)
        self.assertEqual(len(loc.anchor_sample.anchors), 4)
        self.assertTrue(loc.is_fresh())

    def test_stop_clears_fix_and_closes_port(self):
        fake = FakeSerial([LEC.encode('ascii')])
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: loc.position is not None))
        loc.stop()
        self.assertFalse(loc.running)
        self.assertFalse(loc.is_fresh())
        self.assertIsNone(loc.position)
        self.assertIsNone(loc.anchor_sample)
        self.assertFalse(fake.streaming)
        self.assertFalse(fake.is_open)

    def test_serial_failure_invalidates_fix_and_can_reconnect(self):
        fake = FakeSerial([LEC.encode('ascii')])
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: loc.position is not None))
        fake.fail = True
        self.assertTrue(wait_until(lambda: not loc.running))
        self.assertFalse(loc.is_fresh())
        self.assertIsNone(loc.anchor_sample)
        self.assertEqual(loc.error, 'device unplugged')
        self.assertFalse(fake.is_open)
        replacement = FakeSerial([LEC.replace('POS,1,2,9,90', 'POS,3,4,0,90').encode('ascii')])
        with mock.patch('localization.uwb_locator.ExclusiveSerial', return_value=replacement):
            loc.start()
        self.assertTrue(wait_until(lambda: loc.position is not None))
        self.assertEqual(loc.position.x, 3)
        self.assertIsNone(loc.error)

    def test_no_port_never_opens_a_guessed_device(self):
        with mock.patch('localization.uwb_locator.ExclusiveSerial') as opened:
            with self.assertRaises(ValueError):
                UWBLocator().start()
            opened.assert_not_called()

    def test_non_pans_device_is_closed_without_reset_or_stream_commands(self):
        fake = FakeSerial()
        fake.prompt = b'not a DWM1001'
        loc = self.start_locator(fake)
        self.assertTrue(wait_until(lambda: not loc.running))
        self.assertEqual(fake.writes, [b'\r\r'])
        self.assertFalse(fake.is_open)
        self.assertIn('No PANS shell prompt', loc.error)


if __name__ == '__main__':
    unittest.main()
