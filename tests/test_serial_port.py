"""Serial ownership checks and real kernel locking on a disposable pseudo-TTY."""
import errno
import os
import pty
import stat
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

import serial

from localization.serial_port import ExclusiveSerial, busy_error, check_port_free
from localization.uwb_locator import UWBLocator


class SerialOwnershipTest(unittest.TestCase):
    def setUp(self):
        master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        os.close(slave)
        self.addCleanup(os.close, master)

    def test_kernel_blocks_plain_serial_open_and_releases_on_close_or_failure(self):
        with ExclusiveSerial(self.port, timeout=0.1):
            with self.assertRaises(serial.SerialException) as caught:
                ExclusiveSerial(self.port, timeout=0.1)
            self.assertEqual(caught.exception.errno, errno.EBUSY)
            # Linux's TIOCEXCL blocks noncooperating readers too. Darwin's
            # pseudo-TTY open path does not enforce it, so use the advisory check.
            if sys.platform.startswith('linux') and os.geteuid() != 0:
                with self.assertRaises(serial.SerialException) as caught:
                    serial.Serial(self.port, timeout=0.1)
                self.assertEqual(caught.exception.errno, errno.EBUSY)
        with serial.Serial(self.port, timeout=0.1):
            pass

        with mock.patch.object(serial.Serial, '_reconfigure_port',
                               side_effect=serial.SerialException('configuration failed')):
            with self.assertRaisesRegex(serial.SerialException, 'configuration failed'):
                ExclusiveSerial(self.port, timeout=0.1)
        with serial.Serial(self.port, timeout=0.1):
            pass

    def test_existing_screen_lock_and_visible_reader_are_rejected(self):
        with mock.patch('localization.serial_port.open', mock.mock_open(read_data=b'        69\n'), create=True), \
                mock.patch('localization.serial_port.os.kill'):
            with self.assertRaisesRegex(serial.SerialException, 'already in use.*PID 69'):
                check_port_free(self.port)

        # A stale lock does not leave the port permanently blocked.
        with mock.patch('localization.serial_port.open', mock.mock_open(read_data=b'        69\n'), create=True), \
                mock.patch('localization.serial_port.os.kill', side_effect=OSError(errno.ESRCH, 'exited')), \
                mock.patch('localization.serial_port.glob.glob', return_value=[]):
            check_port_free(self.port)

        # Match the device number, so a reader opening an alias is also found.
        device = SimpleNamespace(st_mode=stat.S_IFCHR, st_rdev=123)
        with mock.patch('localization.serial_port.open', side_effect=IOError(errno.ENOENT, 'no lock'), create=True), \
                mock.patch('localization.serial_port.glob.glob', return_value=['/proc/868/fd/16']), \
                mock.patch('localization.serial_port.os.stat', return_value=device):
            with self.assertRaisesRegex(serial.SerialException, 'already in use.*PID 868'):
                check_port_free(self.port)

    def test_busy_start_never_configures_port_or_starts_worker(self):
        loc = UWBLocator(self.port)
        with mock.patch('localization.serial_port.check_port_free', side_effect=busy_error(self.port, 69)), \
                mock.patch.object(serial.Serial, '_reconfigure_port') as configure:
            with self.assertRaisesRegex(serial.SerialException, 'Close the serial Screen window'):
                loc.start()
            configure.assert_not_called()
        self.assertFalse(loc.running)
        self.assertIsNone(loc.position)
        self.assertEqual(loc.reader_state, 'error')
        self.assertIn('already in use', loc.error)
