"""Exclusive POSIX serial access for the Pi's UWB tag (Python 2 compatible)."""
import errno
import fcntl
import glob
import os
import stat
import termios

import serial


def busy_error(port, owner=None):
    detail = ' (PID %s)' % owner if owner else ''
    return serial.SerialException(errno.EBUSY,
        'UWB serial port %s is already in use%s. Close the serial Screen window '
        'or stop the other reader, then retry.' % (port, detail))


def check_port_free(port):
    # Screen may own the port without taking a kernel/advisory lock. Respect its
    # standard UUCP lock, including when its /proc descriptors are not readable.
    device = os.path.realpath(port)
    for directory in ('/var/lock', '/run/lock'):
        path = os.path.join(directory, 'LCK..' + os.path.basename(device))
        try:
            with open(path, 'rb') as lock:
                owner = int(lock.read(32).strip())
        except IOError as error:
            if error.errno == errno.ENOENT:
                continue
            raise busy_error(port)
        except ValueError:
            raise busy_error(port)
        if owner <= 0:
            raise busy_error(port)
        try:
            os.kill(owner, 0)
        except OSError as error:
            if error.errno == errno.ESRCH:
                continue  # Ignore stale lock files; never delete another program's lock.
        raise busy_error(port, owner)

    device_stat = os.stat(device)
    own_pid = str(os.getpid())
    for descriptor in glob.glob('/proc/[0-9]*/fd/*'):
        pid = descriptor.split('/')[2]
        if pid == own_pid:
            continue
        try:
            opened = os.stat(descriptor)
        except OSError:
            continue  # The process exited, or its descriptors are not visible.
        if stat.S_ISCHR(opened.st_mode) and opened.st_rdev == device_stat.st_rdev:
            raise busy_error(port, pid)


class ExclusiveSerial(serial.Serial):
    def open(self):
        # Check before pySerial opens/configures or flushes a terminal in use.
        check_port_free(self.portstr)
        try:
            super(ExclusiveSerial, self).open()
        except (OSError, serial.SerialException) as error:
            if error.errno in (errno.EBUSY, errno.EAGAIN):
                raise busy_error(self.portstr)
            raise

    def _reconfigure_port(self, force_update=False):
        if not getattr(self, '_owns_port', False):
            # Claim before pySerial changes terminal settings or discards input.
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except (IOError, OSError) as error:
                if error.errno in (errno.EAGAIN, errno.EACCES):
                    raise busy_error(self.portstr)
                raise
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self._owns_port = True
        try:
            # Catch an opener that raced the initial check before TIOCEXCL.
            check_port_free(self.portstr)
            super(ExclusiveSerial, self)._reconfigure_port(force_update)
        except Exception:
            self._release_port()
            raise

    def _release_port(self):
        if getattr(self, '_owns_port', False):
            self._owns_port = False
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except (IOError, OSError):
                pass

    def close(self):
        self._release_port()
        super(ExclusiveSerial, self).close()
