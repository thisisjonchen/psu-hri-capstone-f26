#!/usr/bin/env python
"""Wait for ROS 1's XML-RPC API before starting a node (Python 2/3)."""
from __future__ import print_function

import os
import signal
import socket
import sys
import time

try:
    import xmlrpc.client as xmlrpc
except ImportError:
    import xmlrpclib as xmlrpc


class MasterTimeout(Exception):
    pass


def timeout_handler(signum, frame):
    raise MasterTimeout()


def main():
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    try:
        timeout = int(os.environ.get('PIDRONE_ROS_TIMEOUT', '60'))
        if timeout <= 0:
            raise ValueError()
    except ValueError:
        print('PIDRONE_ROS_TIMEOUT must be a positive integer (seconds).', file=sys.stderr)
        return 2

    print('Waiting up to {}s for ROS master at {} ...'.format(timeout, uri))
    sys.stdout.flush()
    socket.setdefaulttimeout(1.0)
    signal.signal(signal.SIGALRM, timeout_handler)
    # Bound the entire wait, including slow DNS resolution and stalled requests.
    signal.alarm(timeout)
    last_error = 'master not ready'
    try:
        master = xmlrpc.ServerProxy(uri)
        while True:
            try:
                code, message, pid = master.getPid('/pidrone_screen_startup')
                if code == 1:
                    print('ROS master is ready.')
                    return 0
                last_error = message
            except (IOError, xmlrpc.Error, ValueError) as error:
                last_error = str(error)
            time.sleep(0.5)
    except MasterTimeout:
        print('ROS master at {} did not become ready within {}s: {}. '
              'Check the rcore window and ROS_MASTER_URI.'.format(
                  uri, timeout, last_error), file=sys.stderr)
        return 1
    except (ValueError, OSError) as error:
        print('Cannot contact ROS master at {}: {}'.format(uri, error), file=sys.stderr)
        return 1
    finally:
        signal.alarm(0)


if __name__ == '__main__':
    sys.exit(main())
