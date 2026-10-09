"""One-time setup of a DWM1001-DEV board over USB using PANS shell commands.

Usage:
    python3 configure_board.py --port /dev/ttyACM0 --anchor A0
    python3 configure_board.py --port /dev/ttyACM0 --tag

On macOS the port is /dev/cu.usbmodemXXXX. Command names can differ between PANS versions;
run `help` in the board shell to check. See SETUP.md.
"""
import argparse
import json
import os
import time

import serial


def enter_shell(ser):
    ser.write(b'\r\r')
    time.sleep(1)
    ser.reset_input_buffer()


def send(ser, cmd, wait=0.5):
    ser.write(cmd.encode('ascii') + b'\r')
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode('ascii', 'ignore')
    print('> {}\n{}'.format(cmd, out.strip()))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--config', default=os.path.join(here, 'anchors.json'))
    role = ap.add_mutually_exclusive_group(required=True)
    role.add_argument('--anchor', help='anchor name from the config, e.g. A0')
    role.add_argument('--tag', action='store_true')
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    with serial.Serial(args.port, 115200, timeout=1) as ser:
        enter_shell(ser)
        send(ser, 'nis {}'.format(cfg['network_id']))

        # Write the whole role config so nothing depends on factory defaults: UWB active (2),
        # LEDs and BLE on, encryption and UWB firmware update off.
        if args.tag:
            # meas_mode=TWR, stationary detection off, low power off, location engine on
            send(ser, 'acts 0 0 0 1 0 1 1 2 0')
        else:
            anchor = cfg['anchors'][args.anchor]
            # initiator, bridge off
            send(ser, 'acas {} 0 0 1 1 2 0'.format(int(anchor['initiator'])))
        send(ser, 'reset', wait=3)
        enter_shell(ser)  # the reset drops the board out of shell mode

        if args.tag:
            rate = cfg['update_rate_100ms']
            send(ser, 'aurs {} {}'.format(rate, rate))
        else:
            # aps takes millimeters
            send(ser, 'aps {} {} {}'.format(*[int(round(v * 1000)) for v in anchor['pos']]))
        send(ser, 'si', wait=1)


if __name__ == '__main__':
    main()
