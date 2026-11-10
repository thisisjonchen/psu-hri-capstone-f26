#!/usr/bin/env python3
"""Warm FC reboot on the Pi host, called under pals-start.sh's maintenance lock."""
import os
import re
from pathlib import Path
import struct
import subprocess
import sys

from flash_fc import check_disarmed, msp, port_unused, require, run, stop_fc, usb_device, wait_for


# Observe the FC's calibrated gyro, before the EKF estimates its own bias.
GYRO_CHECK = '''
from __future__ import print_function
import math
import time
import rospy
from pidrone_pkg.msg import IntegratedImu, Mode
try:
    from time import monotonic
except ImportError:
    from monotonic import monotonic
rospy.init_node('warm_reboot_gyro_check', anonymous=True)
samples = []
modes = []
started = []
received = []
def receive(message):
    if not message.valid or message.integration_time_us <= 0:
        return
    q = list(message.delta_rotation)
    if not all(not math.isnan(x) and not math.isinf(x) for x in q):
        return
    received[:] = [monotonic()]
    if not started:
        started.append(received[0])
        return
    if q[3] < 0:
        q = [-x for x in q]
    norm = math.sqrt(sum(x*x for x in q[:3]))
    scale = (2*math.atan2(norm, q[3])/norm if norm > 1.e-12 else 2.) * 180/math.pi
    samples.append((message.integration_time_us*1.e-6, [x*scale for x in q[:3]]))
rospy.Subscriber('/pidrone/imu_integrated', IntegratedImu, receive, queue_size=100)
rospy.Subscriber('/pidrone/mode', Mode, lambda m: modes.append((m.mode, monotonic())))
deadline = monotonic() + 20
while monotonic() < deadline and not rospy.is_shutdown():
    if started and monotonic() - started[0] >= 10:
        break
    time.sleep(.1)
seconds = sum(dt for dt, rotation in samples)
now = monotonic()
if (seconds < 8 or not received or now - received[0] > .5 or not modes
        or now - modes[-1][1] > .5 or any(mode != 'DISARMED' for mode, stamp in modes)):
    raise RuntimeError('Could not verify stationary gyro: incomplete/stale IMU or mode data, or not disarmed.')
mean = [sum(rotation[axis] for dt, rotation in samples)/seconds for axis in range(3)]
print('Gyro mean (roll/pitch/yaw): %.3f / %.3f / %.3f deg/s; check that each is near zero.' % tuple(mean))
'''


def calibration_complete(connection):
    status = msp(connection, 101)
    require(len(status) >= 16, 'Incomplete Betaflight status.')
    offset = 17 + (status[15] & 15)
    require(len(status) >= offset + 4, 'Missing calibration status.')
    flags = struct.unpack_from('<I', status, offset)[0]
    return not flags & (1 << 12)  # CALIBRATING in Betaflight MSP API 1.48.


def reboot_controller(port):
    """Send one reboot, then verify a new USB enumeration and calibrated FC."""
    import serial

    port_unused(port)
    usb = usb_device(port)
    generation = (usb / 'devnum').read_text()
    with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
        check_disarmed(connection)
        require(msp(connection, 2) == b'BTFL' and msp(connection, 1) == bytes([0, 1, 48]),
                'Expected Betaflight MSP API 1.48; no reboot sent.')
        # MSP_REBOOT resets USB immediately after preparing its reply. Do not
        # read that disappearing connection or resend the reboot on a timeout.
        packet = b'$M<' + bytes([0, 68, 68])
        require(connection.write(packet) == len(packet), 'FC reboot command was not fully written.')

    def reconnected():
        try:
            return (port.exists() and (usb / 'devnum').read_text() != generation
                    and usb_device(port) == usb)
        except (OSError, RuntimeError):
            # sysfs and the by-id link can briefly disagree during enumeration.
            return False

    wait_for(reconnected, 'FC reboot not confirmed: the pinned USB device did not reconnect.')

    def verified():
        if not reconnected():
            return False
        port_unused(port)
        try:
            with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
                check_disarmed(connection)
                return calibration_complete(connection)
        except (serial.SerialException, OSError):
            # Only retry read-only verification after USB returns. Armed state,
            # invalid MSP replies and conflicting serial owners still fail.
            return False

    wait_for(verified, 'FC reconnected, but serial verification or gyro calibration did not finish; '
             'keep the drone still and check its USB connection.')


def stop_all_nodes(port):
    running = run(['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f', '[f]light_controller_node.py'])
    require(running.returncode in (0, 1), 'Could not check the FC process: ' + running.stdout)
    if running.returncode == 0:
        mode = run(['docker', 'exec', 'pidrone_pkg', 'bash', '-ic',
                    'timeout 5 rostopic echo -n1 /pidrone/mode'])
        require(mode.returncode == 0 and re.search(r'^mode: [\"\']?DISARMED[\"\']?\s*$', mode.stdout, re.M),
                'Cannot restart until the running FC reports DISARMED.')
    else:
        import serial
        port_unused(port)
        with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
            check_disarmed(connection)

    # Keep the master available while nodes release sensors and serial ports.
    try:
        graceful = run(['docker', 'exec', 'pidrone_pkg', 'bash', '-c', r'''
            ulimit -Sn 1024 || exit 1
            for window in routine PID SE Vision TOF rosbridge web_vid_serv AprilTags UWB TagMap FC; do
                screen -S pidrone -p "$window" -X stuff '^C' >/dev/null 2>&1 || :
            done
            for _ in $(seq 40); do
                pgrep -f '[r]oslaunch .*\.launch|[f]light_controller_node.py|[p]id_controller.py|[f]usion_localization_node|[e]kf_localization_node|[r]aspicam_node|[v]l53l1x_node|[t]of_node.py|[u]wb_localization_node|[w]eb_video_server|[a]priltag|PALS/[r]outines/' >/dev/null
                status=$?
                [[ $status == 1 ]] && exit 0
                [[ $status == 0 ]] || exit 1
                sleep 0.25
            done
            exit 1
        '''], timeout=20)
        clean = graceful.returncode == 0
    except subprocess.TimeoutExpired:
        clean = False
    if not clean:
        print('Some nodes did not exit cleanly; stopping the container.')
    stopped = run(['docker', 'stop', '--time', '5', 'pidrone_pkg'])
    require(stopped.returncode == 0, 'Could not stop the drone container: ' + stopped.stdout)
    state = run(['docker', 'inspect', '--format', '{{.State.Running}}', 'pidrone_pkg'])
    require(state.returncode == 0 and state.stdout.strip() == 'false',
            'Drone container is not confirmed stopped; replacement nodes were not started.')


def warm_reboot(restart=False):
    require(os.geteuid() == 0, 'Run through pals-start.sh restart.')
    home = Path('/home/duckie')
    pins = home / 'catkin_ws/pals.ports'
    values = dict(line.split('=', 1) for line in pins.read_text().splitlines() if '=' in line)
    port = Path(values.get('PALS_FC_ID', ''))
    require(port.parent == Path('/dev/serial/by-id') and 'SPEEDYBEEF405V3' in port.name
            and port.exists(), 'Pinned SpeedyBee flight controller is unavailable.')
    environment = home / 'catkin_ws/pals.env'
    if restart:
        stop_all_nodes(port)
    else:
        require(environment.is_file(), 'Missing resolved serial-port configuration.')
        stop_fc()
        localization = ['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f',
                        '(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]']
        running = run(localization)
        require(running.returncode in (0, 1), 'Could not check localization: ' + running.stdout)
        if running.returncode == 0:
            result = run(['docker', 'exec', 'pidrone_pkg', 'screen', '-S', 'pidrone', '-p', 'SE',
                          '-X', 'stuff', '\x03'])
            require(result.returncode == 0, 'Could not stop localization: ' + result.stdout)
            wait_for(lambda: run(localization).returncode == 1, 'Localization did not stop.')
    reboot_controller(port)
    # USB enumeration can change the tty name after a reboot.
    lines = environment.read_text().splitlines() if environment.is_file() else []
    lines = [line for line in lines if not line.startswith('PALS_FC_PORT=')]
    lines.append('PALS_FC_PORT=' + str(port.resolve()))
    environment.write_text('\n'.join(lines) + '\n')
    # The launcher starts FC communication once, with the rest of the nodes.


def check_gyro():
    result = subprocess.run(['docker', 'exec', '-i', 'pidrone_pkg', 'bash', '-ic', 'timeout 25 python -'],
                            input=GYRO_CHECK, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True, timeout=30)
    require(result.returncode == 0, 'FC restarted, but gyro verification failed: ' + result.stdout)
    print(result.stdout.strip().splitlines()[-1])


if __name__ == '__main__':
    try:
        require(len(sys.argv) == 2 and sys.argv[1] in ('restart', 'reboot', 'check'),
                'Usage: warm_reboot_fc.py restart|reboot|check (called by pals-start.sh).')
        if sys.argv[1] in ('restart', 'reboot'):
            warm_reboot(restart=sys.argv[1] == 'restart')
        else:
            check_gyro()
    except Exception as error:
        print('Warm reboot incomplete: %s Keep the drone disarmed; check the FC and SE terminals before restarting.' % error)
        sys.exit(1)
