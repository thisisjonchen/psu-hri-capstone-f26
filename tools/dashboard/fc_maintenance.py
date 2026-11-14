#!/usr/bin/env python3
"""Flight-controller firmware updates and warm restarts on the Pi host."""
import argparse
import datetime
import hashlib
import json
import os
import re
import shlex
import struct
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'tools' / 'migration' / 'firmware' / 'betaflight'
FLASH_BASE = 0x08000000
FLASH_SIZE = 1024 * 1024
CONFIG = slice(0x4000, 0x8000)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def bundle(folder):
    manifest = json.loads((folder / 'manifest.json').read_text())
    data = (folder / manifest['file']).read_bytes()
    require(digest(data) == manifest['sha256'], 'Bundled firmware checksum does not match.')
    require(data[:6] == b'DfuSe\x01' and data[10] == 1 and data[17] == 0,
            'Expected one DfuSe target on internal-flash alternate setting 0.')
    size, count = struct.unpack_from('<II', data, 277)
    require(count == 2 and len(data) == 285 + size + 16, 'Unexpected DfuSe layout.')
    elements, offset = [], 285
    for _ in range(count):
        address, length = struct.unpack_from('<II', data, offset)
        offset += 8
        contents = data[offset:offset + length]
        start = address - FLASH_BASE
        require(len(contents) == length and 0 <= start < start + length <= FLASH_SIZE,
                'Firmware element extends beyond internal flash.')
        require(start + length <= CONFIG.start or start >= CONFIG.stop,
                'Firmware would overwrite the configuration sector.')
        elements.append((start, contents))
        offset += length
    require([start for start, _ in elements] == [0, 0x8000], 'Unexpected firmware addresses.')
    return manifest, elements


def run(command, timeout=30):
    return subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          universal_newlines=True, timeout=timeout)


def wait_for(check, message, seconds=15):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if check():
            return
        time.sleep(0.2)
    raise RuntimeError(message)


def read_exact(port, length):
    data, until = b'', time.monotonic() + 3
    while len(data) < length and time.monotonic() < until:
        data += port.read(length - len(data))
    require(len(data) == length, 'Incomplete MSP reply.')
    return data


def msp(port, command):
    port.write(b'$M<' + bytes([0, command, command]))
    require(read_exact(port, 3) == b'$M>', 'Expected a successful MSP reply.')
    length, code = read_exact(port, 2)
    payload = read_exact(port, length)
    checksum = length ^ code
    for value in payload:
        checksum ^= value
    require(read_exact(port, 1)[0] == checksum and code == command, 'Invalid MSP reply.')
    return payload


def check_disarmed(port):
    boxes = msp(port, 119)
    require(0 in boxes and boxes.index(0) < 32, 'Cannot identify the ARM status bit.')
    status = msp(port, 101)
    require(len(status) >= 10, 'Incomplete flight-controller status.')
    flags = struct.unpack_from('<I', status, 6)[0]
    require(not flags & (1 << boxes.index(0)), 'Flight controller is armed; refusing to continue.')


def cli(port, command):
    port.write(command.encode('ascii') + (b'' if command == '#' else b'\r'))
    data, until = b'', time.monotonic() + 15
    while time.monotonic() < until:
        data += port.read(4096)
        if data.endswith(b'\n# '):
            text = data.decode('utf-8', 'replace')
            require('###ERROR' not in text and 'Unknown command' not in text, text)
            return text
    raise RuntimeError('CLI did not finish: ' + command)


def check_environment(text, manifest, installed=False):
    values = dict(re.findall(r'^([A-Z0-9_]+)=([^\r\n]+)', text, re.M))
    expected = {'BOARD_NAME': manifest['board'], 'MANUFACTURER_ID': 'SPBE',
                'FIRMWARE_VERSION': manifest['version'], 'MSP_API_VERSION': '1.48',
                'CONFIG_REVISION': manifest['config_revision'], 'CONFIG_STATE': 'CONFIGURED'}
    for key, value in expected.items():
        require(values.get(key) == value, '%s is %r; expected %s. No settings migration is supported.'
                % (key, values.get(key), value))
    revisions = ([manifest['firmware_revision']] if installed else
                 [image['revision'] for image in manifest['supported_images']])
    require(values.get('GIT_REVISION') in revisions, 'Installed firmware revision is unsupported.')


def config_commands(text):
    lines = [line.strip() for line in text.splitlines()]
    commands = [line for line in lines if line and not line.startswith('#')
                and line != 'diff all' and not line.startswith('set serial_update_rate_hz = ')]
    require('batch start' in commands and 'save' in commands
            and 'board_name SPEEDYBEEF405V3' in commands, 'Incomplete CLI configuration backup.')
    return commands


def firmware_matches(data, regions):
    return all(digest(data[region['address'] - FLASH_BASE:
                           region['address'] - FLASH_BASE + region['size']]) == region['sha256']
               for region in regions)


def verify_readback(before, after, elements):
    require(len(before) == len(after) == FLASH_SIZE, 'Full flash readback is incomplete.')
    require(before[CONFIG] == after[CONFIG], 'Configuration sector changed during flashing.')
    require(all(after[start:start + len(contents)] == contents for start, contents in elements),
            'Firmware readback does not match the bundled image.')


def usb_device(port):
    device = (Path('/sys/class/tty') / port.resolve().name / 'device').resolve()
    for parent in device.parents:
        if (parent / 'idVendor').exists():
            return parent
    raise RuntimeError('Cannot determine the flight controller USB path.')


def port_unused(port):
    target = port.resolve()
    for fd in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            require(fd.resolve() != target, 'Serial port is still owned by PID %s.' % fd.parts[2])
        except FileNotFoundError:
            pass


def stop_fc():
    running = run(['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f', '[f]light_controller_node.py'])
    require(running.returncode in (0, 1), 'Could not check the FC process: ' + running.stdout)
    if running.returncode == 1:
        return False
    mode = run(['docker', 'exec', 'pidrone_pkg', 'bash', '-ic',
                'timeout 5 rostopic echo -n1 /pidrone/mode'])
    require(mode.returncode == 0 and re.search(r'^mode: [\"\']?DISARMED[\"\']?\s*$', mode.stdout, re.M),
            'The running FC node must report DISARMED before it can be stopped.')
    require(run(['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f',
                 '(^|[ /])routines/[A-Za-z0-9_-]+[.]py([[:space:]]|$)']).returncode == 1,
            'Stop the running routine before flashing.')
    result = run(['docker', 'exec', 'pidrone_pkg', 'screen', '-S', 'pidrone', '-p', 'FC',
                  '-X', 'stuff', '\x03'])
    require(result.returncode == 0, 'Could not stop the FC screen window: ' + result.stdout)
    wait_for(lambda: run(['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f',
                          '[f]light_controller_node.py']).returncode == 1,
             'FC node did not stop; firmware was not written.')
    return True


def worker(folder, user_home):
    import serial

    require(os.geteuid() == 0, 'The Pi worker needs sudo for USB and serial ownership checks.')
    manifest, elements = bundle(folder)
    backup = user_home / 'catkin_ws' / 'firmware_backups' / folder.name
    backup.mkdir(parents=True)
    pins = user_home / 'catkin_ws' / 'pals.ports'
    values = dict(line.split('=', 1) for line in pins.read_text().splitlines() if '=' in line) if pins.exists() else {}
    ports = ([Path(values['PALS_FC_ID'])] if values.get('PALS_FC_ID') else
             list(Path('/dev/serial/by-id').glob('*Betaflight*SPEEDYBEEF405V3*')))
    require(len(ports) == 1 and ports[0].parent == Path('/dev/serial/by-id')
            and 'SPEEDYBEEF405V3' in ports[0].name and ports[0].exists(),
            'Expected one SpeedyBee F405 V3 flight controller; check PALS_FC_ID in ~/catkin_ws/pals.ports.')
    port = ports[0]
    usb = usb_device(port)
    print('Using %s on USB %s.' % (port, usb.name), flush=True)
    restart = stop_fc()
    port_unused(port)
    with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
        check_disarmed(connection)
        cli(connection, '#')
        environment = cli(connection, 'env')
        (backup / 'environment-before.txt').write_text(environment)
        check_environment(environment, manifest)
        before_diff = cli(connection, 'diff all')
        (backup / 'settings-before.txt').write_text(before_diff)
        config_commands(before_diff)
        print('Board and firmware match. Entering the USB bootloader.', flush=True)
        connection.write(b'bl rom\r')

    def in_dfu():
        return ((usb / 'idVendor').exists() and (usb / 'idProduct').exists()
                and (usb / 'idVendor').read_text().strip() == '0483'
                and (usb / 'idProduct').read_text().strip() == 'df11')

    wait_for(in_dfu, 'The FC did not appear in DFU mode on the same USB port.')
    serial_number = (usb / 'serial').read_text().strip()
    require(serial_number, 'DFU device has no serial number.')
    dfu = ['dfu-util', '-d', '0483:df11', '-S', serial_number, '-p', usb.name, '-a', '0']

    def transfer(name, arguments):
        result = run(dfu + arguments, timeout=120)
        (backup / (name + '.log')).write_text(result.stdout)
        require(result.returncode == 0, '%s failed; see %s\n%s' %
                (name, backup / (name + '.log'), result.stdout[-600:]))
        return result.stdout

    layout = transfer('device', ['-l'])
    require('@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg' in ' '.join(layout.split()),
            'Unexpected STM32 flash layout; no firmware was written.')
    print('Backing up the full device flash before writing.', flush=True)
    transfer('read-before', ['-s', '0x08000000:1048576', '-U', str(backup / 'flash-before.bin')])
    before = (backup / 'flash-before.bin').read_bytes()
    require(len(before) == FLASH_SIZE, 'Full flash backup is incomplete.')
    require(any(firmware_matches(before, image['regions']) for image in manifest['supported_images']),
            'Installed firmware bytes differ from the supported builds. Backup saved; no firmware was written.')
    if all(before[start:start + len(data)] == data for start, data in elements):
        print('USB firmware is already installed; checking the saved serial setting.', flush=True)
        after = before
    else:
        print('Flashing the firmware, preserving the configuration sector.', flush=True)
        transfer('flash', ['-D', str(folder / manifest['file'])])
        transfer('read-after', ['-s', '0x08000000:1048576', '-U', str(backup / 'flash-after.bin')])
        after = (backup / 'flash-after.bin').read_bytes()
    verify_readback(before, after, elements)
    result = {'firmware_revision': manifest['firmware_revision'], 'firmware_sha256': manifest['sha256'],
              'flash_before_sha256': digest(before), 'flash_after_sha256': digest(after),
              'configuration_sector_preserved': True, 'usb_path': usb.name, 'dfu_serial': serial_number}
    (backup / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    transfer('leave', ['-s', '0x08000000:32:leave', '-U', str(backup / 'vectors.bin')])

    def reconnected():
        time.sleep(1)
        wait_for(port.exists, 'The FC did not reconnect; backups are retained.')
        require(usb_device(port) == usb, 'The reconnected FC is on a different USB path.')
        port_unused(port)

    reconnected()
    print('Saving serial_update_rate_hz = 250.', flush=True)
    with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
        check_disarmed(connection)
        cli(connection, '#')
        check_environment(cli(connection, 'env'), manifest, installed=True)
        cli(connection, 'set serial_update_rate_hz = 250')
        connection.write(b'save\r')
    reconnected()
    with serial.Serial(str(port), 115200, timeout=0.1, write_timeout=3, exclusive=True) as connection:
        check_disarmed(connection)
        cli(connection, '#')
        environment = cli(connection, 'env')
        check_environment(environment, manifest, installed=True)
        setting = cli(connection, 'get serial_update_rate_hz')
        require(re.search(r'^serial_update_rate_hz = 250\r?$', setting, re.M), '250 Hz was not saved.')
        after_diff = cli(connection, 'diff all')
        (backup / 'environment-after.txt').write_text(environment)
        (backup / 'settings-after.txt').write_text(after_diff)
        (backup / 'serial-rate.txt').write_text(setting)
        require(config_commands(before_diff) == config_commands(after_diff),
                'Saved settings changed beyond the serial rate; inspect the backups before restarting.')
        connection.write(b'exit\r')
    reconnected()
    result.update(serial_update_rate_hz=250, other_settings_preserved=True)
    (backup / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    if restart:
        env = user_home / 'catkin_ws' / 'pals.env'
        lines = env.read_text().splitlines() if env.exists() else []
        lines = [line for line in lines if not line.startswith('PALS_FC_PORT=')]
        env.write_text('\n'.join(lines + ['PALS_FC_PORT=' + str(port.resolve())]) + '\n')
        owner = user_home.stat()
        os.chown(str(env), owner.st_uid, owner.st_gid)
        command = '^Ubash $HOME/catkin_ws/src/pidrone_pkg/boot/screen_node.sh fc^M'
        started = run(['docker', 'exec', 'pidrone_pkg', 'screen', '-S', 'pidrone', '-p', 'FC',
                       '-X', 'stuff', command])
        require(started.returncode == 0, 'Firmware verified, but the FC window could not be restarted.')
    print('Verified: %s, serial service 250 Hz, other settings preserved.' % manifest['firmware_revision'], flush=True)
    print('FC node restart requested.' if restart else 'FC node was stopped and remains stopped.', flush=True)


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
                pgrep -f '[r]oslaunch .*\.launch|[f]light_controller_node.py|[p]id_controller.py|[f]usion_localization_node|[e]kf_localization_node|[r]aspicam_node|[v]l53l1x_node|[t]of_node.py|[u]wb_localization_node|[w]eb_video_server|[a]priltag|(^|[ /])[r]outines/[A-Za-z0-9_-]+[.]py([[:space:]]|$)' >/dev/null
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



def remote(client, drone, command):
    with client.get_transport().open_session() as channel:
        channel.set_combine_stderr(True)
        channel.exec_command(command)
        output = []
        for line in channel.makefile('rb'):
            line = line.decode('utf-8', 'replace')
            output.append(line)
            print('[%s] %s' % (drone, line.rstrip()))
        return channel.recv_exit_status(), ''.join(output)


def flash(drone):
    import drones

    manifest, _ = bundle(ASSETS)
    print('%s: install %s and save serial_update_rate_hz = 250.' % (drone, manifest['firmware_revision']))
    print('Keep the FC connected to its Pi. Remove the props and keep the drone disarmed.')
    print('The command backs up this drone, stops its FC node, and restarts that node only after verification.')
    if input('Type %s to continue: ' % drone).strip() != drone:
        print('Cancelled.')
        return
    client = drones.connect(drone)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
    destination = ROOT / 'firmware_backups' / (drone + '_' + stamp)
    work = '/tmp/pals-fc-' + stamp
    try:
        require(remote(client, drone, 'sudo -n true')[0] == 0, 'Passwordless sudo is required on the Pi.')
        dependencies = "command -v dfu-util >/dev/null && python3 -c 'import serial'"
        if remote(client, drone, dependencies)[0]:
            print('Installing dfu-util and python3-serial on the Pi host.')
            require(remote(client, drone, 'sudo -n apt-get update && sudo -n apt-get install -y dfu-util python3-serial')[0] == 0,
                    'Could not install the Pi flashing dependencies.')
        with client.open_sftp() as sftp:
            user_home = sftp.normalize('.')
            remote_backup = user_home + '/catkin_ws/firmware_backups/' + Path(work).name
            sftp.mkdir(work)
            for path in [Path(__file__), ASSETS / 'manifest.json', ASSETS / manifest['file']]:
                sftp.put(str(path), work + '/' + path.name)
        command = 'sudo -n python3 -u %s flash --workdir %s --user-home %s' % (
            shlex.quote(work + '/' + Path(__file__).name), shlex.quote(work), shlex.quote(user_home))
        status, output = remote(client, drone, command)
        destination.mkdir(parents=True)
        (destination / 'session.log').write_text(output)
        remote(client, drone, 'if test -d %s; then sudo -n chown -R --reference=%s %s; fi' % (
            shlex.quote(remote_backup), shlex.quote(user_home), shlex.quote(remote_backup)))
        with client.open_sftp() as sftp:
            try:
                files = sftp.listdir(remote_backup)
            except FileNotFoundError:
                files = []
            for name in files:
                sftp.get(remote_backup + '/' + name, str(destination / name))
        print('Local backup: %s' % destination)
        require(status == 0, 'Update did not complete. Pi backups: %s; inspect the log before retrying.' % remote_backup)
        remote(client, drone, 'rm -rf -- ' + shlex.quote(work))
        print('Firmware and 250 Hz setting are installed. The Pi polling changes are required for 100 Hz IMU delivery.')
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action')
    flash_args = actions.add_parser('flash', help='install the staged firmware on the Pi')
    flash_args.add_argument('--workdir', type=Path, required=True)
    flash_args.add_argument('--user-home', type=Path, required=True)
    actions.add_parser('restart', help='stop all nodes and warm reboot the FC')
    actions.add_parser('reboot', help='stop FC and localization and warm reboot the FC')
    actions.add_parser('check', help='verify stationary gyro readings after reboot')
    args = parser.parse_args()
    if args.action is None:
        parser.error('choose flash, restart, reboot or check')
    if args.action == 'flash':
        # Flash runs directly on the Pi. The launcher already holds the lock for reboot.
        try:
            import fcntl
            lock_path = Path('/tmp/pals-start.lock')
            with lock_path.open('r+' if lock_path.exists() else 'w') as lock:
                owner = args.user_home.stat()
                os.fchown(lock.fileno(), owner.st_uid, owner.st_gid)
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                worker(args.workdir, args.user_home)
        except Exception as error:
            print('Stopped: %s\nThe FC node is not automatically restarted after an error. Backups: %s'
                  % (error, args.user_home / 'catkin_ws' / 'firmware_backups' / args.workdir.name), file=sys.stderr)
            return 1
    else:
        try:
            if args.action == 'check':
                check_gyro()
            else:
                warm_reboot(restart=args.action == 'restart')
        except Exception as error:
            print('Warm reboot incomplete: %s Keep the drone disarmed; check the FC and SE terminals before restarting.' % error)
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
