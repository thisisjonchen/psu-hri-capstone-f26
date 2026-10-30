#!/usr/bin/env python3
"""Install the bundled F405 firmware through the Pi; called by pals.py flash-fc.

The worker runs with Python 3 on the Pi host, outside the ROS container.
"""
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

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'tools' / 'firmware' / 'betaflight'
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
    if running.returncode:
        return False
    mode = run(['docker', 'exec', 'pidrone_pkg', 'bash', '-ic',
                'timeout 5 rostopic echo -n1 /pidrone/mode'])
    require(mode.returncode == 0 and re.search(r'^mode: [\"\']?DISARMED[\"\']?\s*$', mode.stdout, re.M),
            'The running FC node must report DISARMED before it can be stopped.')
    require(run(['docker', 'exec', 'pidrone_pkg', 'pgrep', '-f',
                 'PALS/routines/[A-Za-z0-9_-]+[.]py']).returncode == 1,
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
            'Expected one SpeedyBee F405 V3 flight controller; check tools/pals.py provision.')
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
    require('@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg' in layout,
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
        command = '^Ubash $HOME/catkin_ws/src/pidrone_pkg/scripts/screen_node.sh fc^M'
        started = run(['docker', 'exec', 'pidrone_pkg', 'screen', '-S', 'pidrone', '-p', 'FC',
                       '-X', 'stuff', command])
        require(started.returncode == 0, 'Firmware verified, but the FC window could not be restarted.')
    print('Verified: usb2ms001, serial service 250 Hz, other settings preserved.', flush=True)
    print('FC node restart requested.' if restart else 'FC node was stopped and remains stopped.', flush=True)


def flash(drone):
    import drones
    from pals import remote

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
        command = 'sudo -n python3 -u %s --workdir %s --user-home %s' % (
            shlex.quote(work + '/flash_fc.py'), shlex.quote(work), shlex.quote(user_home))
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Pi worker for tools/pals.py flash-fc')
    parser.add_argument('--workdir', type=Path, required=True)
    parser.add_argument('--user-home', type=Path, required=True)
    args = parser.parse_args()
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
        sys.exit(1)
