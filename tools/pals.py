#!/usr/bin/env python3
"""PALS laptop setup (macOS, Linux, Windows).

  ssh-setup   create a key and install it on the drones using their default password
  provision   pin serial ports and build on the drones (nodes start from the dashboard)
  dashboard   start the dashboard server and open it in the browser
"""
import argparse
import hashlib
import os
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'src' / 'web'
VENV = ROOT / '.venv'
VENV_PYTHON = VENV / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
REQUIREMENTS = ROOT / 'requirements.txt'


def ensure_venv():
    """Re-run inside .venv with the root laptop requirements installed."""
    if not VENV_PYTHON.exists():
        print('Creating', VENV)
        subprocess.check_call([sys.executable, '-m', 'venv', str(VENV)])
    stamp = VENV / 'pals-requirements.sha256'
    digest = hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()
    if not stamp.exists() or stamp.read_text() != digest:
        subprocess.check_call([str(VENV_PYTHON), '-m', 'pip', 'install', '-r', str(REQUIREMENTS)])
        stamp.write_text(digest)
    if Path(sys.prefix).resolve() != VENV.resolve():
        sys.exit(subprocess.call([str(VENV_PYTHON), __file__] + sys.argv[1:]))


def remote(client, drone, command):
    """Run a host command, echo its output prefixed with the drone, return (status, output)."""
    channel = client.get_transport().open_session()
    channel.set_combine_stderr(True)
    channel.exec_command(command)
    output = []
    for line in channel.makefile('rb'):
        line = line.decode('utf-8', 'replace')
        output.append(line)
        print('[%s] %s' % (drone, line.rstrip()))
    return channel.recv_exit_status(), ''.join(output)


def ssh_setup(targets):
    import drones

    ready = True
    for drone in targets:
        try:
            drones.setup_ssh(drone).close()
            print('[%s] key login works' % drone)
        except RuntimeError as exc:
            ready = False
            print('[%s] SSH setup incomplete: %s' % (drone, exc))
    return ready


def choose_port(drone, devices, kind, hints):
    guesses = [name for name in devices if any(hint in name.lower() for hint in hints)]
    if len(guesses) == 1 and input('[%s] %s is %s? [y/N] ' % (drone, kind, guesses[0])).strip().lower() == 'y':
        return guesses[0]
    for number, name in enumerate(devices, 1):
        print('  %d) %s' % (number, name))
    choice = input('[%s] Number of the %s: ' % (drone, kind))
    return devices[int(choice) - 1]


def provision(targets):
    import drones

    for drone in targets:
        client = drones.connect(drone)
        status, package = remote(client, drone, 'readlink -f ~/catkin_ws/src/pidrone_pkg')
        package = package.strip()
        if status or not package:
            sys.exit('%s: ~/catkin_ws/src/pidrone_pkg is missing; see "Cloning the Repo" in src/README.md' % drone)
        if remote(client, drone, 'docker inspect -f "{{.Name}}" pidrone_pkg')[0]:
            sys.exit('%s: no pidrone_pkg container; on the drone run: cd %s && rake build && rake create'
                     % (drone, package))

        _, listing = remote(client, drone, 'ls /dev/serial/by-id/')
        devices = ['/dev/serial/by-id/' + name for name in listing.split()]
        if len(devices) < 2:
            sys.exit('%s: expected the flight controller and UWB tag in /dev/serial/by-id' % drone)
        fc = choose_port(drone, devices, 'flight controller', ('stm', 'betaflight', 'speedybee'))
        uwb = choose_port(drone, devices, 'UWB tag', ('segger', 'j-link'))
        if fc == uwb:
            sys.exit('%s: the flight controller and UWB tag must be different devices' % drone)
        remote(client, drone, "printf 'PALS_FC_ID=%s\\nPALS_UWB_ID=%s\\n' > ~/catkin_ws/pals.ports" % (fc, uwb))

        for step in ('python -m pip install --user -r ~/catkin_ws/src/pidrone_pkg/requirements.txt',
                     'cd ~/catkin_ws && catkin_make'):
            if remote(client, drone, "docker start pidrone_pkg >/dev/null && "
                      "docker exec pidrone_pkg bash -ic '%s'" % step)[0]:
                sys.exit('%s: failed: %s' % (drone, step))
        # Earlier versions of this branch started nodes at boot; that service must go.
        if remote(client, drone, 'systemctl is-enabled pals.service >/dev/null 2>&1')[0] == 0:
            print('[%s] disable the old boot service: ssh -t %s sudo systemctl disable pals.service'
                  % (drone, drone))
        client.close()
        print('[%s] provisioned; start nodes from the dashboard' % drone)


def dashboard(port):
    threading.Timer(1.5, webbrowser.open, ['http://localhost:%d' % port]).start()
    sys.exit(subprocess.call([sys.executable, str(WEB / 'server.py'), '--port', str(port)]))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('ssh-setup', 'provision', 'dashboard'))
    parser.add_argument('--drone', choices=('drone1', 'drone2'), help='only this drone (default: both)')
    parser.add_argument('--port', type=int, default=8000, help='dashboard port')
    args = parser.parse_args()
    ensure_venv()
    sys.path.insert(0, str(WEB))
    targets = [args.drone] if args.drone else ['drone1', 'drone2']
    if args.command == 'ssh-setup':
        if not ssh_setup(targets):
            sys.exit(1)
    elif args.command == 'provision':
        provision(targets)
    else:
        dashboard(args.port)


if __name__ == '__main__':
    main()
