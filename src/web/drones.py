"""SSH access to the drones for the dashboard's script terminals."""
import re
import shlex
import threading
from pathlib import Path

import paramiko

DRONES = ('drone1', 'drone2')
USER = 'duckie'
KEY = Path.home() / '.ssh' / 'pals_ed25519'
CONTAINER = 'pidrone_pkg'
SESSION = 'pidrone'
PACKAGE = '~/catkin_ws/src/pidrone_pkg'
ROUTINES = PACKAGE + '/scripts/PALS/routines'
START = 'bash ' + PACKAGE + '/boot/pals-start.sh'
SCRIPT = re.compile(r'^[A-Za-z0-9_]+\.py$')
# Arguments are typed into a screen window, so exclude quotes and screen/shell escapes.
ARGS = re.compile(r'^[A-Za-z0-9 _.,:=+/@%-]*$')

_clients = {}
_lock = threading.Lock()


def host(drone):
    if drone not in DRONES:
        raise ValueError('Unknown drone: %r' % (drone,))
    return drone + '.local'


def connect(drone, timeout=5):
    """Open a key-authenticated connection; unknown host keys are rejected."""
    if not KEY.exists():
        raise RuntimeError('%s: no SSH key at %s (run tools/pals.py ssh-setup)' % (drone, KEY))
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(host(drone), username=USER, key_filename=str(KEY), timeout=timeout,
                       allow_agent=False, look_for_keys=False)
    except paramiko.SSHException as error:
        client.close()
        raise RuntimeError('%s: %s (run tools/pals.py ssh-setup)' % (drone, error))
    except OSError as error:
        client.close()
        raise RuntimeError('%s is unreachable: %s' % (drone, error))
    return client


def run(drone, command, timeout=10):
    """Run a host command over a reused connection; returns (status, stdout)."""
    with _lock:
        client = _clients.get(drone)
        if client is None or not client.get_transport() or not client.get_transport().is_active():
            client = _clients[drone] = connect(drone)
    _, stdout, _ = client.exec_command(command, timeout=timeout)
    output = stdout.read().decode('utf-8', 'replace')
    return stdout.channel.recv_exit_status(), output


def list_scripts(drone):
    status, output = run(drone, 'ls -1 ' + ROUTINES)
    if status:
        raise RuntimeError('%s has no %s' % (drone, ROUTINES))
    return sorted(name for name in output.split() if SCRIPT.match(name))


def routine_running(drone):
    status, _ = run(drone, 'docker exec %s pgrep -f "PALS/routines/[A-Za-z0-9_]+[.]py"' % CONTAINER)
    return status == 0


def nodes_running(drone):
    return run(drone, START + ' status')[0] == 0


def status(drone):
    return {'scripts': list_scripts(drone), 'nodes': nodes_running(drone), 'running': routine_running(drone)}


def node_action(drone, action):
    """Restart every node, or only fusion (align); fusion aligns map +Y when it starts.

    Returns the script's last output line, which summarizes the result.
    """
    host(drone)
    if action not in ('restart', 'align'):
        raise ValueError('Unknown node action: %r' % (action,))
    if routine_running(drone):
        raise RuntimeError('a routine is running; stop it first')
    code, output = run(drone, START + ' ' + action + ' 2>&1', timeout=90)
    # Interactive container shells print banners first; the script ends with its result.
    lines = output.strip().splitlines() or ['no output']
    if code:
        raise RuntimeError(lines[-1])
    return lines[-1]


def routine_command(script, args):
    if not SCRIPT.match(script):
        raise ValueError('Invalid script name')
    if not ARGS.match(args):
        raise ValueError('Arguments may only contain letters, digits, spaces and _.,:=+/@%-')
    return ' '.join(['python', '-u', ROUTINES + '/' + script] + args.split())


def execute(drone, script, args):
    """Type the routine into the screen session's `routine` window."""
    host(drone)
    command = routine_command(script, args)
    if script not in list_scripts(drone):
        raise ValueError('%s does not have %s' % (drone, script))
    if routine_running(drone):
        raise RuntimeError('%s is already running a routine' % drone)
    # ^U clears any partly typed line; ^M submits.
    stuff = 'docker exec %s screen -S %s -p routine -X stuff %s' % (
        CONTAINER, SESSION, shlex.quote('^U' + command + '^M'))
    status, _ = run(drone, stuff)
    if status:
        raise RuntimeError('%s nodes are not started' % drone)


def open_terminal(drone, cols, rows, start=False):
    """Attach to the drone's screen session; closing only detaches.

    With start (the Connect button), first start the nodes here so their startup output
    is visible, then report the exit status as the escape sequence OSC 777.
    """
    client = connect(drone)
    channel = client.get_transport().open_session()
    channel.get_pty(term='xterm-256color', width=cols, height=rows)
    attach = 'exec docker exec -it %s screen -x %s -p routine' % (CONTAINER, SESSION)
    if start:
        command = START + r" start; code=$?; printf '\033]777;%d\007' $code; [ $code -eq 0 ] && " + attach
    else:
        command = ("until %s status; do [ -n \"$waited\" ] || echo 'Nodes not started: click Connect.'; "
                   'waited=1; sleep 2; done; %s' % (START, attach))
    channel.exec_command(command)
    return client, channel
