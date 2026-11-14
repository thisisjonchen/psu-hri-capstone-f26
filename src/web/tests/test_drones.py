import shutil
import subprocess
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import drones
import drone_git
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import warm_reboot_fc


class ExecuteTest(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.running = 1

        def run(drone, command, timeout=10):
            self.commands.append(command)
            if command.startswith('ls'):
                return 0, 'hover_routine.py\nprototype-1-forward.py\nREADME.md\n'
            if 'pgrep' in command:
                return self.running, ''
            return 0, ''

        patch = mock.patch.object(drones, 'run', side_effect=run)
        patch.start()
        self.addCleanup(patch.stop)

    def test_types_routine_into_screen_window(self):
        drones.execute('drone1', 'prototype-1-forward.py', '--fly --distance 0.25')
        self.assertEqual(self.commands[-1],
                         "docker exec pidrone_pkg screen -S pidrone -p routine -X stuff "
                         "'^Upython -u ~/catkin_ws/src/pidrone_pkg/routines/prototype-1-forward.py --fly --distance 0.25^M'")

    def test_rejects_unsafe_or_unavailable_requests(self):
        for drone, script, args in [('drone3', 'hover_routine.py', ''),
                                    ('drone1', '../flight_controller_node.py', ''),
                                    ('drone1', 'hover_routine.py', "--fly; rm -rf ~"),
                                    ('drone1', 'hover_routine.py', '^C'),
                                    ('drone1', 'missing_routine.py', '--fly')]:
            with self.assertRaises(ValueError):
                drones.execute(drone, script, args)
        for drone, action in [('drone1', 'start'), ('drone1', 'align'),
                              ('drone1', 'stop; reboot'), ('drone3', 'restart')]:
            with self.assertRaises(ValueError):
                drones.node_action(drone, action)
        self.running = 0
        with self.assertRaisesRegex(RuntimeError, 'already running'):
            drones.execute('drone1', 'hover_routine.py', '--fly')
        self.assertFalse(any('stuff' in command or 'pals-start' in command for command in self.commands))

    def test_restart_reports_combined_launcher_result_and_failure(self):
        for code, result in [(0, 'Nodes restarted and localized. Gyro mean: 0.001 deg/s.'),
                             (1, 'Warm reboot incomplete: Incomplete MSP reply.')]:
            with self.subTest(code=code), mock.patch.object(
                    drones, 'run', return_value=(code, 'shell banner\n' + result + '\n')) as run:
                if code:
                    with self.assertRaisesRegex(RuntimeError, 'Warm reboot incomplete'):
                        drones.node_action('drone1', 'restart')
                else:
                    self.assertEqual(drones.node_action('drone1', 'restart'), result)
                self.assertEqual(run.call_args, mock.call(
                    'drone1', drones.START + ' restart 2>&1', timeout=240))


@unittest.skipUnless(shutil.which('git'), 'requires git')
class BranchTest(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.repo = self.root / 'drone'
        self.repo.mkdir()
        self.cwd = self.repo
        self.mode = 'DISARMED'
        self.container_running = True
        self.nodes_running = True
        self.routine_code = 1
        self.hardware_calls = []
        self.build_code = 0
        self.build_calls = []

        def command(*args, **kwargs):
            if args[0] == 'git':
                return subprocess.run(args, cwd=self.cwd, capture_output=True, text=True, timeout=10)
            self.hardware_calls.append(args)
            if args[:2] == ('docker', 'inspect'):
                return subprocess.CompletedProcess(args, 1 if self.container_running is None else 0,
                                                   str(self.container_running).lower(), '')
            if args[:2] == ('docker', 'start'):
                self.container_running = True
                return subprocess.CompletedProcess(args, 0, '', '')
            if args[:4] == ('docker', 'exec', 'pidrone_pkg', 'pgrep'):
                code = self.routine_code if 'routines/' in args[-1] else (
                    2 if self.nodes_running is None else 0 if self.nodes_running else 1)
                return subprocess.CompletedProcess(args, code, '', '')
            if args[:4] == ('docker', 'exec', 'pidrone_pkg', 'timeout'):
                self.build_calls.append((args, kwargs))
                return subprocess.CompletedProcess(args, self.build_code, '', 'compile failed' if self.build_code else '')
            if args[:4] == ('docker', 'exec', 'pidrone_pkg', 'bash'):
                return subprocess.CompletedProcess(args, 0, 'mode: "' + self.mode + '"\n', '')
            self.fail('Unexpected hardware command: ' + repr(args))

        patch = mock.patch.object(drone_git, 'command', side_effect=command)
        patch.start()
        self.addCleanup(patch.stop)
        patch = mock.patch('drone_git.open', lambda *args: open(self.root / 'lock', 'a'), create=True)
        patch.start()
        self.addCleanup(patch.stop)
        drone_git.git('init', '--bare', str(self.root / 'origin'))
        drone_git.git('init')
        drone_git.git('config', 'user.email', 'test@example.invalid')
        drone_git.git('config', 'user.name', 'Test')
        drone_git.git('config', 'commit.gpgsign', 'false')
        drone_git.git('checkout', '-b', 'dual-hover')
        (self.repo / 'node.py').write_text('old code\n')
        drone_git.git('add', 'node.py')
        drone_git.git('commit', '-m', 'baseline')
        drone_git.git('remote', 'add', 'origin', str(self.root / 'origin'))
        drone_git.git('push', '-u', 'origin', 'dual-hover')
        drone_git.git('checkout', '-b', 'coordination-stability')
        (self.repo / 'node.py').write_text('new code\n')
        drone_git.git('commit', '-am', 'update')
        drone_git.git('push', 'origin', 'coordination-stability')
        drone_git.git('checkout', 'dual-hover')
        drone_git.git('branch', '-D', 'coordination-stability')

    def test_switch_tracks_remote_branch_and_connects_future_pulls_to_it(self):
        self.assertEqual(drone_git.status()['branches'], ['coordination-stability', 'dual-hover'])
        prepared = drone_git.change('prepare', 'coordination-stability')
        self.assertEqual(drone_git.state()['branch'], 'dual-hover', 'preparation must not switch')
        result = drone_git.change('switch', 'coordination-stability', prepared['target'])
        self.assertEqual(result['branch'], 'coordination-stability')
        self.assertEqual(drone_git.git('rev-parse', '--abbrev-ref', '@{upstream}'), 'origin/coordination-stability')
        self.assertEqual((self.repo / 'node.py').read_text(), 'new code\n')

    def test_build_checks_mode_and_revision_and_retries_after_failure(self):
        branch = 'coordination-stability'
        revision = drone_git.change('prepare', branch)['target']
        drone_git.change('switch', branch, revision)
        self.mode = 'ARMED'
        with self.assertRaisesRegex(RuntimeError, 'must report DISARMED'):
            drone_git.change('build', branch, revision)
        self.mode = 'DISARMED'
        with self.assertRaisesRegex(RuntimeError, 'checkout changed'):
            drone_git.change('build', branch, '0' * 40)
        self.assertEqual(self.build_calls, [])
        self.build_code = 1
        with self.assertRaisesRegex(RuntimeError, 'Build failed; nodes were not restarted'):
            drone_git.change('build', branch, revision)
        self.build_code = 0
        result = drone_git.change('build', branch, revision)
        self.assertEqual(result['branch'], branch)
        self.assertTrue(result['restart'])
        self.assertEqual(len(self.build_calls), 2, 'retry the build even when the revision is unchanged')
        args, kwargs = self.build_calls[-1]
        self.assertEqual(args[-1], 'cd ~/catkin_ws && catkin_make --pkg pidrone_pkg -j1')
        self.assertGreater(kwargs['timeout'], 600)

    def test_stopped_drone_can_clear_switch_and_build_without_starting_nodes(self):
        self.nodes_running = False
        for running in (False, True):
            self.container_running = running
            (self.repo / 'node.py').write_text('local changes\n')
            drone_git.change('prepare-clear')
            result = drone_git.change('clear')
            self.assertFalse(result['dirty'])
        self.container_running = False
        branch = 'coordination-stability'
        revision = drone_git.change('prepare', branch)['target']
        drone_git.change('switch', branch, revision)
        result = drone_git.change('build', branch, revision)
        self.assertEqual(result['branch'], branch)
        self.assertFalse(result['restart'])
        self.assertEqual(len(self.build_calls), 1)
        self.assertEqual(self.hardware_calls.count(('docker', 'start', 'pidrone_pkg')), 1)
        self.assertFalse(any('rostopic' in str(args) or 'pals-start' in str(args)
                             for args in self.hardware_calls))
        # An unavailable container/process check is not proof that nodes are stopped.
        for container, nodes, routine in [(None, False, 1), (True, None, 1), (True, False, 0), (True, False, 2)]:
            self.container_running, self.nodes_running, self.routine_code = container, nodes, routine
            (self.repo / 'node.py').write_text('keep this work\n')
            with self.assertRaises(RuntimeError):
                drone_git.change('clear')
            self.assertEqual((self.repo / 'node.py').read_text(), 'keep this work\n')

    def test_dirty_armed_and_diverged_repositories_keep_their_work_and_branch(self):
        (self.repo / 'node.py').write_text('local work\n')
        with self.assertRaisesRegex(RuntimeError, 'save or stash'):
            drone_git.change('prepare', 'coordination-stability')
        self.assertEqual((self.repo / 'node.py').read_text(), 'local work\n')
        (self.repo / 'node.py').write_text('old code\n')
        for mode in ('ARMED', 'FLYING', ''):
            self.mode = mode
            with self.assertRaisesRegex(RuntimeError, 'must report DISARMED'):
                drone_git.change('prepare', 'coordination-stability')
        self.mode = 'DISARMED'
        drone_git.git('checkout', '-b', 'coordination-stability', 'origin/coordination-stability')
        (self.repo / 'node.py').write_text('unpublished work\n')
        drone_git.git('commit', '-am', 'local only')
        drone_git.git('checkout', 'dual-hover')
        with self.assertRaisesRegex(RuntimeError, 'commits not on origin'):
            drone_git.change('prepare', 'coordination-stability')
        self.assertEqual(drone_git.state()['branch'], 'dual-hover')

    def test_clear_covers_entire_repo_preserves_commits_and_ignored_files_and_rechecks_mode(self):
        (self.repo / 'committed.txt').write_text('local commit\n')
        drone_git.git('add', 'committed.txt')
        drone_git.git('commit', '-m', 'unpublished commit')
        revision = drone_git.git('rev-parse', 'HEAD')
        (self.repo / '.git/info/exclude').write_text('keep.log\n')
        (self.repo / 'keep.log').write_text('saved log\n')
        (self.repo / 'node.py').write_text('staged changes\n')
        drone_git.git('add', 'node.py')
        (self.repo / 'node.py').write_text('unstaged changes\n')
        (self.repo / 'scratch.txt').write_text('untracked root file\n')
        package = self.repo / 'src'
        package.mkdir()
        (package / 'scratch.txt').write_text('untracked package file\n')
        self.cwd = package  # SSH enters the ROS package, below the Git root.
        self.assertTrue(drone_git.change('prepare-clear')['dirty'])
        self.assertEqual((self.repo / 'node.py').read_text(), 'unstaged changes\n')
        for mode in ('ARMED', 'FLYING', ''):
            self.mode = mode
            for action in ('prepare-clear', 'clear'):
                with self.assertRaisesRegex(RuntimeError, 'must report DISARMED'):
                    drone_git.change(action)
            self.assertTrue((self.repo / 'scratch.txt').exists())
            self.assertEqual((self.repo / 'node.py').read_text(), 'unstaged changes\n')
        self.mode = 'DISARMED'
        # Keep the package directory after cleaning, as in a real checkout.
        (package / 'keep.log').write_text('package log\n')
        result = drone_git.change('clear')
        self.assertFalse(result['dirty'])
        self.assertEqual(result['branch'], 'dual-hover')
        self.assertEqual(drone_git.git('rev-parse', 'HEAD'), revision)
        self.assertEqual((self.repo / 'node.py').read_text(), 'old code\n')
        self.assertFalse((self.repo / 'scratch.txt').exists())
        self.assertFalse((package / 'scratch.txt').exists())
        self.assertEqual((self.repo / 'keep.log').read_text(), 'saved log\n')
        self.assertEqual((package / 'keep.log').read_text(), 'package log\n')

    def test_single_drone_status_and_actions_never_contact_the_other_drone(self):
        for drone in drones.DRONES:
            with self.subTest(drone=drone), mock.patch.object(
                    drones, 'branch_command', return_value={'target': 'abc'}) as command:
                self.assertEqual(drones.branch_results('status', [drone]), {drone: {'target': 'abc'}})
                command.assert_called_once_with(drone, 'status', None, None)
                command.reset_mock()
                drones.branch_action('switch', [drone], 'new')
                self.assertEqual(command.call_args_list, [mock.call(drone, 'prepare', 'new', None),
                                                         mock.call(drone, 'switch', 'new', 'abc'),
                                                         mock.call(drone, 'build', 'new', 'abc')])
                command.reset_mock()
                drones.branch_action('clear', [drone])
                self.assertEqual(command.call_args_list, [mock.call(drone, 'prepare-clear', None, None),
                                                         mock.call(drone, 'clear', None, None)])
        with mock.patch.object(drones, 'branch_command') as command:
            for targets in (None, [], 'drone1', ['drone3'], ['drone1', 'drone1'], [['drone1']]):
                with self.subTest(targets=targets), self.assertRaises(ValueError):
                    drones.branch_action('clear', targets)
            command.assert_not_called()

    def test_both_drones_are_prepared_before_mutation_and_partial_failures_are_reported(self):
        for action, branch, preparation in [('switch', 'coordination-stability', 'prepare'),
                                             ('clear', None, 'prepare-clear')]:
            for failure in ({'error': 'Running routine'}, {'error': 'offline'}):
                prepared = {'drone1': {'target': 'abc'}, 'drone2': failure}
                with mock.patch.object(drones, 'branch_results', return_value=prepared) as calls:
                    self.assertEqual(drones.branch_action(action, drones.DRONES, branch)['phase'], 'prepare')
                    calls.assert_called_once_with(preparation, drones.DRONES, branch)
            prepared = {drone: {'target': 'abc'} for drone in drones.DRONES}
            partial = {'drone1': {'branch': 'coordination-stability'}, 'drone2': {'error': 'connection lost'}}
            with mock.patch.object(drones, 'branch_results', side_effect=[prepared, partial]) as calls:
                self.assertEqual(drones.branch_action(action, drones.DRONES, branch), {'phase': action, 'drones': partial})
                self.assertEqual(calls.call_args, mock.call('clear', drones.DRONES) if action == 'clear'
                                 else mock.call('switch', drones.DRONES, branch, 'abc'))
        with mock.patch.object(drones, 'branch_results') as calls:
            for name in ('--force', 'main; reboot', None):
                with self.assertRaises(ValueError):
                    drones.branch_action('switch', drones.DRONES, name)
            with self.assertRaises(ValueError):
                drones.branch_action('unknown', drones.DRONES)
            with drones._branch_lock:
                with self.assertRaisesRegex(RuntimeError, 'already in progress'):
                    drones.branch_action('clear', drones.DRONES)
            calls.assert_not_called()

        prepared = {drone: {'target': 'abc'} for drone in drones.DRONES}
        switched = {drone: {'branch': 'new', 'revision': 'abc'} for drone in drones.DRONES}
        for failed in (False, True):
            built = dict(switched)
            if failed:
                built['drone2'] = {'error': 'compile failed'}
            with self.subTest(build_failed=failed), mock.patch.object(
                    drones, 'branch_results', side_effect=[prepared, switched, built]) as calls:
                result = drones.branch_action('switch', drones.DRONES, 'new')
                self.assertEqual(result['phase'], 'build')
                self.assertEqual(calls.call_args_list, [mock.call('prepare', drones.DRONES, 'new'),
                    mock.call('switch', drones.DRONES, 'new', 'abc'),
                    mock.call('build', drones.DRONES, 'new', 'abc')])
                self.assertEqual(result['drones']['drone2']['branch'], 'new')
                self.assertEqual('error' in result['drones']['drone2'], failed)


class RestartTest(unittest.TestCase):
    def test_fc_reboot_requires_new_usb_and_safe_verification_without_resending(self):
        # No real serial devices, Docker calls, or delays are used.
        for fault, error in [(None, None), ('unchanged', 'did not reconnect'),
                             ('wrong_usb', 'did not reconnect'), ('armed_before', 'armed'),
                             ('armed_after', 'armed'), ('owned', 'owned'),
                             ('calibrating', 'calibration did not finish'),
                             ('serial', 'serial verification')]:
            with self.subTest(fault=fault), TemporaryDirectory() as directory:
                root = Path(directory)
                port = root / 'serial'
                port.touch()
                usb = root / 'usb'
                usb.mkdir()
                generation = usb / 'devnum'
                generation.write_text('3\n')
                connection = mock.MagicMock()
                connection.__enter__.return_value = connection
                serial = SimpleNamespace(Serial=mock.Mock(return_value=connection), SerialException=OSError)

                def write(packet):
                    if fault != 'unchanged':
                        generation.write_text('5\n')
                    return len(packet)

                def msp(connection, command):
                    if command == 68:
                        raise OSError('device reports readiness to read but returned no data')
                    return {2: b'BTFL', 1: bytes([0, 1, 48])}[command]

                checks = []
                def disarmed(connection):
                    checks.append(True)
                    if fault == 'armed_before' or (fault == 'armed_after' and len(checks) > 1):
                        raise RuntimeError('Flight controller is armed; refusing to continue.')
                    if len(checks) > 1 and (fault == 'serial' or (fault is None and len(checks) == 2)):
                        raise OSError('USB is still becoming ready')

                def wait(check, message):
                    for _ in range(4):
                        if check():
                            return
                    raise RuntimeError(message)

                connection.write.side_effect = write
                with mock.patch.dict(sys.modules, {'serial': serial}), \
                        mock.patch.object(warm_reboot_fc, 'port_unused',
                                          side_effect=RuntimeError('Serial port is still owned') if fault == 'owned' else None), \
                        mock.patch.object(warm_reboot_fc, 'usb_device',
                                          side_effect=lambda port: root if fault == 'wrong_usb' and connection.write.called else usb), \
                        mock.patch.object(warm_reboot_fc, 'msp', side_effect=msp), \
                        mock.patch.object(warm_reboot_fc, 'check_disarmed', side_effect=disarmed), \
                        mock.patch.object(warm_reboot_fc, 'calibration_complete',
                                          side_effect=[False] * 4 if fault == 'calibrating' else [False, True]) as calibrated, \
                        mock.patch.object(warm_reboot_fc, 'wait_for', side_effect=wait):
                    if error:
                        with self.assertRaisesRegex(RuntimeError, error):
                            warm_reboot_fc.reboot_controller(port)
                    else:
                        warm_reboot_fc.reboot_controller(port)
                        self.assertEqual(calibrated.call_count, 2)
                        self.assertEqual(len(checks), 4, 'retry only read-only verification after USB returns')
                if fault in ('armed_before', 'owned'):
                    connection.write.assert_not_called()
                else:
                    connection.write.assert_called_once_with(b'$M<\x00DD')
                if fault in ('unchanged', 'wrong_usb'):
                    self.assertEqual(serial.Serial.call_count, 1, 'never verify the old or a different USB device')

    @unittest.skipUnless(shutil.which('bash'), 'requires bash')
    def test_restart_starts_replacements_only_after_cleanup_and_fc_reboot(self):
        launcher = Path(__file__).resolve().parents[2] / 'boot/pals-start.sh'
        stubs = r'''
flock() { [[ "$failure" != lock ]]; }
docker() {
    printf '%s\n' "$*" >> "$calls"
    case "$*" in
        'start pidrone_pkg') return 0 ;;
        *screen\ -ls*) return 1 ;;
        'exec pidrone_pkg pgrep '*) return 1 ;;
        'exec -d pidrone_pkg '*) echo 'mock replacement launch reached'; return 1 ;;
        *) echo "unexpected Docker call: $*"; return 1 ;;
    esac
}
sudo() {
    [[ "$*" == *' restart' ]] || return 2
    echo 'cleanup and FC verification' >> "$calls"
    [[ "$failure" != reboot ]] || { echo 'mock FC verification failed'; return 1; }
}
calls=$2
failure=$3
source "$1" restart
'''
        for failure in ('', 'reboot', 'lock'):
            with self.subTest(failure=failure), TemporaryDirectory() as directory:
                calls = Path(directory) / 'commands'
                calls.touch()
                result = subprocess.run([shutil.which('bash'), '-c', stubs,
                                         str(launcher), str(launcher), str(calls), failure],
                                        capture_output=True, text=True)
                self.assertNotIn('unexpected Docker call', result.stdout)
                commands = calls.read_text()
                if failure:
                    self.assertNotIn('exec -d pidrone_pkg', commands)
                    self.assertEqual(result.returncode, 1)
                    self.assertIn('mock FC verification failed' if failure == 'reboot' else
                                  'restart was not queued', result.stdout + result.stderr)
                else:
                    self.assertIn('mock replacement launch reached', result.stdout)
                    self.assertLess(commands.index('cleanup and FC verification'),
                                    commands.index('exec -d pidrone_pkg'))

    def test_full_shutdown_recovers_stuck_nodes_but_requires_disarmed_and_stopped_container(self):
        cases = [(None, None), ('missing', None), ('hung', None), ('timeout', None),
                 ('armed', 'reports DISARMED'), ('unknown', 'reports DISARMED'),
                 ('fc_query', 'Could not check the FC'), ('owned', 'owned'),
                 ('fc_armed', 'armed'), ('stop_failed', 'Could not stop the drone container'),
                 ('still_running', 'not confirmed stopped')]
        for fault, error in cases:
            with self.subTest(fault=fault):
                calls = []
                def run(command, **kwargs):
                    calls.append(command)
                    status, output = 0, ''
                    if 'pgrep' in command:
                        status = 2 if fault == 'fc_query' else int(fault in ('missing', 'owned', 'fc_armed'))
                    elif '-ic' in command:
                        output = 'mode: ARMED' if fault == 'armed' else 'mode: "DISARMED"'
                        status = 124 if fault == 'unknown' else 0
                    elif '-c' in command:
                        if fault == 'timeout':
                            raise subprocess.TimeoutExpired(command, 20)
                        status = int(fault == 'hung')
                    elif command[1] == 'stop':
                        status = int(fault == 'stop_failed')
                    elif command[1] == 'inspect':
                        output = 'true' if fault == 'still_running' else 'false'
                    else:
                        self.fail('Unexpected command: %r' % command)
                    return SimpleNamespace(returncode=status, stdout=output)
                serial = SimpleNamespace(Serial=mock.MagicMock())
                with mock.patch.object(warm_reboot_fc, 'run', side_effect=run), \
                        mock.patch.dict(sys.modules, {'serial': serial}), \
                        mock.patch.object(warm_reboot_fc, 'port_unused',
                                          side_effect=RuntimeError('owned') if fault == 'owned' else None), \
                        mock.patch.object(warm_reboot_fc, 'check_disarmed',
                                          side_effect=RuntimeError('armed') if fault == 'fc_armed' else None) as check:
                    if error:
                        with self.assertRaisesRegex(RuntimeError, error):
                            warm_reboot_fc.stop_all_nodes('/dev/serial/by-id/test')
                    else:
                        warm_reboot_fc.stop_all_nodes('/dev/serial/by-id/test')
                    if fault in ('armed', 'unknown', 'fc_query', 'owned', 'fc_armed'):
                        self.assertFalse(any('-c' in c or c[1] == 'stop' for c in calls))
                    elif fault != 'stop_failed':
                        self.assertEqual(calls[-2:], [
                            ['docker', 'stop', '--time', '5', 'pidrone_pkg'],
                            ['docker', 'inspect', '--format', '{{.State.Running}}', 'pidrone_pkg']])
                    if fault == 'missing':
                        check.assert_called_once()

    def test_warm_reboot_with_stopped_nodes_still_verifies_fc(self):
        for restart, localization_status in ((False, 1), (False, 2), (True, 1), (True, 2)):
            with self.subTest(restart=restart, status=localization_status), TemporaryDirectory() as directory:
                home = Path(directory)
                (home / 'catkin_ws').mkdir()
                (home / 'catkin_ws/pals.ports').write_text(
                    'PALS_FC_ID=/dev/serial/by-id/SPEEDYBEEF405V3-test\n')
                environment = home / 'catkin_ws/pals.env'
                environment.write_text('PALS_FC_PORT=/dev/ttyACM0\nPALS_UWB_PORT=/dev/ttyACM1\n')
                port = mock.Mock()
                port.parent = Path('/dev/serial/by-id')
                port.name = 'SPEEDYBEEF405V3-test'
                port.exists.return_value = True
                port.resolve.return_value = '/dev/ttyACM2'
                def path(value):
                    if value == '/home/duckie':
                        return home
                    if value == '/dev/serial/by-id/SPEEDYBEEF405V3-test':
                        return port
                    return Path(value)
                with mock.patch.object(warm_reboot_fc, 'Path', side_effect=path), \
                        mock.patch.object(warm_reboot_fc.os, 'geteuid', return_value=0), \
                        mock.patch.object(warm_reboot_fc, 'stop_fc', return_value=False), \
                        mock.patch.object(warm_reboot_fc, 'run', return_value=SimpleNamespace(
                            returncode=localization_status, stdout='process check')) as run, \
                        mock.patch.object(warm_reboot_fc, 'stop_all_nodes',
                                          side_effect=RuntimeError('container stop failed')
                                          if localization_status == 2 else None) as stop_all, \
                        mock.patch.object(warm_reboot_fc, 'reboot_controller') as reboot:
                    order = mock.Mock()
                    order.attach_mock(stop_all, 'stop')
                    order.attach_mock(reboot, 'reboot')
                    if localization_status == 2:
                        with self.assertRaisesRegex(RuntimeError, 'container stop failed' if restart else
                                                    'Could not check localization'):
                            warm_reboot_fc.warm_reboot(restart=restart)
                        reboot.assert_not_called()
                    else:
                        warm_reboot_fc.warm_reboot(restart=restart)
                        reboot.assert_called_once_with(port)
                        if restart:
                            self.assertEqual(order.mock_calls, [mock.call.stop(port), mock.call.reboot(port)])
                        self.assertIn('PALS_FC_PORT=/dev/ttyACM2', environment.read_text())
                    self.assertEqual(run.call_count, 0 if restart else 1)

    def test_restart_readiness_requires_fresh_tof_and_localization(self):
        launcher = Path(__file__).resolve().parents[2] / 'boot/pals-start.sh'
        code = launcher.read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
        for fault in (None, 'missing', 'stale', 'invalid', 'localization'):
            with self.subTest(fault=fault):
                now, subscribers = [100.0], {}

                def sleep(seconds):
                    now[0] += seconds
                    subscribers['/pidrone/localization/local_ready'](
                        SimpleNamespace(data=fault != 'localization'))
                    if fault != 'missing':
                        stamp = now[0] - (2 if fault == 'stale' else 0)
                        subscribers['/pidrone/range'](SimpleNamespace(
                            range=float('nan') if fault == 'invalid' else .05,
                            min_range=0., max_range=4.,
                            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))))

                modules = {
                    'time': SimpleNamespace(sleep=sleep),
                    'rospy': SimpleNamespace(init_node=lambda *a, **kw: None,
                        get_time=lambda: now[0], is_shutdown=lambda: False,
                        Subscriber=lambda topic, cls, callback: subscribers.update({topic: callback})),
                    'diagnostic_msgs.msg': SimpleNamespace(DiagnosticArray=object),
                    'sensor_msgs.msg': SimpleNamespace(Range=object),
                    'std_msgs.msg': SimpleNamespace(Bool=object)}
                with mock.patch.dict(sys.modules, modules), mock.patch('builtins.print') as report:
                    if fault:
                        with self.assertRaises(SystemExit) as error:
                            exec(code, {})
                        self.assertEqual(error.exception.code, 1)
                        expected = 'Localization is not ready' if fault == 'localization' else 'ToF has no fresh valid range'
                        self.assertIn(expected, report.call_args.args[0])
                    else:
                        exec(code, {})
                        self.assertIn('EKF initialized', report.call_args.args[0])

    def test_gyro_check_requires_fresh_disarmed_telemetry_through_end_of_window(self):
        for fault in (None, 'imu', 'mode', 'armed'):
            with self.subTest(fault=fault):
                now, subscribers = [0.0], {}

                def sleep(seconds):
                    now[0] += seconds
                    if fault != 'imu' or now[0] < 8.5:
                        subscribers['/pidrone/imu_integrated'](SimpleNamespace(
                            valid=True, integration_time_us=100000, delta_rotation=[0, 0, 0, 1]))
                    if fault != 'mode' or now[0] < 8.5:
                        subscribers['/pidrone/mode'](SimpleNamespace(
                            mode='ARMED' if fault == 'armed' and now[0] > 2 else 'DISARMED'))

                modules = {
                    'time': SimpleNamespace(monotonic=lambda: now[0], sleep=sleep),
                    'rospy': SimpleNamespace(init_node=lambda *a, **kw: None, is_shutdown=lambda: False,
                        Subscriber=lambda topic, cls, callback, **kw: subscribers.update({topic: callback})),
                    'pidrone_pkg.msg': SimpleNamespace(IntegratedImu=object, Mode=object)}
                with mock.patch.dict(sys.modules, modules), mock.patch('builtins.print') as report:
                    if fault:
                        with self.assertRaisesRegex(RuntimeError, 'incomplete/stale'):
                            exec(warm_reboot_fc.GYRO_CHECK, {})
                    else:
                        exec(warm_reboot_fc.GYRO_CHECK, {})
                        self.assertIn('0.000 / 0.000 / 0.000', report.call_args.args[0])


class ConnectTest(unittest.TestCase):
    def test_setup_key_works_with_paramiko_and_preserves_existing_key(self):
        with TemporaryDirectory() as directory:
            key = Path(directory) / 'user with spaces' / '.ssh' / 'pals_ed25519'
            drones.create_ssh_key(key)
            original = key.read_bytes()
            loaded = drones.paramiko.Ed25519Key.from_private_key_file(str(key))
            self.assertEqual(key.with_suffix('.pub').read_text(encoding='utf-8').split()[:2],
                             [loaded.get_name(), loaded.get_base64()])
            drones.create_ssh_key(key)
            self.assertEqual(key.read_bytes(), original)

    def test_dashboard_installs_key_only_after_key_authentication_fails(self):
        existing, rejected, password = mock.Mock(), mock.Mock(), mock.Mock()
        rejected.connect.side_effect = drones.paramiko.AuthenticationException('key rejected')
        stdout = mock.Mock()
        stdout.channel.recv_exit_status.return_value = 0
        password.exec_command.return_value = (None, stdout, None)
        with TemporaryDirectory() as directory, \
                mock.patch.object(drones, 'KEY', Path(directory) / 'pals_ed25519'), \
                mock.patch.object(drones.paramiko, 'SSHClient', side_effect=[existing, rejected, password]), \
                mock.patch.object(drones, 'connect') as verify:
            self.assertIs(drones.setup_ssh('drone1'), existing)
            self.assertIs(drones.setup_ssh('drone2'), verify.return_value)
        existing.exec_command.assert_not_called()
        existing.get_transport.return_value.set_keepalive.assert_called_once_with(15)
        password.exec_command.assert_called_once()
        verify.assert_called_once_with('drone2')
        rejected.close.assert_called_once()
        password.close.assert_called_once()

    def test_dashboard_does_not_try_password_after_connection_errors(self):
        key = drones.paramiko.RSAKey.generate(1024)
        for error in (OSError('offline'), EOFError('connection closed'),
                      drones.paramiko.BadHostKeyException('drone1.local', key, key)):
            with self.subTest(error=type(error).__name__), TemporaryDirectory() as directory, \
                    mock.patch.object(drones, 'KEY', Path(directory) / 'pals_ed25519'), \
                    mock.patch.object(drones.paramiko, 'SSHClient') as ssh:
                client = ssh.return_value
                client.connect.side_effect = error
                with self.assertRaisesRegex(RuntimeError, 'SSH setup failed'):
                    drones.setup_ssh('drone1')
                self.assertNotIn('password', client.connect.call_args.kwargs)
                client.connect.assert_called_once()
                client.exec_command.assert_not_called()
                client.close.assert_called_once()

    def test_failed_command_closes_cached_connection_without_retry(self):
        for error in (OSError('connection lost'), EOFError('connection closed'),
                      drones.paramiko.SSHException('session inactive')):
            with self.subTest(error=type(error).__name__):
                client = mock.Mock()
                client.exec_command.side_effect = error
                with mock.patch.dict(drones._clients, {'drone1': client}, clear=True):
                    with self.assertRaisesRegex(RuntimeError, 'SSH command failed'):
                        drones.run('drone1', 'command')
                    self.assertNotIn('drone1', drones._clients)
                client.exec_command.assert_called_once()
                client.close.assert_called_once()

    def test_connect_button_bootstraps_before_starting_nodes(self):
        for start in (True, False):
            with self.subTest(start=start), \
                    mock.patch.object(drones, 'setup_ssh') as setup, \
                    mock.patch.object(drones, 'connect') as connect:
                client, channel = drones.open_terminal('drone2', 80, 24, start=start)
                expected, unused = (setup, connect) if start else (connect, setup)
                expected.assert_called_once_with('drone2')
                unused.assert_not_called()
                self.assertIs(client, expected.return_value)
                command = channel.exec_command.call_args.args[0]
                self.assertEqual('pals-start.sh start;' in command, start)
                self.assertEqual('git -C ~/catkin_ws/src/pidrone_pkg pull --ff-only && ' in command, start)
        with mock.patch.object(drones, 'setup_ssh', side_effect=RuntimeError('offline')), \
                mock.patch.object(drones, 'connect') as connect:
            with self.assertRaisesRegex(RuntimeError, 'offline'):
                drones.open_terminal('drone2', 80, 24, start=True)
            connect.assert_not_called()
        with mock.patch.object(drones, 'setup_ssh') as setup:
            client = setup.return_value
            channel = client.get_transport.return_value.open_session.return_value
            with self.assertRaisesRegex(RuntimeError, 'Connection canceled'):
                drones.open_terminal('drone2', 80, 24, start=True, active=lambda: False)
            channel.exec_command.assert_not_called()
            client.close.assert_called_once()
        with mock.patch.object(drones, 'setup_ssh') as setup:
            client = setup.return_value
            client.get_transport.return_value.open_session.side_effect = OSError('connection lost')
            with self.assertRaisesRegex(RuntimeError, 'terminal connection failed'):
                drones.open_terminal('drone2', 80, 24, start=True)
            client.close.assert_called_once()

    @unittest.skipUnless(shutil.which('bash'), 'requires bash')
    def test_failed_pull_reports_failure_without_starting_or_attaching(self):
        with mock.patch.object(drones, 'setup_ssh'):
            _, channel = drones.open_terminal('drone2', 80, 24, start=True)
        command = channel.exec_command.call_args.args[0]
        # Stub the remote commands; an empty PATH prevents any real Docker call.
        result = subprocess.run([shutil.which('bash'), '-c',
                                 'git() { return 1; }; bash() { echo unexpected-start; }; ' + command],
                                env={'PATH': ''}, capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, '\033]777;1\007')
        self.assertEqual(result.stderr, '')


if __name__ == '__main__':
    unittest.main()
