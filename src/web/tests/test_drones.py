import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import drones
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import pals


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
                         "'^Upython -u ~/catkin_ws/src/pidrone_pkg/scripts/PALS/routines/prototype-1-forward.py --fly --distance 0.25^M'")

    def test_rejects_unsafe_or_unavailable_requests(self):
        for drone, script, args in [('drone3', 'hover_routine.py', ''),
                                    ('drone1', '../flight_controller_node.py', ''),
                                    ('drone1', 'hover_routine.py', "--fly; rm -rf ~"),
                                    ('drone1', 'hover_routine.py', '^C'),
                                    ('drone1', 'basic_routine.py', '--fly')]:
            with self.assertRaises(ValueError):
                drones.execute(drone, script, args)
        for drone, action in [('drone1', 'start'), ('drone1', 'stop; reboot'), ('drone3', 'restart')]:
            with self.assertRaises(ValueError):
                drones.node_action(drone, action)
        self.running = 0
        with self.assertRaisesRegex(RuntimeError, 'already running'):
            drones.execute('drone1', 'hover_routine.py', '--fly')
        with self.assertRaisesRegex(RuntimeError, 'routine is running'):
            drones.node_action('drone1', 'align')
        self.assertFalse(any('stuff' in command or 'pals-start' in command for command in self.commands))


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

    def test_missing_key_points_to_ssh_setup(self):
        with mock.patch.object(drones, 'KEY', Path('/nonexistent/pals_ed25519')):
            with self.assertRaisesRegex(RuntimeError, 'run tools/pals.py ssh-setup'):
                drones.connect('drone1')

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
        with mock.patch.object(drones, 'setup_ssh', side_effect=RuntimeError('offline')), \
                mock.patch.object(drones, 'connect') as connect:
            with self.assertRaisesRegex(RuntimeError, 'offline'):
                drones.open_terminal('drone2', 80, 24, start=True)
            connect.assert_not_called()


class SetupTest(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        patch = mock.patch.object(drones, 'KEY', Path(directory.name) / '.ssh' / 'pals_ed25519')
        patch.start()
        self.addCleanup(patch.stop)

    def test_installs_key_only_when_key_authentication_fails(self):
        existing, rejected, password = mock.Mock(), mock.Mock(), mock.Mock()
        rejected.connect.side_effect = drones.paramiko.AuthenticationException('key rejected')
        stdout = mock.Mock()
        stdout.channel.recv_exit_status.return_value = 0
        password.exec_command.return_value = (None, stdout, None)
        with mock.patch.object(drones.paramiko, 'SSHClient', side_effect=[existing, rejected, password]), \
                mock.patch.object(drones, 'connect') as verify:
            self.assertTrue(pals.ssh_setup(['drone1', 'drone2']))
        existing.exec_command.assert_not_called()
        password.exec_command.assert_called_once()
        self.assertNotIn('password', existing.connect.call_args.kwargs)
        self.assertEqual(password.connect.call_args.kwargs['password'], 'quackquack')
        verify.assert_called_once_with('drone2')
        verify.return_value.close.assert_called_once()
        for client in (existing, rejected, password):
            client.close.assert_called_once()

    def test_connection_errors_do_not_try_password_or_block_other_drone(self):
        key = drones.paramiko.RSAKey.generate(1024)
        for error in (OSError('offline'),
                      drones.paramiko.BadHostKeyException('drone1.local', key, key)):
            with self.subTest(error=type(error).__name__):
                failed, working = mock.Mock(), mock.Mock()
                failed.connect.side_effect = error
                with mock.patch.object(drones.paramiko, 'SSHClient', side_effect=[failed, working]):
                    self.assertFalse(pals.ssh_setup(['drone1', 'drone2']))
                failed.exec_command.assert_not_called()
                working.exec_command.assert_not_called()
                self.assertNotIn('password', failed.connect.call_args.kwargs)
                self.assertEqual(working.connect.call_args.args[0], 'drone2.local')
                failed.close.assert_called_once()
                working.close.assert_called_once()

    def test_dashboard_opens_without_ssh_setup(self):
        with mock.patch.object(sys, 'argv', ['pals.py', 'dashboard']), \
                mock.patch.object(pals, 'ensure_venv'), \
                mock.patch.object(pals, 'ssh_setup', return_value=False) as setup, \
                mock.patch.object(pals, 'dashboard') as dashboard:
            pals.main()
        setup.assert_not_called()
        dashboard.assert_called_once_with(8000)


if __name__ == '__main__':
    unittest.main()
