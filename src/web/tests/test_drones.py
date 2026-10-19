import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import drones


class ExecuteTest(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.running = 1

        def run(drone, command, timeout=10):
            self.commands.append(command)
            if command.startswith('ls'):
                return 0, 'hover_routine.py\nREADME.md\n'
            if 'pgrep' in command:
                return self.running, ''
            return 0, ''

        patch = mock.patch.object(drones, 'run', side_effect=run)
        patch.start()
        self.addCleanup(patch.stop)

    def test_types_routine_into_screen_window(self):
        drones.execute('drone1', 'hover_routine.py', '--fly')
        self.assertEqual(self.commands[-1],
                         "docker exec pidrone_pkg screen -S pidrone -p routine -X stuff "
                         "'^Upython -u ~/catkin_ws/src/pidrone_pkg/scripts/PALS/routines/hover_routine.py --fly^M'")

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
    def test_missing_key_points_to_ssh_setup(self):
        with mock.patch.object(drones, 'KEY', Path('/nonexistent/pals_ed25519')):
            with self.assertRaisesRegex(RuntimeError, 'run tools/pals.py ssh-setup'):
                drones.connect('drone1')


if __name__ == '__main__':
    unittest.main()
