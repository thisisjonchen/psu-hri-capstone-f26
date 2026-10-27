"""Exercise the FC serial loop without opening a serial port or running ROS."""
import contextlib
import io
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

from h2rMultiWii import MultiWii
from localization import test_fusion as fusion_helpers


class FlightControllerLoopTest(unittest.TestCase):
    def setUp(self):
        self.env = fusion_helpers.FusionNodeTest()
        self.env.setUp()
        self.addCleanup(self.env.doCleanups)
        ros = self.env.ros
        self.events, self.pending = [], []
        self.commands = []
        self.missing_reply = None
        self.board = MultiWii.__new__(MultiWii)
        self.board.attitude = dict(cmd=MultiWii.ATTITUDE, angx=0, angy=0, heading=0)
        self.board.rawIMU = dict(cmd=MultiWii.RAW_IMU, ax=0, ay=0, az=2048, gx=1, gy=2, gz=3)
        self.board.motor = dict(cmd=MultiWii.MOTOR, m1=1100., m2=1200., m3=1300., m4=1400.)
        self.board.analog = dict(vbat=120, amperage=0)

        def send(size, code, data):
            self.events.append(('send', code, self.env.ros_now))
            if code == MultiWii.SET_RAW_RC:
                self.commands.append(list(data))
            self.pending.append(code)

        def receive():
            code = self.pending.pop(0)
            self.events.append(('read', code, self.env.ros_now))
            self.env.ros_now += .002  # Simulate time spent waiting for a reply.
            if code == self.missing_reply:
                return None
            return {MultiWii.ATTITUDE: self.board.attitude,
                    MultiWii.RAW_IMU: self.board.rawIMU,
                    MultiWii.MOTOR: self.board.motor,
                    MultiWii.ANALOG: self.board.analog}.get(code)

        self.board.send_raw_command = send
        self.board.receiveDataPacket = receive
        # Use the real getData/getDataBulk methods and the real FC main loop.
        root = os.path.dirname(os.path.dirname(__file__))
        rospkg, yaml = ModuleType('rospkg'), ModuleType('yaml')
        rospkg.RosPack = lambda: NS(get_path=lambda name: os.path.dirname(root))
        yaml.load = lambda stream: dict(az=2048., ax=0., ay=0.)
        patches = mock.patch.dict(sys.modules, {'rospkg': rospkg, 'yaml': yaml})
        patches.start()
        self.addCleanup(patches.stop)
        sys.modules['std_msgs.msg'].Header = sys.modules['std_msgs.msg'].String = NS
        sys.modules['geometry_msgs.msg'].Quaternion = NS
        sys.modules['pidrone_pkg.msg'].Battery = NS
        sys.modules['pidrone_pkg.msg'].FlightRaw = lambda: NS(header=NS())
        self.raw_subscribers = 0
        publisher = ros.Publisher

        def counted(topic, *args, **kwargs):
            result = publisher(topic, *args, **kwargs)
            result.get_num_connections = lambda: self.raw_subscribers if topic == '/pidrone/flight_raw' else 0
            return result
        ros.Publisher = counted
        sys.modules['sensor_msgs.msg'].Imu = lambda: NS(
            orientation=NS(x=0., y=0., z=0., w=1.),
            angular_velocity=fusion_helpers.vector(), linear_acceleration=fusion_helpers.vector())
        with contextlib.redirect_stdout(io.StringIO()):
            namespace = runpy.run_path(os.path.join(root, 'flight_controller_node.py'))
        self.main = namespace['main']
        patched_board = mock.patch.object(namespace['FlightController'], 'getBoard', return_value=self.board)
        patched_board.start()
        self.addCleanup(patched_board.stop)
        start = self.env.ros_now
        ros.is_shutdown = lambda: self.env.ros_now >= start + 1.2

        def sleep():
            self.env.ros_now += 1 / 60.

        ros.Rate = lambda hz: NS(sleep=sleep)

    def run_loop(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.main()
        return output.getvalue()

    def test_commands_precede_batched_imu_and_battery_stays_fresh_at_reduced_rate(self):
        output = self.run_loop()
        self.assertNotIn('internal error', output)
        transactions = [(kind, code) for kind, code, _ in self.events]
        expected = [('send', MultiWii.SET_RAW_RC), ('read', MultiWii.SET_RAW_RC),
                    ('send', MultiWii.ATTITUDE), ('send', MultiWii.RAW_IMU),
                    ('read', MultiWii.ATTITUDE), ('read', MultiWii.RAW_IMU)]
        imu = self.env.messages['/pidrone/imu']
        self.assertEqual(len(self.commands) - 1, len(imu))  # Exclude shutdown disarm.
        for i, event in enumerate(transactions[:-2]):  # Final pair is shutdown disarm.
            if event == ('send', MultiWii.SET_RAW_RC):
                self.assertEqual(transactions[i:i+6], expected)
        battery_times = [t for kind, code, t in self.events if kind == 'send' and code == MultiWii.ANALOG]
        self.assertGreaterEqual(len(battery_times), 2)
        for first, second in zip(battery_times, battery_times[1:]):
            self.assertGreaterEqual(second - first, .5)
            self.assertLess(second - first, 1.)
        self.assertEqual(len(self.env.messages['/pidrone/battery']), len(battery_times))
        self.assertEqual(transactions[-2:], expected[:2])

    def test_raw_log_reads_motors_in_the_same_bulk_request_only_while_subscribed(self):
        self.raw_subscribers = 1
        output = self.run_loop()
        self.assertNotIn('internal error', output)
        imu, raw = self.env.messages['/pidrone/imu'], self.env.messages['/pidrone/flight_raw']
        self.assertGreater(len(raw), 30)
        self.assertEqual(len(raw), len(imu))
        self.assertEqual([m.header.stamp.to_sec() for m in raw], [m.header.stamp.to_sec() for m in imu])
        self.assertEqual((raw[-1].accel, raw[-1].gyro, raw[-1].motors),
                         ([0, 0, 2048], [1, 2, 3], [1100, 1200, 1300, 1400]))
        transactions = [(kind, code) for kind, code, _ in self.events]
        bulk = [('send', MultiWii.ATTITUDE), ('send', MultiWii.RAW_IMU), ('send', MultiWii.MOTOR),
                ('read', MultiWii.ATTITUDE), ('read', MultiWii.RAW_IMU), ('read', MultiWii.MOTOR)]
        self.assertEqual(transactions[2:8], bulk)

    def test_missing_bulk_reply_skips_imu_and_keeps_loop_running_until_shutdown(self):
        self.missing_reply = MultiWii.RAW_IMU
        output = self.run_loop()
        self.assertNotIn('internal error', output)
        self.assertEqual(self.env.messages['/pidrone/imu'], [])
        self.assertTrue(self.env.ros.is_shutdown())
        # Preserve the baseline disarm on actual node shutdown.
        self.assertEqual([(kind, code) for kind, code, _ in self.events[-2:]],
                         [('send', MultiWii.SET_RAW_RC), ('read', MultiWii.SET_RAW_RC)])
        self.assertEqual(self.commands[-1], self.main.__globals__['cmds'].disarm_cmd)
        self.assertEqual(self.pending, [])

    def test_turning_through_north_reports_the_turn_rate_not_a_spin(self):
        receive = self.board.receiveDataPacket

        def turning():  # 1 degree per reply, crossing 359 -> 0 degrees.
            reply = receive()
            if reply is self.board.attitude:
                self.board.attitude = dict(reply, heading=(reply['heading'] + 1) % 360)
            return reply
        self.board.attitude['heading'] = 340
        self.board.receiveDataPacket = turning
        self.run_loop()
        rates = [m.angular_velocity.z for m in self.env.messages['/pidrone/imu'][1:]]
        self.assertGreater(len(rates), 30)
        self.assertLess(max(abs(r) for r in rates), 3, 'heading wrap read as a spin')
        self.assertTrue(all(r < 0 for r in rates if abs(r) > 1e-3), 'one steady turning direction')


if __name__ == '__main__':
    unittest.main()
