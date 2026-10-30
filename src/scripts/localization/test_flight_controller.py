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
        self.board.rawIMU = dict(cmd=MultiWii.RAW_IMU, ax=0, ay=0, az=2048,
                                 gx=0, gy=0, gz=0, timestamp=0)
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
            if code == MultiWii.RAW_IMU:
                self.board.rawIMU['timestamp'] = self.env.ros_now
            return {MultiWii.ATTITUDE: self.board.attitude,
                    MultiWii.RAW_IMU: self.board.rawIMU,
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
        sys.modules['pidrone_pkg.msg'].MspImu = NS
        sys.modules['sensor_msgs.msg'].Imu = lambda: NS(
            orientation=NS(x=0., y=0., z=0., w=1.),
            angular_velocity=fusion_helpers.vector(), linear_acceleration=fusion_helpers.vector())
        with contextlib.redirect_stdout(io.StringIO()):
            namespace = runpy.run_path(os.path.join(root, 'flight_controller_node.py'))
        self.main = namespace['main']
        self.controller_class = namespace['FlightController']
        self.controller_class.preflight_callback.__globals__['monotonic'] = lambda: self.env.now
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

    def controller(self):
        with contextlib.redirect_stdout(io.StringIO()):
            return self.controller_class()

    def readiness(self, fc, **changes):
        report = NS(header=NS(stamp=fusion_helpers.Stamp(self.env.ros_now)),
                    mode=fc.curr_mode, paired=True, ready=True, reason='ready')
        report.__dict__.update(changes)
        fc.preflight_callback(report)
        return report

    def test_arm_and_takeoff_require_readiness_but_disarm_does_not(self):
        fc = self.controller()
        fc.coordination_selection_callback(NS(data=True))
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'DISARMED')
        self.readiness(fc, ready=False, reason='uncertainty_too_large')
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'DISARMED')
        self.env.ros_now += .01
        ground = self.readiness(fc)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'DISARMED', 'direct takeoff cannot bypass arming')
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'ARMED')
        self.assertEqual(fc.command, self.main.__globals__['cmds'].arm_cmd)
        fc.preflight_callback(ground)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'ARMED', 'ground report cannot authorize takeoff')
        self.env.ros_now += .01
        self.readiness(fc)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'FLYING')
        self.readiness(fc, ready=False, reason='peer_state_stale')
        self.assertEqual(fc.curr_mode, 'FLYING', 'readiness loss must not disarm in flight')
        fc.desired_mode_callback(NS(mode='DISARMED'))
        self.assertEqual(fc.command, self.main.__globals__['cmds'].disarm_cmd)
        self.assertEqual(self.commands, [], 'mode callbacks should not send serial commands')

    def test_stale_or_repeated_reports_do_not_restore_readiness(self):
        fc = self.controller()
        fc.coordination_selection_callback(NS(data=True))
        grant = self.readiness(fc)
        self.assertEqual(fc.preflight_failure(), '')
        self.readiness(fc, ready=False, reason='peer_state_stale')
        fc.preflight_callback(grant)
        self.assertEqual(fc.preflight_failure(), 'peer_state_stale')
        self.env.ros_now += .01
        grant = self.readiness(fc)
        self.env.now += .51  # Let the report expire without advancing ROS time.
        fc.preflight_callback(grant)
        self.assertEqual(fc.preflight_failure(), 'preflight_unavailable')
        self.env.ros_now += .01
        self.readiness(fc)
        self.env.ros_now += .51  # The source timestamp must also be fresh.
        self.assertEqual(fc.preflight_failure(), 'preflight_unavailable')

    def test_solo_readiness_matches_selection_and_locks_when_armed(self):
        fc = self.controller()
        self.readiness(fc, paired=False)
        self.assertEqual(fc.preflight_failure(), 'flight_setup_unavailable')
        fc.coordination_selection_callback(NS(data=True))
        self.env.ros_now += .01
        self.readiness(fc, paired=False)
        self.assertEqual(fc.preflight_failure(), 'flight_setup_changed')
        fc.coordination_selection_callback(NS(data=False))
        self.env.ros_now += .01
        self.readiness(fc, paired=False)
        self.env.now += 2.1
        self.env.ros_now += .01
        self.readiness(fc, paired=False)
        self.assertEqual(fc.preflight_failure(), 'flight_setup_unavailable')
        fc.coordination_selection_callback(NS(data=False))
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'ARMED')
        fc.coordination_selection_callback(NS(data=True))
        self.readiness(fc, paired=False)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'FLYING')

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
