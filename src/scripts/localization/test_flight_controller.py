"""Exercise the FC serial loop without opening a serial port or running ROS."""
import contextlib
import io
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from threading import Lock
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
        publisher = ros.Publisher

        def make_publisher(topic, *args, **kwargs):
            pub = publisher(topic, *args, **kwargs)
            pub.get_num_connections = lambda: 1
            if topic == '/pidrone/imu_integrated':
                publish = pub.publish

                def publish_integrated(message):
                    self.events.append(('publish', MultiWii.INTEGRATED_IMU, self.env.ros_now))
                    publish(message)

                pub.publish = publish_integrated
            return pub

        ros.Publisher = make_publisher
        self.commands = []
        self.missing_reply = None
        self.sequence, self.previous_us = 0, None
        self.requests = {}
        self.board = MultiWii.__new__(MultiWii)
        self.board.attitude = dict(cmd=MultiWii.ATTITUDE, angx=0, angy=0, heading=0)
        self.board.rawIMU = dict(cmd=MultiWii.RAW_IMU, ax=0, ay=0, az=2048,
                                 gx=0, gy=0, gz=0, timestamp=0)
        self.board.analog = dict(vbat=120, amperage=0)

        def send(size, code, data):
            self.events.append(('send', code, self.env.ros_now))
            if code == MultiWii.INTEGRATED_IMU:
                self.assertEqual(data[1], self.sequence, 'Received interval must be acknowledged before the next request')
                self.requests[code] = data[0]
            if code == MultiWii.SET_RAW_RC:
                self.commands.append(list(data))
            self.pending.append(code)

        def receive():
            code = self.pending.pop(0)
            self.events.append(('read', code, self.env.ros_now))
            self.env.ros_now += .002  # Simulate time spent waiting for a reply.
            self.env.now += .002
            if code == self.missing_reply:
                return None
            if code == MultiWii.INTEGRATED_IMU:
                self.board.rawIMU['timestamp'] = self.env.ros_now
                end = int(self.env.ros_now*1e6) & 0xffffffff
                duration = 10000 if self.previous_us is None else (end-self.previous_us) & 0xffffffff
                self.sequence += 1
                self.previous_us = end
                return dict(cmd=code, request=self.requests[code], valid=True, sequence=self.sequence, end_us=end, duration_us=duration,
                            rotation=(0.,0.,0.,1.), velocity=(0.,0.,9.8*duration*1e-6),
                            timestamp=self.env.ros_now, legacy=[self.board.attitude,self.board.rawIMU])
            return {MultiWii.SET_RAW_RC: 'Set Raw RC',
                    MultiWii.ATTITUDE: self.board.attitude,
                    MultiWii.RAW_IMU: self.board.rawIMU,
                    MultiWii.ANALOG: self.board.analog}.get(code)

        self.board.send_raw_command = send
        self.board.create_packet = lambda size, code, data: send(size, code, data) or ''
        self.board.serial_port_write_lock = Lock()
        self.board.ser = NS(write=lambda packet: None)
        self.board.receiveDataPacket = receive
        def available():
            while self.pending:
                code = self.pending[0]
                yield code, self.board.receiveDataPacket()
        self.board.receive_available = available
        # Use the real command batching and FC main loop.
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
        sys.modules['pidrone_pkg.msg'].MspImu = sys.modules['pidrone_pkg.msg'].IntegratedImu = NS
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
        ros.is_shutdown = lambda: self.env.ros_now >= start + 1.2 and not self.pending

        def wait(readers, writers, errors, timeout):
            if not self.pending:
                self.env.ros_now += timeout
                self.env.now += timeout
            return (readers if self.pending else [], [], [])

        namespace['main'].__globals__['select'] = NS(select=wait)

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
        fc.fly_commands_callback(NS(roll=1502., pitch=1498., yaw=1500., throttle=1540.))
        flight_command = list(fc.command)
        fc.desired_mode_callback(NS(mode='ARMED'))
        fc.update_command()
        self.assertEqual(fc.curr_mode, 'FLYING', 'an arm request must not end flight control')
        self.assertEqual(fc.command, flight_command)
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
        for i, transaction in enumerate(transactions):
            if transaction == ('read', MultiWii.INTEGRATED_IMU):
                self.assertEqual(transactions[i + 1], ('publish', MultiWii.INTEGRATED_IMU))
        imu = self.env.messages['/pidrone/imu']
        self.assertGreater(len(self.env.messages['/pidrone/imu_msp']), len(imu))
        self.assertGreater(len(imu), 50)
        batches = {}
        for kind, code, stamp in self.events[:-2]:
            if kind == 'send':
                batches.setdefault(stamp, []).append(code)
        for batch in batches.values():
            if MultiWii.SET_RAW_RC in batch:
                self.assertEqual(batch[0], MultiWii.SET_RAW_RC)
        rc_times = [t for kind, code, t in self.events[:-2] if kind == 'send' and code == MultiWii.SET_RAW_RC]
        for first, second in zip(rc_times, rc_times[1:]):
            self.assertLess(second - first, .03)
        battery_times = [t for kind, code, t in self.events if kind == 'send' and code == MultiWii.ANALOG]
        self.assertGreaterEqual(len(battery_times), 2)
        for first, second in zip(battery_times, battery_times[1:]):
            self.assertGreaterEqual(second - first, .5)
            self.assertLess(second - first, 1.)
        self.assertEqual(len(self.env.messages['/pidrone/battery']), len(battery_times))
        self.assertEqual(transactions[-2:],
                         [('send', MultiWii.SET_RAW_RC), ('read', MultiWii.SET_RAW_RC)])

    def test_missing_interval_reply_skips_imu_and_keeps_loop_running_until_shutdown(self):
        self.missing_reply = MultiWii.INTEGRATED_IMU
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
            if isinstance(reply, dict) and reply.get('cmd') == MultiWii.INTEGRATED_IMU:
                self.board.attitude = dict(self.board.attitude,
                                           heading=(self.board.attitude['heading'] + 1) % 360)
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
