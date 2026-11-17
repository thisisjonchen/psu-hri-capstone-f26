"""Exercise the FC serial loop without opening a serial port or running ROS."""
import contextlib
import errno
import io
import os
import runpy
import select
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from threading import Lock
from unittest import mock

from flight.multiwii import MultiWii
import test_fusion as fusion_helpers


DISARM_COMMAND = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]


class MspPacketTest(unittest.TestCase):
    def test_poll_and_shutdown_packets_on_a_fake_serial_port(self):
        ack = bytes.fromhex('244d3e00c8c8')
        imu = bytes.fromhex(
            '24 4d 3e 46 ff 00 01 7f 40 00 01 01 01 00 00 00 09 00 00 00 '
            '10 27 00 00 10 27 00 00 00 00 00 00 00 00 00 00 00 00 00 00 '
            '00 00 80 3f 00 00 00 00 00 00 00 00 39 b4 c8 3d 00 00 00 00 '
            '00 08 00 00 00 00 00 00 00 00 00 00 67 01 3c 1a'
        )
        analog = bytes.fromhex('244d3e096e78000000002a00000035')
        short_analog = bytes.fromhex('244d3e016e7817')
        unsupported = bytes.fromhex('244d3e006464')
        incoming = io.BytesIO(imu + short_analog + unsupported + analog + ack)
        writes = []
        serial = NS(write=writes.append, read=incoming.read,
                    inWaiting=lambda: len(incoming.getvalue()) - incoming.tell())
        with mock.patch('flight.multiwii.serial.Serial', return_value=serial):
            board = MultiWii('fake')
        board.send_commands([(MultiWii.INTEGRATED_IMU, [1, 0]), (MultiWii.ANALOG, [])])
        replies = board.receive_available()
        code, sample = next(replies)
        self.assertEqual(code, MultiWii.INTEGRATED_IMU)
        self.assertEqual((sample['request'], sample['sequence'], sample['duration_us']), (1, 9, 10000))
        self.assertAlmostEqual(sample['velocity'][2], .098)
        attitude, raw = sample['legacy']
        self.assertEqual((attitude['heading'], raw['az']), (359, 2048))
        self.assertEqual(next(replies), (MultiWii.ANALOG, None))
        self.assertEqual(next(replies), (100, None))
        code, battery = next(replies)
        self.assertEqual(code, MultiWii.ANALOG)
        self.assertEqual((battery['vbat'], battery['amperage']), (120, 42))

        board.send_commands([(MultiWii.SET_RAW_RC, DISARM_COMMAND)])
        self.assertEqual(board.receiveDataPacket(), 'Set Raw RC')
        self.assertEqual(writes, [
            bytes.fromhex('244d3c0eff00017f0800010000000000000060e6 244d3c006e6e0a'),
            bytes.fromhex('244d3c10c8dc05dc05dc05e803e803e803e803e803ea'),
        ])


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
        self.incomplete_legacy = False
        self.sequence, self.previous_us = 0, None
        self.requests = {}
        self.board = MultiWii.__new__(MultiWii)
        self.board.attitude = dict(cmd=MultiWii.ATTITUDE, angx=0, angy=0, heading=0)
        self.board.rawIMU = dict(cmd=MultiWii.RAW_IMU, ax=0, ay=0, az=2048,
                                 gx=0, gy=0, gz=0, timestamp=0)
        self.board.analog = dict(vbat=120, amperage=0)

        def send(code, data):
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
                samples = [self.board.attitude, self.board.rawIMU]
                if self.incomplete_legacy and self.sequence == 1:
                    samples.pop()
                return dict(cmd=code, request=self.requests[code], valid=True, sequence=self.sequence, end_us=end, duration_us=duration,
                            rotation=(0.,0.,0.,1.), velocity=(0.,0.,9.8*duration*1e-6),
                            timestamp=self.env.ros_now, legacy=samples)
            return {MultiWii.SET_RAW_RC: 'Set Raw RC',
                    MultiWii.ATTITUDE: self.board.attitude,
                    MultiWii.RAW_IMU: self.board.rawIMU,
                    MultiWii.ANALOG: self.board.analog}.get(code)

        self.board.create_packet = lambda code, data: send(code, data) or b''
        self.board.serial_port_write_lock = Lock()
        self.board.ser = NS(write=lambda packet: None)
        self.board.receiveDataPacket = receive
        def available():
            while self.pending:
                code = self.pending[0]
                yield code, self.board.receiveDataPacket()
        self.board.receive_available = available
        # Use the real command batching and FC main loop.
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
        rospkg = ModuleType('rospkg')
        rospkg.RosPack = lambda: NS(get_path=lambda name: root)
        patches = mock.patch.dict(sys.modules, {'rospkg': rospkg})
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
            namespace = runpy.run_path(os.path.join(root, 'flight', 'flight_controller_node.py'))
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

        namespace['main'].__globals__['select'] = NS(select=wait, error=select.error)

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

    def test_height_does_not_disarm_but_heartbeat_timeouts_do(self):
        fc = self.controller()
        fc.battery_message.vbat = 16.0
        fc.heartbeat_web_interface = fc.heartbeat_pid_controller = 10.0
        fc.heartbeat_infrared = fc.heartbeat_state_estimator = 10.0
        with mock.patch.object(self.env.ros, 'Time', NS(now=lambda: 10.0)), \
                mock.patch.object(self.env.ros, 'Duration', NS(from_sec=float), create=True), \
                contextlib.redirect_stdout(io.StringIO()):
            fc.heartbeat_infrared_callback(NS(range=2.0))
            self.assertFalse(fc.shouldIDisarm())
            for heartbeat in ('heartbeat_web_interface', 'heartbeat_pid_controller',
                              'heartbeat_infrared', 'heartbeat_state_estimator'):
                with self.subTest(heartbeat=heartbeat):
                    setattr(fc, heartbeat, 6.9)
                    self.assertTrue(fc.shouldIDisarm())
                    setattr(fc, heartbeat, 10.0)

    def test_invalid_disarm_preset_is_rejected_before_opening_serial(self):
        namespace = self.main.__globals__
        settings = namespace['load_flight_settings']()
        for invalid in ([1500] * 7, [1500] * 7 + [1000.0], [1500] * 7 + [999]):
            with self.subTest(invalid=invalid), \
                    mock.patch.dict(namespace, {'load_flight_settings': lambda: settings}), \
                    mock.patch.object(self.controller_class, 'getBoard') as open_board:
                settings['rc']['disarm'] = invalid
                with self.assertRaisesRegex(ValueError, 'RC preset disarm'):
                    self.controller()
                open_board.assert_not_called()

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
        self.assertEqual(fc.command, [1500, 1500, 1500, 1000, 1800, 1000, 1000, 1000])
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.command, [1500, 1500, 1500, 1000, 1800, 1800, 1000, 1000])
        fc.preflight_callback(ground)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'ARMED', 'ground report cannot authorize takeoff')
        self.env.ros_now += .01
        self.readiness(fc)
        fc.desired_mode_callback(NS(mode='FLYING'))
        self.assertEqual(fc.curr_mode, 'FLYING')
        fc.fly_commands_callback(NS(roll=1502., pitch=1498., yaw=1500., throttle=1540.))
        flight_command = list(fc.command)
        self.assertEqual(flight_command, [1502, 1498, 1500, 1540, 1800, 1800, 1000, 1000])
        fc.desired_mode_callback(NS(mode='ARMED'))
        self.assertEqual(fc.curr_mode, 'FLYING', 'an arm request must not end flight control')
        self.assertEqual(fc.command, flight_command)
        self.readiness(fc, ready=False, reason='peer_state_stale')
        self.assertEqual(fc.curr_mode, 'FLYING', 'readiness loss must not disarm in flight')
        fc.desired_mode_callback(NS(mode='DISARMED'))
        fc.fly_commands_callback(NS(roll=1502., pitch=1498., yaw=1500., throttle=1540.))
        self.assertEqual(fc.command, DISARM_COMMAND)
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
        polling = self.main.__globals__['select']
        wait = polling.select
        def interrupted_once(*args):
            polling.select = wait
            raise select.error(errno.EINTR, 'Interrupted system call')
        polling.select = interrupted_once
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
        self.assertEqual(self.commands[-1], DISARM_COMMAND)
        self.assertEqual(self.pending, [])

    def test_shutdown_interrupt_exits_cleanly_and_disarms(self):
        def interrupt(*args):
            self.env.ros.is_shutdown = lambda: True
            raise select.error(errno.EINTR, 'Interrupted system call')
        self.main.__globals__['select'].select = interrupt
        output = self.run_loop()
        self.assertNotIn('internal error', output)
        self.assertIn('Sending DISARM command', output)
        self.assertEqual(self.commands[-1], DISARM_COMMAND)

    def test_missing_raw_counts_skips_legacy_telemetry_and_keeps_polling(self):
        self.incomplete_legacy = True
        output = self.run_loop()
        self.assertNotIn('internal error', output)
        self.assertTrue(self.env.ros.is_shutdown())
        self.assertGreater(len(self.env.messages['/pidrone/imu']), 50)
        self.assertEqual(len(self.env.messages['/pidrone/imu_msp']),
                         len(self.env.messages['/pidrone/imu_integrated']) - 1)

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

    def test_attitude_with_no_ros_time_elapsed_keeps_rates_finite(self):
        fc = self.controller()
        fc.update_imu_message(dict(angx=1., angy=-2., heading=3.), self.board.rawIMU)
        rate = fc.imu_message.angular_velocity
        self.assertEqual((rate.x, rate.y, rate.z), (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
