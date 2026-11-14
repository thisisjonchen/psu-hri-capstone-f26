#!/usr/bin/env python
from collections import namedtuple
import errno
import json
import os
import select
import threading
import traceback

import numpy as np
import rospy
import rospkg
import tf
import yaml

from serial import SerialException
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Header, Empty, Bool, String
from pidrone_pkg.msg import Battery, Mode, RC, State, FlightReadiness, MspImu, IntegratedImu
from flight import command_values as cmds
from flight.multiwii import MultiWii
from localization import monotonic
from flight.distance_guard import fresh, STATE_TIMEOUT, SELECTION_TIMEOUT
from flight.imu_clock import ImuClock


ImuRequest = namedtuple('ImuRequest', 'started sent publish_attitude request_id')


class FlightController(object):
    """Send RC commands over MSP and publish flight-controller telemetry."""

    def __init__(self):
        self.board = self.getBoard()
        self.curr_mode = 'DISARMED'
        self.mode_lock = threading.RLock()
        self.preflight = self.preflight_received = self.preflight_stamp = None
        self.coordination_selection = self.selection_received = None
        self.flight_paired = None
        self.calibration_owner = rospy.get_param('/pidrone/localization/calibration_owner', '')
        self.calibration_request = ''
        # Clear the retired flight lock when upgrading an existing ROS session.
        rospy.set_param('/pidrone/localization/calibration_lock', False)
        self.command = cmds.disarm_cmd
        self.last_command = cmds.disarm_cmd
        now = monotonic()
        self.next_rc = self.next_imu = self.next_attitude = self.next_battery = now
        self.imu_pending = self.battery_pending = None
        self.imu_clock = ImuClock()
        self.imu_request = self.imu_ack = 0
        self.integrated_started = False
        self.last_rc_ack = now
        self.time = rospy.Time.now()

        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()

        self.imu_message = Imu()
        self.imu_message.header = header

        self.battery_message = Battery()
        self.battery_message.vbat = None
        self.battery_message.amperage = None
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open(os.path.join(path, 'params', 'multiwii.yaml')) as stream:
            means = yaml.safe_load(stream)
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX = means["ax"] * self.accRawToMss
        self.accZeroY = means["ay"] * self.accRawToMss
        self.accZeroZ = means["az"] * self.accRawToMss

    def desired_mode_callback(self, msg):
        """Check readiness before accepting an arm or takeoff request."""
        with self.mode_lock:
            if msg.mode not in ('DISARMED', 'ARMED', 'FLYING'):
                return
            if self.curr_mode == 'FLYING' and msg.mode == 'ARMED':
                rospy.logwarn_throttle(1, 'Ignoring arm request while flying')
                return
            if (msg.mode in ('ARMED', 'FLYING') and self.curr_mode != 'FLYING'
                    and msg.mode != self.curr_mode):
                reason = self.preflight_failure()
                if self.curr_mode == 'DISARMED' and msg.mode == 'FLYING':
                    reason = 'arm_before_takeoff'
                if reason:
                    rospy.logwarn_throttle(1, 'Refusing %s: %s' % (msg.mode, reason))
                    return
                if self.curr_mode == 'DISARMED':
                    self.flight_paired = self.preflight.paired
            previous = self.curr_mode
            if previous != msg.mode:
                self.preflight = self.preflight_received = self.preflight_stamp = None
            if msg.mode == 'DISARMED':
                self.flight_paired = None
                self.command = cmds.disarm_cmd
            elif msg.mode == 'ARMED':
                self.command = cmds.arm_cmd if previous == 'DISARMED' else cmds.idle_cmd
            self.curr_mode = msg.mode

    def coordination_selection_callback(self, msg):
        with self.mode_lock:
            if self.curr_mode == 'DISARMED':
                if self.coordination_selection != msg.data:
                    self.preflight = self.preflight_received = None
                self.coordination_selection, self.selection_received = msg.data, monotonic()

    def preflight_callback(self, msg):
        with self.mode_lock:
            stamp = msg.header.stamp.to_sec()
            new_sample = self.preflight_stamp is None or stamp > self.preflight_stamp
            if msg.ready and not new_sample:
                return
            if new_sample:
                self.preflight_stamp, self.preflight_received = stamp, monotonic()
            # A refusal still takes effect when its timestamp has not changed.
            self.preflight = msg

    def preflight_failure(self):
        report = self.preflight
        if (report is None or self.preflight_received is None
                or not fresh(report.header.stamp.to_sec(), rospy.get_time())
                or not 0 <= monotonic() - self.preflight_received <= STATE_TIMEOUT
                or report.mode != self.curr_mode):
            return 'preflight_unavailable'
        if self.curr_mode == 'DISARMED':
            if self.selection_received is None or not 0 <= monotonic() - self.selection_received <= SELECTION_TIMEOUT:
                return 'flight_setup_unavailable'
            paired = self.coordination_selection
        else:
            paired = self.flight_paired
        if report.paired != paired:
            return 'flight_setup_changed'
        return '' if report.ready else (report.reason or 'preflight_not_ready')

    def calibration_owner_callback(self, msg):
        try:
            if len(msg.data) > 1024:
                return
            request = json.loads(msg.data)
            owner, action, nonce = request['owner'], request['action'], request['request']
            if not isinstance(owner, type(u'')) or not owner or len(owner) > 128:
                return
            if not isinstance(nonce, type(u'')) or not nonce or len(nonce) > 128 or action not in ('acquire', 'release'):
                return
            with self.mode_lock:
                self.calibration_request = nonce
                if action == 'acquire' and self.curr_mode != 'DISARMED':
                    return
                if action == 'acquire':
                    if self.calibration_owner != owner and self.calibration_owner:
                        return
                    rospy.set_param('/pidrone/localization/calibration_owner', owner)
                    self.calibration_owner = owner
                elif action == 'release' and self.calibration_owner == owner:
                    rospy.set_param('/pidrone/localization/calibration_owner', '')
                    self.calibration_owner = ''
        except (ValueError, KeyError, TypeError):
            return

    def fly_commands_callback(self, msg):
        with self.mode_lock:
            if self.curr_mode == 'FLYING':
                self.command = [msg.roll, msg.pitch, msg.yaw, msg.throttle] + cmds.idle_cmd[4:]

    @staticmethod
    def next_poll(deadline, now, hz):
        return deadline + (int(max(0., now - deadline) * hz) + 1) / hz

    def poll(self, publish_integrated):
        """Schedule RC and telemetry independently on the same serial connection."""
        now = monotonic()
        received = []
        for code, reply in self.board.receive_available():
            if code == MultiWii.SET_RAW_RC:
                if reply == 'Set Raw RC':
                    self.last_rc_ack = now
                else:
                    rospy.logwarn_throttle(1, 'Invalid RC acknowledgment from flight controller')
            elif code == MultiWii.INTEGRATED_IMU:
                pending = self.imu_pending
                if pending is not None:
                    if isinstance(reply, dict):
                        if reply.get('request') != pending.request_id:
                            continue
                        reply['sent'] = pending.sent
                        reply['publish_attitude'] = pending.publish_attitude
                        # Acknowledge this interval before scheduling the next request.
                        integrated = self.integrated_imu_sample(reply)
                        if integrated is not None:
                            publish_integrated(integrated)
                    self.imu_pending = None
                    received.append((code, reply))
            elif code == MultiWii.ANALOG and self.battery_pending is not None:
                self.battery_pending = None
                received.append((code, reply))

        now = monotonic()
        if self.imu_pending is not None and now - self.imu_pending.started >= .05:
            rospy.logwarn_throttle(1, 'IMU response timed out; skipping sample')
            self.imu_pending = None
        if self.battery_pending is not None and now - self.battery_pending >= 1.:
            rospy.logwarn_throttle(1, 'Battery response timed out; skipping sample')
            self.battery_pending = None
        if now - self.last_rc_ack >= 1.:
            rospy.logwarn_throttle(1, 'Missing RC acknowledgment from flight controller')

        requests = []
        command = self.command
        rc_due = now >= self.next_rc
        command_sent = rc_due or command != self.last_command
        if command_sent:
            assert len(command) == 8, "COMMAND HAS WRONG SIZE, expected 8, got " + str(len(command))
            requests.append((MultiWii.SET_RAW_RC, command))
            if rc_due:
                self.next_rc = self.next_poll(self.next_rc, now, 60.)
        if now >= self.next_imu and self.imu_pending is None:
            attitude_due = now >= self.next_attitude
            if attitude_due:
                self.next_attitude = self.next_poll(self.next_attitude, now, 60.)
            self.imu_request = (self.imu_request+1) & 0xffffffff
            requests.append((MultiWii.INTEGRATED_IMU, [self.imu_request, self.imu_ack]))
            self.imu_pending = ImuRequest(now, rospy.Time.now().to_sec(), attitude_due, self.imu_request)
            self.next_imu = self.next_poll(self.next_imu, now, 100.)
        if now >= self.next_battery and self.battery_pending is None:
            requests.append((MultiWii.ANALOG, []))
            self.battery_pending = now
            self.next_battery = now + .5
        if requests:
            self.board.send_commands(requests)
        if command_sent:
            self.last_command = command
        return command_sent, received

    def update_imu_message(self, attitude, raw):
        """Build the legacy IMU message from a validated MSP reply."""
        # Convert the FC's attitude to radians with counterclockwise yaw.
        roll = np.deg2rad(attitude['angx'])
        pitch = -np.deg2rad(attitude['angy'])
        heading = -np.deg2rad(attitude['heading']) % (2 * np.pi)
        previous_quaternion = self.imu_message.orientation
        quaternion_array = [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
        previous_roll, previous_pitch, previous_heading = tf.transformations.euler_from_quaternion(quaternion_array)

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        lin_acc_x = raw['ax'] * self.accRawToMss - self.accZeroX
        lin_acc_y = raw['ay'] * self.accRawToMss - self.accZeroY
        lin_acc_z = raw['az'] * self.accRawToMss - self.accZeroZ

        # Convert forward/left/up to right/forward/up.
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        # Remove gravity after applying the level calibration offsets.
        g = 9.8
        roll_cos, pitch_cos = np.cos(roll), np.cos(pitch)
        lin_acc_x_drone_body += g*np.sin(roll)*pitch_cos
        lin_acc_y_drone_body -= g*roll_cos*np.sin(pitch)
        lin_acc_z_drone_body += g*(1 - roll_cos*pitch_cos)

        time = rospy.Time.now()
        dt = time.to_sec() - self.time.to_sec()
        dr = roll - previous_roll
        dp = pitch - previous_pitch
        # Wrap to [-pi, pi]: crossing north must not read as a full-turn spin.
        dh = np.arctan2(np.sin(heading - previous_heading), np.cos(heading - previous_heading))
        rates = (0.0, 0.0, 0.0) if dt <= 0 else (dr / dt, dp / dt, dh / dt)
        angvx, angvy, angvz = (0.0 if abs(rate) < 0.0001 else rate for rate in rates)
        self.time = time

        self.imu_message.header.stamp = time
        self.imu_message.orientation.x = quaternion[0]
        self.imu_message.orientation.y = quaternion[1]
        self.imu_message.orientation.z = quaternion[2]
        self.imu_message.orientation.w = quaternion[3]
        self.imu_message.angular_velocity.x = angvx
        self.imu_message.angular_velocity.y = angvy
        self.imu_message.angular_velocity.z = angvz
        self.imu_message.linear_acceleration.x = lin_acc_x_drone_body
        self.imu_message.linear_acceleration.y = lin_acc_y_drone_body
        self.imu_message.linear_acceleration.z = lin_acc_z_drone_body

    def msp_imu_sample(self, reply):
        """Build an IMU message from MSP counts and the packet receipt time."""
        try:
            values = [reply[key] for key in ('ax', 'ay', 'az', 'gx', 'gy', 'gz')]
            counts = [int(value) for value in values]
            stamp = float(reply['timestamp'])
            if (counts != values or any(v < -32768 or v > 32767 for v in counts)
                    or not 0 < stamp < 4294967296):
                raise ValueError('invalid MSP counts or receive time')
        except (KeyError, TypeError, ValueError, OverflowError):
            # A bad raw sample should not interrupt the control loop.
            rospy.logwarn_throttle(1, 'Invalid MSP IMU sample; skipping it')
            return None
        message = MspImu()
        message.header = Header()
        message.header.frame_id = 'fc_msp'
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.accelerometer = counts[:3]
        message.gyroscope = counts[3:]
        return message

    def integrated_imu_sample(self, reply):
        try:
            if (not np.all(np.isfinite(reply['rotation'] + reply['velocity']))
                    or not .9 < np.linalg.norm(reply['rotation']) < 1.1
                    or (reply['valid'] and not 0 < reply['duration_us'] <= 200000)):
                raise ValueError('Invalid integrated IMU payload')
            stamp = self.imu_clock.map(reply['end_us'], reply['sent'], reply['timestamp'])
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            rospy.logwarn_throttle(1, 'Invalid integrated IMU sample; skipping: %s' % error)
            return None
        message = IntegratedImu()
        message.header = Header()
        message.header.frame_id = 'fc_msp'
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.sequence = reply['sequence']
        message.sample_time_us = reply['end_us']
        message.integration_time_us = reply['duration_us']
        message.valid = reply['valid']
        message.delta_rotation = reply['rotation']
        message.delta_velocity = reply['velocity']
        if reply['valid'] or not self.integrated_started:
            self.imu_ack = reply['sequence']
        self.integrated_started = self.integrated_started or reply['valid']
        return message

    def update_battery_message(self, reply):
        self.battery_message.vbat = reply['vbat'] * 0.10
        self.battery_message.amperage = reply['amperage']

    def getBoard(self):
        """ Connect to the flight controller board """
        # Provisioned drones pin the board's /dev/serial/by-id path.
        port = os.environ.get('PALS_FC_PORT')
        if port:
            return MultiWii(port)
        try:
            board = MultiWii('/dev/ttyACM0')
        except SerialException as e:
            print(("usb0 failed: " + str(e)))
            try:
                board = MultiWii('/dev/ttyACM1')
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                raise
        return board

    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()

    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat"""
        self.heartbeat_state_estimator = rospy.Time.now()

    def shouldIDisarm(self):
        """Check the flight heartbeat watchdogs."""
        curr_time = rospy.Time.now()
        disarm = False
        if curr_time - self.heartbeat_web_interface > rospy.Duration.from_sec(3):
            print('\nSafety Failure: web interface heartbeat\n')
            print('The web interface stopped responding. Check your browser')
            disarm = True
        if curr_time - self.heartbeat_pid_controller > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving flight commands.')
            print('Check the pid_controller node\n')
            disarm = True
        if curr_time - self.heartbeat_infrared > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving data from the IR sensor.')
            print('Check the infrared node\n')
            disarm = True

        if curr_time - self.heartbeat_state_estimator > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving a state estimate.')
            print('Check the state_estimator node\n')
            disarm = True

        return disarm


def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    fc = FlightController()
    curr_time = rospy.Time.now()
    fc.heartbeat_infrared = curr_time
    fc.heartbeat_web_interface = curr_time
    fc.heartbeat_pid_controller = curr_time
    fc.heartbeat_state_estimator = curr_time

    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    msp_imupub = rospy.Publisher('/pidrone/imu_msp', MspImu, queue_size=1, tcp_nodelay=False)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    integral_pub = rospy.Publisher('/pidrone/imu_integrated', IntegratedImu, queue_size=100, tcp_nodelay=True)
    modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    calibration_pub = rospy.Publisher('/pidrone/localization/calibration_locked', Bool, queue_size=1, latch=True)
    calibration_owner_pub = rospy.Publisher('/pidrone/localization/calibration_owner/status', String, queue_size=1, latch=True)
    print('Publishing:')
    print('/pidrone/imu')
    print('/pidrone/mode')
    print('/pidrone/battery')

    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/coordination/paired', Bool, fc.coordination_selection_callback, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/preflight', FlightReadiness, fc.preflight_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/calibration_owner/request', String, fc.calibration_owner_callback, queue_size=1)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback,
                     queue_size=1, tcp_nodelay=True)
    rospy.Subscriber("/pidrone/range", Range, fc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, fc.heartbeat_web_interface_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, fc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/state", State, fc.heartbeat_state_estimator_callback)

    print("Started flight-controller node!")

    published_mode, last_mode_at = None, None
    published_calibration = None
    try:
        calibration_pub.publish(Bool(data=False))
        while not rospy.is_shutdown():
            if fc.curr_mode != 'DISARMED' and fc.shouldIDisarm():
                print("mode", fc.curr_mode)
                break

            command_sent, replies = fc.poll(integral_pub.publish)
            for code, reply in replies:
                if code == MultiWii.ANALOG:
                    if isinstance(reply, dict):
                        fc.update_battery_message(reply)
                        batpub.publish(fc.battery_message)
                    else:
                        rospy.logwarn_throttle(1, 'Invalid battery response; skipping sample')
                    continue
                if not isinstance(reply, dict) or reply.get('cmd') != MultiWii.INTEGRATED_IMU:
                    rospy.logwarn_throttle(1, 'Missing integrated IMU reply; imu100001 firmware is required')
                    continue
                # The decoder supplies attitude and raw counts in this order.
                samples = reply.get('legacy')
                if (not isinstance(samples, list) or len(samples) != 2
                        or not all(isinstance(sample, dict) for sample in samples)
                        or samples[0].get('cmd') != MultiWii.ATTITUDE
                        or samples[1].get('cmd') != MultiWii.RAW_IMU):
                    rospy.logwarn_throttle(1, 'Incomplete IMU response from flight controller; skipping sample')
                    continue
                attitude, raw = samples
                if reply['publish_attitude']:
                    fc.update_imu_message(attitude, raw)
                    imupub.publish(fc.imu_message)
                if msp_imupub.get_num_connections() > 0:
                    msp_message = fc.msp_imu_sample(raw)
                    if msp_message is not None:
                        msp_imupub.publish(msp_message)

            if command_sent:
                with fc.mode_lock:
                    now = monotonic()
                    # Report the ROS control mode, not Betaflight's arming or Angle flags.
                    if fc.curr_mode != published_mode or now - last_mode_at >= .1:
                        modepub.publish(fc.curr_mode)
                        published_mode, last_mode_at = fc.curr_mode, now
                    calibration = (fc.calibration_owner, fc.calibration_request)
                    if calibration != published_calibration:
                        calibration_owner_pub.publish(String(data=json.dumps({'owner': calibration[0],
                                                                             'request': calibration[1]})))
                        published_calibration = calibration

            # Wake on a reply or the next request deadline; check commands within 5 ms.
            deadline = fc.next_rc
            if fc.imu_pending is None:
                deadline = min(deadline, fc.next_imu)
            if fc.battery_pending is None:
                deadline = min(deadline, fc.next_battery)
            try:
                select.select([fc.board.ser], [], [], max(0., min(.005, deadline - monotonic())))
            except (select.error, OSError) as error:
                # Recheck ROS shutdown on the next iteration after an interrupted wait.
                if not error.args or error.args[0] != errno.EINTR:
                    raise

    except SerialException:
        print('\nCannot connect to the flight controller board.')
        print('The USB is unplugged. Please check connection.')
    except Exception as e:
        print('there was an internal error', e)
        print(traceback.format_exc())
    finally:
        print('Shutdown received')
        print('Sending DISARM command')
        fc.board.send_commands([(MultiWii.SET_RAW_RC, cmds.disarm_cmd)])
        fc.board.receiveDataPacket()


if __name__ == '__main__':
    main()
