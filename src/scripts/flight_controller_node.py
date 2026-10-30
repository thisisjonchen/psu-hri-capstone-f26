#!/usr/bin/env python
import traceback
import sys
import yaml
import threading

import rospy
import rospkg
import signal
import select
import numpy as np

print("tf import")
import tf
print("done tf import")


import command_values as cmds
import json
from sensor_msgs.msg import Imu
from h2rMultiWii import MultiWii
from serial import SerialException
from std_msgs.msg import Header, Empty, Bool, String
from geometry_msgs.msg import Quaternion
from pidrone_pkg.msg import Battery, Mode, RC, State, FlightReadiness, MspImu
from sensor_msgs.msg import Range
from localization import monotonic
from localization.distance_guard import fresh, STATE_TIMEOUT
import os


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/imu_msp (MSP accelerometer and gyro counts)
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/range
    /pidrone/heartbeat/web_interface
    /pidrone/heartbeat/pid_controller
    /pidrone/state
    """

    def __init__(self):
        # Connect to the flight controller board
        print("getboard")
        self.board = self.getBoard()
        print("done")
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'         #initialize as disarmed
        self.prev_mode = 'DISARMED'         #initialize as disarmed
        self.mode_lock = threading.RLock()
        self.preflight = self.preflight_received = self.preflight_stamp = None
        self.coordination_selection = self.selection_received = None
        self.flight_paired = None
        self.calibration_owner = rospy.get_param('/pidrone/localization/calibration_owner', '')
        self.calibration_request = ''
        # Clear the retired flight lock when upgrading an existing ROS session.
        rospy.set_param('/pidrone/localization/calibration_lock', False)
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd      #initialize as disarmed
        self.last_command = cmds.disarm_cmd
        now = monotonic()
        self.next_rc = self.next_imu = self.next_attitude = self.next_battery = now
        self.imu_pending = self.battery_pending = None
        self.last_rc_ack = now
        # store the mode publisher
        self.modepub = None
        # store the time for angular velocity calculations
        self.time = rospy.Time.now()

        # Initialize the Imu Message
        ############################
        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()

        self.imu_message = Imu()
        self.imu_message.header = header

        # Initialize the Battery Message
        ################################
        self.battery_message = Battery()
        self.battery_message.vbat = None
        self.battery_message.amperage = None
        # Adjust this based on how low the battery should discharge
        self.minimum_voltage = 4.5

        # Accelerometer parameters
        ##########################
        print("loading")
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/multiwii.yaml" % path) as f:
            means = yaml.load(f)
        print("done")
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX = means["ax"] * self.accRawToMss
        self.accZeroY = means["ay"] * self.accRawToMss
        self.accZeroZ = means["az"] * self.accRawToMss


    # ROS subscriber callback methods:
    ##################################
    def desired_mode_callback(self, msg):
        """Check readiness before accepting an arm or takeoff request."""
        with self.mode_lock:
            if msg.mode not in ('DISARMED', 'ARMED', 'FLYING'):
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
            self.prev_mode = self.curr_mode
            self.curr_mode = msg.mode
            if self.prev_mode != self.curr_mode:
                self.preflight = self.preflight_received = self.preflight_stamp = None
            if self.curr_mode == 'DISARMED':
                self.flight_paired = None
            self.update_command()

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
            if self.selection_received is None or not 0 <= monotonic() - self.selection_received <= 2.:
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
        """ Store and send the flight commands if the current mode is FLYING """
        if self.curr_mode == 'FLYING':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            self.command = [r, p, y, t] + cmds.idle_cmd[4:8]


    # Update methods:
    #################
    @staticmethod
    def next_poll(deadline, now, hz):
        return deadline + (int(max(0., now - deadline) * hz) + 1) / hz

    def poll(self):
        """Schedule RC and telemetry independently on the same serial connection."""
        now = monotonic()
        received = []
        for code, reply in self.board.receive_available():
            if code == MultiWii.SET_RAW_RC:
                if reply == 'Set Raw RC':
                    self.last_rc_ack = now
                else:
                    rospy.logwarn_throttle(1, 'Invalid RC acknowledgment from flight controller')
            elif code in (MultiWii.RAW_IMU, MultiWii.MULTIPLE_MSP):
                if self.imu_pending is not None and code == self.imu_pending[0]:
                    self.imu_pending = None
                    received.append((code, reply))
            elif code == MultiWii.ANALOG and self.battery_pending is not None:
                self.battery_pending = None
                received.append((code, reply))

        if self.imu_pending is not None and now - self.imu_pending[1] >= 1.:
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
            if now >= self.next_attitude:
                code, args = MultiWii.MULTIPLE_MSP, [MultiWii.ATTITUDE, MultiWii.RAW_IMU]
                self.next_attitude = self.next_poll(self.next_attitude, now, 60.)
            else:
                code, args = MultiWii.RAW_IMU, []
            requests.append((code, args))
            self.imu_pending = (code, now)
            self.next_imu = self.next_poll(self.next_imu, now, 100.)
        if now >= self.next_battery and self.battery_pending is None:
            requests.append((MultiWii.ANALOG, []))
            self.battery_pending = now
            self.next_battery = now + .5
        if requests:
            self.board.send_commands(requests)
        if command_sent and command != self.last_command:
            print('new command sent:', command)
            self.last_command = command
        return command_sent, received

    def update_imu_message(self, replies):
        """Build the legacy IMU message from a fresh attitude/acceleration pair."""
        if (not isinstance(replies, list) or len(replies) != 2
                or any(not isinstance(reply, dict) for reply in replies)
                or set(reply.get('cmd') for reply in replies) != set((MultiWii.ATTITUDE, MultiWii.RAW_IMU))):
            rospy.logwarn_throttle(1, 'Incomplete IMU response from flight controller; skipping sample')
            return False

        attitude = next(reply for reply in replies if reply['cmd'] == MultiWii.ATTITUDE)
        raw = next(reply for reply in replies if reply['cmd'] == MultiWii.RAW_IMU)

        # calculate values to update imu_message:
        roll = np.deg2rad(attitude['angx'])
        pitch = -np.deg2rad(attitude['angy'])
        heading = np.deg2rad(attitude['heading'])
        # Note that at pitch angles near 90 degrees, the roll angle reading can
        # fluctuate a lot
        # transform heading (similar to yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is CCW
        heading = (-heading) % (2 * np.pi)
        # When first powered up, heading should read near 0
        # get the previous roll, pitch, heading values
        previous_quaternion = self.imu_message.orientation
        quaternion_array = [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
        previous_roll, previous_pitch, previous_heading = tf.transformations.euler_from_quaternion(quaternion_array)

        # Although quaternion_from_euler takes a heading in range [0, 2pi),
        # euler_from_quaternion returns a heading in range [0, pi] or [0, -pi).
        # Thus need to convert the returned heading back into the range [0, 2pi).
        previous_heading = previous_heading % (2 * np.pi)

        # transform euler angles into quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        # calculate the linear accelerations
        lin_acc_x = raw['ax'] * self.accRawToMss - self.accZeroX
        lin_acc_y = raw['ay'] * self.accRawToMss - self.accZeroY
        lin_acc_z = raw['az'] * self.accRawToMss - self.accZeroZ

        # Rotate the IMU frame to align with our convention for the drone's body
        # frame. IMU: x is forward, y is left, z is up. We want: x is right,
        # y is forward, z is up.
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        # Account for gravity's affect on linear acceleration values when roll
        # and pitch are nonzero. When the drone is pitched at 90 degrees, for
        # example, the z acceleration reads out as -9.8 m/s^2. This makes sense,
        # as the IMU, when powered up / when the calibration script is called,
        # zeros the body-frame z-axis acceleration to 0, but when it's pitched
        # 90 degrees, the body-frame z-axis is perpendicular to the force of
        # gravity, so, as if the drone were in free-fall (which was roughly
        # confirmed experimentally), the IMU reads -9.8 m/s^2 along the z-axis.
        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g*np.sin(roll)*np.cos(pitch)
        lin_acc_y_drone_body = lin_acc_y_drone_body + g*np.cos(roll)*(-np.sin(pitch))
        lin_acc_z_drone_body = lin_acc_z_drone_body + g*(1 - np.cos(roll)*np.cos(pitch))

        # calculate the angular velocities of roll, pitch, and yaw in rad/s
        time = rospy.Time.now()
        dt = time.to_sec() - self.time.to_sec()
        dr = roll - previous_roll
        dp = pitch - previous_pitch
        # Wrap to [-pi, pi]: crossing north must not read as a full-turn spin.
        dh = np.arctan2(np.sin(heading - previous_heading), np.cos(heading - previous_heading))
        angvx = self.near_zero(dr / dt)
        angvy = self.near_zero(dp / dt)
        angvz = self.near_zero(dh / dt)
        self.time = time

        # Update the imu_message:
        # header stamp
        self.imu_message.header.stamp = time
        # orientation
        self.imu_message.orientation.x = quaternion[0]
        self.imu_message.orientation.y = quaternion[1]
        self.imu_message.orientation.z = quaternion[2]
        self.imu_message.orientation.w = quaternion[3]
        # angular velocities
        self.imu_message.angular_velocity.x = angvx
        self.imu_message.angular_velocity.y = angvy
        self.imu_message.angular_velocity.z = angvz
        # linear accelerations
        self.imu_message.linear_acceleration.x = lin_acc_x_drone_body
        self.imu_message.linear_acceleration.y = lin_acc_y_drone_body
        self.imu_message.linear_acceleration.z = lin_acc_z_drone_body
        return True

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

    def update_battery_message(self, reply):
        self.battery_message.vbat = reply['vbat'] * 0.10
        self.battery_message.amperage = reply['amperage']

    def update_command(self):
        ''' Set command values if the mode is ARMED or DISARMED '''
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd

    # Helper Methods:
    #################
    def getBoard(self):
        """ Connect to the flight controller board """
        # (if the flight cotroll usb is unplugged and plugged back in,
        #  it becomes .../USB1)
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

    def near_zero(self, n):
        """ Set a number to zero if it is below a threshold value """
        return 0 if abs(n) < 0.0001 else n

    def ctrl_c_handler(self, signal, frame):
        """ Disarm the drone and quits the flight controller node """
        print("\nCaught ctrl-c! About to Disarm!")
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)
        self.modepub.publish('DISARMED')
        print("Successfully Disarmed")
        sys.exit()

    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()
        self.range = msg.range

    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat"""
        self.heartbeat_state_estimator = rospy.Time.now()

    def shouldIDisarm(self):
        """
        Disarm the drone if the battery values are too low or if there is a
        missing heartbeat
        """
        curr_time = rospy.Time.now()
        disarm = False
        if self.battery_message.vbat != None and self.battery_message.vbat < self.minimum_voltage:
            #print('\nSafety Failure: low battery\n')
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

        if self.range is not None and self.range > 1:
            print(('\nSafety Failure: too high: ' + str(self.range)))
            disarm = True
        if curr_time - self.heartbeat_state_estimator > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving a state estimate.')
            print('Check the state_estimator node\n')
            disarm = True

        return disarm


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    print("init")
    rospy.init_node(node_name)
    print("done")
    # create the FlightController object
    fc = FlightController()
    curr_time = rospy.Time.now()
    fc.heartbeat_infrared = curr_time
    fc.range = None
    fc.heartbeat_web_interface= curr_time
    fc.heartbeat_pid_controller = curr_time
    fc.heartbeat_flight_controller = curr_time
    fc.heartbeat_state_estimator = curr_time

    # Publishers
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    msp_imupub = rospy.Publisher('/pidrone/imu_msp', MspImu, queue_size=1, tcp_nodelay=False)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    calibration_pub = rospy.Publisher('/pidrone/localization/calibration_locked', Bool, queue_size=1, latch=True)
    calibration_owner_pub = rospy.Publisher('/pidrone/localization/calibration_owner/status', String, queue_size=1, latch=True)
    print('Publishing:')
    print('/pidrone/imu')
    print('/pidrone/mode')
    print('/pidrone/battery')

    # Subscribers
    ############
    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/coordination/paired', Bool, fc.coordination_selection_callback, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/preflight', FlightReadiness, fc.preflight_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/calibration_owner/request', String, fc.calibration_owner_callback, queue_size=1)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback,
                     queue_size=1, tcp_nodelay=True)
    # heartbeat subscribers
    rospy.Subscriber("/pidrone/range", Range, fc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, fc.heartbeat_web_interface_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, fc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/state", State, fc.heartbeat_state_estimator_callback)

    print("Started flight-controller node!")

    # signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    try:
        while not rospy.is_shutdown():
            # if the current mode is anything other than disarmed
            # preform as safety check
                # Break the loop if a safety check has failed
            if fc.curr_mode != 'DISARMED' and fc.shouldIDisarm():
                print("mode", fc.curr_mode)
                break
                
            fc.update_command()
            command_sent, replies = fc.poll()
            for code, reply in replies:
                if code == MultiWii.ANALOG:
                    if isinstance(reply, dict):
                        fc.update_battery_message(reply)
                        batpub.publish(fc.battery_message)
                    else:
                        rospy.logwarn_throttle(1, 'Invalid battery response; skipping sample')
                    continue
                if code == MultiWii.MULTIPLE_MSP:
                    if not fc.update_imu_message(reply):
                        continue
                    imupub.publish(fc.imu_message)
                    raw = next(sample for sample in reply if sample['cmd'] == MultiWii.RAW_IMU)
                else:
                    if not isinstance(reply, dict) or reply.get('cmd') != MultiWii.RAW_IMU:
                        rospy.logwarn_throttle(1, 'Invalid IMU response; skipping sample')
                        continue
                    raw = reply
                msp_message = fc.msp_imu_sample(raw)
                if msp_message is not None:
                    msp_imupub.publish(msp_message)

            if command_sent:
                fc.modepub.publish(fc.curr_mode)
                with fc.mode_lock:
                    calibration_pub.publish(Bool(data=False))
                    calibration_owner_pub.publish(String(data=json.dumps({'owner': fc.calibration_owner,
                                                                         'request': fc.calibration_request})))

            # Wake on a reply, or wait up to 5 ms before checking commands again.
            select.select([fc.board.ser], [], [], .005)
            
    except SerialException:
        print('\nCannot connect to the flight controller board.')
        print('The USB is unplugged. Please check connection.')
    except Exception as e:
        print('there was an internal error', e)
        print(traceback.format_exc())
    finally:
        print('Shutdown received')
        print('Sending DISARM command')
        fc.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        fc.board.receiveDataPacket()


if __name__ == '__main__':
    main()
