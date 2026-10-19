#!/usr/bin/env python

import math
import time

import rosnode
import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from pidrone_pkg.msg import Battery, Mode
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty
from localization.fusion import rotate, target_velocity, XY_PREDICTION_TIMEOUT


class Drone(object):

    # Battery Limits
    MIN_START_V = 15.2 # Minimum voltage to start
    LOW_FLIGHT_V = 14.0 # Lowest acceptable voltage during flight

    # Control Variables
    ALTITUDE_SETTLE_SECONDS = 1.0 # How long drone should settle at target alt
    LAND_STEP_M = 0.05 # Target height decrease per landing step (meters)
    LAND_STEP_SECONDS = 1.0 # Time between landing steps (seconds)
    LAND_DISARM_CLEARANCE_M = 0.05 # Disarm this far above the measured ground
    START_TIMEOUT_S = 30.0 # Wait this long for the dashboard's synchronized start

    def __init__(self):
        self.estimate_timeout = float(rospy.get_param('/pidrone/altitude/measurement_timeout', 0.5))
        self.takeoff_height = float(rospy.get_param('/pidrone/altitude/takeoff_height', 0.25))
        self.xy_hold_gain = float(rospy.get_param('/pidrone/localization/target_gain', 0.20))
        self.xy_speed_limit = float(rospy.get_param('/pidrone/localization/target_speed_limit', 0.10))
        if any(math.isnan(v) or math.isinf(v) or v <= 0
               for v in (self.xy_hold_gain, self.xy_speed_limit)):
            raise ValueError('Room target gain and speed must be positive')
        self.mode = None
        self.mode_at = 0
        self.height = None
        self.height_at = 0
        self.ground_height = None
        self.target_height = self.takeoff_height
        self.xy_anchor = None
        self.xy_frame = None
        self.fused_pose = None
        self.voltage = None
        self.battery_at = 0
        self.low_voltage_at = None
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        self.position_mode_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
        self.heartbeat_pub = rospy.Publisher(
            '/pidrone/heartbeat/web_interface', Empty, queue_size=1)
        # The dashboard releases every drone of one Execute together once all are ready.
        self.start_at = None
        self.ready_pub = rospy.Publisher('/pidrone/routine/ready', Empty, queue_size=1)
        rospy.Subscriber('/pidrone/routine/start', Empty, self._start_received)
        rospy.Subscriber('/pidrone/mode', Mode, self._mode_received)
        rospy.Subscriber('/pidrone/range', Range, self._range_received)
        rospy.Subscriber('/pidrone/localization/fused', PoseWithCovarianceStamped,
                         self._fused_received, queue_size=1)
        rospy.Subscriber('/pidrone/battery', Battery, self._battery_received)
        self.heartbeat = rospy.Timer(
            rospy.Duration(0.5), lambda _: self.heartbeat_pub.publish(Empty()))

    def _start_received(self, msg):
        self.start_at = time.time()

    def _mode_received(self, msg):
        self.mode = msg.mode
        self.mode_at = time.time()

    def _range_received(self, msg):
        if msg.min_range <= msg.range <= msg.max_range:
            self.height = msg.range
            self.height_at = time.time()

    def _fused_received(self, msg):
        self.fused_pose = msg

    def _hold_pose(self, required=True, allow_prediction=False):
        msg = self.fused_pose
        timeout = XY_PREDICTION_TIMEOUT if allow_prediction else self.estimate_timeout
        if (msg is None or not msg.header.frame_id or
                not 0 <= rospy.get_time() - msg.header.stamp.to_sec() <= timeout):
            if not required:
                return None
            raise RuntimeError('Fused position is missing/stale')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if (any(math.isnan(v) or math.isinf(v) for v in values) or
                q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12):
            raise RuntimeError('Fused position is invalid')
        return msg

    def _battery_received(self, msg):
        voltage = msg.vbat
        if not math.isnan(voltage) and 8.0 <= voltage <= 20.0:
            now = time.time()
            self.voltage = voltage
            self.battery_at = now
            if voltage < self.LOW_FLIGHT_V:
                self.low_voltage_at = self.low_voltage_at or now
            else:
                self.low_voltage_at = None

    def _check_battery(self):
        if time.time() - self.battery_at > 1.0:
            raise RuntimeError('Battery telemetry is missing/stale')
        if (self.low_voltage_at is not None and
                time.time() - self.low_voltage_at >= 1.5):
            raise RuntimeError('Battery voltage is low ({:.1f} V)'.format(
                self.voltage))

    def _check_sensors(self):
        now = time.time()
        if now - self.mode_at > 0.5:
            raise RuntimeError('Flight controller mode is missing/stale')
        if now - self.height_at > 1.0:
            raise RuntimeError('Downward range is missing/stale')

    def _check_flight(self, operation, check_battery=True):
        if self.mode != 'FLYING':
            raise RuntimeError('Flight mode changed during ' + operation)
        self._check_sensors()
        if check_battery:
            self._check_battery()

    @staticmethod
    def _limit_planar_speed(command, maximum):
        speed = math.hypot(command.linear.x, command.linear.y)
        if speed > maximum:
            command.linear.x *= maximum / speed
            command.linear.y *= maximum / speed
        return min(speed, maximum)

    def _set_altitude(self, height):
        """Publish a relative pose command for an uncapped absolute height."""
        if math.isnan(height) or math.isinf(height) or height < 0:
            raise ValueError('Target altitude must be finite and nonnegative')
        pose = Pose()
        pose.position.z = height - self.target_height
        self.pose_pub.publish(pose)
        self.target_height = height

    def _settled_since(self, target, tolerance, since, now):
        if abs(self.height - target) > tolerance:
            return None
        return now if since is None else since

    def ready(self):
        """Wait for the boot-started flight nodes; routines do not launch nodes."""
        if '/state_estimator' in rosnode.get_node_names():
            raise RuntimeError('Stop the legacy /state_estimator first; only fusion should publish /pidrone/state')
        deadline = time.time() + 90.0 # 90 second timeout for all ROS nodes to get ready
        publishers = (self.mode_pub, self.twist_pub, self.pose_pub,
                      self.position_mode_pub, self.heartbeat_pub)
        ground_candidate = None
        candidate_at = 0
        reason = 'Waiting for flight nodes'
        reported_reason = None
        last_report = 0
        while time.time() < deadline:
            if rospy.is_shutdown():
                raise RuntimeError('ROS shut down while waiting for readiness')
            try:
                if self.mode != 'DISARMED':
                    raise RuntimeError('Waiting for DISARMED mode (received: {})'.format(self.mode))
                self._check_sensors()
                missing = [pub.name for pub in publishers if not pub.get_num_connections()]
                if missing:
                    raise RuntimeError('Waiting for subscribers: ' + ', '.join(missing))
                self._check_battery()
                if self.height > 0.12:
                    raise RuntimeError('Waiting for ground height <= 0.12 m (received: {:.3f})'.format(self.height))
                now = time.time()
                if ground_candidate is None or abs(self.height - ground_candidate) > 0.01:
                    ground_candidate, candidate_at = self.height, now
                elif now - candidate_at >= 0.5:
                    if self.voltage < self.MIN_START_V:
                        raise ValueError('Battery voltage is too low to start ({:.1f} V)'.format(self.voltage))
                    self.ground_height = self.height
                    rospy.loginfo('Flight nodes ready; ground height %.3f m', self.height)
                    return
                reason = 'Waiting for steady ground height'
            except RuntimeError as error:
                ground_candidate = None
                reason = str(error)
            if reason != reported_reason or time.time() - last_report >= 5.0:
                rospy.loginfo('%s', reason)
                reported_reason = reason
                last_report = time.time()
            time.sleep(0.1)
        raise RuntimeError('Readiness timed out: ' + reason)

    def wait_for_start(self, timeout=START_TIMEOUT_S, hold_xy=False):
        """Post ready until the dashboard's start arrives; never take off alone."""
        rospy.loginfo('Waiting for synchronized start (up to %.0f s)', timeout)
        self.start_at = None
        started = time.time()
        last_ready = 0
        while time.time() - started < timeout:
            if rospy.is_shutdown():
                raise RuntimeError('ROS shut down while waiting for synchronized start')
            if self.mode != 'DISARMED':
                raise RuntimeError('Mode changed to {} while waiting for synchronized start'.format(self.mode))
            self._check_sensors()
            self._check_battery()
            if self.voltage < self.MIN_START_V:
                raise RuntimeError('Battery voltage is too low for synchronized start')
            if hold_xy:
                self._hold_pose()
            if self.start_at is not None and self.start_at >= started:
                rospy.loginfo('Synchronized start received')
                return
            if time.time() - last_ready >= 0.2:
                self.ready_pub.publish(Empty())
                last_ready = time.time()
            time.sleep(0.02)
        raise RuntimeError('No synchronized start within {:.0f} s; not taking off'.format(timeout))

    def _set_mode(self, name):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.mode_pub.publish(Mode(name))
            if self.mode == name:
                return
            rospy.sleep(0.1)
        raise RuntimeError('Mode {} was not confirmed'.format(name))

    def lock_xy(self):
        """Hold the current fused room position through the velocity PID."""
        if self.mode not in ('ARMED', 'FLYING'):
            raise RuntimeError('Arm before locking X/Y')
        self._check_sensors()
        msg = self._hold_pose()
        p = msg.pose.pose.position
        self.xy_anchor = (p.x, p.y)
        self.xy_frame = msg.header.frame_id
        # This routine supplies the position-to-velocity correction; do not add
        # the PID's separate local-position correction on top of it.
        self.position_mode_pub.publish(Bool(False))
        rospy.loginfo('Fused X/Y hold enabled at %.3f, %.3f', p.x, p.y)

    def unlock_xy(self):
        self.xy_anchor = None
        self.xy_frame = None
        self.twist_pub.publish(Twist())

    def _xy_hold_command(self):
        command = Twist()
        if self.xy_anchor is None:
            return command
        msg = self._hold_pose(required=False, allow_prediction=True)
        if msg is None:
            # Preserve the room anchor; stop requesting motion until fusion resumes.
            return command
        if msg.header.frame_id != self.xy_frame:
            raise RuntimeError('Fused position frame changed during X/Y hold')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        # Fused orientation already expresses the controller's right/forward
        # axes in the room frame. Use the same conversion as room navigation.
        yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        command.linear.x, command.linear.y = target_velocity(
            self.xy_anchor, (p.x, p.y), yaw, self.xy_hold_gain, self.xy_speed_limit)
        rospy.loginfo_throttle(1,
            'XY hold: room_error=(%.3f, %.3f) m body_command=(%.3f, %.3f) m/s' % (
                self.xy_anchor[0] - p.x, self.xy_anchor[1] - p.y,
                command.linear.x, command.linear.y))
        return command

    def hover(self, seconds=1.0, command=None):
        """Hover, publishing command() (default: fused X/Y hold) each cycle."""
        command = command or self._xy_hold_command
        self.twist_pub.publish(command())
        deadline = time.time() + seconds
        while time.time() < deadline:
            self._check_flight('hover')
            self.twist_pub.publish(command())
            rospy.sleep(max(0.0, min(0.1, deadline - time.time())))

    def disarm(self):
        self._set_mode('DISARMED')
        self.unlock_xy()

    def takeoff(self, hold_xy=False):
        """Take off, optionally holding fused room X/Y from before liftoff."""
        if self.ground_height is None or self.mode != 'DISARMED':
            raise RuntimeError('Call ready() while the drone is disarmed before takeoff')
        self.wait_for_start(hold_xy=hold_xy)
        self._check_sensors()
        self._check_battery()
        if abs(self.height - self.ground_height) > 0.02:
            raise RuntimeError('Drone is no longer at the measured ground height')
        if self.voltage < self.MIN_START_V:
            raise RuntimeError('Battery voltage is too low to start ({:.1f} V)'.format(
                self.voltage))
        self._set_mode('ARMED')
        self.target_height = self.takeoff_height
        rospy.sleep(1.0)
        self._check_sensors()
        self._check_battery()
        self.twist_pub.publish(self._xy_hold_command())
        if hold_xy:
            self.lock_xy()
        self._set_mode('FLYING')
        deadline = time.time() + 10.0
        settled_since = None
        while time.time() < deadline:
            self._check_flight('takeoff')
            if hold_xy:
                self.twist_pub.publish(self._xy_hold_command())
            now = time.time()
            settled_since = self._settled_since(self.takeoff_height, 0.05, settled_since, now)
            if settled_since is not None and now - settled_since >= self.ALTITUDE_SETTLE_SECONDS:
                self.target_height = self.takeoff_height
                rospy.loginfo('Takeoff altitude held for %.1f seconds; continuing routine',
                              self.ALTITUDE_SETTLE_SECONDS)
                return
            rospy.sleep(0.1)
        raise RuntimeError('Takeoff altitude did not settle within 10 seconds')

    def move(self, x=0.0, y=0.0, z=0.0):
        """Move relative to the drone: x right, y forward, z up (meters).

        Horizontal distance and heading come from the UWB/IMU room pose.
        """
        if self.mode != 'FLYING':
            raise RuntimeError('Drone must be flying before moving')
        if x == 0 and y == 0 and z == 0:
            return
        horizontal = math.hypot(x, y)
        if horizontal and self.xy_anchor is not None:
            raise ValueError('Unlock X/Y before commanding horizontal movement')
        self._check_sensors()
        start_z = self.height
        target_z = start_z + z
        if horizontal:
            msg = self._hold_pose()
            frame = msg.header.frame_id
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            start_xy = (p.x, p.y)
            yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            displacement = rotate((x, y), yaw)
            target_xy = displacement + start_xy
            self.position_mode_pub.publish(Bool(False))
        if z:
            self._set_altitude(target_z)
        started = time.time()
        deadline = started + 15.0
        commanded_distance = 0.0
        last_command_at = started
        last_command_speed = 0.0
        best_vertical_progress = 0.0
        settled_since = None
        try:
            while time.time() < deadline:
                self._check_flight('movement')
                now = time.time()
                commanded_distance += last_command_speed * (now - last_command_at)
                last_command_at = now
                if horizontal and commanded_distance > max(0.4, horizontal * 1.5):
                    raise RuntimeError('Movement limit reached without reaching target')
                ex = ey = 0.0
                if horizontal:
                    msg = self._hold_pose(allow_prediction=True)
                    if msg.header.frame_id != frame:
                        raise RuntimeError('Fused position frame changed during movement')
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
                    ex, ey = rotate(target_xy - (p.x, p.y), -yaw)
                ez = target_z - self.height
                horizontal_done = (not horizontal or
                                   (abs(ex) <= 0.06 and abs(ey) <= 0.06))
                if z:
                    progress_z = (self.height - start_z) * (1 if z > 0 else -1)
                    best_vertical_progress = max(best_vertical_progress, progress_z)
                    settled_since = self._settled_since(target_z, 0.05, settled_since, now)
                vertical_done = (not z or (settled_since is not None and
                                 now - settled_since >= self.ALTITUDE_SETTLE_SECONDS))
                if horizontal_done and vertical_done:
                    return
                if now - started > 3.0:
                    if horizontal:
                        progress_xy = ((p.x - start_xy[0]) * displacement[0] +
                                       (p.y - start_xy[1]) * displacement[1])
                        if progress_xy / horizontal < 0.03:
                            raise RuntimeError('No horizontal movement detected')
                    if z and abs(ez) > 0.03 and best_vertical_progress < 0.02:
                        raise RuntimeError('No vertical movement detected')
                command = self._xy_hold_command()
                if horizontal:
                    for error, axis in ((ex, 'x'), (ey, 'y')):
                        if abs(error) > 0.06:
                            speed = min(0.15, max(0.06, abs(error) * 0.5))
                            setattr(command.linear, axis, speed if error > 0 else -speed)
                planar_speed = self._limit_planar_speed(command, 0.15)
                self.twist_pub.publish(command)
                last_command_speed = planar_speed
                rospy.sleep(0.1)
            if z:
                raise RuntimeError(
                    'Altitude did not settle: target={:.3f} m, measured={:.3f} m, '
                    'best vertical progress={:.3f} m'.format(
                        target_z, self.height, best_vertical_progress))
            raise RuntimeError('Movement timed out')
        finally:
            self.twist_pub.publish(self._xy_hold_command())

    def move_to_altitude(self, height):
        """Move to an absolute downward-range height in meters."""
        if self.height is None:
            raise RuntimeError('Downward range has not been received')
        self.move(z=height - self.height)

    def move_forward(self, meters):
        if meters <= 0:
            raise ValueError('meters must be positive')
        self.move(y=meters)

    def move_back(self, meters):
        if meters <= 0:
            raise ValueError('meters must be positive')
        self.move(y=-meters)

    def land(self, hold_xy=False):
        """Descend and disarm at the ground threshold, optionally holding X/Y."""
        if not hold_xy:
            self.unlock_xy()
        if self.ground_height is None:
            raise RuntimeError('Ground height was not measured before takeoff')
        steps = math.ceil(max(0.0, self.height - self.ground_height) / self.LAND_STEP_M)
        deadline = time.time() + max(20.0, (steps + 1) * self.LAND_STEP_SECONDS + 5.0)
        disarm_height = self.ground_height + self.LAND_DISARM_CLEARANCE_M
        last_step = 0
        while time.time() < deadline:
            if self.mode == 'DISARMED':
                self.unlock_xy()
                return
            self._check_flight('landing', check_battery=False)
            if self.height <= disarm_height:
                rospy.loginfo('Landing height %.3f m reached disarm threshold %.3f m',
                              self.height, disarm_height)
                self.disarm()
                return
            if hold_xy:
                try:
                    self.twist_pub.publish(self._xy_hold_command())
                except RuntimeError as error:
                    # Preserve the existing landing behavior when X/Y hold fails.
                    rospy.logwarn('Releasing X/Y hold during landing: %s', error)
                    self.unlock_xy()
                    hold_xy = False
            now = time.time()
            if now - last_step >= self.LAND_STEP_SECONDS:
                self._set_altitude(max(0.0, self.target_height - self.LAND_STEP_M))
                last_step = now
            rospy.sleep(0.1)
        raise RuntimeError('Landing was not confirmed; drone is still armed')
