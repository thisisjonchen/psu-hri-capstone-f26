#!/usr/bin/env python

import math
import os
import subprocess
import sys
import tempfile
import time

import rosnode
import rospy
from geometry_msgs.msg import Pose, Twist, TwistStamped
from pidrone_pkg.msg import Battery, Mode
from sensor_msgs.msg import Range
from std_msgs.msg import Empty


class Drone(object):

    # Battery Limits
    MIN_START_V = 15.2 # Minimum voltage to start
    LOW_FLIGHT_V = 14.0 # Lowest acceptable voltage during flight

    # Control Variables
    ALTITUDE_SETTLE_SECONDS = 1.0 # How long drone should settle at target alt
    LAND_STEP_M = 0.05 # Target height decrease per landing step (meters)
    LAND_STEP_SECONDS = 1.0 # Time between landing steps (seconds)
    LAND_DISARM_CLEARANCE_M = 0.05 # Disarm this far above the measured ground

    def __init__(self):
        self.takeoff_height = float(rospy.get_param('/pidrone/altitude/takeoff_height', 0.25))
        self.mode = None
        self.mode_at = 0
        self.height = None
        self.height_at = 0
        self.ground_height = None
        self.target_height = self.takeoff_height
        self.flow_at = 0
        self.flow_valid = False
        self.xy_anchor = None
        self.xy_hold_valid = True
        self.voltage = None
        self.battery_at = 0
        self.low_voltage_at = None
        self.right_m = 0.0
        self.forward_m = 0.0
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        self.heartbeat_pub = rospy.Publisher(
            '/pidrone/heartbeat/web_interface', Empty, queue_size=1)
        rospy.Subscriber('/pidrone/mode', Mode, self._mode_received)
        rospy.Subscriber('/pidrone/range', Range, self._range_received)
        rospy.Subscriber('/pidrone/picamera/twist', TwistStamped, self._flow_received)
        rospy.Subscriber('/pidrone/battery', Battery, self._battery_received)
        self.heartbeat = rospy.Timer(
            rospy.Duration(0.5), lambda _: self.heartbeat_pub.publish(Empty()))

    def _mode_received(self, msg):
        self.mode = msg.mode
        self.mode_at = time.time()

    def _range_received(self, msg):
        if msg.min_range <= msg.range <= msg.max_range:
            self.height = msg.range
            self.height_at = time.time()

    def _flow_received(self, msg):
        now = time.time()
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.flow_valid = abs(vx) <= 0.5 and abs(vy) <= 0.5
        continuous = self.flow_at and 0 < now - self.flow_at < 0.2
        if self.xy_anchor is not None and (not self.flow_valid or not continuous):
            # A missed displacement cannot be recovered by later good samples.
            self.xy_hold_valid = False
        if continuous and self.flow_valid:
            dt = now - self.flow_at
            self.right_m += vx * dt
            self.forward_m += vy * dt
        self.flow_at = now

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

    def _check_sensors(self, need_flow=True):
        now = time.time()
        if now - self.mode_at > 0.5:
            raise RuntimeError('Flight controller mode is missing/stale')
        if now - self.height_at > 1.0:
            raise RuntimeError('Downward range is missing/stale')
        if need_flow and (now - self.flow_at > 0.5 or not self.flow_valid):
            raise RuntimeError('Optical flow is missing, stale or invalid')

    def _check_flight(self, operation, need_flow=False, check_battery=True):
        if self.mode != 'FLYING':
            raise RuntimeError('Flight mode changed during ' + operation)
        self._check_sensors(need_flow=need_flow)
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
        self._start_missing_nodes()
        deadline = time.time() + 90.0 # 90 second timeout for all ROS nodes to get ready
        publishers = (self.mode_pub, self.twist_pub, self.pose_pub, self.heartbeat_pub)
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

    def _start_missing_nodes(self):
        """Start missing flight nodes locally, once before takeoff."""
        def running(name):
            return (name in rosnode.get_node_names() and
                    rosnode.rosnode_ping(name, max_count=1, verbose=False))

        if self.mode in ('ARMED', 'FLYING'):
            raise RuntimeError('Node startup requires the drone to be disarmed')
        rospy.loginfo('Checking flight-controller status')
        if running('/flight_controller_node'):
            mode = rospy.wait_for_message('/pidrone/mode', Mode, timeout=2.0)
            if mode.mode != 'DISARMED':
                raise RuntimeError('Node startup requires the drone to be disarmed')
        scripts = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        launch = os.path.join(os.path.dirname(scripts), 'launch')
        nodes = [
            ('/raspicam_node', ['roslaunch', os.path.join(launch, 'raspicam_node.launch')]),
            ('/vl53l1x', ['bash', '-c',
                         'source "$HOME/catkin_ws/install/setup.bash" && exec roslaunch "$1"',
                         'pals-tof', os.path.join(launch, 'tof.launch')]),
            ('/optical_flow_node', [sys.executable, '-u', 'optical_flow_node.py']),
            ('/state_estimator', [sys.executable, '-u', 'state_estimator.py', '-p', 'ema']),
            ('/state_estimator_ema', [sys.executable, '-u', 'StateEstimators/state_estimator_ema.py']),
            ('/pid_controller', [sys.executable, '-u', 'pid_controller.py']),
            ('/flight_controller_node', [sys.executable, '-u', 'flight_controller_node.py']),
        ]
        for name, command in nodes:
            if rospy.is_shutdown():
                raise RuntimeError('ROS shut down during node startup')
            rospy.loginfo('Checking %s', name)
            if running(name):
                rospy.loginfo('%s is running', name)
                continue
            # Detached processes remain available after this routine finishes.
            # Each attempt has a separate log, including early startup failures.
            with tempfile.NamedTemporaryFile(
                    prefix='pals-' + name.strip('/') + '-', suffix='.log',
                    delete=False) as log:
                log_path = log.name
                rospy.loginfo('Starting %s; log: %s', name, log_path)
                with open(os.devnull, 'rb') as stdin:
                    process = subprocess.Popen(
                        ['setsid'] + command, cwd=scripts, stdin=stdin, stdout=log,
                        stderr=subprocess.STDOUT)
            deadline = time.time() + 20.0
            while time.time() < deadline and not rospy.is_shutdown():
                if process.poll() is not None:
                    raise RuntimeError('{} exited during startup; see {}'.format(name, log_path))
                if running(name):
                    rospy.loginfo('%s started', name)
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError('{} did not become reachable; see {}'.format(name, log_path))

    def _set_mode(self, name):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.mode_pub.publish(Mode(name))
            if self.mode == name:
                return
            rospy.sleep(0.1)
        raise RuntimeError('Mode {} was not confirmed'.format(name))

    def lock_xy(self):
        """
        Hold the current optical-flow position while yaw stays unchanged.
        """
        if self.mode not in ('ARMED', 'FLYING'):
            raise RuntimeError('Arm before locking X/Y')
        self._check_sensors()
        self.xy_anchor = (self.right_m, self.forward_m)
        self.xy_hold_valid = True
        rospy.loginfo('Relative X/Y hold enabled')

    def unlock_xy(self):
        self.xy_anchor = None
        self.twist_pub.publish(Twist())

    def _xy_hold_command(self):
        command = Twist()
        if self.xy_anchor is None:
            return command
        self._check_sensors()
        if not self.xy_hold_valid:
            raise RuntimeError('X/Y hold lost optical-flow continuity')
        command.linear.x = 0.5 * (self.xy_anchor[0] - self.right_m)
        command.linear.y = 0.5 * (self.xy_anchor[1] - self.forward_m)
        self._limit_planar_speed(command, 0.10)
        return command

    def hover(self, seconds=1.0):
        self.twist_pub.publish(self._xy_hold_command())
        deadline = time.time() + seconds
        while time.time() < deadline:
            self._check_flight('hover')
            self.twist_pub.publish(self._xy_hold_command())
            rospy.sleep(max(0.0, min(0.1, deadline - time.time())))

    def disarm(self):
        self._set_mode('DISARMED')
        self.unlock_xy()

    def takeoff(self, hold_xy=False):
        """Take off, optionally holding relative X/Y from before liftoff."""
        if self.ground_height is None or self.mode != 'DISARMED':
            raise RuntimeError('Call ready() while the drone is disarmed before takeoff')
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
        self._check_sensors(need_flow=False)
        self._check_battery()
        self.twist_pub.publish(self._xy_hold_command())
        if hold_xy:
            self.lock_xy()
        self._set_mode('FLYING')
        deadline = time.time() + 10.0
        settled_since = None
        while time.time() < deadline:
            self._check_flight('takeoff')
            if self.height > 0.40:
                raise RuntimeError('Takeoff rose above the expected height')
            if hold_xy:
                self.twist_pub.publish(self._xy_hold_command())
            now = time.time()
            settled_since = self._settled_since(self.takeoff_height, 0.04, settled_since, now)
            if settled_since is not None and now - settled_since >= self.ALTITUDE_SETTLE_SECONDS:
                # Keep the anchor in the same coordinate frame.
                if self.xy_anchor is None:
                    self.right_m = self.forward_m = 0.0
                self.target_height = self.takeoff_height
                rospy.loginfo('Takeoff altitude held for %.1f seconds; continuing routine',
                              self.ALTITUDE_SETTLE_SECONDS)
                return
            rospy.sleep(0.1)
        raise RuntimeError('Takeoff altitude did not settle within 10 seconds')

    def move(self, x=0.0, y=0.0, z=0.0):
        """Move relative to the drone: x right, y forward, z up (meters).

        Horizontal distance comes from optical flow and is approximate.
        """
        if self.mode != 'FLYING':
            raise RuntimeError('Drone must be flying before moving')
        if x == 0 and y == 0 and z == 0:
            return
        horizontal = math.hypot(x, y)
        if horizontal and self.xy_anchor is not None:
            raise ValueError('Unlock X/Y before commanding horizontal movement')
        self._check_sensors(need_flow=horizontal > 0)
        start_x, start_y, start_z = self.right_m, self.forward_m, self.height
        target_x, target_y, target_z = start_x + x, start_y + y, start_z + z
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
                self._check_flight('movement', need_flow=horizontal > 0)
                now = time.time()
                commanded_distance += last_command_speed * (now - last_command_at)
                last_command_at = now
                if horizontal and commanded_distance > max(0.4, horizontal * 1.5):
                    raise RuntimeError('Movement limit reached without reaching target')
                ex = target_x - self.right_m
                ey = target_y - self.forward_m
                ez = target_z - self.height
                horizontal_done = (not horizontal or
                                   (abs(ex) <= 0.06 and abs(ey) <= 0.06))
                if z:
                    progress_z = (self.height - start_z) * (1 if z > 0 else -1)
                    best_vertical_progress = max(best_vertical_progress, progress_z)
                    settled_since = self._settled_since(target_z, 0.03, settled_since, now)
                vertical_done = (not z or (settled_since is not None and
                                 now - settled_since >= self.ALTITUDE_SETTLE_SECONDS))
                if horizontal_done and vertical_done:
                    return
                if now - started > 3.0:
                    progress_xy = ((self.right_m - start_x) * x +
                                   (self.forward_m - start_y) * y)
                    if horizontal and progress_xy / horizontal < 0.03:
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
                    # Landing must remain possible after optical-flow failure.
                    rospy.logwarn('Releasing X/Y hold during landing: %s', error)
                    self.unlock_xy()
                    hold_xy = False
            now = time.time()
            if now - last_step >= self.LAND_STEP_SECONDS:
                self._set_altitude(max(0.0, self.target_height - self.LAND_STEP_M))
                last_step = now
            rospy.sleep(0.1)
        raise RuntimeError('Landing was not confirmed; drone is still armed')
