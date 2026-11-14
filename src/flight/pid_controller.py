#!/usr/bin/env python

import tf
import sys
import os
import rospy
import signal
import functools
import math
import threading
import yaml
import numpy as np
from flight.altitude_controller import AltitudeController
from flight.pid_class import PIDaxis
from flight.velocity_controller import VelocityController
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from pidrone_pkg.msg import AnchorLayout, FlightReadiness, Mode, RC, State
from std_msgs.msg import Empty, Bool, UInt32, Float32
from localization.fusion import rotate, target_velocity, XY_PREDICTION_TIMEOUT
from flight.distance_guard import DistanceGuard, fresh, uncertainty, STATE_TIMEOUT, automatic_coordination
from localization import monotonic
from flight.three_dim_vec import Position, Velocity, Error, RPY


def synchronized(method):
    """Keep ROS callbacks from changing a target halfway through a PID step."""
    @functools.wraps(method)
    def call(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return call


class PIDController(object):
    ''' Controls the flight of the drone by running a PID controller on the
    error calculated by the desired and current velocity and position of the drone
    '''

    def __init__(self):
        self.lock = threading.RLock()
        self.current_mode = 'DISARMED'
        self.desired_mode = 'DISARMED'
        self.takeoff_height = float(rospy.get_param('/pidrone/altitude/takeoff_height', 0.25))
        self.requested_takeoff_height = None
        settings = dict(rospy.get_param('/pidrone/altitude/controller', {}))
        if 'throttle_range' in settings:
            settings['throttle_range'] = tuple(settings['throttle_range'])
        self.altitude = AltitudeController(**settings)
        self.measurement_timeout = float(rospy.get_param('/pidrone/altitude/measurement_timeout', 0.5))

        self.position_control = False

        self.current_position = Position()
        self.desired_position = Position(z=self.takeoff_height)
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.1
        self.velocity_deadline = self.yaw_deadline = None

        # Horizontal velocity -> lean angle (roll/pitch); settings from params/velocity.yaml
        self.velocity = VelocityController(**dict(rospy.get_param('/pidrone/velocity/controller', {})))

        self.pid_error = Error()  # Horizontal velocity error, m/s.

        self.lr_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))

        self.last_pose_time = None

        self.desired_yaw_velocity = 0

        self.current_rpy = RPY()

        self.current_state = State()

        self.cmdpub = None

        # Fall back to velocity control beyond this local-position error (m).
        self.safety_threshold = 1.5

        self.local_estimate_ready = self.room_estimate_ready = False
        self.room_pose = self.room_target = self.room_anchors = None
        self.room_generation = None
        self.room_target_gain = float(rospy.get_param('/pidrone/localization/target_gain', 0.30))
        self.room_speed_limit = float(rospy.get_param('/pidrone/localization/target_speed_limit', 0.10))
        if not all(np.isfinite(v) and v > 0 for v in (self.room_target_gain, self.room_speed_limit)):
            raise ValueError('Room target gain and speed must be positive')
        self.coordination_config = rospy.get_param('/pidrone/coordination', None)
        self.distance_guard = None
        self.peer_state = self.peer_received = None
        self.peer_preview = self.peer_preview_received = None
        self.coordination_status_pub = None
        self.preflight_pub = None
        self.coordination_published = None
        self.coordination_published_at = None
        self.coordination_status = 'disabled'
        self.coordination_error = ''
        self.coordination_enabled = False
        self.coordination_active = False
        self.coordination_selection = self.coordination_selection_received = None
        if self.coordination_config is not None:
            try:
                config = self.coordination_config
                if not isinstance(config, dict):
                    raise ValueError('Coordination must be a parameter dictionary')
                own, peer = config['drone_id'], config['peer_id']
                if (not all(isinstance(v, (str, type(u''))) and v for v in (own, peer)) or own == peer):
                    raise ValueError('Use two distinct drone identities')
                speed_limit = min(.15, self.room_speed_limit)
                # The first ID alphabetically leads; the follower needs extra speed.
                if own < peer:
                    speed_limit *= 2. / 3.
                self.distance_guard = DistanceGuard(float(config['min_distance']),
                    float(config['max_distance']), float(config['nominal_separation_m']),
                    self.room_target_gain, speed_limit, follow_peer=own > peer)
                self.coordination_status = 'waiting_for_peer'
            except (KeyError, TypeError, ValueError) as error:
                # If the guard is enabled, block horizontal motion, not altitude control.
                self.coordination_error = 'invalid_configuration'
                rospy.logwarn('Distance control configuration: %s', error)

    @synchronized
    def coordination_enabled_callback(self, msg):
        # An explicit operator override lasts until changed while disarmed.
        if self.current_mode == 'DISARMED':
            self.coordination_enabled = bool(msg.data)
            self.coordination_active = self.coordination_enabled

    @synchronized
    def coordination_selection_callback(self, msg):
        # Keep the flight setup through connection losses after arming.
        if self.current_mode == 'DISARMED':
            self.coordination_selection = msg.data
            self.coordination_selection_received = monotonic()

    def accept_peer(self, msg, previous):
        config = self.coordination_config
        if (self.distance_guard is None or msg.drone_id != config['peer_id']
                or msg.recipient_id != config['drone_id']):
            return False
        stamp = msg.header.stamp.to_sec()
        if not fresh(stamp, rospy.get_time()):
            return False
        # Repeated packets must not extend the freshness timeout.
        return previous is None or stamp > previous.header.stamp.to_sec()

    @synchronized
    def peer_preview_callback(self, msg):
        # Use this stream only before takeoff.
        if self.current_mode in ('DISARMED', 'ARMED') and self.accept_peer(msg, self.peer_preview):
            self.peer_preview, self.peer_preview_received = msg, monotonic()

    @synchronized
    def peer_state_callback(self, msg):
        if not self.accept_peer(msg, self.peer_state):
            return
        self.peer_state, self.peer_received = msg, monotonic()
        if self.current_mode == 'FLYING' and self.coordination_enabled:
            # Once a peer joins, keep paired protection until disarm.
            self.coordination_active = True

    def peer_margin(self, peer, received):
        """Validate both positions and return their combined uncertainty."""
        pose = self.room_pose
        if self.coordination_error:
            return None, self.coordination_error
        if not self.room_ready(allow_prediction=False):
            return None, 'local_position_unavailable'
        if (peer is None or not fresh(peer.header.stamp.to_sec(), rospy.get_time()) or
                not 0 <= monotonic() - received <= STATE_TIMEOUT):
            return None, 'peer_state_stale'
        if peer.header.frame_id != pose.header.frame_id or peer.recipient_generation != self.room_generation:
            return None, 'reference_mismatch'
        try:
            margin = uncertainty(pose.pose.covariance) + uncertainty(peer.pose.covariance)
        except (ValueError, TypeError, IndexError, OverflowError):
            return None, 'invalid_covariance'
        return margin, ''

    def flight_readiness(self):
        report = FlightReadiness()
        report.mode = self.current_mode
        report.paired = self.coordination_enabled and self.coordination_active
        if self.current_mode not in ('DISARMED', 'ARMED'):
            report.reason = 'already_flying'
            return report
        if self.current_mode == 'DISARMED':
            received = self.coordination_selection_received
            if received is None or not 0 <= monotonic() - received <= 2.:
                report.reason = 'flight_setup_unavailable'
                return report
            report.paired = self.coordination_enabled and bool(self.coordination_selection)
        if self.coordination_enabled and (self.coordination_error or self.distance_guard is None):
            report.reason = self.coordination_error or 'guard_unavailable'
            return report
        if not self.local_ready():
            report.reason = 'local_estimate_unavailable'
            return report
        report.header = self.current_state.header
        if report.paired:
            peer = self.peer_preview
            margin, report.reason = self.peer_margin(peer, self.peer_preview_received)
            if margin is None:
                return report
            p, other = self.room_pose.pose.pose.position, peer.pose.pose.position
            try:
                report.reason = self.distance_guard.preflight((p.x, p.y), (other.x, other.y), margin)
            except (ValueError, TypeError, OverflowError):
                report.reason = 'invalid_state'
            if report.reason:
                return report
            report.header = min((self.current_state.header, self.room_pose.header, peer.header),
                                key=lambda header: header.stamp.to_sec())
        report.ready, report.reason = True, 'ready'
        return report

    def coordinated_velocity(self, requested):
        """Filter the final body XY request after all route/hold corrections."""
        peer, pose = self.peer_state, self.room_pose
        margin, error = self.peer_margin(peer, self.peer_received)
        if error:
            return (0, 0), error
        p, q = pose.pose.pose.position, pose.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        world, status = self.distance_guard.filter(
            (p.x, p.y), (peer.pose.pose.position.x, peer.pose.pose.position.y),
            (peer.velocity.x, peer.velocity.y), rotate(requested, angle), margin)
        return rotate(world, -angle), status

    def local_ready(self):
        # Fusion owns sensor validity. Only watchdog its published estimate here.
        return self.local_estimate_ready and self.altitude_state_issue(rospy.get_time()) is None

    def altitude_state_issue(self, now):
        stamp = self.current_state.header.stamp.to_sec()
        if stamp <= 0:
            return 'no state received'
        age = now - stamp
        if not 0 <= age <= self.measurement_timeout:
            return 'state age %.3f s outside [0, %.3f] s' % (age, self.measurement_timeout)
        if not np.isfinite(self.current_position.z):
            return 'nonfinite height'
        return None

    def room_ready(self, allow_prediction=True):
        timeout = XY_PREDICTION_TIMEOUT if allow_prediction else self.measurement_timeout
        return (self.local_ready() and self.room_estimate_ready
                and self.room_pose is not None
                and 0 <= rospy.get_time() - self.room_pose.header.stamp.to_sec() <= timeout)

    @synchronized
    def local_ready_callback(self, msg):
        self.local_estimate_ready = msg.data

    @synchronized
    def room_ready_callback(self, msg):
        self.room_estimate_ready = msg.data

    @synchronized
    def room_pose_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (msg.header.frame_id and np.all(np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]))
                and q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-12):
            self.room_pose = msg

    @synchronized
    def room_anchors_callback(self, msg):
        self.room_anchors = msg

    @synchronized
    def room_generation_callback(self, msg):
        if self.room_generation is not None and self.room_generation != msg.data:
            self.cancel_room_target()
            self.peer_state = self.peer_received = None
            self.peer_preview = self.peer_preview_received = None
        self.room_generation = msg.data

    @synchronized
    def cancel_room_target(self, _msg=None):
        if self.room_target is not None:
            self.room_target = None
            self.desired_velocity.x = self.desired_velocity.y = 0
            self.desired_position.x, self.desired_position.y = self.current_position.x, self.current_position.y
            self.lr_pid.reset()
            self.fb_pid.reset()

    @synchronized
    def room_target_callback(self, msg):
        if (not self.room_ready(allow_prediction=False) or self.current_mode != 'FLYING'
                or msg.header.frame_id != self.room_pose.header.frame_id
                or not 0 <= rospy.get_time() - msg.header.stamp.to_sec() <= 0.5
                or not np.all(np.isfinite([msg.pose.position.x, msg.pose.position.y]))):
            rospy.logwarn('Ignoring room target: flight mode, frames or localization unavailable')
            return
        anchors = self.room_anchors
        if (anchors is None or anchors.header.frame_id != msg.header.frame_id
                or not 0 <= rospy.get_time() - anchors.header.stamp.to_sec() <= 0.5
                or len(anchors.positions) != 4 or len(set(anchors.anchor_ids)) != 4):
            return
        xy = np.array([(p.x, p.y) for p in anchors.positions])
        if not np.all(np.isfinite(xy)):
            return
        center = xy.mean(axis=0)
        hull = xy[np.argsort(np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0]))]
        point = np.array([msg.pose.position.x, msg.pose.position.y])
        edges = np.roll(hull, -1, axis=0) - hull
        offsets = point - hull
        if np.any(edges[:, 0] * offsets[:, 1] - edges[:, 1] * offsets[:, 0] < -1e-6):
            rospy.logwarn('Ignoring room target outside anchor outline')
            return
        self.room_target = point
        self.desired_velocity.x = self.desired_velocity.y = 0
        self.position_control = True
        self.position_control_pub.publish(True)


    @synchronized
    def current_state_callback(self, state):
        """ Store the drone's current state for calculations """
        self.current_state = state
        self.state_to_three_dim_vec_structs()

    @synchronized
    def desired_pose_callback(self, msg):
        """ Update the desired pose """
        self.cancel_room_target()
        desired_z = self.desired_position.z + msg.position.z
        if not np.isfinite(desired_z) or desired_z < 0:
            rospy.logwarn('Ignoring invalid altitude target: %s', desired_z)
            return
        self.altitude.cancel_takeoff()
        self.altitude.cancel_landing()
        if msg.position.x or msg.position.y or not self.position_control:
            # A relative altitude-only command must not move an active X/Y hold
            # point to the drone's latest (possibly drifting) position.
            offset = rotate((msg.position.x, msg.position.y), self.current_rpy.y)
            self.desired_position.x = self.current_position.x + offset[0]
            self.desired_position.y = self.current_position.y + offset[1]
        self.desired_position.z = desired_z

    @synchronized
    def takeoff_height_callback(self, msg):
        """Set an absolute takeoff destination without canceling the climb ramp."""
        if not np.isfinite(msg.data) or msg.data <= 0:
            rospy.logwarn('Ignoring invalid takeoff height: %s', msg.data)
            return
        self.requested_takeoff_height = msg.data
        self.desired_position.z = msg.data

    @synchronized
    def landing_callback(self, msg):
        """Descend using the floor height measured before this flight."""
        if self.current_mode != 'FLYING' or not np.isfinite(msg.data) or not 0 <= msg.data <= 0.12:
            rospy.logwarn('Ignoring landing request: flight mode or floor height invalid')
            return
        self.altitude.start_landing(msg.data)
        self.desired_position.z = msg.data

    @synchronized
    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.cancel_room_target()
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_yaw_velocity = msg.angular.z
        now = rospy.get_time()
        speed = self.desired_velocity.planar_magnitude()
        self.velocity_deadline = now + self.desired_velocity_travel_distance / speed if speed else None
        self.yaw_deadline = now + 0.25 if self.desired_yaw_velocity else None

    @synchronized
    def current_mode_callback(self, msg):
        """ Update the current mode """
        previous = self.current_mode
        if previous == 'DISARMED' and msg.mode in ('ARMED', 'FLYING'):
            received = self.coordination_selection_received
            fresh_selection = received is not None and 0 <= monotonic() - received <= 2.
            # An enabled guard defaults to paired protection if the selection is invalid.
            self.coordination_active = self.coordination_enabled and (
                bool(self.coordination_selection) if fresh_selection and not self.coordination_error else True)
        if msg.mode == 'DISARMED' and previous != 'DISARMED':
            self.requested_takeoff_height = None
            self.coordination_selection = self.coordination_selection_received = None
            self.coordination_active = self.coordination_enabled
        self.current_mode = msg.mode
        if previous != msg.mode:
            self.cancel_room_target()
            self.peer_state = self.peer_received = None
            self.peer_preview = self.peer_preview_received = None
            self.altitude.cancel_takeoff()
            self.altitude.cancel_landing()
            rospy.loginfo('PID mode %s -> %s', previous, msg.mode)
            if msg.mode in ('ARMED', 'FLYING'):
                self.reset()
                if msg.mode == 'FLYING':
                    self.altitude.start_takeoff(self.current_position.z, rospy.get_time())
                if msg.mode == 'ARMED' and hasattr(self, 'position_control_pub'):
                    self.position_control_pub.publish(False)

    @synchronized
    def desired_mode_callback(self, msg):
        """ Update the desired mode """
        self.desired_mode = msg.mode

    @synchronized
    def position_control_callback(self, msg):
        """ Set whether or not position control is enabled """
        if not msg.data:
            self.cancel_room_target()
        if msg.data and not self.position_control:
            self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        self.position_control = msg.data

    @synchronized
    def reset_callback(self, empty):
        """Reset the XY target to the current position and stop XY motion."""
        self.cancel_room_target()
        self.altitude.cancel_landing()
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        self.desired_velocity.x = 0
        self.desired_velocity.y = 0

    @synchronized
    def step(self):
        """ Returns the commands generated by the pid """
        if self.room_target is None:
            self.expire_velocity_command()
        xy_ready = self.calc_error()

        # Learn the constant push only from a fresh IMU/flow velocity estimate.
        # calc_error already applies the spacing filter to the velocity request.
        # Keep adapting to that request so spacing corrections can unwind old trim.
        now = rospy.get_time()
        command = self.velocity.step((self.pid_error.x, self.pid_error.y), now, xy_ready)
        command.append(1500 + self.desired_yaw_velocity)
        tilt_cos = np.cos(self.current_rpy.r) * np.cos(self.current_rpy.p)
        command.append(self.altitude.step(self.desired_position.z, self.current_position.z,
                                          self.current_velocity.z, tilt_cos, now))
        return command

    def state_to_three_dim_vec_structs(self):
        """
        Convert the values from the state estimator into ThreeDimVec structs to
        make calculations concise
        """
        pose = self.current_state.pose_with_covariance.pose
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        self.current_position.z = pose.position.z

        twist = self.current_state.twist_with_covariance.twist
        self.current_velocity.x = twist.linear.x
        self.current_velocity.y = twist.linear.y
        self.current_velocity.z = twist.linear.z

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        r,p,y = tf.transformations.euler_from_quaternion(quaternion)
        self.current_rpy = RPY(r,p,y)

    def expire_velocity_command(self):
        """Expire manual commands before computing the current PID output."""
        now = rospy.get_time()
        if self.velocity_deadline is not None and now >= self.velocity_deadline:
            self.desired_velocity.x = self.desired_velocity.y = 0
            self.velocity_deadline = None
        if self.yaw_deadline is not None and now >= self.yaw_deadline:
            self.desired_yaw_velocity = 0
            self.yaw_deadline = None

    @synchronized
    def calc_error(self):
        """Apply horizontal position and spacing corrections to the velocity error."""
        now = rospy.get_time()
        pose_dt = max(0.0, now - self.last_pose_time) if self.last_pose_time is not None else 0.0
        self.last_pose_time = now
        xy_ready = self.local_ready()
        if not xy_ready or (self.room_target is not None and not self.room_ready()):
            # Pause motion during a gap while preserving the hold target and learned trim.
            self.desired_velocity.x = self.desired_velocity.y = 0
        elif self.room_target is not None:
            p, q = self.room_pose.pose.pose.position, self.room_pose.pose.pose.orientation
            yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            self.desired_velocity.x, self.desired_velocity.y = target_velocity(
                self.room_target, (p.x, p.y), yaw, self.room_target_gain, self.room_speed_limit)

        if xy_ready:
            self.pid_error.x = self.desired_velocity.x - self.current_velocity.x
            self.pid_error.y = self.desired_velocity.y - self.current_velocity.y
        else:
            self.pid_error.x = self.pid_error.y = 0
        if xy_ready and self.position_control and self.room_target is None:
            dx = self.desired_position.x - self.current_position.x
            dy = self.desired_position.y - self.current_position.y
            if math.hypot(dx, dy) >= self.safety_threshold:
                self.position_control = False
                self.position_control_pub.publish(False)
            else:
                # The position PIDs return cm/s; the velocity controller uses m/s.
                right, forward = rotate((dx, dy), -self.current_rpy.y)
                self.pid_error.x += self.lr_pid.step(right, pose_dt) / 100.
                self.pid_error.y += self.fb_pid.step(forward, pose_dt) / 100.
        if not self.coordination_enabled:
            self.coordination_status = 'disabled'
        elif self.coordination_config is not None and self.coordination_active and self.current_mode == 'FLYING':
            # Include both room-target commands and local-position PID corrections.
            # Do not overwrite the route's requested velocity or its stored target.
            requested = (self.current_velocity.x + self.pid_error.x,
                         self.current_velocity.y + self.pid_error.y)
            command, self.coordination_status = self.coordinated_velocity(requested)
            for axis, value in zip(('x', 'y'), command):
                measured = getattr(self.current_velocity, axis)
                setattr(self.pid_error, axis, value - measured if xy_ready and np.isfinite(measured) else 0)
        elif self.coordination_config is not None:
            self.coordination_status = 'inactive' if self.coordination_active else 'solo'
        report = self.flight_readiness() if self.preflight_pub is not None else None
        status = (self.coordination_status, self.current_mode,
                  (report.ready, report.reason, report.paired) if report is not None else None)
        now = monotonic()
        if (status != self.coordination_published or self.coordination_published_at is None
                or now - self.coordination_published_at >= 0.1):
            if self.coordination_status_pub is not None:
                self.coordination_status_pub.publish(self.coordination_status)
            if report is not None:
                self.preflight_pub.publish(report)
            self.coordination_published, self.coordination_published_at = status, now
        return xy_ready

    def reset(self):
        """Reset targets, command deadlines and PID state for a flight mode change."""
        height = self.takeoff_height if self.requested_takeoff_height is None else self.requested_takeoff_height
        self.desired_position = Position(self.current_position.x, self.current_position.y, height)
        rospy.loginfo("PID reset target=%s m source=%s", self.desired_position.z, __file__)
        self.desired_velocity = Velocity(0,0,0)
        self.desired_yaw_velocity = 0
        self.velocity_deadline = self.yaw_deadline = None
        self.last_pose_time = None
        self.velocity.reset()
        self.altitude.reset()
        self.lr_pid.reset()
        self.fb_pid.reset()

    def ctrl_c_handler(self, signal, frame):
        """ Gracefully handles ctrl-c """
        print('Caught ctrl-c\n Stopping Controller')
        sys.exit()

    def publish_cmd(self, cmd):
        """Publish the controls to /pidrone/fly_commands """
        msg = RC()
        msg.roll = cmd[0]
        msg.pitch = cmd[1]
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmdpub.publish(msg)


def main(ControllerClass):
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
    if not rospy.has_param('/pidrone/coordination'):
        try:
            with open(os.path.join(params_dir, 'payload_geometry.yaml')) as stream:
                geometry = yaml.safe_load(stream)
            with open(os.path.join(params_dir, 'coordination.yaml')) as stream:
                settings = yaml.safe_load(stream)
            config = automatic_coordination(geometry, settings, os.environ.get('ROS_MASTER_URI', ''))
        except (IOError, ValueError, TypeError, yaml.YAMLError) as error:
            # Keep the PID running; an enabled guard will block horizontal motion.
            rospy.logwarn('Automatic coordination configuration: %s', error)
            config = {}
        rospy.set_param('/pidrone/coordination', config)
    if not rospy.has_param('/pidrone/altitude/controller'):
        with open(os.path.join(params_dir, 'altitude.yaml')) as stream:
            settings = yaml.safe_load(stream)
        rospy.set_param('/pidrone/altitude/controller', settings)
    if not rospy.has_param('/pidrone/velocity/controller'):
        with open(os.path.join(params_dir, 'velocity.yaml')) as stream:
            rospy.set_param('/pidrone/velocity/controller', yaml.safe_load(stream))

    pid_controller = ControllerClass()

    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1, tcp_nodelay=True)
    pid_controller.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
    pid_controller.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/pid_controller', Empty, queue_size=1)

    rospy.Subscriber('/pidrone/state', State, pid_controller.current_state_callback,
                     queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/pidrone/desired/pose', Pose, pid_controller.desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/land', Float32, pid_controller.landing_callback, queue_size=1)
    rospy.Subscriber('/pidrone/desired/takeoff_height', Float32, pid_controller.takeoff_height_callback, queue_size=1)
    rospy.Subscriber('/pidrone/desired/twist', Twist, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired/mode', Mode, pid_controller.desired_mode_callback)
    rospy.Subscriber('/pidrone/position_control', Bool, pid_controller.position_control_callback)
    rospy.Subscriber('/pidrone/reset_transform', Empty, pid_controller.reset_callback)
    from pidrone_pkg.msg import PeerState
    from std_msgs.msg import String
    pid_controller.coordination_status_pub = rospy.Publisher('/pidrone/coordination/status', String, queue_size=1)
    pid_controller.preflight_pub = rospy.Publisher('/pidrone/coordination/preflight', FlightReadiness, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/enabled', Bool, pid_controller.coordination_enabled_callback, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/paired', Bool, pid_controller.coordination_selection_callback, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/peer', PeerState, pid_controller.peer_state_callback, queue_size=1)
    rospy.Subscriber('/pidrone/coordination/peer_preview', PeerState, pid_controller.peer_preview_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/local_ready', Bool, pid_controller.local_ready_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/ready', Bool, pid_controller.room_ready_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/fused', PoseWithCovarianceStamped, pid_controller.room_pose_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/anchors', AnchorLayout, pid_controller.room_anchors_callback, queue_size=1)
    rospy.Subscriber('/pidrone/localization/generation', UInt32, pid_controller.room_generation_callback, queue_size=1)
    rospy.Subscriber('/pidrone/desired/room_target', PoseStamped, pid_controller.room_target_callback, queue_size=1)
    rospy.Subscriber('/pidrone/desired/cancel_room_target', Empty, pid_controller.cancel_room_target, queue_size=1)

    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    loop_rate = rospy.Rate(60)
    print('PID Controller Started')
    while not rospy.is_shutdown():
        # Validate and use the same state under the callback lock.
        with pid_controller.lock:
            issue = pid_controller.altitude_state_issue(rospy.get_time())
            if issue is None:
                pid_controller.heartbeat_pub.publish(Empty())
                fly_command = pid_controller.step()
                if pid_controller.current_mode == 'FLYING' and pid_controller.desired_mode == 'FLYING':
                    pid_controller.publish_cmd(fly_command)
                    # Once per second in flight, for throttle statistics after each run.
                    rospy.loginfo_throttle(1, 'Altitude target=%s estimated=%.3f m climb=%.3f m/s throttle=%.1f' % (
                        pid_controller.altitude.target, pid_controller.current_position.z,
                        pid_controller.current_velocity.z, fly_command[3]))
        if issue is not None:
            # The FC's existing watchdog handles the missing PID heartbeat.
            rospy.logwarn_throttle(1, 'Altitude unavailable on /pidrone/state: %s. '
                                   'See /pidrone/localization/fusion_status and /pidrone/ekf/status' % issue)
        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)
