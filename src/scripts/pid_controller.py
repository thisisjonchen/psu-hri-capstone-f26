#!/usr/bin/env python

import tf
import sys
import os
import rospy
import signal
import argparse
import functools
import math
import threading
import numpy as np
import command_values as cmds
from altitude_controller import AltitudeController
from pid_class import PIDaxis
from velocity_controller import VelocityController
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from pidrone_pkg.msg import Mode, RC, State, FlightReadiness
from std_msgs.msg import Empty, Bool, UInt32
from localization.fusion import rotate, target_velocity, XY_PREDICTION_TIMEOUT
from localization.distance_guard import DistanceGuard, fresh, uncertainty, STATE_TIMEOUT, automatic_coordination
from localization.uwb_locator import monotonic
from pidrone_pkg.msg import AnchorLayout
from three_dim_vec import Position, Velocity, Error, RPY
try:
    from urllib.parse import urlparse
except ImportError:  # Python 2.7 on the drone
    from urlparse import urlparse


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
        # Initialize the current and desired modes
        self.current_mode = 'DISARMED'
        self.desired_mode = 'DISARMED'
        self.takeoff_height = float(rospy.get_param('/pidrone/altitude/takeoff_height', 0.25))
        settings = dict(rospy.get_param('/pidrone/altitude/controller', {}))
        if 'throttle_range' in settings:
            settings['throttle_range'] = tuple(settings['throttle_range'])
        self.altitude = AltitudeController(**settings)
        self.measurement_timeout = float(rospy.get_param('/pidrone/altitude/measurement_timeout', 0.5))
        rospy.loginfo('PID sources: controller=%s commands=%s', __file__, cmds.__file__)

        # Initialize in velocity control
        self.position_control = False

        # Initialize the current and desired positions
        self.current_position = Position()
        self.desired_position = Position(z=self.takeoff_height)

        # Initialize the position error
        self.position_error = Error()

        # Initialize the current and desired velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Initialize the velocity error
        self.velocity_error = Error()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.1
        self.velocity_deadline = self.yaw_deadline = None

        # Horizontal velocity -> lean angle (roll/pitch); settings from params/velocity.yaml
        self.velocity = VelocityController(**dict(rospy.get_param('/pidrone/velocity/controller', {})))

        # Initialize the planar velocity error used for the roll/pitch PID
        self.pid_error = Error()

        # Initialize the 'position error to velocity error' PIDs:
        # left/right (roll) pid
        self.lr_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))
        # front/back (pitch) pid
        self.fb_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))

        # Initialize the pose callback time
        self.last_pose_time = None

        # Initialize the desired yaw velocity
        self.desired_yaw_velocity = 0

        # Current body orientation
        self.current_rpy = RPY()

        # Latest controller estimate
        self.current_state = State()

        # Store the command publisher
        self.cmdpub = None

        # Fall back to velocity control beyond this local-position error (m).
        self.safety_threshold = 1.5

        # determines if the position of the drone is known
        self.lost = False

        # determines if the desired poses are aboslute or relative to the drone
        self.absolute_desired_position = False

        self.use_fusion = bool(rospy.get_param('/pidrone/localization/use_fusion', True))
        self.local_estimate_ready = self.room_estimate_ready = False
        self.room_pose = self.room_target = self.room_anchors = None
        self.room_generation = None
        self.room_target_gain = float(rospy.get_param('/pidrone/localization/target_gain', 0.20))
        self.room_speed_limit = float(rospy.get_param('/pidrone/localization/target_speed_limit', 0.10))
        if not all(np.isfinite(v) and v > 0 for v in (self.room_target_gain, self.room_speed_limit)):
            raise ValueError('Room target gain and speed must be positive')
        self.coordination_config = rospy.get_param('/pidrone/coordination', None)
        self.distance_guard = None
        self.peer_state = self.peer_received = None
        self.peer_preview = self.peer_preview_received = None
        self.coordination_status_pub = None
        self.preflight_pub = None
        self.coordination_status = 'disabled'
        self.coordination_error = ''
        self.coordination_active = True
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
                # Keep altitude control running, but block horizontal motion.
                self.coordination_error = 'invalid_configuration'
                rospy.logwarn('Distance control configuration: %s', error)

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
        if self.current_mode == 'FLYING':
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
        report.mode, report.paired = self.current_mode, self.coordination_active
        if self.current_mode not in ('DISARMED', 'ARMED'):
            report.reason = 'already_flying'
            return report
        if self.current_mode == 'DISARMED':
            received = self.coordination_selection_received
            if received is None or not 0 <= monotonic() - received <= 2.:
                report.reason = 'flight_setup_unavailable'
                return report
            report.paired = bool(self.coordination_selection)
        if self.coordination_error or self.distance_guard is None:
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
        stamp = self.current_state.header.stamp.to_sec()
        return (not self.use_fusion or self.local_estimate_ready and stamp > 0
                and 0 <= rospy.get_time() - stamp <= self.measurement_timeout)

    def room_ready(self, allow_prediction=True):
        timeout = XY_PREDICTION_TIMEOUT if allow_prediction else self.measurement_timeout
        return (self.use_fusion and self.local_ready() and self.room_estimate_ready
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


    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    @synchronized
    def current_state_callback(self, state):
        """ Store the drone's current state for calculations """
        self.current_state = state
        self.state_to_three_dim_vec_structs()

    @synchronized
    def desired_pose_callback(self, msg):
        """ Update the desired pose """
        self.cancel_room_target()
        desired_z = (msg.position.z if self.absolute_desired_position else
                     self.desired_position.z + msg.position.z)
        if not np.isfinite(desired_z) or desired_z < 0:
            rospy.logwarn('Ignoring invalid altitude target: %s', desired_z)
            return
        self.altitude.cancel_takeoff()
        # set the desired positions equal to the desired pose message
        if self.absolute_desired_position:
            self.desired_position.x = msg.position.x
            self.desired_position.y = msg.position.y
        # set the desired positions relative to the current position (except for z to make it more responsive)
        elif msg.position.x or msg.position.y or not self.position_control:
            # A relative altitude-only command must not move an active X/Y hold
            # point to the drone's latest (possibly drifting) position.
            offset = rotate((msg.position.x, msg.position.y), self.current_rpy.y) if self.use_fusion else (msg.position.x, msg.position.y)
            self.desired_position.x = self.current_position.x + offset[0]
            self.desired_position.y = self.current_position.y + offset[1]
        self.desired_position.z = desired_z

    @synchronized
    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.cancel_room_target()
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_velocity.z = msg.linear.z
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
            # Default to paired protection if the selection is missing or invalid.
            self.coordination_active = (bool(self.coordination_selection)
                                        if fresh_selection and not self.coordination_error else True)
        if msg.mode == 'DISARMED' and previous != 'DISARMED':
            self.coordination_selection = self.coordination_selection_received = None
            self.coordination_active = True
        self.current_mode = msg.mode
        if previous != msg.mode:
            self.cancel_room_target()
            self.peer_state = self.peer_received = None
            self.peer_preview = self.peer_preview_received = None
            self.altitude.cancel_takeoff()
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
        """ Reset the desired and current poses of the drone and set
        desired velocities to zero """
        self.cancel_room_target()
        if not self.use_fusion:
            self.current_position = Position(z=self.current_position.z)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        self.desired_velocity.x = 0
        self.desired_velocity.y = 0

    @synchronized
    def lost_callback(self, msg):
        if not self.use_fusion:
            self.lost = msg.data

    # Step Method
    #############
    @synchronized
    def step(self):
        """ Returns the commands generated by the pid """
        if self.room_target is None:
            self.expire_velocity_command()
        xy_ready = self.calc_error()
        requested_xy = (self.desired_velocity.x, self.desired_velocity.y)

        # Learn the constant push only from a fresh IMU/flow velocity estimate (no UWB needed).
        integrate_xy = xy_ready
        if self.coordination_config is not None and self.coordination_active and self.current_mode == 'FLYING':
            integrate_xy = integrate_xy and self.coordination_status == 'clear'
        now = rospy.get_time()
        command = self.velocity.step((self.pid_error.x / 100., self.pid_error.y / 100.), now, integrate_xy)
        command.append(1500 + self.desired_yaw_velocity)
        tilt_cos = np.cos(self.current_rpy.r) * np.cos(self.current_rpy.p)
        command.append(self.altitude.step(self.desired_position.z, self.current_position.z,
                                          self.current_velocity.z, tilt_cos, now))
        if self.current_mode == 'FLYING':
            filtered_xy = ((self.current_velocity.x + self.pid_error.x / 100.,
                            self.current_velocity.y + self.pid_error.y / 100.)
                           if xy_ready else (0., 0.))
            rospy.loginfo_throttle(1,
                'XY PID: local_ready=%s position_control=%s desired=(%.3f, %.3f) '
                'filtered=(%.3f, %.3f) coordination=%s '
                'measured=(%.3f, %.3f) m/s roll=%.1f pitch=%.1f lean=(%.1f, %.1f) deg '
                'learned push=(%.2f, %.2f) m/s^2' % (
                    xy_ready, self.position_control, requested_xy[0], requested_xy[1],
                    filtered_xy[0], filtered_xy[1], self.coordination_status,
                    self.current_velocity.x, self.current_velocity.y, command[0], command[1],
                    math.degrees(self.velocity.lean[0]), math.degrees(self.velocity.lean[1]),
                    self.velocity.integral[0], self.velocity.integral[1]))
        return command

    # HELPER METHODS
    ################
    def state_to_three_dim_vec_structs(self):
        """
        Convert the values from the state estimator into ThreeDimVec structs to
        make calculations concise
        """
        # store the positions
        pose = self.current_state.pose_with_covariance.pose
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        self.current_position.z = pose.position.z

        # store the linear velocities
        twist = self.current_state.twist_with_covariance.twist
        self.current_velocity.x = twist.linear.x
        self.current_velocity.y = twist.linear.y
        self.current_velocity.z = twist.linear.z

        # store the orientations
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
        """ Calculate the error in velocity, and if in position hold, add the
        error from lr_pid and fb_pid to the velocity error to control the
        position of the drone
        """
        now = rospy.get_time()
        pose_dt = max(0.0, now - self.last_pose_time) if self.last_pose_time is not None else 0.0
        self.last_pose_time = now
        xy_ready = self.local_ready()
        if not xy_ready:
            # Hold the learned trim (the lean that cancels a constant push) rather than
            # flying level: the push is still there while the estimate is unavailable,
            # e.g. below the optical-flow height while landing. Learning pauses meanwhile;
            # arming or a new flight clears it.
            self.desired_velocity.x = self.desired_velocity.y = 0
        if self.room_target is not None and not self.room_ready():
            # Pause correction without replacing the target or switching modes.
            self.desired_velocity.x = self.desired_velocity.y = 0
        elif self.room_target is not None:
            p, q = self.room_pose.pose.pose.position, self.room_pose.pose.pose.orientation
            angle = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            command = target_velocity(self.room_target, (p.x, p.y), angle,
                                      self.room_target_gain, self.room_speed_limit)
            self.desired_velocity.x, self.desired_velocity.y = command
        # calculate the velocity error
        self.velocity_error = self.desired_velocity - self.current_velocity
        self.pid_error.x = self.velocity_error.x
        self.pid_error.y = self.velocity_error.y
        self.pid_error.z = 0
        # multiply by 100 to account for the fact that code was originally written using cm
        self.pid_error = self.pid_error * 100
        if not xy_ready:
            self.pid_error.x = self.pid_error.y = 0
        if self.position_control and self.room_target is None and xy_ready:
            # calculate the position error
            self.position_error = self.desired_position - self.current_position
            if self.use_fusion:
                error = rotate((self.position_error.x, self.position_error.y), -self.current_rpy.y)
                self.position_error.x, self.position_error.y = error
            if self.position_error.planar_magnitude() >= self.safety_threshold or self.lost:
                self.position_control = False
                self.position_control_pub.publish(False)
            else:
                # Position correction supplements, rather than removes, velocity damping.
                self.pid_error.x += self.lr_pid.step(self.position_error.x, pose_dt)
                self.pid_error.y += self.fb_pid.step(self.position_error.y, pose_dt)
        if self.coordination_config is not None and self.coordination_active and self.current_mode == 'FLYING':
            # Include both room-target commands and local-position PID corrections.
            # Do not overwrite the route's requested velocity or its stored target.
            requested = (self.current_velocity.x + self.pid_error.x / 100.,
                         self.current_velocity.y + self.pid_error.y / 100.)
            command, self.coordination_status = self.coordinated_velocity(requested)
            for axis, value in zip(('x', 'y'), command):
                measured = getattr(self.current_velocity, axis)
                setattr(self.pid_error, axis, 100 * (value - measured)
                        if xy_ready and np.isfinite(measured) else 0)
        elif self.coordination_config is not None:
            self.coordination_status = 'inactive' if self.coordination_active else 'solo'
        if self.coordination_status_pub is not None:
            self.coordination_status_pub.publish(self.coordination_status)
        if self.preflight_pub is not None:
            self.preflight_pub.publish(self.flight_readiness())
        return xy_ready

    def reset(self):
        """Reset targets, command deadlines and PID state for a flight mode change."""
        # reset position control variables
        self.position_error = Error(0,0,0)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.takeoff_height)
        rospy.loginfo("PID reset target=%s m source=%s", self.desired_position.z, __file__)
        # reset velocity control_variables
        self.velocity_error = Error(0,0,0)
        self.desired_velocity = Velocity(0,0,0)
        self.desired_yaw_velocity = 0
        self.velocity_deadline = self.yaw_deadline = None
        self.last_pose_time = None
        # reset the pids
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

    parser = argparse.ArgumentParser(
                    prog='PID Controller',
                    description= """
                    Controls the flight of the drone by running a PID controller on the error calculated by 
                    the desired and current velocity and position of the drone
                    """,
                    epilog='For more info check https://github.com/h2r/pidrone_pkg/tree/dd24'
                    )

    parser.add_argument(
        '-v',
        '--verbose',
	type=int,
        choices=[0,1,2],
        default=0,
        help="Verbosity between 0 and 2, 2 is most verbose"
        )
    
    args = parser.parse_args()

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # Load defaults when no coordination settings were supplied.
    if not rospy.has_param('/pidrone/coordination'):
        import yaml
        params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
        try:
            with open(os.path.join(params_dir, 'payload_geometry.yaml')) as stream:
                geometry = yaml.safe_load(stream)
            with open(os.path.join(params_dir, 'coordination.yaml')) as stream:
                settings = yaml.safe_load(stream)
            config = automatic_coordination(geometry, settings, os.environ.get('ROS_MASTER_URI', ''))
        except (IOError, ValueError, TypeError, yaml.YAMLError) as error:
            # Keep the PID running, but block horizontal motion.
            rospy.logwarn('Automatic coordination configuration: %s', error)
            config = {}
        rospy.set_param('/pidrone/coordination', config)
    if not rospy.has_param('/pidrone/altitude/controller'):
        import yaml
        params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
        with open(os.path.join(params_dir, 'altitude.yaml')) as stream:
            settings = yaml.safe_load(stream)
        drone = urlparse(os.environ.get('ROS_MASTER_URI', '')).hostname or ''
        drone = drone[:-6] if drone.endswith('.local') else drone
        per_drone = settings.pop('hover_throttle_by_drone', None) or {}
        settings['hover_throttle'] = per_drone.get(drone, settings['hover_throttle'])
        rospy.set_param('/pidrone/altitude/controller', settings)
    if not rospy.has_param('/pidrone/velocity/controller'):
        import yaml
        params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
        with open(os.path.join(params_dir, 'velocity.yaml')) as stream:
            rospy.set_param('/pidrone/velocity/controller', yaml.safe_load(stream))

    # create the PIDController object
    pid_controller = ControllerClass()

    # Publishers
    ############
    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1, tcp_nodelay=True)
    pid_controller.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
    pid_controller.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/pid_controller', Empty, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/state', State, pid_controller.current_state_callback,
                     queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/pidrone/desired/pose', Pose, pid_controller.desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/twist', Twist, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired/mode', Mode, pid_controller.desired_mode_callback)
    rospy.Subscriber('/pidrone/position_control', Bool, pid_controller.position_control_callback)
    rospy.Subscriber('/pidrone/reset_transform', Empty, pid_controller.reset_callback)
    rospy.Subscriber('/pidrone/picamera/lost', Bool, pid_controller.lost_callback)
    if pid_controller.use_fusion:
        from pidrone_pkg.msg import PeerState
        from std_msgs.msg import String
        pid_controller.coordination_status_pub = rospy.Publisher('/pidrone/coordination/status', String, queue_size=1)
        pid_controller.preflight_pub = rospy.Publisher('/pidrone/coordination/preflight', FlightReadiness, queue_size=1)
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

    # Non-ROS Setup
    ###############
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    # set the loop rate (Hz)
    loop_rate = rospy.Rate(60)
    print('PID Controller Started')
    while not rospy.is_shutdown():


        # Steps the PID. If we are not flying, this can be used to
        # examine the behavior of the PID based on published values
        stamp = pid_controller.current_state.header.stamp.to_sec()
        age = rospy.get_time() - stamp
        if not (np.isfinite(pid_controller.current_position.z) and stamp > 0
                and 0 <= age <= pid_controller.measurement_timeout):
            # Do not feed missing/stale measurements into PID or report a healthy
            # controller heartbeat. The FC's existing timeout remains responsible.
            rospy.logwarn_throttle(1, 'Waiting for valid, fresh altitude')
            loop_rate.sleep()
            continue
        pid_controller.heartbeat_pub.publish(Empty())
        fly_command = pid_controller.step()
        rospy.loginfo_throttle(1, 'Altitude target=%s estimated=%.3f m climb=%.3f m/s throttle=%.1f' % (
            pid_controller.altitude.target, pid_controller.current_position.z,
            pid_controller.current_velocity.z, fly_command[3]))

        if pid_controller.current_mode == 'FLYING':
            if pid_controller.desired_mode == 'FLYING':
                pid_controller.publish_cmd(fly_command)

        if args.verbose >= 2:
            if pid_controller.position_control:
                print('current position:', pid_controller.current_position)
                print('desired position:', pid_controller.desired_position)
                print('position error:', pid_controller.position_error)
            else:
                print('current velocity:', 
                    pid_controller.current_velocity.x, 
                    pid_controller.current_velocity.y, 
                    pid_controller.current_velocity.z)
                print('desired velocity:',
                    pid_controller.desired_velocity.x,
                    pid_controller.desired_velocity.y,
                    pid_controller.desired_velocity.z)
                print('velocity error:',
                    pid_controller.velocity_error.x,
                    pid_controller.velocity_error.y,
                    pid_controller.velocity_error.z)
                print('pid_error:',
                    pid_controller.pid_error.x,
                    pid_controller.pid_error.y,
                    pid_controller.pid_error.z)
        if args.verbose >= 1:
            print('r,p,y,t:', fly_command)

        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)
