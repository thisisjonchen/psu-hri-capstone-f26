#!/usr/bin/env python

import tf
import sys
import os
import rospy
import signal
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
from std_msgs.msg import Empty, Bool, UInt32, Float32
from localization.fusion import rotate, target_velocity, XY_PREDICTION_TIMEOUT
from localization.uwb_locator import monotonic
from pidrone_pkg.msg import AnchorLayout
from three_dim_vec import Position, Velocity, Error, RPY


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
        rospy.loginfo('PID sources: controller=%s commands=%s', __file__, cmds.__file__)

        self.position_control = False

        self.current_position = Position()
        self.desired_position = Position(z=self.takeoff_height)

        self.position_error = Error()

        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        self.velocity_error = Error()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.1
        self.velocity_deadline = self.yaw_deadline = None

        # Horizontal velocity -> lean angle (roll/pitch); settings from params/velocity.yaml
        self.velocity = VelocityController(**dict(rospy.get_param('/pidrone/velocity/controller', {})))

        self.pid_error = Error()

        self.lr_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))

        self.last_pose_time = None

        self.desired_yaw_velocity = 0

        self.current_rpy = RPY()

        self.current_state = State()

        self.cmdpub = None

        # Fall back to velocity control beyond this local-position error (m).
        self.safety_threshold = 1.5

        self.lost = False

        self.absolute_desired_position = False

        self.use_fusion = bool(rospy.get_param('/pidrone/localization/use_fusion', True))
        self.local_estimate_ready = self.room_estimate_ready = False
        self.room_pose = self.room_target = self.room_anchors = None
        self.room_generation = None
        self.room_target_gain = float(rospy.get_param('/pidrone/localization/target_gain', 0.30))
        self.room_speed_limit = float(rospy.get_param('/pidrone/localization/target_speed_limit', 0.10))
        if not all(np.isfinite(v) and v > 0 for v in (self.room_target_gain, self.room_speed_limit)):
            raise ValueError('Room target gain and speed must be positive')
        self.preflight_pub = None
        self.preflight_published = self.preflight_published_at = None

    def flight_readiness(self):
        report = FlightReadiness()
        report.mode = self.current_mode
        if self.current_mode not in ('DISARMED', 'ARMED'):
            report.reason = 'already_flying'
            return report
        if not self.local_ready():
            report.reason = 'local_estimate_unavailable'
            return report
        report.header = self.current_state.header
        report.ready, report.reason = True, 'ready'
        return report

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
        desired_z = (msg.position.z if self.absolute_desired_position else
                     self.desired_position.z + msg.position.z)
        if not np.isfinite(desired_z) or desired_z < 0:
            rospy.logwarn('Ignoring invalid altitude target: %s', desired_z)
            return
        # A target during the takeoff ramp moves where the ramp ends (the ramp
        # stops by itself at a lower target); landing still cancels the ramp.
        self.altitude.cancel_landing()
        if self.absolute_desired_position:
            self.desired_position.x = msg.position.x
            self.desired_position.y = msg.position.y
        elif msg.position.x or msg.position.y or not self.position_control:
            # A relative altitude-only command must not move an active X/Y hold
            # point to the drone's latest (possibly drifting) position.
            offset = rotate((msg.position.x, msg.position.y), self.current_rpy.y) if self.use_fusion else (msg.position.x, msg.position.y)
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
        if msg.mode == 'DISARMED' and previous != 'DISARMED':
            self.requested_takeoff_height = None
        self.current_mode = msg.mode
        if previous != msg.mode:
            self.cancel_room_target()
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
        """ Reset the desired and current poses of the drone and set
        desired velocities to zero """
        self.cancel_room_target()
        self.altitude.cancel_landing()
        if not self.use_fusion:
            self.current_position = Position(z=self.current_position.z)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        self.desired_velocity.x = 0
        self.desired_velocity.y = 0

    @synchronized
    def lost_callback(self, msg):
        if not self.use_fusion:
            self.lost = msg.data

    @synchronized
    def step(self):
        """ Returns the commands generated by the pid """
        if self.room_target is None:
            self.expire_velocity_command()
        xy_ready = self.calc_error()

        # Learn the constant push only from a fresh IMU/flow velocity estimate (no UWB needed).
        now = rospy.get_time()
        command = self.velocity.step((self.pid_error.x / 100., self.pid_error.y / 100.), now, xy_ready)
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
        self.velocity_error = self.desired_velocity - self.current_velocity
        self.pid_error.x = self.velocity_error.x
        self.pid_error.y = self.velocity_error.y
        self.pid_error.z = 0
        # multiply by 100 to account for the fact that code was originally written using cm
        self.pid_error = self.pid_error * 100
        if not xy_ready:
            self.pid_error.x = self.pid_error.y = 0
        if self.position_control and self.room_target is None and xy_ready:
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
        if self.preflight_pub is not None:
            report = self.flight_readiness()
            status = (report.mode, report.ready, report.reason)
            now = monotonic()
            if status != self.preflight_published or now - self.preflight_published_at >= 0.1:
                self.preflight_pub.publish(report)
                self.preflight_published, self.preflight_published_at = status, now
        return xy_ready

    def reset(self):
        """Reset targets, command deadlines and PID state for a flight mode change."""
        self.position_error = Error(0,0,0)
        height = self.takeoff_height if self.requested_takeoff_height is None else self.requested_takeoff_height
        self.desired_position = Position(self.current_position.x, self.current_position.y, height)
        rospy.loginfo("PID reset target=%s m source=%s", self.desired_position.z, __file__)
        self.velocity_error = Error(0,0,0)
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

    if not rospy.has_param('/pidrone/altitude/controller'):
        import yaml
        params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
        with open(os.path.join(params_dir, 'altitude.yaml')) as stream:
            settings = yaml.safe_load(stream)
        rospy.set_param('/pidrone/altitude/controller', settings)
    if not rospy.has_param('/pidrone/velocity/controller'):
        import yaml
        params_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params')
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
    rospy.Subscriber('/pidrone/picamera/lost', Bool, pid_controller.lost_callback)
    if pid_controller.use_fusion:
        pid_controller.preflight_pub = rospy.Publisher('/pidrone/coordination/preflight', FlightReadiness, queue_size=1)
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


        stamp = pid_controller.current_state.header.stamp.to_sec()
        age = rospy.get_time() - stamp
        if not (np.isfinite(pid_controller.current_position.z) and stamp > 0
                and 0 <= age <= pid_controller.measurement_timeout):
            # Do not feed missing/stale measurements into PID or report a healthy
            # controller heartbeat. The FC's existing timeout remains responsible.
            rospy.logwarn_throttle(1, 'Waiting for valid, fresh altitude')
            loop_rate.sleep()
            continue
        # Keep a mode change from publishing a command computed before its reset.
        with pid_controller.lock:
            pid_controller.heartbeat_pub.publish(Empty())
            fly_command = pid_controller.step()
            if pid_controller.current_mode == 'FLYING' and pid_controller.desired_mode == 'FLYING':
                pid_controller.publish_cmd(fly_command)

        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)
