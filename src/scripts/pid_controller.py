#!/usr/bin/env python

import copy
import pid_class as pid_class
import tf
import sys
import os
import rospy
import signal
import traceback
import argparse
import functools
import threading
import numpy as np
import command_values as cmds
from pid_class import PID, PIDaxis
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from pidrone_pkg.msg import Mode, RC, State
from std_msgs.msg import Float32, Empty, Bool, UInt32
from localization.fusion import rotate, target_velocity
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
        # Initialize the current and desired modes
        self.current_mode = 'DISARMED'
        self.desired_mode = 'DISARMED'
        self.takeoff_height = float(rospy.get_param('/pidrone/altitude/takeoff_height', 0.25))
        self.takeoff_ramp = None
        self.altitude_target = self.takeoff_height
        self.measurement_timeout = float(rospy.get_param('/pidrone/altitude/measurement_timeout', 0.5))
        rospy.loginfo('PID sources: controller=%s pid=%s commands=%s', __file__, pid_class.__file__, cmds.__file__)

        # Initialize in velocity control
        self.position_control = False
        self.last_position_control = False

        # Initialize the current and desired positions
        self.current_position = Position()
        self.altitude_sample = (0.0, 0.0)
        self.desired_position = Position(z=self.takeoff_height)
        self.last_desired_position = Position(z=self.takeoff_height)

        # Initialize the position error
        self.position_error = Error()

        # Initialize the current and desired velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Initialize the velocity error
        self.velocity_error = Error()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.1
        # Set a static duration that a velocity command will be held
        self.desired_velocity_travel_time = 0.1

        # Set a static duration that a yaw velocity command will be held
        self.desired_yaw_velocity_travel_time = 0.25

        # Store the start time of the desired velocities
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None

        # Initialize the primary PID
        self.pid = PID()
        self.hover_throttle_midpoint = self.pid.throttle.midpoint
        self.takeoff_throttle_midpoint = 1490

        # Initialize the error used for the PID which is vx, vy, z where vx and
        # vy are velocities, and z is the error in the altitude of the drone
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

        # Initialize the current and  previous roll, pitch, yaw values
        self.current_rpy = RPY()
        self.previous_rpy = RPY()

        # initialize the current and previous states
        self.current_state = State()
        self.previous_state = State()

        # Store the command publisher
        self.cmdpub = None

        # a variable used to determine if the drone is moving between disired
        # positions
        self.moving = False

        # a variable that determines the maximum magnitude of the position error
        # Any greater position error will overide the drone into velocity
        # control
        self.safety_threshold = 1.5

        # determines if the position of the drone is known
        self.lost = False

        # determines if the desired poses are aboslute or relative to the drone
        self.absolute_desired_position = False

        # determines whether to use open loop velocity path planning which is
        # accomplished by calculate_travel_time
        self.path_planning = True
        self.use_fusion = bool(rospy.get_param('/pidrone/localization/use_fusion', True))
        self.local_ready_at = self.room_ready_at = None
        self.room_pose = self.room_target = self.room_anchors = None
        self.room_generation = None
        self.room_target_gain = float(rospy.get_param('/pidrone/localization/target_gain', 0.5))
        self.room_speed_limit = float(rospy.get_param('/pidrone/localization/target_speed_limit', 0.2))
        if not all(np.isfinite(v) and v > 0 for v in (self.room_target_gain, self.room_speed_limit)):
            raise ValueError('Room target gain and speed must be positive')

    def local_ready(self):
        return (not self.use_fusion or self.local_ready_at is not None
                and 0 <= rospy.get_time() - self.local_ready_at <= 0.3)

    def room_ready(self):
        return (self.use_fusion and self.local_ready() and self.room_ready_at is not None
                and 0 <= rospy.get_time() - self.room_ready_at <= 0.3
                and self.room_pose is not None
                and 0 <= rospy.get_time() - self.room_pose.header.stamp.to_sec() <= 0.5)

    @synchronized
    def local_ready_callback(self, msg):
        self.local_ready_at = rospy.get_time() if msg.data else None

    @synchronized
    def room_ready_callback(self, msg):
        self.room_ready_at = rospy.get_time() if msg.data else None

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
        if (not self.room_ready() or self.current_mode != 'FLYING'
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
        self.previous_state = self.current_state
        self.current_state = state
        self.state_to_three_dim_vec_structs()
        self.altitude_sample = (state.pose_with_covariance.pose.position.z * 100,
                                state.header.stamp.to_sec())

    @synchronized
    def desired_pose_callback(self, msg):
        """ Update the desired pose """
        self.cancel_room_target()
        desired_z = (msg.position.z if self.absolute_desired_position else
                     self.desired_position.z + msg.position.z)
        if not np.isfinite(desired_z) or desired_z < 0:
            rospy.logwarn('Ignoring invalid altitude target: %s', desired_z)
            return
        self.takeoff_ramp = None
        # store the previous desired position
        self.last_desired_position = copy.copy(self.desired_position)
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

        if self.desired_position != self.last_desired_position:
            # the drone is moving between desired positions
            self.moving = True
            print('moving')

    @synchronized
    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.cancel_room_target()
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_velocity.z = msg.linear.z
        self.desired_yaw_velocity = msg.angular.z
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None
        if self.path_planning:
            self.calculate_travel_time()

    @synchronized
    def current_mode_callback(self, msg):
        """ Update the current mode """
        previous = self.current_mode
        self.current_mode = msg.mode
        if previous != msg.mode:
            self.cancel_room_target()
            self.takeoff_ramp = None
            rospy.loginfo('PID mode %s -> %s', previous, msg.mode)
            if msg.mode in ('ARMED', 'FLYING'):
                self.reset()
                if msg.mode == 'FLYING':
                    self.takeoff_ramp = (rospy.get_time(), self.current_position.z)
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
        self.position_control = msg.data
        if self.position_control and not self.last_position_control:
            self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        if (self.position_control != self.last_position_control):
            print("Position Control", self.position_control)
            self.last_position_control = self.position_control

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
        self.calc_error()
        if self.position_control and self.room_target is None:
            if self.position_error.planar_magnitude() < self.safety_threshold and not self.lost:
                if self.moving:
                    if self.position_error.magnitude() > 0.05:
                        self.pid_error -= self.velocity_error * 100
                    else:
                        self.moving = False
                        print('not moving')
            else:
                self.position_control_pub.publish(False)

        requested_xy = (self.desired_velocity.x, self.desired_velocity.y)
        if self.room_target is None and (self.desired_velocity.magnitude() > 0 or abs(self.desired_yaw_velocity) > 0):
            self.adjust_desired_velocity()

        command = self.pid.step(self.pid_error, self.desired_yaw_velocity,
                                self.altitude_sample)
        if self.current_mode == 'FLYING':
            rospy.loginfo_throttle(1,
                'XY PID: local_ready=%s position_control=%s desired=(%.3f, %.3f) '
                'measured=(%.3f, %.3f) m/s roll=%.1f pitch=%.1f' % (
                    self.local_ready(), self.position_control, requested_xy[0], requested_xy[1],
                    self.current_velocity.x, self.current_velocity.y, command[0], command[1]))
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
        self.previous_rpy = self.current_rpy
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        r,p,y = tf.transformations.euler_from_quaternion(quaternion)
        self.current_rpy = RPY(r,p,y)

    def adjust_desired_velocity(self):
        """ Set the desired velocity back to 0 once the drone has traveled the
        amount of time that causes it to move the specified desired velocity
        travel distance if path_planning otherwise just set the velocities back
        to 0 after the . This is an open loop method meaning that the specified
        travel distance cannot be guarenteed. If path planning_planning is false,
        just set the velocities back to zero, this allows the user to move the
        drone for as long as they are holding down a key
        """
        curr_time = rospy.get_time()
        # set the desired planar velocities to zero if the duration is up
        if self.desired_velocity_start_time is not None:
            # the amount of time the set point velocity is not zero
            duration = curr_time - self.desired_velocity_start_time
            if duration > self.desired_velocity_travel_time:
                self.desired_velocity.x = 0
                self.desired_velocity.y = 0
                self.desired_velocity_start_time = None
        else:
            self.desired_velocity_start_time = curr_time

        # set the desired yaw velocity to zero if the duration is up
        if self.desired_yaw_velocity_start_time != None:
            # the amount of time the set point velocity is not zero
            duration = curr_time - self.desired_yaw_velocity_start_time
            if duration > self.desired_yaw_velocity_travel_time:
                self.desired_yaw_velocity = 0
                self.desired_yaw_velocity_start_time = None
        else:
            self.desired_yaw_velocity_start_time = curr_time

    @synchronized
    def calc_error(self):
        """ Calculate the error in velocity, and if in position hold, add the
        error from lr_pid and fb_pid to the velocity error to control the
        position of the drone
        """
        # store the time difference
        pose_dt = 0
        if self.last_pose_time != None:
            pose_dt = rospy.get_time() - self.last_pose_time
        self.last_pose_time = rospy.get_time()
        if self.room_target is not None and not self.room_ready():
            self.cancel_room_target()
        if not self.local_ready():
            self.cancel_room_target()
            self.desired_velocity.x = self.desired_velocity.y = 0
            if self.position_control:
                self.position_control = False
                self.position_control_pub.publish(False)
            self.pid.roll.reset()
            self.pid.pitch.reset()
            self.pid.roll_low.reset()
            self.pid.pitch_low.reset()
        if self.room_target is not None:
            p, q = self.room_pose.pose.pose.position, self.room_pose.pose.pose.orientation
            angle = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            command = target_velocity(self.room_target, (p.x, p.y), angle,
                                      self.room_target_gain, self.room_speed_limit)
            self.desired_velocity.x, self.desired_velocity.y = command
        # calculate the velocity error
        self.velocity_error = self.desired_velocity - self.current_velocity
        # calculate the z position error
        self.altitude_target = self.desired_position.z
        if self.takeoff_ramp is not None:
            started, initial_height = self.takeoff_ramp
            ramp_height = initial_height + 0.10 * max(0.0, rospy.get_time() - started)
            self.altitude_target = min(self.desired_position.z, ramp_height)
            if ramp_height >= self.desired_position.z:
                self.takeoff_ramp = None
        # Reduce initial thrust only while the altitude target is ramping.
        # A pose command (including landing) cancels the ramp above.
        midpoint = (self.takeoff_throttle_midpoint
                    if self.current_mode == 'FLYING' and self.takeoff_ramp is not None
                    else self.hover_throttle_midpoint)
        if self.pid.throttle.midpoint != midpoint:
            self.pid.throttle.midpoint = midpoint
            rospy.loginfo('Altitude throttle midpoint=%.1f; takeoff ramp=%s',
                          midpoint, self.takeoff_ramp is not None)
        dz = self.altitude_target - self.current_position.z
        # calculate the pid_error from the above values
        self.pid_error.x = self.velocity_error.x
        self.pid_error.y = self.velocity_error.y
        self.pid_error.z = dz
        # multiply by 100 to account for the fact that code was originally written using cm
        self.pid_error = self.pid_error * 100
        if not self.local_ready():
            self.pid_error.x = self.pid_error.y = 0
        if self.position_control and self.room_target is None:
            # calculate the position error
            self.position_error = self.desired_position - self.current_position
            if self.use_fusion:
                error = rotate((self.position_error.x, self.position_error.y), -self.current_rpy.y)
                self.position_error.x, self.position_error.y = error
            # calculate a value to add to the velocity error based based on the
            # position error in the x (roll) direction
            lr_step = self.lr_pid.step(self.position_error.x, pose_dt)
            # calculate a value to add to the velocity error based based on the
            # position error in the y (pitch) direction
            fb_step = self.fb_pid.step(self.position_error.y, pose_dt)
            self.pid_error.x += lr_step
            self.pid_error.y += fb_step

    def calculate_travel_time(self):
        ''' return the amount of time that desired velocity should be used to
        calculate the error in order to move the drone the specified travel
        distance for a desired velocity
        '''
        if self.desired_velocity.magnitude() > 0:
            # tiime = distance / velocity
            travel_time = self.desired_velocity_travel_distance / self.desired_velocity.planar_magnitude()
        else:
            travel_time = 0.0
        self.desired_velocity_travel_time = travel_time


    def reset(self):
        ''' Set desired_position to be current position, set
        filtered_desired_velocity to be zero, and reset both the PositionPID
        and VelocityPID
        '''
        # reset position control variables
        self.position_error = Error(0,0,0)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.takeoff_height)
        rospy.loginfo("PID reset target=%s m source=%s", self.desired_position.z, __file__)
        # reset velocity control_variables
        self.velocity_error = Error(0,0,0)
        self.desired_velocity = Velocity(0,0,0)
        # reset the pids
        self.pid.reset()
        self.pid.throttle.midpoint = self.hover_throttle_midpoint
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
        rospy.loginfo_throttle(1, "PID target=%s estimated=%s" % (pid_controller.altitude_target, pid_controller.current_position.z))

        if pid_controller.current_mode == 'FLYING':
            if pid_controller.desired_mode == 'FLYING':

                pid_controller.publish_cmd(fly_command)
            # after flying, take the converged low i terms and set these as the
            # initial values, this allows the drone to "learn" and get steadier
            # with each flight until it converges
            # NOTE: do not store the throttle_low.init_i or else the drone will
            # take off abruptly after the first flight
            elif pid_controller.desired_mode == 'DISARMED':
                pid_controller.pid.roll_low.init_i = pid_controller.pid.roll_low._i
                pid_controller.pid.pitch_low.init_i = pid_controller.pid.pitch_low._i
                # Uncomment below statements to print the converged values.
                # Make sure verbose = 0 so that you can see these values
                print('roll_low.init_i', pid_controller.pid.roll_low.init_i)
                print('pitch_low.init_i', pid_controller.pid.pitch_low.init_i)

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
