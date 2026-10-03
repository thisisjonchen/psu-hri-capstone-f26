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
import numpy as np
import command_values as cmds
from pid_class import PID, PIDaxis
from geometry_msgs.msg import Pose, Twist
from pidrone_pkg.msg import Mode, RC, State
from std_msgs.msg import Float32, Empty, Bool
from three_dim_vec import Position, Velocity, Error, RPY


class PIDController(object):
    ''' Controls the flight of the drone by running a PID controller on the
    error calculated by the desired and current velocity and position of the drone
    '''

    def __init__(self):
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


    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_state_callback(self, state):
        """ Store the drone's current state for calculations """
        self.previous_state = self.current_state
        self.current_state = state
        self.state_to_three_dim_vec_structs()
        self.altitude_sample = (state.pose_with_covariance.pose.position.z * 100,
                                state.header.stamp.to_sec())

    def desired_pose_callback(self, msg):
        """ Update the desired pose """
        self.takeoff_ramp = None
        # store the previous desired position
        self.last_desired_position = copy.copy(self.desired_position)
        # set the desired positions equal to the desired pose message
        if self.absolute_desired_position:
            self.desired_position.x = msg.position.x
            self.desired_position.y = msg.position.y
            # Preserve the existing commanded-height range.
            desired_z = msg.position.z
            self.desired_position.z = desired_z if 0 <= desired_z <= 0.5 else self.last_desired_position.z
        # set the desired positions relative to the current position (except for z to make it more responsive)
        else:
            self.desired_position.x = self.current_position.x + msg.position.x
            self.desired_position.y = self.current_position.y + msg.position.y
            # set the disired z position relative to the last desired position (doesn't limit the mag of the error)
            # Preserve the existing commanded-height range.
            desired_z = self.last_desired_position.z + msg.position.z
            self.desired_position.z = desired_z if 0 <= desired_z <= 0.5 else self.last_desired_position.z

        if self.desired_position != self.last_desired_position:
            # the drone is moving between desired positions
            self.moving = True
            print('moving')

    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_velocity.z = msg.linear.z
        self.desired_yaw_velocity = msg.angular.z
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None
        if self.path_planning:
            self.calculate_travel_time()

    def current_mode_callback(self, msg):
        """ Update the current mode """
        previous = self.current_mode
        self.current_mode = msg.mode
        if previous != msg.mode:
            self.takeoff_ramp = None
            rospy.loginfo('PID mode %s -> %s', previous, msg.mode)
            if msg.mode in ('ARMED', 'FLYING'):
                self.reset()
                if msg.mode == 'FLYING':
                    self.takeoff_ramp = (rospy.get_time(), self.current_position.z)
                if msg.mode == 'ARMED' and hasattr(self, 'position_control_pub'):
                    self.position_control_pub.publish(False)

    def desired_mode_callback(self, msg):
        """ Update the desired mode """
        self.desired_mode = msg.mode

    def position_control_callback(self, msg):
        """ Set whether or not position control is enabled """
        self.position_control = msg.data
        if self.position_control and not self.last_position_control:
            self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        if (self.position_control != self.last_position_control):
            print("Position Control", self.position_control)
            self.last_position_control = self.position_control

    def reset_callback(self, empty):
        """ Reset the desired and current poses of the drone and set
        desired velocities to zero """
        self.current_position = Position(z=self.current_position.z)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.desired_position.z)
        self.desired_velocity.x = 0
        self.desired_velocity.y = 0

    def lost_callback(self, msg):
        self.lost = msg.data

    # Step Method
    #############
    def step(self):
        """ Returns the commands generated by the pid """
        self.calc_error()
        if self.position_control:
            if self.position_error.planar_magnitude() < self.safety_threshold and not self.lost:
                if self.moving:
                    if self.position_error.magnitude() > 0.05:
                        self.pid_error -= self.velocity_error * 100
                    else:
                        self.moving = False
                        print('not moving')
            else:
                self.position_control_pub.publish(False)

        if self.desired_velocity.magnitude() > 0 or abs(self.desired_yaw_velocity) > 0:
            self.adjust_desired_velocity()

        return self.pid.step(self.pid_error, self.desired_yaw_velocity,
                             self.altitude_sample)

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
        if self.position_control:
            # calculate the position error
            self.position_error = self.desired_position - self.current_position
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
