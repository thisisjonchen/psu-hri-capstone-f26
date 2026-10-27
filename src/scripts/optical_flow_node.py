#!/usr/bin/env python
"""Publish body velocity from the tilted camera's encoder motion vectors.

Geometry, rotation removal and floor selection live in localization.camera_flow.
The camera mounting comes from apriltag_mapping.yaml, flow settings from
optical_flow.yaml, the lens from camera_info, height from the ToF and attitude
from the flight-controller IMU.
"""
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from pidrone_pkg.msg import FlowQuality
from raspicam_node.msg import MotionVectors
from sensor_msgs.msg import CameraInfo, Imu, Range

from localization.apriltag_geometry import camera_rotation
from localization.camera_flow import CameraFlow, interpolate


class OpticalFlowNode(object):
    def __init__(self):
        rospy.init_node('optical_flow_node')
        self.measurement_timeout = float(rospy.get_param('pidrone/altitude/measurement_timeout', 0.5))
        self.body_frame = rospy.get_param('~body_frame', 'pidrone_body')
        self.rotation = camera_rotation(
            float(rospy.get_param('~camera_pitch_down_deg', 45.0)),
            float(rospy.get_param('~camera_azimuth_deg', 0.0)),
            float(rospy.get_param('~camera_image_roll_deg', 0.0)))
        self.settings = dict(
            frame_interval=1.0 / float(rospy.get_param('~framerate', 30.0)),
            max_floor_distance=float(rospy.get_param('~max_floor_distance', 1.5)),
            min_blocks=int(rospy.get_param('~min_blocks', 8)),
            gain=float(rospy.get_param('~flow_gain', 1.0)),
            flip_vertically=bool(rospy.get_param('~flip_vectors_vertically', True)))
        self.flow, self.calibration = None, None
        self.range = None
        self.attitude = deque(maxlen=100)
        self.twistpub = rospy.Publisher('pidrone/picamera/twist', TwistStamped, queue_size=1)
        self.qualitypub = rospy.Publisher('pidrone/picamera/flow_quality', FlowQuality, queue_size=10)
        rospy.Subscriber('raspicam_node/camera_info', CameraInfo, self.camera_cb, queue_size=1)
        rospy.Subscriber('pidrone/imu', Imu, self.imu_cb, queue_size=10)
        rospy.Subscriber('pidrone/range', Range, self.range_cb, queue_size=1)
        rospy.Subscriber('raspicam_node/motion_vectors', MotionVectors, self.motion_cb, queue_size=1)

    def camera_cb(self, msg):
        calibration = (msg.width, msg.height, tuple(msg.K), tuple(msg.D), msg.distortion_model)
        if calibration != self.calibration:
            try:
                self.flow = CameraFlow(msg.K, msg.D, msg.distortion_model, (msg.width, msg.height),
                                       self.rotation, **self.settings)
            except ValueError as error:
                self.flow = None
                rospy.logwarn('Optical flow disabled: %s', error)
            self.calibration = calibration

    def imu_cb(self, msg):
        q = msg.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        stamp = msg.header.stamp.to_sec()
        if (msg.orientation_covariance[0] != -1 and np.all(np.isfinite(quaternion))
                and np.linalg.norm(quaternion) > 1e-6
                and (not self.attitude or stamp > self.attitude[-1][0])):
            self.attitude.append((stamp, quaternion / np.linalg.norm(quaternion)))

    def range_cb(self, msg):
        valid = (np.all(np.isfinite([msg.range, msg.min_range, msg.max_range]))
                 and 0 <= msg.min_range < msg.max_range and msg.min_range <= msg.range <= msg.max_range
                 and msg.header.stamp.to_sec() > 0)
        self.range = (msg.range, msg.header.stamp.to_sec()) if valid else None

    def motion_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        now = rospy.get_time()
        if (self.flow is None or self.range is None
                or not 0 <= now - self.range[1] <= self.measurement_timeout):
            return
        attitude = interpolate(self.attitude, stamp)
        previous = interpolate(self.attitude, stamp - self.flow.frame_interval)
        if attitude is None or previous is None:
            return
        x, y = attitude[0], attitude[1]
        height = self.range[0] * (1 - 2 * (x * x + y * y))  # ToF slant range -> vertical height
        velocity, blocks = self.flow.velocity(msg.x, msg.y, msg.mbx, attitude, previous, height)
        quality = FlowQuality()
        quality.header.stamp = msg.header.stamp
        quality.valid = velocity is not None
        quality.blocks = blocks if velocity is not None else 0
        self.qualitypub.publish(quality)
        if velocity is None:
            return
        twist = TwistStamped()
        twist.header.stamp = msg.header.stamp
        twist.header.frame_id = self.body_frame
        twist.twist.linear.x, twist.twist.linear.y = velocity  # right, forward (m/s)
        self.twistpub.publish(twist)


def main():
    OpticalFlowNode()
    rospy.spin()


if __name__ == '__main__':
    main()
