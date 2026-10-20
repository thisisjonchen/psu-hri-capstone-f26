#!/usr/bin/env python
"""Hold the drone's offset from a selected floor AprilTag.

While that tag is out of view, keep the fused X/Y position captured at
acquisition and resume the tag offset when it returns.
Camera mounting comes from the AprilTag mapper's parameters.
"""

import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from pidrone_pkg.msg import TagDetection
from sensor_msgs.msg import CameraInfo

from localization.apriltag_geometry import camera_rotation, project_floor_tag
from localization.fusion import target_velocity


class TagHold(object):
    def __init__(self, drone):
        self.drone = drone
        self.rotation = camera_rotation(
            float(rospy.get_param('/apriltag_mapping/camera_pitch_down_deg')),
            float(rospy.get_param('/apriltag_mapping/camera_azimuth_deg')),
            float(rospy.get_param('/apriltag_mapping/camera_image_roll_deg')))
        self.camera_offset = np.asarray(
            rospy.get_param('/apriltag_mapping/camera_offset_body'), dtype=float)
        self.timeout = float(rospy.get_param('/apriltag_detection/camera_timeout', 0.5))
        if (self.camera_offset.shape != (3,) or not np.all(np.isfinite(self.camera_offset))
                or not self.timeout > 0):
            raise ValueError('Invalid tag-hold camera offset or timeout')
        self.camera_info = None
        self.detection = None
        self.used_stamp = None
        self.tag_id = None
        self.target = None  # desired level-frame (right, forward) tag offset
        self.offset = None
        self.offset_stamp = None
        self.lost = False
        rospy.Subscriber('/pidrone/apriltag/detections', TagDetection,
                         self._detection_received, queue_size=1)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo,
                         self._camera_received, queue_size=1)

    def _detection_received(self, msg):
        self.detection = msg

    def _camera_received(self, msg):
        self.camera_info = msg

    def wait_ready(self, timeout=10.0):
        """Before arming, require intrinsics that match the detector's frames."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            info, msg = self.camera_info, self.detection
            if (info is not None and len(info.K) == 9 and info.K[0] > 0 and info.K[4] > 0 and
                    msg is not None and msg.camera_online and
                    (msg.image_width, msg.image_height) == (info.width, info.height)):
                return
            rospy.sleep(0.1)
        raise RuntimeError('Tag hold needs AprilTag detections and matching camera_info')

    def _new_detection(self):
        msg = self.detection
        if msg is None or not msg.camera_online:
            return None
        stamp = msg.header.stamp.to_sec()
        if (self.used_stamp is not None and stamp <= self.used_stamp or
                not 0 <= rospy.get_time() - stamp <= self.timeout or
                len(msg.tag_ids) != len(msg.center_x) or len(msg.tag_ids) != len(msg.center_y)):
            return None
        self.used_stamp = stamp
        return msg

    def _tag_offset(self, msg, index):
        """Tag center in the drone's level heading frame (right, forward), meters."""
        info = self.camera_info
        if info is None or (msg.image_width, msg.image_height) != (info.width, info.height):
            rospy.logwarn_throttle(5, 'Tag hold: camera_info does not match detection frames')
            return None
        pose = self.drone._hold_pose(required=False, allow_prediction=True)
        if pose is None:
            return None
        q = pose.pose.pose.orientation
        roll, pitch, _ = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        level = tf.transformations.quaternion_from_euler(roll, pitch, 0.0)
        try:
            point = project_floor_tag(
                (msg.center_x[index], msg.center_y[index]), info.K, info.D,
                (0.0, 0.0, self.drone.height), level, self.camera_offset, self.rotation,
                distortion_model=info.distortion_model)
        except ValueError as error:
            rospy.logwarn_throttle(5, 'Tag hold geometry rejected: {}'.format(error))
            return None
        return point[:2]

    def acquire(self, timeout=3.0, tag_id=None):
        """Lock the requested tag, or the first visible tag when no ID is given."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.drone.hover(0.1)
            msg = self._new_detection()
            if msg is None or not msg.tag_ids:
                continue
            if tag_id is not None and tag_id not in msg.tag_ids:
                continue
            index = 0 if tag_id is None else list(msg.tag_ids).index(tag_id)
            offset = self._tag_offset(msg, index)
            if offset is None:
                continue
            self.tag_id = msg.tag_ids[index]
            self.drone.lock_xy()
            self.target = self.offset = offset
            self.offset_stamp = self.used_stamp
            rospy.loginfo('Stabilization: Kalman -> AprilTag (locked tag %d at offset (%.3f, %.3f) m)',
                          self.tag_id, offset[0], offset[1])
            return
        name = 'AprilTag' if tag_id is None else 'AprilTag {}'.format(tag_id)
        raise RuntimeError('No {} seen within {:.1f} s'.format(name, timeout))

    def command(self):
        msg = self._new_detection()
        if msg is not None and self.tag_id in msg.tag_ids:
            offset = self._tag_offset(msg, list(msg.tag_ids).index(self.tag_id))
            if offset is not None:
                self.offset, self.offset_stamp = offset, self.used_stamp
        if (self.offset_stamp is not None and
                0 <= rospy.get_time() - self.offset_stamp <= self.timeout):
            # Log only switches between tag and fallback hold, not every cycle.
            if self.lost:
                rospy.loginfo('Stabilization: Kalman fallback -> AprilTag (tag %d reacquired)', self.tag_id)
                self.lost = False
            command = Twist()
            # Moving by (offset - target) restores the tag to its locked offset.
            command.linear.x, command.linear.y = target_velocity(
                self.offset, self.target, 0.0,
                self.drone.xy_hold_gain, self.drone.xy_speed_limit)
            return command
        if not self.lost:
            rospy.logwarn('Stabilization: AprilTag -> Kalman fallback (tag %d lost)', self.tag_id)
            self.lost = True
        return self.drone._xy_hold_command()

    def finish(self):
        """Hold the current fused position (e.g. for landing)."""
        self.drone.lock_xy()
