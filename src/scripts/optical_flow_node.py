#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty



class OpticalFlowNode(object):
    """
    Subscribe to the optical flow vectors and publish linear velocity as a Twist message.
    """
    def __init__(self, node_name):

        rospy.init_node(node_name)
        # flow variables
        camera_wh = (320, 240)        
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = float(rospy.get_param('/pidrone/localization/flow_scale', .165))
        if not np.isfinite(self.flow_scale) or self.flow_scale <= 0:
            raise ValueError('Optical-flow scale must be positive')
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)
        self.altitude = None
        self.altitude_ts = rospy.Time(0)
        self.measurement_timeout = float(rospy.get_param('/pidrone/altitude/measurement_timeout', 0.5))
        self.setup()

    def setup(self):
        # publisher
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)

        # subscribers
        self._sub_mv = rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors, self.motion_cb, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range, self.altitude_cb, queue_size=1)

    def motion_cb(self, msg):
        ''' Average the motion vectors and publish the
        twist message. 
        '''
        # signed 1-byte values
        x = msg.x
        y = msg.y
        if (self.altitude is None or not 0 <= (rospy.Time.now() - self.altitude_ts).to_sec() <= self.measurement_timeout
                or not len(x) or len(x) != len(y)):
            return

        # calculate the planar and yaw motions
        x_motion = np.sum(x) * self.flow_coeff * self.altitude
        y_motion = np.sum(y) * self.flow_coeff * self.altitude
        twist_msg = TwistStamped()
        # Some legacy raspicam MotionVectors messages have no Header.
        header = getattr(msg, 'header', None)
        twist_msg.header.stamp = header.stamp if header is not None else rospy.Time.now()
        twist_msg.header.frame_id = 'pidrone_body'
        twist_msg.twist.linear.x = x_motion
        # DD24 camera mounting: forward translation must publish positive y.
        # The velocity PID then lowers Betaflight pitch to oppose that motion.
        twist_msg.twist.linear.y = y_motion
        #print(self.altitude)
        #print(x_motion,y_motion)
        # Update and publish the twist message
        self.twistpub.publish(twist_msg)
        duration_from_last_altitude = rospy.Time.now() - self.altitude_ts
        if duration_from_last_altitude.to_sec() > 10:
            rospy.logwarn("No altitude received for {:10.4f} seconds.".format(duration_from_last_altitude.to_sec()))

    def altitude_cb(self, msg):
        """
        The altitude of the robot
        Args:
            msg:  the message publishing the altitude

        """
        self.altitude = msg.range if (np.all(np.isfinite([msg.range, msg.min_range, msg.max_range]))
            and 0 <= msg.min_range < msg.max_range and msg.min_range <= msg.range <= msg.max_range
            and msg.header.stamp.to_sec() > 0) else None
        self.altitude_ts = msg.header.stamp
    
def main():
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()

if __name__ == '__main__':
    main()
