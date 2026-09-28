"""Shared ROS1 altitude contract; flight limits are separate from sensor limits."""
import math
import rospy


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


class AltitudeConfig(object):
    def __init__(self):
        prefix = '/pidrone/altitude/'
        self.takeoff = float(rospy.get_param(prefix + 'takeoff_height', 0.25))
        self.limit = float(rospy.get_param(prefix + 'emergency_height', 0.50))
        self.timeout = float(rospy.get_param(prefix + 'measurement_timeout', 0.5))
        if not all(finite(v) and v > 0 for v in (self.takeoff, self.limit, self.timeout)) or self.takeoff >= self.limit:
            raise ValueError('Require 0 < takeoff_height < emergency_height and positive timeout')
        rospy.loginfo('Altitude config: takeoff=%s emergency=%s timeout=%s source=%s',
                      self.takeoff, self.limit, self.timeout, __file__)


def fresh(stamp, now, timeout):
    age = now - stamp.to_sec()
    return stamp.to_sec() > 0 and 0 <= age <= timeout


class RangeMonitor(object):
    def __init__(self, timeout):
        self.timeout = timeout
        self.message = None

    def update(self, message):
        self.message = message

    def fault(self, now):
        msg = self.message
        if msg is None:
            return 'range missing'
        if not all(finite(v) for v in (msg.range, msg.min_range, msg.max_range)):
            return 'range nonfinite'
        if not 0 <= msg.min_range < msg.max_range or not msg.min_range <= msg.range <= msg.max_range:
            return 'range unsupported'
        if not fresh(msg.header.stamp, now, self.timeout):
            return 'range stale or invalid timestamp'
        return None
