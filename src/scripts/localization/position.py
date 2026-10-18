"""Validation and UWB/ToF position assembly without a ROS dependency."""
import math
from collections import namedtuple

from .uwb_locator import finite

RangeSample = namedtuple('RangeSample', 'distance stamp received')
Attitude = namedtuple('Attitude', 'vertical stamp received')
Fix = namedtuple('Fix', 'x y z quality stamp')
Height = namedtuple('Height', 'z stamp')


class PositionCombiner(object):
    def __init__(self, stale_after=0.5, max_skew=0.15, compensate_tilt=True,
                 floor_z=0.0, tof_offset_z=0.0, max_tilt_degrees=45.0):
        if not all(finite(v) for v in (stale_after, max_skew, floor_z,
                                      tof_offset_z, max_tilt_degrees)):
            raise ValueError('Position parameters must be finite')
        if stale_after <= 0 or max_skew <= 0 or not 0 < max_tilt_degrees < 90:
            raise ValueError('Invalid timeout, skew or tilt limit')
        self.stale_after, self.max_skew = stale_after, max_skew
        self.compensate_tilt = compensate_tilt
        self.floor_z, self.tof_offset_z = floor_z, tof_offset_z
        self.min_vertical = math.cos(math.radians(max_tilt_degrees))
        self.range = None
        self.attitude = None

    def update_range(self, distance, minimum, maximum, stamp, received):
        # An explicit invalid reading invalidates the previous height immediately.
        self.range = None
        if (all(finite(v) for v in (distance, minimum, maximum, stamp, received))
                and 0 <= minimum < maximum and minimum <= distance <= maximum
                and stamp > 0):
            self.range = RangeSample(distance, stamp, received)

    def update_attitude(self, quaternion, stamp, received, available=True):
        self.attitude = None
        if not available or not all(finite(v) for v in tuple(quaternion) + (stamp, received)):
            return
        norm = sum(v * v for v in quaternion)
        if norm < 1e-12 or stamp <= 0:
            return
        # R[2,2] = cos(roll)*cos(pitch). Quaternion order is x,y,z,w.
        x, y, _, _ = quaternion
        vertical = 1.0 - 2.0 * (x * x + y * y) / norm
        if vertical >= self.min_vertical:
            self.attitude = Attitude(vertical, stamp, received)

    def _fresh(self, sample, now, ros_now):
        return (sample is not None and 0 <= now - sample.received <= self.stale_after
                and 0 <= ros_now - sample.stamp <= self.stale_after)

    def height(self, now, ros_now):
        """Read tilt-compensated ToF height without inventing a UWB sample."""
        if not self._fresh(self.range, now, ros_now):
            return None, 'ToF missing, invalid or stale'
        stamps = [self.range.stamp]
        vertical = 1.0
        if self.compensate_tilt:
            if not self._fresh(self.attitude, now, ros_now):
                return None, 'IMU missing, invalid, stale or excessive tilt'
            stamps.append(self.attitude.stamp)
            vertical = self.attitude.vertical
        if max(stamps) - min(stamps) > self.max_skew:
            return None, 'Sensor timestamps are not aligned'
        z = self.floor_z + (self.range.distance - self.tof_offset_z) * vertical
        if not finite(z):
            return None, 'Invalid ToF height'
        return Height(z, min(stamps)), 'OK'

    def combine(self, position, now, ros_now):
        if position is None or not 0 <= now - position.timestamp <= self.stale_after:
            return None, 'UWB missing or stale'
        if not all(finite(v) for v in (position.x, position.y, position.timestamp, ros_now)):
            return None, 'Invalid UWB position'
        height, reason = self.height(now, ros_now)
        if height is None:
            return None, reason
        # PANS has no acquisition stamp; estimate receipt time in ROS coordinates.
        uwb_stamp = ros_now - (now - position.timestamp)
        stamps = [uwb_stamp, self.range.stamp]
        if self.compensate_tilt:
            stamps.append(self.attitude.stamp)
        if min(stamps) <= 0 or max(stamps) - min(stamps) > self.max_skew:
            return None, 'Sensor timestamps are not aligned'
        return Fix(position.x, position.y, height.z, position.quality, min(stamps)), 'OK'
