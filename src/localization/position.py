"""ToF height validation and horizontal multilateration from UWB ranges."""
import math
from collections import namedtuple

import numpy as np

from localization import finite
from localization.fusion import anchor_outline
from localization.rotations import quaternion_matrix

RangeSample = namedtuple('RangeSample', 'distance stamp received')
Attitude = namedtuple('Attitude', 'up stamp received')
Fix = namedtuple('Fix', 'x y')
Height = namedtuple('Height', 'z stamp')


def valid_range(distance, minimum, maximum):
    return (all(finite(v) for v in (distance, minimum, maximum))
            and 0 <= minimum < maximum and minimum <= distance <= maximum)


def solve_uwb_xy(anchors, tag_height, max_range_error):
    """Fit room XY to four slant ranges with the tag height held fixed."""
    xyz = np.array([(a.x, a.y, a.z) for a in anchors], dtype=float)
    if anchor_outline([a.id for a in anchors], xyz) is None:
        return None
    ranges = np.array([a.distance for a in anchors], dtype=float)
    if not np.all(np.isfinite(ranges)) or np.any(ranges <= 0):
        return None
    center = xyz[:, :2].mean(axis=0)
    points = xyz[:, :2] - center
    dz = tag_height - xyz[:, 2]
    if np.any(np.abs(dz) - ranges > max_range_error):
        return None
    # Subtract squared range equations to seed the nonlinear fit without PANS.
    rhs = np.sum(points * points, axis=1) + dz * dz - ranges * ranges
    if not np.all(np.isfinite(rhs)):
        return None
    try:
        xy = np.linalg.lstsq(2 * points, rhs - rhs.mean(), rcond=-1)[0]
        for _ in range(20):
            delta = xy - points
            predicted = np.sqrt(np.sum(delta * delta, axis=1) + dz * dz)
            errors = predicted - ranges
            jacobian = delta / np.maximum(predicted, 1e-6)[:, None]
            step, _, rank, _ = np.linalg.lstsq(jacobian, -errors, rcond=-1)
            if rank < 2 or not np.all(np.isfinite(step)):
                return None
            if np.linalg.norm(step) < 1e-6:
                if np.max(np.abs(errors)) <= max_range_error:
                    return tuple(float(v) for v in xy + center)
                return None
            xy += step
    except np.linalg.LinAlgError:
        pass
    return None


class PositionCombiner(object):
    def __init__(self, stale_after=0.5, max_skew=0.15, compensate_tilt=True,
                 floor_z=0.0, tof_offset_z=0.0, max_tilt_degrees=45.0,
                 uwb_offset_body=(0., 0., 0.), max_range_error=0.10,
                 range_offset_body=(0., 0., 0.), range_beam_body=(0., 0., -1.)):
        if not all(finite(v) for v in (stale_after, max_skew, floor_z,
                                      tof_offset_z, max_tilt_degrees, max_range_error)):
            raise ValueError('Position parameters must be finite')
        if stale_after <= 0 or max_skew <= 0 or not 0 < max_tilt_degrees < 90 or max_range_error <= 0:
            raise ValueError('Invalid timeout, skew, tilt or range error limit')
        self.uwb_offset = np.asarray(uwb_offset_body, dtype=float)
        self.range_offset = np.array(range_offset_body, dtype=float)
        self.range_beam = np.asarray(range_beam_body, dtype=float)
        if any(v.shape != (3,) or not np.all(np.isfinite(v))
               for v in (self.uwb_offset, self.range_offset, self.range_beam)):
            raise ValueError('Sensor geometry must have three finite components per vector')
        if abs(np.linalg.norm(self.range_beam) - 1.) > 1e-8:
            raise ValueError('ToF beam must be a unit vector')
        self.range_offset[2] += tof_offset_z
        self.max_range_error = max_range_error
        self.stale_after, self.max_skew = stale_after, max_skew
        self.compensate_tilt = compensate_tilt
        self.floor_z = floor_z
        self.min_vertical = math.cos(math.radians(max_tilt_degrees))
        self.range = None
        self.attitude = None

    def update_range(self, distance, minimum, maximum, stamp, received):
        # An explicit invalid reading invalidates the previous height immediately.
        self.range = None
        if valid_range(distance, minimum, maximum) and finite(stamp) and finite(received) and stamp > 0:
            self.range = RangeSample(distance, stamp, received)

    def update_attitude(self, quaternion, stamp, received, available=True):
        self.attitude = None
        if not available or not all(finite(v) for v in tuple(quaternion) + (stamp, received)):
            return
        if stamp <= 0:
            return
        try:
            up = quaternion_matrix(quaternion)[2]
        except ValueError:
            return
        if up[2] >= self.min_vertical:
            self.attitude = Attitude(up, stamp, received)

    def _fresh(self, sample, now, ros_now):
        return (sample is not None and 0 <= now - sample.received <= self.stale_after
                and 0 <= ros_now - sample.stamp <= self.stale_after)

    def height(self, now, ros_now):
        """Read tilt-compensated ToF height without inventing a UWB sample."""
        if not self._fresh(self.range, now, ros_now):
            return None, 'ToF missing, invalid or stale'
        stamps = [self.range.stamp]
        up = (0., 0., 1.)
        if self.compensate_tilt:
            if not self._fresh(self.attitude, now, ros_now):
                return None, 'IMU missing, invalid, stale or excessive tilt'
            stamps.append(self.attitude.stamp)
            up = self.attitude.up
        if max(stamps) - min(stamps) > self.max_skew:
            return None, 'Sensor timestamps are not aligned'
        beam_z = np.dot(up, self.range_beam)
        if beam_z >= -.1:
            return None, 'ToF beam points too far from the floor'
        z = self.floor_z - (np.dot(up, self.range_offset) + self.range.distance * beam_z)
        if not finite(z):
            return None, 'Invalid ToF height'
        return Height(z, min(stamps)), 'OK'

    def combine(self, sample, now, ros_now):
        if sample is None or not 0 <= now - sample.timestamp <= self.stale_after:
            return None, 'UWB ranges missing or stale'
        if len(sample.anchors) != 4:
            return None, 'Waiting for four UWB ranges in one report'
        height, reason = self.height(now, ros_now)
        if height is None:
            return None, reason
        # The tag reports ranges without acquisition stamps. Use serial receipt.
        uwb_stamp = ros_now - (now - sample.timestamp)
        stamps = [uwb_stamp, self.range.stamp]
        up = (0., 0., 1.)
        if self.compensate_tilt:
            stamps.append(self.attitude.stamp)
            up = self.attitude.up
        if min(stamps) <= 0 or max(stamps) - min(stamps) > self.max_skew:
            return None, 'Sensor timestamps are not aligned'
        tag_height = height.z + np.dot(up, self.uwb_offset)
        xy = solve_uwb_xy(sample.anchors, tag_height, self.max_range_error)
        if xy is None:
            return None, 'UWB ranges inconsistent with height or anchor geometry'
        return Fix(xy[0], xy[1]), 'OK'
