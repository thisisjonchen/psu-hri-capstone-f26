"""Planar Kalman fusion and frame conversion, independent of ROS.

The existing controller uses x=right, y=forward. The FC's IMU yaw describes
forward in a conventional x/y world, so the controller basis is yaw - pi/2.
Camera velocity drives continuous odometry; UWB corrects only the room offset.
"""
import math
from collections import deque

import numpy as np


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    x, y = vector
    return np.array([c * x - s * y, s * x + c * y])


def body_angle(imu_yaw):
    return imu_yaw - math.pi / 2.0


def target_velocity(target, position, angle, gain, limit):
    velocity = rotate(np.asarray(target) - position, -angle) * gain
    speed = np.linalg.norm(velocity)
    return velocity * min(1.0, limit / speed) if speed else velocity


class PlanarFusion(object):
    def __init__(self, yaw_offset=None, velocity_stddev=0.12,
                 drift_stddev=0.08, velocity_tau=0.08,
                 max_gap=0.25, gate=9.21, max_innovation=1.0):
        values = (velocity_stddev, drift_stddev, velocity_tau, max_gap, gate, max_innovation)
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('Fusion noise, timeouts and gates must be positive')
        if yaw_offset is not None and not np.isfinite(yaw_offset):
            raise ValueError('Room yaw offset must be finite')
        self.yaw_offset = yaw_offset
        self.velocity_variance = velocity_stddev ** 2
        self.drift_variance = drift_stddev ** 2
        self.velocity_tau, self.max_gap = velocity_tau, max_gap
        self.gate, self.max_innovation = gate, max_innovation
        self.odom = np.zeros(2)
        self.velocity = np.zeros(2)
        self.room = None
        self.covariance = np.eye(2)
        self.stamp = None
        self.uwb_stamp = None
        self.history = deque()

    def reset_room(self):
        self.room = None
        self.uwb_stamp = None

    def update_flow(self, velocity, imu_yaw, stamp):
        if not all(np.isfinite(v) for v in tuple(velocity) + (imu_yaw, stamp)) or stamp <= 0:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        dt = stamp - self.stamp if self.stamp is not None else 0.0
        measured = rotate(velocity, body_angle(imu_yaw))
        if not dt or dt > self.max_gap:
            # Do not integrate across a camera outage. Require a new UWB fix.
            self.reset_room()
            self.history.clear()
            self.velocity = measured
        else:
            previous = self.velocity.copy()
            alpha = 1.0 - math.exp(-dt / self.velocity_tau)
            self.velocity += alpha * (measured - self.velocity)
            displacement = 0.5 * (previous + self.velocity) * dt
            self.odom += displacement
            if self.room is not None:
                self.room += rotate(displacement, self.yaw_offset)
                self.covariance += np.eye(2) * (self.velocity_variance * dt ** 2 + self.drift_variance * dt)
        self.stamp = stamp
        self.history.append((stamp, self.odom.copy()))
        while self.history and stamp - self.history[0][0] > 2.0:
            self.history.popleft()
        return True

    def odom_at(self, stamp):
        if not self.history or stamp < self.history[0][0] or stamp > self.history[-1][0]:
            return None
        for i, (time, position) in enumerate(self.history):
            if time == stamp:
                return position.copy()
            if time > stamp:
                before, p = self.history[i - 1]
                return p + (position - p) * ((stamp - before) / (time - before))
        return None

    def update_uwb(self, position, covariance, stamp):
        if self.yaw_offset is None or self.stamp is None:
            return False, 'Room heading is not calibrated or camera is missing'
        if self.uwb_stamp is not None and stamp <= self.uwb_stamp:
            return False, 'Repeated or out-of-order UWB fix'
        position, covariance = np.asarray(position, dtype=float), np.asarray(covariance, dtype=float)
        if (position.shape != (2,) or covariance.shape != (2, 2)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(covariance))
                or not np.isfinite(stamp) or stamp <= 0
                or not np.allclose(covariance, covariance.T)
                or np.min(np.linalg.eigvalsh(covariance)) <= 0):
            return False, 'Invalid UWB position/covariance'
        acquisition = self.odom_at(stamp)
        if acquisition is None:
            return False, 'UWB timestamp is outside camera history'
        # Bring delayed UWB to the latest camera time using measured displacement.
        observation = position + rotate(self.odom - acquisition, self.yaw_offset)
        age = self.stamp - stamp
        noise = covariance + np.eye(2) * (self.velocity_variance * age ** 2 + self.drift_variance * age)
        if self.room is None:
            self.room = observation
            self.covariance = noise
        else:
            innovation = observation - self.room
            total = self.covariance + noise
            if (np.linalg.norm(innovation) > self.max_innovation
                    or float(innovation.dot(np.linalg.solve(total, innovation))) > self.gate):
                return False, 'UWB innovation rejected'
            gain = np.linalg.solve(total.T, self.covariance.T).T
            self.room += gain.dot(innovation)
            remainder = np.eye(2) - gain
            self.covariance = remainder.dot(self.covariance).dot(remainder.T) + gain.dot(noise).dot(gain.T)
        self.uwb_stamp = stamp
        return True, 'OK'
