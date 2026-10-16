"""Planar Kalman fusion and frame conversion, independent of ROS.

The existing controller uses x=right, y=forward. The FC's IMU yaw describes
forward in a conventional x/y world, so the controller basis is yaw - pi/2.
Camera velocity drives prediction; UWB corrects room position and velocity.
Local odometry integrates the corrected velocity without position jumps.
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
                 max_gap=0.25, gate=9.21, max_innovation=1.0,
                 velocity_correction_stddev=0.3):
        values = (velocity_stddev, drift_stddev, velocity_tau, max_gap, gate,
                  max_innovation, velocity_correction_stddev)
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('Fusion noise, timeouts and gates must be positive')
        if yaw_offset is not None and not np.isfinite(yaw_offset):
            raise ValueError('Room yaw offset must be finite')
        self.yaw_offset = yaw_offset
        self.velocity_variance = velocity_stddev ** 2
        self.drift_variance = drift_stddev ** 2
        self.velocity_correction_variance = velocity_correction_stddev ** 2
        self.velocity_tau, self.max_gap = velocity_tau, max_gap
        self.gate, self.max_innovation = gate, max_innovation
        self.odom = np.zeros(2)
        self.flow_odom = np.zeros(2)
        self.flow_velocity = np.zeros(2)
        self.velocity = np.zeros(2)
        # Kalman state: room X/Y and additive room-frame camera velocity error.
        # Keeping this correction separate prevents fresh zero flow from erasing it.
        self.velocity_correction = np.zeros(2)
        self.room = None
        self.covariance = np.eye(4)
        self.stamp = None
        self.flow_stamp = None
        self.uwb_stamp = None
        self.history = deque()

    def reset_room(self):
        self.room = None
        self.uwb_stamp = None
        self.velocity_correction[:] = 0
        self.velocity = self.flow_velocity.copy()

    def _correct_velocity(self):
        self.velocity = self.flow_velocity.copy()
        if self.yaw_offset is not None:
            self.velocity += rotate(self.velocity_correction, -self.yaw_offset)

    def predict(self, stamp):
        """Advance on the estimator clock, even when the camera is unavailable."""
        if not np.isfinite(stamp) or stamp <= 0:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        dt = stamp - self.stamp if self.stamp is not None else 0.0
        if dt > self.max_gap:
            # A stopped estimator cannot reconstruct unobserved motion.
            self.reset_room()
            self.history.clear()
        elif dt:
            displacement = self.velocity * dt
            self.odom += displacement
            self.flow_odom += self.flow_velocity * dt
            if self.room is not None:
                self.room += rotate(displacement, self.yaw_offset)
                transition = np.eye(4)
                transition[:2, 2:] = np.eye(2) * dt
                # ROS Kinetic's NumPy predates np.block.
                noise = np.zeros((4, 4))
                noise[:2, :2] = np.eye(2) * dt**3 / 3
                noise[:2, 2:] = noise[2:, :2] = np.eye(2) * dt**2 / 2
                noise[2:, 2:] = np.eye(2) * dt
                noise *= self.velocity_correction_variance
                noise[:2, :2] += np.eye(2) * (self.velocity_variance * dt**2 + self.drift_variance * dt)
                self.covariance = transition.dot(self.covariance).dot(transition.T) + noise
        self.stamp = stamp
        self.history.append((stamp, self.flow_odom.copy()))
        while self.history and stamp - self.history[0][0] > 2.0:
            self.history.popleft()
        return True

    def update_flow(self, velocity, imu_yaw, stamp):
        if not all(np.isfinite(v) for v in tuple(velocity) + (imu_yaw, stamp)) or stamp <= 0:
            return False
        if self.flow_stamp is not None and stamp <= self.flow_stamp:
            return False
        # Sensor delivery may lag the prediction clock; never rewind it.
        self.predict(stamp)
        measured = rotate(velocity, body_angle(imu_yaw))
        dt = stamp - self.flow_stamp if self.flow_stamp is not None else 0.0
        previous = self.flow_velocity.copy()
        if not dt or dt > self.max_gap:
            self.flow_velocity = measured
            if self.room is not None:
                # Rebase after camera recovery without double-counting the motion
                # that UWB has already learned while the camera was unavailable.
                self.velocity_correction += rotate(previous - measured, self.yaw_offset)
        else:
            alpha = 1.0 - math.exp(-dt / self.velocity_tau)
            self.flow_velocity += alpha * (measured - self.flow_velocity)
        self.flow_stamp = stamp
        self._correct_velocity()
        return True

    def flow_odom_at(self, stamp):
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
            return False, 'Room heading or prediction is not initialized'
        if self.uwb_stamp is not None and stamp <= self.uwb_stamp:
            return False, 'Repeated or out-of-order UWB fix'
        position, covariance = np.asarray(position, dtype=float), np.asarray(covariance, dtype=float)
        if (position.shape != (2,) or covariance.shape != (2, 2)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(covariance))
                or not np.isfinite(stamp) or stamp <= 0
                or not np.allclose(covariance, covariance.T)
                or np.min(np.linalg.eigvalsh(covariance)) <= 0):
            return False, 'Invalid UWB position/covariance'
        acquisition = self.flow_odom_at(stamp)
        if acquisition is None:
            return False, 'UWB timestamp is outside prediction history'
        # Advance delayed fixes using camera displacement. The observation matrix
        # accounts for the estimated velocity correction over the same delay.
        observation = position + rotate(self.flow_odom - acquisition, self.yaw_offset)
        age = self.stamp - stamp
        noise = covariance + np.eye(2) * (self.velocity_variance * age ** 2 + self.drift_variance * age)
        if self.room is None:
            self.room = observation
            self.covariance = np.zeros((4, 4))
            self.covariance[2:, 2:] = np.eye(2) * self.velocity_correction_variance
            self.covariance[:2, :2] = noise + age**2 * self.covariance[2:, 2:]
            self.covariance[:2, 2:] = self.covariance[2:, :2] = age * self.covariance[2:, 2:]
        else:
            observation_matrix = np.hstack((np.eye(2), -age * np.eye(2)))
            innovation = observation - (self.room - age * self.velocity_correction)
            cross = self.covariance.dot(observation_matrix.T)
            total = observation_matrix.dot(cross) + noise
            if (np.linalg.norm(innovation) > self.max_innovation
                    or float(innovation.dot(np.linalg.solve(total, innovation))) > self.gate):
                return False, 'UWB innovation rejected'
            gain = np.linalg.solve(total.T, cross.T).T
            correction = gain.dot(innovation)
            self.room += correction[:2]
            self.velocity_correction += correction[2:]
            self._correct_velocity()
            remainder = np.eye(4) - gain.dot(observation_matrix)
            self.covariance = remainder.dot(self.covariance).dot(remainder.T) + gain.dot(noise).dot(gain.T)
        self.uwb_stamp = stamp
        return True, 'OK'
