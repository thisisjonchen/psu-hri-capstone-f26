"""Planar velocity and position estimation, independent of ROS.

The existing controller uses x=right, y=forward. The FC's IMU yaw describes
forward in a conventional x/y world, so the controller basis is yaw - pi/2.
Local ("odom") vectors use that IMU world frame.

This is the standard loosely coupled layout (ROS REP-105):
- VelocityFilter: IMU acceleration predicts local velocity; optical flow, or a
  zero-velocity observation while not flying, corrects it. An acceleration
  bias state absorbs slowly varying error such as small attitude errors.
  Absolute position fixes have no input into velocity.
- Local odometry integrates that velocity, so it is smooth and never jumps.
- RoomOffset: room position = rotated odometry + offset. Absolute fixes (UWB)
  correct only the offset, which is the map -> odom transform.
- VerticalFilter: IMU vertical acceleration predicts height and climb rate;
  tilt-compensated ToF height corrects them.
"""
import math
from collections import deque

import numpy as np

# Maximum age of an accepted absolute fix while odometry carries the room pose.
XY_PREDICTION_TIMEOUT = 1.5
# Sensor history kept for delayed camera and UWB measurements (seconds).
HISTORY_SECONDS = 2.0


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    x, y = vector
    return np.array([c * x - s * y, s * x + c * y])


def body_angle(imu_yaw):
    return imu_yaw - math.pi / 2.0


def horizontal_acceleration(acceleration, attitude):
    """Gravity-compensated right/forward/up acceleration -> local world XY.

    The FC quaternion uses forward/left/up, unlike its acceleration topic.
    Include tilt before projecting onto XY; do not subtract gravity again.
    """
    right, forward, up = acceleration
    roll, pitch, yaw = attitude
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    return rotate((cp * forward + sp * (-sr * right + cr * up),
                   -cr * right - sr * up), yaw)


def vertical_acceleration(acceleration, attitude):
    """Gravity-compensated right/forward/up acceleration -> world up (m/s^2)."""
    right, forward, up = acceleration
    roll, pitch, _ = attitude
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    return -sp * forward + cp * (-sr * right + cr * up)


def target_velocity(target, position, angle, gain, limit):
    velocity = rotate(np.asarray(target) - position, -angle) * gain
    speed = np.linalg.norm(velocity)
    return velocity * min(1.0, limit / speed) if speed else velocity


def positive(*values):
    return all(np.isfinite(v) and v > 0 for v in values)


def valid_measurement(vector, covariance, stamp):
    vector, covariance = np.asarray(vector, dtype=float), np.asarray(covariance, dtype=float)
    return (vector.shape == (2,) and covariance.shape == (2, 2)
            and np.all(np.isfinite(vector)) and np.all(np.isfinite(covariance))
            and np.isfinite(stamp) and stamp > 0
            and np.allclose(covariance, covariance.T)
            and np.min(np.linalg.eigvalsh(covariance)) > 0)


def within_gate(innovation, total, gate):
    """Chi-square innovation consistency check, as used by PX4/ArduPilot EKFs."""
    return float(innovation.dot(np.linalg.solve(total, innovation))) <= gate


def kalman_update(state, covariance, observation, innovation, noise):
    """Joseph-form update, which keeps the covariance symmetric and positive."""
    total = observation.dot(covariance).dot(observation.T) + noise
    gain = np.linalg.solve(total.T, observation.dot(covariance.T)).T
    remainder = np.eye(len(state)) - gain.dot(observation)
    return (state + gain.dot(innovation),
            remainder.dot(covariance).dot(remainder.T) + gain.dot(noise).dot(gain.T))


class VelocityFilter(object):
    """Kalman filter on local [vx, vy, bias_x, bias_y].

    The bias is measured minus true acceleration. Odometry integrates the
    estimated velocity. Corrections may be delayed; they are compared with the
    velocity at their own timestamp using the IMU increments recorded since.
    """

    def __init__(self, acceleration_stddev=0.3, bias_stddev=0.02, timeout=0.5,
                 max_gap=0.5, gate=9.21, max_rejections=5):
        if not positive(acceleration_stddev, bias_stddev, timeout, max_gap, gate) or max_rejections < 1:
            raise ValueError('Velocity filter noise, timeouts, gate and rejection count must be positive')
        self.acceleration_variance = acceleration_stddev ** 2
        self.bias_variance = bias_stddev ** 2
        self.timeout, self.max_gap = timeout, max_gap
        self.gate, self.max_rejections = gate, max_rejections
        self.state = np.zeros(4)
        self.covariance = np.zeros((4, 4))
        # Prior bias uncertainty: about a 1 degree attitude error (0.17 m/s^2).
        self.covariance[2:, 2:] = np.eye(2) * 0.17 ** 2
        self.odom = np.zeros(2)
        self.stamp = None
        self.corrected_stamp = None
        self.initialized = False
        self.rejections = 0
        self.integrated = np.zeros(2)  # Sum of IMU velocity increments.
        self.history = deque()

    @property
    def velocity(self):
        return self.state[:2]

    def velocity_covariance(self):
        return self.covariance[:2, :2]

    def ready(self, now):
        return bool(self.initialized and self.corrected_stamp is not None
                    and 0 <= now - self.corrected_stamp <= self.timeout)

    def reset(self):
        """Forget velocity; keep the learned bias, which remains valid."""
        self.initialized = False
        self.corrected_stamp = None
        self.rejections = 0
        self.state[:2] = 0
        self.covariance[:2, :] = self.covariance[:, :2] = 0

    def predict(self, acceleration, stamp):
        """Integrate one local acceleration sample (m/s^2), or None when invalid."""
        if acceleration is not None:
            acceleration = np.asarray(acceleration, dtype=float)
            if acceleration.shape != (2,) or not np.all(np.isfinite(acceleration)):
                return False
        if not np.isfinite(stamp) or stamp <= 0 or (self.stamp is not None and stamp <= self.stamp):
            return False
        dt = 0.0 if self.stamp is None else stamp - self.stamp
        if dt > self.max_gap:
            # A stopped estimator cannot reconstruct unobserved motion.
            self.reset()
            self.history.clear()
            dt = 0.0
        if self.initialized and stamp - self.corrected_stamp > self.timeout:
            self.reset()
        self.stamp = stamp
        transition = np.eye(4)
        transition[:2, 2:] = -np.eye(2) * dt
        noise = np.zeros((4, 4))
        noise[:2, :2] = np.eye(2) * self.acceleration_variance * dt
        noise[2:, 2:] = np.eye(2) * self.bias_variance * dt
        if self.initialized:
            previous = self.state[:2].copy()
            if acceleration is not None:
                increment = (acceleration - self.state[2:]) * dt
                self.state[:2] += increment
                self.integrated += increment
            self.covariance = transition.dot(self.covariance).dot(transition.T) + noise
            self.odom += (previous + self.state[:2]) * dt / 2.0
        else:
            self.covariance[2:, 2:] += noise[2:, 2:]
        self.history.append((stamp, self.odom.copy(), self.integrated.copy()))
        while self.history and stamp - self.history[0][0] > HISTORY_SECONDS:
            self.history.popleft()
        return True

    def _at(self, stamp):
        """Interpolated (odometry, IMU increment total) at a past stamp."""
        if not self.history or stamp < self.history[0][0]:
            return None
        if stamp >= self.history[-1][0]:
            return self.history[-1][1], self.history[-1][2]
        for (t0, odom0, sum0), (t1, odom1, sum1) in zip(self.history, list(self.history)[1:]):
            if t0 <= stamp <= t1:
                w = (stamp - t0) / (t1 - t0)
                return odom0 + w * (odom1 - odom0), sum0 + w * (sum1 - sum0)
        return None

    def odom_at(self, stamp):
        sample = self._at(stamp)
        return None if sample is None else sample[0]

    def correct(self, velocity, variance, stamp, gated=True):
        """Correct with a measured local velocity, such as optical flow or zero."""
        velocity = np.asarray(velocity, dtype=float)
        if not valid_measurement(velocity, np.eye(2) * variance, stamp):
            return False, 'Invalid velocity measurement'
        if self.corrected_stamp is not None and stamp <= self.corrected_stamp:
            return False, 'Repeated or out-of-order velocity measurement'
        sample = self._at(stamp)
        if sample is None:
            return False, 'Velocity timestamp is outside IMU history'
        noise = np.eye(2) * variance
        # IMU increments since the measurement carry it to the present.
        present = velocity + self.integrated - sample[1]
        if not self.initialized:
            return self._reset_to(present, noise, stamp, 'Velocity initialized')
        innovation = present - self.state[:2]
        observation = np.hstack((np.eye(2), np.zeros((2, 2))))
        if gated and not within_gate(innovation, self.covariance[:2, :2] + noise, self.gate):
            self.rejections += 1
            if self.rejections < self.max_rejections:
                return False, 'Optical-flow innovation rejected'
            # Persistent disagreement means the estimate, not the sensor, is wrong.
            return self._reset_to(present, noise, stamp, 'Velocity reset to measurement')
        self.state, self.covariance = kalman_update(
            self.state, self.covariance, observation, innovation, noise)
        self.corrected_stamp, self.rejections = stamp, 0
        return True, 'OK'

    def _reset_to(self, velocity, noise, stamp, reason):
        self.state[:2] = velocity
        self.covariance[:2, :] = self.covariance[:, :2] = 0
        self.covariance[:2, :2] = noise
        self.initialized, self.corrected_stamp, self.rejections = True, stamp, 0
        return True, reason


class RoomOffset(object):
    """Room XY = rotate(local odometry, yaw_offset) + offset.

    Absolute fixes correct only the offset, so they never change velocity and
    a noisy fix moves the room estimate by a filtered fraction of its error.
    """

    def __init__(self, yaw_offset=None, drift_stddev=0.08, gate=9.21,
                 max_innovation=1.0, timeout=XY_PREDICTION_TIMEOUT):
        if not positive(drift_stddev, gate, max_innovation, timeout):
            raise ValueError('Room drift, gate, innovation limit and timeout must be positive')
        if yaw_offset is not None and not np.isfinite(yaw_offset):
            raise ValueError('Room yaw offset must be finite')
        self.yaw_offset = yaw_offset
        self.drift_variance = drift_stddev ** 2
        self.gate, self.max_innovation, self.timeout = gate, max_innovation, timeout
        self.reset()

    def reset(self):
        self.offset = None
        self.offset_covariance = None
        self.stamp = None

    def ready(self, now):
        return bool(self.offset is not None and 0 <= now - self.stamp <= self.timeout)

    def position(self, odom):
        return rotate(odom, self.yaw_offset) + self.offset

    def covariance(self, now):
        """Offset uncertainty grows with odometry drift since the last fix."""
        return self.offset_covariance + np.eye(2) * self.drift_variance * max(0.0, now - self.stamp)

    def update(self, position, covariance, stamp, odom):
        """Correct the offset with an absolute fix and odometry at its timestamp."""
        if self.yaw_offset is None:
            return False, 'Room heading is not initialized'
        if odom is None:
            return False, 'Fix timestamp is outside odometry history'
        if not valid_measurement(position, covariance, stamp):
            return False, 'Invalid absolute position/covariance'
        if self.stamp is not None and stamp <= self.stamp:
            return False, 'Repeated or out-of-order absolute fix'
        position, covariance = np.asarray(position, dtype=float), np.asarray(covariance, dtype=float)
        observed = position - rotate(odom, self.yaw_offset)
        if self.offset is None:
            self.offset, self.offset_covariance, self.stamp = observed, covariance, stamp
            return True, 'OK'
        prior = self.covariance(stamp)
        innovation = observed - self.offset
        if (np.linalg.norm(innovation) > self.max_innovation
                or not within_gate(innovation, prior + covariance, self.gate)):
            return False, 'UWB innovation rejected'
        self.offset, self.offset_covariance = kalman_update(
            self.offset, prior, np.eye(2), innovation, covariance)
        self.stamp = stamp
        return True, 'OK'

    def correct_yaw(self, delta, odom):
        """Correct heading without moving the current room position."""
        if self.yaw_offset is None or not np.isfinite(delta):
            return False
        room = self.position(odom) if self.offset is not None else None
        self.yaw_offset += delta
        if room is not None:
            self.offset = room - rotate(odom, self.yaw_offset)
        return True


class VerticalFilter(object):
    """Kalman filter on [height, climb rate, vertical acceleration bias].

    ToF readings arrive after their timing budget; each is compared with the
    height at its own stamp using the displacement predicted since.
    """

    def __init__(self, acceleration_stddev=0.3, bias_stddev=0.05, max_gap=0.5):
        if not positive(acceleration_stddev, bias_stddev, max_gap):
            raise ValueError('Vertical filter noise and gap must be positive')
        self.acceleration_variance = acceleration_stddev ** 2
        self.bias_variance = bias_stddev ** 2
        self.max_gap = max_gap
        self.state = np.zeros(3)
        self.covariance = np.diag((0.0, 0.0, 0.5 ** 2))
        self.stamp = None
        self.initialized = False
        self.moved = 0.0  # Predicted height change since start.
        self.history = deque()

    @property
    def climb_rate(self):
        return self.state[1] if self.initialized else 0.0

    def predict(self, acceleration, stamp):
        """Integrate one world-up acceleration sample (m/s^2), or None when invalid."""
        if acceleration is not None and not np.isfinite(acceleration):
            return False
        if not np.isfinite(stamp) or stamp <= 0 or (self.stamp is not None and stamp <= self.stamp):
            return False
        dt = 0.0 if self.stamp is None else stamp - self.stamp
        if dt > self.max_gap:
            self.initialized = False
            self.history.clear()
            dt = 0.0
        self.stamp = stamp
        if self.initialized:
            a = 0.0 if acceleration is None else acceleration - self.state[2]
            step = self.state[1] * dt + 0.5 * a * dt * dt
            self.state[0] += step
            self.state[1] += a * dt
            self.moved += step
            transition = np.array([[1, dt, -0.5 * dt * dt], [0, 1, -dt], [0, 0, 1]])
            noise = np.zeros((3, 3))
            noise[:2, :2] = np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]]) * self.acceleration_variance
            noise[2, 2] = self.bias_variance * dt
            self.covariance = transition.dot(self.covariance).dot(transition.T) + noise
        self.history.append((stamp, self.moved))
        while self.history and stamp - self.history[0][0] > HISTORY_SECONDS:
            self.history.popleft()
        return True

    def _moved_at(self, stamp):
        if not self.history or stamp < self.history[0][0]:
            return None
        if stamp >= self.history[-1][0]:
            return self.history[-1][1]
        for (t0, m0), (t1, m1) in zip(self.history, list(self.history)[1:]):
            if t0 <= stamp <= t1:
                return m0 + (stamp - t0) / (t1 - t0) * (m1 - m0)
        return None

    def correct(self, height, variance, stamp):
        """Correct with a vertical height measurement (m) taken at stamp."""
        if not (np.isfinite(height) and positive(variance) and np.isfinite(stamp) and stamp > 0):
            return False
        moved = self._moved_at(stamp)
        if moved is None:
            return False
        present = height + self.moved - moved
        if not self.initialized:
            self.state[:2] = present, 0.0
            self.covariance[:2, :] = self.covariance[:, :2] = 0
            self.covariance[0, 0], self.covariance[1, 1] = variance, 0.1 ** 2
            self.initialized = True
            return True
        self.state, self.covariance = kalman_update(
            self.state, self.covariance, np.array([[1.0, 0.0, 0.0]]),
            np.array([present - self.state[0]]), np.array([[variance]]))
        return True
