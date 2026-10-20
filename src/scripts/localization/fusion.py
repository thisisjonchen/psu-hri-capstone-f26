"""Planar Kalman fusion and frame conversion, independent of ROS.

The existing controller uses x=right, y=forward. The FC's IMU yaw describes
forward in a conventional x/y world, so the controller basis is yaw - pi/2.
UWB corrects room position and velocity; IMU acceleration predicts between fixes.
Controller velocity blends that estimate with timestamped optical flow.
Local odometry integrates the corrected velocity without position jumps.
"""
import math
from collections import deque

import numpy as np

# Maximum age of an accepted UWB fix while continuing an existing XY hold.
# IMU/height freshness and acceptance of newly received fixes remain separate.
XY_PREDICTION_TIMEOUT = 1.0


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


def target_velocity(target, position, angle, gain, limit):
    velocity = rotate(np.asarray(target) - position, -angle) * gain
    speed = np.linalg.norm(velocity)
    return velocity * min(1.0, limit / speed) if speed else velocity


class PlanarFusion(object):
    def __init__(self, yaw_offset=None, velocity_stddev=0.12,
                 drift_stddev=0.08,
                 max_gap=0.25, gate=9.21, max_innovation=1.0,
                 acceleration_stddev=0.3, flow_velocity_weight=0.3):
        values = (velocity_stddev, drift_stddev, max_gap, gate,
                  max_innovation, acceleration_stddev)
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('Fusion noise, timeouts and gates must be positive')
        if yaw_offset is not None and not np.isfinite(yaw_offset):
            raise ValueError('Room yaw offset must be finite')
        if not np.isfinite(flow_velocity_weight) or not 0 <= flow_velocity_weight <= 1:
            raise ValueError('Optical-flow velocity weight must be between zero and one')
        self.yaw_offset = yaw_offset
        self.velocity_variance = velocity_stddev ** 2
        self.drift_variance = drift_stddev ** 2
        self.acceleration_variance = acceleration_stddev ** 2
        self.max_gap = max_gap
        self.gate, self.max_innovation = gate, max_innovation
        self.odom = np.zeros(2)
        self.velocity = np.zeros(2)
        # Kalman state: room X/Y position and room X/Y velocity.
        self.room_velocity = np.zeros(2)
        self.room = None
        self.covariance = np.eye(4)
        self.stamp = None
        self.uwb_stamp = None
        self.flow_stamp = None
        self.flow_velocity = np.zeros(2)
        self.flow_variance = 0.0
        self.flow_velocity_weight = flow_velocity_weight
        self.history = deque()
        self.accelerations = deque()

    def update_acceleration(self, acceleration, stamp):
        """Store a room-frame input at its measurement time, never renew its age."""
        acceleration = np.asarray(acceleration, dtype=float)
        if (acceleration.shape != (2,) or not np.all(np.isfinite(acceleration))
                or not np.isfinite(stamp) or stamp <= 0
                or (self.accelerations and stamp <= self.accelerations[-1][0])):
            return False
        self.accelerations.append((stamp, acceleration.copy()))
        # Keep the last input before the observation history as well.
        while len(self.accelerations) > 1 and self.accelerations[1][0] < stamp - 2.0:
            self.accelerations.popleft()
        return True

    def acceleration_motion(self, start, end):
        """Known velocity/displacement increments for held, timestamped inputs."""
        velocity, displacement = np.zeros(2), np.zeros(2)
        samples = list(self.accelerations)
        for i, (stamp, acceleration) in enumerate(samples):
            stop = samples[i + 1][0] if i + 1 < len(samples) else end
            left, right = max(start, stamp), min(end, stop, stamp + self.max_gap)
            if right > left:
                duration = right - left
                velocity += acceleration * duration
                displacement += acceleration * duration * (end - (left + right) / 2.0)
        return velocity, displacement

    def reset_room(self):
        self.room = None
        self.uwb_stamp = None
        self.flow_stamp = None
        self.room_velocity[:] = 0
        self.velocity[:] = 0

    def correct_yaw(self, delta):
        """Correct sensor heading without moving UWB state or continuous odometry."""
        if self.yaw_offset is None or not np.isfinite(delta):
            return False
        self.yaw_offset += delta
        # These vectors came from body sensors using the old offset. UWB state
        # and its covariance are already in the surveyed map and must not rotate.
        self.flow_velocity = rotate(self.flow_velocity, delta)
        self.accelerations = deque((stamp, rotate(a, delta)) for stamp, a in self.accelerations)
        if self.room is not None:
            self._update_local_velocity()
        return True

    def flow_active(self, stamp=None):
        now = self.stamp if stamp is None else stamp
        return bool(self.room is not None and self.flow_velocity_weight > 0
                    and self.flow_stamp is not None and now is not None
                    and 0 <= now - self.flow_stamp <= self.max_gap)

    def _update_local_velocity(self, stamp=None):
        now = self.stamp if stamp is None else stamp
        velocity = self.room_velocity.copy()
        if self.flow_active(now):
            delta_velocity, _ = self.acceleration_motion(self.flow_stamp, now)
            weight = self.flow_velocity_weight
            velocity = (1 - weight) * velocity + weight * (self.flow_velocity + delta_velocity)
        self.velocity = rotate(velocity, -self.yaw_offset)

    def velocity_covariance(self):
        covariance = self.covariance[2:, 2:] + np.eye(2) * self.velocity_variance
        if self.flow_active():
            noise = self.flow_variance + self.acceleration_variance * (self.stamp - self.flow_stamp)
            weight = self.flow_velocity_weight
            # Conservative bound: both branches share IMU attitude/prediction,
            # so do not assume their errors are independent.
            covariance = (1 - weight) * covariance + weight * np.eye(2) * noise
        return covariance

    def predict(self, stamp):
        """Predict with measured acceleration; fall back to constant velocity."""
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
            if self.room is not None:
                delta_velocity, delta_position = self.acceleration_motion(self.stamp, stamp)
                displacement += rotate(delta_position, -self.yaw_offset)
                self.room += self.room_velocity * dt + delta_position
                self.room_velocity += delta_velocity
                self._update_local_velocity(stamp)
                transition = np.eye(4)
                transition[:2, 2:] = np.eye(2) * dt
                # ROS Kinetic's NumPy predates np.block.
                noise = np.zeros((4, 4))
                noise[:2, :2] = np.eye(2) * dt**3 / 3
                noise[:2, 2:] = noise[2:, :2] = np.eye(2) * dt**2 / 2
                noise[2:, 2:] = np.eye(2) * dt
                noise *= self.acceleration_variance
                noise[:2, :2] += np.eye(2) * (self.velocity_variance * dt**2 + self.drift_variance * dt)
                self.covariance = transition.dot(self.covariance).dot(transition.T) + noise
            self.odom += displacement
        self.stamp = stamp
        self.history.append(stamp)
        while self.history and stamp - self.history[0] > 2.0:
            self.history.popleft()
        return True

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
        if not self.history or stamp < self.history[0] or stamp > self.history[-1]:
            return False, 'UWB timestamp is outside prediction history'
        # Undo known acceleration as well when observing a past position.
        age = self.stamp - stamp
        delta_velocity, delta_position = self.acceleration_motion(stamp, self.stamp)
        acceleration_offset = age * delta_velocity - delta_position
        noise = covariance + np.eye(2) * (self.velocity_variance * age ** 2 + self.drift_variance * age)
        if self.room is None:
            self.room = position.copy() + delta_position
            self.room_velocity = delta_velocity
            self._update_local_velocity()
            self.covariance = np.zeros((4, 4))
            self.covariance[2:, 2:] = np.eye(2) * self.acceleration_variance
            self.covariance[:2, :2] = noise + age**2 * self.covariance[2:, 2:]
            self.covariance[:2, 2:] = self.covariance[2:, :2] = age * self.covariance[2:, 2:]
        else:
            observation_matrix = np.hstack((np.eye(2), -age * np.eye(2)))
            innovation = position - (self.room - age * self.room_velocity + acceleration_offset)
            cross = self.covariance.dot(observation_matrix.T)
            total = observation_matrix.dot(cross) + noise
            if (np.linalg.norm(innovation) > self.max_innovation
                    or float(innovation.dot(np.linalg.solve(total, innovation))) > self.gate):
                return False, 'UWB innovation rejected'
            gain = np.linalg.solve(total.T, cross.T).T
            correction = gain.dot(innovation)
            self.room += correction[:2]
            self.room_velocity += correction[2:]
            self._update_local_velocity()
            remainder = np.eye(4) - gain.dot(observation_matrix)
            self.covariance = remainder.dot(self.covariance).dot(remainder.T) + gain.dot(noise).dot(gain.T)
        self.uwb_stamp = stamp
        return True, 'OK'

    def update_flow(self, velocity, variance, stamp):
        """Validate/cache flow for a fixed blend; never feed it back into UWB."""
        velocity = np.asarray(velocity, dtype=float)
        if (velocity.shape != (2,) or not np.all(np.isfinite(velocity))
                or not np.isfinite(variance) or variance <= 0
                or not np.isfinite(stamp) or stamp <= 0):
            return False, 'Invalid optical-flow velocity'
        if self.flow_stamp is not None and stamp <= self.flow_stamp:
            return False, 'Repeated or out-of-order optical flow'
        if self.room is None:
            return False, 'UWB room is not initialized'
        self.predict(stamp)
        if self.room is None or not self.history or stamp < self.history[0]:
            return False, 'Optical-flow timestamp is outside prediction history'
        # A delayed camera observation measures velocity before the known IMU
        # increments. Keep the prediction clock and continuous odometry intact.
        delta_velocity, _ = self.acceleration_motion(stamp, self.stamp)
        noise = np.eye(2) * (variance + self.acceleration_variance * (self.stamp - stamp))
        innovation = velocity + delta_velocity - self.room_velocity
        total = self.covariance[2:, 2:] + noise
        if float(innovation.dot(np.linalg.solve(total, innovation))) > self.gate:
            return False, 'Optical-flow innovation rejected'
        self.flow_velocity = velocity.copy()
        self.flow_variance = variance
        self.flow_stamp = stamp
        self._update_local_velocity()
        return True, 'OK'
