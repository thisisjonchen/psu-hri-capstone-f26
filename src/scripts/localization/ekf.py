"""Inertial EKF with position, velocity, attitude, and IMU bias states.

Body axes are forward/left/up and local Z is up. The quaternion (x, y, z, w)
rotates body vectors into the local frame. Inputs are specific force including
gravity (m/s^2) and body angular rate (rad/s).

Attitude error: q_true = q_nominal * Exp(delta_theta).
Covariance order: [delta_p, delta_v, delta_theta, delta_ba, delta_bg].
See Sola, https://arxiv.org/abs/1711.02508, sections 5-6.

ekf_replay aligns measurements with the state timestamp; ekf_sensors provides
the range and camera models. Default noise values still need flight tuning.
"""
from __future__ import division

import math
import numpy as np

from localization.rotations import (skew, quaternion_matrix, quaternion_multiply,
                                    quaternion_exp, right_jacobian)


def _array(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError('Invalid shape or nonfinite value')
    return result.copy()


def _stamp(value):
    value = float(value)
    if not np.isfinite(value) or value < 0:
        raise ValueError('Invalid timestamp')
    return value


class ErrorStateEKF(object):
    """Noise densities use m/s^2/sqrt(Hz) for acceleration and rad/s/sqrt(Hz)
    for gyro. Bias random walks use m/s^2/sqrt(s) and rad/s/sqrt(s).
    """
    def __init__(self, acceleration_noise=.3, gyro_noise=.015,
                 acceleration_bias_walk=.02, gyro_bias_walk=.001,
                 gravity=9.8, max_gap=.2):
        noise = _array((acceleration_noise, gyro_noise,
                        acceleration_bias_walk, gyro_bias_walk), (4,))
        if np.any(noise < 0) or not np.isfinite(gravity) or gravity <= 0:
            raise ValueError('Noise must be nonnegative and gravity positive')
        if not np.isfinite(max_gap) or not 0 < max_gap <= 1:
            raise ValueError('max_gap must be in (0, 1] seconds')
        # Isotropic body noise: rotation leaves each 3x3 diagonal unchanged.
        self.noise = np.diag(np.r_[np.zeros(3), np.repeat(noise**2, 3)])
        self.gravity = np.array([0., 0., -gravity])
        self.max_gap = max_gap
        self.stamp = None
        self.last_update = None
        self.camera_clone = None

    def initialize(self, stamp, quaternion, position=(0., 0., 0.),
                   velocity=(0., 0., 0.), acceleration_bias=(0., 0., 0.),
                   gyro_bias=(0., 0., 0.), covariance=None):
        """Set the initial state and optional 15x15 covariance.

        The caller supplies attitude and decides when to reinitialize after gaps.
        """
        stamp = _stamp(stamp)
        q = _array(quaternion, (4,))
        quaternion_matrix(q)  # Check the norm before normalizing.
        q /= np.linalg.norm(q)
        p, v, ba, bg = [_array(x, (3,)) for x in
                        (position, velocity, acceleration_bias, gyro_bias)]
        if covariance is None:
            # Start with more uncertainty in yaw than in tilt.
            std = np.r_[[.1]*3, [.2]*3, np.deg2rad([10., 10., 45.]), [.5]*3, [.05]*3]
            covariance = np.diag(std**2)
        covariance = _array(covariance, (15, 15))
        if (not np.allclose(covariance, covariance.T, rtol=0, atol=1e-12)
                or np.min(np.linalg.eigvalsh(covariance)) < 0):
            raise ValueError('Initial covariance must be symmetric positive semidefinite')
        self.position, self.velocity, self.quaternion = p, v, q
        self.acceleration_bias, self.gyro_bias = ba, bg
        self.covariance, self.stamp = covariance, stamp
        self.last_update = None
        self._measurement_stamps = {}
        self.camera_clone = None

    def clone_camera_pose(self):
        """Save the camera frame's pose and its covariance with the current state."""
        indices = [0, 1, 2, 6, 7, 8]
        self.camera_clone = dict(position=self.position.copy(), quaternion=self.quaternion.copy(),
                                 stamp=self.stamp, cross=self.covariance[:, indices].copy(),
                                 covariance=self.covariance[np.ix_(indices, indices)].copy())

    def joint_covariance(self):
        if self.camera_clone is None:
            return self.covariance
        clone = self.camera_clone
        return np.vstack((np.hstack((self.covariance, clone['cross'])),
                          np.hstack((clone['cross'].T, clone['covariance']))))

    def predict(self, specific_force, angular_rate, stamp):
        """Integrate this interval's mean IMU reading up to stamp.

        Invalid samples and gaps over max_gap leave the state unchanged.
        """
        if self.stamp is None:
            return False, 'not_initialized'
        try:
            stamp = _stamp(stamp)
            force = _array(specific_force, (3,)) - self.acceleration_bias
            rate = _array(angular_rate, (3,)) - self.gyro_bias
        except (TypeError, ValueError, OverflowError):
            return False, 'invalid_imu'
        if not np.isfinite(np.linalg.norm(force)) or not np.isfinite(np.linalg.norm(rate)):
            return False, 'invalid_imu'
        dt = stamp - self.stamp
        if dt <= 0:
            return False, 'non_increasing_stamp'
        if dt > self.max_gap:
            return False, 'imu_gap'
        # Keep integration steps at 10 ms or less.
        steps = max(1, int(math.ceil(dt/.01 - 1e-10)))
        step = dt/steps
        p, v, q, covariance = (self.position.copy(), self.velocity.copy(),
                                self.quaternion.copy(), self.covariance.copy())
        cross = None if self.camera_clone is None else self.camera_clone['cross'].copy()
        for _ in range(steps):
            mid_q = quaternion_multiply(q, quaternion_exp(rate * step/2))
            rotation = quaternion_matrix(mid_q)
            acceleration = rotation.dot(force) + self.gravity
            p += v*step + .5*acceleration*step**2
            v += acceleration*step
            q = quaternion_multiply(q, quaternion_exp(rate*step))
            q /= np.linalg.norm(q)

            dynamics = np.zeros((15, 15))
            dynamics[:3, 3:6] = np.eye(3)
            dynamics[3:6, 6:9] = -rotation.dot(skew(force))
            dynamics[3:6, 9:12] = -rotation
            dynamics[6:9, 6:9] = -skew(rate)
            dynamics[6:9, 12:15] = -np.eye(3)
            a = dynamics*step
            a2 = a.dot(a)
            a3 = a2.dot(a)
            transition = np.eye(15) + a + a2/2 + a3/6
            half = np.eye(15) + a/2 + a2/8 + a3/48
            # Integrate process noise with Simpson's rule.
            noise = step/6 * (self.noise + 4*half.dot(self.noise).dot(half.T)
                             + transition.dot(self.noise).dot(transition.T))
            covariance = transition.dot(covariance).dot(transition.T) + noise
            if cross is not None:
                cross = transition.dot(cross)
        if not all(np.all(np.isfinite(x)) for x in (p, v, q, covariance)):
            return False, 'numerical_failure'
        self.position, self.velocity, self.quaternion = p, v, q
        self.covariance = .5*(covariance + covariance.T)
        self.stamp = stamp
        if cross is not None:
            self.camera_clone['cross'] = cross
        return True, 'predicted'

    def _reject(self, reason):
        self.last_update = dict(accepted=False, reason=reason)
        return False, reason

    def _correct(self, measured, predicted, observation, noise, stamp, sensor, gate):
        size = len(predicted)
        try:
            stamp = _stamp(stamp)
            measured = _array(measured, (size,))
            noise = np.asarray(noise, dtype=float)
            if noise.shape == ():
                noise = np.eye(size)*noise
            noise = _array(noise, (size, size))
            if not np.allclose(noise, noise.T, rtol=0, atol=1e-12):
                return self._reject('invalid_measurement_covariance')
            np.linalg.cholesky(noise)
            if not np.isfinite(gate) or gate <= 0:
                return self._reject('invalid_gate')
        except (ValueError, TypeError, OverflowError, np.linalg.LinAlgError):
            return self._reject('invalid_measurement')
        if abs(stamp - self.stamp) > 1e-6:
            return self._reject('measurement_not_at_state_time')
        if self.stamp <= self._measurement_stamps.get(sensor, -1):
            return self._reject('repeated_measurement')

        prior = self.joint_covariance()
        if observation.shape[1] == 15 and self.camera_clone is not None:
            observation = np.hstack((observation, np.zeros((size, 6))))
        innovation = measured - predicted
        total = observation.dot(prior).dot(observation.T) + noise
        try:
            nis = float(innovation.dot(np.linalg.solve(total, innovation)))
            gain = np.linalg.solve(total, observation.dot(prior)).T
        except np.linalg.LinAlgError:
            return self._reject('numerical_failure')
        if not np.isfinite(nis):
            return self._reject('numerical_failure')
        # Use state time so timestamp rounding cannot admit a sample twice.
        self._measurement_stamps[sensor] = self.stamp
        self.last_update = dict(sensor=sensor, stamp=stamp, innovation=innovation.copy(),
                                innovation_covariance=total.copy(), nis=nis, accepted=nis <= gate,
                                reason='corrected' if nis <= gate else 'innovation_rejected')
        if nis > gate:
            return False, 'innovation_rejected'
        delta = gain.dot(innovation)
        remainder = np.eye(len(prior)) - gain.dot(observation)
        covariance = (remainder.dot(prior).dot(remainder.T)
                      + gain.dot(noise).dot(gain.T))  # Joseph form.
        reset = np.eye(len(prior))
        reset[6:9, 6:9] = right_jacobian(delta[6:9])
        if self.camera_clone is not None:
            reset[18:21, 18:21] = right_jacobian(delta[18:21])
        covariance = reset.dot(covariance).dot(reset.T)
        q = quaternion_multiply(self.quaternion, quaternion_exp(delta[6:9]))
        if not np.all(np.isfinite(delta)) or not np.all(np.isfinite(covariance)):
            return self._reject('numerical_failure')
        self.position += delta[:3]
        self.velocity += delta[3:6]
        self.quaternion = q/np.linalg.norm(q)
        self.acceleration_bias += delta[9:12]
        self.gyro_bias += delta[12:15]
        covariance = .5*(covariance + covariance.T)
        self.covariance = covariance[:15, :15]
        if self.camera_clone is not None:
            clone = self.camera_clone
            clone['position'] += delta[15:18]
            cq = quaternion_multiply(clone['quaternion'], quaternion_exp(delta[18:21]))
            clone['quaternion'] = cq/np.linalg.norm(cq)
            clone['cross'], clone['covariance'] = covariance[:15, 15:], covariance[15:, 15:]
        return True, 'corrected'
