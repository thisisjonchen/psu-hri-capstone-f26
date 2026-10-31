"""Inertial EKF with position, velocity, attitude, and IMU bias states.

Body axes are forward/left/up and local Z is up. The quaternion (x, y, z, w)
rotates body vectors into the local frame. Inputs are specific force including
gravity (m/s^2) and body angular rate (rad/s).

The public covariance uses additive local position/velocity errors and right
body attitude error: q_true = q_nominal * Exp(delta_theta). Its order is
[delta_p, delta_v, delta_theta, delta_ba, delta_bg]. Internally, prediction and
correction use a right-invariant navigation error: one local-frame rotation
acts on attitude, position and velocity together. This keeps global yaw
unobservable to the floor sensors across successive corrections.

ekf_replay aligns measurements with the state timestamp; ekf_sensors provides
the range and camera models. Default noise values still need flight tuning.
"""
from __future__ import division

import numpy as np

from localization import ekf_native
from localization.rotations import skew, quaternion_matrix


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


def _error_transforms(position, velocity, rotation):
    """Convert between the public errors and invariant navigation errors."""
    to_invariant, to_public = np.eye(15), np.eye(15)
    for section, vector in ((slice(0, 3), position), (slice(3, 6), velocity)):
        cross = skew(vector)
        to_invariant[section, 6:9] = cross.dot(rotation)
        to_public[section, 6:9] = -cross
    to_invariant[6:9, 6:9] = rotation
    to_public[6:9, 6:9] = rotation.T
    return to_invariant, to_public


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

    @property
    def covariance(self):
        """Return a copy in local position/velocity and body attitude coordinates."""
        transform = _error_transforms(self.position, self.velocity,
                                      quaternion_matrix(self.quaternion))[1]
        covariance = transform.dot(self._covariance).dot(transform.T)
        return .5*(covariance + covariance.T)

    @covariance.setter
    def covariance(self, value):
        transform = _error_transforms(self.position, self.velocity,
                                      quaternion_matrix(self.quaternion))[0]
        self._covariance = transform.dot(value).dot(transform.T)

    def covariance_diagonal(self):
        transform = _error_transforms(self.position, self.velocity,
                                      quaternion_matrix(self.quaternion))[1]
        return np.einsum('ij,ij->i', transform.dot(self._covariance), transform)

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
        state = np.concatenate((self.position, self.velocity, self.quaternion))
        covariance = self._covariance.copy()
        if ekf_native.predict(state, covariance, force, rate, self.gravity, self.noise, dt):
            return False, 'numerical_failure'
        self.position, self.velocity, self.quaternion = state[:3].copy(), state[3:6].copy(), state[6:].copy()
        self._covariance = covariance
        self.stamp = stamp
        return True, 'predicted'

    def correct_zero_velocity(self, stamp, variance=.02**2):
        """Fuse v=0 when the caller has established that the drone is stationary."""
        if self.stamp is None:
            return self._reject('not_initialized')
        observation = np.zeros((3, 15))
        observation[:, 3:6] = np.eye(3)
        # The known zero velocity is unchanged by a rotation of the local frame.
        observation[:, 6:9] = skew(self.velocity).dot(quaternion_matrix(self.quaternion))
        return self._correct(np.zeros(3), self.velocity, observation, variance,
                             stamp, 'zero_velocity', 11.34)

    def correct_stationary_gyro(self, mean_rate, duration, stamp):
        """Use a confirmed period without rotation to observe gyro bias."""
        if self.stamp is None:
            return self._reject('not_initialized')
        if not np.isfinite(duration) or duration <= 0:
            return self._reject('invalid_stationary_duration')
        observation = np.zeros((3, 15))
        observation[:, 12:15] = np.eye(3)
        # Averaging white rate noise over this duration reduces its variance.
        variance = self.noise[6, 6]/duration
        return self._correct(mean_rate, self.gyro_bias, observation, variance,
                             stamp, 'stationary_gyro', 11.34)

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

        innovation = measured - predicted
        state = np.concatenate((self.position, self.velocity, self.quaternion,
                                self.acceleration_bias, self.gyro_bias))
        covariance = self._covariance.copy()
        status, total, nis = ekf_native.correct(state, covariance, innovation,
                                               observation, noise, gate)
        if status == 2:
            return self._reject('numerical_failure')
        # Use state time so timestamp rounding cannot admit a sample twice.
        self._measurement_stamps[sensor] = self.stamp
        self.last_update = dict(sensor=sensor, stamp=stamp, innovation=innovation.copy(),
                                innovation_covariance=total.copy(), nis=nis, accepted=nis <= gate,
                                reason='corrected' if nis <= gate else 'innovation_rejected')
        if nis > gate:
            return False, 'innovation_rejected'
        if status:
            return self._reject('numerical_failure')
        self.position, self.velocity = state[:3].copy(), state[3:6].copy()
        self.quaternion = state[6:10].copy()
        self.acceleration_bias, self.gyro_bias = state[10:13].copy(), state[13:16].copy()
        self._covariance = covariance
        return True, 'corrected'
