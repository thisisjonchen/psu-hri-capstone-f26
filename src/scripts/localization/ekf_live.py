"""Live sensor adapter for the EKF EKF node.

Start with the drone resting on the floor. Ground rest can be reacquired after
set-down, but it cannot restore a local position reference lost while coasting.
"""
from __future__ import division

import numpy as np

from localization.ekf import ErrorStateEKF, _array
from localization.ekf_replay import DelayedEKF
from localization.ekf_sensors import FloorCamera, FloorRange, mounting_rotation
from localization.rotations import quaternion_matrix, quaternion_multiply, skew


class LiveEKF(object):
    # Provisional uncertainty limits; flight validation is still needed.
    REST_CONFIRMATION = 1.0
    MAX_UNAIDED_INTERVAL = 2.0
    MAX_POSITION_STDDEV = .5
    MAX_VELOCITY_STDDEV = .5

    def __init__(self, profile):
        self.rotation = mounting_rotation(profile['rotation_body_msp'])
        self.scale = _array([profile['acceleration_m_s2_per_count'],
                             profile['gyro_rad_s_per_count']], (2,))
        if np.any(self.scale <= 0):
            raise ValueError('Positive IMU scales are required')
        self.offset = _array(profile['acceleration_offset_msp_counts'], (3,))
        self.range_model = FloorRange(profile['range_offset_body'], profile['range_beam_body'])
        self.camera_rotation = mounting_rotation(profile['rotation_body_camera'])
        self.camera_offset = _array(profile['camera_offset_body'], (3,))
        self.delay = float(profile['fusion_delay_s'])
        if not 0 <= self.delay <= .4:
            raise ValueError('Live fusion delay must be in [0, 0.4] seconds')
        self.origin_ns = None
        self.timeline = self.camera = self.camera_signature = None
        self.mode, self.mode_at = None, -float('inf')
        self.range_sample = None
        self.samples = []
        self.last_imu = None
        self.rate = np.zeros(3)
        self.ground_range = None
        self.rest_since = self.rest_reference = None
        self.stationary = False
        self.reference_valid = True
        self.last_aid = {}
        self.fault = None
        self.reason = 'Hold still on the floor for eight seconds while disarmed'

    def fail(self, reason):
        self.fault, self.reason = reason, reason
        self._clear_rest()

    def _clear_rest(self):
        self.stationary = False
        self.rest_since = self.rest_reference = None
        self.samples = []

    def set_mode(self, mode, received):
        self.mode, self.mode_at = mode, received
        if mode != 'DISARMED' and not (mode == 'ARMED' and self.timeline is not None):
            self._clear_rest()

    def set_camera(self, width, height, matrix, distortion, model):
        signature = (width, height, tuple(matrix), tuple(distortion), model)
        if signature != self.camera_signature:
            self.camera = FloorCamera(matrix, distortion, model, (width, height),
                                      self.camera_rotation, self.camera_offset)
            self.camera_signature = signature
            self.last_aid.pop('camera', None)

    def add_range(self, stamp_ns, distance, minimum, maximum):
        valid = (np.all(np.isfinite([distance, minimum, maximum]))
                 and 0 <= minimum < maximum and minimum <= distance <= maximum)
        if stamp_ns <= 0 or not valid:
            self.range_sample = None
            self._clear_rest()
            return
        if self.range_sample is not None and stamp_ns <= self.range_sample[0]:
            return
        self.range_sample = (stamp_ns, distance)
        if self.timeline is not None and not self.fault:
            stamp = (stamp_ns-self.origin_ns)/1e9
            self.timeline.add_measurement(stamp, 'range',
                lambda f, t: self.range_model.correct(f, distance, .02**2, t, minimum, maximum))

    def add_image(self, stamp_ns, encoded):
        if self.timeline is None or self.camera is None or self.fault or stamp_ns <= 0:
            return
        camera, timeline = self.camera, self.timeline
        pair, _ = camera.image_pair(encoded, (stamp_ns-self.origin_ns)/1e9)
        if pair is not None:
            timeline.add_measurement((pair[0]+pair[1])/2, 'camera',
                lambda f, t: camera.correct_image(f, pair, t,
                    timeline.mean_angular_rate(pair[0], pair[1])))

    def _on_floor(self, stamp_ns, received):
        ground_mode = (self.mode == 'DISARMED'
                       or self.mode == 'ARMED' and self.timeline is not None)
        return (ground_mode and 0 <= received-self.mode_at <= .5
                and self.range_sample is not None
                and -.1 <= (stamp_ns-self.range_sample[0])/1e9 <= .3
                and .015 <= self.range_sample[1] <= .08)

    def _ground_rest(self, stamp_ns, received, force, rate):
        quiet = (self._on_floor(stamp_ns, received)
                 and np.linalg.norm(rate) < .03
                 and abs(np.linalg.norm(force)-9.8) < .5 and force[2] > 9.)
        if quiet and self.timeline is not None:
            quiet = np.linalg.norm(rate-self.timeline.ekf.gyro_bias) < .015
        if quiet and self.ground_range is not None:
            quiet = abs(self.range_sample[1]-self.ground_range) < .015
        if quiet and self.rest_reference is not None:
            quiet = (np.linalg.norm(force-self.rest_reference[0]) < .3
                     and abs(self.range_sample[1]-self.rest_reference[1]) < .015)
        if not quiet:
            self._clear_rest()
            return False
        if self.rest_since is None:
            self.rest_since = stamp_ns
            self.rest_reference = (force.copy(), self.range_sample[1])
        return (stamp_ns-self.rest_since)/1e9 >= self.REST_CONFIRMATION

    def _check_reference(self, stamp):
        aiding = max(self.last_aid.get('camera', 0.), self.last_aid.get('zero_velocity', 0.))
        if stamp-aiding > self.MAX_UNAIDED_INTERVAL:
            self.reference_valid = False

    def imu_reading(self, accelerometer, gyroscope):
        force = self.rotation.dot((_array(accelerometer, (3,))-self.offset)*self.scale[0])
        rate = self.rotation.dot(_array(gyroscope, (3,))*self.scale[1])
        return force, rate

    def add_imu(self, stamp_ns, accelerometer, gyroscope, received):
        if self.fault or stamp_ns <= 0:
            return
        force, rate = self.imu_reading(accelerometer, gyroscope)
        if self.last_imu is not None:
            if stamp_ns <= self.last_imu:
                return
            if (stamp_ns-self.last_imu)/1e9 > .2:
                if self.timeline is not None:
                    self.fail('IMU gap; restart the EKF node on the floor')
                    return
                self._clear_rest()
        self.last_imu, self.rate = stamp_ns, rate
        self.stationary = self._ground_rest(stamp_ns, received, force, rate)
        if self.timeline is None:
            if self.rest_since is None:
                return
            self.samples.append((stamp_ns, force, rate, self.range_sample[1]))
            if (stamp_ns-self.samples[0][0])/1e9 < 8:
                return
            force_mean = np.mean([s[1] for s in self.samples], axis=0)
            rate_mean = np.mean([s[2] for s in self.samples], axis=0)
            distance = float(np.mean([s[3] for s in self.samples]))
            up = force_mean/np.linalg.norm(force_mean)
            q = np.r_[np.cross(up, [0., 0., 1.]), 1+up[2]]
            q /= np.linalg.norm(q)
            rotation = quaternion_matrix(q)
            height = -(rotation.dot(self.range_model.offset)
                       + distance*rotation.dot(self.range_model.beam))[2]
            if not np.isfinite(height) or height <= 0:
                self.fail('Invalid initial floor geometry')
                return
            self.origin_ns = stamp_ns
            ekf = ErrorStateEKF()
            ekf.initialize(0., q, position=[0., 0., height],
                acceleration_bias=force_mean-rotation.T.dot([0., 0., 9.8]), gyro_bias=rate_mean)
            self.timeline = DelayedEKF(ekf, self.delay, record_states=False)
            self.ground_range = distance
            self.samples = []
            self.reason = 'Initialized; waiting for fused range'
            return
        stamp = (stamp_ns-self.origin_ns)/1e9
        _, events = self.timeline.add_imu(force, rate, stamp, stationary=self.stationary)
        for event in events:
            # Check the gap before a returning measurement can hide it.
            self._check_reference(event['stamp'])
            if event['accepted'] and event['sensor'] in ('camera', 'range', 'zero_velocity'):
                self.last_aid[event['sensor']] = event['stamp']
        self._check_reference(self.timeline.ekf.stamp)
        if self.timeline.fault:
            self.fail(self.timeline.fault)

    def range_ready(self, now):
        return (self.origin_ns is not None
                and 0 <= now-self.origin_ns/1e9-self.last_aid.get('range', -float('inf')) <= .5)

    def ready(self, now, estimate=None):
        if self.fault or self.timeline is None:
            return False
        if not self.reference_valid:
            self.reason = 'Local reference lost; restart on the floor before flight'
            return False
        current = now-self.origin_ns/1e9
        output_stamp = self.timeline.last_imu if estimate is None else estimate.stamp
        if not 0 <= current-output_stamp <= .15:
            self.reason = 'Waiting for fresh IMU'
            return False
        if not self.range_ready(now):
            self.reason = 'Waiting for accepted range'
            return False
        aiding = max(self.last_aid.get('camera', -float('inf')),
                     self.last_aid.get('zero_velocity', -float('inf')))
        if current-aiding > .5:
            self.reason = 'Coasting; waiting for accepted camera flow'
            return False
        variances = (estimate or self.timeline.ekf).body_navigation_variances()
        if (not np.all(np.isfinite(variances)) or np.any(variances < 0)
                or max(variances[:3]) > self.MAX_POSITION_STDDEV**2
                or max(variances[3:6]) > self.MAX_VELOCITY_STDDEV**2):
            self.reason = 'Position or velocity uncertainty is too large'
            return False
        self.reason = 'Ready'
        return True

    def output(self, queued_imu=()):
        """Project through received IMU samples without advancing delayed fusion."""
        if self.timeline is None or self.fault:
            return None
        intervals = list(self.timeline.imu)
        latest, rate = self.timeline.last_imu, self.rate
        for stamp_ns, accelerometer, gyroscope in queued_imu:
            stamp = (stamp_ns-self.origin_ns)/1e9
            if stamp <= latest:
                continue
            force, rate = self.imu_reading(accelerometer, gyroscope)
            intervals.append((stamp, force, rate))
            latest = stamp
        estimate = self.timeline.ekf.project(intervals)
        estimate.angular_rate = rate.copy()
        return estimate


def body_output(ekf, angular_rate):
    """Convert FLU navigation errors to the existing ROS RFU body convention."""
    rotation = quaternion_matrix(ekf.quaternion)
    flu_to_rfu = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    body_velocity = rotation.T.dot(ekf.velocity)
    q = quaternion_multiply(ekf.quaternion, [0., 0., -2**-.5, 2**-.5])
    pose_jacobian, twist_jacobian = np.zeros((6, 15)), np.zeros((6, 15))
    pose_jacobian[:3, :3], pose_jacobian[3:, 6:9] = np.eye(3), rotation
    twist_jacobian[:3, 3:6] = flu_to_rfu.dot(rotation.T)
    twist_jacobian[:3, 6:9] = flu_to_rfu.dot(skew(body_velocity))
    twist_jacobian[3:, 12:15] = -flu_to_rfu
    covariance = ekf.covariance
    pose_covariance = pose_jacobian.dot(covariance).dot(pose_jacobian.T)
    twist_covariance = twist_jacobian.dot(covariance).dot(twist_jacobian.T)
    # The rate is an IMU observation, rather than a separate estimated state.
    twist_covariance[3:, 3:] += np.eye(3)*ekf.noise[6, 6]/.01
    return (q, flu_to_rfu.dot(body_velocity), flu_to_rfu.dot(angular_rate-ekf.gyro_bias),
            pose_covariance, twist_covariance)
