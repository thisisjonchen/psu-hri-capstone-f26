"""Buffer arriving measurements and fuse them in timestamp order.

IMU intervals are split at measurement times. Samples older than the completed
fusion horizon are rejected.
"""
from __future__ import division

import heapq
from collections import deque
import numpy as np

from localization.ekf import _array, _stamp


class DelayedEKF(object):
    def __init__(self, ekf, delay=.25, record_states=True):
        if ekf.stamp is None or not np.isfinite(delay) or not 0 <= delay <= 2:
            raise ValueError('Initialize the EKF and choose a delay in [0, 2] s')
        self.ekf, self.delay = ekf, delay
        self.record_states = record_states
        self.last_imu = ekf.stamp
        self.closed_through = ekf.stamp-1e-9
        self.imu, self.measurements = deque(), []
        self.angular_rates = deque()
        self.pending_keys = set()
        self.sequence = 0
        self.fault = None
        self.stationary_angle = np.zeros(3)
        self.stationary_duration = 0.
        self.was_stationary = False

    def add_imu(self, force, rate, stamp, stationary=False):
        """Queue an IMU interval and optional externally confirmed rest constraint."""
        if self.fault:
            return self.fault, []
        try:
            force, rate, stamp = _array(force, (3,)), _array(rate, (3,)), _stamp(stamp)
        except (ValueError, TypeError):
            return 'invalid_imu', []
        if stamp <= self.last_imu:
            return 'non_increasing_imu', []
        if stamp-self.last_imu > self.ekf.max_gap:
            self.fault = 'imu_gap'
            return self.fault, []
        dt = stamp-self.last_imu
        self.angular_rates.append((self.last_imu, stamp, rate))
        while self.angular_rates and self.angular_rates[0][1] < stamp-self.delay-.4:
            self.angular_rates.popleft()
        self.imu.append((stamp, force, rate))
        self.last_imu = stamp
        if stationary:
            self.add_measurement(stamp, 'zero_velocity',
                                 lambda ekf, t: ekf.correct_zero_velocity(t))
            if self.was_stationary:
                self.stationary_angle += rate*dt
                self.stationary_duration += dt
                if self.stationary_duration >= .2:
                    duration = self.stationary_duration
                    mean_rate = self.stationary_angle/duration
                    self.add_measurement(stamp, 'stationary_gyro',
                        lambda ekf, t, rate=mean_rate, dt=duration:
                            ekf.correct_stationary_gyro(rate, dt, t))
                    self.stationary_angle[:] = 0.
                    self.stationary_duration = 0.
        else:
            self.stationary_angle[:] = 0.
            self.stationary_duration = 0.
        self.was_stationary = stationary
        return 'queued', self._drain(stamp-self.delay)

    def mean_angular_rate(self, start, end):
        """Average the recorded IMU intervals covering a camera frame pair."""
        if (end <= start or not self.angular_rates
                or start < self.angular_rates[0][0] or end > self.angular_rates[-1][1]):
            return None
        angle = np.zeros(3)
        for first, last, rate in self.angular_rates:
            overlap = min(last, end)-max(first, start)
            if overlap > 0:
                angle += overlap*rate
            if last >= end:
                break
        return angle/(end-start)

    def add_measurement(self, stamp, sensor, update):
        """Queue update(ekf, stamp); the callback must retain its measurement data."""
        if self.fault:
            return self.fault
        try:
            stamp = _stamp(stamp)
        except (ValueError, TypeError):
            return 'invalid_stamp'
        if stamp < self.ekf.stamp or stamp <= self.closed_through:
            return 'measurement_too_late'
        if stamp > self.last_imu+max(self.delay, .1):
            return 'measurement_too_far_ahead'
        key = (sensor, stamp)
        if key in self.pending_keys:
            return 'repeated_measurement'
        if len(self.measurements) >= 512:
            return 'measurement_buffer_full'
        self.pending_keys.add(key)
        # Use a consistent order for measurements with the same timestamp.
        heapq.heappush(self.measurements, (stamp, sensor, self.sequence, update))
        self.sequence += 1
        return 'queued'

    def _drain(self, through):
        events = []
        while self.imu and not self.fault:
            end, force, rate = self.imu[0]
            aid_stamp = self.measurements[0][0] if self.measurements else float('inf')
            stamp = min(end, aid_stamp)
            if stamp > through:
                break
            if stamp > self.ekf.stamp:
                accepted, reason = self.ekf.predict(force, rate, stamp)
                if not accepted:
                    self.fault = reason
                    break
            if end == stamp:
                self.imu.popleft()
                if self.record_states:
                    events.append(self._event('imu', True, 'predicted'))
            while self.measurements and self.measurements[0][0] == stamp:
                _, sensor, _, update = heapq.heappop(self.measurements)
                self.pending_keys.remove((sensor, stamp))
                accepted, reason = update(self.ekf, stamp)
                events.append(self._event(sensor, accepted, reason))
        self.closed_through = max(self.closed_through, through)
        return events

    def finish(self):
        """Process remaining measurements through the final IMU timestamp."""
        events = self._drain(self.last_imu)
        remaining = len(self.measurements)
        self.measurements, self.pending_keys = [], set()
        return events, remaining

    def current_state(self):
        """Return an output at the newest IMU stamp, retaining the fusion horizon."""
        if self.fault:
            return None
        return self.ekf.project(list(self.imu))

    def _event(self, sensor, accepted, reason):
        f = self.ekf
        event = dict(stamp=f.stamp, sensor=sensor, accepted=accepted, reason=reason)
        if self.record_states:
            event.update(position=f.position.copy(), velocity=f.velocity.copy(), quaternion=f.quaternion.copy(),
                         acceleration_bias=f.acceleration_bias.copy(), gyro_bias=f.gyro_bias.copy(),
                         variances=f.covariance_diagonal(),
                         nis=None if sensor == 'imu' else (f.last_update or {}).get('nis'))
        return event
