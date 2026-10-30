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
    def __init__(self, ekf, delay=.25):
        if ekf.stamp is None or not np.isfinite(delay) or not 0 <= delay <= 2:
            raise ValueError('Initialize the EKF and choose a delay in [0, 2] s')
        self.ekf, self.delay = ekf, delay
        self.last_imu = ekf.stamp
        self.closed_through = ekf.stamp-1e-9
        self.imu, self.measurements = deque(), []
        self.pending_keys = set()
        self.sequence = 0
        self.fault = None

    def add_imu(self, force, rate, stamp):
        """Queue an IMU sample covering the preceding interval and process ready data."""
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
        self.imu.append((stamp, force, rate))
        self.last_imu = stamp
        return 'queued', self._drain(stamp-self.delay)

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

    def _event(self, sensor, accepted, reason):
        f = self.ekf
        return dict(stamp=f.stamp, sensor=sensor, accepted=accepted, reason=reason,
                    position=f.position.copy(), velocity=f.velocity.copy(), quaternion=f.quaternion.copy(),
                    acceleration_bias=f.acceleration_bias.copy(), gyro_bias=f.gyro_bias.copy(),
                    variances=np.diag(f.covariance).copy(),
                    nis=None if sensor == 'imu' else (f.last_update or {}).get('nis'))
