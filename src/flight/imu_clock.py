"""Map the FC's wrapping sample clock without using USB jitter as integration time."""
from __future__ import division


class ImuClock(object):
    def __init__(self):
        self.end = self.stamp = None

    def map(self, end_us, sent, received):
        if not 0 <= end_us <= 0xffffffff or not 0 <= received-sent <= 1.:
            raise ValueError('Invalid IMU clock or request interval')
        if self.end is None:
            # Request time is a conservative lower bound on the FC snapshot time.
            stamp = sent
        else:
            elapsed = (end_us-self.end) & 0xffffffff
            if not 0 < elapsed <= 1000000:
                raise ValueError('FC clock restarted or IMU gap exceeded one second')
            stamp = self.stamp+elapsed*1.e-6
            # Follow FC clock drift, limiting each adjustment to 0.5% of the interval.
            slew = elapsed*5.e-9
            stamp += max(-slew,min(slew,sent-stamp))
            stamp = min(stamp,received)
            if stamp <= self.stamp:
                raise ValueError('IMU clock moved backwards')
        if not 0 <= received-stamp <= .2:
            raise ValueError('FC clock is no longer aligned with ROS time; restart localization and FC')
        self.end, self.stamp = end_us, stamp
        return stamp
