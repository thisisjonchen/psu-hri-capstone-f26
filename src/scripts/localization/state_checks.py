"""Freshness and covariance checks shared by the peer relay and preflight. Python 2.7 compatible."""
from __future__ import division

import math

STATE_TIMEOUT = 0.5
# NTP-synced drone and laptop clocks disagree by tens of milliseconds over Wi-Fi.
CLOCK_TOLERANCE = 0.1


def finite(values):
    return all(not math.isnan(v) and not math.isinf(v) for v in values)


def fresh(stamp, now):
    return finite((stamp, now)) and stamp > 0 and -CLOCK_TOLERANCE <= now - stamp <= STATE_TIMEOUT


def uncertainty(covariance):
    """One-sigma largest horizontal covariance axis; not a total error bound."""
    xx, xy, yx, yy = (covariance[i] for i in (0, 1, 6, 7))
    if (not finite((xx, xy, yx, yy)) or xx < 0 or yy < 0 or
            abs(xy - yx) > 1e-6 or xx * yy < xy * xy - 1e-12):
        raise ValueError('invalid XY covariance')
    return math.sqrt((xx + yy + math.hypot(xx - yy, 2 * xy)) / 2)
