"""Route-independent horizontal distance control. Compatible with Python 2.7."""
from __future__ import division

import math
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

STATE_TIMEOUT = 0.5


def automatic_coordination(geometry, settings, master_uri):
    """Identify our separate ROS master, never the Docker container hostname."""
    config = dict(geometry)
    config.update(settings)
    if config.get('enabled') is False:
        return config
    host = urlparse(master_uri).hostname
    identities = {'drone1': 'drone2', 'drone2': 'drone1'}
    # Only accept our known hostnames, including their mDNS form.
    own = host[:-6] if host and host.endswith('.local') else host
    if own not in identities:
        raise ValueError('Cannot identify drone from ROS_MASTER_URI; use drone1/drone2 '
                         'or explicitly set /pidrone/coordination identities')
    config.update(drone_id=own, peer_id=identities[own])
    return config


def finite(values):
    return all(not math.isnan(v) and not math.isinf(v) for v in values)


def fresh(stamp, now):
    return finite((stamp, now)) and stamp > 0 and 0 <= now - stamp <= STATE_TIMEOUT


def uncertainty(covariance):
    """Three-sigma largest horizontal covariance axis; not a total error bound."""
    xx, xy, yx, yy = (covariance[i] for i in (0, 1, 6, 7))
    if (not finite((xx, xy, yx, yy)) or xx < 0 or yy < 0 or
            abs(xy - yx) > 1e-6 or xx * yy < xy * xy - 1e-12):
        raise ValueError('invalid XY covariance')
    return 3 * math.sqrt((xx + yy + math.hypot(xx - yy, 2 * xy)) / 2)


class DistanceGuard(object):
    def __init__(self, minimum, maximum, nominal, gain, speed_limit, follow_peer=False):
        values = (minimum, maximum, nominal, gain, speed_limit)
        if not finite(values) or not 0 < minimum < nominal < maximum or gain <= 0 or speed_limit <= 0:
            raise ValueError('Require 0 < min_distance < nominal_separation_m < max_distance and positive gain/speed')
        self.minimum, self.maximum = minimum, maximum
        self.nominal, self.follow_peer = nominal, follow_peer
        self.gain, self.speed_limit = gain, speed_limit

    def filter(self, own, peer, peer_velocity, requested, margin):
        """Limit relative radial velocity; retain tangential route motion if feasible.

        The follower matches peer radial velocity plus a nominal-gap correction;
        the leader retains route radial velocity subject to the distance bounds.
        This velocity-level controller does not guarantee a physical hard bound.
        """
        values = tuple(own) + tuple(peer) + tuple(peer_velocity) + tuple(requested) + (margin,)
        if not finite(values) or margin < 0:
            return (0, 0), 'invalid_state'
        minimum, maximum = self.minimum + margin, self.maximum - margin
        if minimum >= maximum or not minimum <= self.nominal <= maximum:
            return (0, 0), 'uncertainty_too_large'
        dx, dy = own[0] - peer[0], own[1] - peer[1]
        distance = math.hypot(dx, dy)
        if not finite((dx, dy, distance)):
            return (0, 0), 'invalid_state'
        if distance < 1e-6:
            return (0, 0), 'coincident_positions'
        nx, ny = dx / distance, dy / distance
        peer_radial = peer_velocity[0] * nx + peer_velocity[1] * ny
        requested_radial = requested[0] * nx + requested[1] * ny
        lower = peer_radial + self.gain * (minimum - distance)
        upper = peer_radial + self.gain * (maximum - distance)
        preferred_radial = (peer_radial + self.gain * (self.nominal - distance)
                            if self.follow_peer else requested_radial)
        radial = max(lower, min(upper, preferred_radial))
        radial = max(-self.speed_limit, min(self.speed_limit, radial))
        tx, ty = requested[0] - requested_radial * nx, requested[1] - requested_radial * ny
        tangent = math.hypot(tx, ty)
        available = math.sqrt(max(0, self.speed_limit ** 2 - radial ** 2))
        scale = min(1, available / tangent) if tangent else 1
        command = (radial * nx + scale * tx, radial * ny + scale * ty)
        if not finite(command):
            return (0, 0), 'invalid_state'
        reason = ('too_close' if distance < minimum else 'too_far' if distance > maximum
                  else 'limited' if abs(radial - preferred_radial) > 1e-6 or scale < 1
                  else 'tracking_spacing' if math.hypot(command[0]-requested[0], command[1]-requested[1]) > 1e-6
                  else 'clear')
        return command, reason
