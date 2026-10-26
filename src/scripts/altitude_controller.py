"""Altitude control in the PX4/ArduPilot cascade, independent of ROS.

height error -> P (+ takeoff climb-rate feed-forward) -> climb-rate target (limited)
-> PI on climb-rate error
-> vertical acceleration -> thrust relative to hover -> angle boost
-> throttle through the thrust curve (ArduPilot MOT_THST_EXPO).

Throttle is the Betaflight RC value (1000-2000 us); thrust is normalized 0-1.
"""
from __future__ import division

import math

GRAVITY = 9.80665


class AltitudeController(object):
    def __init__(self, hover_throttle=1510.0, takeoff_throttle=1490.0,
                 throttle_range=(1200.0, 1700.0), height_p=0.8, rate_p=1.43,
                 rate_i=0.71, rate_i_limit=0.71, climb_limit=0.3, descent_limit=0.2,
                 takeoff_rate=0.10, airborne_height=0.03, thrust_expo=0.65,
                 max_boost_tilt_deg=30.0):
        lower, upper = throttle_range
        values = (hover_throttle, takeoff_throttle, lower, upper, height_p, rate_p, rate_i_limit,
                  climb_limit, descent_limit, takeoff_rate, airborne_height, max_boost_tilt_deg)
        # math.isfinite is unavailable on the drone's Python 2.7.
        if (not all(not math.isnan(v) and not math.isinf(v) and v > 0 for v in values) or rate_i < 0
                or not 1000 < lower < min(hover_throttle, takeoff_throttle) <= max(hover_throttle, takeoff_throttle) < upper <= 2000
                or not 0 <= thrust_expo < 1 or not max_boost_tilt_deg < 90):
            raise ValueError('Invalid altitude controller settings')
        self.hover_throttle, self.takeoff_throttle = hover_throttle, takeoff_throttle
        self.throttle_range = throttle_range
        self.height_p, self.rate_p = height_p, rate_p
        self.rate_i, self.rate_i_limit = rate_i, rate_i_limit
        self.climb_limit, self.descent_limit = climb_limit, descent_limit
        self.takeoff_rate, self.airborne_height = takeoff_rate, airborne_height
        self.thrust_expo = thrust_expo
        self.min_boost_cos = math.cos(math.radians(max_boost_tilt_deg))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.time = None
        self.takeoff = None
        self.ground = None
        self.airborne = False
        self.target = None

    def start_takeoff(self, height, now):
        """Ramp the target up from the current height; freeze I until airborne."""
        self.reset()
        self.takeoff = (now, height)
        self.ground = height

    def cancel_takeoff(self):
        self.takeoff = None

    def thrust(self, throttle):
        """ArduPilot thrust curve: thrust = (1 - expo) * a + expo * a^2."""
        a = (throttle - 1000.0) / 1000.0
        return (1 - self.thrust_expo) * a + self.thrust_expo * a * a

    def throttle(self, thrust):
        e = self.thrust_expo
        a = thrust if e == 0 else ((e - 1) + math.sqrt((1 - e) ** 2 + 4 * e * max(0.0, thrust))) / (2 * e)
        return 1000.0 + 1000.0 * a

    def step(self, desired_height, height, climb_rate, tilt_cos, now):
        """Return the throttle command for the measured height and climb rate (m, m/s)."""
        dt = 0.0 if self.time is None else max(0.0, now - self.time)
        self.time = now
        if self.ground is not None and height >= self.ground + self.airborne_height:
            self.airborne = True
        hover = self.hover_throttle
        self.target = desired_height
        feed_forward = 0.0
        if self.takeoff is not None:
            started, start = self.takeoff
            self.target = min(desired_height, start + self.takeoff_rate * (now - started))
            progress = 1.0 if desired_height <= start else (self.target - start) / (desired_height - start)
            # Blend the takeoff feed-forward into hover so the climb ends without a step.
            hover = self.takeoff_throttle + (self.hover_throttle - self.takeoff_throttle) * max(0.0, progress)
            if self.target >= desired_height:
                self.takeoff = None
            else:
                feed_forward = self.takeoff_rate  # The ramp's own climb rate, as PX4 does.
        rate_target = max(-self.descent_limit, min(self.climb_limit,
                                                    feed_forward + self.height_p * (self.target - height)))
        rate_error = rate_target - climb_rate
        previous = self.integral
        if self.airborne or self.ground is None:
            self.integral = max(-self.rate_i_limit, min(self.rate_i_limit,
                                                        self.integral + self.rate_i * rate_error * dt))
        acceleration = self.rate_p * rate_error + self.integral
        thrust = self.thrust(hover) * (1 + acceleration / GRAVITY)
        # Angle boost: keep the vertical thrust component while leaning.
        thrust /= max(self.min_boost_cos, min(1.0, tilt_cos))
        command = self.throttle(thrust)
        lower, upper = self.throttle_range
        if (command > upper and rate_error > 0) or (command < lower and rate_error < 0):
            self.integral = previous  # Anti-windup: do not integrate into saturation.
        return max(lower, min(upper, command))
