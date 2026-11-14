"""Height P and climb-rate PI control with a thrust curve and tilt compensation."""
from __future__ import division

import math

from flight.settings import load_flight_settings


class AltitudeController(object):
    def __init__(self, hover_throttle=1510.0, takeoff_throttle=1490.0,
                 throttle_range=(1200.0, 1700.0), height_p=0.8, rate_p=1.43,
                 rate_i=0.71, rate_i_limit=0.71, climb_limit=0.3, descent_limit=0.2,
                 takeoff_rate=0.10, thrust_expo=0.65, max_boost_tilt_deg=10.0,
                 takeoff_throttle_ramp_s=2.0, landing_rate=0.15, landing_final_rate=0.10,
                 landing_slow_height_m=0.15, flight_settings=None):
        lower, upper = throttle_range
        values = (hover_throttle, takeoff_throttle, lower, upper, height_p, rate_p, rate_i_limit,
                  climb_limit, descent_limit, takeoff_rate, max_boost_tilt_deg,
                  takeoff_throttle_ramp_s, landing_rate, landing_final_rate, landing_slow_height_m)
        if (not all(not math.isnan(v) and not math.isinf(v) and v > 0 for v in values)
                or not 0 <= rate_i < float('inf')
                or not 1000 < lower < min(hover_throttle, takeoff_throttle) <= max(hover_throttle, takeoff_throttle) < upper <= 2000
                or not 0 <= thrust_expo < 1 or not max_boost_tilt_deg < 90
                or landing_final_rate > landing_rate):
            raise ValueError('Invalid altitude controller settings')
        self.hover_throttle, self.takeoff_throttle = hover_throttle, takeoff_throttle
        self.throttle_range = throttle_range
        self.height_p, self.rate_p = height_p, rate_p
        self.rate_i, self.rate_i_limit = rate_i, rate_i_limit
        self.climb_limit, self.descent_limit = climb_limit, descent_limit
        self.takeoff_rate = takeoff_rate
        self.takeoff_throttle_ramp_s = takeoff_throttle_ramp_s
        self.landing_rate, self.landing_final_rate = landing_rate, landing_final_rate
        self.landing_slow_height_m = landing_slow_height_m
        self.thrust_expo = thrust_expo
        self.min_boost_cos = math.cos(math.radians(max_boost_tilt_deg))
        flight = load_flight_settings() if flight_settings is None else flight_settings
        self.gravity = flight['gravity_m_s2']
        self.max_floor_height = flight['max_floor_height_m']
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.time = None
        self.takeoff = None
        self.landing_ground = None
        self.target = None

    def start_takeoff(self, height, now):
        """Ramp the target up from the current height."""
        self.reset()
        self.takeoff = (now, height)

    def cancel_takeoff(self):
        self.takeoff = None

    def start_landing(self, ground_height):
        if not 0 <= ground_height <= self.max_floor_height:
            raise ValueError('Landing requires a valid preflight floor height')
        self.takeoff = None
        self.landing_ground = ground_height

    def cancel_landing(self):
        self.landing_ground = None

    def thrust(self, throttle):
        """Approximate thrust from RC throttle; Betaflight also scales and mixes it."""
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
        hover = self.hover_throttle
        self.target = desired_height
        feed_forward = 0.0
        if self.takeoff is not None:
            started, start = self.takeoff
            elapsed = max(0.0, now - started)
            self.target = min(desired_height, start + self.takeoff_rate * elapsed)
            progress = min(1.0, elapsed / self.takeoff_throttle_ramp_s)
            hover = self.takeoff_throttle + (self.hover_throttle - self.takeoff_throttle) * progress
            if self.target < desired_height:
                feed_forward = self.takeoff_rate
            elif progress >= 1.0:
                self.takeoff = None
        rate_target = max(-self.descent_limit, min(self.climb_limit,
                                                    feed_forward + self.height_p * (self.target - height)))
        if self.landing_ground is not None:
            self.target = self.landing_ground
            descent = (self.landing_final_rate if height - self.landing_ground <= self.landing_slow_height_m
                       else self.landing_rate)
            rate_target = -min(self.descent_limit, descent)
        rate_error = rate_target - climb_rate
        previous = self.integral
        self.integral = max(-self.rate_i_limit, min(self.rate_i_limit,
                                                    self.integral + self.rate_i * rate_error * dt))
        acceleration = self.rate_p * rate_error + self.integral
        command = self.throttle(self.thrust(hover) * (1 + acceleration / self.gravity))
        # Tilt compensation assumes thrust is proportional to squared throttle above idle.
        command = 1000.0 + (command - 1000.0) / math.sqrt(max(self.min_boost_cos, min(1.0, tilt_cos)))
        lower, upper = self.throttle_range
        if (command > upper and rate_error > 0) or (command < lower and rate_error < 0):
            self.integral = previous  # Anti-windup: do not integrate into saturation.
        return max(lower, min(upper, command))
