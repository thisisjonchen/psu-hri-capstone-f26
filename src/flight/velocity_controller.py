"""Velocity PI for Betaflight angle mode. Body X is right and Y is forward."""
from __future__ import division

import math

from flight.settings import load_flight_settings


class VelocityController(object):
    def __init__(self, gain_p=1.2, gain_i=0.4, i_limit_deg=3.0, max_lean_deg=5.0,
                 angle_limit_deg=60.0, flight_settings=None):
        values = (gain_p, i_limit_deg, max_lean_deg, angle_limit_deg)
        if (not all(not math.isnan(v) and not math.isinf(v) and v > 0 for v in values)
                or not 0 <= gain_i < float('inf')
                or not max_lean_deg < angle_limit_deg <= 90 or not i_limit_deg <= max_lean_deg):
            raise ValueError('Invalid velocity controller settings')
        self.gain_p, self.gain_i = gain_p, gain_i
        flight = load_flight_settings() if flight_settings is None else flight_settings
        self.gravity = flight['gravity_m_s2']
        self.i_limit = self.gravity * math.tan(math.radians(i_limit_deg))  # as acceleration
        self.max_lean = math.radians(max_lean_deg)
        self.us_per_radian = 500.0 / math.radians(angle_limit_deg)
        self.reset()

    def reset(self):
        self.integral = [0.0, 0.0]
        self.time = None

    def step(self, error, now, integrate=True):
        """Return (roll, pitch) RC values for a velocity error (m/s)."""
        dt = 0.0 if self.time is None else max(0.0, now - self.time)
        self.time = now
        commands = []
        for axis in (0, 1):
            previous = self.integral[axis]
            if integrate:
                self.integral[axis] = max(-self.i_limit, min(self.i_limit,
                                          previous + self.gain_i * error[axis] * dt))
            acceleration = self.gain_p * error[axis] + self.integral[axis]
            lean = math.atan2(acceleration, self.gravity)
            if abs(lean) > self.max_lean:
                if lean * error[axis] > 0:
                    self.integral[axis] = previous  # Anti-windup: not into the lean limit.
                lean = math.copysign(self.max_lean, lean)
            commands.append(1500.0 + lean * self.us_per_radian)
        return commands
