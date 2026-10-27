"""Horizontal velocity control in the PX4/ArduPilot layout, independent of ROS.

velocity error (m/s) -> PI -> horizontal acceleration (m/s^2) -> lean angle atan(a / g), capped at max_lean_deg
-> Betaflight angle-mode RC value: 500 us of stick = angle_limit_deg of lean.

Axes are the controller body frame: x = right (roll), y = forward (pitch).
Positive roll/pitch commands lean right/forward. Both axes share one tuning.
Acceleration feed-forward belongs with a planned trajectory, not here.
"""
from __future__ import division

import math

GRAVITY = 9.80665


class VelocityController(object):
    def __init__(self, gain_p=1.2, gain_i=0.4, i_limit_deg=3.0, max_lean_deg=5.0,
                 angle_limit_deg=55.0):
        values = (gain_p, i_limit_deg, max_lean_deg, angle_limit_deg)
        # math.isfinite is unavailable on the drone's Python 2.7.
        if (not all(not math.isnan(v) and not math.isinf(v) and v > 0 for v in values)
                or not gain_i >= 0
                or not max_lean_deg < angle_limit_deg <= 90 or not i_limit_deg <= max_lean_deg):
            raise ValueError('Invalid velocity controller settings')
        self.gain_p, self.gain_i = gain_p, gain_i
        self.i_limit = GRAVITY * math.tan(math.radians(i_limit_deg))  # as acceleration
        self.max_lean = math.radians(max_lean_deg)
        self.us_per_radian = 500.0 / math.radians(angle_limit_deg)
        self.reset()

    def reset(self):
        self.integral = [0.0, 0.0]
        self.time = None
        self.lean = (0.0, 0.0)  # radians, for logging

    def step(self, error, now, integrate=True):
        """Return (roll, pitch) RC values for a velocity error (m/s)."""
        dt = 0.0 if self.time is None else max(0.0, now - self.time)
        self.time = now
        commands, leans = [], []
        for axis in (0, 1):
            previous = self.integral[axis]
            if integrate:
                self.integral[axis] = max(-self.i_limit, min(self.i_limit,
                                          previous + self.gain_i * error[axis] * dt))
            acceleration = self.gain_p * error[axis] + self.integral[axis]
            lean = math.atan2(acceleration, GRAVITY)
            if abs(lean) > self.max_lean:
                if lean * error[axis] > 0:
                    self.integral[axis] = previous  # Anti-windup: not into the lean limit.
                lean = math.copysign(self.max_lean, lean)
            leans.append(lean)
            commands.append(1500.0 + lean * self.us_per_radian)
        self.lean = tuple(leans)
        return commands
