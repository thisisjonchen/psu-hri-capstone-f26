"""Horizontal velocity controller against an independent simulated drone.

The simulator models Betaflight angle mode as a lagged lean response with a
different full-stick angle (65 deg) than the controller assumes (55 deg), air
drag, a constant push from a tilted IMU level, and a delayed, noisy velocity
estimate. Requirements are basic flight behavior, not the controller's formulas.
"""
import math
import random
import unittest
from collections import deque

from velocity_controller import VelocityController

G = 9.80665
RATE = 60.0


class SimulatedAxis(object):
    """One horizontal axis: lean follows the stick with a 0.1 s lag."""

    def __init__(self, push_deg=0.0, angle_limit_deg=65.0, drag=0.3):
        self.push = G * math.tan(math.radians(push_deg))
        self.angle_limit, self.drag = math.radians(angle_limit_deg), drag
        self.lean, self.velocity = 0.0, 0.0

    def step(self, command, dt=1 / RATE):
        target = (command - 1500.0) / 500.0 * self.angle_limit
        self.lean += (target - self.lean) * min(1.0, dt / 0.1)
        accel = G * math.tan(self.lean) + self.push - self.drag * self.velocity
        self.velocity += accel * dt


class Flight(object):
    def __init__(self, push_deg=(0.0, 0.0), delay=0.05, noise=0.03, controller=None, seed=1):
        self.axes = [SimulatedAxis(push_deg[0]), SimulatedAxis(push_deg[1])]
        self.controller = controller or VelocityController()
        self.rng, self.delay, self.noise = random.Random(seed), delay, noise
        self.history = deque()
        self.now, self.velocities, self.commands = 10.0, [], []

    def run(self, seconds, requested=(0.0, 0.0), integrate=True):
        for _ in range(int(seconds * RATE)):
            self.now += 1 / RATE
            self.history.append((self.now, [a.velocity for a in self.axes]))
            while self.history[0][0] < self.now - self.delay:
                self.history.popleft()
            measured = [v + self.rng.gauss(0, self.noise) for v in self.history[0][1]]
            error = [r - m for r, m in zip(requested, measured)]
            command = self.controller.step(error, self.now, integrate)
            for axis, c in zip(self.axes, command):
                axis.step(c)
            self.commands.append(command)
            self.velocities.append([a.velocity for a in self.axes])
        return self


def mean(values):
    return sum(values) / len(values)


class VelocityControllerTest(unittest.TestCase):
    def test_lean_direction_and_limit(self):
        c = VelocityController()
        roll, pitch = c.step((.2, .2), 10.)
        self.assertGreater(roll, 1500, 'right error leans right')
        self.assertGreater(pitch, 1500, 'forward error leans forward')
        limit = 500 * 5.0 / 55.0
        for error in ((10, -10), (-3, 4), (50, 50)):
            for value in VelocityController().step(error, 10.):
                self.assertLessEqual(abs(value - 1500), limit + 1e-9)

    def test_constant_push_is_cancelled_instead_of_drifting(self):
        # The drone2 symptom: steady drift in one direction from a constant push.
        flight = Flight(push_deg=(-1.5, 2.0)).run(25)
        late = flight.velocities[-int(5 * RATE):]
        for axis in (0, 1):
            self.assertLess(abs(mean([v[axis] for v in late])), .02, 'still drifting on axis %d' % axis)

    def test_push_beyond_authority_saturates_then_recovers_without_windup(self):
        flight = Flight(push_deg=(8.0, 0.0))
        flight.run(10)  # Needs more lean than the 5 deg limit allows.
        self.assertLessEqual(max(abs(c[0] - 1500) for c in flight.commands), 500 * 5 / 55. + 1e-9)
        flight.axes[0].push = 0.0
        start = len(flight.velocities)
        flight.run(8)
        after = [v[0] for v in flight.velocities[start:]]
        self.assertLess(abs(mean(after[-int(2 * RATE):])), .03, 'did not settle after the push stopped')
        self.assertGreater(min(after), -.4, 'wound-up integral flung it the other way')

    def test_requested_move_is_reached_without_oscillation(self):
        flight = Flight(push_deg=(1.0, -1.0), delay=.1).run(8, requested=(0.0, .2))
        late = [v[1] for v in flight.velocities[-int(3 * RATE):]]
        self.assertAlmostEqual(mean(late), .2, delta=.03)
        spread = math.sqrt(mean([(v - mean(late)) ** 2 for v in late]))
        self.assertLess(spread, .03, 'oscillating around the requested speed')

    def test_no_learning_without_a_fresh_estimate(self):
        flight = Flight(push_deg=(2.0, 2.0)).run(5, integrate=False)
        self.assertEqual(flight.controller.integral, [0.0, 0.0])

    def test_invalid_settings_are_rejected(self):
        for settings in (dict(max_lean_deg=0), dict(max_lean_deg=60), dict(gain_i=-1),
                         dict(i_limit_deg=6), dict(gain_p=float('nan'))):
            with self.assertRaises(ValueError):
                VelocityController(**settings)


if __name__ == '__main__':
    unittest.main()
