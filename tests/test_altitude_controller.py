"""Altitude cascade behavior without ROS.

Flight tests run the controller against an independent simulated drone whose
thrust curve, hover throttle and battery deliberately differ from the
controller's settings. Feedback is the simulated height and climb rate, with
optional noise to exercise the cascade independently of the estimator.
Expectations are basic yes/no flight requirements (lifts off, no throttle jolts,
no runaway, no climb while leaning), not the controller's formulas or tuned values.
The simulator exaggerates overshoot (no drag, ground effect or motor lag), so
overshoot and takeoff feel are judged in real flight testing.
"""
import math
import random
import unittest

from flight.altitude_controller import AltitudeController

G = 9.80665
RATE = 60.0


class SimulatedDrone(object):
    """Vertical rigid-body model with ground contact.

    Thrust follows its own curve (expo 0.5 rather than the controller's 0.65)
    and holds a hover at true_hover rather than the configured 1510.
    """

    def __init__(self, true_hover=1520.0, expo=0.5, sag_per_s=0.0, ground=0.05):
        self.expo, self.sag_per_s, self.ground = expo, sag_per_s, ground
        self.hover_thrust = self._thrust(true_hover)
        self.height, self.climb, self.time = ground, 0.0, 0.0

    def _thrust(self, throttle):
        a = max(0.0, (throttle - 1000.0) / 1000.0)
        return (1 - self.expo) * a + self.expo * a * a

    def step(self, throttle, tilt=0.0, dt=1 / RATE):
        battery = 1.0 - self.sag_per_s * self.time
        accel = G * self._thrust(throttle) / self.hover_thrust * battery * math.cos(tilt) - G
        if self.height <= self.ground and accel < 0:
            accel, self.climb = 0.0, 0.0
        self.climb += accel * dt
        self.height = max(self.ground, self.height + self.climb * dt)
        self.time += dt
        return accel


class FlightTest(object):
    """Controller driving a simulated drone with noisy state feedback."""

    def __init__(self, drone, controller=None, climb_noise=.02, seed=1):
        self.drone, self.rng = drone, random.Random(seed)
        self.controller = controller or AltitudeController()
        self.climb_noise = climb_noise
        self.commands, self.heights, self.climbs = [], [], []
        self.now = 10.0

    def run(self, seconds, target, tilt=0.0):
        for _ in range(int(seconds * RATE)):
            self.now += 1 / RATE
            height = self.drone.height + self.rng.gauss(0, .005)
            climb = self.drone.climb + self.rng.gauss(0, self.climb_noise)
            command = self.controller.step(target, height, climb, math.cos(tilt), self.now)
            self.drone.step(command, tilt)
            self.commands.append(command)
            self.heights.append(self.drone.height)
            self.climbs.append(self.drone.climb)
        return self

    def takeoff(self, target=0.25, seconds=6.0, **kwargs):
        self.controller.start_takeoff(self.drone.height, self.now)
        return self.run(seconds, target, **kwargs)


def largest_step(commands):
    return max(abs(b - a) for a, b in zip(commands, commands[1:]))


class AltitudeControllerTest(unittest.TestCase):
    def step(self, c, target=.25, height=.25, climb=0., tilt_cos=1., now=10.):
        return c.step(target, height, climb, tilt_cos, now)

    def test_hover_feed_forward_and_correction_directions(self):
        c = AltitudeController()
        self.assertAlmostEqual(self.step(c), 1510)
        self.assertGreater(self.step(AltitudeController(), height=.20), 1510)
        self.assertLess(self.step(AltitudeController(), height=.30), 1510)
        self.assertLess(self.step(AltitudeController(), climb=.2), 1510, 'brake a climb')

    def test_stuck_height_sensor_cannot_run_the_throttle_away(self):
        c = AltitudeController()
        c.start_takeoff(.05, 10.)
        for i in range(60*20):  # Snagged, or a ToF stuck at the ground reading.
            command = self.step(c, height=.05, now=10 + i/60.0)
        self.assertLess(command, 1510 + 50, 'throttle ran away to %.1f us' % command)

    def test_launch_throttle_ramp_is_independent_of_target_height(self):
        for target in (.10, .25, .75):
            with self.subTest(target=target):
                c = AltitudeController(hover_throttle=1540., rate_i=0.)
                c.start_takeoff(.05, 10.)
                commands = []
                for i in range(151):
                    elapsed = i / RATE
                    height = min(target, .05 + .1 * elapsed)
                    climb = .1 if height < target else 0.
                    commands.append(c.step(target, height, climb, 1., 10. + elapsed))
                self.assertAlmostEqual(commands[0], 1490.)
                self.assertAlmostEqual(commands[120], 1540.)
                self.assertAlmostEqual(commands[-1], 1540.)
                self.assertLess(largest_step(commands), 1., 'launch throttle jumped')
                self.assertAlmostEqual(c.target, min(target, .30))

    def test_no_integration_into_throttle_saturation(self):
        c = AltitudeController(throttle_range=(1200., 1520.), rate_i_limit=5.)
        for i in range(600):
            self.assertLessEqual(self.step(c, height=-1., now=10 + i/60.0), 1520)
        self.assertEqual(c.integral, 0)

    def test_invalid_settings_are_rejected(self):
        for settings in (dict(thrust_expo=1), dict(hover_throttle=1100), dict(rate_p=float('nan')),
                         dict(throttle_range=(1600, 1500))):
            with self.assertRaises(ValueError):
                AltitudeController(**settings)

    # Adversarial flight tests against the simulated drone.

    def test_takeoff_lifts_off_without_jolts_even_if_hover_throttle_is_off(self):
        for true_hover in (1490., 1510., 1530.):
            with self.subTest(true_hover=true_hover):
                flight = FlightTest(SimulatedDrone(true_hover=true_hover)).takeoff(seconds=15)
                reached = next((i / RATE for i, h in enumerate(flight.heights) if h >= .22), None)
                self.assertIsNotNone(reached, 'never lifted off; peak %.3f m' % max(flight.heights))
                self.assertLess(reached, 8.0, 'takeoff took %.1f s' % reached)
                self.assertLess(largest_step(flight.commands), 10, 'throttle jolt')
                self.assertLess(abs(flight.heights[-1] - .25), .03, 'did not settle at the target')

    def test_hover_holds_through_battery_sag_and_hover_mismatch(self):
        # 10 us hover mismatch plus 3% thrust loss per minute.
        flight = FlightTest(SimulatedDrone(true_hover=1520.0, sag_per_s=.03/60)).takeoff(seconds=15)
        flight.run(60, .25)
        last = flight.heights[-int(10 * RATE):]
        self.assertLess(max(abs(h - .25) for h in last), .03, 'sagged to %.3f m' % min(last))

    def test_leaning_never_climbs_and_at_most_sinks_gently(self):
        for expo in (.4, .8):  # The real thrust curve is unknown until measured.
            with self.subTest(true_expo=expo):
                flight = FlightTest(SimulatedDrone(expo=expo)).takeoff(seconds=15)
                start = len(flight.heights)
                flight.run(3, .25, tilt=math.radians(10))
                lean = flight.heights[start:]
                self.assertLess(max(lean), .25 + .02, 'extra thrust while leaning')
                self.assertGreater(min(lean), .25 - .10, 'fell while leaning')

    def test_landing_reaches_floor_and_respects_descent_limit(self):
        for true_hover in (1490., 1530.):
            flight = FlightTest(SimulatedDrone(true_hover=true_hover)).takeoff(seconds=15)
            start = len(flight.commands)
            flight.controller.start_landing(flight.drone.ground)
            flight.run(12, .25)
            self.assertLess(flight.heights[-1], flight.drone.ground + .01)
            self.assertGreater(min(flight.climbs[start:]), -.3)
            self.assertLess(largest_step(flight.commands[start - 1:]), 10)
        # A landing request during the takeoff ramp must not resume climbing.
        flight = FlightTest(SimulatedDrone())
        flight.controller.start_takeoff(flight.drone.height, flight.now)
        flight.run(.5, .25)
        flight.controller.start_landing(flight.drone.ground)
        flight.run(3, .25)
        self.assertLess(max(flight.heights), .06, 'lifted off after landing was requested')

    def test_climb_and_descent_speeds_respect_limits(self):
        flight = FlightTest(SimulatedDrone()).takeoff(seconds=12)
        flight.run(6, 1.2)
        self.assertLess(max(flight.climbs), .3 * 1.5, 'climbed at %.2f m/s' % max(flight.climbs))
        flight.run(12, .25)
        self.assertGreater(min(flight.climbs), -.2 * 1.5, 'descended at %.2f m/s' % -min(flight.climbs))
        self.assertLess(abs(flight.heights[-1] - .25), .03, 'did not settle back at the target')

    def test_climb_rate_noise_does_not_shake_the_throttle(self):
        flight = FlightTest(SimulatedDrone(), climb_noise=.08).takeoff(seconds=15)
        commands = flight.commands[-int(5 * RATE):]
        mean = sum(commands) / len(commands)
        spread = math.sqrt(sum((c - mean) ** 2 for c in commands) / len(commands))
        self.assertLess(spread, 10, 'throttle jitter %.1f us RMS' % spread)


if __name__ == '__main__':
    unittest.main()
