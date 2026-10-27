"""Altitude cascade behavior without ROS.

Flight tests run the controller against an independent simulated drone whose
thrust curve, hover throttle and battery deliberately differ from the
controller's settings, with delayed noisy 10 Hz ToF and a biased noisy IMU.
Expectations are basic yes/no flight requirements (lifts off, no throttle jolts,
no runaway, no climb while leaning), not the controller's formulas or tuned values.
The simulator exaggerates overshoot (no drag, ground effect or motor lag), so
overshoot and takeoff feel are judged in real flight testing.
"""
import math
import random
import unittest
from collections import deque

from altitude_controller import AltitudeController
from localization.fusion import VerticalFilter

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
    """Controller + vertical filter + sensors flying the simulated drone."""

    def __init__(self, drone, controller=None, imu_bias=0.2, imu_noise=0.3, seed=1):
        self.drone, self.rng = drone, random.Random(seed)
        self.controller = controller or AltitudeController()
        self.filter = VerticalFilter()
        self.imu_bias, self.imu_noise = imu_bias, imu_noise
        self.history = deque()  # (time, true height) for 50 ms ToF latency
        self.tof = drone.height
        self.accel, self.commands, self.heights, self.climbs = 0.0, [], [], []
        self.now = 10.0

    def run(self, seconds, target, tilt=0.0, on_step=None):
        for _ in range(int(seconds * RATE)):
            self.now += 1 / RATE
            self.filter.predict(self.accel + self.imu_bias + self.rng.gauss(0, self.imu_noise), self.now)
            self.history.append((self.now, self.drone.height))
            if int(round(self.now * RATE)) % 6 == 0:  # 10 Hz ToF
                stamp, height = [s for s in self.history if s[0] <= self.now - 0.05][-1]
                self.tof = height + self.rng.gauss(0, 0.01)
                self.filter.correct(self.tof, 0.02 ** 2, stamp)
            while self.history[0][0] < self.now - 1:
                self.history.popleft()
            if on_step:
                target = on_step(self, target)
            command = self.controller.step(target, self.tof, self.filter.climb_rate, math.cos(tilt), self.now)
            self.accel = self.drone.step(command, tilt)
            self.commands.append(command)
            self.heights.append(self.drone.height)
            self.climbs.append(self.drone.climb)
        return self

    def takeoff(self, target=0.25, seconds=6.0, **kwargs):
        self.controller.start_takeoff(self.tof, self.now)
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

    def test_land_requests_during_takeoff_have_no_throttle_jump(self):
        def land(at_height, target_height):
            def request(flight, target):
                if flight.drone.height >= at_height and target != target_height:
                    flight.controller.cancel_takeoff()  # As pid_controller does on a pose command.
                    return target_height
                return target
            return request
        flight = FlightTest(SimulatedDrone()).takeoff(seconds=10, on_step=land(.15, .10))
        self.assertLess(largest_step(flight.commands), 10)
        self.assertLess(abs(flight.heights[-1] - .10), .03, 'did not come back down to the request')
        # Landing requested before the drone has left the ground: it stays down.
        flight = FlightTest(SimulatedDrone())
        flight.controller.start_takeoff(flight.tof, flight.now)
        flight.run(.5, .25)
        flight.controller.cancel_takeoff()
        flight.run(3, .05)
        self.assertLess(max(flight.heights), .06, 'lifted off after landing was requested')

    def test_climb_and_descent_speeds_respect_limits(self):
        flight = FlightTest(SimulatedDrone()).takeoff(seconds=12)
        flight.run(6, 1.2)
        self.assertLess(max(flight.climbs), .3 * 1.5, 'climbed at %.2f m/s' % max(flight.climbs))
        flight.run(12, .25)
        self.assertGreater(min(flight.climbs), -.2 * 1.5, 'descended at %.2f m/s' % -min(flight.climbs))
        self.assertLess(abs(flight.heights[-1] - .25), .03, 'did not settle back at the target')

    def test_vibration_noise_does_not_shake_the_throttle(self):
        flight = FlightTest(SimulatedDrone(), imu_noise=1.5).takeoff(seconds=15)
        commands = flight.commands[-int(5 * RATE):]
        mean = sum(commands) / len(commands)
        spread = math.sqrt(sum((c - mean) ** 2 for c in commands) / len(commands))
        self.assertLess(spread, 10, 'throttle jitter %.1f us RMS' % spread)


if __name__ == '__main__':
    unittest.main()
