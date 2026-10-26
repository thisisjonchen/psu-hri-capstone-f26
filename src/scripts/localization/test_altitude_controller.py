"""Altitude cascade behavior without ROS."""
import math
import unittest

from altitude_controller import AltitudeController


class AltitudeControllerTest(unittest.TestCase):
    def step(self, c, target=.25, height=.25, climb=0., tilt_cos=1., now=10.):
        return c.step(target, height, climb, tilt_cos, now)

    def test_hover_feed_forward_and_correction_directions(self):
        c = AltitudeController()
        self.assertAlmostEqual(self.step(c), 1510)
        self.assertGreater(self.step(AltitudeController(), height=.20), 1510)
        self.assertLess(self.step(AltitudeController(), height=.30), 1510)
        self.assertLess(self.step(AltitudeController(), climb=.2), 1510, 'brake a climb')
        # Large errors are limited to the climb/descent speeds.
        self.assertAlmostEqual(self.step(AltitudeController(), height=-1.),
                               self.step(AltitudeController(), height=-2.))
        self.assertGreater(self.step(AltitudeController(), height=-1.) - 1510,
                           1510 - self.step(AltitudeController(), height=2.), 'descent limit is lower')

    def test_takeoff_blends_into_hover_without_a_step(self):
        c = AltitudeController()
        c.start_takeoff(.05, 10.)
        commands, t = [], 10.
        while c.takeoff is not None or len(commands) < 5:
            # The drone follows the 0.10 m/s ramp exactly.
            height = min(.25, .05 + .10*(t - 10.))
            commands.append(self.step(c, height=height, climb=.10 if height < .25 else 0., now=t))
            t += 1/60.0
        self.assertAlmostEqual(commands[0], 1490, delta=1)  # Climb rate matches the ramp.
        self.assertLess(max(abs(b - a) for a, b in zip(commands, commands[1:])), 6,
                        'the old controller jumped 20 us when the climb ended')
        self.assertAlmostEqual(commands[-1], 1510, delta=1)

    def test_integral_frozen_on_ground_then_limited_to_about_25_us(self):
        c = AltitudeController()
        c.start_takeoff(.05, 10.)
        for i in range(120):  # Stuck on the ground for two seconds.
            self.step(c, height=.05, now=10 + i/60.0)
        self.assertEqual(c.integral, 0)
        for i in range(120, 1200):  # Airborne, sagging 15 cm below target.
            self.step(c, height=.10, now=10 + i/60.0)
        self.assertAlmostEqual(c.integral, .71)
        level = AltitudeController()
        self.assertAlmostEqual(c.throttle(c.thrust(1510) * (1 + .71/9.80665)) - 1510, 25, delta=1)
        self.assertGreater(self.step(c, height=.25, now=40), self.step(level, height=.25, now=40))

    def test_no_integration_into_throttle_saturation(self):
        c = AltitudeController(throttle_range=(1200., 1520.), rate_i_limit=5.)
        for i in range(600):
            self.assertLessEqual(self.step(c, height=-1., now=10 + i/60.0), 1520)
        self.assertEqual(c.integral, 0)

    def test_angle_boost_keeps_vertical_thrust_and_is_capped(self):
        c = AltitudeController()
        hover = c.thrust(1510)
        for degrees in (10, 20, 30):
            command = self.step(AltitudeController(), tilt_cos=math.cos(math.radians(degrees)))
            self.assertGreater(command, 1510)
            self.assertAlmostEqual(c.thrust(command) * math.cos(math.radians(degrees)), hover)
        self.assertAlmostEqual(self.step(AltitudeController(), tilt_cos=math.cos(math.radians(60))),
                               self.step(AltitudeController(), tilt_cos=math.cos(math.radians(30))))

    def test_thrust_curve_round_trip_and_invalid_settings(self):
        for expo in (0, .65):
            c = AltitudeController(thrust_expo=expo)
            for throttle in (1200, 1510, 1700):
                self.assertAlmostEqual(c.throttle(c.thrust(throttle)), throttle)
        for settings in (dict(thrust_expo=1), dict(hover_throttle=1100), dict(rate_p=float('nan')),
                         dict(throttle_range=(1600, 1500))):
            with self.assertRaises(ValueError):
                AltitudeController(**settings)


if __name__ == '__main__':
    unittest.main()
