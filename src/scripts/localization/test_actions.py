"""Paired action blocks: plan params, shared durations and rejections, without ROS."""
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'actions.py')


class ActionsTest(unittest.TestCase):
    def setUp(self):
        coordination = ModuleType('coordination')
        coordination.Action, coordination.register = object, lambda cls: cls
        with mock.patch.dict(sys.modules, {'coordination': coordination}):
            self.actions = runpy.run_path(SCRIPT)
        self.drone = NS(mode='DISARMED', voltage=16., ALTITUDE_SETTLE_SECONDS=1., MIN_START_V=15.2,
                        takeoff_seconds=lambda height: 1 + (height - .05) / .1,
                        _check_battery=lambda: None, _hold_pose=lambda local=False: None)

    def test_takeoff_plan_adds_a_settle_pad_and_rejects_unsafe_starts(self):
        takeoff = self.actions['Takeoff']()
        params = takeoff.prepare(self.drone, [.5])
        self.assertAlmostEqual(params[1], 1 + 4.5 + 3)
        takeoff.validate(self.drone, params)
        self.assertEqual(takeoff.duration(params), params[1])
        stale = mock.Mock(side_effect=RuntimeError('Fused position is missing/stale'))
        for change, bad, reason in [
                ({}, [1.2, 20], 'height'), ({}, [.5, 6], 'needs'), ({'mode': 'FLYING'}, params, 'disarmed'),
                ({'voltage': 15.}, params, 'battery'), ({'_hold_pose': stale}, params, 'stale')]:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex((ValueError, RuntimeError), reason):
                    takeoff.validate(NS(**dict(vars(self.drone), **change)), bad)

    def test_land_plan_adds_a_pad_and_rejects_unsafe_starts(self):
        land = self.actions['Land']()
        drone = NS(mode='FLYING', landing_seconds=lambda: 4.)
        params = land.prepare(drone, [])
        self.assertEqual(params, [7.])
        land.validate(drone, params)
        self.assertEqual(land.duration(params), 7.)
        for change, bad, reason in [({}, [3.], 'needs'), ({'mode': 'DISARMED'}, params, 'in flight')]:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(ValueError, reason):
                    land.validate(NS(**dict(vars(drone), **change)), bad)


if __name__ == '__main__':
    unittest.main()
