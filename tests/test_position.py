import math
import unittest

from localization.position import PositionCombiner, solve_uwb_xy
from localization.uwb_locator import Anchor, AnchorSample


def anchor_ranges(tag=(1., 2., .5)):
    coords = [(0., 0., 0.), (3., 0., .1), (3., 4., .2), (0., 4., 0.)]
    return tuple(Anchor(str(i), x, y, z, math.dist((x, y, z), tag))
                 for i, (x, y, z) in enumerate(coords))


class PositionTest(unittest.TestCase):
    def setUp(self):
        self.combiner = PositionCombiner()
        self.sample = AnchorSample(anchor_ranges(), 10)
        self.combiner.update_range(0.5, 0.05, 4, 100, 10)
        self.combiner.update_attitude((0, 0, 0, 1), 100, 10)

    def combine(self, now=10, ros_now=100):
        return self.combiner.combine(self.sample, now, ros_now)

    def test_floor_offset_and_body_sensor_offset_with_tilt(self):
        beam_tilt = math.radians(10)
        self.combiner = PositionCombiner(
            floor_z=.2, tof_offset_z=.1, uwb_offset_body=(0., .038, 0.),
            range_offset_body=(0., .06, .02),
            range_beam_body=(0., math.sin(beam_tilt), -math.cos(beam_tilt)))
        self.combiner.update_range(.8, .05, 4, 100, 10)
        angle = math.radians(30)
        self.combiner.update_attitude((math.sin(angle / 2), 0, 0, math.cos(angle / 2)), 100, 10)
        height = .2 - .06 * math.sin(angle) - .12 * math.cos(angle) + .8 * math.cos(angle + beam_tilt)
        self.sample = self.sample._replace(anchors=anchor_ranges((1., 2., height + .038 * math.sin(angle))))
        fix, _ = self.combine()
        self.assertAlmostEqual(fix.z, height)
        self.assertAlmostEqual(fix.x, 1.)
        self.assertAlmostEqual(fix.y, 2.)

    def test_range_fit_rejects_missing_degenerate_or_inconsistent_ranges(self):
        anchors = self.sample.anchors
        outlier = anchors[0]._replace(distance=anchors[0].distance + 1.22)
        for invalid in (anchors[1:], (outlier,) + anchors[1:],
                        tuple(a._replace(y=0.) for a in anchors),
                        (anchors[0]._replace(distance=0.),) + anchors[1:]):
            self.assertIsNone(solve_uwb_xy(invalid, .5, .10))

    def test_invalid_tof_immediately_withdraws_previous_height(self):
        for value in (float('nan'), float('inf'), 8.0, -1.0):
            self.combiner.update_range(value, 0.05, 4, 100, 10)
            self.assertIsNone(self.combine()[0])

    def test_each_sensor_expiry_and_future_stamps_are_rejected(self):
        self.assertIsNone(self.combine(now=10.6, ros_now=100.6)[0])
        self.sample = self.sample._replace(timestamp=10.1)
        self.assertIsNone(self.combine()[0])
        self.sample = self.sample._replace(timestamp=10)
        self.combiner.update_range(0.5, 0.05, 4, 101, 10)
        self.assertIsNone(self.combine()[0])
        self.combiner.update_range(0.5, 0.05, 4, 100, 9)
        self.assertIsNone(self.combine()[0])
        self.combiner.update_range(0.5, 0.05, 4, 100, 10)
        self.combiner.update_attitude((0, 0, 0, 1), 99, 10)
        self.assertIsNone(self.combine()[0])

    def test_height_uses_real_sensor_timing_without_a_uwb_sample(self):
        height, reason = self.combiner.height(10.3, 100.3)
        self.assertEqual(reason, 'OK')
        self.assertEqual((height.z, height.stamp), (.5, 100))
        self.assertIsNone(self.combiner.height(10.6, 100.6)[0])
        self.combiner.update_range(.5, .05, 4, 99.7, 10)
        self.assertEqual(self.combiner.height(10, 100)[1], 'Sensor timestamps are not aligned')

    def test_invalid_unavailable_or_excessively_tilted_imu_is_rejected(self):
        for quaternion, available in [((0, 0, 0, 0), True), ((0, 0, 0, 1), False),
                                       ((float('nan'), 0, 0, 1), True), ((1, 0, 0, 0), True),
                                       ((0.5, 0, 0, math.sqrt(0.75)), True)]:
            self.combiner.update_attitude(quaternion, 100, 10, available)
            self.assertIsNone(self.combine()[0])

    def test_raw_tof_mode_explicitly_does_not_require_imu(self):
        self.combiner.compensate_tilt = False
        self.combiner.attitude = None
        self.assertEqual(self.combine()[0].z, 0.5)


if __name__ == '__main__':
    unittest.main()
