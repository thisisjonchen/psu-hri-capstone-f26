"""Tilted fisheye optical flow against an independently built camera simulation.

The simulator builds the camera pose from first principles (explicit roll/pitch/
yaw matrices, camera axes from cross products) and projects with OpenCV's own
fisheye model, using drone2's calibration. It does not use the module's frame
helpers. The motion-vector sign convention (a vector points to where the block's
content was in the previous frame) was confirmed on drone2 hand-carry data.
"""
import math
import unittest

import cv2
import numpy as np

from localization.apriltag_geometry import camera_rotation
from localization.camera_flow import CameraFlow, interpolate
from localization.test_fusion import quaternion

K = np.array([[154.1638736, 0, 118.8348587], [0, 155.2131826, 153.5541936], [0, 0, 1]])
D = np.array([-0.04846490008, 0.03309016118, -0.03411731679, 0.009736962306])
FPS = 30.0


def body_to_world(roll, pitch, yaw):
    """Forward/left/up body -> world, standard Z-Y-X Euler angles."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz.dot(ry).dot(rx)


def camera_in_body(pitch_down_deg=45.0):
    """OpenCV camera axes (right, down, optical) as columns, in forward/left/up body."""
    a = math.radians(pitch_down_deg)
    optical = np.array([math.cos(a), 0, -math.sin(a)])
    right = np.array([0., -1., 0.])
    return np.column_stack((right, np.cross(optical, right), optical))


def simulate(velocity_world, rates, attitude=(0., 0., 0.), height=.3, untracked=0., aliased=0.,
             wall=None, seed=0):
    """Integer motion vectors for one frame of camera motion over a flat floor.

    velocity_world: camera (x, y) m/s in the world. rates: roll/pitch/yaw rad/s.
    untracked: fraction of floor blocks the encoder misses (reported as zero);
    scenery (the wall) is always well textured and tracked, as on drone2.
    aliased: fraction locked onto a repeating floor line at a fixed wrong offset.
    wall: distance (m) of a wall ahead along world +x; rays beyond it see the wall.
    Returns (x list, y list, current quaternion, previous quaternion).
    """
    rng = np.random.RandomState(seed)
    now = np.array(attitude, dtype=float)
    before = now - np.array(rates, dtype=float) / FPS
    position_now = np.array([0., 0., height])
    position_before = position_now - np.array([velocity_world[0], velocity_world[1], 0.]) / FPS
    world_now = body_to_world(*now).dot(camera_in_body())
    world_before = body_to_world(*before).dot(camera_in_body())
    u, v = np.meshgrid(np.arange(15) * 16 + 8.0, np.arange(20) * 16 + 8.0)
    centers = np.column_stack((u.ravel(), v.ravel()))
    rays = cv2.fisheye.undistortPoints(centers.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    xs, ys = np.zeros(len(centers)), np.zeros(len(centers))
    scenery = np.zeros(len(centers), dtype=bool)
    for i, (ray, center) in enumerate(zip(rays, centers)):
        direction = world_now.dot([ray[0], ray[1], 1.])
        if direction[2] >= -1e-3:
            continue  # Sky: the encoder still reports something; leave zero here.
        point = position_now + direction * (position_now[2] / -direction[2])
        if wall is not None and point[0] > wall:  # Scenery, not floor, at this pixel.
            point = position_now + direction * (wall / direction[0])
            scenery[i] = True
        in_camera = world_before.T.dot(point - position_before)
        if in_camera[2] <= 0:
            continue
        pixel = cv2.fisheye.projectPoints(in_camera.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0]
        moved = pixel.reshape(2) - center
        xs[i], ys[i] = np.round(moved)
    tracked = (rng.rand(len(xs)) >= untracked) | scenery
    xs, ys = xs * tracked, ys * tracked
    locked = (rng.rand(len(xs)) < aliased) & tracked
    xs[locked], ys[locked] = xs[locked] + 6, ys[locked]  # One stripe period off, all the same way.
    padded = lambda a: np.column_stack((a.reshape(20, 15), np.zeros(20))).ravel()  # Encoder pad column.
    return (list(padded(xs)), list(padded(ys)), quaternion(*now), quaternion(*before))


def flow(**kwargs):
    return CameraFlow(K, D, 'equidistant', (240, 320), camera_rotation(45.0), frame_interval=1 / FPS, **kwargs)


def body_velocity(world, yaw):
    """World (x, y) velocity -> (right, forward) for a drone facing yaw."""
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    return np.array([world.dot((forward[1], -forward[0])), world.dot(forward)])


class CameraFlowTest(unittest.TestCase):
    def measure(self, velocity_world, rates=(0., 0., 0.), attitude=(0., 0., 0.), height=.3, **kwargs):
        x, y, q, previous = simulate(velocity_world, rates, attitude, height, **kwargs)
        return flow().velocity(x, y, 15, q, previous, height)

    def test_slides_read_true_velocity_at_any_heading_and_height(self):
        for world, yaw, height in [((.3, 0), 0, .3), ((0, .25), 0, .3), ((.2, -.2), 1.0, .5),
                                   ((-.3, .1), -2.5, .25)]:
            with self.subTest(world=world, yaw=yaw, height=height):
                (right, forward), used = self.measure(world, attitude=(0, 0, yaw), height=height)
                truth = body_velocity(np.array(world, dtype=float), yaw)
                self.assertGreater(used, 20)
                np.testing.assert_allclose((right, forward), truth, atol=.04)

    def test_pure_rotation_reads_no_motion(self):
        # A 90 deg/s turn produced the largest false reading before rotation removal.
        for rates in [(0, 0, math.radians(90)), (math.radians(60), 0, 0), (0, math.radians(60), 0),
                      (.6, -.5, 1.2)]:
            with self.subTest(rates=rates):
                (right, forward), _ = self.measure((0, 0), rates)
                self.assertLess(math.hypot(right, forward), .05)

    def test_turning_while_sliding_and_tilted_still_reads_the_slide(self):
        (right, forward), _ = self.measure((.25, 0), (0.1, -.1, math.radians(60)),
                                           attitude=(math.radians(8), math.radians(-6), 0))
        np.testing.assert_allclose((right, forward), (0, .25), atol=.05)

    def test_mostly_untracked_floor_and_aliased_stripes_do_not_bias_the_estimate(self):
        (right, forward), used = self.measure((0, .3), untracked=.7, aliased=.3, seed=4)
        np.testing.assert_allclose((right, forward), (-.3, 0), atol=.06)
        self.assertLess(used, 120)

    def test_well_tracked_scenery_does_not_outvote_a_poorly_tracked_floor(self):
        # As in the drone2 recording: the room near the horizon tracks better than
        # the floor. Its pixels must not be cast onto (distant) floor.
        (right, forward), _ = self.measure((.3, 0), wall=2.5, untracked=.7, seed=2)
        np.testing.assert_allclose((right, forward), (0, .3), atol=.05)

    def test_standstill_and_isolated_noise_read_zero(self):
        self.assertEqual(self.measure((0, 0))[0], (0.0, 0.0))
        x, y, q, previous = simulate((0, 0), (0, 0, 0))
        for i in (3, 40, 101, 250):  # A handful of spurious encoder vectors.
            x[i], y[i] = 4, -3
        self.assertEqual(flow().velocity(x, y, 15, q, q, .3)[0], (0.0, 0.0))

    def test_hover_vibration_jitter_in_distant_scenery_reads_no_motion(self):
        # Hovering: the floor reads zero, while vibration shakes the textured upper
        # image by one pixel. Far away, one pixel is a large distance on the floor.
        x, y, q, previous = simulate((0, 0), (0, 0, 0))
        rng = np.random.RandomState(1)
        for i in range(32, 64):  # Rows 2-3: floor 2-8 m away (the last of each 16 is padding).
            if i % 16 != 15:
                x[i], y[i] = rng.choice((-1, 1)), rng.choice((-1, 1))
        (right, forward), _ = flow().velocity(x, y, 15, q, q, .3)
        self.assertLess(math.hypot(right, forward), .05)

    def test_nose_up_drone_ignores_rays_above_the_horizon(self):
        # Pitched 35 deg nose up: much of the image sees the room, not the floor.
        (right, forward), used = self.measure((0, .2), attitude=(0, math.radians(-35), 0))
        np.testing.assert_allclose((right, forward), (-.2, 0), atol=.05)

    def test_vertically_mirrored_vectors_are_restored(self):
        x, y, q, previous = simulate((.15, .2), (0, 0, .5))
        mirror = lambda a, sign: list((sign * np.array(a).reshape(20, 16))[::-1].ravel())
        expected = flow().velocity(x, y, 15, q, previous, .3)
        restored = flow(flip_vertically=True).velocity(mirror(x, 1), mirror(y, -1), 15, q, previous, .3)
        np.testing.assert_allclose(restored[0], expected[0])

    def test_invalid_inputs_are_rejected(self):
        x, y, q, previous = simulate((0, 0), (0, 0, 0))
        self.assertIsNone(flow().velocity(x, y, 16, q, previous, .3)[0])
        self.assertIsNone(flow().velocity(x[:-1], y, 15, q, previous, .3)[0])
        self.assertIsNone(flow().velocity(x, y, 15, q, previous, 0)[0])
        with self.assertRaises(ValueError):
            CameraFlow(K, D, 'equidistant', (250, 320), camera_rotation(45.0))
        with self.assertRaises(ValueError):
            CameraFlow(K, D, 'unknown', (240, 320), camera_rotation(45.0))

    def test_attitude_interpolates_and_extrapolates_only_briefly(self):
        a, b = np.array(quaternion(yaw=0.)), np.array(quaternion(yaw=.2))
        history = [(10., a), (10.1, b)]
        np.testing.assert_allclose(interpolate(history, 10.05), quaternion(yaw=.1), atol=1e-3)
        np.testing.assert_allclose(interpolate(history, 10.15), quaternion(yaw=.3), atol=5e-3)
        self.assertIsNone(interpolate(history, 10.25))
        self.assertIsNone(interpolate(history, 9.9))


if __name__ == '__main__':
    unittest.main()
