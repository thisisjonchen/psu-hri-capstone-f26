"""Optical-flow velocity from a tilted camera's encoder motion vectors, without ROS.

Each 16x16 macroblock's motion vector says where its image came from in the
previous frame. Its pixel at both frames is cast onto the floor plane using the
calibrated lens, the camera mounting and the drone attitude *at each frame*, so
roll, pitch and yaw rotation between frames cancel exactly. The floor is static,
so the camera moved by the difference of the two floor points. Blocks whose ray
misses nearby floor (near the horizon) are ignored.

Frames: camera mounting and body follow localization.apriltag_geometry
(controller body +X right, +Y forward, +Z up). Velocity is returned in the
level body heading frame: (right, forward) m/s.
"""
from __future__ import division

import bisect

import cv2
import numpy as np

from localization.apriltag_geometry import quaternion_matrix

# Body right/forward/up -> the flight controller's forward/left/up quaternion frame.
BODY_TO_FC = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def undistort(pixels, camera_matrix, distortion, model):
    """Normalized camera rays [x, y, 1] for (N, 2) pixels."""
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    d = np.asarray(distortion, dtype=float).reshape(-1)
    points = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
    if model == 'equidistant':
        if not hasattr(cv2, 'fisheye') or not hasattr(cv2.fisheye, 'undistortPoints'):
            raise ValueError('OpenCV fisheye.undistortPoints is required for this lens')
        xy = cv2.fisheye.undistortPoints(points, k, d)
    elif model in ('plumb_bob', 'rational_polynomial'):
        xy = cv2.undistortPoints(points, k, d)
    else:
        raise ValueError('Unsupported distortion model: {}'.format(model))
    xy = xy.reshape(-1, 2)
    return np.column_stack((xy, np.ones(len(xy))))


def interpolate(history, stamp, max_extrapolation=0.1):
    """Attitude quaternion at stamp from (stamp, quaternion) samples, or None.

    Frames often arrive just before the IMU sample covering them, so continue
    the last rotation briefly rather than drop the frame or freeze the attitude.
    """
    stamps = [s for s, _ in history]
    i = bisect.bisect_left(stamps, stamp)
    if i == len(stamps) and len(stamps) >= 2 and stamp - stamps[-1] <= max_extrapolation:
        i = len(stamps) - 1
    if i == 0 or i == len(stamps):
        return None
    (t0, q0), (t1, q1) = history[i - 1], history[i]
    q1 = q1 if np.dot(q0, q1) >= 0 else -q1
    q = q0 + (q1 - q0) * (stamp - t0) / (t1 - t0)
    return q / np.linalg.norm(q)


class CameraFlow(object):
    def __init__(self, camera_matrix, distortion, model, image_size, rotation_body_camera,
                 frame_interval=1 / 30.0, max_floor_distance=1.5, min_blocks=8, gain=1.0,
                 flip_vertically=False):
        width, height = image_size
        if (width % 16 or height % 16 or not 0 < frame_interval < 1 or max_floor_distance <= 0
                or min_blocks < 1 or not gain > 0):
            raise ValueError('Invalid camera flow settings')
        self.columns, self.rows = width // 16, height // 16
        self.camera = (camera_matrix, distortion, model)
        self.rotation_body_camera = np.asarray(rotation_body_camera, dtype=float)
        self.frame_interval, self.max_floor_distance = frame_interval, max_floor_distance
        self.min_blocks, self.gain = min_blocks, gain
        # The Pi encoder's vector rows can be stored bottom-up relative to the image.
        self.flip_vertically = flip_vertically
        u, v = np.meshgrid(np.arange(self.columns) * 16 + 8.0, np.arange(self.rows) * 16 + 8.0)
        self.centers = np.column_stack((u.ravel(), v.ravel()))
        self.center_rays = undistort(self.centers, *self.camera)

    def _floor(self, rays, quaternion, height):
        """Floor points (world x, y) relative to the camera, and which rays reach nearby floor."""
        body_to_world = quaternion_matrix(quaternion).dot(BODY_TO_FC)
        world = rays.dot((body_to_world.dot(self.rotation_body_camera)).T)
        down = world[:, 2] < -1e-3
        scale = np.where(down, height / np.where(down, -world[:, 2], 1.0), 0.0)
        points = world[:, :2] * scale[:, None]
        reach = down & (np.linalg.norm(points, axis=1) <= self.max_floor_distance)
        return points, reach, body_to_world

    def velocity(self, vectors_x, vectors_y, mbx, quaternion, previous_quaternion, height):
        """Return ((right, forward) m/s, blocks used), or (None, reason).

        vectors_x/y: encoder rows of mbx + 1 values (the last column is padding).
        quaternion/previous_quaternion: flight-controller attitude (x, y, z, w) at
        this frame and the previous one. height: camera above the floor (m).
        """
        x = np.asarray(vectors_x, dtype=float)
        y = np.asarray(vectors_y, dtype=float)
        if (mbx != self.columns or x.shape != y.shape
                or x.size != (mbx + 1) * self.rows or not height > 0):
            return None, 'Motion vectors do not match the calibrated image size'
        x = x.reshape(self.rows, mbx + 1)[:, :mbx]
        y = y.reshape(self.rows, mbx + 1)[:, :mbx]
        if self.flip_vertically:
            x, y = x[::-1], -y[::-1]
        x, y = x.ravel(), y.ravel()
        here, reach, body_to_world = self._floor(self.center_rays, quaternion, height)
        tracked = reach & ((x != 0) | (y != 0))
        if np.count_nonzero(reach) < self.min_blocks:
            return None, 'Too few blocks see nearby floor'
        if np.count_nonzero(tracked) < self.min_blocks:
            return (0.0, 0.0), 0  # Below one pixel per frame: no measurable motion.
        # A vector points from the block to where its content was in the previous frame.
        before_rays = undistort(self.centers[tracked] + np.column_stack((x, y))[tracked], *self.camera)
        before, before_reach, _ = self._floor(before_rays, previous_quaternion, height)
        if np.count_nonzero(before_reach) < self.min_blocks:
            return None, 'Too few tracked blocks see nearby floor'
        moved = (before - here[tracked])[before_reach] / self.frame_interval
        world = np.median(moved, axis=0) * self.gain
        forward = body_to_world[:2, 1] / max(1e-9, np.linalg.norm(body_to_world[:2, 1]))
        right = np.array([forward[1], -forward[0]])
        return (float(world.dot(right)), float(world.dot(forward))), int(np.count_nonzero(before_reach))
