"""Map a floor AprilTag center into the UWB room from a calibrated camera ray.

Coordinate frames: map (+Z up); drone controller body (+X right, +Y forward,
+Z up); OpenCV optical camera (+X image right, +Y image down, +Z forward).
Only static images of tags lying on a known horizontal plane are supported.
This module has no ROS dependency and works with Python 2.7 and older NumPy.
"""
from __future__ import division

import math

import cv2
import numpy as np

from localization.rotations import quaternion_matrix


def camera_rotation(pitch_down_deg, azimuth_deg=0.0, image_roll_deg=0.0):
    """Camera optical axes expressed in controller body right/forward/up.

    pitch_down=90 points vertically at the floor; 0 points horizontally.
    azimuth rotates viewing direction around body +Z; 0 looks forward.
    image_roll rotates sensor X/Y axes around its optical Z axis.
    """
    angles = (pitch_down_deg, azimuth_deg, image_roll_deg)
    if not all(np.isfinite(v) for v in angles) or not 0 < pitch_down_deg <= 90:
        raise ValueError('Invalid camera mounting angles')
    a = math.radians(pitch_down_deg)
    c, s = math.cos(a), math.sin(a)
    # Columns: optical image-right, optical image-down, optical forward.
    base = np.array([[1, 0, 0], [0, -s, c], [0, -c, -s]], dtype=float)
    theta = math.radians(azimuth_deg)
    cy, sy = math.cos(theta), math.sin(theta)
    yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    roll = math.radians(image_roll_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    in_image = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return np.dot(np.dot(yaw, base), in_image)


def project_floor_tag(pixel, camera_matrix, distortion, drone_position,
                      drone_quaternion, camera_offset_body, rotation_body_camera,
                      plane_z=0.0, max_distance=5.0, distortion_model='plumb_bob'):
    """Return [map_x, map_y, plane_z], or raise ValueError for invalid ray.

    pixel: detected tag center in the *unrectified* compressed camera image.
    camera_matrix/distortion: matching ROS CameraInfo intrinsics for that image.
    distortion_model: ROS pinhole model or equidistant (four fisheye coefficients).
    drone_position: UWB/ToF fused position (origin is the UWB measurement point).
    camera_offset_body: UWB point -> optical center in controller right/forward/up.
    rotation_body_camera: camera optical vectors -> controller body vectors.
    """
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    d = np.asarray(distortion, dtype=float).reshape(-1)
    uv = np.asarray(pixel, dtype=float).reshape(1, 1, 2)
    origin = np.asarray(drone_position, dtype=float).reshape(3)
    offset = np.asarray(camera_offset_body, dtype=float).reshape(3)
    rotate_camera = np.asarray(rotation_body_camera, dtype=float).reshape(3, 3)
    if (not all(np.all(np.isfinite(v)) for v in (k, d, uv, origin, offset, rotate_camera))
            or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.isfinite(plane_z)
            or not np.isfinite(max_distance) or max_distance <= 0):
        raise ValueError('Invalid intrinsics or pose')
    try:
        if distortion_model == 'equidistant':
            if d.size != 4:
                raise ValueError('Equidistant calibration requires four coefficients')
            if not hasattr(cv2, 'fisheye') or not hasattr(cv2.fisheye, 'undistortPoints'):
                raise ValueError('OpenCV fisheye.undistortPoints is required for equidistant calibration')
            ray_xy = cv2.fisheye.undistortPoints(uv, k, d).reshape(2)
        elif distortion_model in ('plumb_bob', 'rational_polynomial'):
            ray_xy = cv2.undistortPoints(uv, k, d).reshape(2)
        else:
            raise ValueError('Unsupported distortion model: {}'.format(distortion_model))
    except cv2.error as exc:
        raise ValueError('Cannot undistort tag: {}'.format(exc))
    ray_camera = np.array([ray_xy[0], ray_xy[1], 1.0])
    body_to_map = quaternion_matrix(drone_quaternion)
    ray_map = np.dot(np.dot(body_to_map, rotate_camera), ray_camera)
    camera_map = origin + np.dot(body_to_map, offset)
    if ray_map[2] >= -1e-4 or camera_map[2] <= plane_z:
        raise ValueError('Camera ray does not point down to the tag plane')
    distance_scale = (plane_z - camera_map[2]) / ray_map[2]
    distance = distance_scale * np.linalg.norm(ray_map)
    if distance_scale <= 0 or distance > max_distance:
        raise ValueError('Tag plane intersection is too far away')
    intersection = camera_map + distance_scale * ray_map
    if not np.all(np.isfinite(intersection)):
        raise ValueError('Non-finite tag coordinates')
    return np.array([intersection[0], intersection[1], plane_z])


def floor_tag_yaw_offset(corners, camera_matrix, distortion, local_quaternion,
                         rotation_body_camera, distortion_model='plumb_bob'):
    """Offset rotating a floor tag's printed top to map +Y, in radians.

    Corners are TL, TR, BR, BL; quaternion maps body to the IMU's local world.
    Unit camera height suffices because only edge direction matters.
    """
    pixels = np.asarray(corners, dtype=float)
    if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
        raise ValueError('Invalid tag corners')
    if np.min(np.linalg.norm(np.roll(pixels, -1, axis=0) - pixels, axis=1)) < 12:
        raise ValueError('Tag too small for yaw correction')
    points = np.array([project_floor_tag(
        pixel, camera_matrix, distortion, (0, 0, 1), local_quaternion,
        (0, 0, 0), rotation_body_camera, max_distance=10,
        distortion_model=distortion_model)[:2] for pixel in pixels])
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    # Require similar side lengths and clockwise, approximately right-angle corners.
    if np.min(lengths) < 1e-6 or np.max(lengths) > 1.5 * np.min(lengths):
        raise ValueError('Tag does not reconstruct as a floor square')
    directions = edges / lengths[:, None]
    following = np.roll(directions, -1, axis=0)
    turns = directions[:, 0] * following[:, 1] - directions[:, 1] * following[:, 0]
    cosines = np.sum(directions * following, axis=1)
    if not np.all(turns < 0) or np.any(np.abs(cosines) > .3):
        raise ValueError('Tag does not reconstruct as a floor square')
    top = (points[0] + points[1] - points[2] - points[3]) / 2.0
    offset = math.pi / 2 - math.atan2(top[1], top[0])
    return math.atan2(math.sin(offset), math.cos(offset))
