"""Map a floor AprilTag center into the UWB room from a calibrated camera ray.

Coordinate frames: map (+Z up); drone controller body (+X right, +Y forward,
+Z up); OpenCV optical camera (+X image right, +Y image down, +Z forward).
"""
from __future__ import division

import math

import cv2
import numpy as np

from localization.rotations import quaternion_matrix


def undistort_pixels(pixels, k, d, model):
    uv = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
    k = np.asarray(k, dtype=float).reshape(3, 3)
    d = np.asarray(d, dtype=float).reshape(-1)
    try:
        if model == 'equidistant':
            if d.size != 4 or not hasattr(getattr(cv2, 'fisheye', None), 'undistortPoints'):
                raise ValueError('Equidistant calibration requires four coefficients and OpenCV fisheye support')
            return cv2.fisheye.undistortPoints(uv, k, d)
        if model in ('plumb_bob', 'rational_polynomial'):
            return cv2.undistortPoints(uv, k, d)
        raise ValueError('Unsupported distortion model: {}'.format(model))
    except cv2.error as exc:
        raise ValueError('Cannot undistort tag: {}'.format(exc))


def estimate_tag_center(corners, size, camera_matrix, distortion, drone_position,
                        drone_quaternion, camera_offset_body, rotation_body_camera,
                        max_distance, distortion_model):
    """Solve the tag-to-camera pose and return its center in map meters."""
    half = size / 2.
    # Any cyclic start or winding describes the same square center.
    square = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=float)
    points = undistort_pixels(corners, camera_matrix, distortion, distortion_model)
    if not np.all(np.isfinite(points)):
        raise ValueError('Invalid undistorted tag corners')
    try:
        # Default iterative planar PnP is available on older OpenCV builds.
        ok, rvec, tvec = cv2.solvePnP(square, points, np.eye(3), None)
        if not ok or not np.all(np.isfinite(tvec)) or not np.all(np.isfinite(rvec)):
            raise ValueError('Tag pose solve failed')
        center = tvec.reshape(3)
        camera_corners = square.dot(cv2.Rodrigues(rvec)[0].T) + center
    except cv2.error as exc:
        raise ValueError('Cannot solve tag pose: {}'.format(exc))
    if np.any(camera_corners[:, 2] <= 0) or np.linalg.norm(center) > max_distance:
        raise ValueError('Tag pose is behind the camera or out of range')
    body_to_map = quaternion_matrix(drone_quaternion)
    return drone_position + body_to_map.dot(camera_offset_body + rotation_body_camera.dot(center))


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
    ray_xy = undistort_pixels(uv, k, d, distortion_model).reshape(2)
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
