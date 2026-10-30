"""Range and camera measurements over a flat floor.

Mounting is relative to the IMU in forward/left/up body axes.
The camera's optical axes are right/down/forward.
"""
from __future__ import division

import numpy as np

from localization.camera_flow import undistort
from localization.rotations import quaternion_matrix, skew
from localization.ekf import _array


def mounting_rotation(value):
    matrix = _array(value, (3, 3))
    if (not np.allclose(matrix.T.dot(matrix), np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix), 1., atol=1e-8, rtol=0)):
        raise ValueError('Mounting must be a proper rotation')
    return matrix


class FloorRange(object):
    def __init__(self, offset_body, beam_body, floor_z=0.):
        self.offset = _array(offset_body, (3,))
        self.beam = _array(beam_body, (3,))
        if not np.isclose(np.linalg.norm(self.beam), 1., atol=1e-8, rtol=0):
            raise ValueError('Range beam must be a unit vector')
        self.floor_z = float(floor_z)
        if not np.isfinite(self.floor_z):
            raise ValueError('Invalid floor height')

    def linearize(self, position, quaternion):
        rotation = quaternion_matrix(quaternion)
        center = position + rotation.dot(self.offset)
        direction = rotation.dot(self.beam)
        if center[2] <= self.floor_z or direction[2] >= -.1:
            raise ValueError('Range beam does not reach the floor reliably')
        distance = (self.floor_z-center[2])/direction[2]
        observation = np.zeros((1, 15))
        observation[0, 2] = -1/direction[2]
        observation[0, 6:9] = (rotation.dot(skew(self.offset))[2]
                                + distance*rotation.dot(skew(self.beam))[2])/direction[2]
        return distance, observation

    def correct(self, ekf, distance, variance, stamp, min_range, max_range):
        if ekf.stamp is None:
            return ekf._reject('not_initialized')
        if (not np.all(np.isfinite([distance, min_range, max_range]))
                or not 0 <= min_range < max_range or not min_range <= distance <= max_range):
            return ekf._reject('invalid_range')
        try:
            prediction, observation = self.linearize(ekf.position, ekf.quaternion)
        except ValueError:
            return ekf._reject('range_geometry')
        return ekf._correct([distance], [prediction], observation, variance, stamp, 'range', 6.63)


class FloorCamera(object):
    """Predict block motion between camera frames using their poses and the floor.

    The EKF retains the previous pose and its uncertainty. Pixel noise is
    provisional until measured from recordings.
    """
    def __init__(self, camera_matrix, distortion, model, image_size,
                 rotation_body_camera, offset_body, frame_interval=1/30.,
                 flip_vertically=True, floor_z=0., pixel_stddev=1.5,
                 min_height=.1, max_floor_distance=1.5):
        width, height = image_size
        if width <= 0 or height <= 0 or width % 16 or height % 16:
            raise ValueError('Camera dimensions must be positive multiples of 16')
        values = [frame_interval, pixel_stddev, min_height, max_floor_distance]
        if not np.all(np.isfinite(values + [floor_z])) or min(values) <= 0:
            raise ValueError('Invalid camera model settings')
        matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        distortion = np.asarray(distortion, dtype=float)
        if (not np.all(np.isfinite(matrix)) or min(matrix[0, 0], matrix[1, 1]) <= 0
                or not np.allclose(matrix[2], [0., 0., 1.])
                or not np.all(np.isfinite(distortion))
                or (model == 'equidistant' and distortion.shape != (4,))):
            raise ValueError('Invalid lens calibration')
        self.camera = (matrix, distortion, model)
        self.rotation = mounting_rotation(rotation_body_camera)
        self.offset = _array(offset_body, (3,))
        self.columns, self.rows = width//16, height//16
        self.interval, self.flip = frame_interval, flip_vertically
        self.floor_z, self.pixel_stddev = floor_z, pixel_stddev
        self.min_height, self.max_floor_distance = min_height, max_floor_distance
        u, v = np.meshgrid(np.arange(self.columns)*16+8., np.arange(self.rows)*16+8.)
        self.centers = np.column_stack((u.ravel(), v.ravel()))
        self.rays = undistort(self.centers, *self.camera)

    def _reference(self, ekf):
        ekf.clone_camera_pose()
        ekf.camera_clone['model_id'] = id(self)

    def linearize(self, position, quaternion, previous_position, previous_quaternion, ray):
        """Project a floor point into the previous frame and return its Jacobian.

        Columns cover the 15 current-state errors and six previous-pose errors.
        """
        rotation, previous = quaternion_matrix(quaternion), quaternion_matrix(previous_quaternion)
        center = position + rotation.dot(self.offset)
        body_ray = self.rotation.dot(ray)
        direction = rotation.dot(body_ray)
        if center[2]-self.floor_z < self.min_height or direction[2] >= -1e-3:
            raise ValueError('No nearby floor')
        length = (self.floor_z-center[2])/direction[2]
        if np.linalg.norm(length*direction[:2]) > self.max_floor_distance:
            raise ValueError('Distant floor')
        point = center + length*direction
        in_previous_body = previous.T.dot(point-previous_position)
        in_camera = self.rotation.T.dot(in_previous_body-self.offset)
        x, y, z = in_camera
        if z <= 1e-3:
            raise ValueError('Point behind previous camera')
        projection = np.array([[1/z, 0., -x/z**2], [0., 1/z, -y/z**2]])
        intersection = np.eye(3) - np.outer(direction, [0., 0., 1.])/direction[2]
        camera_from_world = self.rotation.T.dot(previous.T)
        observation = np.zeros((2, 21))
        observation[:, :3] = projection.dot(camera_from_world).dot(intersection)
        observation[:, 6:9] = -projection.dot(camera_from_world).dot(intersection).dot(rotation).dot(
            skew(self.offset) + length*skew(body_ray))
        observation[:, 15:18] = -projection.dot(camera_from_world)
        observation[:, 18:21] = projection.dot(self.rotation.T).dot(skew(in_previous_body))
        return in_camera[:2]/z, observation

    def correct(self, ekf, vectors_x, vectors_y, mbx, stamp):
        if ekf.stamp is None:
            return ekf._reject('not_initialized')
        if not np.isfinite(stamp) or abs(stamp-ekf.stamp) > 1e-6:
            return ekf._reject('measurement_not_at_state_time')
        clone = ekf.camera_clone
        if clone is not None and stamp <= clone['stamp']+1e-6:
            return ekf._reject('repeated_camera_frame')
        try:
            shape = ((self.columns+1)*self.rows,)
            x, y = _array(vectors_x, shape), _array(vectors_y, shape)
            if mbx != self.columns:
                raise ValueError('Invalid vector width')
        except (ValueError, TypeError):
            return ekf._reject('invalid_camera_vectors')
        x, y = x.reshape(self.rows, mbx+1)[:, :mbx], y.reshape(self.rows, mbx+1)[:, :mbx]
        if self.flip:
            x, y = x[::-1], -y[::-1]
        pixels = self.centers + np.column_stack((x.ravel(), y.ravel()))
        # Encoder vectors match adjacent frames, so restart after a gap.
        if (clone is None or clone.get('model_id') != id(self)
                or abs(stamp-clone['stamp']-self.interval) > .2*self.interval):
            self._reference(ekf)
            return ekf._reject('camera_reference_initialized')
        previous_rays = undistort(pixels, *self.camera)
        # Local lens Jacobians convert pixel-match noise to normalized-ray noise.
        pixel_dx = (undistort(pixels+[.01, 0], *self.camera)[:, :2]-previous_rays[:, :2])/.01
        pixel_dy = (undistort(pixels+[0, .01], *self.camera)[:, :2]-previous_rays[:, :2])/.01
        prior = ekf.joint_covariance()
        candidates = []
        for i, ray in enumerate(self.rays):
            # Zero encoder vectors are ambiguous (subpixel motion or no match).
            if not (x.ravel()[i] or y.ravel()[i]):
                continue
            if not (0 <= pixels[i, 0] < self.columns*16 and 0 <= pixels[i, 1] < self.rows*16):
                continue
            try:
                predicted, observation = self.linearize(ekf.position, ekf.quaternion,
                    clone['position'], clone['quaternion'], ray)
                jacobian = np.column_stack((pixel_dx[i], pixel_dy[i]))
                noise = self.pixel_stddev**2 * jacobian.dot(jacobian.T)
                residual = previous_rays[i, :2]-predicted
                total = observation.dot(prior).dot(observation.T)+noise
                nis = residual.dot(np.linalg.solve(total, residual))
                if np.isfinite(nis) and nis <= 9.21:
                    candidates.append((previous_rays[i, :2], predicted, observation, noise))
            except (ValueError, np.linalg.LinAlgError):
                continue
        if len(candidates) < 8:
            result = ekf._reject('too_few_camera_matches')
        else:
            selected = np.linspace(0, len(candidates)-1, min(12, len(candidates))).astype(int)
            chosen = [candidates[i] for i in selected]
            measured = np.concatenate([item[0] for item in chosen])
            predicted = np.concatenate([item[1] for item in chosen])
            observation = np.vstack([item[2] for item in chosen])
            noise = np.zeros((len(measured), len(measured)))
            for i, item in enumerate(chosen):
                noise[2*i:2*i+2, 2*i:2*i+2] = item[3]
            # Allow a mean squared normalized residual of nine after per-block gating.
            result = ekf._correct(measured, predicted, observation, noise, stamp,
                                  'camera', 9*len(measured))
        self._reference(ekf)
        return result
