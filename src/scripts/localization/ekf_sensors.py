"""Range and camera measurements over a flat floor.

Mounting is relative to the IMU in forward/left/up body axes.
The camera's optical axes are right/down/forward.
"""
from __future__ import division

import cv2
import numpy as np

from localization.camera_flow import undistort
from localization.rotations import quaternion_matrix, skew
from localization.ekf import _array
from localization import ekf_native


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
        # Range can correct a height prediction that has crossed below the floor.
        if direction[2] >= -.1:
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
    """Fuse image-motion rates using velocity, attitude and height above the floor.

    Pixels are unrectified. Raw encoder vectors remain available for comparison.
    Each pair measures average flow at its midpoint. Pixel noise is provisional
    until measured from recordings; no previous camera pose is an EKF state.
    """
    def __init__(self, camera_matrix, distortion, model, image_size,
                 rotation_body_camera, offset_body, frame_interval=1/30.,
                 flip_vertically=False, floor_z=0., pixel_stddev=1.5,
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
        self.previous_image = None
        self.image_stamp = None

    def image_pair(self, encoded, stamp):
        """Retain the actual images and timestamps used by each flow measurement."""
        if not np.isfinite(stamp) or (self.image_stamp is not None and stamp <= self.image_stamp):
            return None, 'repeated_camera_frame'
        try:
            image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        except (cv2.error, TypeError, ValueError):
            image = None
        previous, start = self.previous_image, self.image_stamp
        self.previous_image, self.image_stamp = image, stamp
        if image is None or image.shape != (self.rows*16, self.columns*16):
            self.previous_image = None
            return None, 'invalid_camera_image'
        if previous is None or stamp-start > .2:
            return None, 'camera_reference_initialized'
        return (start, stamp, previous, image), None

    def vector_pair(self, vectors_x, vectors_y, mbx, stamp):
        if not np.isfinite(stamp) or (self.image_stamp is not None and stamp <= self.image_stamp):
            return None, 'repeated_camera_frame'
        start, self.image_stamp = self.image_stamp, stamp
        try:
            shape = ((self.columns+1)*self.rows,)
            x, y = _array(vectors_x, shape), _array(vectors_y, shape)
            if mbx != self.columns:
                raise ValueError('Invalid vector width')
        except (ValueError, TypeError):
            return None, 'invalid_camera_vectors'
        if start is None or abs(stamp-start-self.interval) > .2*self.interval:
            return None, 'camera_reference_initialized'
        x, y = x.reshape(self.rows, mbx+1)[:, :mbx], y.reshape(self.rows, mbx+1)[:, :mbx]
        if self.flip:
            x, y = x[::-1], -y[::-1]
        flow = np.column_stack((x.ravel(), y.ravel()))
        valid = np.any(flow != 0, axis=1)
        return (start, stamp, self.centers[valid], (self.centers+flow)[valid]), None

    def _check_pair(self, ekf, sample, stamp, angular_rate):
        if ekf.stamp is None:
            return ekf._reject('not_initialized')
        start, end = sample[:2]
        if (not np.isfinite(stamp) or abs(stamp-ekf.stamp) > 1e-6
                or abs(stamp-(start+end)/2) > 1e-6 or not 0 < end-start <= .2):
            return ekf._reject('measurement_not_at_state_time')
        if angular_rate is None:
            return ekf._reject('missing_camera_imu')
        rotation = quaternion_matrix(ekf.quaternion)
        height = (ekf.position+rotation.dot(self.offset))[2]-self.floor_z
        velocity = ekf.velocity + rotation.dot(np.cross(angular_rate-ekf.gyro_bias, self.offset))
        # Both ends of the flow interval must clear the floor-height limit.
        if height-abs(velocity[2])*(end-start)/2 < self.min_height:
            return ekf._reject('camera_below_min_height')
        return None

    def linearize(self, position, velocity, quaternion, gyro_bias, angular_rate, rays):
        """Predict backward normalized-image rates and their 15-state Jacobians."""
        return ekf_native.camera_linearize(position, velocity, quaternion, gyro_bias,
            angular_rate, self.rotation, self.offset, rays, self.floor_z,
            self.min_height, self.max_floor_distance)

    def correct_image(self, ekf, sample, stamp, angular_rate):
        rejected = self._check_pair(ekf, sample, stamp, angular_rate)
        if rejected is not None:
            return rejected
        start, end, previous, image = sample
        indices = ekf_native.camera_indices(ekf.position, ekf.quaternion, self.rotation,
            self.offset, self.rays, self.floor_z, self.min_height, self.max_floor_distance)
        if len(indices) < 8:
            return ekf._reject('too_few_camera_matches')
        # Spread candidates across the floor, with room for tracking failures.
        if len(indices) > 48:
            indices = indices[np.linspace(0, len(indices)-1, 48).astype(int)]
        points = self.centers[indices].astype(np.float32).reshape(-1, 1, 2)
        settings = dict(winSize=(15, 15), maxLevel=2, minEigThreshold=1e-4,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        try:
            back, found_back, error_back = cv2.calcOpticalFlowPyrLK(image, previous, points, None, **settings)
            forward, found_forward, error_forward = cv2.calcOpticalFlowPyrLK(previous, image, back, None, **settings)
            valid = ((found_back.ravel() != 0) & (found_forward.ravel() != 0)
                     & (np.linalg.norm(forward[:, 0]-points[:, 0], axis=1) <= 1.)
                     & (error_back.ravel() <= 15.) & (error_forward.ravel() <= 15.))
            valid &= (np.all(np.isfinite(back[:, 0]), axis=1)
                      & (back[:, 0, 0] >= 0) & (back[:, 0, 0] < self.columns*16)
                      & (back[:, 0, 1] >= 0) & (back[:, 0, 1] < self.rows*16))
        except cv2.error:
            return ekf._reject('camera_tracking_failed')
        pixels, indices = back[valid, 0], indices[valid]
        if len(indices) > 24:
            selected = np.linspace(0, len(indices)-1, 24).astype(int)
            indices, pixels = indices[selected], pixels[selected]
        return self._correct_flow(ekf, self.centers[indices], pixels, end-start, stamp, angular_rate)

    def correct_vectors(self, ekf, sample, stamp, angular_rate):
        rejected = self._check_pair(ekf, sample, stamp, angular_rate)
        if rejected is not None:
            return rejected
        return self._correct_flow(ekf, sample[2], sample[3], sample[1]-sample[0], stamp, angular_rate)

    def _correct_flow(self, ekf, current_pixels, pixels, duration, stamp, angular_rate):
        if len(pixels) < 8:
            return ekf._reject('too_few_camera_matches')
        rays = undistort(current_pixels, *self.camera)
        previous_rays = undistort(pixels, *self.camera)
        indices, predicted, observation = self.linearize(ekf.position, ekf.velocity, ekf.quaternion,
            ekf.gyro_bias, angular_rate, (rays+previous_rays)/2)
        if len(indices) < 8:
            return ekf._reject('too_few_camera_matches')
        measured = (previous_rays[indices, :2]-rays[indices, :2])/duration
        pixels, previous_rays = pixels[indices], previous_rays[indices]
        jacobians = np.empty((len(indices), 2, 2))
        jacobians[:, :, 0] = (undistort(pixels+[.01, 0], *self.camera)[:, :2]-previous_rays[:, :2])/.01
        jacobians[:, :, 1] = (undistort(pixels+[0, .01], *self.camera)[:, :2]-previous_rays[:, :2])/.01
        noises = (self.pixel_stddev/duration)**2*np.einsum('nik,njk->nij', jacobians, jacobians)
        residuals = measured-predicted
        centered = residuals-np.median(residuals, axis=0)
        try:
            scores = np.einsum('ni,ni->n', centered, np.linalg.solve(noises, centered[:, :, None])[:, :, 0])
        except np.linalg.LinAlgError:
            return ekf._reject('camera_numerical_failure')
        inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < self.columns*16)
                  & (pixels[:, 1] >= 0) & (pixels[:, 1] < self.rows*16))
        keep = np.flatnonzero(inside & np.isfinite(scores) & (scores <= 9.21))
        if len(keep) < 8:
            return ekf._reject('too_few_camera_matches')
        # Fuse one flow vector; gyro uncertainty is shared by all image matches.
        information = np.linalg.inv(noises[keep])
        noise = np.linalg.inv(np.sum(information, axis=0))
        weights = np.einsum('ij,njk->nik', noise, information)
        observation = np.einsum('nij,njk->ik', weights, observation[keep])
        measured = np.einsum('nij,nj->i', weights, measured[keep])
        predicted = np.einsum('nij,nj->i', weights, predicted[keep])
        gyro_projection = observation[:, 12:15]
        noise += gyro_projection.dot(gyro_projection.T)*ekf.noise[6, 6]/duration
        return ekf._correct(measured, predicted,
                            observation, noise, stamp, 'camera', 9.21)
