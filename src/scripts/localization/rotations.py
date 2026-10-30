"""Rotation helpers shared by geometry and inertial estimation.

Hamilton quaternions are stored (x, y, z, w). Multiplication composes rotations
in matrix order: R(multiply(a, b)) = R(a) R(b).
"""
from __future__ import division

import math
import numpy as np


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_matrix(quaternion):
    """Return a 3x3 body-to-map rotation matrix for quaternion (x,y,z,w)."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Invalid quaternion')
    norm = np.dot(q, q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Zero or unbounded quaternion')
    x, y, z, w = q / math.sqrt(norm)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def quaternion_multiply(a, b):
    av, bv = np.asarray(a[:3]), np.asarray(b[:3])
    return np.r_[a[3]*bv + b[3]*av + np.cross(av, bv), a[3]*b[3] - av.dot(bv)]


def quaternion_exp(rotation_vector):
    """Convert a rotation vector in radians to a unit quaternion."""
    vector = np.asarray(rotation_vector, dtype=float)
    angle = np.linalg.norm(vector)
    scale = .5 - angle*angle/48 if angle < 1e-6 else math.sin(angle/2)/angle
    return np.r_[scale*vector, math.cos(angle/2)]


def right_jacobian(rotation_vector):
    """Derivative of Log(Exp(-v) Exp(v + delta)) at delta=0."""
    angle = np.linalg.norm(rotation_vector)
    matrix = skew(rotation_vector)
    if angle < 1e-4:
        a, b = .5 - angle*angle/24, 1/6.0 - angle*angle/120
    else:
        a = (1 - math.cos(angle)) / angle**2
        b = (angle - math.sin(angle)) / angle**3
    return np.eye(3) - a*matrix + b*matrix.dot(matrix)
