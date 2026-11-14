"""Horizontal target and frame math shared by flight control and routines."""
import math

import numpy as np

# Maximum age of the room pose used for room targets and coordination.
XY_PREDICTION_TIMEOUT = 1.5


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    x, y = vector
    return np.array([c * x - s * y, s * x + c * y])


def target_velocity(target, position, angle, gain, limit):
    velocity = rotate(np.asarray(target) - position, -angle) * gain
    speed = np.linalg.norm(velocity)
    return velocity * min(1.0, limit / speed) if speed else velocity
