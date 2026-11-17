"""Horizontal geometry shared by localization and flight control."""
import math

import numpy as np

ANCHOR_AREA_TOLERANCE = 1e-6


def anchor_outline(anchor_ids, positions):
    """Return the four corners in counterclockwise order, or None for an invalid layout."""
    if len(anchor_ids) != 4 or not all(anchor_ids) or len(set(anchor_ids)) != 4 or len(positions) != 4:
        return None
    xyz = np.asarray(positions)
    if not np.all(np.isfinite(xyz)):
        return None
    xy = xyz[:, :2]
    offsets = xy - xy.mean(axis=0)
    outline = xy[np.argsort(np.arctan2(offsets[:, 1], offsets[:, 0]))]
    edges = np.roll(outline, -1, axis=0) - outline
    next_edges = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * next_edges[:, 1] - edges[:, 1] * next_edges[:, 0]
    return outline if np.all(turns > ANCHOR_AREA_TOLERANCE) else None


def inside_outline(point, outline):
    edges = np.roll(outline, -1, axis=0) - outline
    offsets = point - outline
    return np.all(edges[:, 0] * offsets[:, 1] - edges[:, 1] * offsets[:, 0] >= -ANCHOR_AREA_TOLERANCE)


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    x, y = vector
    return np.array([c * x - s * y, s * x + c * y])


def target_velocity(target, position, angle, gain, limit):
    velocity = rotate(np.asarray(target) - position, -angle) * gain
    speed = np.linalg.norm(velocity)
    return velocity * min(1.0, limit / speed) if speed else velocity
