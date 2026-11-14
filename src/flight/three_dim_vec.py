from __future__ import division

import math


class ThreeDimVec(object):
    # Basic Cartesian Vector
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "x: %f, y: %f, z: %f" % (self.x, self.y, self.z)

    def __mul__(self, other):
        return ThreeDimVec(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        return ThreeDimVec(self.x / other, self.y / other, self.z / other)

    __div__ = __truediv__

    def __add__(self, other):
        return ThreeDimVec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return ThreeDimVec(self.x - other.x, self.y - other.y, self.z - other.z)

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def planar_magnitude(self):
        return math.hypot(self.x, self.y)


class Position(ThreeDimVec):
    """Position components x, y, z."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Position, self).__init__(x, y, z)

class Velocity(ThreeDimVec):
    """Velocity components x, y, z."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Velocity, self).__init__(x, y, z)


class Error(ThreeDimVec):
    """Controller error components x, y, z."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Error, self).__init__(x, y, z)


class RPY(object):
    """Roll, pitch, and yaw in radians."""
    def __init__(self, r=0.0, p=0.0, y=0.0):
        self.r = r
        self.p = p
        self.y = y
