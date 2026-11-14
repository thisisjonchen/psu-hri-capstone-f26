import math


class ThreeDimVec(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "x: %f, y: %f, z: %f" % (self.x, self.y, self.z)

    def planar_magnitude(self):
        return math.hypot(self.x, self.y)


class Position(ThreeDimVec):
    """Position components x, y, z."""


class Velocity(ThreeDimVec):
    """Velocity components x, y, z."""


class Error(ThreeDimVec):
    """Controller error components x, y, z."""


class RPY(object):
    """Roll, pitch, and yaw in radians."""
    def __init__(self, r=0.0, p=0.0, y=0.0):
        self.r = r
        self.p = p
        self.y = y
