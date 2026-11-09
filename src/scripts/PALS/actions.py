"""Paired action blocks for coordination.Leader. Compatible with Python 2.7.

Each block is a thin wrapper over Drone: it builds and checks the plan params,
gives the shared duration, and calls one Drone method. Flight behaviour and
its timing belong to Drone; pads and limits here are mission policy.
"""
import math

from coordination import Action, register


def finite(params):
    return all(not math.isnan(v) and not math.isinf(v) for v in params)


@register
class Takeoff(Action):
    """Plan params [height m, action seconds]; the routine passes only [height].

    Arm at the start, fly one second later, ramp to the height and be settled
    by the shared end. The leader adds a settle pad to its own climb time.
    """
    name = 'takeoff'
    MIN_HEIGHT, MAX_HEIGHT = 0.25, 1.0
    SETTLE_PAD_S = 3.0
    MAX_SECONDS = 60.0

    def prepare(self, drone, params):
        if len(params) != 1:
            raise ValueError('takeoff takes [height]')
        return [params[0], drone.takeoff_seconds(params[0]) + self.SETTLE_PAD_S]

    def validate(self, drone, params):
        if len(params) != 2 or not finite(params):
            raise ValueError('takeoff params are [height, seconds]')
        height, seconds = params
        if not self.MIN_HEIGHT <= height <= self.MAX_HEIGHT:
            raise ValueError('takeoff height must be {}-{} m'.format(self.MIN_HEIGHT, self.MAX_HEIGHT))
        if drone.mode != 'DISARMED':
            raise ValueError('takeoff starts disarmed on the ground')
        needed = drone.takeoff_seconds(height) + drone.ALTITUDE_SETTLE_SECONDS
        if not needed <= seconds <= self.MAX_SECONDS:
            raise ValueError('takeoff needs {:.1f}-{:.0f} s, plan gives {:.1f} s'.format(
                needed, self.MAX_SECONDS, seconds))
        drone._check_battery()
        if drone.voltage < drone.MIN_START_V:
            raise ValueError('battery {:.1f} V is too low to start'.format(drone.voltage))
        drone._hold_pose()            # EKF room estimate ready and fresh
        drone._hold_pose(local=True)  # EKF local estimate ready and fresh

    def duration(self, params):
        return params[1]

    def run(self, drone, start, end, params, check):
        drone.takeoff_at(params[0], start, end, check)


@register
class Land(Action):
    """Plan params [action seconds]; the routine passes nothing.

    Descend holding X/Y and disarm at touchdown by the shared end. The leader
    adds a pad to its own descent time; each drone checks its own descent fits.
    """
    name = 'land'
    PAD_S = 3.0
    MAX_SECONDS = 30.0

    def prepare(self, drone, params):
        if params:
            raise ValueError('land takes no params')
        return [drone.landing_seconds() + self.PAD_S]

    def validate(self, drone, params):
        if len(params) != 1 or not finite(params):
            raise ValueError('land params are [seconds]')
        if drone.mode != 'FLYING':
            raise ValueError('land starts in flight')
        needed = drone.landing_seconds()
        if not needed <= params[0] <= self.MAX_SECONDS:
            raise ValueError('land needs {:.1f}-{:.0f} s, plan gives {:.1f} s'.format(
                needed, self.MAX_SECONDS, params[0]))

    def duration(self, params):
        return params[0]

    def run(self, drone, start, end, params, check):
        drone.land(hold_xy=True, check=check, deadline=end)
