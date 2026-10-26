#!/usr/bin/env python

import copy
import rospy


#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, i_range=None, control_range=(1000, 2000), midpoint=1500, smoothing=True):
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Config
        self.i_range = i_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing
        # Internal
        self.reset()

    def reset(self):
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0

    def step(self, err, time_elapsed):
        if self._old_err is None:
            # First time around prevent d term spike
            self._old_err = err

        # Find the p component
        self._p = err * self.kp

        time_elapsed = max(0.0, time_elapsed)
        # Find the i component
        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        if time_elapsed > 0:
            self._d = (err - self._old_err) * self.kd / time_elapsed
            self._old_err = err
            if self.smoothing:
                self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
                self._ddd = self._dd
                self._dd = self._d

        # Calculate control output
        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]), self.control_range[1])

        return output


class PID:

    def __init__(self,

                 # Use proportional XY feedback while diagnosing oscillation; trim is disabled.
                 roll=PIDaxis(0.50, 0.0, 0.0, control_range=(1470, 1530), midpoint=1500, i_range=(0, 0)),
                 roll_low=PIDaxis(0.0, 0.0, 0.0, control_range=(1470, 1530), midpoint=1500, i_range=(-5, 5)),

                 pitch=PIDaxis(0.75, 0.0, 0.0, control_range=(1470, 1530), midpoint=1500, i_range=(0, 0)),
                 pitch_low=PIDaxis(0.0, 0.0, 0.0, control_range=(1470, 1530), midpoint=1500, i_range=(-5, 5)),

                 yaw=PIDaxis(0.0, 0.0, 0.0),
                 ):

        # Retain the input cap for trim if re-enabled; its default Ki is zero.
        # PID errors arrive in cm/s: 5 cm/s is 0.05 m/s.
        self.trim_controller_cap_plane = 5.0

        self.roll = copy.deepcopy(roll)
        self.roll_low = copy.deepcopy(roll_low)

        self.pitch = copy.deepcopy(pitch)
        self.pitch_low = copy.deepcopy(pitch_low)

        self.yaw = copy.deepcopy(yaw)

        self._t = None

        # Tuning values specific to each drone
        self.roll_low.init_i = 0.0
        self.pitch_low.init_i = 0.0
        self.reset()

    def reset(self):
        """ Reset each pid and restore the initial i terms """
        # reset time variable
        self._t = None

        # reset individual PIDs
        self.roll.reset()
        self.roll_low.reset()
        self.pitch.reset()
        self.pitch_low.reset()

        # restore tuning values
        self.roll_low._i = self.roll_low.init_i
        self.pitch_low._i = self.pitch_low.init_i

    def _planar_step(self, error, axis, trim, dt, integrate=True):
        cap = self.trim_controller_cap_plane
        previous_trim = trim._i
        if integrate:
            trim.step(max(-cap, min(cap, error)), dt)
        axis.step(error, dt)
        output = axis.midpoint + axis._p + axis._i + axis._d + trim._i
        lower, upper = axis.control_range
        # Limit the combined command, and allow trim to unwind at saturation.
        if (output > upper and error > 0) or (output < lower and error < 0):
            trim._i = previous_trim
            output = axis.midpoint + axis._p + axis._i + axis._d + trim._i
        return max(lower, min(upper, output))

    def step(self, error, cmd_yaw_velocity=0, integrate_xy=True):
        """ Compute roll, pitch and yaw commands; altitude_controller.py owns throttle.
        """
        now = rospy.get_time()
        time_elapsed = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now

        cmd_r = self._planar_step(error.x, self.roll, self.roll_low, time_elapsed, integrate_xy)
        cmd_p = self._planar_step(error.y, self.pitch, self.pitch_low, time_elapsed, integrate_xy)

        # Compute yaw command
        cmd_y = 1500 + cmd_yaw_velocity
        return [cmd_r, cmd_p, cmd_y]
