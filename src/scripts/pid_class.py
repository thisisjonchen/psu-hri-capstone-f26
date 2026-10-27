#!/usr/bin/env python



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
