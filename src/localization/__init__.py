"""Shared localization helpers; import onboard serial readers explicitly.

The laptop relay uses this package too, including on Windows without POSIX serial.
"""
import math

try:
    from time import monotonic
except ImportError:  # Python 2.7 on the DD24's ROS Kinetic image
    from monotonic import monotonic


def finite(value):
    return not (math.isnan(value) or math.isinf(value))
