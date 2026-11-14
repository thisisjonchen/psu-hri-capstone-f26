"""Shared localization helpers; import onboard serial readers explicitly.

The laptop relay uses this package too, including on Windows without POSIX serial.
"""
import math

try:
    from time import monotonic
except ImportError:
    from monotonic import monotonic


def finite(value):
    return not (math.isnan(value) or math.isinf(value))
