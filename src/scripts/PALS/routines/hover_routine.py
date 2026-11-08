#!/usr/bin/env python
"""Hover at the takeoff altitude (default 0.25 m), or at --height after takeoff."""

import argparse
import math
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='hover duration after takeoff in seconds (default: 5)')
    parser.add_argument('--height', type=float,
                        help='hover height in meters, reached after takeoff (default: takeoff height)')
    args = parser.parse_args()
    if math.isnan(args.seconds) or math.isinf(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if args.height is not None and (math.isnan(args.height) or math.isinf(args.height) or args.height <= 0):
        parser.error('--height must be finite and positive')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('hover_routine', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        height = drone.takeoff_height if args.height is None else args.height
        if height != drone.takeoff_height:
            drone.move_to_altitude(height)
        rospy.loginfo('Holding altitude %.2f m for %.2f seconds with fused X/Y hold',
                      height, args.seconds)
        drone.hover(args.seconds)
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
