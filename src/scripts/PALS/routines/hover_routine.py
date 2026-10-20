#!/usr/bin/env python
"""Hover for 5 seconds at the configured takeoff altitude (default 0.25 m)."""

import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('hover_routine', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        rospy.loginfo('Holding altitude %.2f m for 5 seconds with fused X/Y hold',
                      drone.takeoff_height)
        drone.hover(5.0)
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
