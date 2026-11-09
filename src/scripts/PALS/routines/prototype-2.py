#!/usr/bin/env python
"""Climb straight to 1 m, hold, descend to 0.25 m, move 0.3556 m forward (+Y), then land."""

import argparse
import math
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


CLIMB_HEIGHT_M = 1.0
DESCEND_HEIGHT_M = 0.25
FORWARD_M = 0.3556 # 14 inches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='hold duration at 1 m in seconds (default: 5)')
    args = parser.parse_args()
    if math.isnan(args.seconds) or math.isinf(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('prototype_2', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        # One takeoff ramp to the final height, without stopping at the default 0.25 m.
        drone.takeoff(hold_xy=True, height=CLIMB_HEIGHT_M)
        rospy.loginfo('Holding %.2f m for %.2f seconds', CLIMB_HEIGHT_M, args.seconds)
        drone.hover(args.seconds)
        rospy.loginfo('Descending to %.2f m', DESCEND_HEIGHT_M)
        drone.move_to_altitude(DESCEND_HEIGHT_M)
        drone.unlock_xy()
        rospy.loginfo('Moving forward %.4f m', FORWARD_M)
        drone.move_forward(FORWARD_M)
        drone.lock_xy()
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
