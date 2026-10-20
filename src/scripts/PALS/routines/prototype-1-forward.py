#!/usr/bin/env python
"""Start together, climb for one second, then move forward and land independently."""

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
    parser.add_argument('--distance', type=float, default=0.25, help='forward distance in meters (default: 0.25)')
    args = parser.parse_args()
    if math.isnan(args.distance) or math.isinf(args.distance) or args.distance <= 0.06:
        parser.error('--distance must be finite and greater than the 0.06 m arrival tolerance')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('prototype_1_forward', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True, continue_after=1.0)
        drone.unlock_xy()
        rospy.loginfo('Moving forward %.2f m', args.distance)
        drone.move_forward(args.distance, speed_limit=0.5,
                           timeout=max(15.0, 20 * args.distance + 5.0))
        drone.lock_xy()
    finally:
        # On failure, use the existing landing recovery without waiting for a peer.
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
