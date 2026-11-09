#!/usr/bin/env python
"""Paired hover: both drones take off together, hold X/Y, and land together.

Select both drones and run this once; drone1 leads the plan handshake and
drone2 follows. Both must be connected to the dashboard (it relays the plans).
"""

import argparse
import math
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import actions  # registers the paired blocks
from coordination import LEADER, Follower, Leader, identity
from drone_controller import Drone

HOLD_PAD_S = 5.0  # slack for proposing the next plan before the hold expires


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--height', type=float, default=0.5,
                        help='hover height in meters, 0.25-1.0 (default: 0.5)')
    parser.add_argument('--seconds', type=float, default=10.0,
                        help='hover time before landing, 2-60 s (default: 10)')
    args = parser.parse_args()
    if math.isnan(args.height) or not 0.25 <= args.height <= 1.0:
        parser.error('--height must be 0.25-1.0 m')
    if math.isnan(args.seconds) or not 2 <= args.seconds <= 60:
        parser.error('--seconds must be 2-60')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('paired_hover', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        if identity() == LEADER:
            pair = Leader(drone)
            pair.takeoff(args.height, hold_s=args.seconds + HOLD_PAD_S)
            pair.land(lead_s=args.seconds, hold_s=HOLD_PAD_S)
            pair.finish()
        else:
            Follower(drone).run()
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
