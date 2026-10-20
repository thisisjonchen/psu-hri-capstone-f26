#!/usr/bin/env python
"""Take off to 0.5 m, hold the initial offset from a floor AprilTag, then land."""

import argparse
import math
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone
from tag_hold import TagHold


ACQUIRE_SECONDS = 3.0
HOVER_HEIGHT_M = 0.5 # High enough for the camera to see a floor tag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--tag', type=int, help='tag ID; otherwise lock the first visible tag')
    parser.add_argument('--seconds', type=float, default=10.0, help='hold duration after acquiring the tag')
    args = parser.parse_args()
    if args.tag is not None and args.tag < 0:
        parser.error('--tag must be nonnegative')
    if math.isnan(args.seconds) or math.isinf(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('apriltag_lockon', disable_signals=True)
    drone = Drone()
    tag_hold = TagHold(drone)
    tag_hold.wait_ready()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        drone.move_to_altitude(HOVER_HEIGHT_M)
        tag_hold.acquire(ACQUIRE_SECONDS, tag_id=args.tag)
        rospy.loginfo('Holding tag %d offset for %.0f seconds', tag_hold.tag_id, args.seconds)
        drone.hover(args.seconds, tag_hold.command)
        tag_hold.finish()
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
