#!/usr/bin/env python
"""Take off, climb to 0.5 m, hold the offset from the first floor AprilTag seen, then land."""

import argparse
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
    parser.add_argument('--seconds', type=float, default=10.0, help='tag hover duration')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('tag_hover_routine', disable_signals=True)
    drone = Drone()
    tag_hold = TagHold(drone)
    tag_hold.wait_ready()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        drone.move_to_altitude(HOVER_HEIGHT_M)
        tag_hold.acquire(ACQUIRE_SECONDS)
        rospy.loginfo('Holding tag %d offset for %.0f seconds', tag_hold.tag_id, args.seconds)
        drone.hover(args.seconds, tag_hold.command)
        tag_hold.finish()
    finally:
        if drone.mode == 'FLYING':
            try:
                drone.land(hold_xy=True)
            except Exception as error:
                rospy.logerr('Landing needs manual recovery: %s', error)
                # Keep the heartbeat active for manual landing.
                while drone.mode != 'DISARMED':
                    rospy.sleep(0.5)
        else:
            drone.disarm()


if __name__ == '__main__':
    main()
