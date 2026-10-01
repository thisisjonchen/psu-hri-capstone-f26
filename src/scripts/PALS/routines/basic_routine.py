#!/usr/bin/env python
"""Take off, move about 0.1 m forward, stop, then land on DD24."""

import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.scripts.PALS.drone_controller import Drone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('basic_routine', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff()
        drone.move_forward(0.1)
        drone.hover()
    finally:
        if drone.mode == 'FLYING':
            try:
                drone.land()
            except Exception as error:
                rospy.logerr('Landing needs manual recovery: %s', error)
                # Keep the heartbeat active for manual landing.
                while drone.mode != 'DISARMED':
                    rospy.sleep(0.5)
        else:
            drone.disarm()


if __name__ == '__main__':
    main()
