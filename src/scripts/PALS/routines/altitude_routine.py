#!/usr/bin/env python
"""Take off, hover at 0.10, 0.30, 0.45, and 0.20 m, with relative X/Y hold, then land on DD24."""

import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


ALTITUDES = (0.10, 0.30, 0.45, 0.20)  # Downward range readings in meters.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('altitude_routine', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        for altitude in ALTITUDES:
            rospy.loginfo('Moving to altitude %.2f m', altitude)
            # move() takes a displacement from the current measured height.
            drone.move(z=altitude - drone.height)
            rospy.loginfo('Holding altitude %.2f m for 2 seconds', altitude)
            drone.hover(2.0)
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
